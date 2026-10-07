/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Host test for the Ground Station <-> Vega remote configuration.
///
/// Runs the real Vega server (flight_computer/src/comm/remote_config.cpp with the real
/// settings table) against the real Ground Station client and profile model over a
/// simulated radio link that reproduces the receiver behaviour: the Ground Station radio
/// answers every received Vega packet with the last payload written to it, before the
/// ESP32 has processed that packet.

#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "comm/remote_config.hpp"
#include "remote/vega_config_client.hpp"
#include "remote/vega_profile.hpp"
#include "telemetry/crc.hpp"

// The Vega fills this map at runtime from the control frequency.
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static char kSpeedNames[NUM_REC_SPEEDS][14] = {"100Hz",   "50Hz",    "33.33Hz", "25Hz",    "20Hz",
                                               "16.67Hz", "14.29Hz", "12.5Hz",  "11.11Hz", "10Hz"};
std::array<char*, NUM_REC_SPEEDS> recorder_speed_map = {kSpeedNames[0], kSpeedNames[1], kSpeedNames[2], kSpeedNames[3],
                                                        kSpeedNames[4], kSpeedNames[5], kSpeedNames[6], kSpeedNames[7],
                                                        kSpeedNames[8], kSpeedNames[9]};
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

namespace {

int failures = 0;

void check(bool condition, const char* what) {
  if (!condition) {
    std::cerr << "FAILED: " << what << '\n';
    ++failures;
  }
}

uint32_t phraseCrc(const char* phrase) {
  // Same CRC as the Vega and the Ground Station use for the testing phrase.
  return crc32(reinterpret_cast<const uint8_t*>(phrase), strlen(phrase));
}

class FakeVega final : public remote_config::ServerEnvironment {
 public:
  FakeVega() {
    const char* lines[] = {
        "main_altitude=200",       "acc_threshold=35",       "servo1_init_pos=0",       "servo2_init_pos=1000",
        "tele_link_phrase=LINK1",  "tele_test_phrase=TEST1", "tele_power_level=24",     "tele_enable=ON",
        "tele_adaptive_power=OFF", "test_mode=OFF",          "ev_liftoff=1,500,7,2",    "ev_apogee=2,1",
        "ev_main_deployment=3,1",  "timer1_start=LIFTOFF",   "timer1_trigger=CUSTOM_1", "timer1_duration=4000",
        "buzzer_volume=50",        "battery_type=LI-ION",    "rec_speed=100Hz",         "rec_elements=4294967295"};
    for (const char* line : lines) {
      const auto result = remote_config::ApplyLine(&stored, line, strlen(line));
      if (result != remote_config::Result::kOk) {
        std::cerr << "fixture line rejected: " << line << '\n';
        ++failures;
      }
    }
    stored.config_version = 212;
  }

  uint32_t NowMs() override { return now; }
  uint8_t FlightState() override { return state; }
  bool TestingMode() override { return testing; }
  bool RecorderWriting() override { return recorderWriting; }
  void CopyStoredConfig(cats_config_t* out) override { *out = stored; }
  bool SaveConfig(const cats_config_t& config) override {
    ++saves;
    if (failSave) {
      return false;
    }
    stored = config;
    return true;
  }
  void Reboot(remote_config::RebootMode mode) override {
    ++reboots;
    lastRebootMode = mode;
    rebootPending = true;
  }
  const char* BoardName() override { return "CATS Vega"; }
  const char* FirmwareVersion() override { return "3.0.2+gscfg.1"; }
  const char* TelemetryVersion() override { return "1.1.3"; }

  uint32_t now{1000};
  uint8_t state{2};  // READY
  bool testing{false};
  bool recorderWriting{false};
  bool failSave{false};
  cats_config_t stored{};
  int saves{0};
  int reboots{0};
  bool rebootPending{false};
  remote_config::RebootMode lastRebootMode{remote_config::RebootMode::kNormal};
};

/// Vega + radio + Ground Station client.
class Link {
 public:
  explicit Link(const char* vegaPhrase, const char* gsPhrase, double loss = 0.0, uint32_t seed = 1)
      : rng(seed), lossRate(loss), gsPasscode(phraseCrc(gsPhrase)) {
    server = std::make_unique<remote_config::Server>(vega);
    server->Configure(phraseCrc(vegaPhrase), token);
    client.begin(gsPasscode, 7);
  }

  /// Simulates a Vega reboot: new server instance and token; the GS radio keeps its payload.
  void rebootVega() {
    ++token;
    server = std::make_unique<remote_config::Server>(vega);
    server->Configure(phraseCrc("TEST1"), token);
    vega.rebootPending = false;
  }

  bool lost() { return lossRate > 0.0 && std::uniform_real_distribution<double>(0.0, 1.0)(rng) < lossRate; }

  /// One Vega telemetry period (100 ms).
  void step() {
    vega.now += 100;
    std::array<uint8_t, remote_config::kFrameSize> downlink{};
    const bool config = server->NextDownlink(downlink.data());
    if (!config) {
      downlink.fill(0);
      downlink[0] = vega.state;  // ordinary telemetry
      ++telemetryPackets;
    } else {
      ++configPackets;
    }
    if (vega.rebootPending) {
      rebootVega();
      ++vegaReboots;
      return;
    }
    if (lost()) {
      return;
    }
    // The receiver answers immediately with its last payload.
    if (gsRadioEnabled && radioPayloadValid && !lost()) {
      // The Vega radio forwards 16 bytes; the 16th is radio padding.
      uint8_t forwarded[16]{};
      memcpy(forwarded, radioPayload.data(), remote_config::kFrameSize);
      server->OnUplink(forwarded, sizeof(forwarded));
    }
    // Then the ESP32 processes the packet.
    if (remote_config::DownlinkTypeOf(downlink.data(), downlink.size()) != remote_config::DownlinkType::kNone) {
      client.onDownlink(downlink.data(), vega.now);
    }
    if (client.onPacket(radioPayload.data(), vega.now)) {
      radioPayloadValid = true;
    }
    client.tick(vega.now);
  }

  bool run(uint32_t maxSteps) {
    for (uint32_t i = 0; i < maxSteps; ++i) {
      step();
      if (client.state() != VegaConfig::Client::State::kBusy) {
        return client.state() == VegaConfig::Client::State::kDone;
      }
    }
    return false;
  }

  FakeVega vega;
  std::unique_ptr<remote_config::Server> server;
  VegaConfig::Client client;
  std::mt19937 rng;
  double lossRate;
  uint32_t gsPasscode;
  uint8_t token{5};
  std::array<uint8_t, remote_config::kFrameSize> radioPayload{};
  bool radioPayloadValid{false};
  bool gsRadioEnabled{true};
  int telemetryPackets{0};
  int configPackets{0};
  int vegaReboots{0};
};

bool readModel(Link& link, VegaConfig::Model* model) {
  link.client.startRead(link.vega.now);
  if (!link.run(1500)) {
    std::cerr << "read failed: " << link.client.error() << '\n';
    return false;
  }
  return model->loadText(link.client.text(), link.client.textLength());
}

void testProtocolHeadersIdentical(const char* vegaHeader, const char* gsHeader) {
  std::ifstream first(vegaHeader);
  std::ifstream second(gsHeader);
  const std::string a((std::istreambuf_iterator<char>(first)), std::istreambuf_iterator<char>());
  const std::string b((std::istreambuf_iterator<char>(second)), std::istreambuf_iterator<char>());
  check(!a.empty() && a == b, "remote_config_protocol.hpp copies must be identical");
}

void testApplyLine() {
  cats_config_t config{};
  using remote_config::ApplyLine;
  using remote_config::Result;
  auto apply = [&config](const char* line) { return ApplyLine(&config, line, strlen(line)); };
  check(apply("main_altitude=150") == Result::kOk && config.control_settings.main_altitude == 150, "number");
  check(apply("main_altitude = 9") == Result::kBadValue, "number below range");
  check(apply("main_altitude=65536") == Result::kBadValue, "number above range");
  check(apply("main_altitude=12a") == Result::kBadValue, "number with garbage");
  check(apply("acc_threshold=80") == Result::kOk, "upper limit");
  check(apply("unknown_key=1") == Result::kBadKey, "unknown key");
  check(apply("no equals sign") == Result::kBadArgs, "missing equals");
  check(apply("tele_adaptive_power=on") == Result::kOk && config.telemetry_settings.adaptive_power == ON,
        "lookup is case insensitive");
  check(apply("tele_adaptive_power=MAYBE") == Result::kBadValue, "invalid lookup");
  check(apply("timer2_start=APOGEE") == Result::kOk && config.timers[1].start_event == EV_APOGEE, "event lookup");
  check(apply("timer2_duration=1200000") == Result::kOk && config.timers[1].duration == 1200000, "u32 max");
  check(apply("timer2_duration=1200001") == Result::kBadValue, "u32 above max");
  check(apply("ev_apogee=2,1,7,0") == Result::kOk && config.action_array[EV_APOGEE][2] == 7 &&
            config.action_array[EV_APOGEE][4] == 0,
        "short array is zero padded");
  check(apply("ev_apogee=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17") == Result::kBadValue, "array too long");
  check(apply("ev_apogee=1,,2") == Result::kBadValue, "empty array item");
  check(apply("tele_link_phrase=abc") == Result::kBadValue, "phrase too short");
  check(apply("tele_link_phrase=abcd efg") == Result::kBadValue, "phrase with space");
  check(apply("tele_link_phrase=ABCDEFGHIJKLMNOP") == Result::kOk &&
            strcmp(config.telemetry_settings.link_phrase, "ABCDEFGHIJKLMNOP") == 0,
        "phrase max length");
  check(apply("tele_link_phrase=-") == Result::kOk && config.telemetry_settings.link_phrase[0] == '\0', "clear phrase");

  char value[remote_config::kMaxLineLength]{};
  for (uint16_t i = 0; i < value_table_entry_count; ++i) {
    if (strcmp(value_table[i].name, "ev_apogee") == 0) {
      remote_config::FormatValue(config, value_table[i], value, sizeof(value));
    }
  }
  check(strcmp(value, "2,1,7,0,0,0,0,0,0,0,0,0,0,0,0,0") == 0, "array formatting");
}

void testConfigTextFits() {
  // Worst case: every action slot holds the most negative value.
  cats_config_t config{};
  for (auto& event : config.action_array) {
    for (auto& action : event) {
      action = INT16_MIN;
    }
  }
  config.rec_mask = UINT32_MAX;
  for (auto& timer : config.timers) {
    timer.duration = 1200000;
  }
  strcpy(config.telemetry_settings.link_phrase, "ABCDEFGHIJKLMNOP");
  strcpy(config.telemetry_settings.test_phrase, "ABCDEFGHIJKLMNOP");
  char text[remote_config::kMaxTextLength]{};
  const size_t length = remote_config::BuildConfigText(config, "CATS Vega", "3.0.2+gscfg.1-dbg-dev", "1.2.0-plus-extra",
                                                       text, sizeof(text));
  check(length > 0, "worst-case configuration text must fit into kMaxTextLength");
  std::cout << "  worst-case configuration text: " << length << " bytes\n";
}

void testReadWriteCommit() {
  Link link("TEST1", "TEST1");
  link.client.startConnect(link.vega.now);
  check(link.run(200), "connect");
  check(link.client.status().token == 5, "token reported");
  check((link.client.status().flags & remote_config::kFlagWritable) != 0, "writable when READY");

  VegaConfig::Model model;
  check(readModel(link, &model), "read and parse");
  std::cout << "  read finished after " << link.configPackets + link.telemetryPackets << " packets ("
            << link.client.textLength() << " bytes)\n";
  check(strcmp(model.identity().board, "CATS Vega") == 0, "identity board");
  check(strcmp(model.identity().firmware, "3.0.2+gscfg.1") == 0, "identity firmware");
  const int altitude = VegaConfig::findKey("main_altitude", strlen("main_altitude"));
  const int timerStart = VegaConfig::findKey("timer1_start", strlen("timer1_start"));
  const int liftoff = VegaConfig::findKey("ev_liftoff", strlen("ev_liftoff"));
  const int phrase = VegaConfig::findKey("tele_link_phrase", strlen("tele_link_phrase"));
  check(strcmp(model.current(altitude), "200") == 0, "read main_altitude");
  check(strcmp(model.current(timerStart), "LIFTOFF") == 0, "read timer1_start");
  check(strcmp(model.current(liftoff), "1,500,7,2,0,0,0,0,0,0,0,0,0,0,0,0") == 0, "read ev_liftoff");
  check(strcmp(model.current(phrase), "LINK1") == 0, "read link phrase");
  check(link.telemetryPackets > 0, "telemetry keeps flowing during a session");

  check(model.adjust(altitude, +1, 5), "edit main_altitude");
  check(strcmp(model.pending(altitude), "250") == 0, "edited value");
  check(model.adjust(timerStart, +1), "edit timer1_start");
  check(strcmp(model.pending(timerStart), "MAX_V") == 0, "event cycles");
  check(!model.adjust(phrase, +1), "link phrase is not editable");
  check(model.setPending(liftoff, "1,250"), "set actions");
  check(model.pendingCount() == 3, "three pending changes");

  const auto lines = model.writeLines();
  link.client.startWrite(lines, link.vega.now);
  check(link.run(600), "write and commit");
  check(link.vega.saves == 1, "saved once");
  check(link.vega.stored.control_settings.main_altitude == 250, "stored altitude");
  check(link.vega.stored.timers[0].start_event == EV_MAX_V, "stored timer start");
  check(link.vega.stored.action_array[EV_LIFTOFF][1] == 250 && link.vega.stored.action_array[EV_LIFTOFF][2] == 0,
        "stored actions");
  check(link.vega.stored.is_set_by_user, "set_by_user");

  // The Vega reboots 1.5 s after the commit. The GS radio may still hold the commit frame.
  for (int i = 0; i < 40 && link.vegaReboots == 0; ++i) {
    link.step();
  }
  check(link.vegaReboots == 1, "Vega rebooted after commit");
  check(link.vega.lastRebootMode == remote_config::RebootMode::kNormal, "normal reboot after commit");
  for (int i = 0; i < 100; ++i) {
    link.step();
  }
  check(link.vega.saves == 1 && link.vegaReboots == 1, "stale frames cause no second save or reboot");

  // Verify after reboot.
  link.client.startConnect(link.vega.now);
  check(link.run(200), "reconnect after reboot");
  check(link.client.status().token == 6, "new token after reboot");
  VegaConfig::Model readBack;
  check(readModel(link, &readBack), "read back");
  char failed[32]{};
  check(model.verify(readBack, failed, sizeof(failed)), "verify written values");
}

void testRebootModes() {
  Link link("TEST1", "TEST1");
  link.client.startConnect(link.vega.now);
  check(link.run(200), "connect for reboot");
  link.client.startReboot(remote_config::RebootMode::kTesting, link.vega.now);
  check(link.run(200), "reboot request acknowledged");
  for (int i = 0; i < 40 && link.vegaReboots == 0; ++i) {
    link.step();
  }
  check(link.vegaReboots == 1 && link.vega.lastRebootMode == remote_config::RebootMode::kTesting,
        "reboot into testing");
  // The stale reboot frame carries the old token: it must not reboot the Vega again.
  for (int i = 0; i < 100; ++i) {
    link.step();
  }
  check(link.vegaReboots == 1, "no reboot loop from stale frame");

  // In testing mode the Vega accepts a reboot back to flight mode.
  link.vega.testing = true;
  link.vega.state = 0;
  link.client.startConnect(link.vega.now);
  check(link.run(200), "connect in testing mode");
  check((link.client.status().flags & remote_config::kFlagTestingMode) != 0, "testing flag");
  link.client.startReboot(remote_config::RebootMode::kNormal, link.vega.now);
  check(link.run(200), "reboot from testing acknowledged");
  for (int i = 0; i < 40 && link.vegaReboots == 1; ++i) {
    link.step();
  }
  check(link.vegaReboots == 2 && link.vega.lastRebootMode == remote_config::RebootMode::kNormal,
        "reboot back to flight mode");
}

void testSafetyGates() {
  {
    Link link("TEST1", "WRONG");
    link.client.startConnect(link.vega.now);
    check(!link.run(200), "wrong phrase does not connect");
    check(strcmp(link.client.error(), "No answer from Vega") == 0, "wrong phrase error");
    check(link.configPackets == 0, "no configuration frames for a wrong phrase");
  }
  {
    Link link("TEST1", "TEST1");
    link.vega.state = 3;  // THRUSTING
    link.client.startConnect(link.vega.now);
    check(!link.run(200), "no session in flight");
    check(link.configPackets == 0, "telemetry untouched in flight");
  }
  {
    Link link("TEST1", "TEST1");
    link.client.startConnect(link.vega.now);
    check(link.run(200), "connect before liftoff");
    VegaConfig::Model model;
    check(readModel(link, &model), "read before liftoff");
    link.vega.state = 3;  // liftoff during the session
    for (int i = 0; i < 5; ++i) {
      link.step();
    }
    const int before = link.configPackets;
    for (int i = 0; i < 50; ++i) {
      link.step();
    }
    check(link.configPackets == before, "session ends immediately at liftoff");
  }
  {
    Link link("TEST1", "TEST1");
    link.vega.state = 7;  // TOUCHDOWN: sessions only exist before liftoff
    link.client.startConnect(link.vega.now);
    check(!link.run(200), "no session after touchdown");
    check(link.configPackets == 0, "telemetry untouched after touchdown");
  }
  {
    // A restart accepted on the pad must not reset the Vega if liftoff happens before it fires.
    Link link("TEST1", "TEST1");
    link.client.startConnect(link.vega.now);
    check(link.run(200), "connect before restart race");
    link.client.startReboot(remote_config::RebootMode::kNormal, link.vega.now);
    check(link.run(200), "restart acknowledged on the pad");
    link.vega.state = 3;  // liftoff inside the restart delay
    for (int i = 0; i < 40; ++i) {
      link.step();
    }
    check(link.vegaReboots == 0 && link.vega.reboots == 0, "no restart after liftoff");
  }
  {
    Link link("TEST1", "TEST1");
    link.vega.recorderWriting = true;
    link.client.startConnect(link.vega.now);
    check(link.run(200), "connect while logging");
    link.client.startReboot(remote_config::RebootMode::kNormal, link.vega.now);
    check(!link.run(200), "no reboot while the recorder writes");
  }
  {
    Link link("TEST1", "TEST1");
    link.client.startConnect(link.vega.now);
    check(link.run(200), "connect for invalid value");
    link.client.startWrite({"main_altitude=5"}, link.vega.now);
    check(!link.run(300), "invalid value rejected");
    check(strstr(link.client.error(), "main_altitude") != nullptr, "error names the setting");
    check(link.client.failedLine() == 0, "failed line index");
    check(link.vega.saves == 0, "nothing saved after a rejected value");
  }
  {
    Link link("TEST1", "TEST1");
    link.vega.failSave = true;
    link.client.startConnect(link.vega.now);
    check(link.run(200), "connect for save failure");
    link.client.startWrite({"main_altitude=300"}, link.vega.now);
    check(!link.run(300), "save failure reported");
    for (int i = 0; i < 40; ++i) {
      link.step();
    }
    check(link.vegaReboots == 0, "no reboot after a failed save");
  }
}

void testLossyLink() {
  for (uint32_t seed = 1; seed <= 5; ++seed) {
    Link link("TEST1", "TEST1", 0.2, seed);
    link.client.startConnect(link.vega.now);
    check(link.run(400), "lossy connect");
    VegaConfig::Model model;
    check(readModel(link, &model), "lossy read");
    model.setPending(VegaConfig::findKey("ev_apogee", 9), "2,1,5,500,6,1000");
    model.setPending(VegaConfig::findKey("main_altitude", 13), "123");
    link.client.startWrite(model.writeLines(), link.vega.now);
    check(link.run(2000), "lossy write");
    check(link.vega.stored.control_settings.main_altitude == 123, "lossy stored value");
    check(link.vega.stored.action_array[EV_APOGEE][3] == 500, "lossy stored actions");
  }
}

void testProfiles() {
  Link link("TEST1", "TEST1");
  link.client.startConnect(link.vega.now);
  check(link.run(200), "connect for profiles");
  VegaConfig::Model model;
  check(readModel(link, &model), "read for profiles");

  std::vector<char> json(8192);
  const size_t size = model.exportProfile(json.data(), json.size(), "2026-10-07T12:00:00.000Z", "CATS Ground Station");
  check(size > 0, "export profile");
  const std::string exported(json.data(), size);
  check(exported.find("\"format\": \"cats-configurator-profile\"") != std::string::npos, "profile format");
  check(exported.find("\"main_altitude\": 200") != std::string::npos, "numbers exported as numbers");
  check(exported.find("\"start\": \"LIFTOFF\"") != std::string::npos, "timers exported as objects");
  check(exported.find("\"logging\": {}") != std::string::npos, "empty logging section");
  check(exported.find("buzzer_volume") == std::string::npos, "only Configurator profile keys");

  // Round trip without changes.
  char error[64]{};
  check(model.importProfile(json.data(), size, error, sizeof(error)), "import own profile");
  check(model.pendingCount() == 0 && model.diffRows().empty(), "own profile has no changes");

  // A profile as written by the Configurator (floats, mixed case, skipped and unknown fields).
  const char* configurator = R"({
    "format": "cats-configurator-profile", "schemaVersion": 1, "createdAt": "2026-09-01T10:00:00.000Z",
    "source": {"boardModel": "CATS Vega", "firmwareVersion": "3.0.2", "telemetryFirmwareVersion": null,
               "configuratorVersion": "2.0.0"},
    "configuration": {"main_altitude": 300.0, "acc_threshold": 40, "tele_link_phrase": "OTHER",
                      "tele_adaptive_power": "on", "test_mode": "ON", "future_key": 5,
                      "tele_power_level": 99},
    "events": {"ev_apogee": "2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0"},
    "timers": {"timer1": {"start": "apogee", "duration": 1500.0, "trigger": "CUSTOM_1"}},
    "logging": {}
  })";
  check(model.importProfile(configurator, strlen(configurator), error, sizeof(error)), "import Configurator profile");
  const auto rows = model.diffRows();
  int changes = 0;
  int skipped = 0;
  int invalid = 0;
  int unsupported = 0;
  for (const auto& row : rows) {
    changes += row.status == VegaConfig::RowStatus::kChange ? 1 : 0;
    skipped += row.status == VegaConfig::RowStatus::kSkipped ? 1 : 0;
    invalid += row.status == VegaConfig::RowStatus::kInvalid ? 1 : 0;
    unsupported += row.status == VegaConfig::RowStatus::kUnsupported ? 1 : 0;
  }
  // main_altitude, acc_threshold, tele_adaptive_power, timer1_start, timer1_duration (ev_apogee unchanged).
  check(changes == 5, "Configurator profile changes");
  check(skipped == 2, "link phrase and permanent testing mode skipped");
  check(invalid == 1, "out-of-range power rejected");
  check(unsupported == 1, "unknown key reported");

  link.client.startWrite(model.writeLines(), link.vega.now);
  check(link.run(800), "write profile");
  check(link.vega.stored.control_settings.main_altitude == 300, "profile altitude stored");
  check(link.vega.stored.timers[0].duration == 1500, "profile timer stored");
  check(strcmp(link.vega.stored.telemetry_settings.link_phrase, "LINK1") == 0, "link phrase untouched");
  check(!link.vega.stored.enable_testing_mode, "permanent testing mode never written by radio");

  // Rejected profiles leave the model unchanged.
  VegaConfig::Model untouched = model;
  const char* wrongBoard = R"({"format": "cats-configurator-profile", "schemaVersion": 1, "createdAt": "x",
    "source": {"boardModel": "Other Board"}, "configuration": {}, "events": {}, "timers": {}, "logging": {}})";
  check(!model.importProfile(wrongBoard, strlen(wrongBoard), error, sizeof(error)), "wrong board rejected");
  const char* duplicate = R"({"format": "cats-configurator-profile", "schemaVersion": 1, "createdAt": "x",
    "source": {}, "configuration": {"main_altitude": 20}, "events": {"main_altitude": 30}, "timers": {},
    "logging": {}})";
  check(!model.importProfile(duplicate, strlen(duplicate), error, sizeof(error)), "duplicate rejected");
  check(!model.importProfile("{not json", 9, error, sizeof(error)), "invalid JSON rejected");
  const char* schema2 = R"({"format": "cats-configurator-profile", "schemaVersion": 2})";
  check(!model.importProfile(schema2, strlen(schema2), error, sizeof(error)), "future schema rejected");
  check(model.pendingCount() == untouched.pendingCount(), "rejected imports keep pending changes");

  char display[32]{};
  VegaConfig::displayValue(VegaConfig::keyTable()[VegaConfig::findKey("ev_liftoff", 10)],
                           "1,500,7,2,0,0,0,0,0,0,0,0,0,0,0,0", display, sizeof(display));
  check(strcmp(display, "2 actions") == 0, "action summary");
  VegaConfig::displayValue(VegaConfig::keyTable()[VegaConfig::findKey("ev_liftoff", 10)],
                           "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", display, sizeof(display));
  check(strcmp(display, "none") == 0, "empty action summary");
}

void testNumberFormatting() {
  // Values from a real Vega profile (CATS Configurator 2.0.3). The ESP32 links newlib-nano printf,
  // so these must not depend on 64-bit printf conversions.
  const auto& keys = VegaConfig::keyTable();
  char out[VegaConfig::kValueSize]{};
  check(VegaConfig::canonicalize(keys[VegaConfig::findKey("main_altitude", 13)], "450", out, sizeof(out)) &&
            strcmp(out, "450") == 0,
        "number canonical form");
  check(VegaConfig::canonicalize(keys[VegaConfig::findKey("timer1_duration", 15)], "1200000", out, sizeof(out)) &&
            strcmp(out, "1200000") == 0,
        "large number canonical form");
  check(VegaConfig::canonicalize(keys[VegaConfig::findKey("ev_main_deployment", 18)],
                                 "3,1,2,1,1,1000,2,0,3,0,0,0,0,0,0,0", out, sizeof(out)) &&
            strcmp(out, "3,1,2,1,1,1000,2,0,3,0,0,0,0,0,0,0") == 0,
        "actions canonical form");
  check(VegaConfig::canonicalize(keys[VegaConfig::findKey("ev_liftoff", 10)], "7,2", out, sizeof(out)) &&
            strcmp(out, "7,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0") == 0,
        "short action list padded");

  VegaConfig::Model model;
  const char* text =
      "#board=CATS Vega\n#fw=3.0.2+gscfg.1\n#tele=1.2.0\nmain_altitude=450\nacc_threshold=35\nservo1_init_pos=500\n"
      "ev_apogee=5,160,6,160,0,0,0,0,0,0,0,0,0,0,0,0\nev_main_deployment=3,1,2,1,1,1000,2,0,3,0,0,0,0,0,0,0\n";
  check(model.loadText(text, strlen(text)), "load real configuration text");
  char display[40]{};
  const int apogee = VegaConfig::findKey("ev_apogee", 9);
  VegaConfig::displayValue(keys[apogee], model.current(static_cast<size_t>(apogee)), display, sizeof(display));
  check(strcmp(display, "2 actions") == 0, "apogee actions shown");
  const int main = VegaConfig::findKey("ev_main_deployment", 18);
  VegaConfig::displayValue(keys[main], model.current(static_cast<size_t>(main)), display, sizeof(display));
  check(strcmp(display, "5 actions") == 0, "main actions shown");

  const char* profile = R"({"format": "cats-configurator-profile", "schemaVersion": 1,
    "source": {"boardModel": "CATS Vega"},
    "configuration": {"main_altitude": 450.0, "acc_threshold": 35.0, "servo1_init_pos": 750.0},
    "events": {"ev_apogee": "5,160,6,160,0,0,0,0,0,0,0,0,0,0,0,0"}})";
  char error[64]{};
  check(model.importProfile(profile, strlen(profile), error, sizeof(error)), "import Configurator floats");
  check(model.pendingCount() == 1, "only servo1_init_pos differs");
  const int servo = VegaConfig::findKey("servo1_init_pos", 15);
  check(strcmp(model.pending(static_cast<size_t>(servo)), "750") == 0, "float imported as integer");
}

void testActionView() {
  const auto& keys = VegaConfig::keyTable();
  VegaConfig::Model model;
  const char* text =
      "#board=CATS Vega\n#fw=3.0.2+gscfg.1\n#tele=1.2.0\n"
      "ev_liftoff=7,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n"
      "ev_apogee=5,160,6,160,0,0,0,0,0,0,0,0,0,0,0,0\n"
      "ev_main_deployment=3,1,2,1,1,1000,2,0,3,0,0,0,0,0,0,0\n"
      "ev_touchdown=0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n";
  check(model.loadText(text, strlen(text)), "load actions");
  const auto liftoff = static_cast<size_t>(VegaConfig::findKey("ev_liftoff", 10));
  const auto apogee = static_cast<size_t>(VegaConfig::findKey("ev_apogee", 9));
  const auto main = static_cast<size_t>(VegaConfig::findKey("ev_main_deployment", 18));
  const auto touchdown = static_cast<size_t>(VegaConfig::findKey("ev_touchdown", 12));
  check(model.actionCount(liftoff) == 1 && model.actionCount(apogee) == 2 && model.actionCount(main) == 5 &&
            model.actionCount(touchdown) == 0,
        "action counts");

  const char* expected[][2] = {{"Pyro 2", "ON"}, {"Pyro 1", "ON"}, {"Delay", "1000 ms"}, {"Pyro 1", "OFF"},
                               {"Pyro 2", "OFF"}};
  for (size_t slot = 0; slot < 5; ++slot) {
    VegaConfig::ActionItem now{};
    VegaConfig::ActionItem before{};
    check(model.action(main, slot, now, before), "decode main action");
    char value[24]{};
    VegaConfig::actionArgument(now, value, sizeof(value));
    check(strcmp(VegaConfig::actionName(now.action), expected[slot][0]) == 0 && strcmp(value, expected[slot][1]) == 0,
          "main action text");
  }
  VegaConfig::ActionItem now{};
  VegaConfig::ActionItem before{};
  check(model.action(liftoff, 0, now, before), "decode liftoff");
  char value[24]{};
  VegaConfig::actionArgument(now, value, sizeof(value));
  check(strcmp(VegaConfig::actionName(now.action), "Recorder") == 0 && strcmp(value, "ON") == 0, "recorder text");

  // Only servo positions are editable, within 0..1000.
  check(!model.adjustAction(main, 0, +1), "pyro action not editable");
  check(!model.adjustAction(main, 2, +1), "delay not editable");
  check(!model.adjustAction(liftoff, 0, +1), "recorder not editable");
  check(!model.adjustAction(apogee, 2, +1), "slot past the end not editable");
  check(model.adjustAction(apogee, 1, +1, 10), "servo 2 +10");
  check(model.adjustAction(apogee, 1, -1), "servo 2 -1");
  check(strcmp(model.pending(apogee), "5,160,6,169,0,0,0,0,0,0,0,0,0,0,0,0") == 0, "servo edit keeps the list");
  check(model.pendingCount() == 1, "one pending change");
  check(model.action(apogee, 1, now, before) && now.argument == 169 && before.argument == 160, "changed slot");
  check(model.adjustAction(apogee, 0, -1, 1000), "servo clamps at 0");
  check(!model.adjustAction(apogee, 0, -1), "servo stays at 0");
  check(model.adjustAction(apogee, 0, +1, 5000) && model.action(apogee, 0, now, before) && now.argument == 1000,
        "servo clamps at 1000");
  check(!model.adjustAction(apogee, 0, +1), "servo stays at 1000");

  char from[40]{};
  char to[40]{};
  VegaConfig::describeActionChange("5,160,6,160", "5,160,6,300", from, sizeof(from), to, sizeof(to));
  check(strcmp(from, "Servo 2 160") == 0 && strcmp(to, "300") == 0, "review text for one servo");
  VegaConfig::describeActionChange("5,160,6,160", "5,170,6,300", from, sizeof(from), to, sizeof(to));
  check(strcmp(from, "2 actions") == 0 && strcmp(to, "2 actions") == 0, "review text for several changes");
  (void)keys;

  // A servo edit is written over the radio like any other change.
  Link link("TEST1", "TEST1");
  link.client.startConnect(link.vega.now);
  check(link.run(200), "connect for action edit");
  VegaConfig::Model remote;
  check(readModel(link, &remote), "read for action edit");
  check(remote.setPending(apogee, "5,160,6,160"), "servo actions pending");
  remote.acceptPending();
  check(remote.adjustAction(apogee, 1, +1, 25), "edit servo 2");
  link.client.startWrite(remote.writeLines(), link.vega.now);
  check(link.run(600), "write servo edit");
  check(link.vega.stored.action_array[EV_APOGEE][0] == 5 && link.vega.stored.action_array[EV_APOGEE][1] == 160 &&
            link.vega.stored.action_array[EV_APOGEE][2] == 6 && link.vega.stored.action_array[EV_APOGEE][3] == 185,
        "stored servo edit");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 3) {
    testProtocolHeadersIdentical(argv[1], argv[2]);
  }
  testApplyLine();
  testConfigTextFits();
  testReadWriteCommit();
  testRebootModes();
  testSafetyGates();
  testLossyLink();
  testProfiles();
  testNumberFormatting();
  testActionView();
  if (failures != 0) {
    std::cerr << failures << " remote configuration check(s) failed\n";
    return 1;
  }
  std::cout << "Remote configuration tests passed\n";
  return 0;
}
