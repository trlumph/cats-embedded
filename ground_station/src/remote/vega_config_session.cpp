/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "remote/vega_config_session.hpp"

#include <esp_random.h>

#include <algorithm>
#include <cstring>

#include "config.hpp"
#include "console.hpp"
#include "telemetry/crc.hpp"
#include "utils.hpp"

namespace VegaConfig {

namespace {

constexpr const char* kProfileDirectory = "/vega_profiles";
constexpr size_t kMaxProfiles = 64;
constexpr size_t kMaxProfileSize = 16384;
constexpr size_t kProfileBufferSize = 8192;
constexpr uint32_t kConnectWindowMs = 8000;
constexpr uint32_t kReadWindowMs = 70000;
constexpr uint32_t kRebootWindowMs = 10000;
constexpr uint32_t kReturnWindowMs = 45000;
/// The session is considered lost when no status frame arrives for this long.
constexpr uint32_t kLinkLostMs = 12000;

constexpr uint8_t kReadyState = 2;

void copyText(char* destination, size_t capacity, const char* source) {
  if (capacity == 0) {
    return;
  }
  strncpy(destination, source, capacity - 1);
  destination[capacity - 1] = '\0';
}

void idleFrame(uint32_t passcode, uint8_t out[remote_config::kFrameSize]) {
  remote_config::UplinkFrame frame{};
  frame.passcode = passcode;
  frame.op = remote_config::Op::kNop;
  remote_config::EncodeUplink(frame, out);
}

bool isJsonName(const char* name, size_t length) {
  if (length < 6) {
    return false;
  }
  const char* suffix = name + length - 5;
  return strcasecmp(suffix, ".json") == 0 && name[0] != '.';
}

uint32_t testingPasscode() {
  const char* phrase = systemConfig.config.testingPhrase;
  return crc32(reinterpret_cast<const uint8_t*>(phrase), strnlen(phrase, kMaxPhraseLen));
}

}  // namespace

bool Session::begin() {
  requests = xQueueCreate(1, sizeof(Request));
  packets = xQueueCreate(8, sizeof(Packet));
  modelMutex = xSemaphoreCreateMutex();
  if (requests == nullptr || packets == nullptr || modelMutex == nullptr) {
    return false;
  }
  return xTaskCreate(worker, "vega_config", 8192, this, 1, nullptr) == pdPASS;
}

Snapshot Session::snapshot() const {
  portENTER_CRITICAL(&stateMux);
  const Snapshot copy = state;
  portEXIT_CRITICAL(&stateMux);
  return copy;
}

bool Session::idle() const {
  portENTER_CRITICAL(&stateMux);
  const bool result = !busy && state.phase == Phase::kIdle;
  portEXIT_CRITICAL(&stateMux);
  return result;
}

bool Session::submit(const Request& request, const char* activity) {
  portENTER_CRITICAL(&stateMux);
  bool allowed = requests != nullptr && !busy;
  if (allowed) {
    const bool open = state.sessionOpen;
    switch (request.action) {
      case Action::kOpen:
      case Action::kQuickReboot:
        allowed = !open;
        break;
      case Action::kClose:
        allowed = true;
        break;
      default:
        allowed =
            open && (state.phase == Phase::kReady || state.phase == Phase::kBrowsing || state.phase == Phase::kReview);
        break;
    }
  }
  if (allowed) {
    cancelRequested = false;
    busy = true;
    state.phase = Phase::kBusy;
    state.percent = 0;
    copyText(state.activity, sizeof(state.activity), activity);
    ++state.revision;
  }
  portEXIT_CRITICAL(&stateMux);
  if (!allowed) {
    return false;
  }
  if (xQueueSend(requests, &request, 0) != pdPASS) {
    result(false, "Vega Configuration", "Request queue unavailable");
    return false;
  }
  return true;
}

bool Session::open(uint8_t link) { return submit(Request{Action::kOpen, link}, "Connecting to Vega"); }
bool Session::read() { return submit(Request{Action::kRead}, "Reading configuration"); }
bool Session::write() { return submit(Request{Action::kWrite}, "Writing settings"); }
bool Session::reboot(remote_config::RebootMode mode) {
  return submit(Request{Action::kReboot, 0, mode}, "Sending restart");
}
bool Session::quickReboot(uint8_t link, remote_config::RebootMode mode) {
  return submit(Request{Action::kQuickReboot, link, mode}, "Connecting to Vega");
}
bool Session::saveProfile() { return submit(Request{Action::kSave}, "Saving profile"); }
bool Session::browseProfiles() { return submit(Request{Action::kBrowse}, "Reading USB drive"); }
bool Session::loadProfile(size_t index) {
  Request request{Action::kLoad};
  request.index = index;
  return submit(request, "Loading profile");
}
bool Session::close() { return submit(Request{Action::kClose}, "Closing session"); }

void Session::cancel() { cancelRequested = true; }

bool Session::acknowledge() {
  portENTER_CRITICAL(&stateMux);
  const bool allowed =
      !busy && (state.phase == Phase::kResult || state.phase == Phase::kReview || state.phase == Phase::kBrowsing);
  if (allowed) {
    state.phase = state.sessionOpen ? Phase::kReady : Phase::kIdle;
    state.storageUsed = false;
    ++state.revision;
  }
  portEXIT_CRITICAL(&stateMux);
  return allowed;
}

void Session::setBusy(const char* activity, uint8_t percent) {
  portENTER_CRITICAL(&stateMux);
  if (strcmp(state.activity, activity) != 0 || state.percent != percent) {
    copyText(state.activity, sizeof(state.activity), activity);
    state.percent = percent;
    ++state.revision;
  }
  portEXIT_CRITICAL(&stateMux);
}

void Session::setPhase(Phase phase) {
  size_t pending = 0;
  if (xSemaphoreTake(modelMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    pending = model.pendingCount();
    xSemaphoreGive(modelMutex);
  }
  portENTER_CRITICAL(&stateMux);
  state.phase = phase;
  state.pendingCount = pending;
  busy = false;
  ++state.revision;
  portEXIT_CRITICAL(&stateMux);
}

void Session::result(bool success, const char* title, const char* message) {
  portENTER_CRITICAL(&stateMux);
  state.success = success;
  copyText(state.title, sizeof(state.title), title);
  copyText(state.message, sizeof(state.message), message);
  portEXIT_CRITICAL(&stateMux);
  setPhase(Phase::kResult);
  console.log.printf("[VEGA CONFIG] %s: %s\n", title, message);
}

void Session::publishStatus() {
  const auto& status = client.status();
  portENTER_CRITICAL(&stateMux);
  if (state.fsmState != status.fsmState || state.flags != status.flags) {
    state.fsmState = status.fsmState;
    state.flags = status.flags;
    ++state.revision;
  }
  portEXIT_CRITICAL(&stateMux);
}

void Session::onRadioPacket(uint8_t source, const uint8_t* payload, bool configFrame) {
  if (source != sinkSource.load()) {
    return;
  }
  Packet packet{};
  memcpy(packet.payload, payload, remote_config::kFrameSize);
  packet.config = configFrame;
  // Never block the telemetry task; a full queue only delays the next request.
  (void)xQueueSend(packets, &packet, 0);
}

void Session::pump() {
  Packet packet{};
  while (xQueueReceive(packets, &packet, 0) == pdPASS) {
    const uint32_t now = millis();
    if (packet.config) {
      client.onDownlink(packet.payload, now);
    }
    uint8_t frame[remote_config::kFrameSize]{};
    if (active != nullptr && client.onPacket(frame, now)) {
      (void)active->sendRemoteFrame(frame);
    }
  }
  client.tick(millis());
  if (client.haveStatus()) {
    lastStatusSeen = client.lastStatusMs();
    publishStatus();
  }
}

bool Session::runClient(uint32_t timeoutMs, const char* activity) {
  const uint32_t started = millis();
  while (client.state() == Client::State::kBusy) {
    pump();
    setBusy(activity, client.percent());
    if (millis() - started > timeoutMs || cancelRequested.load()) {
      client.stop();
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return client.state() == Client::State::kDone;
}

bool Session::connect(uint8_t link, bool readConfig) {
  if (link < 1 || link > 2) {
    result(false, "Vega Configuration", "Invalid link");
    return false;
  }
  if (systemConfig.config.testingPhrase[0] == '\0') {
    result(false, "Vega Configuration", "Set the Test Phrase in Settings > Telemetry");
    return false;
  }
  Telemetry* telemetry = links[link - 1];
  const auto packet = telemetry->data.snapshot();
  const bool recent = (xTaskGetTickCount() - telemetry->data.getLastUpdateTime()) <= pdMS_TO_TICKS(3000);
  if (!recent) {
    result(false, "Vega Configuration", "No telemetry from the Vega on this link");
    return false;
  }
  if (!packet.testing_mode && packet.state > kReadyState) {
    result(false, "Vega Configuration", "Only available before liftoff (or in testing mode)");
    return false;
  }

  const uint32_t passcode = testingPasscode();
  uint8_t idle[remote_config::kFrameSize]{};
  idleFrame(passcode, idle);
  client.begin(passcode, static_cast<uint8_t>(esp_random()));
  xQueueReset(packets);
  sinkSource = link;
  telemetry->setRemoteSink(this);
  if (!telemetry->beginRemoteSession(idle)) {
    telemetry->setRemoteSink(nullptr);
    sinkSource = 0;
    result(false, "Vega Configuration", "Radio link busy; try again");
    return false;
  }
  active = telemetry;
  activeSource = link;

  client.startConnect(millis());
  if (!runClient(kConnectWindowMs, "Connecting to Vega")) {
    disconnect();
    if (cancelRequested.load()) {
      result(false, "Cancelled", "The connection was cancelled");
      return false;
    }
    result(false, "No answer from Vega",
           "Check that both Test Phrases match and the Vega runs firmware with remote configuration");
    return false;
  }
  lastStatusSeen = millis();
  portENTER_CRITICAL(&stateMux);
  state.sessionOpen = true;
  state.link = link;
  state.configLoaded = false;
  ++state.revision;
  portEXIT_CRITICAL(&stateMux);

  if (!readConfig) {
    return true;
  }
  Model fresh{};
  if (!readInto(fresh)) {
    return false;
  }
  if (xSemaphoreTake(modelMutex, portMAX_DELAY) == pdTRUE) {
    model = fresh;
    xSemaphoreGive(modelMutex);
  }
  portENTER_CRITICAL(&stateMux);
  state.configLoaded = true;
  state.identity = fresh.identity();
  portEXIT_CRITICAL(&stateMux);
  return true;
}

bool Session::readInto(Model& target) {
  client.startRead(millis());
  if (!runClient(kReadWindowMs, "Reading configuration")) {
    if (cancelRequested.load()) {
      result(false, "Cancelled", "Reading was cancelled");
      return false;
    }
    result(false, "Reading failed", client.error()[0] != '\0' ? client.error() : "Reading timed out");
    return false;
  }
  if (!target.loadText(client.text(), client.textLength())) {
    result(false, "Reading failed", "The Vega sent an unexpected configuration");
    return false;
  }
  return true;
}

void Session::doWrite() {
  std::vector<std::string> lines;
  if (xSemaphoreTake(modelMutex, portMAX_DELAY) == pdTRUE) {
    lines = model.writeLines();
    xSemaphoreGive(modelMutex);
  }
  if (lines.empty()) {
    result(true, "Nothing to write", "There are no pending changes");
    return;
  }
  const uint8_t oldToken = client.status().token;
  client.startWrite(lines, millis());
  if (!runClient(static_cast<uint32_t>(lines.size()) * 20000U + 10000U, "Writing settings")) {
    if (cancelRequested.load()) {
      result(false, "Cancelled", "Nothing was saved on the Vega");
      return;
    }
    result(false, "Write failed", client.error()[0] != '\0' ? client.error() : "Vega stopped answering");
    return;
  }

  // The Vega saved the settings and restarts. Wait until it answers with a new token.
  const uint32_t started = millis();
  bool back = false;
  while (millis() - started < kReturnWindowMs && !cancelRequested.load()) {
    client.startConnect(millis());
    if (runClient(kConnectWindowMs, "Vega is restarting") && client.status().token != oldToken &&
        (client.status().flags & remote_config::kFlagRebootPending) == 0) {
      back = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  if (!back) {
    result(false, "Settings saved", "Not verified: the Vega did not answer after the restart. Read it again.");
    return;
  }

  setBusy("Verifying", 0);
  Model readBack{};
  if (!readInto(readBack)) {
    return;
  }
  char failedKey[32]{};
  bool verified = false;
  if (xSemaphoreTake(modelMutex, portMAX_DELAY) == pdTRUE) {
    verified = model.verify(readBack, failedKey, sizeof(failedKey));
    if (verified) {
      model = readBack;
    }
    xSemaphoreGive(modelMutex);
  }
  if (!verified) {
    char message[96]{};
    snprintf(message, sizeof(message), "%s differs after the restart", failedKey);
    result(false, "Verification failed", message);
    return;
  }
  char message[96]{};
  snprintf(message, sizeof(message), "%u setting%s saved, Vega restarted and verified",
           static_cast<unsigned>(lines.size()), lines.size() == 1 ? "" : "s");
  result(true, "Settings written", message);
}

void Session::doReboot(remote_config::RebootMode mode, bool quick) {
  client.startReboot(mode, millis());
  const bool ok = runClient(kRebootWindowMs, "Sending restart");
  const char* error = client.error();
  // Replace the request in the receiver before the Vega restarts.
  disconnect();
  if (!ok) {
    result(false, cancelRequested.load() ? "Cancelled" : "Restart failed",
           error[0] != '\0' ? error : (cancelRequested.load() ? "No restart was sent" : "Vega stopped answering"));
    return;
  }
  (void)quick;
  result(true, mode == remote_config::RebootMode::kTesting ? "Testing mode" : "Vega restarting",
         mode == remote_config::RebootMode::kTesting ? "The Vega restarts in testing mode for one boot"
                                                     : "The Vega restarts in flight mode");
}

bool Session::claimStorage(const char** error) {
  if (recorder.getStatus().state != RecorderState::Idle) {
    *error = "Finalize the active flight log first";
    return false;
  }
  // Takes the drive back from the PC if it is shared; the HMI shares it again afterwards.
  if (!Utils::isFilesystemAvailable() && !Utils::claimFirmwareStorage()) {
    *error = "USB drive unavailable; eject it on the PC";
    return false;
  }
  portENTER_CRITICAL(&stateMux);
  state.storageUsed = true;
  portEXIT_CRITICAL(&stateMux);
  // Like the radio updater: the recorder must not claim the volume while profiles are written.
  // A flight starting meanwhile loses at most the samples of this short file operation.
  recorderPaused = recorder.pauseForRadioUpdate();
  if (!recorderPaused) {
    *error = "Finalize the active flight log first";
    return false;
  }
  // Exclusive access while files are written (shared with the radio updater).
  storageOwned = Utils::beginRadioUpdateStorage();
  if (!storageOwned) {
    *error = "USB drive busy; try again";
    return false;
  }
  return true;
}

void Session::releaseStorage() {
  if (storageOwned) {
    Utils::endRadioUpdateStorage();
    storageOwned = false;
  }
  if (recorderPaused) {
    recorder.resumeAfterRadioUpdate();
    recorderPaused = false;
  }
}

void Session::doSave() {
  std::vector<char> json(kProfileBufferSize);
  size_t size = 0;
  if (xSemaphoreTake(modelMutex, portMAX_DELAY) == pdTRUE) {
    char creator[48]{};
    snprintf(creator, sizeof(creator), "CATS Ground Station %s", FIRMWARE_VERSION);
    // The Ground Station only knows the time of day from GNSS, not the date.
    size = model.exportProfile(json.data(), json.size(), "1970-01-01T00:00:00.000Z", creator);
    xSemaphoreGive(modelMutex);
  }
  if (size == 0) {
    result(false, "Save failed", "Could not create the profile");
    return;
  }
  const char* error = "";
  if (!claimStorage(&error)) {
    releaseStorage();
    result(false, "Save failed", error);
    return;
  }
  if (!fatfs.exists(kProfileDirectory) && !fatfs.mkdir(kProfileDirectory)) {
    releaseStorage();
    result(false, "Save failed", "Could not create /vega_profiles");
    return;
  }
  // Next free vega_NNN.json.
  unsigned highest = 0;
  auto directory = fatfs.open(kProfileDirectory);
  if (directory && directory.isDirectory()) {
    auto entry = directory.openNextFile();
    while (entry) {
      char name[64]{};
      entry.getName(name, sizeof(name));
      unsigned number = 0;
      if (sscanf(name, "vega_%3u.json", &number) == 1 && number > highest) {
        highest = number;
      }
      entry.close();
      entry = directory.openNextFile();
    }
  }
  directory.close();
  char path[64]{};
  snprintf(path, sizeof(path), "%s/vega_%03u.json", kProfileDirectory, highest + 1U);
  // NOLINTNEXTLINE(hicpp-signed-bitwise) FILE_WRITE is an SdFat macro
  auto file = fatfs.open(path, FILE_WRITE);
  const bool written = file && file.write(reinterpret_cast<const uint8_t*>(json.data()), size) == size;
  if (file) {
    file.close();
  }
  releaseStorage();
  if (!written) {
    result(false, "Save failed", "Could not write the profile file");
    return;
  }
  result(true, "Profile saved", path);
}

void Session::doBrowse() {
  const char* error = "";
  std::vector<std::string> names;
  if (!claimStorage(&error)) {
    releaseStorage();
    result(false, "Load profile", error);
    return;
  }
  auto directory = fatfs.open(kProfileDirectory);
  if (directory && directory.isDirectory()) {
    auto entry = directory.openNextFile();
    while (entry && names.size() < kMaxProfiles) {
      char name[256]{};
      const size_t length = entry.getName(name, sizeof(name));
      if (!entry.isDirectory() && length < 64 && isJsonName(name, length)) {
        names.emplace_back(name);
      }
      entry.close();
      entry = directory.openNextFile();
    }
  }
  directory.close();
  releaseStorage();
  if (names.empty()) {
    result(false, "Load profile", "Copy profiles (.json) into /vega_profiles");
    return;
  }
  std::sort(names.begin(), names.end());
  if (xSemaphoreTake(modelMutex, portMAX_DELAY) == pdTRUE) {
    files = names;
    xSemaphoreGive(modelMutex);
  }
  portENTER_CRITICAL(&stateMux);
  state.fileCount = names.size();
  portEXIT_CRITICAL(&stateMux);
  setPhase(Phase::kBrowsing);
}

void Session::doLoad(size_t index) {
  std::string name;
  if (xSemaphoreTake(modelMutex, portMAX_DELAY) == pdTRUE) {
    if (index < files.size()) {
      name = files[index];
    }
    xSemaphoreGive(modelMutex);
  }
  if (name.empty()) {
    result(false, "Load profile", "No profile selected");
    return;
  }
  const char* error = "";
  if (!claimStorage(&error)) {
    releaseStorage();
    result(false, "Load profile", error);
    return;
  }
  char path[96]{};
  snprintf(path, sizeof(path), "%s/%s", kProfileDirectory, name.c_str());
  auto file = fatfs.open(path);
  std::vector<char> json;
  bool ok = false;
  if (file && !file.isDirectory() && file.size() > 0 && file.size() <= kMaxProfileSize) {
    json.resize(file.size());
    ok = file.read(reinterpret_cast<uint8_t*>(json.data()), json.size()) == static_cast<int>(json.size());
  }
  if (file) {
    file.close();
  }
  releaseStorage();
  if (!ok) {
    result(false, "Load profile", "Could not read the profile file");
    return;
  }
  char importError[64]{};
  bool imported = false;
  if (xSemaphoreTake(modelMutex, portMAX_DELAY) == pdTRUE) {
    imported = model.importProfile(json.data(), json.size(), importError, sizeof(importError));
    xSemaphoreGive(modelMutex);
  }
  if (!imported) {
    result(false, "Load profile", importError);
    return;
  }
  setPhase(Phase::kReview);
}

void Session::disconnect() {
  if (active != nullptr) {
    uint8_t idle[remote_config::kFrameSize]{};
    idleFrame(testingPasscode(), idle);
    // The receiver must not keep repeating a request; retry until the idle frame is written.
    for (int attempt = 0; attempt < 5 && !active->endRemoteSession(idle); ++attempt) {
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    active->setRemoteSink(nullptr);
  }
  active = nullptr;
  activeSource = 0;
  sinkSource = 0;
  client.stop();
  xQueueReset(packets);
  portENTER_CRITICAL(&stateMux);
  state.sessionOpen = false;
  state.link = 0;
  ++state.revision;
  portEXIT_CRITICAL(&stateMux);
}

size_t Session::entryCount() const { return keyCount(); }

bool Session::entry(size_t index, EntryView& view) const {
  if (index >= keyCount() || xSemaphoreTake(modelMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }
  const KeyInfo& info = keyTable()[index];
  copyText(view.label, sizeof(view.label), info.label);
  view.known = model.known(index);
  view.changed = model.changed(index);
  view.editable = model.editable(index);
  view.actions = info.kind == Kind::kActions && view.known;
  displayValue(info, view.known ? model.pending(index) : "", view.value, sizeof(view.value));
  xSemaphoreGive(modelMutex);
  return true;
}

bool Session::adjust(size_t index, int direction, int32_t multiplier) {
  if (xSemaphoreTake(modelMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }
  const bool changed = model.adjust(index, direction, multiplier);
  const size_t pending = model.pendingCount();
  xSemaphoreGive(modelMutex);
  portENTER_CRITICAL(&stateMux);
  state.pendingCount = pending;
  portEXIT_CRITICAL(&stateMux);
  return changed;
}

size_t Session::actionCount(size_t index) const {
  if (xSemaphoreTake(modelMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return 0;
  }
  const size_t count = model.actionCount(index);
  xSemaphoreGive(modelMutex);
  return count;
}

bool Session::actionRow(size_t index, size_t slot, ActionRowView& view) const {
  if (xSemaphoreTake(modelMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }
  ActionItem now{};
  ActionItem before{};
  const bool valid = model.action(index, slot, now, before);
  if (valid) {
    copyText(view.name, sizeof(view.name), actionName(now.action));
    actionArgument(now, view.value, sizeof(view.value));
    view.changed = now.action != before.action || now.argument != before.argument;
    view.editable = isServoAction(now.action) && keyTable()[index].remoteWritable;
  }
  xSemaphoreGive(modelMutex);
  return valid;
}

bool Session::adjustAction(size_t index, size_t slot, int direction, int32_t multiplier) {
  if (xSemaphoreTake(modelMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }
  const bool changed = model.adjustAction(index, slot, direction, multiplier);
  const size_t pending = model.pendingCount();
  xSemaphoreGive(modelMutex);
  portENTER_CRITICAL(&stateMux);
  state.pendingCount = pending;
  portEXIT_CRITICAL(&stateMux);
  return changed;
}

void Session::discard() {
  if (xSemaphoreTake(modelMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return;
  }
  model.discard();
  xSemaphoreGive(modelMutex);
  portENTER_CRITICAL(&stateMux);
  state.pendingCount = 0;
  ++state.revision;
  portEXIT_CRITICAL(&stateMux);
}

size_t Session::rowCount() const {
  if (xSemaphoreTake(modelMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return 0;
  }
  const size_t count = model.diffRows().size();
  xSemaphoreGive(modelMutex);
  return count;
}

bool Session::row(size_t index, RowView& view) const {
  if (xSemaphoreTake(modelMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }
  const auto rows = model.diffRows();
  const bool valid = index < rows.size();
  if (valid) {
    const DiffRow& diff = rows[index];
    view.status = diff.status;
    if (diff.key >= 0) {
      const KeyInfo& info = keyTable()[diff.key];
      copyText(view.label, sizeof(view.label), info.label);
      if (info.kind == Kind::kActions) {
        describeActionChange(model.current(static_cast<size_t>(diff.key)), model.pending(static_cast<size_t>(diff.key)),
                             view.from, sizeof(view.from), view.to, sizeof(view.to));
      } else {
        displayValue(info, model.current(static_cast<size_t>(diff.key)), view.from, sizeof(view.from));
        displayValue(info, model.pending(static_cast<size_t>(diff.key)), view.to, sizeof(view.to));
      }
    } else {
      copyText(view.label, sizeof(view.label), diff.name);
      view.from[0] = '\0';
      view.to[0] = '\0';
    }
    if (diff.status == RowStatus::kInvalid || diff.status == RowStatus::kUnsupported) {
      copyText(view.label, sizeof(view.label), diff.name);
    }
  }
  xSemaphoreGive(modelMutex);
  return valid;
}

bool Session::fileName(size_t index, char* destination, size_t capacity) const {
  if (xSemaphoreTake(modelMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }
  const bool valid = index < files.size() && capacity > 0;
  if (valid) {
    copyText(destination, capacity, files[index].c_str());
  }
  xSemaphoreGive(modelMutex);
  return valid;
}

void Session::worker(void* parameter) {
  auto* self = static_cast<Session*>(parameter);
  Request request{};
  for (;;) {
    if (xQueueReceive(self->requests, &request, pdMS_TO_TICKS(10)) == pdPASS) {
      switch (request.action) {
        case Action::kOpen:
          if (self->connect(request.link, true)) {
            self->setPhase(Phase::kReady);
          }
          break;
        case Action::kRead: {
          Model fresh{};
          if (self->readInto(fresh)) {
            if (xSemaphoreTake(self->modelMutex, portMAX_DELAY) == pdTRUE) {
              self->model = fresh;
              xSemaphoreGive(self->modelMutex);
            }
            portENTER_CRITICAL(&self->stateMux);
            self->state.configLoaded = true;
            self->state.identity = fresh.identity();
            portEXIT_CRITICAL(&self->stateMux);
            self->setPhase(Phase::kReady);
          }
        } break;
        case Action::kWrite:
          self->doWrite();
          break;
        case Action::kReboot:
          self->doReboot(request.mode, false);
          break;
        case Action::kQuickReboot:
          if (self->connect(request.link, false)) {
            self->doReboot(request.mode, true);
          }
          break;
        case Action::kSave:
          self->doSave();
          break;
        case Action::kBrowse:
          self->doBrowse();
          break;
        case Action::kLoad:
          self->doLoad(request.index);
          break;
        case Action::kClose:
          self->disconnect();
          self->setPhase(Phase::kIdle);
          break;
      }
      // A failed connection never leaves the radio in bidirectional mode.
      if (self->active != nullptr && !self->snapshot().sessionOpen) {
        self->disconnect();
      }
    }
    if (self->active != nullptr) {
      self->pump();
      const Snapshot current = self->snapshot();
      if (current.sessionOpen && current.phase != Phase::kBusy && millis() - self->lastStatusSeen > kLinkLostMs) {
        self->disconnect();
        self->result(false, "Connection lost", "The Vega stopped answering; the session was closed");
      }
    }
  }
}

}  // namespace VegaConfig
