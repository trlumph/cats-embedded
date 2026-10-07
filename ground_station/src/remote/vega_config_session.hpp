/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Background worker for the Vega remote configuration (read, edit, profiles, write,
/// reboot into testing mode). The HMI submits requests and polls snapshot(); all radio
/// and storage work runs in the worker task.

#pragma once

#include <Arduino.h>

#include <atomic>
#include <string>
#include <vector>

#include "logging/recorder.hpp"
#include "remote/vega_config_client.hpp"
#include "remote/vega_profile.hpp"
#include "telemetry/telemetry.hpp"

namespace VegaConfig {

enum class Phase : uint8_t {
  kIdle,      ///< No session.
  kBusy,      ///< Working; see Snapshot::activity and percent.
  kReady,     ///< Session open, configuration available.
  kBrowsing,  ///< Profile list available.
  kReview,    ///< A profile was loaded; pending changes ready for review.
  kResult,    ///< Result message; acknowledge() returns to kReady or kIdle.
};

struct Snapshot {
  Phase phase{Phase::kIdle};
  uint32_t revision{0};
  uint8_t link{0};  ///< 1 or 2 while a session is open.
  uint8_t percent{0};
  char activity[40]{};
  char title[32]{};
  char message[96]{};
  bool success{false};
  bool sessionOpen{false};
  bool configLoaded{false};
  uint8_t fsmState{0};
  uint8_t flags{0};
  Identity identity{};
  size_t pendingCount{0};
  size_t fileCount{0};
  /// The FAT volume was used; the HMI may share it with the PC again.
  bool storageUsed{false};
};

struct EntryView {
  char label[32]{};
  char value[40]{};
  bool known{false};
  bool changed{false};
  bool editable{false};
  bool actions{false};  ///< Event action list; opens the action view.
};

struct ActionRowView {
  char name[16]{};
  char value[24]{};
  bool changed{false};
  bool editable{false};  ///< Servo position.
};

struct RowView {
  RowStatus status{RowStatus::kChange};
  char label[32]{};
  char from[40]{};
  char to[40]{};
};

class Session final : public IRemoteConfigSink {
 public:
  Session(Recorder& recorder, Telemetry& first, Telemetry& second) : recorder(recorder), links{&first, &second} {}
  bool begin();

  /// Opens a session on link 1 or 2 and reads the configuration.
  bool open(uint8_t link);
  bool read();
  bool write();
  bool reboot(remote_config::RebootMode mode);
  /// Opens a short session only to reboot the Vega (used by the Testing screen).
  bool quickReboot(uint8_t link, remote_config::RebootMode mode);
  bool saveProfile();
  bool browseProfiles();
  bool loadProfile(size_t index);
  bool close();
  /// Stops the running radio operation (connect, read, write, restart) as soon as possible.
  void cancel();
  /// Leaves kResult (to kReady if the session is still open, else kIdle) or kReview/kBrowsing.
  bool acknowledge();

  [[nodiscard]] Snapshot snapshot() const;
  [[nodiscard]] bool idle() const;

  /* Model access from the HMI task. */
  size_t entryCount() const;
  bool entry(size_t index, EntryView& view) const;
  bool adjust(size_t index, int direction, int32_t multiplier);
  size_t actionCount(size_t index) const;
  bool actionRow(size_t index, size_t slot, ActionRowView& view) const;
  bool adjustAction(size_t index, size_t slot, int direction, int32_t multiplier);
  void discard();
  size_t rowCount() const;
  bool row(size_t index, RowView& view) const;
  bool fileName(size_t index, char* destination, size_t capacity) const;

  /* IRemoteConfigSink, called from the telemetry task of the session link. */
  void onRadioPacket(uint8_t source, const uint8_t* payload, bool configFrame) override;

 private:
  enum class Action : uint8_t { kOpen, kRead, kWrite, kReboot, kQuickReboot, kSave, kBrowse, kLoad, kClose };
  struct Request {
    Action action{Action::kOpen};
    uint8_t link{0};
    remote_config::RebootMode mode{remote_config::RebootMode::kNormal};
    size_t index{0};
  };
  struct Packet {
    uint8_t payload[remote_config::kFrameSize]{};
    bool config{false};
  };

  bool submit(const Request& request, const char* activity);
  void setBusy(const char* activity, uint8_t percent = 0);
  void setPhase(Phase phase);
  void result(bool success, const char* title, const char* message);
  void publishStatus();

  bool connect(uint8_t link, bool readConfig);
  bool runClient(uint32_t timeoutMs, const char* activity);
  bool readInto(Model& target);
  void doWrite();
  void doReboot(remote_config::RebootMode mode, bool quick);
  void doSave();
  void doBrowse();
  void doLoad(size_t index);
  void disconnect();
  void pump();
  bool claimStorage(const char** error);
  void releaseStorage();

  static void worker(void* parameter);

  Recorder& recorder;
  Telemetry* links[2];
  Telemetry* active{nullptr};
  uint8_t activeSource{0};

  QueueHandle_t requests{nullptr};
  QueueHandle_t packets{nullptr};
  SemaphoreHandle_t modelMutex{nullptr};
  mutable portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
  Snapshot state{};
  bool busy{false};
  std::atomic<uint8_t> sinkSource{0};

  Client client{};
  Model model{};
  std::vector<std::string> files{};
  bool storageOwned{false};
  bool recorderPaused{false};
  std::atomic<bool> cancelRequested{false};
  uint32_t lastStatusSeen{0};
};

}  // namespace VegaConfig
