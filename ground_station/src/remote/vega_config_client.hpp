/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Ground Station side of the remote configuration protocol (no Arduino dependencies).
///
/// The receiver radio answers every Vega packet with the last payload written to
/// it. The client is therefore paced by received packets: onPacket() is called
/// for every packet of the session link and returns the payload that the radio
/// shall send with its next answer. Operations are acknowledged through status
/// frames; until then the radio keeps repeating the request, which also covers
/// lost frames.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "remote/remote_config_protocol.hpp"

namespace VegaConfig {

inline constexpr uint32_t kConnectTimeoutMs = 6000;
inline constexpr uint32_t kAckTimeoutMs = 8000;
inline constexpr uint32_t kReadTimeoutMs = 60000;
/// Attempts per line. Retries send every chunk twice to get through a lossy uplink.
inline constexpr uint8_t kLineRetries = 6;

class Client {
 public:
  enum class Task : uint8_t { kNone, kConnect, kRead, kWrite, kReboot };
  enum class State : uint8_t { kIdle, kBusy, kDone, kFailed };

  void begin(uint32_t passcode, uint8_t seqSeed);
  void startConnect(uint32_t now);
  void startRead(uint32_t now);
  void startWrite(const std::vector<std::string>& lines, uint32_t now);
  void startReboot(remote_config::RebootMode mode, uint32_t now);
  /// Stops the current task; the radio then repeats a keep-alive frame.
  void stop();

  /// Handles a downlink configuration frame of the session link.
  void onDownlink(const uint8_t* frame, uint32_t now);
  /// Called for every packet received on the session link. Returns true and fills
  /// out with the payload that the radio shall send next.
  bool onPacket(uint8_t out[remote_config::kFrameSize], uint32_t now);
  /// Handles timeouts.
  void tick(uint32_t now);

  [[nodiscard]] Task task() const { return currentTask; }
  [[nodiscard]] State state() const { return currentState; }
  [[nodiscard]] uint8_t percent() const;
  [[nodiscard]] const char* error() const { return errorText; }
  [[nodiscard]] bool haveStatus() const { return statusSeen; }
  [[nodiscard]] const remote_config::StatusFrame& status() const { return lastStatus; }
  [[nodiscard]] uint32_t lastStatusMs() const { return lastStatusAt; }
  [[nodiscard]] const char* text() const { return textBuffer; }
  [[nodiscard]] size_t textLength() const { return textReady ? expectedLength : 0; }
  /// Index of the line that failed during a write, or -1.
  [[nodiscard]] int failedLine() const { return failedLineIndex; }

 private:
  enum class Step : uint8_t { kIdle, kWaitStatus, kWaitAck, kLineChunks, kCollect };

  uint8_t nextSeq();
  void fail(const char* message);
  void finish();
  void startOp(remote_config::Op op, uint32_t now);
  void startLine(uint32_t now);
  void handleAck(remote_config::Result result, uint32_t now);
  void handleStatus(const remote_config::StatusFrame& status, uint32_t now);
  void handleData(const remote_config::DataFrame& data);
  void checkReadComplete();
  [[nodiscard]] bool restartedSinceRequest() const;
  void buildFrame(uint8_t out[remote_config::kFrameSize]);
  void resetCollection();

  uint32_t passcode{0};
  uint8_t seq{1};
  Task currentTask{Task::kNone};
  State currentState{State::kIdle};
  Step step{Step::kIdle};
  char errorText[64]{};
  uint32_t stepStarted{0};

  remote_config::StatusFrame lastStatus{};
  bool statusSeen{false};
  uint32_t lastStatusAt{0};

  remote_config::UplinkFrame request{};

  // Read
  char textBuffer[remote_config::kMaxTextLength + 1]{};
  bool received[(remote_config::kMaxTextLength + remote_config::kDataChunkSize - 1) / remote_config::kDataChunkSize]{};
  size_t expectedLength{0};
  uint32_t expectedCrc{0};
  bool lengthKnown{false};
  bool textReady{false};

  // Write
  std::vector<std::string> lines;
  size_t lineIndex{0};
  size_t chunkOffset{0};
  bool chunkRepeated{false};
  uint8_t lineAttempts{0};
  int failedLineIndex{-1};
  bool committing{false};

  remote_config::RebootMode rebootMode{remote_config::RebootMode::kNormal};
};

}  // namespace VegaConfig
