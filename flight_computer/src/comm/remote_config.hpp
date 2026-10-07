/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Vega side of the Ground Station remote configuration protocol.
///
/// The server is platform independent so that it can be exercised by host
/// tests. Hardware access (time, flight state, flash and reset) is injected via
/// ServerEnvironment. All methods are called from the telemetry task only.

#pragma once

#include <cstddef>
#include <cstdint>

#include "cli/settings.hpp"
#include "comm/remote_config_protocol.hpp"
#include "config/cats_config.hpp"

namespace remote_config {

/// Time a session stays open without a valid uplink frame.
inline constexpr uint32_t kSessionTimeoutMs = 5000;
/// Delay between acknowledging a commit/reboot and resetting the MCU, so the
/// acknowledgement reaches the Ground Station.
inline constexpr uint32_t kRebootDelayMs = 1500;
/// One of kDownlinkCycle radio packets stays ordinary telemetry during a session.
inline constexpr uint32_t kDownlinkCycle = 10;

class ServerEnvironment {
 public:
  ServerEnvironment(const ServerEnvironment&) = delete;
  ServerEnvironment& operator=(const ServerEnvironment&) = delete;
  ServerEnvironment(ServerEnvironment&&) = delete;
  ServerEnvironment& operator=(ServerEnvironment&&) = delete;

  virtual uint32_t NowMs() = 0;
  /// Current flight_fsm_e value.
  virtual uint8_t FlightState() = 0;
  /// True when the Vega was started in testing mode.
  virtual bool TestingMode() = 0;
  /// True while the recorder writes a flight log to flash.
  virtual bool RecorderWriting() = 0;
  /// Copies the configuration as stored in flash (runtime-only overrides removed).
  virtual void CopyStoredConfig(cats_config_t* out) = 0;
  /// Writes the configuration to flash. Must not modify the running configuration.
  virtual bool SaveConfig(const cats_config_t& config) = 0;
  /// Resets the MCU. Does not return on the target.
  virtual void Reboot(RebootMode mode) = 0;
  virtual const char* BoardName() = 0;
  virtual const char* FirmwareVersion() = 0;
  virtual const char* TelemetryVersion() = 0;

 protected:
  ServerEnvironment() = default;
  /* Not virtual: environments are never deleted through this interface (tasks cannot be deleted). */
  ~ServerEnvironment() = default;
};

/// Formats a setting value exactly like the CLI 'get'/'dump' output (without limits).
/// Returns the number of characters written (excluding the terminator), 0 on error.
size_t FormatValue(const cats_config_t& config, const cli_value_t& setting, char* out, size_t capacity);

/// Parses "key=value" and stores the value in config. The line does not need a terminator.
Result ApplyLine(cats_config_t* config, const char* line, size_t length);

/// Builds the configuration text: identity lines starting with '#' followed by one
/// "key=value" line per setting. Returns the text length, or 0 if it does not fit.
size_t BuildConfigText(const cats_config_t& config, const char* board, const char* firmware, const char* telemetry,
                       char* out, size_t capacity);

class Server {
 public:
  explicit Server(ServerEnvironment& environment) : m_env(environment) {}

  /// Enables the server. A session can only be opened with frames carrying passcode.
  void Configure(uint32_t passcode, uint8_t token);

  /// Handles a received radio payload. Returns true if it was a remote configuration frame.
  bool OnUplink(const uint8_t* data, size_t length);

  /// Called once per telemetry period. Returns true and fills out if a configuration
  /// frame must be transmitted instead of the telemetry packet.
  bool NextDownlink(uint8_t out[kFrameSize]);

  [[nodiscard]] bool SessionActive() const { return m_session_active; }

 private:
  [[nodiscard]] bool InFlight();
  [[nodiscard]] bool Writable();
  void EndSession();
  Result Handle(const UplinkFrame& frame);
  Result HandleRead();
  Result HandleBegin();
  Result HandleLineApply(const UplinkFrame& frame);
  Result HandleCommit(const UplinkFrame& frame);
  Result HandleReboot(const UplinkFrame& frame);
  void EncodeStatusFrame(uint8_t out[kFrameSize]);

  ServerEnvironment& m_env;
  bool m_enabled{false};
  uint32_t m_passcode{0};
  uint8_t m_token{1};

  bool m_session_active{false};
  uint32_t m_last_activity_ms{0};
  bool m_have_seq{false};
  uint8_t m_last_seq{0};
  Op m_last_op{Op::kNop};
  uint8_t m_last_args[kUplinkArgsSize]{};
  Result m_last_result{Result::kNone};
  uint32_t m_cycle{0};

  bool m_reading{false};
  char m_text[kMaxTextLength]{};
  size_t m_text_length{0};
  uint32_t m_text_crc{0};
  size_t m_next_offset{0};

  bool m_staging_active{false};
  cats_config_t m_staging{};
  char m_line[kMaxLineLength]{};

  bool m_reboot_pending{false};
  RebootMode m_reboot_mode{RebootMode::kNormal};
  uint32_t m_reboot_at_ms{0};
};

}  // namespace remote_config
