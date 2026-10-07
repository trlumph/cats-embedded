/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Ground Station <-> Vega remote configuration protocol.
///
/// This header is shared verbatim between flight_computer/src/comm and
/// ground_station/src/remote. Keep both copies byte-identical; the host test in
/// flight_computer/test/remote_config compares them.
///
/// The protocol reuses the existing 15-byte radio payload in both directions and
/// does not require any change to the telemetry (radio MCU) firmware:
///
///  * Uplink (GS -> Vega) frames start with kUplinkHeader (the testing frames use
///    0x72) and carry the CRC32 of the testing phrase as passcode.
///  * Downlink (Vega -> GS) frames replace individual telemetry packets while a
///    session is active. They set byte 14 (the always-zero telemetry dummy byte)
///    to kDownlinkMarker and keep the low three bits of byte 0 (the telemetry
///    flight state) at zero, so older Ground Stations see an INVALID state and
///    never start a flight log because of them.
///
/// See ground_station/VEGA-REMOTE-CONFIG.md for the full description.

// Include guard instead of #pragma once: host tests include both identical copies.
#ifndef CATS_REMOTE_CONFIG_PROTOCOL_HPP
#define CATS_REMOTE_CONFIG_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace remote_config {

inline constexpr uint8_t kProtocolVersion = 1;

/// Radio payload bytes used by the protocol in both directions.
inline constexpr size_t kFrameSize = 15;

/// Uplink header byte. Testing mode frames use 0x72.
inline constexpr uint8_t kUplinkHeader = 0x73;
inline constexpr size_t kUplinkArgsSize = 8;

/// Downlink marker stored in the last payload byte.
inline constexpr uint8_t kDownlinkMarker = 0xC7;
inline constexpr size_t kMarkerIndex = kFrameSize - 1;

/// Payload bytes carried by one DATA frame.
inline constexpr size_t kDataChunkSize = 11;
/// Payload bytes carried by one LINE_DATA uplink frame.
inline constexpr size_t kLineChunkSize = 6;
/// Longest accepted "key=value" line, without terminator.
inline constexpr size_t kMaxLineLength = 160;
/// Upper bound of the configuration text produced by the Vega.
inline constexpr size_t kMaxTextLength = 2048;

/// Arguments that must accompany destructive operations.
inline constexpr uint8_t kCommitMagic[4] = {'S', 'A', 'V', 'E'};
inline constexpr uint8_t kRebootMagic[4] = {'B', 'O', 'O', 'T'};

enum class Op : uint8_t {
  kNop = 0,        ///< Keep the session alive and request a status frame.
  kRead = 1,       ///< Start streaming the configuration text.
  kBegin = 2,      ///< Start a write transaction (staging copy of the current configuration).
  kLineData = 3,   ///< args: offset, length, up to kLineChunkSize bytes of a "key=value" line.
  kLineApply = 4,  ///< args: token, length, CRC32 (LE). Parses the line into the staging copy.
  kCommit = 5,     ///< args: token, kCommitMagic. Saves the staging copy and reboots the Vega.
  kAbort = 6,      ///< Discards the staging copy.
  kReboot = 7,     ///< args: token, RebootMode, kRebootMagic.
};

enum class Result : uint8_t {
  kNone = 0,
  kOk = 1,
  kBadState = 2,      ///< Not allowed in the current flight state or while logging.
  kNoStaging = 3,     ///< kBegin was not received in this session.
  kBadCrc = 4,        ///< Line length or CRC does not match the received chunks.
  kBadKey = 5,        ///< Unknown setting name.
  kBadValue = 6,      ///< Value rejected by the setting limits.
  kSaveFailed = 7,    ///< Writing the configuration to flash failed.
  kBadArgs = 8,       ///< Malformed arguments or wrong magic.
  kBadToken = 9,      ///< Token does not match this boot of the Vega.
  kUnsupported = 10,  ///< Unknown operation.
};

enum class RebootMode : uint8_t { kNormal = 0, kTesting = 1 };

enum class DownlinkType : uint8_t { kNone = 0, kStatus = 1, kData = 2 };

/// Status flags.
inline constexpr uint8_t kFlagStaging = 1U << 0U;
inline constexpr uint8_t kFlagTestingMode = 1U << 1U;
inline constexpr uint8_t kFlagRebootPending = 1U << 2U;
inline constexpr uint8_t kFlagWritable = 1U << 3U;
inline constexpr uint8_t kFlagTextReady = 1U << 4U;

struct UplinkFrame {
  uint32_t passcode{0};
  uint8_t seq{0};
  Op op{Op::kNop};
  uint8_t args[kUplinkArgsSize]{};
};

struct StatusFrame {
  uint8_t version{kProtocolVersion};
  uint8_t seq{0};
  Result result{Result::kNone};
  uint8_t flags{0};
  uint8_t fsmState{0};
  uint16_t textLength{0};
  uint32_t textCrc{0};
  uint8_t valueCount{0};
  uint8_t token{0};
};

struct DataFrame {
  uint16_t offset{0};
  uint8_t data[kDataChunkSize]{};
};

inline void PutU16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value & 0xFFU);
  out[1] = static_cast<uint8_t>(value >> 8U);
}

inline void PutU32(uint8_t* out, uint32_t value) {
  for (size_t i = 0; i < 4; ++i) {
    out[i] = static_cast<uint8_t>((value >> (8U * i)) & 0xFFU);
  }
}

inline uint16_t GetU16(const uint8_t* in) { return static_cast<uint16_t>(in[0] | (in[1] << 8U)); }

inline uint32_t GetU32(const uint8_t* in) {
  return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8U) | (static_cast<uint32_t>(in[2]) << 16U) |
         (static_cast<uint32_t>(in[3]) << 24U);
}

inline void EncodeUplink(const UplinkFrame& frame, uint8_t out[kFrameSize]) {
  out[0] = kUplinkHeader;
  PutU32(&out[1], frame.passcode);
  out[5] = frame.seq;
  out[6] = static_cast<uint8_t>(frame.op);
  memcpy(&out[7], frame.args, kUplinkArgsSize);
}

inline bool DecodeUplink(const uint8_t* in, size_t length, UplinkFrame* frame) {
  if (length < kFrameSize || in[0] != kUplinkHeader) {
    return false;
  }
  frame->passcode = GetU32(&in[1]);
  frame->seq = in[5];
  frame->op = static_cast<Op>(in[6]);
  memcpy(frame->args, &in[7], kUplinkArgsSize);
  return true;
}

inline void EncodeStatus(const StatusFrame& status, uint8_t out[kFrameSize]) {
  memset(out, 0, kFrameSize);
  out[0] = static_cast<uint8_t>(static_cast<uint8_t>(DownlinkType::kStatus) << 3U);
  out[1] = status.version;
  out[2] = status.seq;
  out[3] = static_cast<uint8_t>(status.result);
  out[4] = status.flags;
  out[5] = status.fsmState;
  PutU16(&out[6], status.textLength);
  PutU32(&out[8], status.textCrc);
  out[12] = status.valueCount;
  out[13] = status.token;
  out[kMarkerIndex] = kDownlinkMarker;
}

inline void EncodeData(const DataFrame& data, uint8_t out[kFrameSize]) {
  memset(out, 0, kFrameSize);
  out[0] = static_cast<uint8_t>(static_cast<uint8_t>(DownlinkType::kData) << 3U);
  PutU16(&out[1], data.offset);
  memcpy(&out[3], data.data, kDataChunkSize);
  out[kMarkerIndex] = kDownlinkMarker;
}

/// Returns the type of a downlink frame, or kNone for ordinary telemetry.
inline DownlinkType DownlinkTypeOf(const uint8_t* in, size_t length) {
  if (length < kFrameSize || in[kMarkerIndex] != kDownlinkMarker || (in[0] & 0x07U) != 0U) {
    return DownlinkType::kNone;
  }
  const auto type = static_cast<DownlinkType>(in[0] >> 3U);
  if (type != DownlinkType::kStatus && type != DownlinkType::kData) {
    return DownlinkType::kNone;
  }
  return type;
}

inline void DecodeStatus(const uint8_t* in, StatusFrame* status) {
  status->version = in[1];
  status->seq = in[2];
  status->result = static_cast<Result>(in[3]);
  status->flags = in[4];
  status->fsmState = in[5];
  status->textLength = GetU16(&in[6]);
  status->textCrc = GetU32(&in[8]);
  status->valueCount = in[12];
  status->token = in[13];
}

inline void DecodeData(const uint8_t* in, DataFrame* data) {
  data->offset = GetU16(&in[1]);
  memcpy(data->data, &in[3], kDataChunkSize);
}

inline const char* ResultName(Result result) {
  switch (result) {
    case Result::kNone:
      return "No answer";
    case Result::kOk:
      return "OK";
    case Result::kBadState:
      return "Not allowed now";
    case Result::kNoStaging:
      return "No write session";
    case Result::kBadCrc:
      return "Transfer error";
    case Result::kBadKey:
      return "Unknown setting";
    case Result::kBadValue:
      return "Value rejected";
    case Result::kSaveFailed:
      return "Flash save failed";
    case Result::kBadArgs:
      return "Bad request";
    case Result::kBadToken:
      return "Stale request";
    case Result::kUnsupported:
      return "Unsupported";
  }
  return "Unknown";
}

}  // namespace remote_config

#endif  // CATS_REMOTE_CONFIG_PROTOCOL_HPP
