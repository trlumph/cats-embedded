/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "comm/remote_config.hpp"

#include <strings.h>
#include <climits>
#include <cstring>

#include "util/crc.hpp"
#include "util/enum_str_maps.hpp"

namespace remote_config {

namespace {

// flight_fsm_e values; kept numeric so the server does not depend on the FSM headers.
constexpr uint8_t kStateInvalid = 0;
constexpr uint8_t kStateCalibrating = 1;
constexpr uint8_t kStateReady = 2;
constexpr uint8_t kStateThrusting = 3;

const void* MemberPtr(const cats_config_t& config, const cli_value_t& setting) {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  return reinterpret_cast<const uint8_t*>(&config) + setting.member_offset;
}

void* MemberPtr(cats_config_t* config, const cli_value_t& setting) {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  return reinterpret_cast<uint8_t*>(config) + setting.member_offset;
}

uint32_t TypeOf(const cli_value_t& setting) { return setting.type & VALUE_TYPE_MASK; }
uint32_t ModeOf(const cli_value_t& setting) { return setting.type & VALUE_MODE_MASK; }

int64_t LoadElement(const void* base, uint32_t type, size_t index) {
  switch (type) {
    case VAR_INT8:
      return static_cast<const int8_t*>(base)[index];
    case VAR_UINT16:
      return static_cast<const uint16_t*>(base)[index];
    case VAR_INT16:
      return static_cast<const int16_t*>(base)[index];
    case VAR_UINT32:
      return static_cast<const uint32_t*>(base)[index];
    case VAR_UINT8:
    default:
      return static_cast<const uint8_t*>(base)[index];
  }
}

bool ElementInTypeRange(uint32_t type, int64_t value) {
  switch (type) {
    case VAR_INT8:
      return value >= INT8_MIN && value <= INT8_MAX;
    case VAR_UINT16:
      return value >= 0 && value <= UINT16_MAX;
    case VAR_INT16:
      return value >= INT16_MIN && value <= INT16_MAX;
    case VAR_UINT32:
      return value >= 0 && value <= static_cast<int64_t>(UINT32_MAX);
    case VAR_UINT8:
      return value >= 0 && value <= UINT8_MAX;
    default:
      return false;
  }
}

void StoreElement(void* base, uint32_t type, size_t index, int64_t value) {
  switch (type) {
    case VAR_INT8:
      static_cast<int8_t*>(base)[index] = static_cast<int8_t>(value);
      break;
    case VAR_UINT16:
      static_cast<uint16_t*>(base)[index] = static_cast<uint16_t>(value);
      break;
    case VAR_INT16:
      static_cast<int16_t*>(base)[index] = static_cast<int16_t>(value);
      break;
    case VAR_UINT32:
      static_cast<uint32_t*>(base)[index] = static_cast<uint32_t>(value);
      break;
    case VAR_UINT8:
    default:
      static_cast<uint8_t*>(base)[index] = static_cast<uint8_t>(value);
      break;
  }
}

/// Mirrors get_min_max() from the CLI without depending on the USB console.
void SettingMinMax(const cli_value_t& setting, int64_t* min, int64_t* max) {
  switch (TypeOf(setting)) {
    case VAR_UINT8:
    case VAR_UINT16:
      *min = setting.config.minmax_unsigned.min;
      *max = setting.config.minmax_unsigned.max;
      break;
    default:
      *min = setting.config.minmax.min;
      *max = setting.config.minmax.max;
      break;
  }
}

/// Strict decimal integer parser: optional sign, at least one digit, nothing else.
bool ParseInteger(const char* text, size_t length, int64_t* value) {
  if (length == 0 || length > 12) {
    return false;
  }
  size_t index = 0;
  bool negative = false;
  if (text[0] == '-' || text[0] == '+') {
    negative = text[0] == '-';
    index = 1;
  }
  if (index == length) {
    return false;
  }
  int64_t result = 0;
  for (; index < length; ++index) {
    if (text[index] < '0' || text[index] > '9') {
      return false;
    }
    result = result * 10 + (text[index] - '0');
  }
  *value = negative ? -result : result;
  return true;
}

void Trim(const char** text, size_t* length) {
  while (*length > 0 && (**text == ' ' || **text == '\t')) {
    ++*text;
    --*length;
  }
  while (*length > 0 && ((*text)[*length - 1] == ' ' || (*text)[*length - 1] == '\t' || (*text)[*length - 1] == '\r' ||
                         (*text)[*length - 1] == '\n')) {
    --*length;
  }
}

const cli_value_t* FindSetting(const char* name, size_t length) {
  for (uint16_t i = 0; i < value_table_entry_count; ++i) {
    const char* candidate = value_table[i].name;
    if (strlen(candidate) == length && strncmp(candidate, name, length) == 0) {
      return &value_table[i];
    }
  }
  return nullptr;
}

/// Appends formatted text, tracking overflow.
class Writer {
 public:
  Writer(char* out, size_t capacity) : m_out(out), m_capacity(capacity) {
    if (m_capacity > 0) {
      m_out[0] = '\0';
    }
  }

  void Append(const char* text) { Append(text, strlen(text)); }

  void Append(const char* text, size_t length) {
    if (m_overflow || m_length + length + 1 > m_capacity) {
      m_overflow = true;
      return;
    }
    memcpy(m_out + m_length, text, length);
    m_length += length;
    m_out[m_length] = '\0';
  }

  void AppendInteger(int64_t value) {
    // newlib-nano printf has no 64-bit support; format manually.
    char buffer[24]{};
    size_t index = sizeof(buffer) - 1;
    const bool negative = value < 0;
    uint64_t magnitude = negative ? static_cast<uint64_t>(-(value + 1)) + 1U : static_cast<uint64_t>(value);
    buffer[--index] = static_cast<char>('0' + (magnitude % 10U));
    magnitude /= 10U;
    while (magnitude > 0U && index > 1) {
      buffer[--index] = static_cast<char>('0' + (magnitude % 10U));
      magnitude /= 10U;
    }
    if (negative) {
      buffer[--index] = '-';
    }
    Append(&buffer[index], sizeof(buffer) - 1 - index);
  }

  [[nodiscard]] size_t Length() const { return m_overflow ? 0 : m_length; }

 private:
  char* m_out;
  size_t m_capacity;
  size_t m_length{0};
  bool m_overflow{false};
};

}  // namespace

size_t FormatValue(const cats_config_t& config, const cli_value_t& setting, char* out, size_t capacity) {
  Writer writer(out, capacity);
  const void* ptr = MemberPtr(config, setting);
  const uint32_t type = TypeOf(setting);

  switch (ModeOf(setting)) {
    case MODE_ARRAY:
      for (size_t i = 0; i < setting.config.array.length; ++i) {
        if (i > 0) {
          writer.Append(",");
        }
        writer.AppendInteger(LoadElement(ptr, type, i));
      }
      break;
    case MODE_DIRECT:
      writer.AppendInteger(LoadElement(ptr, type, 0));
      break;
    case MODE_LOOKUP: {
      const auto index = LoadElement(ptr, type, 0);
      const auto& table = lookup_tables[setting.config.lookup.table_index];
      if (index < 0 || static_cast<size_t>(index) >= table.size() || table[static_cast<size_t>(index)] == nullptr) {
        return 0;
      }
      writer.Append(table[static_cast<size_t>(index)]);
    } break;
    case MODE_BITSET: {
      const auto value = static_cast<uint32_t>(LoadElement(ptr, type, 0));
      writer.Append(((value >> setting.config.bitpos) & 1U) != 0U ? "ON" : "OFF");
    } break;
    case MODE_STRING: {
      const auto* text = static_cast<const char*>(ptr);
      size_t length = 0;
      while (length < setting.config.string.max_length && text[length] != '\0') {
        ++length;
      }
      if (length == 0) {
        writer.Append("-");
      } else {
        writer.Append(text, length);
      }
    } break;
    default:
      return 0;
  }
  return writer.Length();
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
Result ApplyLine(cats_config_t* config, const char* line, size_t length) {
  const auto* equals = static_cast<const char*>(memchr(line, '=', length));
  if (equals == nullptr) {
    return Result::kBadArgs;
  }
  const char* key = line;
  auto key_length = static_cast<size_t>(equals - line);
  const char* value = equals + 1;
  size_t value_length = length - key_length - 1;
  Trim(&key, &key_length);
  Trim(&value, &value_length);

  const cli_value_t* setting = FindSetting(key, key_length);
  if (setting == nullptr) {
    return Result::kBadKey;
  }
  void* ptr = MemberPtr(config, *setting);
  const uint32_t type = TypeOf(*setting);

  switch (ModeOf(*setting)) {
    case MODE_DIRECT: {
      int64_t parsed = 0;
      if (!ParseInteger(value, value_length, &parsed)) {
        return Result::kBadValue;
      }
      if (type == VAR_UINT32) {
        if (parsed < 0 || parsed > static_cast<int64_t>(setting->config.u32_max)) {
          return Result::kBadValue;
        }
      } else {
        int64_t min = 0;
        int64_t max = 0;
        SettingMinMax(*setting, &min, &max);
        if (parsed < min || parsed > max || !ElementInTypeRange(type, parsed)) {
          return Result::kBadValue;
        }
      }
      StoreElement(ptr, type, 0, parsed);
    } break;

    case MODE_LOOKUP: {
      const auto& table = lookup_tables[setting->config.lookup.table_index];
      bool matched = false;
      for (size_t index = 0; index < table.size() && !matched; ++index) {
        const char* candidate = table[index];
        if (candidate != nullptr && strlen(candidate) == value_length &&
            strncasecmp(candidate, value, value_length) == 0) {
          StoreElement(ptr, type, 0, static_cast<int64_t>(index));
          matched = true;
        }
      }
      if (!matched) {
        return Result::kBadValue;
      }
    } break;

    case MODE_ARRAY: {
      constexpr size_t kMaxArrayLength = 16;
      const size_t array_length = setting->config.array.length;
      if (array_length > kMaxArrayLength) {
        return Result::kBadValue;
      }
      int64_t parsed[kMaxArrayLength]{};
      size_t count = 0;
      const char* cursor = value;
      size_t remaining = value_length;
      while (true) {
        const auto* comma = static_cast<const char*>(memchr(cursor, ',', remaining));
        const size_t item_length = comma != nullptr ? static_cast<size_t>(comma - cursor) : remaining;
        const char* item = cursor;
        size_t trimmed_length = item_length;
        Trim(&item, &trimmed_length);
        if (count >= array_length || !ParseInteger(item, trimmed_length, &parsed[count]) ||
            !ElementInTypeRange(type, parsed[count])) {
          return Result::kBadValue;
        }
        ++count;
        if (comma == nullptr) {
          break;
        }
        remaining -= item_length + 1;
        cursor = comma + 1;
      }
      for (size_t index = 0; index < array_length; ++index) {
        StoreElement(ptr, type, index, index < count ? parsed[index] : 0);
      }
    } break;

    case MODE_STRING: {
      const uint8_t min = setting->config.string.min_length;
      const uint8_t max = setting->config.string.max_length;
      auto* text = static_cast<char*>(ptr);
      const bool clear = value_length == 1 && value[0] == '-';
      if (!clear) {
        if (value_length < min || value_length > max) {
          return Result::kBadValue;
        }
        for (size_t index = 0; index < value_length; ++index) {
          if (value[index] < 33 || value[index] > 126) {
            return Result::kBadValue;
          }
        }
      }
      // The storage holds max + 1 bytes (terminator).
      memset(text, 0, static_cast<size_t>(max) + 1U);
      if (!clear) {
        memcpy(text, value, value_length);
      }
    } break;

    default:
      return Result::kBadValue;
  }

  config->is_set_by_user = true;
  return Result::kOk;
}

size_t BuildConfigText(const cats_config_t& config, const char* board, const char* firmware, const char* telemetry,
                       char* out, size_t capacity) {
  Writer writer(out, capacity);
  writer.Append("#board=");
  writer.Append(board);
  writer.Append("\n#fw=");
  writer.Append(firmware);
  writer.Append("\n#tele=");
  writer.Append(telemetry);
  writer.Append("\n#proto=");
  writer.AppendInteger(kProtocolVersion);
  writer.Append("\n");
  char value[kMaxLineLength]{};
  for (uint16_t i = 0; i < value_table_entry_count; ++i) {
    if (FormatValue(config, value_table[i], value, sizeof(value)) == 0) {
      // A corrupted value is reported as an empty value; the GS treats it as unknown.
      value[0] = '\0';
    }
    writer.Append(value_table[i].name);
    writer.Append("=");
    writer.Append(value);
    writer.Append("\n");
  }
  return writer.Length();
}

void Server::Configure(uint32_t passcode, uint8_t token) {
  m_enabled = true;
  m_passcode = passcode;
  m_token = token == 0 ? 1 : token;
}

bool Server::InFlight() {
  if (m_env.TestingMode()) {
    return false;
  }
  // Sessions only before liftoff: configuration frames replace telemetry packets, and Ground
  // Stations without this feature read them as state INVALID, which would re-arm their recorder
  // after touchdown.
  const uint8_t state = m_env.FlightState();
  return state >= kStateThrusting;
}

bool Server::Writable() {
  if (m_env.RecorderWriting()) {
    return false;
  }
  if (m_env.TestingMode()) {
    return true;
  }
  const uint8_t state = m_env.FlightState();
  return state == kStateInvalid || state == kStateCalibrating || state == kStateReady;
}

void Server::EndSession() {
  m_session_active = false;
  m_reading = false;
  m_staging_active = false;
  m_have_seq = false;
  m_last_result = Result::kNone;
  memset(m_line, 0, sizeof(m_line));
}

bool Server::OnUplink(const uint8_t* data, size_t length) {
  UplinkFrame frame{};
  if (!DecodeUplink(data, length, &frame)) {
    return false;
  }
  if (!m_enabled || frame.passcode != m_passcode || InFlight()) {
    // Consumed but ignored: wrong phrase, or never during flight.
    return true;
  }

  const uint32_t now = m_env.NowMs();
  if (!m_session_active) {
    m_session_active = true;
    m_cycle = 0;
  }
  m_last_activity_ms = now;

  if (frame.op == Op::kLineData) {
    const uint8_t offset = frame.args[0];
    const uint8_t chunk = frame.args[1];
    if (chunk > 0 && chunk <= kLineChunkSize && static_cast<size_t>(offset) + chunk <= kMaxLineLength) {
      memcpy(&m_line[offset], &frame.args[2], chunk);
    }
    m_reading = false;
    return true;
  }

  if (frame.op == Op::kNop) {
    return true;
  }

  if (m_have_seq && frame.seq == m_last_seq && frame.op == m_last_op &&
      memcmp(frame.args, m_last_args, kUplinkArgsSize) == 0) {
    // The Ground Station radio repeats its last frame until it is replaced.
    return true;
  }

  m_last_result = Handle(frame);
  m_last_seq = frame.seq;
  m_last_op = frame.op;
  memcpy(m_last_args, frame.args, kUplinkArgsSize);
  m_have_seq = true;
  return true;
}

Result Server::Handle(const UplinkFrame& frame) {
  if (frame.op != Op::kRead) {
    m_reading = false;
  }
  switch (frame.op) {
    case Op::kRead:
      return HandleRead();
    case Op::kBegin:
      return HandleBegin();
    case Op::kLineApply:
      return HandleLineApply(frame);
    case Op::kCommit:
      return HandleCommit(frame);
    case Op::kAbort:
      m_staging_active = false;
      return Result::kOk;
    case Op::kReboot:
      return HandleReboot(frame);
    default:
      return Result::kUnsupported;
  }
}

Result Server::HandleRead() {
  cats_config_t stored{};
  m_env.CopyStoredConfig(&stored);
  m_text_length = BuildConfigText(stored, m_env.BoardName(), m_env.FirmwareVersion(), m_env.TelemetryVersion(), m_text,
                                  sizeof(m_text));
  if (m_text_length == 0) {
    return Result::kBadState;
  }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  m_text_crc = crc32(reinterpret_cast<const uint8_t*>(m_text), static_cast<uint32_t>(m_text_length));
  m_next_offset = 0;
  m_reading = true;
  return Result::kOk;
}

Result Server::HandleBegin() {
  if (!Writable()) {
    return Result::kBadState;
  }
  m_env.CopyStoredConfig(&m_staging);
  m_staging_active = true;
  memset(m_line, 0, sizeof(m_line));
  return Result::kOk;
}

Result Server::HandleLineApply(const UplinkFrame& frame) {
  if (frame.args[0] != m_token) {
    return Result::kBadToken;
  }
  if (!m_staging_active) {
    return Result::kNoStaging;
  }
  const size_t length = frame.args[1];
  const uint32_t expected = GetU32(&frame.args[2]);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  if (length == 0 || length > kMaxLineLength || crc32(reinterpret_cast<const uint8_t*>(m_line), length) != expected) {
    return Result::kBadCrc;
  }
  const Result result = ApplyLine(&m_staging, m_line, length);
  memset(m_line, 0, sizeof(m_line));
  return result;
}

Result Server::HandleCommit(const UplinkFrame& frame) {
  if (frame.args[0] != m_token) {
    return Result::kBadToken;
  }
  if (memcmp(&frame.args[1], kCommitMagic, sizeof(kCommitMagic)) != 0) {
    return Result::kBadArgs;
  }
  if (!m_staging_active) {
    return Result::kNoStaging;
  }
  if (!Writable()) {
    return Result::kBadState;
  }
  m_staging_active = false;
  if (!m_env.SaveConfig(m_staging)) {
    return Result::kSaveFailed;
  }
  m_reboot_pending = true;
  m_reboot_mode = RebootMode::kNormal;
  m_reboot_at_ms = m_env.NowMs() + kRebootDelayMs;
  return Result::kOk;
}

Result Server::HandleReboot(const UplinkFrame& frame) {
  if (frame.args[0] != m_token) {
    return Result::kBadToken;
  }
  const auto mode = static_cast<RebootMode>(frame.args[1]);
  if ((mode != RebootMode::kNormal && mode != RebootMode::kTesting) ||
      memcmp(&frame.args[2], kRebootMagic, sizeof(kRebootMagic)) != 0) {
    return Result::kBadArgs;
  }
  if (!Writable()) {
    return Result::kBadState;
  }
  m_staging_active = false;
  m_reboot_pending = true;
  m_reboot_mode = mode;
  m_reboot_at_ms = m_env.NowMs() + kRebootDelayMs;
  return Result::kOk;
}

void Server::EncodeStatusFrame(uint8_t out[kFrameSize]) {
  StatusFrame status{};
  status.seq = m_have_seq ? m_last_seq : 0;
  status.result = m_last_result;
  status.flags =
      static_cast<uint8_t>((m_staging_active ? kFlagStaging : 0U) | (m_env.TestingMode() ? kFlagTestingMode : 0U) |
                           (m_reboot_pending ? kFlagRebootPending : 0U) | (Writable() ? kFlagWritable : 0U) |
                           (m_text_length > 0 ? kFlagTextReady : 0U));
  status.fsmState = m_env.FlightState();
  status.textLength = static_cast<uint16_t>(m_text_length);
  status.textCrc = m_text_crc;
  status.valueCount = static_cast<uint8_t>(value_table_entry_count);
  status.token = m_token;
  EncodeStatus(status, out);
}

bool Server::NextDownlink(uint8_t out[kFrameSize]) {
  const uint32_t now = m_env.NowMs();
  if (m_reboot_pending && static_cast<int32_t>(now - m_reboot_at_ms) >= 0) {
    m_reboot_pending = false;
    // The request was accepted on the pad; never reset if liftoff happened in the meantime.
    if (!Writable()) {
      m_last_result = Result::kBadState;
      EndSession();
      return false;
    }
    m_env.Reboot(m_reboot_mode);
    // Only reached in host tests.
    EndSession();
    return false;
  }
  if (!m_session_active) {
    return false;
  }
  if (InFlight() || now - m_last_activity_ms > kSessionTimeoutMs) {
    EndSession();
    return false;
  }

  const uint32_t slot = m_cycle % kDownlinkCycle;
  ++m_cycle;
  if (slot == 0 && !m_reboot_pending) {
    // Keep a slow telemetry stream so the Ground Station still sees the flight state.
    return false;
  }
  if (m_reading && m_text_length > 0 && slot != kDownlinkCycle / 2) {
    DataFrame data{};
    data.offset = static_cast<uint16_t>(m_next_offset);
    const size_t chunk =
        m_text_length - m_next_offset < kDataChunkSize ? m_text_length - m_next_offset : kDataChunkSize;
    memcpy(data.data, &m_text[m_next_offset], chunk);
    m_next_offset += chunk;
    if (m_next_offset >= m_text_length) {
      m_next_offset = 0;
    }
    EncodeData(data, out);
    return true;
  }
  EncodeStatusFrame(out);
  return true;
}

}  // namespace remote_config
