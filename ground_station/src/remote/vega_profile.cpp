/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "remote/vega_profile.hpp"

#include <ArduinoJson.h>

#include <strings.h>
#include <cctype>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace VegaConfig {

namespace {

constexpr const char* kProfileFormat = "cats-configurator-profile";
constexpr int kProfileSchemaVersion = 1;

// Same order as the CATS Configurator profile (CONFIG_KEYS, EVENT_KEYS, TIMER_KEYS x TIMER_FIELDS).
// Limits mirror flight_computer/src/cli/settings.cpp; the Vega validates every value again.
constexpr KeyInfo kKeys[] = {
    {"main_altitude", "Main Altitude [m]", Section::kConfiguration, Kind::kNumber, 10, 65535, 10, true},
    {"acc_threshold", "Liftoff Acc. [m/s2]", Section::kConfiguration, Kind::kNumber, 30, 80, 1, true},
    {"servo1_init_pos", "Servo 1 Init [permil]", Section::kConfiguration, Kind::kNumber, 0, 1000, 10, true},
    {"servo2_init_pos", "Servo 2 Init [permil]", Section::kConfiguration, Kind::kNumber, 0, 1000, 10, true},
    {"tele_enable", "Telemetry", Section::kConfiguration, Kind::kOnOff, 0, 1, 1, false},
    {"tele_link_phrase", "Link Phrase", Section::kConfiguration, Kind::kPhrase, 4, 16, 0, false},
    {"tele_power_level", "Telemetry Power [dBm]", Section::kConfiguration, Kind::kNumber, 16, 30, 1, true},
    {"tele_adaptive_power", "Adaptive Power", Section::kConfiguration, Kind::kOnOff, 0, 1, 1, true},
    {"test_mode", "Testing Mode (always)", Section::kConfiguration, Kind::kOnOff, 0, 1, 1, false},
    {"tele_test_phrase", "Test Phrase", Section::kConfiguration, Kind::kPhrase, 4, 16, 0, false},
    {"ev_liftoff", "Liftoff Actions", Section::kEvents, Kind::kActions, 0, 0, 0, true},
    {"ev_burnout", "Burnout Actions", Section::kEvents, Kind::kActions, 0, 0, 0, true},
    {"ev_apogee", "Apogee Actions", Section::kEvents, Kind::kActions, 0, 0, 0, true},
    {"ev_main_deployment", "Main Actions", Section::kEvents, Kind::kActions, 0, 0, 0, true},
    {"ev_touchdown", "Touchdown Actions", Section::kEvents, Kind::kActions, 0, 0, 0, true},
    {"ev_custom1", "Custom 1 Actions", Section::kEvents, Kind::kActions, 0, 0, 0, true},
    {"ev_custom2", "Custom 2 Actions", Section::kEvents, Kind::kActions, 0, 0, 0, true},
    {"timer1_start", "Timer 1 Start", Section::kTimers, Kind::kEvent, 0, 0, 0, true},
    {"timer1_duration", "Timer 1 Duration [ms]", Section::kTimers, Kind::kNumber, 0, 1200000, 100, true},
    {"timer1_trigger", "Timer 1 Trigger", Section::kTimers, Kind::kEvent, 0, 0, 0, true},
    {"timer2_start", "Timer 2 Start", Section::kTimers, Kind::kEvent, 0, 0, 0, true},
    {"timer2_duration", "Timer 2 Duration [ms]", Section::kTimers, Kind::kNumber, 0, 1200000, 100, true},
    {"timer2_trigger", "Timer 2 Trigger", Section::kTimers, Kind::kEvent, 0, 0, 0, true},
    {"timer3_start", "Timer 3 Start", Section::kTimers, Kind::kEvent, 0, 0, 0, true},
    {"timer3_duration", "Timer 3 Duration [ms]", Section::kTimers, Kind::kNumber, 0, 1200000, 100, true},
    {"timer3_trigger", "Timer 3 Trigger", Section::kTimers, Kind::kEvent, 0, 0, 0, true},
    {"timer4_start", "Timer 4 Start", Section::kTimers, Kind::kEvent, 0, 0, 0, true},
    {"timer4_duration", "Timer 4 Duration [ms]", Section::kTimers, Kind::kNumber, 0, 1200000, 100, true},
    {"timer4_trigger", "Timer 4 Trigger", Section::kTimers, Kind::kEvent, 0, 0, 0, true},
};

constexpr const char* kTimerNames[] = {"timer1", "timer2", "timer3", "timer4"};

// flight_computer/src/util/enum_str_maps.hpp event_map
constexpr const char* kEvents[] = {"CALIBRATE",       "READY",     "LIFTOFF",  "MAX_V",   "APOGEE",
                                   "MAIN_DEPLOYMENT", "TOUCHDOWN", "CUSTOM_1", "CUSTOM_2"};
constexpr size_t kEventCount = sizeof(kEvents) / sizeof(kEvents[0]);

void copyText(char* out, size_t capacity, const char* text, size_t length) {
  if (capacity == 0) {
    return;
  }
  const size_t count = length < capacity - 1 ? length : capacity - 1;
  memcpy(out, text, count);
  out[count] = '\0';
}

void copyText(char* out, size_t capacity, const char* text) { copyText(out, capacity, text, strlen(text)); }

void trim(const char** text, size_t* length) {
  while (*length > 0 && isspace(static_cast<unsigned char>(**text)) != 0) {
    ++*text;
    --*length;
  }
  while (*length > 0 && isspace(static_cast<unsigned char>((*text)[*length - 1])) != 0) {
    --*length;
  }
}

bool parseInteger(const char* text, size_t length, int64_t* value) {
  trim(&text, &length);
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
      // Accept a Configurator float with an all-zero fraction, e.g. "200.0".
      if (text[index] == '.') {
        for (size_t fraction = index + 1; fraction < length; ++fraction) {
          if (text[fraction] != '0') {
            return false;
          }
        }
        break;
      }
      return false;
    }
    result = result * 10 + (text[index] - '0');
  }
  *value = negative ? -result : result;
  return true;
}

size_t countActions(const char* canonical) {
  // Pairs of (action, argument); the Vega stops at the first NONE (0) action.
  int64_t numbers[kActionCount]{};
  size_t parsed = 0;
  const char* cursor = canonical;
  while (parsed < kActionCount && *cursor != '\0') {
    char* end = nullptr;
    numbers[parsed++] = strtoll(cursor, &end, 10);
    if (end == cursor || *end != ',') {
      break;
    }
    cursor = end + 1;
  }
  size_t count = 0;
  for (size_t pair = 0; pair * 2 < parsed; ++pair) {
    if (numbers[pair * 2] == 0) {
      break;
    }
    ++count;
  }
  return count;
}

}  // namespace

const KeyInfo* keyTable() { return kKeys; }

size_t keyCount() { return sizeof(kKeys) / sizeof(kKeys[0]); }

int findKey(const char* key, size_t length) {
  for (size_t index = 0; index < keyCount(); ++index) {
    if (strlen(kKeys[index].key) == length && strncmp(kKeys[index].key, key, length) == 0) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool canonicalize(const KeyInfo& info, const char* value, char* out, size_t capacity) {
  const char* text = value;
  size_t length = strlen(value);
  trim(&text, &length);
  switch (info.kind) {
    case Kind::kNumber: {
      int64_t number = 0;
      if (!parseInteger(text, length, &number) || number < info.min || number > info.max) {
        return false;
      }
      snprintf(out, capacity, "%" PRId64, number);
      return true;
    }
    case Kind::kOnOff:
      if (length == 2 && strncasecmp(text, "ON", 2) == 0) {
        copyText(out, capacity, "ON");
        return true;
      }
      if (length == 3 && strncasecmp(text, "OFF", 3) == 0) {
        copyText(out, capacity, "OFF");
        return true;
      }
      return false;
    case Kind::kEvent:
      for (const char* event : kEvents) {
        if (strlen(event) == length && strncasecmp(event, text, length) == 0) {
          copyText(out, capacity, event);
          return true;
        }
      }
      return false;
    case Kind::kActions: {
      int64_t numbers[kActionCount]{};
      size_t count = 0;
      const char* cursor = text;
      size_t remaining = length;
      while (remaining > 0) {
        const auto* comma = static_cast<const char*>(memchr(cursor, ',', remaining));
        const size_t itemLength = comma != nullptr ? static_cast<size_t>(comma - cursor) : remaining;
        if (count >= kActionCount || !parseInteger(cursor, itemLength, &numbers[count]) || numbers[count] < INT16_MIN ||
            numbers[count] > INT16_MAX) {
          return false;
        }
        ++count;
        if (comma == nullptr) {
          break;
        }
        remaining -= itemLength + 1;
        cursor = comma + 1;
      }
      if (count == 0) {
        return false;
      }
      size_t used = 0;
      for (size_t index = 0; index < kActionCount; ++index) {
        const int written =
            snprintf(out + used, capacity - used, index == 0 ? "%" PRId64 : ",%" PRId64, numbers[index]);
        if (written < 0 || static_cast<size_t>(written) >= capacity - used) {
          return false;
        }
        used += static_cast<size_t>(written);
      }
      return true;
    }
    case Kind::kPhrase:
      if (length == 0 || (length == 1 && text[0] == '-')) {
        copyText(out, capacity, "-");
        return true;
      }
      if (length < static_cast<size_t>(info.min) || length > static_cast<size_t>(info.max)) {
        return false;
      }
      for (size_t index = 0; index < length; ++index) {
        if (text[index] < 33 || text[index] > 126) {
          return false;
        }
      }
      copyText(out, capacity, text, length);
      return true;
  }
  return false;
}

void displayValue(const KeyInfo& info, const char* canonical, char* out, size_t capacity) {
  if (canonical == nullptr || canonical[0] == '\0') {
    copyText(out, capacity, "?");
    return;
  }
  if (info.kind == Kind::kActions) {
    const size_t count = countActions(canonical);
    if (count == 0) {
      copyText(out, capacity, "none");
    } else {
      snprintf(out, capacity, "%u action%s", static_cast<unsigned>(count), count == 1 ? "" : "s");
    }
    return;
  }
  copyText(out, capacity, canonical);
}

Model::Model() : values(keyCount()) {}

bool Model::loadText(const char* text, size_t length) {
  Model parsed{};
  const char* cursor = text;
  const char* end = text + length;
  size_t knownCount = 0;
  while (cursor < end) {
    const auto* newline = static_cast<const char*>(memchr(cursor, '\n', static_cast<size_t>(end - cursor)));
    const char* lineEnd = newline != nullptr ? newline : end;
    const auto* equals = static_cast<const char*>(memchr(cursor, '=', static_cast<size_t>(lineEnd - cursor)));
    if (equals != nullptr) {
      const char* key = cursor;
      auto keyLength = static_cast<size_t>(equals - cursor);
      const char* value = equals + 1;
      auto valueLength = static_cast<size_t>(lineEnd - value);
      trim(&key, &keyLength);
      trim(&value, &valueLength);
      char valueText[kValueSize]{};
      copyText(valueText, sizeof(valueText), value, valueLength);
      if (keyLength > 0 && key[0] == '#') {
        if (keyLength == 6 && strncmp(key, "#board", 6) == 0) {
          copyText(parsed.vegaIdentity.board, kIdentitySize, valueText);
        } else if (keyLength == 3 && strncmp(key, "#fw", 3) == 0) {
          copyText(parsed.vegaIdentity.firmware, kIdentitySize, valueText);
        } else if (keyLength == 5 && strncmp(key, "#tele", 5) == 0) {
          copyText(parsed.vegaIdentity.telemetry, kIdentitySize, valueText);
        }
      } else {
        const int index = findKey(key, keyLength);
        if (index >= 0) {
          Value& slot = parsed.values[static_cast<size_t>(index)];
          if (canonicalize(kKeys[index], valueText, slot.current, kValueSize)) {
            if (!slot.known) {
              ++knownCount;
            }
            slot.known = true;
            copyText(slot.pending, kValueSize, slot.current);
          }
        }
      }
    }
    cursor = lineEnd + 1;
  }
  if (knownCount == 0) {
    return false;
  }
  parsed.isLoaded = true;
  *this = parsed;
  return true;
}

bool Model::known(size_t index) const { return index < values.size() && values[index].known; }

const char* Model::current(size_t index) const { return index < values.size() ? values[index].current : ""; }

const char* Model::pending(size_t index) const { return index < values.size() ? values[index].pending : ""; }

bool Model::changed(size_t index) const {
  return known(index) && strcmp(values[index].current, values[index].pending) != 0;
}

bool Model::editable(size_t index) const {
  if (!known(index) || !kKeys[index].remoteWritable) {
    return false;
  }
  const Kind kind = kKeys[index].kind;
  return kind == Kind::kNumber || kind == Kind::kOnOff || kind == Kind::kEvent;
}

size_t Model::pendingCount() const {
  size_t count = 0;
  for (size_t index = 0; index < values.size(); ++index) {
    if (changed(index) && kKeys[index].remoteWritable) {
      ++count;
    }
  }
  return count;
}

bool Model::adjust(size_t index, int direction, int32_t multiplier) {
  if (!editable(index) || direction == 0) {
    return false;
  }
  const KeyInfo& info = kKeys[index];
  Value& value = values[index];
  char next[kValueSize]{};
  switch (info.kind) {
    case Kind::kNumber: {
      const int64_t now = strtoll(value.pending, nullptr, 10);
      int64_t target =
          now + static_cast<int64_t>(direction > 0 ? 1 : -1) * info.step * (multiplier < 1 ? 1 : multiplier);
      if (target < info.min) {
        target = info.min;
      }
      if (target > info.max) {
        target = info.max;
      }
      if (target == now) {
        return false;
      }
      snprintf(next, sizeof(next), "%" PRId64, target);
    } break;
    case Kind::kOnOff:
      copyText(next, sizeof(next), strcmp(value.pending, "ON") == 0 ? "OFF" : "ON");
      break;
    case Kind::kEvent: {
      size_t position = 0;
      for (size_t event = 0; event < kEventCount; ++event) {
        if (strcmp(kEvents[event], value.pending) == 0) {
          position = event;
        }
      }
      position = (position + (direction > 0 ? 1 : kEventCount - 1)) % kEventCount;
      copyText(next, sizeof(next), kEvents[position]);
    } break;
    default:
      return false;
  }
  copyText(value.pending, kValueSize, next);
  return true;
}

bool Model::setPending(size_t index, const char* value) {
  if (!known(index)) {
    return false;
  }
  char canonical[kValueSize]{};
  if (!canonicalize(kKeys[index], value, canonical, sizeof(canonical))) {
    return false;
  }
  copyText(values[index].pending, kValueSize, canonical);
  return true;
}

void Model::discard() {
  for (auto& value : values) {
    copyText(value.pending, kValueSize, value.current);
  }
  notes.clear();
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool Model::importProfile(const char* json, size_t length, char* error, size_t errorCapacity) {
  if (!isLoaded) {
    copyText(error, errorCapacity, "Read the Vega first");
    return false;
  }
  JsonDocument document;
  if (deserializeJson(document, json, length) != DeserializationError::Ok || !document.is<JsonObject>()) {
    copyText(error, errorCapacity, "File is not valid JSON");
    return false;
  }
  const char* format = document["format"] | "";
  if (strcmp(format, kProfileFormat) != 0) {
    copyText(error, errorCapacity, "Not a CATS Configurator profile");
    return false;
  }
  if (!document["schemaVersion"].is<int>() || document["schemaVersion"].as<int>() != kProfileSchemaVersion) {
    copyText(error, errorCapacity, "Unsupported profile schema");
    return false;
  }
  const char* board = document["source"]["boardModel"] | "";
  if (board[0] != '\0' && vegaIdentity.board[0] != '\0' && strcmp(board, vegaIdentity.board) != 0) {
    snprintf(error, errorCapacity, "Profile is for %s", board);
    return false;
  }

  struct Item {
    std::string key;
    JsonVariantConst value;
  };
  std::vector<Item> items;
  for (const char* section : {"configuration", "events"}) {
    const auto object = document[section].as<JsonObjectConst>();
    if (document[section].isNull()) {
      continue;
    }
    if (object.isNull()) {
      snprintf(error, errorCapacity, "Profile %s must be an object", section);
      return false;
    }
    for (const JsonPairConst pair : object) {
      items.push_back({pair.key().c_str(), pair.value()});
    }
  }
  if (!document["timers"].isNull()) {
    const auto timers = document["timers"].as<JsonObjectConst>();
    if (timers.isNull()) {
      copyText(error, errorCapacity, "Profile timers must be an object");
      return false;
    }
    for (const JsonPairConst timer : timers) {
      const auto fields = timer.value().as<JsonObjectConst>();
      if (fields.isNull()) {
        copyText(error, errorCapacity, "Profile timer must be an object");
        return false;
      }
      for (const JsonPairConst field : fields) {
        items.push_back({std::string(timer.key().c_str()) + "_" + field.key().c_str(), field.value()});
      }
    }
  }
  for (size_t first = 0; first < items.size(); ++first) {
    for (size_t second = first + 1; second < items.size(); ++second) {
      if (items[first].key == items[second].key) {
        snprintf(error, errorCapacity, "Duplicate field %s", items[first].key.c_str());
        return false;
      }
    }
  }

  // Validation passed; apply to a copy so a failure leaves the model untouched.
  std::vector<Value> updated = values;
  std::vector<DiffRow> newNotes;
  for (auto& value : updated) {
    copyText(value.pending, kValueSize, value.current);
  }
  for (const auto& item : items) {
    DiffRow note{};
    copyText(note.name, sizeof(note.name), item.key.c_str());
    const int index = findKey(item.key.c_str(), item.key.size());
    note.key = static_cast<int16_t>(index);
    if (index < 0 || !updated[static_cast<size_t>(index)].known) {
      note.status = RowStatus::kUnsupported;
      newNotes.push_back(note);
      continue;
    }
    char text[kValueSize]{};
    if (item.value.is<bool>()) {
      copyText(text, sizeof(text), item.value.as<bool>() ? "ON" : "OFF");
    } else if (item.value.is<int64_t>()) {
      snprintf(text, sizeof(text), "%" PRId64, item.value.as<int64_t>());
    } else if (item.value.is<double>()) {
      const auto number = item.value.as<double>();
      if (!std::isfinite(number) || std::fabs(number) > 1e10) {
        note.status = RowStatus::kInvalid;
        newNotes.push_back(note);
        continue;
      }
      snprintf(text, sizeof(text), "%.3f", number);
    } else if (item.value.is<const char*>()) {
      const auto* value = item.value.as<const char*>();
      if (strlen(value) >= sizeof(text) || strpbrk(value, "\r\n") != nullptr) {
        note.status = RowStatus::kInvalid;
        newNotes.push_back(note);
        continue;
      }
      copyText(text, sizeof(text), value);
    } else {
      note.status = RowStatus::kInvalid;
      newNotes.push_back(note);
      continue;
    }
    char canonical[kValueSize]{};
    if (!canonicalize(kKeys[index], text, canonical, sizeof(canonical))) {
      note.status = RowStatus::kInvalid;
      newNotes.push_back(note);
      continue;
    }
    copyText(updated[static_cast<size_t>(index)].pending, kValueSize, canonical);
  }
  values = updated;
  notes = newNotes;
  return true;
}

size_t Model::exportProfile(char* out, size_t capacity, const char* createdAt, const char* creator) const {
  JsonDocument document;
  document["format"] = kProfileFormat;
  document["schemaVersion"] = kProfileSchemaVersion;
  document["createdAt"] = createdAt;
  JsonObject source = document["source"].to<JsonObject>();
  auto setIdentity = [&source](const char* key, const char* value) {
    if (value[0] == '\0') {
      source[key] = nullptr;
    } else {
      source[key] = value;
    }
  };
  setIdentity("boardModel", vegaIdentity.board);
  setIdentity("firmwareVersion", vegaIdentity.firmware);
  setIdentity("telemetryFirmwareVersion", vegaIdentity.telemetry);
  source["configuratorVersion"] = creator;
  const JsonObject configuration = document["configuration"].to<JsonObject>();
  const JsonObject events = document["events"].to<JsonObject>();
  const JsonObject timers = document["timers"].to<JsonObject>();
  for (const char* timer : kTimerNames) {
    timers[timer].to<JsonObject>();
  }
  document["logging"].to<JsonObject>();

  for (size_t index = 0; index < keyCount(); ++index) {
    if (!values[index].known) {
      continue;
    }
    const KeyInfo& info = kKeys[index];
    const char* text = values[index].current;
    auto assign = [&info, text](auto&& slot) {
      if (info.kind == Kind::kNumber) {
        slot = static_cast<int64_t>(strtoll(text, nullptr, 10));
      } else {
        slot = static_cast<const char*>(text);
      }
    };
    if (info.section == Section::kConfiguration) {
      assign(configuration[info.key]);
    } else if (info.section == Section::kEvents) {
      assign(events[info.key]);
    } else {
      // "timerN_field"
      const char* separator = strchr(info.key, '_');
      const std::string timer(info.key, static_cast<size_t>(separator - info.key));
      assign(timers[timer][separator + 1]);
    }
  }
  const size_t written = serializeJsonPretty(document, out, capacity);
  return written >= capacity ? 0 : written;
}

std::vector<DiffRow> Model::diffRows() const {
  std::vector<DiffRow> rows;
  for (size_t index = 0; index < values.size(); ++index) {
    if (!changed(index)) {
      continue;
    }
    DiffRow row{};
    row.key = static_cast<int16_t>(index);
    row.status = kKeys[index].remoteWritable ? RowStatus::kChange : RowStatus::kSkipped;
    copyText(row.name, sizeof(row.name), kKeys[index].key);
    rows.push_back(row);
  }
  rows.insert(rows.end(), notes.begin(), notes.end());
  return rows;
}

std::vector<std::string> Model::writeLines() const {
  std::vector<std::string> lines;
  for (size_t index = 0; index < values.size(); ++index) {
    if (changed(index) && kKeys[index].remoteWritable) {
      lines.push_back(std::string(kKeys[index].key) + "=" + values[index].pending);
    }
  }
  return lines;
}

bool Model::verify(const Model& readBack, char* failedKey, size_t capacity) const {
  for (size_t index = 0; index < values.size(); ++index) {
    if (!changed(index) || !kKeys[index].remoteWritable) {
      continue;
    }
    if (!readBack.known(index) || strcmp(readBack.current(index), values[index].pending) != 0) {
      copyText(failedKey, capacity, kKeys[index].key);
      return false;
    }
  }
  return true;
}

void Model::acceptPending() {
  for (size_t index = 0; index < values.size(); ++index) {
    if (kKeys[index].remoteWritable) {
      copyText(values[index].current, kValueSize, values[index].pending);
    } else {
      copyText(values[index].pending, kValueSize, values[index].current);
    }
  }
  notes.clear();
}

}  // namespace VegaConfig
