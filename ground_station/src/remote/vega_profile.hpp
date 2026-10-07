/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Vega configuration model used by the Ground Station remote configuration.
///
/// Values are kept in the canonical text form printed by the Vega CLI ("200",
/// "ON", "APOGEE", "1,500,7,2,0,..."). Profiles use the CATS Configurator
/// profile format (format "cats-configurator-profile", schema 1) so files can be
/// exchanged between the Configurator and the Ground Station in both directions.
///
/// This file has no Arduino dependencies and is covered by host tests.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace VegaConfig {

enum class Kind : uint8_t {
  kNumber,   ///< Integer with limits, written to JSON as a number.
  kOnOff,    ///< "ON" / "OFF".
  kEvent,    ///< Vega event name (timer start/trigger).
  kActions,  ///< Event action list "action,argument,..." (16 integers).
  kPhrase,   ///< Link or testing phrase, "-" when empty.
};

enum class Section : uint8_t { kConfiguration, kEvents, kTimers };

struct KeyInfo {
  const char* key;
  const char* label;
  Section section;
  Kind kind;
  int32_t min;
  int32_t max;
  int32_t step;
  /// False for settings that would cut the radio link if changed over the radio.
  bool remoteWritable;
};

/// Canonical value buffer: 16 int16 values with commas fit into 111 characters.
inline constexpr size_t kValueSize = 112;
inline constexpr size_t kActionCount = 16;
inline constexpr size_t kIdentitySize = 24;

const KeyInfo* keyTable();
size_t keyCount();
int findKey(const char* key, size_t length);

/// Converts a value into the canonical form of the given kind. Returns false if invalid.
bool canonicalize(const KeyInfo& info, const char* value, char* out, size_t capacity);

/// Short human readable form for the 400 px display.
void displayValue(const KeyInfo& info, const char* canonical, char* out, size_t capacity);

/// One entry of an event action list (Vega action_function_e and its argument).
struct ActionItem {
  int16_t action{0};
  int16_t argument{0};
};
inline constexpr size_t kActionSlots = kActionCount / 2;
inline constexpr int16_t kServoMin = 0;
inline constexpr int16_t kServoMax = 1000;

/// Decodes an action list up to the first NONE action. Returns the number of actions.
size_t decodeActions(const char* canonical, ActionItem* items, size_t capacity);
/// Display name of a Vega action ("Pyro 1", "Servo 2", "Delay", ...).
const char* actionName(int16_t action);
/// Display form of an action argument ("ON", "1000 ms", "160", ...).
void actionArgument(const ActionItem& item, char* out, size_t capacity);
/// Servo actions are the only actions whose argument can be edited on the Ground Station.
bool isServoAction(int16_t action);
/// Short review text for a changed action list: one changed servo shows "Servo 1 160" > "300".
void describeActionChange(const char* from, const char* to, char* outFrom, size_t fromCapacity, char* outTo,
                          size_t toCapacity);

struct Identity {
  char board[kIdentitySize]{};
  char firmware[kIdentitySize]{};
  char telemetry[kIdentitySize]{};
};

enum class RowStatus : uint8_t {
  kChange,       ///< Will be written.
  kSkipped,      ///< Changed, but not written over the radio (link settings).
  kInvalid,      ///< Profile value rejected by the Ground Station.
  kUnsupported,  ///< Profile key unknown to this Ground Station or not reported by the Vega.
};

struct DiffRow {
  RowStatus status{RowStatus::kChange};
  int16_t key{-1};  ///< Index into keyTable(), -1 for unknown keys.
  char name[32]{};  ///< Profile key (also for unknown keys).
};

class Model {
 public:
  Model();

  /// Parses the configuration text received from the Vega. Resets pending changes.
  bool loadText(const char* text, size_t length);

  [[nodiscard]] bool loaded() const { return isLoaded; }
  [[nodiscard]] const Identity& identity() const { return vegaIdentity; }
  [[nodiscard]] bool known(size_t index) const;
  [[nodiscard]] const char* current(size_t index) const;
  [[nodiscard]] const char* pending(size_t index) const;
  [[nodiscard]] bool changed(size_t index) const;
  [[nodiscard]] bool editable(size_t index) const;
  /// Number of changes that will be written.
  [[nodiscard]] size_t pendingCount() const;

  /// Steps a value (numbers by step * multiplier, cycles lookups). Returns true if changed.
  bool adjust(size_t index, int direction, int32_t multiplier = 1);
  /// Pending actions of an event entry; `before` is the Vega value of the same slot.
  [[nodiscard]] size_t actionCount(size_t index) const;
  bool action(size_t index, size_t slot, ActionItem& now, ActionItem& before) const;
  /// Steps a servo position in an event action list. Returns true if changed.
  bool adjustAction(size_t index, size_t slot, int direction, int32_t multiplier = 1);
  /// Sets a pending value from any accepted representation.
  bool setPending(size_t index, const char* value);
  void discard();

  /// Applies a Configurator profile as pending changes.
  bool importProfile(const char* json, size_t length, char* error, size_t errorCapacity);
  /// Writes the current Vega values (not pending changes) as a Configurator profile.
  size_t exportProfile(char* out, size_t capacity, const char* createdAt, const char* creator) const;

  /// Rows for the review screen: changes, skipped changes, and profile notes.
  [[nodiscard]] std::vector<DiffRow> diffRows() const;
  /// "key=value" lines for every pending change.
  [[nodiscard]] std::vector<std::string> writeLines() const;
  /// Checks that every pending change is present in a freshly read model.
  bool verify(const Model& readBack, char* failedKey, size_t capacity) const;
  /// Marks pending values as current (after a verified write).
  void acceptPending();

 private:
  struct Value {
    bool known{false};
    char current[kValueSize]{};
    char pending[kValueSize]{};
  };
  bool isLoaded{false};
  Identity vegaIdentity{};
  std::vector<Value> values;
  std::vector<DiffRow> notes;
};

}  // namespace VegaConfig
