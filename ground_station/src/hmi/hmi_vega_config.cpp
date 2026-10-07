/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Settings > Vega > Vega Configuration: read, edit, load/save profiles and write the
/// Vega configuration by radio. All radio and storage work runs in VegaConfig::Session.

#include <cstdio>
#include <cstring>

#include "hmi.hpp"
#include "utils.hpp"

namespace {

constexpr const char* kTitle = "Vega Configuration";

enum MenuItem : uint8_t {
  kMenuEdit = 0,
  kMenuReview,
  kMenuDiscard,
  kMenuLoad,
  kMenuSave,
  kMenuRead,
  kMenuTesting,
  kMenuRestart,
  kMenuCount,
};

const char* flightStateName(uint8_t state, uint8_t flags) {
  if ((flags & remote_config::kFlagTestingMode) != 0) {
    return "TESTING";
  }
  constexpr const char* kNames[] = {"BOOT",     "CALIBRATING", "READY", "THRUSTING",
                                    "COASTING", "DROGUE",      "MAIN",  "TOUCHDOWN"};
  return state < sizeof(kNames) / sizeof(kNames[0]) ? kNames[state] : "?";
}

void sessionTitle(const VegaConfig::Snapshot& snapshot, char* out, size_t capacity) {
  if (snapshot.sessionOpen) {
    snprintf(out, capacity, "Vega on Link %u", snapshot.link);
  } else {
    snprintf(out, capacity, "%s", kTitle);
  }
}

}  // namespace

void Hmi::initVegaConfig() {
  state = VEGA_CONFIG;
  automaticUsbSharePending = false;
  vegaView = VegaView::LinkSelect;
  vegaSelection = 0;
  vegaWindowStart = 0;
  vegaMenuSelection = 0;
  vegaRevision = UINT32_MAX;
  vegaRedraw = true;
  vegaLeaving = false;
  if (vegaSession == nullptr) {
    window.vegaConfigMessage(kTitle, "Unavailable", "The Vega configuration worker could not be started.");
    return;
  }
  if (vegaSession->snapshot().sessionOpen) {
    vegaView = VegaView::Menu;
  }
}

void Hmi::leaveVegaConfig() {
  vegaLeaving = false;
  // Profiles may have been written; share the drive with the PC again.
  automaticUsbSharePending = Utils::isConnected();
  state = SETTINGS;
  settingSubMenu = SETTING_VEGA;
  settingIndex = 0;
  window.initSettings(settingSubMenu);
  window.updateSettings(settingIndex);
}

void Hmi::moveVegaSelection(int direction, size_t count) {
  if (count == 0) {
    vegaSelection = 0;
    vegaWindowStart = 0;
    return;
  }
  if (direction > 0 && vegaSelection + 1 < count) {
    ++vegaSelection;
  } else if (direction < 0 && vegaSelection > 0) {
    --vegaSelection;
  }
  if (vegaSelection >= count) {
    vegaSelection = count - 1;
  }
  if (vegaSelection < vegaWindowStart) {
    vegaWindowStart = vegaSelection;
  }
  if (vegaSelection >= vegaWindowStart + Window::kListRows) {
    vegaWindowStart = vegaSelection - Window::kListRows + 1;
  }
}

void Hmi::drawVegaMenu(const VegaConfig::Snapshot& snapshot) {
  char title[32]{};
  sessionTitle(snapshot, title, sizeof(title));
  char review[32]{};
  snprintf(review, sizeof(review), "%u", static_cast<unsigned>(snapshot.pendingCount));
  const bool loaded = snapshot.configLoaded;
  const Window::ListRow rows[kMenuCount] = {
      {"Edit Settings", loaded ? "" : "read first"},
      {"Review & Write Changes", loaded ? review : "read first"},
      {"Discard Changes", ""},
      {"Load Profile from USB Drive", loaded ? "" : "read first"},
      {"Save Profile to USB Drive", loaded ? "" : "read first"},
      {"Read Again", ""},
      {"Restart in Testing Mode", ""},
      {"Restart Vega", ""},
  };
  char hint[40]{};
  snprintf(hint, sizeof(hint), "%s  %s", flightStateName(snapshot.fsmState, snapshot.flags),
           snapshot.identity.firmware[0] != '\0' ? snapshot.identity.firmware : "");
  window.vegaConfigList(title, rows, kMenuCount, vegaMenuSelection, false, false, "Select (A)", hint);
}

void Hmi::drawVegaEdit() {
  const size_t count = vegaSession->entryCount();
  Window::ListRow rows[Window::kListRows]{};
  VegaConfig::EntryView views[Window::kListRows]{};
  char labels[Window::kListRows][40]{};
  size_t shown = 0;
  for (size_t index = vegaWindowStart; index < count && shown < Window::kListRows; ++index, ++shown) {
    vegaSession->entry(index, views[shown]);
    snprintf(labels[shown], sizeof(labels[shown]), "%s%s", views[shown].changed ? "* " : "", views[shown].label);
    rows[shown] = {labels[shown], views[shown].value};
  }
  VegaConfig::EntryView selected{};
  vegaSession->entry(vegaSelection, selected);
  const char* hint = selected.editable ? "< > change" : (selected.actions ? nullptr : "view only");
  window.vegaConfigList("Edit Settings", rows, shown, vegaSelection - vegaWindowStart, vegaWindowStart > 0,
                        vegaWindowStart + Window::kListRows < count, selected.actions ? "Actions (A)" : "", hint);
}

void Hmi::drawVegaActions() {
  VegaConfig::EntryView entry{};
  vegaSession->entry(vegaActionKey, entry);
  const size_t count = vegaSession->actionCount(vegaActionKey);
  if (count == 0) {
    window.vegaConfigMessage(entry.label, "No actions",
                             "This event has no actions. Actions are added with the "
                             "CATS Configurator or a profile.");
    return;
  }
  Window::ListRow rows[Window::kListRows]{};
  VegaConfig::ActionRowView views[Window::kListRows]{};
  char labels[Window::kListRows][32]{};
  size_t shown = 0;
  for (size_t slot = 0; slot < count && shown < Window::kListRows; ++slot, ++shown) {
    vegaSession->actionRow(vegaActionKey, slot, views[shown]);
    snprintf(labels[shown], sizeof(labels[shown]), "%s%u  %s", views[shown].changed ? "* " : "",
             static_cast<unsigned>(slot + 1), views[shown].name);
    rows[shown] = {labels[shown], views[shown].value};
  }
  const bool servo = vegaSelection < shown && views[vegaSelection].editable;
  window.vegaConfigList(entry.label, rows, shown, vegaSelection, false, false, "",
                        servo ? "< > servo position" : "view only");
}

int Hmi::vegaAdjustInput(int32_t* multiplier) {
  int direction = 0;
  Button* held = nullptr;
  *multiplier = 1;
  if (rightButton.wasPressed()) {
    direction = +1;
  } else if (leftButton.wasPressed()) {
    direction = -1;
  } else if (rightButton.pressedFor(500)) {
    direction = +1;
    held = &rightButton;
  } else if (leftButton.pressedFor(500)) {
    direction = -1;
    held = &leftButton;
  }
  if (direction != 0 && held != nullptr) {
    // Accelerate long holds; repeat at most every 100 ms.
    if (millis() - vegaHoldStarted < 100U) {
      return 0;
    }
    vegaHoldStarted = millis();
    *multiplier = held->pressedFor(5000) ? 100 : (held->pressedFor(2000) ? 10 : 1);
  }
  return direction;
}

void Hmi::drawVegaFiles(const VegaConfig::Snapshot& snapshot) {
  Window::ListRow rows[Window::kListRows]{};
  char names[Window::kListRows][64]{};
  size_t shown = 0;
  for (size_t index = vegaWindowStart; index < snapshot.fileCount && shown < Window::kListRows; ++index, ++shown) {
    vegaSession->fileName(index, names[shown], sizeof(names[shown]));
    rows[shown] = {names[shown], ""};
  }
  window.vegaConfigList("Load Profile", rows, shown, vegaSelection - vegaWindowStart, vegaWindowStart > 0,
                        vegaWindowStart + Window::kListRows < snapshot.fileCount, "Load (A)", "/vega_profiles");
}

void Hmi::drawVegaReview(const VegaConfig::Snapshot& snapshot) {
  const size_t count = vegaSession->rowCount();
  if (count == 0) {
    window.vegaConfigMessage("Review Changes", "No changes", "The profile matches the Vega configuration.");
    return;
  }
  Window::ListRow rows[Window::kListRows]{};
  VegaConfig::RowView views[Window::kListRows]{};
  char rights[Window::kListRows][64]{};
  size_t shown = 0;
  for (size_t index = vegaWindowStart; index < count && shown < Window::kListRows; ++index, ++shown) {
    vegaSession->row(index, views[shown]);
    switch (views[shown].status) {
      case VegaConfig::RowStatus::kChange:
        snprintf(rights[shown], sizeof(rights[shown]), "%s > %s", views[shown].from, views[shown].to);
        break;
      case VegaConfig::RowStatus::kSkipped:
        snprintf(rights[shown], sizeof(rights[shown]), "skipped: radio link");
        break;
      case VegaConfig::RowStatus::kInvalid:
        snprintf(rights[shown], sizeof(rights[shown]), "invalid value");
        break;
      case VegaConfig::RowStatus::kUnsupported:
        snprintf(rights[shown], sizeof(rights[shown]), "not supported");
        break;
    }
    rows[shown] = {views[shown].label, rights[shown]};
  }
  char title[32]{};
  snprintf(title, sizeof(title), "%u Change%s", static_cast<unsigned>(snapshot.pendingCount),
           snapshot.pendingCount == 1 ? "" : "s");
  window.vegaConfigList(title, rows, shown, vegaSelection - vegaWindowStart, vegaWindowStart > 0,
                        vegaWindowStart + Window::kListRows < count, snapshot.pendingCount > 0 ? "Write (A)" : "",
                        nullptr);
}

void Hmi::drawVegaConfig(const VegaConfig::Snapshot& snapshot) {
  char title[32]{};
  sessionTitle(snapshot, title, sizeof(title));
  switch (snapshot.phase) {
    case VegaConfig::Phase::kIdle: {
      const bool dual = systemConfig.config.receiverMode == DUAL;
      const Window::ListRow single[] = {{"Connect", systemConfig.config.linkPhrase1}};
      const Window::ListRow both[] = {{"Link 1", systemConfig.config.linkPhrase1},
                                      {"Link 2", systemConfig.config.linkPhrase2}};
      window.vegaConfigList(kTitle, dual ? both : single, dual ? 2 : 1, vegaSelection, false, false, "Connect (A)",
                            nullptr);
    } break;
    case VegaConfig::Phase::kBusy:
      window.vegaConfigProgress(title, snapshot.activity, snapshot.percent, "Keep the Vega in range. Back (B) cancels");
      break;
    case VegaConfig::Phase::kResult:
      window.vegaConfigMessage(title, snapshot.title, snapshot.message);
      break;
    case VegaConfig::Phase::kBrowsing:
      drawVegaFiles(snapshot);
      break;
    case VegaConfig::Phase::kReview:
    case VegaConfig::Phase::kReady:
      switch (vegaView) {
        case VegaView::Edit:
          drawVegaEdit();
          break;
        case VegaView::Actions:
          drawVegaActions();
          break;
        case VegaView::Review:
          drawVegaReview(snapshot);
          break;
        case VegaView::ConfirmWrite: {
          char question[40]{};
          snprintf(question, sizeof(question), "Write %u change%s?", static_cast<unsigned>(snapshot.pendingCount),
                   snapshot.pendingCount == 1 ? "" : "s");
          window.vegaConfigConfirm(title, question,
                                   "The Vega saves the settings, restarts and is read again to verify them. "
                                   "Keep it powered.",
                                   "Write (A)");
        } break;
        case VegaView::ConfirmTesting:
          window.vegaConfigConfirm(title, "Restart in testing mode?",
                                   "For one boot the Testing screen can trigger events and pyro channels. The "
                                   "next restart or power cycle returns to flight mode.",
                                   "Restart (A)");
          break;
        case VegaView::ConfirmRestart:
          window.vegaConfigConfirm(title, "Restart the Vega?",
                                   "The session closes; changes that were not written are discarded.", "Restart (A)");
          break;
        case VegaView::ConfirmClose: {
          char question[40]{};
          snprintf(question, sizeof(question), "Discard %u change%s?", static_cast<unsigned>(snapshot.pendingCount),
                   snapshot.pendingCount == 1 ? "" : "s");
          window.vegaConfigConfirm(title, question, "The changes were not written to the Vega.", "Discard (A)");
        } break;
        default:
          if (snapshot.phase == VegaConfig::Phase::kReview) {
            drawVegaReview(snapshot);
          } else {
            drawVegaMenu(snapshot);
          }
          break;
      }
      break;
  }
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void Hmi::vegaConfig() {
  if (vegaSession == nullptr) {
    if (backButton.wasPressed()) {
      leaveVegaConfig();
    }
    return;
  }
  const VegaConfig::Snapshot snapshot = vegaSession->snapshot();
  if (snapshot.revision != vegaRevision) {
    vegaRevision = snapshot.revision;
    vegaRedraw = true;
  }

  switch (snapshot.phase) {
    case VegaConfig::Phase::kIdle: {
      if (vegaLeaving) {
        leaveVegaConfig();
        return;
      }
      vegaView = VegaView::LinkSelect;
      const size_t links = systemConfig.config.receiverMode == DUAL ? 2 : 1;
      if (backButton.wasPressed()) {
        leaveVegaConfig();
        return;
      }
      if (downButton.wasPressed() && vegaSelection + 1 < links) {
        ++vegaSelection;
        vegaRedraw = true;
      }
      if (upButton.wasPressed() && vegaSelection > 0) {
        --vegaSelection;
        vegaRedraw = true;
      }
      if (vegaSelection >= links) {
        vegaSelection = 0;
      }
      if (okButton.wasPressed()) {
        vegaMenuSelection = 0;
        (void)vegaSession->open(static_cast<uint8_t>(vegaSelection + 1));
        return;
      }
    } break;

    case VegaConfig::Phase::kBusy:
      // B stops radio operations; file operations are short and finish on their own.
      if (backButton.wasPressed()) {
        vegaSession->cancel();
      }
      break;

    case VegaConfig::Phase::kResult:
      if (okButton.wasPressed() || backButton.wasPressed()) {
        vegaSession->acknowledge();
        vegaView = VegaView::Menu;
        vegaSelection = 0;
        vegaWindowStart = 0;
        vegaRedraw = true;
        return;
      }
      break;

    case VegaConfig::Phase::kBrowsing:
      if (backButton.wasPressed()) {
        vegaSession->acknowledge();
        vegaView = VegaView::Menu;
        return;
      }
      if (downButton.wasPressed()) {
        moveVegaSelection(+1, snapshot.fileCount);
        vegaRedraw = true;
      }
      if (upButton.wasPressed()) {
        moveVegaSelection(-1, snapshot.fileCount);
        vegaRedraw = true;
      }
      if (okButton.wasPressed() && snapshot.fileCount > 0) {
        const size_t file = vegaSelection;
        vegaSelection = 0;
        vegaWindowStart = 0;
        vegaView = VegaView::Review;
        (void)vegaSession->loadProfile(file);
        return;
      }
      break;

    case VegaConfig::Phase::kReview:
    case VegaConfig::Phase::kReady:
      if (snapshot.phase == VegaConfig::Phase::kReview && vegaView != VegaView::ConfirmWrite) {
        vegaView = VegaView::Review;
      }
      switch (vegaView) {
        case VegaView::LinkSelect:
        case VegaView::Files:
          vegaView = VegaView::Menu;
          vegaRedraw = true;
          break;

        case VegaView::Menu:
          if (downButton.wasPressed() && vegaMenuSelection + 1 < kMenuCount) {
            ++vegaMenuSelection;
            vegaRedraw = true;
          }
          if (upButton.wasPressed() && vegaMenuSelection > 0) {
            --vegaMenuSelection;
            vegaRedraw = true;
          }
          if (backButton.wasPressed()) {
            if (snapshot.pendingCount > 0) {
              vegaView = VegaView::ConfirmClose;
              vegaRedraw = true;
            } else {
              vegaLeaving = true;
              (void)vegaSession->close();
            }
            return;
          }
          if (okButton.wasPressed()) {
            const bool loaded = snapshot.configLoaded;
            vegaSelection = 0;
            vegaWindowStart = 0;
            switch (vegaMenuSelection) {
              case kMenuEdit:
                if (loaded) {
                  vegaView = VegaView::Edit;
                }
                break;
              case kMenuReview:
                if (loaded) {
                  vegaView = VegaView::Review;
                }
                break;
              case kMenuDiscard:
                vegaSession->discard();
                break;
              case kMenuLoad:
                if (loaded) {
                  (void)vegaSession->browseProfiles();
                }
                break;
              case kMenuSave:
                if (loaded) {
                  (void)vegaSession->saveProfile();
                }
                break;
              case kMenuRead:
                (void)vegaSession->read();
                break;
              case kMenuTesting:
                vegaView = VegaView::ConfirmTesting;
                break;
              case kMenuRestart:
                vegaView = VegaView::ConfirmRestart;
                break;
              default:
                break;
            }
            vegaRedraw = true;
            return;
          }
          break;

        case VegaView::Edit: {
          const size_t count = vegaSession->entryCount();
          if (backButton.wasPressed()) {
            vegaView = VegaView::Menu;
            vegaRedraw = true;
            return;
          }
          const bool repeat = millis() - vegaHoldStarted >= 100U;
          if (downButton.wasPressed() || (downButton.pressedFor(500) && repeat)) {
            vegaHoldStarted = millis();
            moveVegaSelection(+1, count);
            vegaRedraw = true;
          }
          if (upButton.wasPressed() || (upButton.pressedFor(500) && repeat)) {
            vegaHoldStarted = millis();
            moveVegaSelection(-1, count);
            vegaRedraw = true;
          }
          if (okButton.wasPressed()) {
            VegaConfig::EntryView selected{};
            if (vegaSession->entry(vegaSelection, selected) && selected.actions) {
              vegaActionKey = vegaSelection;
              vegaEditSelection = vegaSelection;
              vegaEditWindowStart = vegaWindowStart;
              vegaSelection = 0;
              vegaWindowStart = 0;
              vegaView = VegaView::Actions;
              vegaRedraw = true;
              return;
            }
          }
          int32_t multiplier = 1;
          const int direction = vegaAdjustInput(&multiplier);
          if (direction != 0 && vegaSession->adjust(vegaSelection, direction, multiplier)) {
            vegaRedraw = true;
          }
        } break;

        case VegaView::Actions: {
          const size_t count = vegaSession->actionCount(vegaActionKey);
          if (backButton.wasPressed()) {
            vegaSelection = vegaEditSelection;
            vegaWindowStart = vegaEditWindowStart;
            vegaView = VegaView::Edit;
            vegaRedraw = true;
            return;
          }
          if (downButton.wasPressed()) {
            moveVegaSelection(+1, count);
            vegaRedraw = true;
          }
          if (upButton.wasPressed()) {
            moveVegaSelection(-1, count);
            vegaRedraw = true;
          }
          int32_t multiplier = 1;
          const int direction = vegaAdjustInput(&multiplier);
          if (direction != 0 && vegaSession->adjustAction(vegaActionKey, vegaSelection, direction, multiplier)) {
            vegaRedraw = true;
          }
        } break;

        case VegaView::Review: {
          const size_t count = vegaSession->rowCount();
          if (backButton.wasPressed()) {
            if (snapshot.phase == VegaConfig::Phase::kReview) {
              vegaSession->acknowledge();
            }
            vegaView = VegaView::Menu;
            vegaRedraw = true;
            return;
          }
          if (downButton.wasPressed()) {
            moveVegaSelection(+1, count);
            vegaRedraw = true;
          }
          if (upButton.wasPressed()) {
            moveVegaSelection(-1, count);
            vegaRedraw = true;
          }
          if (okButton.wasPressed() && snapshot.pendingCount > 0) {
            vegaView = VegaView::ConfirmWrite;
            vegaRedraw = true;
            return;
          }
        } break;

        case VegaView::ConfirmWrite:
          if (okButton.wasPressed()) {
            if (snapshot.phase == VegaConfig::Phase::kReview) {
              vegaSession->acknowledge();
            }
            vegaView = VegaView::Menu;
            (void)vegaSession->write();
            return;
          }
          if (backButton.wasPressed()) {
            vegaView = VegaView::Review;
            vegaRedraw = true;
            return;
          }
          break;

        case VegaView::ConfirmTesting:
        case VegaView::ConfirmRestart:
          if (okButton.wasPressed()) {
            const auto mode = vegaView == VegaView::ConfirmTesting ? remote_config::RebootMode::kTesting
                                                                   : remote_config::RebootMode::kNormal;
            vegaView = VegaView::Menu;
            (void)vegaSession->reboot(mode);
            return;
          }
          if (backButton.wasPressed()) {
            vegaView = VegaView::Menu;
            vegaRedraw = true;
            return;
          }
          break;

        case VegaView::ConfirmClose:
          if (okButton.wasPressed()) {
            vegaSession->discard();
            vegaLeaving = true;
            (void)vegaSession->close();
            return;
          }
          if (backButton.wasPressed()) {
            vegaView = VegaView::Menu;
            vegaRedraw = true;
            return;
          }
          break;
      }
      break;
  }

  if (vegaRedraw) {
    // Progress changes continuously; limit full LCD transfers while busy.
    const uint32_t now = millis();
    if (snapshot.phase == VegaConfig::Phase::kBusy && now - vegaLastDraw < 400U) {
      return;
    }
    vegaLastDraw = now;
    vegaRedraw = false;
    drawVegaConfig(vegaSession->snapshot());
  }
}
