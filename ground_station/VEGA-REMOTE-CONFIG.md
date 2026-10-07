# Vega remote configuration

The Ground Station can read, edit and write the configuration of a CATS Vega over the radio link, exchange
configuration profiles with the CATS Configurator, and restart the Vega into testing mode for one boot.

Both firmwares must contain this feature: Ground Station `1.3.1+gscfg.2` and Vega `3.0.2+gscfg.1` or newer.
The telemetry (radio) firmware is not changed; receivers and Vega radios on 1.1.3 or 1.2.0 both work.

## Requirements

- The **Test Phrase** must be set and identical on the Vega (Configurator, *Testing Phrase*) and on the Ground Station
  (*Settings > Telemetry > Test Phrase*). It authenticates every request, exactly like testing mode.
- The Vega must be on the ground: **before liftoff** (boot, calibrating, ready) or in testing mode. Sessions are refused,
  and an open session ends immediately, once the Vega reports liftoff.
- Writing and restarting are additionally refused while the Vega records a flight log.

## Using it

*Settings > Vega > Vega Configuration*

1. Select the link (Dual mode) and press **Connect**. The Ground Station connects and reads the full configuration
   (about 10-15 s). During a session one in ten packets is still normal telemetry, so the Vega state stays visible.
2. Menu:
   - **Edit Settings** - numbers (Left/Right, hold to repeat and accelerate), ON/OFF values and timer events.
     Event action lists and phrases are shown but are edited with the Configurator or a profile.
   - **Review & Write Changes** - lists every change (`old > new`). *Write* saves the settings on the Vega, the Vega
     restarts, and the Ground Station reads the configuration again and verifies every written value.
   - **Discard Changes**
   - **Load Profile from USB Drive** - lists `*.json` files in `/vega_profiles` on the Ground Station drive. Profiles
     exported by the CATS Configurator can be copied there. Loading shows the review screen; nothing is written until
     you confirm.
   - **Save Profile to USB Drive** - writes the configuration read from the Vega to `/vega_profiles/vega_NNN.json`
     in the Configurator profile format, so it can be imported by the Configurator.
   - **Read Again**
   - **Restart in Testing Mode** - the Vega restarts in testing mode for **one boot only**. Any later restart or power
     cycle returns to flight mode. Then open *Testing* on the main menu.
   - **Restart Vega** - restarts the Vega in flight mode.
3. **Back** closes the session (pending changes are discarded after a confirmation). **Back** during a radio operation
   cancels it.

### Testing screen

- If the Vega is not in testing mode, *Testing* offers **Restart (A)** to restart it in testing mode (one boot).
- When leaving *Testing* while the Vega is in testing mode, the Ground Station disarms it and offers to restart it in
  flight mode. *Leave as is* keeps it in testing mode until its next restart.

### Settings that are never written over the radio

`tele_link_phrase`, `tele_test_phrase` and `tele_enable` would cut the radio link that carries the request, and a
permanent `test_mode = ON` would make the Vega skip its flight logic at every boot. These keys are shown as
*skipped* in the review and must be changed with the Configurator. Use *Restart in Testing Mode* instead of the
permanent flag.

## Profiles

Profiles use the CATS Configurator format (`"format": "cats-configurator-profile"`, `"schemaVersion": 1`) with the
`configuration`, `events`, `timers` and `logging` sections. Numbers may be integers or Configurator floats (`200.0`).
Keys the Ground Station does not know or that the Vega does not report are listed as *not supported*; invalid values are
listed as *invalid value*; neither is written. Profiles for another board model are rejected. The Ground Station does
not know the date, so `createdAt` is `1970-01-01T00:00:00.000Z` in files it writes.

## Protocol

The protocol lives in `src/remote/remote_config_protocol.hpp`, which must stay byte-identical to
`flight_computer/src/comm/remote_config_protocol.hpp` (checked by the host test).

- **Transport.** Every radio packet carries 15 payload bytes. While a session is open the Ground Station receiver runs
  in bidirectional mode and answers each Vega packet with the last payload written to it. The Vega radio already
  forwards every received payload to the Vega MCU, so the radio firmware is unchanged.
- **Uplink** (`0x73`, testing uses `0x72`): `header, passcode[4] = CRC32(test phrase), seq, op, args[8]`.
  Operations: `NOP`, `READ`, `BEGIN`, `LINE_DATA(offset, length, 6 bytes)`, `LINE_APPLY(token, length, CRC32)`,
  `COMMIT(token, "SAVE")`, `ABORT`, `REBOOT(token, mode, "BOOT")`.
- **Downlink.** Configuration frames replace telemetry packets. Byte 14 (the always-zero telemetry dummy byte) is
  `0xC7` and the low three bits of byte 0 are zero, so Ground Stations without this feature decode state INVALID.
  `STATUS` frames report protocol version, last sequence number and result, flags, flight state, text length/CRC,
  setting count and the boot token. `DATA` frames carry 11 bytes of the configuration text at an offset.
- **Reading.** The Vega serialises its configuration as text (`#board=`, `#fw=`, `#tele=`, then one `key=value` per
  setting, identical to the CLI `dump` values) and streams it as a carousel; the Ground Station fills gaps on the next
  lap and checks the CRC32.
- **Writing.** `BEGIN` copies the stored configuration into a staging copy. Each changed setting is sent as a
  `key=value` line in 6-byte chunks and applied with `LINE_APPLY`, which checks length and CRC32 and parses the value
  with the same limits as the CLI. `COMMIT` writes the staging copy to flash without changing the running
  configuration; the Vega restarts 1.5 s later.
- **Repetition.** The receiver repeats the last request until it is replaced, which covers lost frames. The Vega
  executes a request once per (seq, op, args).
- **Stale requests.** Destructive requests carry the boot token from the status frame (RTC backup counter mixed with
  ADC noise). After a restart a repeated old request is rejected (`Stale request`), so it can never cause a second
  save or restart. The Ground Station always replaces its request with `NOP` before closing a session.
- **Testing restart.** `REBOOT(testing)` sets RTC backup register DR1 to `0x7E57B007`. The next boot clears it and
  enters testing mode if a test phrase is configured. The stored `test_mode` value is not changed.
- **Safety gates on the Vega.** No session after liftoff (states >= THRUSTING) unless in testing mode; `BEGIN`,
  `COMMIT` and `REBOOT` only in INVALID/CALIBRATING/READY or testing mode and never while the recorder writes; the
  pending restart is checked again just before the reset. Testing frames (`0x72`) are now accepted only when the Vega
  runs in testing mode with a test phrase (previously a matching frame could arm the pyro channels outside testing
  mode).

## Tests and evidence

- `tests/remote_config_test.cpp` runs the real Vega server and settings table against the Ground Station client and
  profile model over a simulated radio link, including 20 % packet loss, stale frames after restarts, flight-state and
  recorder gates, rejected values, save failures and Configurator profile import/export. It runs in `gs-wsl-check.sh`
  (`test_remote_config`).
- Both firmwares build. The new screens were rendered with the production `Window` code and reviewed.
- **Not hardware-tested yet:** radio timing with real receivers, Vega flash save and restart, RTC backup register
  behaviour, LCD and buttons, USB drive sharing after profile operations. The desktop/WebAssembly simulator does not
  implement the *Vega* settings page (the controller ignores its button) and its golden System-page image changes
  because the page now has a right arrow.
