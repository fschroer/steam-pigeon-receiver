# Steam Pigeon — Receiver firmware

Part of the **Steam Pigeon** system (Locator + this Receiver + the app). The Receiver is a
transparent LoRa↔BLE relay: it forwards locator telemetry to the app over a VG6328A BLE
module and injects its own channel/name/battery/RSSI/version.

## System docs live in the Locator repo — read them first

All cross-system documentation is centralized at **`C:\STM32_Projects\Locator\docs\`**
(this repo carries only this pointer). Before non-trivial work, read:

1. **`C:\STM32_Projects\Locator\docs\SESSION_HANDOFF.md`** — the "resume here" map.
2. **`C:\STM32_Projects\Locator\docs\adr\README.md`** — the ADR index. Reference ADRs by
   **title, not number**. Receiver-relevant ones include the locator LoRa-channel-from-app
   decision (receiver auto-follows) and the app BLE connection-health probe.
3. **`C:\STM32_Projects\Locator\docs\SteamPigeon_SystemSummary.md`** — canonical reference
   (§3.6 Receiver, §4.2 features).

(Absolute paths assume the standard local layout; on a fresh clone elsewhere, open the
Locator repo to read its `docs/`.)

## Load-bearing points

- **The BLE module (VG6328A) advertises service UUID `FFE0` by default**, and this firmware
  must not change that (no `AT+UIDS`/`AT+SADV`/`AT+UADV`). iOS background scanning depends on
  it — see the iOS-port ADR. Firmware AT commands are in `Rocket/Communication/`.
- **BLE device name ≤ 22 chars.** Name + FFE0 + flags must fit the 31-byte advertisement;
  `device_name_length` is 20. Raising it past 22 risks dropping FFE0 from the advert, which
  silently breaks iOS background BLE.
- **Wire format / shared enums are hand-synced across firmware, app, and (planned) iOS.**
  Change all copies in the same session.

## Cross-repo work

When a change spans repos (a wire-format or protocol change), commit all sides in the same
session with cross-referenced hashes. `C:\STM32_Projects\Locator\Scripts\sp-status.sh`
reports commit/push state across all three repos.
