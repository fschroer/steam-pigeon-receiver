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
- **Wire format / shared enums are hand-synced across firmware, app, and iOS.**
  Change all copies in the same session.

## The `/sp-*` commands

`.claude/commands/` here holds **pointers only**. The definitions live in
`C:\STM32_Projects\Locator\.claude\commands\` and must not be copied — they encode
rules the project learned the hard way, and two copies that can drift apart is the
problem the pointers exist to avoid. `sp-commit` and `sp-handoff` act on the Locator
repo's own files and should be run from there.

## The pre-commit hook

`.githooks/pre-commit` runs `Scripts/check-wire-format.sh` before any commit that stages a
`.c/.cpp/.h/.hpp`, and refuses the commit if it fails. Tracked, so **each clone installs it
once**:

```
git config core.hooksPath .githooks
```

It skips instantly for docs-only commits; `git commit --no-verify` is the deliberate escape.

**What it checks.** Every header under `Rocket/` carrying a `static_assert` is compiled as
its own translation unit, syntax-only, with the real target compiler and the real build's
defines (`-DSTM32WL5Mxx -DCORE_CM4 -DUSE_HAL_DRIVER`, read out of `Debug/**/subdir.mk`
rather than guessed). 23 assertions, about a second. Headers are **discovered**, not listed,
so one added tomorrow is guarded tomorrow.

**Why it exists.** The wire format is defined by hand in four places — the locator's
`MessageProtocol.hpp`, this repo's copy, `WireLayoutTest.kt` and `WireLayoutTests.swift` —
and a mismatch is not a compile error anywhere. It is a rocket whose telemetry decodes into
the wrong fields. The `static_assert`s were already the guard; they simply never fired
anywhere convenient, because only a full firmware build ran them and that needs the CubeIDE
toolchain plus an IDE-refreshed `Debug/` tree.

**A green run does not mean the firmware works.** Nothing is linked and nothing runs on the
MCU. It proves only that the asserted sizes and offsets still hold under the target
compiler — which is the whole of what those asserts claim.

The locator needs no equivalent: its `check-bench-flags.sh` compiles files that reach
`MessageProtocol.hpp`, so its asserts already fire there (verified 2026-09-04 by perturbing
`sizeof(TelemetryData)` and watching that hook refuse).

## Cross-repo work

When a change spans repos (a wire-format or protocol change), commit all sides in the same
session with cross-referenced hashes. `C:\STM32_Projects\Locator\Scripts\sp-status.sh`
reports commit/push state across all three repos.
