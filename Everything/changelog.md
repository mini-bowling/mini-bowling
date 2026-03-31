# Everything Script - Changelog

## Version 1.3.0 - 2026-03-30

### Added
- **Debug ring buffer** (`debug` serial command): Stores last 100 turret/IR debug messages in a ring buffer (4800 bytes SRAM). Dumped on pinsetter reset or via `debug` command.
- **IR jitter episode tracking**: Detects and logs IR sensor jitter episodes where raw transitions occur but debounce never confirms — helps diagnose missed pin detections (Case 1).
- **IR timing diagnostics**: Logs raw IR blocked/cleared transitions with timestamps and gap durations, mirroring the Master_Test buffered log format.
- **Deferred 10th pin release** (`tenthPinReady`): When the 10th pin is detected while the deck is down (e.g., during throw-1 grab/sweep/drop), the turret holds at slot 9 and waits for the deck to come back up before releasing. Prevents pins from falling while deck is occupied.
- **Release head-start delay** (`RELEASE_HEAD_START_MS`): Delays conveyor resume after a deferred turret release starts, giving the turret time to reach slot 10 before the conveyor feeds the next pin.
- **Conveyor timer freeze**: Catch delay and ninth settle timers are frozen while the conveyor is off, preventing premature expiration when a pin is still in transit on the belt.
- **Stepper enable pin support**: Configures `STEPPER_ENABLE_PIN` at startup if defined in `pin_config.h`.
- Config: `RELEASE_HEAD_START_MS`, `TLOAD_ARM_DELAY_MS` added to `general_config.h`
- Config: Real-world pin timing documentation added to `DEBOUNCE_MS`, `CATCH_DELAY_MS`, and turret timing section comments

### Changed
- **Strike sweep is now non-blocking**: Replaced blocking `StrikeSweepClearLane()` (which used `pumpAll()` waits) with FSM steps 11-14 in `runSequence()`. Strike detection at step 1 now transitions to a sweep-back/guard/wipe sequence without blocking the main loop.
- **IR re-arm uses arm delay instead of catch-delay suppression**: Removed the third re-arm suppression layer that prevented re-arming during the entire `CATCH_DELAY` phase. This caused missed pin detections when pins arrived during the 800ms window. Now uses only debounce (50ms) + arm delay (200ms), which safely re-arms ~170ms before the next pin arrives.
- **Catch delay starts from pin detection time**, not queue consumption time. Prevents cumulative timing drift when pins arrive during a previous catch delay.
- **Catch delay waits for pin to clear sensor** (`irStableState == HIGH` gate). Prevents advancing while a pin is still blocking the beam.
- **Ninth settle waits for pin to clear**: Requires `irStableState == HIGH` before transitioning to 10th pin wait. Eliminates phantom 10th pin detections from slow-clearing 9th pins or post-clear IR echoes.
- **Ninth settle resets debounce state**: Full IR state resync after ninth settle, starting the arm-delay countdown from a clean baseline.
- **Homing clears stale pin events**: `queuedPinEvents` reset to 0 on homing completion, preventing phantom detections from pre-homing IR activity.
- **Homing counts pre-blocked sensor**: If a pin is already at the IR sensor when homing completes, it's immediately queued rather than waiting for re-arm.
- **Step 7 gates on turret release completion**: Deck won't lower for pin drop while the turret is still releasing pins onto the sliding deck. Prevents collision when 10th pin release fires during throw-1 steps.
- **Scissors stay open during turret release**: Removed `ScissorsServo.write(SCISSOR_GRAB_ANGLE)` from 10th pin handling. Scissors only close during pin pickup.
- **Disabled postSetResumeDelay**: Removed the 2-second conveyor pause after deck-up during pin set. This could strand a pin mid-transit on the conveyor belt, causing catch delay expiration before pin arrival.
- **Conveyor stops when 10th pin is deferred** (`tenthPinReady`): No more pins are needed, so the conveyor shuts off immediately to prevent double-loading.
- **IR re-arm suppressed when `tenthPinReady`**: Prevents phantom 11th pin events from conveyor inertia or pin wobble after the 10th pin is detected.

### Fixed
- Removed blocking `StrikeSweepClearLane()` that could stall the main loop during strikes
- Fixed race condition where background refill at step 2 could trigger turret release while deck was not ready

## Version 1.2.3 - 2026-02-23

### Added
- `SCOREMORE_USER` config setting to enable/disable waiting to connect to Scoremore for initialization. Default is `0` (off) — set to `1` to wait for Scoremore to connect. (thanks to Wahapainan)
- Added project README.md file

### Fixed
- Config redefinition warnings: every `#define` in `general_config.h`, `pin_config.h`, and `.user.h` files is now wrapped in `#ifndef`/`#endif` guards
- Include order corrected so the user file is included first and the default file second; user values now take priority without requiring redefinition

## Version 1.2.2 - 2026-02-21

### Changed
- Version numbering is now kept in sync between Master Test and Everything scripts
- `general_config.h` and `pin_config.h` no longer use a file-level `#ifndef` guard, so user config files (`.user.h`) now properly override default values when present

### Migration
- If you have a `general_config.user.h` or `pin_config.user.h`, re-copy it from the corresponding default config file to remove the `#ifndef`/`#define` guard at the top of the file (e.g. `#ifndef GENERAL_CONFIG_H` / `#define GENERAL_CONFIG_H`). Only your customized values need to remain in the user file.

## Version 1.2.0 - 2026-02-19

### Fixed
- Fixed scoremore ACK code with strikes
- Fixed config to load only overrides in user level configs

## Version 1.2.0 - 2026-02-19

### Added
- `pin_config.h` — hardware pin assignments extracted from inline defines, matching the test script pattern
- `general_config.h` — all user-adjustable settings (servo angles, LED lengths, timing constants) extracted into a shared config file
- `DECK_LED_BRIGHTNESS` — separate brightness setting for deck LEDs, independent of lane LED brightness
- Optional `pin_config.user.h` and `general_config.user.h` overrides (git-ignored) for per-machine customization
- Added support for PINSETTER:RESET command from Scoremore
- Added descriptions for currently unused Scoremore pins for completeness
- Added support for reset button
- Added blink to throw two led as entering setup indicator
- Added CHECK_READY command to improve connectivity with Scoremore
- Added VERSION command to allow reporting of version to Scoremore
- Added ACK responses to all commands

### Changed
- Simplified raise servo config to match sweep servo pattern: define one angle, compute the mirror (R = 180 - L)
- Fixed bug that was causing pins to be dropped while the deck was lowered
- Fixed bug that was causing scoremore to loose communications to the Arduino
- Fixed bug, enabled INPUT_PULLUP on BALL_SPEED_PIN
- Cleaned mapping of pin assignments between Arduino and Scoremore, utilizing a struct now
- Renamed LED strip pin defines from A/B to L/R (`DECK_PIN_A`→`DECK_PIN_L`, `DECK_PIN_B`→`DECK_PIN_R`, `LANE_PIN_A`→`LANE_PIN_L`, `LANE_PIN_B`→`LANE_PIN_R`) for consistency with config values that already use L/R naming
- Renamed NeoPixel objects from A/B to L/R (`deckA`→`deckL`, `deckB`→`deckR`, `laneA`→`laneL`, `laneB`→`laneR`)

## Version 1.0 (Rev67) - 2026-02-14

Original files by Danny Lum (Rev67), moved to a new GitHub repository as version 1.0.
