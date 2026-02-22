# Changelog

All notable changes to the Master Test Script will be documented in this file.

## [v1.2.2] - 2026-02-21

### Changed
- Version numbering is now kept in sync between Master Test and Everything scripts
- `general_config.h` and `pin_config.h` no longer use a file-level `#ifndef` guard, so user config files (`.user.h`) now properly override default values when present

### Migration
- If you have a `general_config.user.h` or `pin_config.user.h`, re-copy it from the corresponding default config file to remove the `#ifndef`/`#define` guard at the top of the file (e.g. `#ifndef GENERAL_CONFIG_H` / `#define GENERAL_CONFIG_H`). Only your customized values need to remain in the user file.

## [v1.4.2] - 2026-02-21

### Fixed
- Fixed config to load only overrides in user level configs
- Fixed test script to set baud from general_config

## [v1.4.1] - 2026-02-18

### Changed
- Renamed LED strip pin defines from A/B to L/R (`DECK_PIN_A`→`DECK_PIN_L`, `DECK_PIN_B`→`DECK_PIN_R`, `LANE_PIN_A`→`LANE_PIN_L`, `LANE_PIN_B`→`LANE_PIN_R`) for consistency with config values that already use L/R naming
- Renamed NeoPixel objects from A/B to L/R (`deckA`→`deckL`, `deckB`→`deckR`, `laneA`→`laneL`, `laneB`→`laneR`)

## [v1.4] - 2026-02-03

### Added
- **Sensor menu** (`sensor` / `se` from main menu) for controlling sensor monitoring:
  - Toggle individual sensors: `hall`, `ir`, `ball`, `speed`
  - Bulk enable/disable: `allon`, `alloff`
  - `status (s)` shows enabled/disabled state alongside current pin readings
- **IR flap detection**: Automatically disables IR sensor monitoring when rapid toggling is detected
  - Triggers after 5 consecutive state changes within 200ms windows
  - Prints a warning and instructs the user to re-enable via the sensor menu
  - Prevents serial output flooding from noisy/disconnected IR sensors
- **Turret homing guard**: The `home` command in the turret menu now checks that Hall sensor monitoring is enabled before starting the homing sequence, since homing depends on the Hall effect sensor
- **Scissor cycle command**: `cycle (cy)` in the scissor menu continuously alternates between grab and drop; `stop (x)` halts cycling
  - Cycle interval controlled by `SCISSOR_CYCLE_MS` constant (default 2000ms)
  - Menu and status messages display the interval dynamically from the constant
  - Supports custom angles: `cy,90,140` or `cycle 90 140` cycles between any two angles

- **Sequence menu** (`sequence` / `sq` from main menu) for running automated multi-step sequences:
  - Interactive y/n prompts for sequence options (tracked by `SequencePrompt` enum)
  - **Pin drop sequence** (`pindrop` / `pd`):
    - Asks whether to pre-clear lane (sweep before setting) and post-clear (sweep after setting)
    - Pre-clear: raises deck, sweeps back, sweeps to guard
    - Drop: opens scissor, lowers deck to down/set position, extends sliding deck, raises deck (while slider still extended), retracts sliding deck
    - Post-clear: sweeps back, sweeps to guard
    - Correct ordering: release → raise up → slider home (matches production code)
    - Uses `PDROP_RAISE_SETTLE_MS` (1300ms) for raise servo moves, `PDROP_SETTLE_MS` (500ms) for small servo moves
  - **Sweep clear sequence** (`sweep` / `sw`):
    - Reusable FSM used both standalone and by pin drop for pre/post clear
    - Raises deck if not already up, sweeps back, sweeps to guard
    - Checks current raise position to skip redundant raise
  - **Turret load sequence** (`turretload` / `tl`):
    - Homes turret, starts conveyor, catches 9 pins into slots 1-9
    - Waits for 10th pin to trigger release at spring-safe speed
    - Prepares deck before release (sliding deck to home/catch, scissor to grab)
    - Re-homes turret after release, stops conveyor
    - Independent IR debounce for pin detection (separate from monitorInputs)
    - Pin queue for handling pins arriving faster than catch delay
    - Handles IR sensor already blocked when loading starts (always arms detection)
    - Persistent pin count tracking (`turretPinsLoaded`) survives sequence stop/restart
    - Resume from partial load: shows current count, asks user to confirm before continuing
  - **Global stop command** (`x` / `stop`): Stops any running sequence from any menu
- **Pin drop timing constants** in `general_config.h`:
  - `PDROP_SETTLE_MS` (500ms) — small servo settle time
  - `PDROP_RAISE_SETTLE_MS` (1300ms) — raise servo full-travel settle time
  - `PDROP_DROP_MS` (800ms) — pin drop dwell time
- **Turret load timing constants** in `general_config.h`:
  - `TLOAD_CATCH_DELAY_MS` (800ms) — pause after catching pins 1-8
  - `TLOAD_RELEASE_DWELL_MS` (1000ms) — dwell at release position
  - `TLOAD_NINTH_SETTLE_MS` (300ms) — settle after 9th pin caught
- **Homing improvement**: Hall sensor now checked during all homing phases including the initial prep move, preventing the turret from passing through the sensor without stopping

### Changed
- Simplified raise servo config to match sweep servo pattern: define one angle, compute the mirror (R = 180 - L)
  - Replaced `RAISE_UP_ANGLE_L`/`_R`, `RAISE_DOWN_ANGLE_L`/`_R`, `RAISE_GRAB_ANGLE_L`/`_R`, `RAISE_DROP_ANGLE_L`/`_R` with `RAISE_UP_ANGLE`, `RAISE_DOWN_ANGLE`, `RAISE_GRAB_ANGLE`, `RAISE_DROP_ANGLE`
  - Right servo angle is now computed as `180 - ANGLE` in code, same as sweep servos
- `monitorInputs()` now respects per-sensor enable/disable flags
- Renamed script file to `Master_Test_v1.4.ino`
- Renamed "Slider" to "Sliding Deck" in all user-facing text (menu titles, serial messages, status display)
- Sliding deck home position labeled "Home / Catch" in menus and config comments
- Pin drop moved from main menu to sequence menu

### Fixed
- Pin drop now uses `RAISE_DOWN_ANGLE` (down/set) instead of `RAISE_DROP_ANGLE` (drop) when setting pins on the lane
- Pin drop ordering corrected: deck raises while slider is still extended, then slider retracts (was previously slider home → raise up)
- Homing no longer misses the hall sensor during the prep move (could cause full extra revolution)

## [v1.3] - 2026-02-01

### Added
- `home (ho)` command added to all servo submenus for quick return to home/neutral position:
  - **Scissor menu**: Home sets to SCISSOR_DROP_ANGLE (open position)
  - **Slider menu**: Already had home command (SLIDER_HOME_ANGLE)
  - **Raise menu**: Home sets to RAISE_UP_ANGLE (deck up position)
  - **Sweep menu**: Home sets to SWEEP_BACK_ANGLE (back of lane position)
  - **Ball Door menu**: Home sets to BALL_DOOR_CLOSED_ANGLE (closed position)
- Home positions are configurable via `general_config.h`
- Sweep servo tweening for smooth movement between positions (like production script)
- Independent LED strip lengths for left and right sides:
  - `DECK_LED_LENGTH_L` / `DECK_LED_LENGTH_R` for deck strips
  - `LANE_LED_LENGTH_L` / `LANE_LED_LENGTH_R` for lane strips
  - All LED functions and animations updated to handle different lengths
  - Uses `SWEEP_TWEEN_MS` from `general_config.h` (default: 500ms)
  - Sweep status now shows "(moving...)" when tween is in progress
- `disengage (de)` command added to all servo and turret menus:
  - Detaches servos so they can be moved manually without resistance
  - Status display shows "(disengaged)" when servos are detached
  - Servos automatically re-engage on next move command
  - Turret disengage requires `STEPPER_ENABLE_PIN` to be defined in `pin_config.h`
- Optional `STEPPER_ENABLE_PIN` support in `pin_config.h` for turret stepper driver

### Changed
- Menu displays now show dynamic angle values from `general_config.h` instead of hardcoded values
  - Scissor, Slider, Raise, Sweep, and Ball Door menus all updated
  - Angle values in help text now reflect actual configured values
- Renamed script file to `Master_Test_v1.3.ino`

### Fixed
- Servos no longer move when entering their respective test menus
  - Servos are now attached lazily - only when the user issues their first move command
  - Added `ensureXxxAttached()` helper functions for each servo group

## [v1.2] - 2026-01-29

### Added
- Independent left/right strip control for Lane LEDs
  - `left (l)` - Control LEFT strip only (pin 52)
  - `right (r)` - Control RIGHT strip only (pin 53)
  - `both (bo)` - Control BOTH strips (default)
- Independent left/right strip control for Deck LEDs
  - `left (l)` - Control LEFT strip only (pin 50)
  - `right (r)` - Control RIGHT strip only (pin 51)
  - `both (bo)` - Control BOTH strips (default)
- New `StripSelect` enum for tracking which strip(s) are targeted
- New helper functions for selected strip operations:
  - `deckSetColor()` / `laneSetColor()` - Set color on selected strip(s)
  - `deckShowSelected()` / `laneShowSelected()` - Update selected strip(s)
  - `deckSetBrightnessSelected()` / `laneSetBrightnessSelected()` - Set brightness on selected strip(s)
- Status display now shows which strip(s) are currently selected

### Changed
- Lane LED menu now shows strip selection options at the top
- Deck LED menu now shows strip selection options at the top
- All LED animations (wipe, flash, comet, rainbow) now respect strip selection
- Renamed `red (r)` shortcut to `red (re)` in LED menus to avoid conflict with `right (r)`
- Renamed script file to `Master_Test_v1.2.ino`

## [v1.1] - 2026-01-28

### Added
- `pin_config.h` - Separate configuration file for all hardware pin assignments
- `general_config.h` - Separate configuration file for all user-adjustable settings
- `DECK_LED_BRIGHTNESS` — separate brightness setting for deck LEDs, independent of lane LED brightness
- Optional `pin_config.user.h` and `general_config.user.h` overrides (git-ignored) for per-machine customization

### Changed
- Renamed `FRAME_LED1` to `FRAME_LED1_PIN` for consistency with other pin definitions
- Renamed `FRAME_LED2` to `FRAME_LED2_PIN` for consistency with other pin definitions
- Renamed `MOTOR_RELAY` to `MOTOR_RELAY_PIN` for consistency
- Renamed `IR_SENSOR` to `IR_SENSOR_PIN` for consistency
- Renamed `HALL_EFFECT` to `HALL_EFFECT_PIN` for consistency
- Renamed `DECK_LENGTH` to `DECK_LED_LENGTH` for clarity
- Renamed `LANE_LENGTH` to `LANE_LED_LENGTH` for clarity
- Moved all hardcoded servo angles to `general_config.h`:
  - Sweep servo angles (Guard, Up, Back positions)
  - Ball door angles (Open, Closed)
  - Scissor angles (Grab, Drop)
  - Slider angles (Home, Release)
  - Raise servo angles (Up, Down, Grab, Drop for both L/R)
- Moved turret configuration to `general_config.h`:
  - Speed and acceleration settings
  - Home adjuster value
  - Pin positions array values
  - Pin 10 release offset
- Moved LED and timing configuration to `general_config.h`:
  - LED brightness
  - Animation timing values
  - Debounce timing
  - Frame LED blink interval
- Renamed script file to `Master_Test_v1.1.ino`

### Improved
- Configuration is now centralized for easier calibration
- Pin assignments separated from tunable parameters
- All servo angles now use named constants for clarity

## [v1.0] - Initial Release

### Features
- Interactive serial menu system for testing all pinsetter components
- Continuous sensor monitoring (Hall effect, IR, Ball trigger, Ball speed)
- Component test menus:
  - Conveyor relay control
  - Sweep servo positioning
  - Ball door servo control
  - Lane LED strip control with animations
  - Deck LED strip control with animations
  - Scissor servo positioning
  - Slider servo positioning
  - Raise servo positioning
  - Turret stepper control with homing sequence
  - Frame indicator LED control with blink modes
- LED animations: Wipe, Flash, Comet, Rainbow, Strike effect
- Servo detachment on menu exit to prevent buzzing
- Turret homing state machine with Hall effect sensor
