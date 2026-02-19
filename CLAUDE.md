# Bowling Pinsetter - Arduino Project

## Project Overview
This is firmware for a custom-built **bowling pinsetter** controlled by an Arduino Mega. The code provides a serial-based interactive test menu for calibrating and operating individual pinsetter components.

## Physical Machine Description

The pinsetter is a mechanical device that sets bowling pins onto a lane and clears them after each throw. It has these major assemblies:

### Deck (Upper Assembly)
The **deck** sits above the lane and holds the pins before they are dropped. It contains:
- **Scissor mechanism** (servo on pin 7): Opens/closes to grip or release the pins from the lane. `SCISSOR_GRAB_ANGLE` grips, `SCISSOR_DROP_ANGLE` releases.
- **Sliding deck** (servo on pin 8): Slides the sliding deck into the drop position and back. Key positions:
  - **Home / Catch** (`SLIDER_HOME_ANGLE`): Retracted position — pins from the turret fall into the sliding deck here
  - **Release** (`SLIDER_RELEASE_ANGLE`): Extended position — slides the pin plate forward to drop pins onto the lane. Scissor must be open/drop position to drop the pins from the sliding deck.
- **Raise servos** (2 servos, pins 9 & 10): Raise and lower the entire deck assembly. Left and right are mirrored (R = 180 - L conceptually). Key positions:
  - **Up** (`RAISE_UP_ANGLE`): Deck is fully raised, out of the way of the lane
  - **Drop** (`RAISE_DROP_ANGLE`): Deck lowered partway — used only when scissors have picked up pins and need to drop them again (re-drop scenario)
  - **Grab** (`RAISE_GRAB_ANGLE`): Deck is at grab height, for the scissors to pick up pins from the lane
  - **Down/Set** (`RAISE_DOWN_ANGLE`): Deck fully lowered to the lane surface — used for **setting pins** onto the lane, releasing them from the sliding deck (pin drop sequence)

### Lane Level
- **Sweep arms** (2 servos, pins 11 & 12): Left and right arms that sweep across the lane to clear fallen pins. They are mirrored (R = 180 - L). It's important to note that these servos are not feedback servos. When the system is off, assume the position is in the guard position. Key positions:
  - **Guard** (`SWEEP_GUARD_ANGLE`): Arms are in guard position, partially blocking the lane (prevents pins from rolling forward, protects against ball throws)
  - **Back** (`SWEEP_BACK_ANGLE`, typically 0): Arms sweep fully back across the lane, pushing all pins/debris onto the conveyor
  - **Up** (`SWEEP_UP_ANGLE`): Arms raised up out of the way. Note that the sweep should only be in the up position for no more than 5 minutes. At this time, the system should pause.
- **Ball return door** (servo on pin 13): Opens/closes to let the bowling ball pass through to the return track. Closes to protect from pins rolling out.

### Turret (Pin Loading)
- **Stepper motor** (pins 2 & 3): Rotates the turret carousel that holds individual pins in slots (1-9). Slot 10 is the release position. Uses a hall effect sensor for homing.
- **Hall effect sensor** (pin 6): Detects a magnet on the turret for homing to a known position.

### Conveyor
- **Motor relay** (pin 4): Drives the conveyor belt that transports pins from the pit back up to the turret.

### Sensors
- **IR sensor** (pin 5): Detects pins crossing the conveyor path (pin present on conveyor)
- **Ball trigger sensor** (A0): Detects when a ball has been thrown
- **Ball speed sensor** (A1): Second sensor for ball speed measurement

### LEDs
- **Deck LED strips** (NeoPixel, pins 50 & 51): Left and right LED strips on the deck
- **Lane LED strips** (NeoPixel, pins 52 & 53): Left and right LED strips along the lane
- **Frame indicator LEDs** (pins 46 & 47): Simple on/off LEDs for frame status

## Physical Safety Rules & Constraints

These rules MUST be followed in any automated sequence to avoid mechanical damage:

1. **Deck must be raised before sweeping to clear pins.** The sweep arms pass underneath the deck. If the deck is down while sweeping, the arms will collide with the deck or pins still in the mechanism.

2. **Sweep arms should return to guard position after any sweep operation.** Guard position keeps them partially blocking the lane as a backstop.

3. **Scissor must be in drop (open) position before lowering the deck to set pins.** If the scissor is in grab position while the deck lowers, the pins will not be able to slide through the scissors to drop properly.

4. **Sliding deck must return to home/catch after releasing pins.** It must be in the catch position so it can catch the next batch of pins.

5. **Raise servos are mirrored.** Left and right always move together but to opposite angles. They always move as a pair.

6. **Raise servos need longer settle time than other servos.** The raise servos travel up to 160 degrees (e.g. UP 180 → DOWN/SET 20). Use `PDROP_RAISE_SETTLE_MS` (1300ms) for raise moves, not `PDROP_SETTLE_MS` (500ms). The production code gives 1300-1500ms for raise moves (`DECK_EXTRA_SETTLE_MS` 500ms internal + 800-1000ms external). Always wait for the raise to fully complete before moving the slider or sweeping.

7. **Sweep servos are mirrored.** Right angle = 180 - left angle. They always move as a pair.

8. **Always use tweening for sweep movements.** Never call `LeftSweepServo.write()` / `RightSweepServo.write()` directly. Always use `startSweepTo()` which provides smooth tweened motion. The only direct `.write()` calls should be inside the tween engine (`updateSweepTween()`).

9. **Sweep arms are always in guard position at rest.** On startup, the sweep tweens to guard. Every sweep operation ends by returning to guard. All sweep sequences should assume the sweep starts in guard position.

10. **Servos are only attached when needed** and detached when leaving their menu. This applies only to the test script. This prevents buzzing/holding current when not in use. Use the `ensure*Attached()` helpers.

11. **Turret must be homed before slot positions are accurate.** The homing sequence uses the hall effect sensor to find the reference point. Homing must be performed at least once before setting pin positions.

12. **Turret must never pass through the hall sensor without stopping** (except when moving to the release position for pin drop). The homing FSM checks the hall sensor during ALL phases including the initial prep move. If the hall sensor is triggered at any point during homing, the turret stops and enters the backoff/precision-find sequence.

13. **Use spring-safe speed when moving to turret release position.** The release position (slot 10) has a spring mechanism. Moving at full speed can eject pins violently. Always use `TURRET_SPRING_MAXSPEED` / `TURRET_SPRING_ACCEL` for this move.

14. **Turret must be empty before running a load sequence.** Loading pins into an already-loaded turret will cause jams. The test script prompts the user to verify this.

15. **Use Down/Set position (not Drop) when setting pins on the lane.** `RAISE_DOWN_ANGLE` lowers the deck fully to the lane surface for setting pins. `RAISE_DROP_ANGLE` is only for re-dropping pins that were picked up by the scissors — it does not lower all the way.

16. **Sliding deck must be at home/catch position before turret releases pins.** The sliding deck catches pins from the turret. If it is extended (release position), pins won't seat into the deck properly.

17. **Scissor should be in drop position before sliding deck releases pins.** This keeps the scissors open so the pins can slide through the opening before the sliding deck moves to the release position.

## Test Script: Pin Drop Sequence (Automated)

The pin drop sequence sets a rack of pins on the lane. It is accessed via the **Sequence** menu (`sq` from main menu), then `pd`/`pindrop`. The user is prompted for pre-clear and post-clear options.

### Full sequence with pre-clear and post-clear:
1. **Raise deck up** - Get deck out of the way so sweep can clear
2. **Sweep back** - Push any existing pins/debris off the lane
3. **Sweep to guard** - Return sweep arms to guard position
4. **Scissor open** - Open scissor to release position
5. **Raise to down/set** - Lower deck fully to the lane surface to set pins
6. **Sliding deck release** - Extend sliding deck to drop pins through the opening
7. **Raise up** - Raise deck back up **while slider is still extended** (pins must clear the deck before slider retracts)
8. **Sliding deck home** - Retract sliding deck back to home/catch position
9. **Sweep back** - Clear the pins just dropped (post-clear)
10. **Sweep to guard** - Return to guard

**IMPORTANT ordering: release → raise up → slider home.** The deck must rise while the slider is still extended so that pins fall free. If the slider retracts before the deck rises, pins can get caught between the sliding deck and the deck frame. This matches the production code sequence (`SlidingDeckRelease` → `DeckUp` → `SlidingDeckHome`).

### Without pre-clear:
Starts at step 4 (scissor open), skipping steps 1-3.

### Without post-clear:
Ends at step 8 (sliding deck home), skipping steps 9-10.

## Test Script: Sweep Clear Sequence (Automated / Reusable)

The sweep clear sequence clears fallen pins from the lane by sweeping the arms across. It is a **reusable FSM** used both standalone (from the Sequence menu via `sw`/`sweep`) and as a sub-operation within the pin drop sequence (for pre-clear and post-clear).

### Sequence steps:
1. **Check deck position** - If the deck is already raised (raise servos at `RAISE_UP_ANGLE`), skip to step 3
2. **Raise deck up** - Raise the deck so sweep arms can pass underneath. Wait `PDROP_SETTLE_MS` for servos to reach position
3. **Sweep back** - Tween sweep arms to `SWEEP_BACK_ANGLE` to push pins/debris off the lane
4. **Wait for sweep animation** - Wait for the tween to complete
5. **Sweep to guard** - Tween sweep arms to `SWEEP_GUARD_ANGLE` (backstop position)
6. **Wait for sweep animation** - Wait for the tween to complete
7. **Done** - Set `sweepClearActive = false`, print completion message

### Usage by pin drop:
- **Pre-clear**: `startPinDrop()` calls `startSweepClear()` and enters `PDROP_PRECLEAR_WAIT`, which polls `sweepClearActive` until the sweep finishes
- **Post-clear**: After raising the deck, `runPinDropFSM()` calls `startSweepClear()` and enters `PDROP_POSTCLEAR_WAIT`, same polling pattern

### Key design decisions:
- **Assumes sweep starts in guard position.** On startup, sweep tweens to guard. Every sweep clear ends at guard. All code should maintain this invariant.
- **All sweep movements use tweening** via `startSweepTo()`. Never write directly to sweep servos.
- The deck-up check (`raiseLeftPos == RAISE_UP_ANGLE`) avoids redundant raise operations when called after pin drop already raised the deck
- Only one sweep clear can run at a time (checked at start)

## Test Script: Turret Load Sequence (Automated)

The turret load sequence fills the turret with 10 pins from the conveyor, then releases them all onto the deck. Pins 1-9 go into turret slots; the 10th pin triggers the move to the release position, where all pins fall off. The sliding deck must be in the catch position before moving to this position. Accessed via the **Sequence** menu (`sq` from main menu), then `tl`/`turretload`. The user is prompted to confirm the turret is empty before starting.

### How the turret works:
- The turret is a rotating carousel with 9 pin-holding slots (positions 1-9) and a release position (position 10)
- A stepper motor rotates it, and a hall effect sensor + magnet provides a home reference
- The IR sensor (pin 5) detects when a pin arrives from the conveyor into the current slot
- The conveyor belt continuously feeds pins upward from the pit to the turret

### Pin count tracking:
The global variable `turretPinsLoaded` persists across sequence starts/stops. It tracks how many pins are currently in the turret (0-9). It is updated as each pin is caught and reset to 0 when pins are released to the deck. If the turret load is stopped mid-sequence, the count reflects however many pins were successfully loaded.

When starting a turret load:
- **0 pins**: Prompts "Verify turret is empty" (y/n). Loads from slot 1.
- **1-9 pins**: Shows the current count and asks user to confirm. If confirmed, resumes loading from the next slot. If denied, resets count to 0 and cancels (user should clear turret manually).

### Loading sequence:
1. **Home turret** - Find the hall sensor reference point, move to next slot to fill
2. **Start conveyor** - Begin feeding pins from the pit
3. **Catch pins 1-8** - For each pin:
   - Wait for IR sensor to detect pin arrival
   - Wait `TLOAD_CATCH_DELAY_MS` (800ms) for the pin to settle in the slot
   - Advance stepper to the next slot position
4. **Catch pin 9** - Wait for IR detection, then `TLOAD_NINTH_SETTLE_MS` (300ms) settle. Turret stays at slot 9.
5. **Wait for 10th pin** - Turret remains at slot 9, conveyor keeps running. When the IR sensor detects the 10th pin arriving, prepare deck and advance to release.
6. **Prepare deck** - Ensure sliding deck is at home/catch position (so pins fall in properly) and scissor is in grab position (to hold pins on the deck).
7. **Move to release** - Rotate to position 10 at spring-safe speed (`TURRET_SPRING_MAXSPEED`) to avoid launching pins. The 10th pin falls through the release mechanism along with all others.
7. **Dwell at release** - Wait `TLOAD_RELEASE_DWELL_MS` (1000ms) for pins to fall off onto the deck
8. **Re-home turret** - Home again to reset position reference
9. **Stop conveyor** - Turn off the conveyor belt

### Physical constraints for turret loading:
- **Turret must be empty before loading.** Loading into an already-loaded turret will jam pins.
- **Hall sensor must be enabled.** Homing requires the hall effect sensor to be active.
- **Sliding deck must be at home/catch position before turret release.** The sliding deck catches pins falling from the turret. If it is extended, pins won't seat properly.
- **Scissor should be in grab position before turret release.** The scissor grips the pin plate on the deck to hold pins in place as they fall from the turret.
- **Spring-safe speed for release.** Moving to the release position (slot 10) uses slower speed (`TURRET_SPRING_MAXSPEED` / `TURRET_SPRING_ACCEL`) because the spring mechanism that holds pins can eject them violently at high speed.
- **Pin queue.** If pins arrive from the conveyor faster than the catch delay, they are queued and processed one at a time to avoid missed detections.
- **IR debounce.** The turret load FSM uses its own independent IR debounce (separate from `monitorInputs()`) using `DEBOUNCE_MS` (50ms) to reliably detect pin arrivals.
- **IR sensor may already be blocked when loading starts.** A pin can be sitting at the IR sensor before the turret load begins (or before the 10th pin wait begins). The code always sets `tlPinArmed = true` when entering a catch phase, so a pre-blocked sensor immediately triggers a pin detection. This avoids the turret getting stuck waiting for a pin that's already there.

### Relationship to Everything:
The turret loading logic in the test script mirrors how `Everything.ino` loads pins in production. Key differences:
- Everything uses `pumpAll()` for blocking waits; the test script uses a non-blocking FSM
- Everything has complex background refill logic with `backgroundRefillRequested`, `conesFullHold`, etc.; the test script does a single load-and-release cycle
- Everything tracks `loadedCount`, `NowCatching`, `queuedPinEvents` globally; the test script uses `tl`-prefixed local state

## Test Script: Code Architecture

### Menu System
- State machine driven by `MenuState` enum
- `processCommand()` routes serial input to the current menu's handler
- Global commands (`help`, `back`) work from any menu
- Each component has a `print*Menu()` and `handle*Menu()` function pair
- The **Sequence** menu has interactive y/n prompts (tracked by `SequencePrompt` state)

### FSM Pattern
Long-running operations (homing, sweep clear, pin drop, turret load) use non-blocking finite state machines:
- Each phase does its action, records `millis()` timestamp, and advances to a wait phase
- Wait phases check elapsed time or animation completion before advancing
- The FSM runs every `loop()` iteration, never blocks

### Servo Management
- Servos are attached lazily via `ensure*Attached()` when first used
- Servos are detached when leaving a menu (`detachCurrentServos()`) to prevent holding torque
- Exception: Sweep servos tween to guard on startup and stay active during pin drop FSM

### Sweep Tweening
- Sweep servos use smooth tweening (`startSweepTo()` / `updateSweepTween()`) instead of instant jumps
- `sweepAnimating` flag indicates tween in progress
- Duration controlled by `SWEEP_TWEEN_MS`

### Configuration Files
- `pin_config.h` - Hardware pin assignments (rarely changed)
- `general_config.h` - Calibration values, servo angles, timing constants (frequently adjusted per machine)
- `general_config.user.h` - Optional per-machine override (copy of general_config.h, git-ignored)

## Test Script: Coding Conventions
- Use `F()` macro for all `Serial.println` string literals (stores in flash, saves RAM)
- Use `Serial.println(cond ? "YES" : "NO")` for boolean status prints
- All servo angles constrained to 0-180
- Timing uses `unsigned long` and `millis()`, never `delay()`
- Menu commands are lowercase, checked with `==` after `cmd.toLowerCase()`
- Short aliases provided for all commands (e.g., `sw` for `sweep`, `pd` for `pindrop`)
