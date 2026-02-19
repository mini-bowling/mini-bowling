# Mini Bowling Pinsetter

Firmware for the Danny Lum designed and built mini bowling pinsetter controlled by an **Arduino Mega 2560**. The
pinsetter automatically sets pins onto the lane, clears fallen pins after each throw, and communicates with the
Scoremore scoring system.

## Arduino Sketches

### `Everything/` — Production Firmware

The main firmware that runs on the machine during normal operation. Handles the full pinsetter cycle: loading pins from
the turret, setting them on the lane, clearing after throws, and LED effects. Communicates with a Scoremore scoring
system over serial.

### `Master_Test/` — Component Test Script

An interactive serial menu for calibrating and testing each component individually. Connect via any serial monitor at
the configured baud rate and type `help` for the main menu. Useful during initial setup, calibration, or
troubleshooting.

### `Homing/` — Servo Homing Utility

A minimal standalone sketch for setting the servo homing positions for each servo (homing means to set the servo to
their declared home position). Useful during the assembly when the assembly calls for a servo to be homed. Note that the
Master Test script can also be used to set servos to their home positions.

## Dependencies

| Library                                                               | Purpose                        |
|-----------------------------------------------------------------------|--------------------------------|
| [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)    | LED strip control              |
| [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) | Turret stepper motor           |
| Servo                                                                 | Arduino built-in servo control |

The dependency libraries need to be installed in Arduino IDE prior to compiling these programs or else compile errors
will occur.

## Configuration

The Everything and Master_Test sketches use two configuration headers. Note that these files are identical to each other
between the Everything and Master_Test skatches/programs.

| File               | Purpose                                                          |
|--------------------|------------------------------------------------------------------|
| `pin_config.h`     | Hardware pin assignments                                         |
| `general_config.h` | Calibration values: servo angles, timing constants, LED settings |

### Per-Machine Configuration Overrides

To customize settings for a specific machine without modifying the checked-in defaults:

1. Copy `general_config.h` to `general_config.user.h` in the same sketch directory (and/or `pin_config.h` to
   `pin_config.user.h` if pin assignments need to change).
2. Open the `.user.h` file and change the values you want to customize.
3. Save the file and recompile.

Each setting in the config files is wrapped in an `#ifndef` guard, and the `.user.h` file is included before the
default `.h` file. This means the user file's values take priority — any setting you define in your `.user.h` will
be used, and any setting you leave unchanged will fall back to the default. You do not need to remove settings from
the `.user.h` file; unchanged values will simply match the defaults.

The `.user.h` files are git-ignored, so your per-machine customizations will not be overwritten by future updates.
If a new setting is added to `general_config.h` or `pin_config.h`, it will automatically use its default value
until you add an override to your `.user.h` file.
