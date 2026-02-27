# Pin Configuration Tool

**Version 1.0.0**

A browser-based tool for building and customizing `pin_config.user.h` for the Mini Bowling Pinsetter (Arduino Mega). Open `index.html` directly in any modern browser — no server or build step required.

## What it does

Displays all hardware pin assignments from `pin_config.h` in a clean, grouped UI. You can edit any pin number, catch errors in real time, load an existing config file, and download a ready-to-use `pin_config.user.h` override file with step-by-step instructions on where to place it.

## Features

- **All pin groups displayed as cards** — Stepper motor, relay, sensors, servo motors, reset input, frame indicator LEDs, and NeoPixel LED strips
- **Arduino Mega 2560 board diagram** — live SVG visualization of the board with pins color-coded by group; conflicts shown in red; hover tooltips show pin name and group
- **Live validation** — inputs are checked against valid Arduino Mega pin ranges (`0–53` for digital, `A0–A15` for analog)
- **Conflict detection** — warns when the same pin number is assigned to two different signals
- **Optional pin toggle** — `STEPPER_ENABLE_PIN` is disabled by default; a toggle enables it
- **Color coding** — purple for modified values, red for conflicts, red for invalid inputs
- **Load Config File** — load an existing `pin_config.h` or `pin_config.user.h` to populate the tool with current pin assignments
- **All-defaults detection** — if all values match defaults, the tool warns you that a user override file isn't needed before downloading
- **Post-download directions** — a dialog walks you through exactly what to do with the downloaded file
- **Pin compatibility disclaimer** — reminds users that not all pins support the same hardware functions
- **Download disabled on errors** — the download button stays disabled until all conflicts and invalid values are resolved
- **Reset to Defaults** — restores all fields to their original values with a single click
- **Self-contained** — single HTML file, no dependencies, works from the filesystem (`file://`)

## How to use

1. Open `config-tool/index.html` in your browser
2. Optionally load an existing config file with **Load Config** to start from your current settings
3. Edit pin assignments as needed — the board diagram updates live
4. Click **Download pin_config.user.h**
5. Follow the on-screen directions to copy the file into your sketch directory:
   - `Everything/pin_config.user.h`
   - `Master_Test/pin_config.user.h`

The sketch automatically picks up `pin_config.user.h` when present (via `#if __has_include`).

## Pin reference

| Format | Range | Used for |
|--------|-------|----------|
| `0`–`53` | Digital pins | Steppers, relays, servos, sensors, LEDs |
| `A0`–`A15` | Analog pins | Ball sensor, speed sensor |

## Notes

- Not all Arduino pins support the same functionality. Some pins have hardware restrictions (PWM, interrupts, serial, I²C, SPI) that may affect component behavior. Reassign pins at your own discretion.
- `STEPPER_ENABLE_PIN` is optional — only enable it if your stepper driver has and requires an enable line.
