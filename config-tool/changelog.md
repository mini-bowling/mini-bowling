# Pin Configuration Tool - Changelog

## [v1.0.0] - 2026-02-20

Initial release.

- Browser-based single-file tool (`index.html`) — no server or build step required
- Displays all pin groups from `pin_config.h` as interactive cards (stepper, relay, sensors, servos, reset, frame LEDs, NeoPixel strips)
- Live input validation for Arduino Mega pin ranges (`0–53`, `A0–A15`)
- Real-time conflict detection
- Optional pin toggle for `STEPPER_ENABLE_PIN`
- Color-coded row states: purple = modified, amber = conflict, red = invalid
- Download button disabled until all errors are resolved
- Reset to Defaults button with confirmation prompt
- Collapsible instructions panel with step-by-step guide, color legend, and pin compatibility disclaimer
- Arduino Mega 2560 board diagram — live SVG with pins color-coded by group, conflict pins in red, hover tooltips
- Load Config File — parse an existing `pin_config.h` or `pin_config.user.h` to populate the tool
- All-defaults detection — warns before downloading if no changes have been made
- Post-download directions modal with placement instructions
- Inline SVG favicon, no external file required
- All processing done in the browser — no data sent anywhere
