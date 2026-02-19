# Everything Script - Changelog

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
