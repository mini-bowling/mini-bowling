// =====================================================
// USER CONFIGURATION - DEFAULT VALUES
// This is the distribution template with factory defaults.
// Copy to general_config.user.h and customize for your machine.
// =====================================================

#ifndef GENERAL_CONFIG_H
#define GENERAL_CONFIG_H

// =====================================================
// SCOREMORE SERIAL BAUD RATE
// =====================================================
#define SCOREMORE_BAUD      9600  // DEFAULT: 9600

// =====================================================
// LED STRIP LENGTHS
// Left and right can be set independently if needed
// =====================================================
#define DECK_LED_LENGTH_L   11    // Left deck strip.  DEFAULT: 11
#define DECK_LED_LENGTH_R   11    // Right deck strip. DEFAULT: 11
#define LANE_LED_LENGTH_L   41    // Left lane strip.  DEFAULT: 41
#define LANE_LED_LENGTH_R   41    // Right lane strip. DEFAULT: 41

// =====================================================
// CONVEYOR SETTINGS
// =====================================================
// Set to 1 if relay is active HIGH, 0 if active LOW.
#define CONVEYOR_ACTIVE_HIGH 1    // DEFAULT: 1

// =====================================================
// LED BRIGHTNESS (0-255)
// =====================================================
#define LED_BRIGHTNESS_NORMAL  80 // Lane LED brightness (0-255). DEFAULT: 80
#define DECK_LED_BRIGHTNESS    80 // Deck LED brightness (0-255). DEFAULT: 80
#define LED_BRIGHTNESS_STRIKE  40 // Reduced brightness during strike animation (0-255). DEFAULT: 40

// =====================================================
// SWEEP SERVO ANGLES
// Left and Right servos are mirrored (R = 180 - L)
// =====================================================
#define SWEEP_GUARD_ANGLE   50    // Guard position.  DEFAULT: 50
#define SWEEP_UP_ANGLE      85    // Up position.     DEFAULT: 85
#define SWEEP_BACK_ANGLE    0     // Back position.   DEFAULT: 0
// Tweening is what slows the sweep down, instead of moving fast from position to position.
// This is the duration it will spend to move between different positions, in ms.
// Bigger number is slower, smaller number is faster. Default is half a second, 500.
#define SWEEP_TWEEN_MS      500   // Tween duration.  DEFAULT: 500

// =====================================================
// BALL DOOR SERVO ANGLES
// =====================================================
#define BALL_DOOR_OPEN_ANGLE    180  // DEFAULT: 180
#define BALL_DOOR_CLOSED_ANGLE  0    // DEFAULT: 0

// =====================================================
// SCISSOR SERVO ANGLES
// =====================================================
#define SCISSOR_GRAB_ANGLE  140   // DEFAULT: 140
#define SCISSOR_DROP_ANGLE  90    // DEFAULT: 90

// =====================================================
// SLIDING DECK SERVO ANGLES
// =====================================================
#define SLIDER_HOME_ANGLE       180  // Home / Catch position. DEFAULT: 180
#define SLIDER_RELEASE_ANGLE    100  // Release position. DEFAULT: 100

// =====================================================
// RAISE SERVO ANGLES
// Left and Right servos are mirrored (R = 180 - L)
// =====================================================
#define RAISE_UP_ANGLE          180  // Up position.       DEFAULT: 180
#define RAISE_DOWN_ANGLE        20   // Down/Set position. DEFAULT: 20
#define RAISE_GRAB_ANGLE        60   // Grab position.     DEFAULT: 60
#define RAISE_DROP_ANGLE        80   // Drop position.     DEFAULT: 80

// =====================================================
// TURRET STEPPER SETTINGS
// =====================================================
// Defaults are listed on settings that are commonly adjusted.
#define TURRET_NORMAL_MAXSPEED  650.0
#define TURRET_NORMAL_ACCEL     3000.0
#define TURRET_SPRING_MAXSPEED  300.0
#define TURRET_SPRING_ACCEL     1500.0

// Home position adjuster (fine-tune after homing).
// Less negative means CCW
#define TURRET_HOME_ADJUSTER    -63     // DEFAULT: -63

// Pin release offset for position 10
#define TURRET_PIN10_RELEASE_OFFSET -30

// Extra offset for empty-turret purge at boot
#define TURRET_EMPTY_EXTRA_OFFSET   -60 // DEFAULT: -60

// =====================================================
// TURRET PIN POSITIONS (steps from home)
// =====================================================
// These are the step positions for each pin slot
// Slot 0 is unused, slots 1-9 are pins, slot 10 is release
#define PIN_POS_0       0
#define PIN_POS_1       -133
#define PIN_POS_2       -267
#define PIN_POS_3       -400
#define PIN_POS_4       -667
#define PIN_POS_5       -800
#define PIN_POS_6       -933
#define PIN_POS_7       -1200
#define PIN_POS_8       -1333
#define PIN_POS_9       -1467
#define PIN_POS_10      -1600

// =====================================================
// INPUT DEBOUNCE TIME (milliseconds)
// =====================================================
#define DEBOUNCE_MS     50

// =====================================================
// TURRET TIMING (milliseconds)
// =====================================================
#define CATCH_DELAY_MS          800   // Pause after catching pin at slots 1-8. DEFAULT: 800
#define RELEASE_DWELL_MS        1000  // Dwell at release position for pins to fall. DEFAULT: 1000
#define RELEASE_FEED_ASSIST_MS  250   // Conveyor assist during release dwell. DEFAULT: 250
#define NINTH_SETTLE_MS         300   // Settle time after 9th pin caught. DEFAULT: 300

// =====================================================
// EVERYTHING-SPECIFIC TIMING (milliseconds)
// Used only by Everything.ino
// =====================================================
// Extra settle time for raise servos (added to each raise move)
#define DECK_EXTRA_SETTLE_MS    500   // DEFAULT: 500

// Strike animation timing
#define STRIKE_WIPE_MS          300   // DEFAULT: 300
#define STRIKE_FRAME_MS         15    // DEFAULT: 15
#define STRIKE_SWEEP_PAUSE_MS   1000  // DEFAULT: 1000

// Ball comet animation
#define BALL_COMET_MS           500   // DEFAULT: 500
#define BALL_COMET_FRAME_MS     15    // DEFAULT: 15

// Startup LED wipe speed
#define STARTUP_WIPE_MS_PER_STEP 5   // DEFAULT: 5

// Pause mode idle timeout
#define PAUSE_IDLE_MS           300000 // 5 minutes. DEFAULT: 300000

// Score interval (how often to poll serial/inputs)
#define SCORE_INTERVAL          5     // DEFAULT: 5

// Ball trigger timing
#define BALL_LOW_CONFIRM_US     1000  // Microseconds to confirm ball sensor LOW. DEFAULT: 1000
#define BALL_REARM_MS           300   // Rearm delay after ball trigger. DEFAULT: 300

// ScoreMore ball pulse duration
#define SCOREMORE_BALL_PULSE_MS 150   // DEFAULT: 150

// Ball return door timing
#define BR_CLOSE_AFTER_SWEEPBACK_MS 5000 // DEFAULT: 5000

// Conveyor idle-stall timeout
#define NO_CATCH_TIMEOUT_MS     30000 // DEFAULT: 30000

// Post-set resume delay after deck up
#define RESUME_AFTER_DECKUP_MS  2000  // DEFAULT: 2000

// Strike flash timing
#define FLASH_ON_MS             120   // DEFAULT: 120
#define FLASH_OFF_MS            120   // DEFAULT: 120
#define FLASH_COUNT             3     // DEFAULT: 3

// Ball comet length (pixels)
#define COMET_LEN               4     // DEFAULT: 4

// =====================================================
// MASTER_TEST-SPECIFIC SETTINGS
// Used only by Master_Test.ino
// =====================================================
// IR Flap detection (auto-disables IR monitoring on rapid toggling)
#define IR_FLAP_COUNT       5     // Toggles within window to trigger
#define IR_FLAP_WINDOW_MS   200   // Window in ms

// Scissor cycle timing
#define SCISSOR_CYCLE_MS    2000  // DEFAULT: 2000

// LED animation timing (Master_Test)
#define WIPE_MS             500
#define WIPE_FRAME_MS       15
#define RAINBOW_FRAME_MS    20
#define COMET_MS            500   // DEFAULT: 500
#define COMET_FRAME_MS      15    // DEFAULT: 15

// Pin drop sequence timing
#define PDROP_SETTLE_MS         500   // Pause for small servo moves. DEFAULT: 500
#define PDROP_RAISE_SETTLE_MS   1300  // Pause for raise servo full travel. DEFAULT: 1300
#define PDROP_DROP_MS           800   // Pause for pins to fall after slider release. DEFAULT: 800

// Frame LED blink interval
#define FRAME_BLINK_INTERVAL    500   // DEFAULT: 500

#endif // GENERAL_CONFIG_H
