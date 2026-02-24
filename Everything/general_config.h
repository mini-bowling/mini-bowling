// =====================================================
// USER CONFIGURATION - DEFAULT VALUES
// This is the distribution template with factory defaults.
// Copy to general_config.user.h and customize for your machine.
// =====================================================

// =====================================================
// SCOREMORE SERIAL BAUD RATE & CONFIG
// =====================================================
#ifndef SCOREMORE_BAUD
#define SCOREMORE_BAUD      9600  // DEFAULT: 9600
#endif
#ifndef SCOREMORE_USER
#define SCOREMORE_USER      0     // DEFAULT: 0, do not wait for Scoremore connect. Set to 1 to wait for Scoremore before intialization.
#endif


// =====================================================
// LED STRIP LENGTHS
// Left and right can be set independently if needed
// =====================================================
#ifndef DECK_LED_LENGTH_L
#define DECK_LED_LENGTH_L   11    // Left deck strip.  DEFAULT: 11
#endif
#ifndef DECK_LED_LENGTH_R
#define DECK_LED_LENGTH_R   11    // Right deck strip. DEFAULT: 11
#endif
#ifndef LANE_LED_LENGTH_L
#define LANE_LED_LENGTH_L   41    // Left lane strip.  DEFAULT: 41
#endif
#ifndef LANE_LED_LENGTH_R
#define LANE_LED_LENGTH_R   41    // Right lane strip. DEFAULT: 41
#endif

// =====================================================
// CONVEYOR SETTINGS
// =====================================================
// Set to 1 if relay is active HIGH, 0 if active LOW.
#ifndef CONVEYOR_ACTIVE_HIGH
#define CONVEYOR_ACTIVE_HIGH 1    // DEFAULT: 1
#endif

// =====================================================
// LED BRIGHTNESS (0-255)
// =====================================================
#ifndef LED_BRIGHTNESS_NORMAL
#define LED_BRIGHTNESS_NORMAL  80 // Lane LED brightness (0-255). DEFAULT: 80
#endif
#ifndef DECK_LED_BRIGHTNESS
#define DECK_LED_BRIGHTNESS    80 // Deck LED brightness (0-255). DEFAULT: 80
#endif
#ifndef LED_BRIGHTNESS_STRIKE
#define LED_BRIGHTNESS_STRIKE  40 // Reduced brightness during strike animation (0-255). DEFAULT: 40
#endif

// =====================================================
// SWEEP SERVO ANGLES
// Left and Right servos are mirrored (R = 180 - L)
// =====================================================
#ifndef SWEEP_GUARD_ANGLE
#define SWEEP_GUARD_ANGLE   50    // Guard position.  DEFAULT: 50
#endif
#ifndef SWEEP_UP_ANGLE
#define SWEEP_UP_ANGLE      85    // Up position.     DEFAULT: 85
#endif
#ifndef SWEEP_BACK_ANGLE
#define SWEEP_BACK_ANGLE    0     // Back position.   DEFAULT: 0
#endif
// Tweening is what slows the sweep down, instead of moving fast from position to position.
// This is the duration it will spend to move between different positions, in ms.
// Bigger number is slower, smaller number is faster. Default is half a second, 500.
#ifndef SWEEP_TWEEN_MS
#define SWEEP_TWEEN_MS      500   // Tween duration.  DEFAULT: 500
#endif

// =====================================================
// BALL DOOR SERVO ANGLES
// =====================================================
#ifndef BALL_DOOR_OPEN_ANGLE
#define BALL_DOOR_OPEN_ANGLE    180  // DEFAULT: 180
#endif
#ifndef BALL_DOOR_CLOSED_ANGLE
#define BALL_DOOR_CLOSED_ANGLE  0    // DEFAULT: 0
#endif

// =====================================================
// SCISSOR SERVO ANGLES
// =====================================================
#ifndef SCISSOR_GRAB_ANGLE
#define SCISSOR_GRAB_ANGLE  140   // DEFAULT: 140
#endif
#ifndef SCISSOR_DROP_ANGLE
#define SCISSOR_DROP_ANGLE  90    // DEFAULT: 90
#endif

// =====================================================
// SLIDING DECK SERVO ANGLES
// =====================================================
#ifndef SLIDER_HOME_ANGLE
#define SLIDER_HOME_ANGLE       180  // Home / Catch position. DEFAULT: 180
#endif
#ifndef SLIDER_RELEASE_ANGLE
#define SLIDER_RELEASE_ANGLE    100  // Release position. DEFAULT: 100
#endif

// =====================================================
// RAISE SERVO ANGLES
// Left and Right servos are mirrored (R = 180 - L)
// =====================================================
#ifndef RAISE_UP_ANGLE
#define RAISE_UP_ANGLE          180  // Up position.       DEFAULT: 180
#endif
#ifndef RAISE_DOWN_ANGLE
#define RAISE_DOWN_ANGLE        20   // Down/Set position. DEFAULT: 20
#endif
#ifndef RAISE_GRAB_ANGLE
#define RAISE_GRAB_ANGLE        60   // Grab position.     DEFAULT: 60
#endif
#ifndef RAISE_DROP_ANGLE
#define RAISE_DROP_ANGLE        80   // Drop position.     DEFAULT: 80
#endif

// =====================================================
// TURRET STEPPER SETTINGS
// =====================================================
// Defaults are listed on settings that are commonly adjusted.
#ifndef TURRET_NORMAL_MAXSPEED
#define TURRET_NORMAL_MAXSPEED  650.0
#endif
#ifndef TURRET_NORMAL_ACCEL
#define TURRET_NORMAL_ACCEL     3000.0
#endif
#ifndef TURRET_SPRING_MAXSPEED
#define TURRET_SPRING_MAXSPEED  300.0
#endif
#ifndef TURRET_SPRING_ACCEL
#define TURRET_SPRING_ACCEL     1500.0
#endif

// Home position adjuster (fine-tune after homing).
// Less negative means CCW
#ifndef TURRET_HOME_ADJUSTER
#define TURRET_HOME_ADJUSTER    -63     // DEFAULT: -63
#endif

// Pin release offset for position 10
#ifndef TURRET_PIN10_RELEASE_OFFSET
#define TURRET_PIN10_RELEASE_OFFSET -30
#endif

// Extra offset for empty-turret purge at boot
#ifndef TURRET_EMPTY_EXTRA_OFFSET
#define TURRET_EMPTY_EXTRA_OFFSET   -60 // DEFAULT: -60
#endif

// =====================================================
// TURRET PIN POSITIONS (steps from home)
// =====================================================
// These are the step positions for each pin slot
// Slot 0 is unused, slots 1-9 are pins, slot 10 is release
#ifndef PIN_POS_0
#define PIN_POS_0       0
#endif
#ifndef PIN_POS_1
#define PIN_POS_1       -133
#endif
#ifndef PIN_POS_2
#define PIN_POS_2       -267
#endif
#ifndef PIN_POS_3
#define PIN_POS_3       -400
#endif
#ifndef PIN_POS_4
#define PIN_POS_4       -667
#endif
#ifndef PIN_POS_5
#define PIN_POS_5       -800
#endif
#ifndef PIN_POS_6
#define PIN_POS_6       -933
#endif
#ifndef PIN_POS_7
#define PIN_POS_7       -1200
#endif
#ifndef PIN_POS_8
#define PIN_POS_8       -1333
#endif
#ifndef PIN_POS_9
#define PIN_POS_9       -1467
#endif
#ifndef PIN_POS_10
#define PIN_POS_10      -1600
#endif

// =====================================================
// INPUT DEBOUNCE TIME (milliseconds)
// =====================================================
#ifndef DEBOUNCE_MS
#define DEBOUNCE_MS     50
#endif

// =====================================================
// TURRET TIMING (milliseconds)
// =====================================================
#ifndef CATCH_DELAY_MS
#define CATCH_DELAY_MS          800   // Pause after catching pin at slots 1-8. DEFAULT: 800
#endif
#ifndef RELEASE_DWELL_MS
#define RELEASE_DWELL_MS        1000  // Dwell at release position for pins to fall. DEFAULT: 1000
#endif
#ifndef RELEASE_FEED_ASSIST_MS
#define RELEASE_FEED_ASSIST_MS  250   // Conveyor assist during release dwell. DEFAULT: 250
#endif
#ifndef NINTH_SETTLE_MS
#define NINTH_SETTLE_MS         300   // Settle time after 9th pin caught. DEFAULT: 300
#endif

// =====================================================
// EVERYTHING-SPECIFIC TIMING (milliseconds)
// Used only by Everything.ino
// =====================================================
// Extra settle time for raise servos (added to each raise move)
#ifndef DECK_EXTRA_SETTLE_MS
#define DECK_EXTRA_SETTLE_MS    500   // DEFAULT: 500
#endif

// Strike animation timing
#ifndef STRIKE_WIPE_MS
#define STRIKE_WIPE_MS          300   // DEFAULT: 300
#endif
#ifndef STRIKE_FRAME_MS
#define STRIKE_FRAME_MS         15    // DEFAULT: 15
#endif
#ifndef STRIKE_SWEEP_PAUSE_MS
#define STRIKE_SWEEP_PAUSE_MS   1000  // DEFAULT: 1000
#endif

// Ball comet animation
#ifndef BALL_COMET_MS
#define BALL_COMET_MS           500   // DEFAULT: 500
#endif
#ifndef BALL_COMET_FRAME_MS
#define BALL_COMET_FRAME_MS     15    // DEFAULT: 15
#endif

// Startup LED wipe speed
#ifndef STARTUP_WIPE_MS_PER_STEP
#define STARTUP_WIPE_MS_PER_STEP 5   // DEFAULT: 5
#endif

// Pause mode idle timeout
#ifndef PAUSE_IDLE_MS
#define PAUSE_IDLE_MS           300000 // 5 minutes. DEFAULT: 300000
#endif

// Score interval (how often to poll serial/inputs)
#ifndef SCORE_INTERVAL
#define SCORE_INTERVAL          5     // DEFAULT: 5
#endif

// Ball trigger timing
#ifndef BALL_LOW_CONFIRM_US
#define BALL_LOW_CONFIRM_US     1000  // Microseconds to confirm ball sensor LOW. DEFAULT: 1000
#endif
#ifndef BALL_REARM_MS
#define BALL_REARM_MS           300   // Rearm delay after ball trigger. DEFAULT: 300
#endif

// ScoreMore ball pulse duration
#ifndef SCOREMORE_BALL_PULSE_MS
#define SCOREMORE_BALL_PULSE_MS 150   // DEFAULT: 150
#endif

// Ball return door timing
#ifndef BR_CLOSE_AFTER_SWEEPBACK_MS
#define BR_CLOSE_AFTER_SWEEPBACK_MS 5000 // DEFAULT: 5000
#endif

// Conveyor idle-stall timeout
#ifndef NO_CATCH_TIMEOUT_MS
#define NO_CATCH_TIMEOUT_MS     30000 // DEFAULT: 30000
#endif

// Post-set resume delay after deck up
#ifndef RESUME_AFTER_DECKUP_MS
#define RESUME_AFTER_DECKUP_MS  2000  // DEFAULT: 2000
#endif

// Strike flash timing
#ifndef FLASH_ON_MS
#define FLASH_ON_MS             120   // DEFAULT: 120
#endif
#ifndef FLASH_OFF_MS
#define FLASH_OFF_MS            120   // DEFAULT: 120
#endif
#ifndef FLASH_COUNT
#define FLASH_COUNT             3     // DEFAULT: 3
#endif

// Ball comet length (pixels)
#ifndef COMET_LEN
#define COMET_LEN               4     // DEFAULT: 4
#endif

// =====================================================
// MASTER_TEST-SPECIFIC SETTINGS
// Used only by Master_Test.ino
// =====================================================
// IR Flap detection (auto-disables IR monitoring on rapid toggling)
#ifndef IR_FLAP_COUNT
#define IR_FLAP_COUNT       5     // Toggles within window to trigger
#endif
#ifndef IR_FLAP_WINDOW_MS
#define IR_FLAP_WINDOW_MS   200   // Window in ms
#endif

// Scissor cycle timing
#ifndef SCISSOR_CYCLE_MS
#define SCISSOR_CYCLE_MS    2000  // DEFAULT: 2000
#endif

// LED animation timing (Master_Test)
#ifndef WIPE_MS
#define WIPE_MS             500
#endif
#ifndef WIPE_FRAME_MS
#define WIPE_FRAME_MS       15
#endif
#ifndef RAINBOW_FRAME_MS
#define RAINBOW_FRAME_MS    20
#endif
#ifndef COMET_MS
#define COMET_MS            500   // DEFAULT: 500
#endif
#ifndef COMET_FRAME_MS
#define COMET_FRAME_MS      15    // DEFAULT: 15
#endif

// Pin drop sequence timing
#ifndef PDROP_SETTLE_MS
#define PDROP_SETTLE_MS         500   // Pause for small servo moves. DEFAULT: 500
#endif
#ifndef PDROP_RAISE_SETTLE_MS
#define PDROP_RAISE_SETTLE_MS   1300  // Pause for raise servo full travel. DEFAULT: 1300
#endif
#ifndef PDROP_DROP_MS
#define PDROP_DROP_MS           800   // Pause for pins to fall after slider release. DEFAULT: 800
#endif

// Frame LED blink interval
#ifndef FRAME_BLINK_INTERVAL
#define FRAME_BLINK_INTERVAL    500   // DEFAULT: 500
#endif
