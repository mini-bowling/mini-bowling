// =====================================================
// PIN CONFIGURATION - DEFAULT VALUES
// All hardware pin assignments for the pinsetter
//
// To create a user-specific override, copy this file
// to "pin_config.user.h" in the same directory.
// That file will be used instead and is ignored by git.
// =====================================================

// =====================================================
// STEPPER MOTOR PINS
// =====================================================
#define STEP_PIN        2
#define DIR_PIN         3
// #define STEPPER_ENABLE_PIN  22  // Uncomment if your driver has an enable pin

// =====================================================
// RELAY PIN
// =====================================================
#define MOTOR_RELAY_PIN 4

// =====================================================
// SENSOR PINS (Inputs - continuously monitored)
// =====================================================
#define IR_SENSOR_PIN       5   // Pin/Cross-conveyor IR sensor
#define HALL_EFFECT_PIN     6   // Hall effect sensor for turret homing
#define BALL_SENSOR_PIN     A0  // Ball detection sensor (trigger)
#define BALL_SPEED_PIN      A1  // Ball speed sensor

// =====================================================
// SERVO PINS
// =====================================================
#define SCISSOR_PIN         7
#define SLIDE_PIN           8
#define RAISE_LEFT_PIN      9
#define RAISE_RIGHT_PIN     10
#define LEFT_SWEEP_PIN      11
#define RIGHT_SWEEP_PIN     12
#define BALL_RETURN_PIN     13

// =====================================================
// PINSETTER RESET PIN
// =====================================================
#define PINSETTER_RESET_PIN 41

// =====================================================
// FRAME INDICATOR LED PINS
// =====================================================
#define FRAME_LED1_PIN      46
#define FRAME_LED2_PIN      47

// =====================================================
// NEOPIXEL LED STRIP PINS
// =====================================================
#define DECK_PIN_L          50  // Left deck
#define DECK_PIN_R          51  // Right deck
#define LANE_PIN_L          52  // Left lane
#define LANE_PIN_R          53  // Right lane

// =====================================================
// SCOREMORE LOGICAL PIN MAPPING
// =====================================================
#define SM_PIN_1            14
#define SM_PIN_2            2
#define SM_PIN_3            3
#define SM_PIN_4            4
#define SM_PIN_5            5
#define SM_PIN_6            15
#define SM_PIN_7            16
#define SM_PIN_8            17
#define SM_PIN_9            18
#define SM_PIN_10           19
#define SM_BALL_TRIGGER     6
#define SM_AUTO_RESET       7
#define SM_SPEED_SENSOR     8
#define SM_SPARE_LIGHT      9
#define SM_STRIKE_LIGHT     10
#define SM_FIRST_BALL       11
#define SM_SECOND_BALL      12
#define SM_PINSETTER_RESET  20
