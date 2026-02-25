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
#ifndef STEP_PIN
#define STEP_PIN        2
#endif
#ifndef DIR_PIN
#define DIR_PIN         3
#endif
// Uncomment STEPPER_ENABLE_PIN if stepper uses an enable pin 
// (required for user reset button w/ maintenance mode)
#ifndef STEPPER_ENABLE_PIN      
#define STEPPER_ENABLE_PIN  49  
#endif                         

// =====================================================
// RELAY PIN
// =====================================================
#ifndef MOTOR_RELAY_PIN
#define MOTOR_RELAY_PIN 4
#endif

// =====================================================
// SENSOR PINS (Inputs - continuously monitored)
// =====================================================
#ifndef IR_SENSOR_PIN
#define IR_SENSOR_PIN       5   // Pin/Cross-conveyor IR sensor
#endif
#ifndef HALL_EFFECT_PIN
#define HALL_EFFECT_PIN     6   // Hall effect sensor for turret homing
#endif
#ifndef BALL_SENSOR_PIN
#define BALL_SENSOR_PIN     A0  // Ball detection sensor (trigger)
#endif
#ifndef BALL_SPEED_PIN
#define BALL_SPEED_PIN      A1  // Ball speed sensor
#endif

// =====================================================
// SERVO PINS
// =====================================================
#ifndef SCISSOR_PIN
#define SCISSOR_PIN         7
#endif
#ifndef SLIDE_PIN
#define SLIDE_PIN           8
#endif
#ifndef RAISE_LEFT_PIN
#define RAISE_LEFT_PIN      9
#endif
#ifndef RAISE_RIGHT_PIN
#define RAISE_RIGHT_PIN     10
#endif
#ifndef LEFT_SWEEP_PIN
#define LEFT_SWEEP_PIN      11
#endif
#ifndef RIGHT_SWEEP_PIN
#define RIGHT_SWEEP_PIN     12
#endif
#ifndef BALL_RETURN_PIN
#define BALL_RETURN_PIN     13
#endif

// =====================================================
// PINSETTER RESET (BUTTON) PIN
// =====================================================
#ifndef PINSETTER_RESET_PIN
#define PINSETTER_RESET_PIN 41
#endif

// =====================================================
// FRAME INDICATOR LED PINS
// =====================================================
#ifndef FRAME_LED1_PIN
#define FRAME_LED1_PIN      46
#endif
#ifndef FRAME_LED2_PIN
#define FRAME_LED2_PIN      47
#endif

// =====================================================
// NEOPIXEL LED STRIP PINS
// =====================================================
#ifndef DECK_PIN_L
#define DECK_PIN_L          50  // Left deck
#endif
#ifndef DECK_PIN_R
#define DECK_PIN_R          51  // Right deck
#endif
#ifndef LANE_PIN_L
#define LANE_PIN_L          52  // Left lane
#endif
#ifndef LANE_PIN_R
#define LANE_PIN_R          53  // Right lane
#endif

// =====================================================
// SCOREMORE LOGICAL PIN MAPPING
// =====================================================
#ifndef SM_PIN_1
#define SM_PIN_1            14
#endif
#ifndef SM_PIN_2
#define SM_PIN_2            2
#endif
#ifndef SM_PIN_3
#define SM_PIN_3            3
#endif
#ifndef SM_PIN_4
#define SM_PIN_4            4
#endif
#ifndef SM_PIN_5
#define SM_PIN_5            5
#endif
#ifndef SM_PIN_6
#define SM_PIN_6            15
#endif
#ifndef SM_PIN_7
#define SM_PIN_7            16
#endif
#ifndef SM_PIN_8
#define SM_PIN_8            17
#endif
#ifndef SM_PIN_9
#define SM_PIN_9            18
#endif
#ifndef SM_PIN_10
#define SM_PIN_10           19
#endif
#ifndef SM_BALL_TRIGGER
#define SM_BALL_TRIGGER     6
#endif
#ifndef SM_AUTO_RESET
#define SM_AUTO_RESET       7
#endif
#ifndef SM_SPEED_SENSOR
#define SM_SPEED_SENSOR     8
#endif
#ifndef SM_SPARE_LIGHT
#define SM_SPARE_LIGHT      9
#endif
#ifndef SM_STRIKE_LIGHT
#define SM_STRIKE_LIGHT     10
#endif
#ifndef SM_FIRST_BALL
#define SM_FIRST_BALL       11
#endif
#ifndef SM_SECOND_BALL
#define SM_SECOND_BALL      12
#endif
#ifndef SM_PINSETTER_RESET
#define SM_PINSETTER_RESET  20
#endif
