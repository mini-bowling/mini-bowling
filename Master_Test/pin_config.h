// =====================================================
// PIN CONFIGURATION
// All hardware pin assignments for the pinsetter
// =====================================================

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

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
// FRAME INDICATOR LED PINS
// =====================================================
#define FRAME_LED1_PIN      46
#define FRAME_LED2_PIN      47

// =====================================================
// NEOPIXEL LED STRIP PINS
// =====================================================
#define DECK_PIN_A          50  // Left deck
#define DECK_PIN_B          51  // Right deck
#define LANE_PIN_A          52  // Left lane
#define LANE_PIN_B          53  // Right lane

#endif // PIN_CONFIG_H
