// =====================================================
// AutoBowl Pinsetter Firmware
// Version: 1.1
//
// Upload this firmware to your Arduino Mega 2560 using
// the Arduino IDE. Required libraries:
//   - Adafruit NeoPixel
//   - AccelStepper
//   - Servo (built-in)
// =====================================================

// =====================================================
// Pinsetter Deck + Turret Controller - Hardware Layer
//
// This firmware acts as a thin hardware abstraction layer.
// All game logic and state machine runs on the Go application.
// The Arduino receives commands and reports sensor events.
//
// Serial Protocol (115200 baud):
// Commands from Go:
//   SERVO:<id>:<angle>           - Set servo position (id=0-6, angle=0-180)
//   STEPPER:<position>           - Move stepper to absolute position
//   STEPPER_SPEED:<speed>:<accel> - Set stepper speed profile
//   HOME                         - Start homing sequence
//   CONVEYOR:<0|1>               - Conveyor relay on/off (legacy)
//   CONVEYOR_MODE:<off|on|low>   - Conveyor mode (low = pulse mode)
//   LED:<mode>                   - LED mode (off|white|red|green|strike_wipe|comet|flash)
//   FRAME_LED:<0|1>:<0|1>        - Frame indicator LEDs
//   PING                         - Heartbeat test
//   GET_STATE                    - Request full state report
//   GET_STEPPER                  - Request stepper status
//
// Events to Go:
//   READY                        - Boot complete (followed by initial sensor states)
//   SENSOR_BALL:<0|1>            - Ball sensor state/edge
//   SENSOR_IR:<0|1>              - IR beam state/edge (debounced)
//   SENSOR_HALL:<0|1>            - Hall sensor state/edge
//   STEPPER_HOMED                - Homing complete
//   STEPPER_DONE:<position>      - Stepper move complete with final position
//   STATE:ball=X,ir=X,hall=X,stepper=X,moving=X,homing=X - Full state
//   STEPPER_STATUS:pos=X,target=X,moving=X,homing=X      - Stepper status
//   PONG                         - Heartbeat response
// =====================================================

#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <AccelStepper.h>

// =============== SERIAL CONFIG =====================
#define SERIAL_BAUD 115200

// ======================= LOOP TIMING DEBUG =======================
#define DEBUG_LOOP_TIMING  // Comment out to disable
#ifdef DEBUG_LOOP_TIMING
unsigned long _loopMax=0, _loopMin=ULONG_MAX, _loopSum=0, _loopCount=0, _loopReportMs=0;
#endif

// =============== LED CONFIG =====================
#define DECK_PIN_A   50  // LEFT
#define DECK_PIN_B   51  // RIGHT
#define LANE_PIN_A   52  // LEFT
#define LANE_PIN_B   53  // RIGHT

#define DECK_LEN1    11
#define DECK_LEN2    11
#define LANE_LEN1    41
#define LANE_LEN2    41

#define LED_BRIGHTNESS_NORMAL  80
#define LED_BRIGHTNESS_STRIKE  40

Adafruit_NeoPixel deckA(DECK_LEN1, DECK_PIN_A, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel deckB(DECK_LEN2, DECK_PIN_B, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel laneA(LANE_LEN1, LANE_PIN_A, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel laneB(LANE_LEN2, LANE_PIN_B, NEO_GRB + NEO_KHZ800);

static inline uint32_t C_WHITE(Adafruit_NeoPixel &s){ return s.Color(255,255,255); }
static inline uint32_t C_RED  (Adafruit_NeoPixel &s){ return s.Color(255,  0,  0); }
static inline uint32_t C_GREEN(Adafruit_NeoPixel &s){ return s.Color(  0,255,  0); }
static inline uint32_t C_OFF  (Adafruit_NeoPixel &s){ return s.Color(  0,  0,  0); }

// --- LED animation state ---
enum LaneAnimMode { LANE_IDLE=0, LANE_STRIKE_WIPE, LANE_STRIKE_FLASH, LANE_BALL_COMET };
LaneAnimMode laneMode = LANE_IDLE;

// --- Strike timings ---
const unsigned long STRIKE_WIPE_MS = 300;
const unsigned long FLASH_ON_MS = 120;
const unsigned long FLASH_OFF_MS = 120;
const int FLASH_COUNT = 3;
const unsigned long STRIKE_FRAME_MS = 15;

unsigned long strikeWipeStartMs = 0;
unsigned long strikeLastFrameMs = 0;
unsigned long flashLastMs = 0;
bool flashOnPhase = false;
int flashCycles = 0;

// --- Ball comet timings ---
const unsigned long BALL_COMET_MS = 500;
const unsigned long BALL_COMET_FRAME_MS = 15;
const int COMET_LEN = 4;
unsigned long ballCometStartMs = 0;
unsigned long ballCometLastFrame = 0;

// ======================= HW PINS =======================
#define MOTOR_RELAY      4
#define IR_SENSOR        5
#define HALL_EFFECT      6
#define SCISSOR_PIN      7
#define SLIDE_PIN        8
#define RAISE_LEFT_PIN   9
#define RAISE_RIGHT_PIN 10
#define LEFT_SWEEP_PIN  11
#define RIGHT_SWEEP_PIN 12
#define BALL_RETURN_PIN 13
#define BALL_SENSOR_PIN A0
#define CONVEYOR_ACTIVE_HIGH 1

// Frame LEDs
#define FRAME_LED1 46
#define FRAME_LED2 47

// ======================= CONVEYOR PULSE MODE =======================
// For simulating slower conveyor speed with relay hardware
enum ConveyorMode { CONV_OFF = 0, CONV_ON, CONV_LOW };
ConveyorMode conveyorMode = CONV_OFF;
bool conveyorPulseState = false;
unsigned long conveyorPulseLastToggle = 0;
const unsigned long CONVEYOR_PULSE_ON_MS = 800;   // 80% duty cycle
const unsigned long CONVEYOR_PULSE_OFF_MS = 200;

// ======================= SERVOS =======================
Servo LeftRaiseServo, RightRaiseServo, SlideServo, ScissorsServo;
Servo LeftSweepServo, RightSweepServo, BallReturnServo;

// ======================= STEPPER =======================
AccelStepper stepper1(1, 2, 3);  // Driver, STEP pin 2, DIR pin 3

// Stepper speed profiles
float stepperMaxSpeed = 650.0;
float stepperAccel = 3000.0;

// Homing state
bool homingActive = false;
enum HomingPhase {
  HOME_IDLE = 0,
  HOME_PREP_MOVE_TO_SLOT1,
  HOME_ADVANCE_TO_SWITCH,
  HOME_BACKOFF,
  HOME_CREEP_TO_SWITCH,
  HOME_SETZERO_AND_MOVE_SLOT1,
  HOME_DONE
};
HomingPhase homingPhase = HOME_IDLE;
int HomeAdjuster = -63;

// Pin positions (from original code)
const int PinPositions[] = {0, -133, -267, -400, -667, -800, -933, -1200, -1333, -1467, -1600};

// Stepper movement tracking
long targetPos = 0;
bool stepperMoving = false;

// ======================= SENSOR STATE =======================
// Ball sensor
bool lastBallState = false;
const unsigned long BALL_DEBOUNCE_US = 1000;
unsigned long ballLowStartUs = 0;
bool ballPending = false;

// IR sensor (debounced)
#define IR_DEBOUNCE_MS 50
int irStableState = HIGH;
int irLastRead = HIGH;
unsigned long irLastChange = 0;

// Hall sensor
bool lastHallState = false;

// Sensor polling interval
const unsigned long SENSOR_POLL_MS = 5;
unsigned long lastSensorPoll = 0;

// ======================= SERVO/LED TIMING =======================
// Track last servo command to avoid NeoPixel interrupt conflicts
unsigned long lastServoCommandTime = 0;
const unsigned long SERVO_SETTLE_MS = 50;

// ======================= SWEEP TWEEN =======================
int sweepStartL = 0, sweepStartR = 0;
int sweepTargetL = 0, sweepTargetR = 0;
int sweepCurL = 0, sweepCurR = 0;
unsigned long sweepStartMs = 0;
unsigned long sweepDurationMs = 500;
bool sweepAnimating = false;

// ======================= FUNCTION DECLARATIONS =======================
void checkSerial();
void handleCommand(String cmd);
void pollAndReportSensors();
void laneUpdate();
void updateSweepTween();
void runHomingFSM();

// LED functions
void ledsBegin();
void deckAll(uint32_t col);
void laneAll(uint32_t col);
void ledsShowAll();
void laneShowOnly();
void startStrikeWipe();
void startStrikeFlash();
void startBallCometImmediate();
void endAllLaneAnimsToWhite();

// State reporting
void reportInitialSensorStates();

// Sweep tween
static int clampInt(int v, int lo, int hi);
void setSweepInstant(int leftDeg, int rightDeg);
void startSweepTo(int leftDeg, int rightDeg, unsigned long durMs);

// ======================= SETUP =======================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  // Initialize LEDs
  ledsBegin();

  // Frame LEDs
  pinMode(FRAME_LED1, OUTPUT);
  pinMode(FRAME_LED2, OUTPUT);
  digitalWrite(FRAME_LED1, LOW);
  digitalWrite(FRAME_LED2, LOW);

  // Ball sensor
  pinMode(BALL_SENSOR_PIN, INPUT_PULLUP);

  // Stepper
  stepper1.setMaxSpeed(stepperMaxSpeed);
  stepper1.setAcceleration(stepperAccel);

  // Servos
  LeftRaiseServo.attach(RAISE_LEFT_PIN);
  RightRaiseServo.attach(RAISE_RIGHT_PIN);
  SlideServo.attach(SLIDE_PIN);
  ScissorsServo.attach(SCISSOR_PIN);
  LeftSweepServo.attach(LEFT_SWEEP_PIN);
  RightSweepServo.attach(RIGHT_SWEEP_PIN);
  BallReturnServo.attach(BALL_RETURN_PIN);

  // Conveyor relay
  pinMode(MOTOR_RELAY, OUTPUT);
  digitalWrite(MOTOR_RELAY, CONVEYOR_ACTIVE_HIGH ? LOW : HIGH);  // Start OFF

  // Sensors
  pinMode(IR_SENSOR, INPUT);
  pinMode(HALL_EFFECT, INPUT_PULLUP);

  // Initialize sensor states
  lastBallState = digitalRead(BALL_SENSOR_PIN) == LOW;
  irLastRead = digitalRead(IR_SENSOR);
  irStableState = irLastRead;
  irLastChange = millis();
  lastHallState = digitalRead(HALL_EFFECT) == LOW;

  // Set LEDs to white (idle)
  deckAll(C_WHITE(deckA));
  laneAll(C_WHITE(laneA));
  ledsShowAll();

  Serial.println("READY");

  // Report initial sensor states so Go knows current state at boot
  reportInitialSensorStates();
}

// ======================= INITIAL STATE REPORTING =======================
void reportInitialSensorStates() {
  Serial.print("SENSOR_BALL:");
  Serial.println(lastBallState ? "1" : "0");
  Serial.print("SENSOR_IR:");
  Serial.println(irStableState == LOW ? "1" : "0");
  Serial.print("SENSOR_HALL:");
  Serial.println(lastHallState ? "1" : "0");
}

// ======================= LOOP =======================
void loop() {
#ifdef DEBUG_LOOP_TIMING
  unsigned long _loopStart=micros();
#endif
  unsigned long now = millis();

  // Process serial commands
  checkSerial();

  // Poll sensors at fixed interval
  if (now - lastSensorPoll >= SENSOR_POLL_MS) {
    lastSensorPoll = now;
    pollAndReportSensors();
  }

  // Update sweep tween animation
  updateSweepTween();

  // Update LED animations
  laneUpdate();

  // Run stepper
  stepper1.run();

  // Check if stepper finished moving
  if (stepperMoving && stepper1.distanceToGo() == 0) {
    stepperMoving = false;
    Serial.print("STEPPER_DONE:");
    Serial.println(stepper1.currentPosition());
  }

  // Run homing FSM
  runHomingFSM();

  // Conveyor pulse mode (simulates low speed via PWM-like pulsing)
  if (conveyorMode == CONV_LOW) {
    unsigned long duration = conveyorPulseState ? CONVEYOR_PULSE_ON_MS : CONVEYOR_PULSE_OFF_MS;
    if (now - conveyorPulseLastToggle >= duration) {
      conveyorPulseState = !conveyorPulseState;
      digitalWrite(MOTOR_RELAY, (conveyorPulseState && CONVEYOR_ACTIVE_HIGH) || (!conveyorPulseState && !CONVEYOR_ACTIVE_HIGH) ? HIGH : LOW);
      conveyorPulseLastToggle = now;
    }
  }

#ifdef DEBUG_LOOP_TIMING
  unsigned long _loopTime=micros()-_loopStart;
  if(_loopTime>_loopMax) _loopMax=_loopTime;
  if(_loopTime<_loopMin) _loopMin=_loopTime;
  _loopSum+=_loopTime; _loopCount++;
  if(millis()-_loopReportMs>=5000){
    _loopReportMs=millis();
    Serial.print("DEBUG:LOOP_US max="); Serial.print(_loopMax);
    Serial.print(" min="); Serial.print(_loopMin);
    Serial.print(" avg="); Serial.print(_loopCount?_loopSum/_loopCount:0);
    Serial.print(" n="); Serial.println(_loopCount);
    _loopMax=0; _loopMin=ULONG_MAX; _loopSum=0; _loopCount=0;
  }
#endif
}

// ======================= SENSOR POLLING =======================
void pollAndReportSensors() {
  // Ball sensor - edge detection with debounce
  int rawBall = digitalRead(BALL_SENSOR_PIN);
  bool currentBall = (rawBall == LOW);

  if (currentBall && !lastBallState) {
    // Rising edge on ball present
    ballPending = true;
    ballLowStartUs = micros();
  }

  if (ballPending) {
    if (currentBall) {
      if ((micros() - ballLowStartUs) >= BALL_DEBOUNCE_US) {
        // Debounced - report ball present
        Serial.println("SENSOR_BALL:1");
        lastBallState = true;
        ballPending = false;
      }
    } else {
      ballPending = false;
    }
  }

  if (!currentBall && lastBallState) {
    // Ball no longer present
    Serial.println("SENSOR_BALL:0");
    lastBallState = false;
  }

  // IR sensor - debounced edge detection
  int irRaw = digitalRead(IR_SENSOR);
  if (irRaw != irLastRead) {
    irLastChange = millis();
    irLastRead = irRaw;
  }

  if ((millis() - irLastChange) > IR_DEBOUNCE_MS) {
    if (irStableState != irRaw) {
      irStableState = irRaw;
      // Report edge: 1 = blocked (LOW), 0 = clear (HIGH)
      Serial.print("SENSOR_IR:");
      Serial.println(irStableState == LOW ? "1" : "0");
    }
  }

  // Hall sensor - edge detection
  bool currentHall = (digitalRead(HALL_EFFECT) == LOW);
  if (currentHall != lastHallState) {
    Serial.print("SENSOR_HALL:");
    Serial.println(currentHall ? "1" : "0");
    lastHallState = currentHall;
  }
}

// ======================= SERIAL COMMANDS =======================
void checkSerial() {
  static String input = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      input.trim();
      if (input.length() > 0) handleCommand(input);
      input = "";
    } else {
      input += c;
    }
  }
}

void handleCommand(String cmd) {
  cmd.trim();

  // PING - heartbeat test
  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  // GET_STATE - report all sensor states for resync
  if (cmd == "GET_STATE") {
    Serial.print("STATE:ball=");
    Serial.print(lastBallState ? "1" : "0");
    Serial.print(",ir=");
    Serial.print(irStableState == LOW ? "1" : "0");
    Serial.print(",hall=");
    Serial.print(lastHallState ? "1" : "0");
    Serial.print(",stepper=");
    Serial.print(stepper1.currentPosition());
    Serial.print(",moving=");
    Serial.print(stepperMoving ? "1" : "0");
    Serial.print(",homing=");
    Serial.println(homingActive ? "1" : "0");
    return;
  }

  // GET_STEPPER - report stepper status
  if (cmd == "GET_STEPPER") {
    Serial.print("STEPPER_STATUS:pos=");
    Serial.print(stepper1.currentPosition());
    Serial.print(",target=");
    Serial.print(targetPos);
    Serial.print(",moving=");
    Serial.print(stepperMoving ? "1" : "0");
    Serial.print(",homing=");
    Serial.println(homingActive ? "1" : "0");
    return;
  }

  // SERVO:<id>:<angle>
  if (cmd.startsWith("SERVO:")) {
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    if (secondColon > firstColon) {
      int servoId = cmd.substring(firstColon + 1, secondColon).toInt();
      int angle = cmd.substring(secondColon + 1).toInt();

      // Clamp angle
      if (angle < 0) angle = 0;
      if (angle > 180) angle = 180;

      switch (servoId) {
        case 0: LeftRaiseServo.write(angle); break;
        case 1: RightRaiseServo.write(angle); break;
        case 2: SlideServo.write(angle); break;
        case 3: ScissorsServo.write(angle); break;
        case 4: LeftSweepServo.write(angle); break;
        case 5: RightSweepServo.write(angle); break;
        case 6: BallReturnServo.write(angle); break;
        default:
          Serial.print("ERROR:Invalid servo ID:");
          Serial.println(servoId);
          return;
      }

      Serial.print("ACK_SERVO:");
      Serial.print(servoId);
      Serial.print(":");
      Serial.println(angle);

      // Track servo command time to avoid LED update conflicts
      lastServoCommandTime = millis();
    }
    return;
  }

  // STEPPER:<position>
  if (cmd.startsWith("STEPPER:")) {
    long position = cmd.substring(8).toInt();
    stepper1.moveTo(position);
    targetPos = position;
    stepperMoving = true;
    Serial.print("ACK_STEPPER:");
    Serial.println(position);
    return;
  }

  // STEPPER_SPEED:<speed>:<accel>
  if (cmd.startsWith("STEPPER_SPEED:")) {
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    if (secondColon > firstColon) {
      float speed = cmd.substring(firstColon + 1, secondColon).toFloat();
      float accel = cmd.substring(secondColon + 1).toFloat();

      stepperMaxSpeed = speed;
      stepperAccel = accel;
      stepper1.setMaxSpeed(speed);
      stepper1.setAcceleration(accel);

      Serial.print("ACK_STEPPER_SPEED:");
      Serial.print((int)speed);
      Serial.print(":");
      Serial.println((int)accel);
    }
    return;
  }

  // HOME
  if (cmd == "HOME") {
    homingActive = true;
    homingPhase = HOME_PREP_MOVE_TO_SLOT1;
    stepper1.setAcceleration(3000);
    stepper1.setMaxSpeed(500);
    stepper1.moveTo(PinPositions[1]);
    Serial.println("ACK_HOME");
    return;
  }

  // CONVEYOR:<0|1> - legacy binary control
  if (cmd.startsWith("CONVEYOR:") && !cmd.startsWith("CONVEYOR_MODE:")) {
    int value = cmd.substring(9).toInt();
    conveyorMode = value ? CONV_ON : CONV_OFF;
    digitalWrite(MOTOR_RELAY, (value && CONVEYOR_ACTIVE_HIGH) || (!value && !CONVEYOR_ACTIVE_HIGH) ? HIGH : LOW);
    Serial.print("ACK_CONVEYOR:");
    Serial.println(value);
    return;
  }

  // CONVEYOR_MODE:<off|on|low> - extended control with pulse mode
  if (cmd.startsWith("CONVEYOR_MODE:")) {
    String mode = cmd.substring(14);
    mode.toLowerCase();

    if (mode == "off") {
      conveyorMode = CONV_OFF;
      digitalWrite(MOTOR_RELAY, CONVEYOR_ACTIVE_HIGH ? LOW : HIGH);
    } else if (mode == "on") {
      conveyorMode = CONV_ON;
      digitalWrite(MOTOR_RELAY, CONVEYOR_ACTIVE_HIGH ? HIGH : LOW);
    } else if (mode == "low") {
      conveyorMode = CONV_LOW;
      conveyorPulseState = true;
      conveyorPulseLastToggle = millis();
      digitalWrite(MOTOR_RELAY, CONVEYOR_ACTIVE_HIGH ? HIGH : LOW);
    } else {
      Serial.print("ERROR:Unknown conveyor mode:");
      Serial.println(mode);
      return;
    }

    Serial.print("ACK_CONVEYOR_MODE:");
    Serial.println(mode);
    return;
  }

  // LED:<mode>
  if (cmd.startsWith("LED:")) {
    String mode = cmd.substring(4);
    mode.toLowerCase();

    if (mode == "off") {
      laneMode = LANE_IDLE;
      deckAll(C_OFF(deckA));
      laneAll(C_OFF(laneA));
      ledsShowAll();
    } else if (mode == "white") {
      laneMode = LANE_IDLE;
      laneA.setBrightness(LED_BRIGHTNESS_NORMAL);
      laneB.setBrightness(LED_BRIGHTNESS_NORMAL);
      deckAll(C_WHITE(deckA));
      laneAll(C_WHITE(laneA));
      ledsShowAll();
    } else if (mode == "red") {
      laneMode = LANE_IDLE;
      deckAll(C_RED(deckA));
      laneAll(C_RED(laneA));
      ledsShowAll();
    } else if (mode == "green") {
      laneMode = LANE_IDLE;
      // Green on lanes only (like fill ball mode)
      for (int i = 0; i < LANE_LEN1; i++) laneA.setPixelColor(i, C_GREEN(laneA));
      for (int i = 0; i < LANE_LEN2; i++) laneB.setPixelColor(i, C_GREEN(laneB));
      laneShowOnly();
    } else if (mode == "strike_wipe") {
      startStrikeWipe();
    } else if (mode == "comet") {
      startBallCometImmediate();
    } else if (mode == "flash") {
      startStrikeFlash();
    } else {
      Serial.print("ERROR:Unknown LED mode:");
      Serial.println(mode);
      return;
    }

    Serial.print("ACK_LED:");
    Serial.println(mode);
    return;
  }

  // FRAME_LED:<0|1>:<0|1>
  if (cmd.startsWith("FRAME_LED:")) {
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    if (secondColon > firstColon) {
      int led1 = cmd.substring(firstColon + 1, secondColon).toInt();
      int led2 = cmd.substring(secondColon + 1).toInt();

      digitalWrite(FRAME_LED1, led1 ? HIGH : LOW);
      digitalWrite(FRAME_LED2, led2 ? HIGH : LOW);

      Serial.print("ACK_FRAME_LED:");
      Serial.print(led1);
      Serial.print(":");
      Serial.println(led2);
    }
    return;
  }

  // SWEEP_TWEEN:<leftDeg>:<rightDeg>:<durationMs>
  if (cmd.startsWith("SWEEP_TWEEN:")) {
    int c1 = cmd.indexOf(':');
    int c2 = cmd.indexOf(':', c1 + 1);
    int c3 = cmd.indexOf(':', c2 + 1);
    if (c2 > c1 && c3 > c2) {
      int leftDeg = cmd.substring(c1 + 1, c2).toInt();
      int rightDeg = cmd.substring(c2 + 1, c3).toInt();
      unsigned long durMs = cmd.substring(c3 + 1).toInt();
      startSweepTo(leftDeg, rightDeg, durMs);
      Serial.print("ACK_SWEEP_TWEEN:");
      Serial.print(leftDeg);
      Serial.print(":");
      Serial.print(rightDeg);
      Serial.print(":");
      Serial.println(durMs);
    }
    return;
  }

  // Unknown command
  Serial.print("ERROR:Unknown command:");
  Serial.println(cmd);
}

// ======================= HOMING FSM =======================
void runHomingFSM() {
  if (!homingActive) return;

  switch (homingPhase) {
    case HOME_PREP_MOVE_TO_SLOT1:
      if (stepper1.distanceToGo() == 0) {
        homingPhase = HOME_ADVANCE_TO_SWITCH;
      }
      break;

    case HOME_ADVANCE_TO_SWITCH:
      if (digitalRead(HALL_EFFECT) == HIGH) {
        if (stepper1.distanceToGo() == 0) {
          stepper1.moveTo(stepper1.currentPosition() + 10);
        }
      } else {
        homingPhase = HOME_BACKOFF;
        stepper1.moveTo(stepper1.currentPosition() - 150);
      }
      break;

    case HOME_BACKOFF:
      if (stepper1.distanceToGo() == 0) {
        stepper1.setMaxSpeed(100);
        homingPhase = HOME_CREEP_TO_SWITCH;
      }
      break;

    case HOME_CREEP_TO_SWITCH:
      if (digitalRead(HALL_EFFECT) == HIGH) {
        if (stepper1.distanceToGo() == 0) {
          stepper1.moveTo(stepper1.currentPosition() + 2);
        }
      } else {
        stepper1.setCurrentPosition(HomeAdjuster);
        stepper1.setMaxSpeed(500);
        stepper1.setAcceleration(3000);
        stepper1.moveTo(PinPositions[1]);
        homingPhase = HOME_SETZERO_AND_MOVE_SLOT1;
      }
      break;

    case HOME_SETZERO_AND_MOVE_SLOT1:
      if (stepper1.distanceToGo() == 0) {
        homingPhase = HOME_DONE;
      }
      break;

    case HOME_DONE:
      homingActive = false;
      homingPhase = HOME_IDLE;
      stepper1.setMaxSpeed(stepperMaxSpeed);
      stepper1.setAcceleration(stepperAccel);
      Serial.println("STEPPER_HOMED");
      break;

    default:
      break;
  }
}

// ======================= SWEEP TWEEN =======================
static int clampInt(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

void setSweepInstant(int leftDeg, int rightDeg) {
  leftDeg = clampInt(leftDeg, 0, 180);
  rightDeg = clampInt(rightDeg, 0, 180);
  sweepCurL = sweepStartL = sweepTargetL = leftDeg;
  sweepCurR = sweepStartR = sweepTargetR = rightDeg;
  sweepAnimating = false;
  LeftSweepServo.write(leftDeg);
  RightSweepServo.write(rightDeg);
}

void startSweepTo(int leftDeg, int rightDeg, unsigned long durMs) {
  leftDeg = clampInt(leftDeg, 0, 180);
  rightDeg = clampInt(rightDeg, 0, 180);
  sweepStartL = sweepCurL;
  sweepStartR = sweepCurR;
  sweepTargetL = leftDeg;
  sweepTargetR = rightDeg;
  sweepStartMs = millis();
  sweepDurationMs = (durMs == 0) ? 1UL : durMs;
  sweepAnimating = !((sweepStartL == sweepTargetL) && (sweepStartR == sweepTargetR));
  if (!sweepAnimating) {
    LeftSweepServo.write(sweepTargetL);
    RightSweepServo.write(sweepTargetR);
  }
}

void updateSweepTween() {
  if (!sweepAnimating) return;

  unsigned long now = millis();
  unsigned long elapsed = now - sweepStartMs;

  if (elapsed >= sweepDurationMs) {
    sweepCurL = sweepTargetL;
    sweepCurR = sweepTargetR;
    LeftSweepServo.write(sweepCurL);
    RightSweepServo.write(sweepCurR);
    sweepAnimating = false;
    Serial.println("SWEEP_DONE");
    return;
  }

  float t = (float)elapsed / (float)sweepDurationMs;
  int newL = sweepStartL + (int)((sweepTargetL - sweepStartL) * t + (t >= 0 ? 0.5f : -0.5f));
  int newR = sweepStartR + (int)((sweepTargetR - sweepStartR) * t + (t >= 0 ? 0.5f : -0.5f));

  if (newL != sweepCurL) {
    sweepCurL = newL;
    LeftSweepServo.write(sweepCurL);
  }
  if (newR != sweepCurR) {
    sweepCurR = newR;
    RightSweepServo.write(sweepCurR);
  }
}

// ======================= LED IMPL =======================
void ledsBegin() {
  deckA.begin();
  deckB.begin();
  laneA.begin();
  laneB.begin();
  deckA.setBrightness(LED_BRIGHTNESS_NORMAL);
  deckB.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneA.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneB.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneMode = LANE_IDLE;
}

void deckAll(uint32_t col) {
  for (int i = 0; i < DECK_LEN1; i++) deckA.setPixelColor(i, col);
  for (int i = 0; i < DECK_LEN2; i++) deckB.setPixelColor(i, col);
}

void laneAll(uint32_t col) {
  for (int i = 0; i < LANE_LEN1; i++) laneA.setPixelColor(i, col);
  for (int i = 0; i < LANE_LEN2; i++) laneB.setPixelColor(i, col);
}

void ledsShowAll() {
  // Process any pending serial data before disabling interrupts
  checkSerial();
  deckA.show();
  deckB.show();
  laneA.show();
  laneB.show();
  // Small delay then process any data that arrived during show()
  delayMicroseconds(500);
  checkSerial();
}

void laneShowOnly() {
  // Process any pending serial data before disabling interrupts
  checkSerial();
  laneA.show();
  laneB.show();
  // Small delay then process any data that arrived during show()
  delayMicroseconds(500);
  checkSerial();
}

void startStrikeWipe() {
  laneA.setBrightness(LED_BRIGHTNESS_STRIKE);
  laneB.setBrightness(LED_BRIGHTNESS_STRIKE);
  strikeWipeStartMs = millis();
  strikeLastFrameMs = 0;
  laneMode = LANE_STRIKE_WIPE;
}

void startStrikeFlash() {
  flashOnPhase = true;
  flashCycles = 0;
  flashLastMs = millis();
  laneMode = LANE_STRIKE_FLASH;
  uint32_t redA = C_RED(laneA);
  uint32_t redB = C_RED(laneB);
  for (int i = 0; i < LANE_LEN1; i++) laneA.setPixelColor(i, redA);
  for (int i = 0; i < LANE_LEN2; i++) laneB.setPixelColor(i, redB);
  laneShowOnly();
}

void startBallCometImmediate() {
  laneAll(C_OFF(laneA));
  laneShowOnly();
  ballCometStartMs = millis();
  ballCometLastFrame = 0;
  laneMode = LANE_BALL_COMET;
}

void endAllLaneAnimsToWhite() {
  laneAll(C_WHITE(laneA));
  laneShowOnly();
  laneMode = LANE_IDLE;
  laneA.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneB.setBrightness(LED_BRIGHTNESS_NORMAL);
}

void laneUpdate() {
  unsigned long now = millis();

  // Skip LED updates briefly after servo commands to let servo PWM settle
  // NeoPixel show() disables interrupts which can corrupt servo timing
  if (now - lastServoCommandTime < SERVO_SETTLE_MS) {
    return;
  }

  // Strike wipe animation
  if (laneMode == LANE_STRIKE_WIPE) {
    unsigned long elapsed = now - strikeWipeStartMs;
    if (elapsed >= STRIKE_WIPE_MS) {
      startStrikeFlash();
    } else if (strikeLastFrameMs == 0 || (now - strikeLastFrameMs) >= STRIKE_FRAME_MS) {
      strikeLastFrameMs = now;
      float t = (float)elapsed / (float)STRIKE_WIPE_MS;
      int n1 = (int)(t * LANE_LEN1 + 0.5f);
      int n2 = (int)(t * LANE_LEN2 + 0.5f);
      uint32_t redA = C_RED(laneA);
      uint32_t redB = C_RED(laneB);
      for (int i = 0; i < LANE_LEN1; i++)
        laneA.setPixelColor(i, (i < n1) ? redA : C_WHITE(laneA));
      for (int i = 0; i < LANE_LEN2; i++)
        laneB.setPixelColor(i, (i < n2) ? redB : C_WHITE(laneB));
      laneShowOnly();
    }
  }

  // Strike flash animation
  if (laneMode == LANE_STRIKE_FLASH) {
    if (flashOnPhase) {
      if (now - flashLastMs >= FLASH_ON_MS) {
        flashOnPhase = false;
        flashLastMs = now;
        laneAll(C_WHITE(laneA));
        laneShowOnly();
      }
    } else {
      if (now - flashLastMs >= FLASH_OFF_MS) {
        flashLastMs = now;
        flashCycles++;
        if (flashCycles >= FLASH_COUNT) {
          endAllLaneAnimsToWhite();
        } else {
          flashOnPhase = true;
          uint32_t redA = C_RED(laneA);
          uint32_t redB = C_RED(laneB);
          for (int i = 0; i < LANE_LEN1; i++) laneA.setPixelColor(i, redA);
          for (int i = 0; i < LANE_LEN2; i++) laneB.setPixelColor(i, redB);
          laneShowOnly();
        }
      }
    }
  }

  // Ball comet animation
  if (laneMode == LANE_BALL_COMET) {
    unsigned long elapsed = now - ballCometStartMs;
    if (elapsed >= BALL_COMET_MS) {
      endAllLaneAnimsToWhite();
      return;
    }
    if (now - ballCometLastFrame >= BALL_COMET_FRAME_MS) {
      ballCometLastFrame = now;
      float t = (float)elapsed / (float)BALL_COMET_MS;
      int maxIndexA = LANE_LEN1 + COMET_LEN;
      int maxIndexB = LANE_LEN2 + COMET_LEN;
      int headA = (int)(t * maxIndexA + 0.5f);
      int headB = (int)(t * maxIndexB + 0.5f);

      for (int i = 0; i < LANE_LEN1; i++) laneA.setPixelColor(i, C_OFF(laneA));
      for (int i = 0; i < LANE_LEN2; i++) laneB.setPixelColor(i, C_OFF(laneB));

      for (int k = 0; k < COMET_LEN; k++) {
        int idxA = headA - k;
        int idxB = headB - k;
        if (idxA >= 0 && idxA < LANE_LEN1) laneA.setPixelColor(idxA, C_WHITE(laneA));
        if (idxB >= 0 && idxB < LANE_LEN2) laneB.setPixelColor(idxB, C_WHITE(laneB));
      }
      laneShowOnly();
    }
  }
}
