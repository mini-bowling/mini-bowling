// =====================================================
// MASTER TEST SCRIPT v1.3.0 - Pinsetter Component Tester
// Consolidated test script for all pinsetter components
//
// Type 'help' for main menu, 'back' to return from sub-menus
// All inputs (sensors) are continuously monitored
// =====================================================

#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <AccelStepper.h>

// Configuration files
#if __has_include("pin_config.user.h")
  #include "pin_config.user.h"
  #include "pin_config.h"
#else
  #include "pin_config.h"
#endif

#if __has_include("general_config.user.h")
  #include "general_config.user.h"
  #include "general_config.h"
#else
  #include "general_config.h"
#endif

// =====================================================
// OBJECTS
// =====================================================

// Servos
Servo ScissorsServo;
Servo SlideServo;
Servo LeftRaiseServo;
Servo RightRaiseServo;
Servo LeftSweepServo;
Servo RightSweepServo;
Servo BallReturnServo;

// Stepper
AccelStepper stepper(1, STEP_PIN, DIR_PIN);

// NeoPixels
Adafruit_NeoPixel deckL(DECK_LED_LENGTH_L, DECK_PIN_L, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel deckR(DECK_LED_LENGTH_R, DECK_PIN_R, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel laneL(LANE_LED_LENGTH_L, LANE_PIN_L, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel laneR(LANE_LED_LENGTH_R, LANE_PIN_R, NEO_GRB + NEO_KHZ800);

// =====================================================
// MENU STATE
// =====================================================

enum MenuState {
  MENU_MAIN,
  MENU_CONVEYOR,
  MENU_SWEEP,
  MENU_BALLDOOR,
  MENU_LANELED,
  MENU_DECKLED,
  MENU_SCISSOR,
  MENU_SLIDER,
  MENU_RAISE,
  MENU_TURRET,
  MENU_FRAMELED,
  MENU_SENSOR,
  MENU_SEQUENCE
};

MenuState currentMenu = MENU_MAIN;

// =====================================================
// INPUT MONITORING STATE
// =====================================================

int lastHallState = HIGH;
int lastIRState = HIGH;
int lastBallState = HIGH;
int lastBallSpeedState = HIGH;

// Sensor monitoring enable/disable
bool hallMonEnabled = true;
bool irMonEnabled = true;
bool ballMonEnabled = true;
bool ballSpeedMonEnabled = true;

// IR flap detection (auto-disable on rapid toggling)
int irFlapCount = 0;
unsigned long irFlapLastChangeMs = 0;

// Debounce
unsigned long irLastChange = 0;
unsigned long ballLastChange = 0;
unsigned long ballSpeedLastChange = 0;

// =====================================================
// SERVO POSITION TRACKING
// =====================================================

// Sweep positions (start at guard so attach doesn't jump to 90)
int sweepCurL = SWEEP_GUARD_ANGLE;
int sweepCurR = 180 - SWEEP_GUARD_ANGLE;

// Sweep tween state (smooth animation like Everything script)
int sweepStartL = SWEEP_GUARD_ANGLE, sweepStartR = 180 - SWEEP_GUARD_ANGLE;
int sweepTargetL = SWEEP_GUARD_ANGLE, sweepTargetR = 180 - SWEEP_GUARD_ANGLE;
unsigned long sweepStartMs = 0;
unsigned long sweepDurationMs = SWEEP_TWEEN_MS;
bool sweepAnimating = false;

// Raise positions (initialized from config)
int raiseLeftPos = RAISE_DROP_ANGLE;
int raiseRightPos = 180 - RAISE_DROP_ANGLE;

// Additional tracking for status
bool conveyorIsOn = false;
int ballDoorAngle = BALL_DOOR_CLOSED_ANGLE;
int scissorAngle = SCISSOR_DROP_ANGLE;
bool scissorCycling = false;
bool scissorCyclePhase = false;  // false = angle1, true = angle2
unsigned long scissorCycleLastMs = 0;
int scissorCycleAngle1 = 0;
int scissorCycleAngle2 = 0;
int sliderAngle = SLIDER_HOME_ANGLE;

// =====================================================
// TURRET STATE
// =====================================================

const int PinPositions[] = {PIN_POS_0, PIN_POS_1, PIN_POS_2, PIN_POS_3, PIN_POS_4,
                            PIN_POS_5, PIN_POS_6, PIN_POS_7, PIN_POS_8, PIN_POS_9, PIN_POS_10};

bool turretMoving = false;
long turretTargetPos = 0;

// Homing state machine
enum HomingPhase {
  HOME_IDLE = 0,
  HOME_ADVANCE_TO_SWITCH,
  HOME_BACKOFF,
  HOME_CREEP_TO_SWITCH,
  HOME_SETZERO_AND_MOVE_SLOT1,
  HOME_DONE,
  HOME_ADJUST_READY
};
HomingPhase homingPhase = HOME_IDLE;
bool homingActive = false;
// After turret load release, the turret is right next to the hall sensor magnet.
// When re-homing, the advance phase could immediately re-trigger the same magnet
// instead of finding it after a full revolution. homingMinSteps tells the advance
// phase to ignore the hall sensor until the turret has moved at least that many
// steps, ensuring it clears the nearby magnet before searching for it properly.
long homingStartPos = 0;
long homingMinSteps = 0;

// Home adjuster tuning state
bool homeAdjustActive = false;
int homeAdjustValue = TURRET_HOME_ADJUSTER;

// Turret homed flag (skip redundant homing across sequences)
bool turretIsHomed = false;

// Sweep Clear state machine (shared by sweep sequence + pin drop pre/post clear)
enum SweepClearPhase {
  SCLEAR_IDLE = 0,
  SCLEAR_RAISE_UP,
  SCLEAR_RAISE_WAIT,
  SCLEAR_SWEEP_BACK,
  SCLEAR_SWEEP_WAIT,
  SCLEAR_BACK_PAUSE,
  SCLEAR_SWEEP_GUARD,
  SCLEAR_SWEEP_GUARD_WAIT,
  SCLEAR_DONE
};
SweepClearPhase sweepClearPhase = SCLEAR_IDLE;
bool sweepClearActive = false;
unsigned long sweepClearPhaseMs = 0;

// Pin Drop state machine (uses sweep clear for pre/post clear)
enum PinDropPhase {
  PDROP_IDLE = 0,
  PDROP_PRECLEAR_WAIT,
  PDROP_SWEEP_GUARD,
  PDROP_SWEEP_GUARD_WAIT,
  PDROP_SCISSOR_OPEN,
  PDROP_RAISE_DOWN,
  PDROP_SLIDER_RELEASE,
  PDROP_RAISE_UP,
  PDROP_SLIDER_HOME,
  PDROP_POSTCLEAR_WAIT,
  PDROP_DONE
};
PinDropPhase pinDropPhase = PDROP_IDLE;
bool pinDropActive = false;
bool pinDropPostClear = false;
unsigned long pinDropPhaseStartMs = 0;

// Sequence menu prompt state (interactive y/n questions)
enum SequencePrompt {
  SEQPROMPT_NONE = 0,
  SEQPROMPT_PD_PRECLEAR,
  SEQPROMPT_PD_POSTCLEAR,
  SEQPROMPT_TL_CONFIRM,
  SEQPROMPT_TL_RESUME,
  SEQPROMPT_PP_READY,
  SEQPROMPT_PS_CONFIRM,
  SEQPROMPT_PC_READY,
  SEQPROMPT_FULL_CONFIRM,
  SEQPROMPT_FULL_PRECLEAR,
  SEQPROMPT_TL_TIMING,
  SEQPROMPT_CT_PRECLEAR,
  SEQPROMPT_CT_READY,
  SEQPROMPT_CT_RAWTIMING
};
SequencePrompt seqPrompt = SEQPROMPT_NONE;
bool pinDropPreClear = false;

// Pin Pickup state machine
enum PinPickupPhase {
  PPU_IDLE = 0,
  PPU_SCISSOR_OPEN,
  PPU_RAISE_GRAB,
  PPU_SCISSOR_CLOSE,
  PPU_RAISE_UP,
  PPU_DONE
};
PinPickupPhase pinPickupPhase = PPU_IDLE;
bool pinPickupActive = false;
unsigned long pinPickupPhaseStartMs = 0;

// Pin Set state machine
enum PinSetPhase {
  PSET_IDLE = 0,
  PSET_RAISE_DROP,
  PSET_SCISSOR_OPEN,
  PSET_RAISE_UP,
  PSET_DONE
};
PinSetPhase pinSetPhase = PSET_IDLE;
bool pinSetActive = false;
unsigned long pinSetPhaseStartMs = 0;

// Pin Cycle (repeating pickup + set)
bool pinCycleActive = false;

// Clear All state machine (clears lane, sliding deck, and turret)
enum ClearAllPhase {
  CLALL_IDLE = 0,
  CLALL_CLOSE_BALL_DOOR,
  CLALL_SWEEP1,
  CLALL_WAIT_SWEEP1,
  CLALL_RAISE_DROP1,
  CLALL_SCISSOR_OPEN1,
  CLALL_RAISE_DOWN1,
  CLALL_SLIDER_RELEASE1,
  CLALL_RAISE_UP1,
  CLALL_SLIDER_HOME1,
  CLALL_SWEEP2,
  CLALL_WAIT_SWEEP2,
  CLALL_HOMING,
  CLALL_WAIT_HOMING,
  CLALL_FAST_TO_SLOT9,
  CLALL_WAIT_FAST_TO_SLOT9,
  CLALL_MOVE_RELEASE,
  CLALL_WAIT_MOVE_RELEASE,
  CLALL_RELEASE_DWELL,
  CLALL_MOVE_SLOT1,
  CLALL_WAIT_MOVE_SLOT1,
  CLALL_RAISE_DOWN2,
  CLALL_SLIDER_RELEASE2,
  CLALL_RAISE_UP2,
  CLALL_SLIDER_HOME2,
  CLALL_SWEEP3,
  CLALL_WAIT_SWEEP3,
  CLALL_SCISSOR_FINAL,
  CLALL_DONE
};
ClearAllPhase clearAllPhase = CLALL_IDLE;
bool clearAllActive = false;
unsigned long clearAllPhaseMs = 0;

// Full Test state machine (continuous load → drop → pickup → set → sweep cycle)
enum FullTestPhase {
  FTEST_IDLE = 0,
  FTEST_PRECLEAR,
  FTEST_WAIT_PRECLEAR,
  FTEST_CLOSE_BALL_DOOR,
  FTEST_FIRST_LOAD,
  FTEST_WAIT_FIRST_LOAD,
  FTEST_PIN_DROP,
  FTEST_WAIT_DROP,
  FTEST_START_BG_LOAD,
  FTEST_PICKUP,
  FTEST_WAIT_PICKUP,
  FTEST_SETTLE,
  FTEST_SET,
  FTEST_WAIT_SET,
  FTEST_SWEEP,
  FTEST_WAIT_SWEEP,
  FTEST_RELEASE,
  FTEST_WAIT_BG_LOAD,
  FTEST_LOOP
};
FullTestPhase fullTestPhase = FTEST_IDLE;
bool fullTestActive = false;
unsigned long fullTestPhaseMs = 0;
int fullTestCycle = 0;
int fullTestTargetCycles = 0;  // 0 = unlimited

// Conveyor Timing Test state machine
enum ConvTimingPhase {
  CTIME_IDLE = 0,
  CTIME_PRECLEAR,
  CTIME_WAIT_PRECLEAR,
  CTIME_WAIT_LOAD,
  CTIME_DONE
};
ConvTimingPhase convTimingPhase = CTIME_IDLE;
bool convTimingActive = false;

// Turret Load state machine
enum TurretLoadPhase {
  TLOAD_IDLE = 0,
  TLOAD_HOMING,
  TLOAD_MOVE_TO_SLOT1,
  TLOAD_WAIT_CATCH,
  TLOAD_CATCH_DELAY,
  TLOAD_ADVANCING,
  TLOAD_NINTH_SETTLE,
  TLOAD_WAIT_TENTH,
  TLOAD_HOLD_RELEASE,
  TLOAD_TENTH_SETTLE,
  TLOAD_RELEASE_DWELL,
  TLOAD_VERIFY_MOVE_SLOT1,
  TLOAD_VERIFY_APPROACH,
  TLOAD_VERIFY_CREEP,
  TLOAD_VERIFY_EVAL,
  TLOAD_REHOMING,
  TLOAD_MOVE_SLOT1,
  TLOAD_DONE
};
TurretLoadPhase turretLoadPhase = TLOAD_IDLE;
bool turretLoadActive = false;
int tlLoadedCount = 0;
int tlNowCatching = 1;

// Persistent turret pin count (survives sequence stop/restart)
int turretPinsLoaded = 0;
unsigned long tlPhaseStartMs = 0;

// IR detection for turret load (separate from monitorInputs)
int tlIrLastRaw = HIGH;
int tlIrStable = HIGH;
unsigned long tlIrLastChange = 0;
bool tlPinArmed = true;
int tlQueuedPins = 0;
unsigned long tlQueuedDetectMs = 0;  // millis() when the queued pin was first detected
bool tlConveyorStopped = false;

// Turret load IR timing (raw sensor transitions, no debounce)
// Stores events in a buffer and prints on sequence completion/stop
struct TlTimingEvent {
  unsigned long timeMs;     // elapsed ms since conveyor start
  unsigned long durationMs; // blocked duration (for clear events) or gap since last clear (for detect events)
  int pinNum;               // which pin (raw block count), or load number for marker events
  char type;                // 'D' = detected, 'C' = cleared, 'R' = turret released (marker)
};
#define TL_TIMING_MAX 200
TlTimingEvent tlTimingLog[TL_TIMING_MAX];
int tlTimingLogCount = 0;   // total events written (may exceed TL_TIMING_MAX)
int tlTimingLogHead = 0;    // next write index (ring buffer)

unsigned long tlTimingStart = 0;
unsigned long tlTimingBlockStart = 0;
unsigned long tlTimingLastClear = 0;
bool tlTimingBlocked = false;
int tlTimingPinCount = 0;
int tlTimingLoadNum = 0;

// Turret load hold-release (for full test background loading)
bool turretLoadHoldRelease = false;

// Hall sensor drift verification after release
bool tlVerifyHallSeen = false;       // Hall sensor triggered during verification creep
long tlVerifyHallPos = 0;            // Stepper position when hall first triggered
bool tlVerifyForFullTest = false;    // Which path to resume after verify/rehome
// Signal that pins have been released to deck (set in RELEASE_DWELL, consumed by full test FSM)
bool turretPinsReleasedToDeck = false;

// =====================================================
// LED STRIP SELECTION
// =====================================================

enum StripSelect { STRIP_BOTH, STRIP_LEFT, STRIP_RIGHT };
StripSelect laneStripSelect = STRIP_BOTH;
StripSelect deckStripSelect = STRIP_BOTH;

// =====================================================
// LED ANIMATION STATE
// =====================================================

enum LEDTarget { LED_NONE, LED_DECK, LED_LANE };
LEDTarget ledAnimTarget = LED_NONE;

enum AnimMode { ANIM_IDLE = 0, ANIM_WIPE, ANIM_FLASH, ANIM_COMET, ANIM_RAINBOW };
AnimMode animMode = ANIM_IDLE;

// Animation timing
unsigned long animStartMs = 0;
unsigned long animLastFrameMs = 0;
uint32_t animColor = 0;
bool flashOnPhase = false;
int flashCycles = 0;
uint16_t rainbowOffset = 0;

uint32_t currentDeckColor;
uint32_t currentLaneColor;
String deckColorName = "WHITE";
String laneColorName = "WHITE";

// Frame LED state
enum FrameBlinkMode { FBLINK_NONE, FBLINK_BOTH, FBLINK_ALT };
FrameBlinkMode frameBlinkMode = FBLINK_NONE;
unsigned long frameBlinkLastMs = 0;
bool frameBlinkState = false;

// =====================================================
// HELPER FUNCTIONS
// =====================================================

// Color helpers
uint32_t C_WHITE() { return deckL.Color(255, 255, 255); }
uint32_t C_RED()   { return deckL.Color(255, 0, 0); }
uint32_t C_GREEN() { return deckL.Color(0, 255, 0); }
uint32_t C_BLUE()  { return deckL.Color(0, 0, 255); }
uint32_t C_YELLOW(){ return deckL.Color(255, 255, 0); }
uint32_t C_PURPLE(){ return deckL.Color(128, 0, 128); }
uint32_t C_ORANGE(){ return deckL.Color(255, 165, 0); }
uint32_t C_OFF()   { return deckL.Color(0, 0, 0); }

void ConveyorOn()  { digitalWrite(MOTOR_RELAY_PIN, CONVEYOR_ACTIVE_HIGH ? HIGH : LOW); }
void ConveyorOff() { digitalWrite(MOTOR_RELAY_PIN, CONVEYOR_ACTIVE_HIGH ? LOW : HIGH); }

// Forward declaration (default parameter requires declaration before first use)
void startTurretHome(long minSteps = 0);

// Check if any automated sequence is currently running
bool anySequenceRunning() {
  return sweepClearActive || pinDropActive || turretLoadActive || homingActive
      || pinPickupActive || pinSetActive || pinCycleActive || fullTestActive
      || clearAllActive || convTimingActive;
}

// Strip selection helper strings
String getStripSelectName(StripSelect sel) {
  switch (sel) {
    case STRIP_LEFT:  return "LEFT";
    case STRIP_RIGHT: return "RIGHT";
    default:          return "BOTH";
  }
}

// =====================================================
// SERVO ATTACH HELPERS (attach on first use)
// =====================================================

void ensureSweepAttached() {
  bool justAttached = false;
  if (!LeftSweepServo.attached()) {
    LeftSweepServo.attach(LEFT_SWEEP_PIN);
    LeftSweepServo.write(sweepCurL);
    justAttached = true;
  }
  if (!RightSweepServo.attached()) {
    RightSweepServo.attach(RIGHT_SWEEP_PIN);
    RightSweepServo.write(sweepCurR);
    justAttached = true;
  }
  if (justAttached) {
    sweepStartL = sweepCurL;
    sweepStartR = sweepCurR;
    sweepTargetL = SWEEP_GUARD_ANGLE;
    sweepTargetR = 180 - SWEEP_GUARD_ANGLE;
    sweepStartMs = millis();
    sweepDurationMs = SWEEP_TWEEN_MS;
    sweepAnimating = !((sweepStartL == sweepTargetL) && (sweepStartR == sweepTargetR));
  }
}

void ensureBallDoorAttached() {
  if (!BallReturnServo.attached()) {
    BallReturnServo.attach(BALL_RETURN_PIN);
    BallReturnServo.write(ballDoorAngle);
  }
}

void ensureScissorAttached() {
  if (!ScissorsServo.attached()) {
    ScissorsServo.attach(SCISSOR_PIN);
    ScissorsServo.write(scissorAngle);
  }
}

void ensureSliderAttached() {
  if (!SlideServo.attached()) {
    SlideServo.attach(SLIDE_PIN);
    SlideServo.write(sliderAngle);
  }
}

void ensureRaiseAttached() {
  if (!LeftRaiseServo.attached()) {
    LeftRaiseServo.attach(RAISE_LEFT_PIN);
    LeftRaiseServo.write(raiseLeftPos);
  }
  if (!RightRaiseServo.attached()) {
    RightRaiseServo.attach(RAISE_RIGHT_PIN);
    RightRaiseServo.write(raiseRightPos);
  }
}

// =====================================================
// SWEEP TWEEN FUNCTIONS (smooth servo movement)
// =====================================================

void startSweepTo(int leftDeg, int rightDeg, unsigned long durMs = SWEEP_TWEEN_MS) {
  ensureSweepAttached();
  leftDeg = constrain(leftDeg, 0, 180);
  rightDeg = constrain(rightDeg, 0, 180);
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
    sweepCurL = sweepTargetL;
    sweepCurR = sweepTargetR;
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
    return;
  }
  float t = (float)elapsed / (float)sweepDurationMs;
  int newL = sweepStartL + (int)((sweepTargetL - sweepStartL) * t + 0.5f);
  int newR = sweepStartR + (int)((sweepTargetR - sweepStartR) * t + 0.5f);
  if (newL != sweepCurL) {
    sweepCurL = newL;
    LeftSweepServo.write(sweepCurL);
  }
  if (newR != sweepCurR) {
    sweepCurR = newR;
    RightSweepServo.write(sweepCurR);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(SCOREMORE_BAUD);
  delay(1000);

  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("   MASTER TEST SCRIPT v1.3.0"));
  Serial.println(F("   Pinsetter Component Tester"));
  Serial.println(F("========================================"));
  Serial.println(F(""));
  Serial.println(F("All components start OFF/neutral."));
  Serial.println(F("Sensors are continuously monitored."));
  Serial.println(F(""));

  // Initialize outputs
  pinMode(MOTOR_RELAY_PIN, OUTPUT);
  ConveyorOff();

  pinMode(FRAME_LED1_PIN, OUTPUT);
  pinMode(FRAME_LED2_PIN, OUTPUT);
  digitalWrite(FRAME_LED1_PIN, LOW);
  digitalWrite(FRAME_LED2_PIN, LOW);

  // Initialize inputs
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(HALL_EFFECT_PIN, INPUT_PULLUP);
  pinMode(BALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BALL_SPEED_PIN, INPUT_PULLUP);

  // Read initial states
  lastHallState = digitalRead(HALL_EFFECT_PIN);
  lastIRState = digitalRead(IR_SENSOR_PIN);
  lastBallState = digitalRead(BALL_SENSOR_PIN);
  lastBallSpeedState = digitalRead(BALL_SPEED_PIN);

  // Initialize stepper (but don't move)
  stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
  stepper.setAcceleration(TURRET_NORMAL_ACCEL);
#ifdef STEPPER_ENABLE_PIN
  stepper.setEnablePin(STEPPER_ENABLE_PIN);
  stepper.setPinsInverted(false, false, true);  // Enable pin is active LOW
  stepper.enableOutputs();
#endif

  // Initialize NeoPixels (start OFF)
  deckL.begin();
  deckR.begin();
  laneL.begin();
  laneR.begin();
  deckL.setBrightness(DECK_LED_BRIGHTNESS);
  deckR.setBrightness(DECK_LED_BRIGHTNESS);
  laneL.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneR.setBrightness(LED_BRIGHTNESS_NORMAL);

  // All LEDs off
  deckAll(C_OFF());
  laneAll(C_OFF());
  deckShow();
  laneShow();

  currentDeckColor = C_WHITE();
  currentLaneColor = C_WHITE();

  // NOTE: Servos are NOT attached at startup.
  // They will be attached lazily when first needed (menu entry or sequence).

  printMainMenu();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  // Always run these
  monitorInputs();
  stepper.run();
  updateSweepTween();

  if (homingActive) {
    runHomingFSM();
  }

  if (sweepClearActive) {
    runSweepClearFSM();
  }

  if (pinDropActive) {
    runPinDropFSM();
  }

  if (turretLoadActive) {
    runTurretLoadFSM();
  }

  if (pinPickupActive) {
    runPinPickupFSM();
  }

  if (pinSetActive) {
    runPinSetFSM();
  }

  if (fullTestActive) {
    runFullTestFSM();
  }

  if (clearAllActive) {
    runClearAllFSM();
  }

  if (convTimingActive) {
    runConvTimingFSM();
  }

  // Check turret movement complete (suppress during automated sequences)
  if (turretMoving && stepper.distanceToGo() == 0) {
    turretMoving = false;
    if (!turretLoadActive && !homeAdjustActive && !clearAllActive) {
      Serial.println(F(">> Movement complete"));
    }
  }

  // Update scissor cycling
  updateScissorCycle();

  // Update LED animations
  updateLEDAnimation();

  // Update frame LED blink
  updateFrameBlink();

  // Check serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    processCommand(input);
  }
}

// =====================================================
// INPUT MONITORING
// =====================================================

void monitorInputs() {
  unsigned long now = millis();

  // Hall Effect (no debounce needed - hardware)
  if (hallMonEnabled) {
    int hallState = digitalRead(HALL_EFFECT_PIN);
    if (hallState != lastHallState) {
      Serial.println(F(""));
      Serial.print(F(">>> HALL EFFECT: "));
      if (hallState == LOW) {
        Serial.println(F("MAGNET DETECTED"));
      } else {
        Serial.println(F("MAGNET RELEASED"));
      }
      lastHallState = hallState;
    }
  }

  // IR Sensor (debounced, with flap detection)
  if (irMonEnabled) {
    int irState = digitalRead(IR_SENSOR_PIN);
    if (irState != lastIRState) {
      if (now - irLastChange > DEBOUNCE_MS) {
        Serial.println(F(""));
        Serial.print(F(">>> IR SENSOR (Pin 5): "));
        if (irState == LOW) {
          Serial.println(F("BEAM BLOCKED (pin detected)"));
        } else {
          Serial.println(F("BEAM CLEAR"));
        }
        lastIRState = irState;

        // IR flap detection
        if (now - irFlapLastChangeMs < IR_FLAP_WINDOW_MS) {
          irFlapCount++;
        } else {
          irFlapCount = 1;
        }
        irFlapLastChangeMs = now;

        if (irFlapCount >= IR_FLAP_COUNT) {
          irMonEnabled = false;
          irFlapCount = 0;
          Serial.println(F(""));
          Serial.println(F(">>> WARNING: IR sensor flapping detected!"));
          Serial.println(F(">>> IR monitoring auto-disabled."));
          Serial.println(F(">>> Use sensor menu to re-enable."));
        }

        irLastChange = now;
      }
    }
  }

  // Ball Sensor (debounced)
  if (ballMonEnabled) {
    int ballState = digitalRead(BALL_SENSOR_PIN);
    if (ballState != lastBallState) {
      if (now - ballLastChange > DEBOUNCE_MS) {
        Serial.println(F(""));
        Serial.print(F(">>> BALL TRIGGER (A0): "));
        if (ballState == LOW) {
          Serial.println(F("BALL DETECTED"));
        } else {
          Serial.println(F("NO BALL"));
        }
        lastBallState = ballState;
        ballLastChange = now;
      }
    }
  }

  // Ball Speed Sensor (debounced)
  if (ballSpeedMonEnabled) {
    int ballSpeedState = digitalRead(BALL_SPEED_PIN);
    if (ballSpeedState != lastBallSpeedState) {
      if (now - ballSpeedLastChange > DEBOUNCE_MS) {
        Serial.println(F(""));
        Serial.print(F(">>> BALL SPEED (A1): "));
        if (ballSpeedState == LOW) {
          Serial.println(F("BEAM BLOCKED"));
        } else {
          Serial.println(F("BEAM CLEAR"));
        }
        lastBallSpeedState = ballSpeedState;
        ballSpeedLastChange = now;
      }
    }
  }
}

// =====================================================
// COMMAND PROCESSING
// =====================================================

void stopAllSequences() {
  seqPrompt = SEQPROMPT_NONE;

  if (sweepClearActive) {
    sweepClearActive = false;
    sweepClearPhase = SCLEAR_IDLE;
    Serial.println(F(">> Sweep clear STOPPED"));
  }

  if (pinDropActive) {
    pinDropActive = false;
    pinDropPhase = PDROP_IDLE;
    Serial.println(F(">> Pin drop sequence STOPPED"));
  }

  bool hasTiming = false;

  if (turretLoadActive) {
    turretLoadActive = false;
    turretLoadPhase = TLOAD_IDLE;
    ConveyorOff();
    conveyorIsOn = false;
    Serial.print(F(">> Turret load sequence STOPPED ("));
    Serial.print(turretPinsLoaded);
    Serial.println(F(" pins in turret)"));
    if (tlTimingLogCount > 0) hasTiming = true;
  }

  if (pinPickupActive) {
    pinPickupActive = false;
    pinPickupPhase = PPU_IDLE;
    Serial.println(F(">> Pin pickup sequence STOPPED"));
  }

  if (pinSetActive) {
    pinSetActive = false;
    pinSetPhase = PSET_IDLE;
    Serial.println(F(">> Pin set sequence STOPPED"));
  }

  if (pinCycleActive) {
    pinCycleActive = false;
    Serial.println(F(">> Pin cycle STOPPED"));
  }

  if (fullTestActive) {
    fullTestActive = false;
    fullTestPhase = FTEST_IDLE;
    turretLoadHoldRelease = false;
    turretPinsReleasedToDeck = false;
    deckAll(C_OFF());
    deckShow();
    Serial.println(F(">> Full test STOPPED (deck LEDs off)"));
  }

  if (clearAllActive) {
    clearAllActive = false;
    clearAllPhase = CLALL_IDLE;
    Serial.println(F(">> Clear all STOPPED"));
  }

  if (convTimingActive) {
    convTimingActive = false;
    convTimingPhase = CTIME_IDLE;
    Serial.println(F(">> Conveyor timing test STOPPED"));
  }

  if (homeAdjustActive) {
    homeAdjustActive = false;
    homeAdjustValue = TURRET_HOME_ADJUSTER;
    Serial.println(F(">> Home adjustment STOPPED"));
  }

  if (homingActive) {
    homingActive = false;
    homingPhase = HOME_IDLE;
    turretIsHomed = false;  // Interrupted homing invalidates position
    Serial.println(F(">> Homing STOPPED"));
  }

  stepper.stop();
  stepper.setCurrentPosition(stepper.currentPosition());
  turretMoving = false;
  Serial.println(F(""));

  if (hasTiming) {
    seqPrompt = SEQPROMPT_TL_TIMING;
    Serial.println(F("Display IR sensor and turret timing diagnostic data? (y/N)"));
  }
}

void processCommand(String cmd) {
  cmd.toLowerCase();

  // Empty input (bare Enter): if a y/n prompt is active, treat as 'y';
  // otherwise stops any running sequence
  if (cmd.length() == 0) {
    if (seqPrompt != SEQPROMPT_NONE) {
      // Timing prompts default to 'n' on bare Enter; all others default to 'y'
      cmd = (seqPrompt == SEQPROMPT_TL_TIMING || seqPrompt == SEQPROMPT_CT_RAWTIMING) ? "n" : "y";
      // Fall through to menu handler
    } else if (sweepClearActive || pinDropActive || turretLoadActive || homingActive || pinPickupActive || pinSetActive || pinCycleActive || homeAdjustActive || fullTestActive || clearAllActive || convTimingActive) {
      stopAllSequences();
      return;
    } else {
      return;
    }
  }

  // Global commands
  if (cmd == "x" || cmd == "stop") {
    if (sweepClearActive || pinDropActive || turretLoadActive || homingActive || pinPickupActive || pinSetActive || pinCycleActive || homeAdjustActive || fullTestActive || clearAllActive || convTimingActive) {
      stopAllSequences();
      return;
    }
    // Fall through to menu handler (e.g., turret menu manual stop)
  }

  if (cmd == "help" || cmd == "h" || cmd == "?") {
    if (homeAdjustActive && homingPhase == HOME_ADJUST_READY) {
      Serial.println(F(""));
      Serial.println(F(">> HOME ADJUSTER TUNING"));
      Serial.print(F(">> Current value: "));
      Serial.println(homeAdjustValue);
      Serial.println(F(">>   +       Adjust +1 step"));
      Serial.println(F(">>   -       Adjust -1 step"));
      Serial.println(F(">>   ++      Adjust +5 steps"));
      Serial.println(F(">>   --      Adjust -5 steps"));
      Serial.println(F(">>   <num>   Set to specific value"));
      Serial.println(F(">>   done    Accept & show config line"));
      Serial.println(F(">>   cancel  Discard changes"));
    } else if (currentMenu == MENU_MAIN) {
      printMainMenu();
    } else {
      printCurrentSubMenu();
    }
    return;
  }

  if (cmd == "back" || cmd == "b" || cmd == "exit" || cmd == "main") {
    // Block back when turret adjustment sub-menu is active
    if (homeAdjustActive && homingPhase == HOME_ADJUST_READY) {
      Serial.println(F(">> Use 'done' or 'cancel' to exit adjustment mode"));
      return;
    }
    if (currentMenu != MENU_MAIN) {
      seqPrompt = SEQPROMPT_NONE;
      detachCurrentServos();
      currentMenu = MENU_MAIN;
      Serial.println(F(""));
      Serial.println(F(">> Returned to MAIN MENU"));
      printMainMenu();
    }
    return;
  }

  // Route to current menu handler
  switch (currentMenu) {
    case MENU_MAIN:      handleMainMenu(cmd); break;
    case MENU_CONVEYOR:  handleConveyorMenu(cmd); break;
    case MENU_SWEEP:     handleSweepMenu(cmd); break;
    case MENU_BALLDOOR:  handleBallDoorMenu(cmd); break;
    case MENU_LANELED:   handleLaneLEDMenu(cmd); break;
    case MENU_DECKLED:   handleDeckLEDMenu(cmd); break;
    case MENU_SCISSOR:   handleScissorMenu(cmd); break;
    case MENU_SLIDER:    handleSliderMenu(cmd); break;
    case MENU_RAISE:     handleRaiseMenu(cmd); break;
    case MENU_TURRET:    handleTurretMenu(cmd); break;
    case MENU_FRAMELED:  handleFrameLEDMenu(cmd); break;
    case MENU_SENSOR:    handleSensorMenu(cmd); break;
    case MENU_SEQUENCE:  handleSequenceMenu(cmd); break;
  }
}

void detachCurrentServos() {
  // Detach servos when leaving a menu to prevent holding/buzzing
  switch (currentMenu) {
    case MENU_SWEEP:
      LeftSweepServo.detach();
      RightSweepServo.detach();
      break;
    case MENU_BALLDOOR:
      BallReturnServo.detach();
      break;
    case MENU_SCISSOR:
      ScissorsServo.detach();
      break;
    case MENU_SLIDER:
      SlideServo.detach();
      break;
    case MENU_RAISE:
      LeftRaiseServo.detach();
      RightRaiseServo.detach();
      break;
    default:
      break;
  }
}

// =====================================================
// MAIN MENU
// =====================================================

void printMainMenu() {
  Serial.println(F(""));
  Serial.println(F("============ MAIN MENU ============"));
  Serial.println(F("Enter component name to test:"));
  Serial.println(F(""));
  Serial.println(F("  conveyor (co) - Conveyor relay (pin 4)"));
  Serial.println(F("  sweep (sw)    - Sweep servos (pins 11,12)"));
  Serial.println(F("  balldoor (bd) - Ball return door (pin 13)"));
  Serial.println(F("  laneled (ll)  - Lane LED strips (pins 52,53)"));
  Serial.println(F("  deckled (dl)  - Deck LED strips (pins 50,51)"));
  Serial.println(F("  scissor (sc)  - Scissor servo (pin 7)"));
  Serial.println(F("  slider (sl)   - Sliding deck servo (pin 8)"));
  Serial.println(F("  raise (ra)    - Raise servos (pins 9,10)"));
  Serial.println(F("  turret (tu)   - Turret stepper (pins 2,3)"));
  Serial.println(F("  frameled (fl) - Frame indicator LEDs (46,47)"));
  Serial.println(F("  sensor (se)   - Sensor monitoring settings"));
  Serial.println(F("  sequence (sq) - Run automated sequences"));
  Serial.println(F(""));
  Serial.println(F("  status (s)    - Show all sensor states"));
  Serial.println(F("  disengage (di)- Disengage all servos & motors"));
  Serial.println(F("  help (h)      - Show this menu"));
  Serial.println(F(""));
  Serial.println(F("Sensors monitored: Hall(6), IR(5), BallTrig(A0), BallSpd(A1)"));
  Serial.println(F("==================================="));
  Serial.println(F(""));
}

void handleMainMenu(String cmd) {
  if (cmd == "status" || cmd == "s") {
    printSensorStatus();
    return;
  }
  if (cmd == "conveyor" || cmd == "co") {
    currentMenu = MENU_CONVEYOR;
    Serial.println(F(">> Entering CONVEYOR test"));
    printConveyorMenu();
  }
  else if (cmd == "sweep" || cmd == "sw") {
    currentMenu = MENU_SWEEP;
    // Servos attached on first move command, not here
    Serial.println(F(">> Entering SWEEP test"));
    printSweepMenu();
  }
  else if (cmd == "balldoor" || cmd == "bd") {
    currentMenu = MENU_BALLDOOR;
    // Servo attached on first move command, not here
    Serial.println(F(">> Entering BALL DOOR test"));
    printBallDoorMenu();
  }
  else if (cmd == "laneled" || cmd == "ll") {
    currentMenu = MENU_LANELED;
    ledAnimTarget = LED_LANE;
    Serial.println(F(">> Entering LANE LED test"));
    printLaneLEDMenu();
  }
  else if (cmd == "deckled" || cmd == "dl") {
    currentMenu = MENU_DECKLED;
    ledAnimTarget = LED_DECK;
    Serial.println(F(">> Entering DECK LED test"));
    printDeckLEDMenu();
  }
  else if (cmd == "scissor" || cmd == "sc") {
    currentMenu = MENU_SCISSOR;
    // Servo attached on first move command, not here
    Serial.println(F(">> Entering SCISSOR test"));
    printScissorMenu();
  }
  else if (cmd == "slider" || cmd == "sl") {
    currentMenu = MENU_SLIDER;
    // Servo attached on first move command, not here
    Serial.println(F(">> Entering SLIDING DECK test"));
    printSliderMenu();
  }
  else if (cmd == "raise" || cmd == "ra") {
    currentMenu = MENU_RAISE;
    // Servos attached on first move command, not here
    Serial.println(F(">> Entering RAISE test"));
    printRaiseMenu();
  }
  else if (cmd == "turret" || cmd == "tu") {
    currentMenu = MENU_TURRET;
    Serial.println(F(">> Entering TURRET test"));
    printTurretMenu();
  }
  else if (cmd == "frameled" || cmd == "fl") {
    currentMenu = MENU_FRAMELED;
    Serial.println(F(">> Entering FRAME LED test"));
    printFrameLEDMenu();
  }
  else if (cmd == "sensor" || cmd == "se") {
    currentMenu = MENU_SENSOR;
    Serial.println(F(">> Entering SENSOR settings"));
    printSensorMenu();
  }
  else if (cmd == "sequence" || cmd == "sq") {
    currentMenu = MENU_SEQUENCE;
    Serial.println(F(">> Entering SEQUENCE menu"));
    printSequenceMenu();
  }
  else if (cmd == "disengage" || cmd == "di") {
    stopAllSequences();
    ConveyorOff();
    conveyorIsOn = false;
    ScissorsServo.detach();
    SlideServo.detach();
    LeftRaiseServo.detach();
    RightRaiseServo.detach();
    LeftSweepServo.detach();
    RightSweepServo.detach();
    BallReturnServo.detach();
#ifdef STEPPER_ENABLE_PIN
    stepper.disableOutputs();
#endif
    Serial.println(F(">> All servos and motors DISENGAGED"));
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
    Serial.println(F("Type 'help' or 'h' for menu"));
  }
}

void printCurrentSubMenu() {
  switch (currentMenu) {
    case MENU_CONVEYOR:  printConveyorMenu(); break;
    case MENU_SWEEP:     printSweepMenu(); break;
    case MENU_BALLDOOR:  printBallDoorMenu(); break;
    case MENU_LANELED:   printLaneLEDMenu(); break;
    case MENU_DECKLED:   printDeckLEDMenu(); break;
    case MENU_SCISSOR:   printScissorMenu(); break;
    case MENU_SLIDER:    printSliderMenu(); break;
    case MENU_RAISE:     printRaiseMenu(); break;
    case MENU_TURRET:    printTurretMenu(); break;
    case MENU_FRAMELED:  printFrameLEDMenu(); break;
    case MENU_SENSOR:    printSensorMenu(); break;
    case MENU_SEQUENCE:  printSequenceMenu(); break;
    default: printMainMenu(); break;
  }
}

void printSensorStatus() {
  Serial.println(F(""));
  Serial.println(F("======== SENSOR STATUS ========"));
  Serial.print(F("Hall Effect (pin 6):  "));
  if (digitalRead(HALL_EFFECT_PIN) == LOW) {
    Serial.println(F("LOW (magnet detected)"));
  } else {
    Serial.println(F("HIGH (no magnet)"));
  }
  Serial.print(F("IR Sensor (pin 5):    "));
  if (digitalRead(IR_SENSOR_PIN) == LOW) {
    Serial.println(F("LOW (beam blocked)"));
  } else {
    Serial.println(F("HIGH (beam clear)"));
  }
  Serial.print(F("Ball Trigger (A0):    "));
  if (digitalRead(BALL_SENSOR_PIN) == LOW) {
    Serial.println(F("LOW (ball detected)"));
  } else {
    Serial.println(F("HIGH (no ball)"));
  }
  Serial.print(F("Ball Speed (A1):      "));
  if (digitalRead(BALL_SPEED_PIN) == LOW) {
    Serial.println(F("LOW (beam blocked)"));
  } else {
    Serial.println(F("HIGH (beam clear)"));
  }
  Serial.println(F("==============================="));
  Serial.println(F(""));
}

// =====================================================
// SENSOR MENU
// =====================================================

void printSensorMenu() {
  Serial.println(F(""));
  Serial.println(F("===== SENSOR SETTINGS ====="));
  Serial.println(F("Toggle individual sensors:"));
  Serial.println(F("  hall      - Toggle Hall effect monitoring"));
  Serial.println(F("  ir        - Toggle IR sensor monitoring"));
  Serial.println(F("  ball      - Toggle Ball trigger monitoring"));
  Serial.println(F("  speed     - Toggle Ball speed monitoring"));
  Serial.println(F(""));
  Serial.println(F("Bulk:"));
  Serial.println(F("  allon     - Enable all sensors"));
  Serial.println(F("  alloff    - Disable all sensors"));
  Serial.println(F(""));
  Serial.println(F("  status (s) - Show enabled/disabled + pin states"));
  Serial.println(F("  back (b)   - Return to main menu"));
  Serial.println(F("============================"));
  Serial.println(F(""));
}

void handleSensorMenu(String cmd) {
  if (cmd == "hall") {
    hallMonEnabled = !hallMonEnabled;
    Serial.print(F(">> Hall effect monitoring: "));
    Serial.println(hallMonEnabled ? "ENABLED" : "DISABLED");
  }
  else if (cmd == "ir") {
    irMonEnabled = !irMonEnabled;
    if (irMonEnabled) {
      irFlapCount = 0;
    }
    Serial.print(F(">> IR sensor monitoring: "));
    Serial.println(irMonEnabled ? "ENABLED" : "DISABLED");
  }
  else if (cmd == "ball") {
    ballMonEnabled = !ballMonEnabled;
    Serial.print(F(">> Ball trigger monitoring: "));
    Serial.println(ballMonEnabled ? "ENABLED" : "DISABLED");
  }
  else if (cmd == "speed") {
    ballSpeedMonEnabled = !ballSpeedMonEnabled;
    Serial.print(F(">> Ball speed monitoring: "));
    Serial.println(ballSpeedMonEnabled ? "ENABLED" : "DISABLED");
  }
  else if (cmd == "allon") {
    hallMonEnabled = true;
    irMonEnabled = true;
    irFlapCount = 0;
    ballMonEnabled = true;
    ballSpeedMonEnabled = true;
    Serial.println(F(">> All sensors ENABLED"));
  }
  else if (cmd == "alloff") {
    hallMonEnabled = false;
    irMonEnabled = false;
    ballMonEnabled = false;
    ballSpeedMonEnabled = false;
    Serial.println(F(">> All sensors DISABLED"));
  }
  else if (cmd == "status" || cmd == "s") {
    Serial.println(F(""));
    Serial.println(F("======== SENSOR STATUS ========"));
    Serial.print(F("Hall Effect (pin 6):  "));
    Serial.print(hallMonEnabled ? "[ON]  " : "[OFF] ");
    if (digitalRead(HALL_EFFECT_PIN) == LOW) {
      Serial.println(F("LOW (magnet detected)"));
    } else {
      Serial.println(F("HIGH (no magnet)"));
    }
    Serial.print(F("IR Sensor (pin 5):    "));
    Serial.print(irMonEnabled ? "[ON]  " : "[OFF] ");
    if (digitalRead(IR_SENSOR_PIN) == LOW) {
      Serial.println(F("LOW (beam blocked)"));
    } else {
      Serial.println(F("HIGH (beam clear)"));
    }
    Serial.print(F("Ball Trigger (A0):    "));
    Serial.print(ballMonEnabled ? "[ON]  " : "[OFF] ");
    if (digitalRead(BALL_SENSOR_PIN) == LOW) {
      Serial.println(F("LOW (ball detected)"));
    } else {
      Serial.println(F("HIGH (no ball)"));
    }
    Serial.print(F("Ball Speed (A1):      "));
    Serial.print(ballSpeedMonEnabled ? "[ON]  " : "[OFF] ");
    if (digitalRead(BALL_SPEED_PIN) == LOW) {
      Serial.println(F("LOW (beam blocked)"));
    } else {
      Serial.println(F("HIGH (beam clear)"));
    }
    Serial.println(F("==============================="));
    Serial.println(F(""));
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

// =====================================================
// CONVEYOR MENU
// =====================================================

void printConveyorMenu() {
  Serial.println(F(""));
  Serial.println(F("===== CONVEYOR TEST (pin 4) ====="));
  Serial.println(F("  on (1)     - Turn conveyor ON"));
  Serial.println(F("  off (0)    - Turn conveyor OFF"));
  Serial.println(F("  status (s) - Show status"));
  Serial.println(F("  back (b)   - Return to main menu"));
  Serial.println(F("================================="));
  Serial.println(F(""));
}

void handleConveyorMenu(String cmd) {
  if (cmd == "on" || cmd == "1") {
    ConveyorOn();
    conveyorIsOn = true;
    Serial.println(F(">> Conveyor ON"));
  }
  else if (cmd == "off" || cmd == "0") {
    ConveyorOff();
    conveyorIsOn = false;
    Serial.println(F(">> Conveyor OFF"));
  }
  else if (cmd == "status" || cmd == "s") {
    Serial.print(F("Conveyor: "));
    Serial.println(conveyorIsOn ? "ON" : "OFF");
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

// =====================================================
// SWEEP MENU
// =====================================================

void printSweepMenu() {
  Serial.println(F(""));
  Serial.println(F("===== SWEEP TEST (pins 11,12) ====="));
  Serial.println(F("Presets:"));
  Serial.print(F("  home (ho)     - Home/Back (L="));
  Serial.print(SWEEP_BACK_ANGLE);
  Serial.print(F(", R="));
  Serial.print(180 - SWEEP_BACK_ANGLE);
  Serial.println(F(")"));
  Serial.print(F("  up (u)        - Up (L="));
  Serial.print(SWEEP_UP_ANGLE);
  Serial.print(F(", R="));
  Serial.print(180 - SWEEP_UP_ANGLE);
  Serial.println(F(")"));
  Serial.print(F("  guard (g)     - Guard (L="));
  Serial.print(SWEEP_GUARD_ANGLE);
  Serial.print(F(", R="));
  Serial.print(180 - SWEEP_GUARD_ANGLE);
  Serial.println(F(")"));
  Serial.println(F(""));
  Serial.println(F("Custom angle (both servos move together):"));
  Serial.println(F("  a50           - Set angle (L=50, R=130)"));
  Serial.println(F(""));
  Serial.println(F("  disengage (de)- Detach servos"));
  Serial.println(F("  status (s)    - Show status"));
  Serial.println(F("  back (b)      - Return to main menu"));
  Serial.println(F("====================================="));
  Serial.println(F(""));
}

void handleSweepMenu(String cmd) {
  if (cmd == "home" || cmd == "ho") {
    int targetL = SWEEP_BACK_ANGLE;
    int targetR = 180 - SWEEP_BACK_ANGLE;
    startSweepTo(targetL, targetR);
    Serial.print(F(">> HOME (L="));
    Serial.print(targetL);
    Serial.print(F(", R="));
    Serial.print(targetR);
    Serial.println(F(")"));
  }
  else if (cmd == "up" || cmd == "u") {
    int targetL = SWEEP_UP_ANGLE;
    int targetR = 180 - SWEEP_UP_ANGLE;
    startSweepTo(targetL, targetR);
    Serial.print(F(">> SWEEP UP (L="));
    Serial.print(targetL);
    Serial.print(F(", R="));
    Serial.print(targetR);
    Serial.println(F(")"));
  }
  else if (cmd == "guard" || cmd == "g") {
    int targetL = SWEEP_GUARD_ANGLE;
    int targetR = 180 - SWEEP_GUARD_ANGLE;
    startSweepTo(targetL, targetR);
    Serial.print(F(">> SWEEP GUARD (L="));
    Serial.print(targetL);
    Serial.print(F(", R="));
    Serial.print(targetR);
    Serial.println(F(")"));
  }
  else if (cmd.startsWith("a")) {
    int angle = cmd.substring(1).toInt();
    angle = constrain(angle, 0, 180);
    int targetL = angle;
    int targetR = 180 - angle;
    startSweepTo(targetL, targetR);
    Serial.print(F(">> Angle set: L="));
    Serial.print(targetL);
    Serial.print(F(", R="));
    Serial.println(targetR);
  }
  else if (cmd == "disengage" || cmd == "de") {
    LeftSweepServo.detach();
    RightSweepServo.detach();
    sweepAnimating = false;
    Serial.println(F(">> Sweep servos DISENGAGED"));
  }
  else if (cmd == "status" || cmd == "s") {
    Serial.print(F("Current: L="));
    Serial.print(sweepCurL);
    Serial.print(F(", R="));
    Serial.print(sweepCurR);
    if (sweepAnimating) {
      Serial.println(F(" (moving...)"));
    } else if (!LeftSweepServo.attached()) {
      Serial.println(F(" (disengaged)"));
    } else {
      Serial.println(F(""));
    }
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

// =====================================================
// BALL DOOR MENU
// =====================================================

void printBallDoorMenu() {
  Serial.println(F(""));
  Serial.println(F("===== BALL DOOR TEST (pin 13) ====="));
  Serial.print(F("  home (ho)  - Home/Closed ("));
  Serial.print(BALL_DOOR_CLOSED_ANGLE);
  Serial.println(F(" deg)"));
  Serial.print(F("  open (o)   - Open door ("));
  Serial.print(BALL_DOOR_OPEN_ANGLE);
  Serial.println(F(" deg)"));
  Serial.print(F("  close (c)  - Close door ("));
  Serial.print(BALL_DOOR_CLOSED_ANGLE);
  Serial.println(F(" deg)"));
  Serial.println(F("  a90        - Custom angle"));
  Serial.println(F("  disengage (de) - Detach servo"));
  Serial.println(F("  status (s) - Show status"));
  Serial.println(F("  back (b)   - Return to main menu"));
  Serial.println(F("====================================="));
  Serial.println(F(""));
}

void handleBallDoorMenu(String cmd) {
  if (cmd == "home" || cmd == "ho") {
    ensureBallDoorAttached();
    ballDoorAngle = BALL_DOOR_CLOSED_ANGLE;
    BallReturnServo.write(ballDoorAngle);
    Serial.print(F(">> HOME ("));
    Serial.print(ballDoorAngle);
    Serial.println(F(")"));
  }
  else if (cmd == "open" || cmd == "o" || cmd == "1") {
    ensureBallDoorAttached();
    ballDoorAngle = BALL_DOOR_OPEN_ANGLE;
    BallReturnServo.write(ballDoorAngle);
    Serial.print(F(">> Ball door OPEN ("));
    Serial.print(ballDoorAngle);
    Serial.println(F(")"));
  }
  else if (cmd == "close" || cmd == "c" || cmd == "0") {
    ensureBallDoorAttached();
    ballDoorAngle = BALL_DOOR_CLOSED_ANGLE;
    BallReturnServo.write(ballDoorAngle);
    Serial.print(F(">> Ball door CLOSED ("));
    Serial.print(ballDoorAngle);
    Serial.println(F(")"));
  }
  else if (cmd.startsWith("a")) {
    ensureBallDoorAttached();
    int angle = cmd.substring(1).toInt();
    angle = constrain(angle, 0, 180);
    ballDoorAngle = angle;
    BallReturnServo.write(ballDoorAngle);
    Serial.print(F(">> Ball door angle: "));
    Serial.println(ballDoorAngle);
  }
  else if (cmd == "disengage" || cmd == "de") {
    BallReturnServo.detach();
    Serial.println(F(">> Ball door servo DISENGAGED"));
  }
  else if (cmd == "status" || cmd == "s") {
    Serial.print(F("Ball door angle: "));
    Serial.print(ballDoorAngle);
    if (!BallReturnServo.attached()) {
      Serial.println(F(" (disengaged)"));
    } else if (ballDoorAngle <= 10) {
      Serial.println(F(" (CLOSED)"));
    } else if (ballDoorAngle >= 170) {
      Serial.println(F(" (OPEN)"));
    } else {
      Serial.println(F(""));
    }
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

// =====================================================
// LANE LED MENU
// =====================================================

void printLaneLEDMenu() {
  Serial.println(F(""));
  Serial.println(F("===== LANE LED TEST (pins 52,53) ====="));
  Serial.println(F("Strip Selection:"));
  Serial.println(F("  left (l)  - Control LEFT strip only (pin 52)"));
  Serial.println(F("  right (r) - Control RIGHT strip only (pin 53)"));
  Serial.println(F("  both (bo) - Control BOTH strips (default)"));
  Serial.println(F(""));
  Serial.println(F("On/Off:"));
  Serial.println(F("  on (1)  - LEDs on (current color)"));
  Serial.println(F("  off (0) - LEDs off"));
  Serial.println(F(""));
  Serial.println(F("Colors:"));
  Serial.println(F("  white (w), red (re), green (g), blue (bl)"));
  Serial.println(F("  yellow (y), purple (p), orange (o)"));
  Serial.println(F(""));
  Serial.println(F("Animations:"));
  Serial.println(F("  wipe (wi)   - Color wipe"));
  Serial.println(F("  strike (st) - Strike effect"));
  Serial.println(F("  flash (fl)  - Flash effect"));
  Serial.println(F("  comet (cm)  - Comet effect"));
  Serial.println(F("  rainbow (rb)- Rainbow cycle"));
  Serial.println(F("  stop (x)    - Stop animation"));
  Serial.println(F(""));
  Serial.println(F("Settings:"));
  Serial.println(F("  br80        - Set brightness (0-255)"));
  Serial.println(F(""));
  Serial.println(F("  status (s)  - Show status"));
  Serial.println(F("  back (b)    - Return to main menu"));
  Serial.println(F("========================================"));
  Serial.println(F(""));
}

void handleLaneLEDMenu(String cmd) {
  // Strip selection commands
  if (cmd == "left" || cmd == "l") {
    laneStripSelect = STRIP_LEFT;
    Serial.println(F(">> Selected: LEFT strip only"));
    return;
  }
  else if (cmd == "right" || cmd == "r") {
    laneStripSelect = STRIP_RIGHT;
    Serial.println(F(">> Selected: RIGHT strip only"));
    return;
  }
  else if (cmd == "both" || cmd == "bo") {
    laneStripSelect = STRIP_BOTH;
    Serial.println(F(">> Selected: BOTH strips"));
    return;
  }

  if (cmd == "on" || cmd == "1") {
    animMode = ANIM_IDLE;
    laneSetColor(currentLaneColor);
    laneShowSelected();
    Serial.print(F(">> Lane LEDs ON ("));
    Serial.print(laneColorName);
    Serial.print(F(") ["));
    Serial.print(getStripSelectName(laneStripSelect));
    Serial.println(F("]"));
  }
  else if (cmd == "off" || cmd == "0") {
    animMode = ANIM_IDLE;
    laneSetColor(C_OFF());
    laneShowSelected();
    Serial.print(F(">> Lane LEDs OFF ["));
    Serial.print(getStripSelectName(laneStripSelect));
    Serial.println(F("]"));
  }
  else if (cmd == "white" || cmd == "w") {
    currentLaneColor = C_WHITE(); laneColorName = "WHITE";
    animMode = ANIM_IDLE; laneSetColor(currentLaneColor); laneShowSelected();
    Serial.println(F(">> Color: WHITE"));
  }
  else if (cmd == "red" || cmd == "re") {
    currentLaneColor = C_RED(); laneColorName = "RED";
    animMode = ANIM_IDLE; laneSetColor(currentLaneColor); laneShowSelected();
    Serial.println(F(">> Color: RED"));
  }
  else if (cmd == "green" || cmd == "g") {
    currentLaneColor = C_GREEN(); laneColorName = "GREEN";
    animMode = ANIM_IDLE; laneSetColor(currentLaneColor); laneShowSelected();
    Serial.println(F(">> Color: GREEN"));
  }
  else if (cmd == "blue" || cmd == "bl") {
    currentLaneColor = C_BLUE(); laneColorName = "BLUE";
    animMode = ANIM_IDLE; laneSetColor(currentLaneColor); laneShowSelected();
    Serial.println(F(">> Color: BLUE"));
  }
  else if (cmd == "yellow" || cmd == "y") {
    currentLaneColor = C_YELLOW(); laneColorName = "YELLOW";
    animMode = ANIM_IDLE; laneSetColor(currentLaneColor); laneShowSelected();
    Serial.println(F(">> Color: YELLOW"));
  }
  else if (cmd == "purple" || cmd == "p") {
    currentLaneColor = C_PURPLE(); laneColorName = "PURPLE";
    animMode = ANIM_IDLE; laneSetColor(currentLaneColor); laneShowSelected();
    Serial.println(F(">> Color: PURPLE"));
  }
  else if (cmd == "orange" || cmd == "o") {
    currentLaneColor = C_ORANGE(); laneColorName = "ORANGE";
    animMode = ANIM_IDLE; laneSetColor(currentLaneColor); laneShowSelected();
    Serial.println(F(">> Color: ORANGE"));
  }
  else if (cmd == "wipe" || cmd == "wi") {
    ledAnimTarget = LED_LANE;
    startWipeAnim(currentLaneColor);
    Serial.println(F(">> Wipe animation"));
  }
  else if (cmd == "strike" || cmd == "st") {
    ledAnimTarget = LED_LANE;
    laneSetBrightnessSelected(40);
    startWipeAnim(C_RED());
    Serial.println(F(">> Strike animation"));
  }
  else if (cmd == "flash" || cmd == "fl") {
    ledAnimTarget = LED_LANE;
    startFlashAnim(currentLaneColor);
    Serial.println(F(">> Flash animation"));
  }
  else if (cmd == "comet" || cmd == "cm") {
    ledAnimTarget = LED_LANE;
    startCometAnim();
    Serial.println(F(">> Comet animation"));
  }
  else if (cmd == "rainbow" || cmd == "rb") {
    ledAnimTarget = LED_LANE;
    startRainbowAnim();
    Serial.println(F(">> Rainbow animation"));
  }
  else if (cmd == "stop" || cmd == "x") {
    animMode = ANIM_IDLE;
    laneSetBrightnessSelected(LED_BRIGHTNESS_NORMAL);
    laneSetColor(currentLaneColor);
    laneShowSelected();
    Serial.println(F(">> Animation stopped"));
  }
  else if (cmd.startsWith("br")) {
    int br = cmd.substring(2).toInt();
    br = constrain(br, 0, 255);
    laneSetBrightnessSelected(br);
    laneShowSelected();
    Serial.print(F(">> Brightness: "));
    Serial.println(br);
  }
  else if (cmd == "status" || cmd == "s") {
    Serial.print(F("Color: "));
    Serial.print(laneColorName);
    Serial.print(F(", Brightness: "));
    Serial.print(laneL.getBrightness());
    Serial.print(F(", Target: "));
    Serial.println(getStripSelectName(laneStripSelect));
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

// =====================================================
// DECK LED MENU
// =====================================================

void printDeckLEDMenu() {
  Serial.println(F(""));
  Serial.println(F("===== DECK LED TEST (pins 50,51) ====="));
  Serial.println(F("Strip Selection:"));
  Serial.println(F("  left (l)  - Control LEFT strip only (pin 50)"));
  Serial.println(F("  right (r) - Control RIGHT strip only (pin 51)"));
  Serial.println(F("  both (bo) - Control BOTH strips (default)"));
  Serial.println(F(""));
  Serial.println(F("On/Off:"));
  Serial.println(F("  on (1)  - LEDs on (current color)"));
  Serial.println(F("  off (0) - LEDs off"));
  Serial.println(F(""));
  Serial.println(F("Colors:"));
  Serial.println(F("  white (w), red (re), green (g), blue (bl)"));
  Serial.println(F("  yellow (y), purple (p), orange (o)"));
  Serial.println(F(""));
  Serial.println(F("Animations:"));
  Serial.println(F("  wipe (wi)   - Color wipe"));
  Serial.println(F("  strike (st) - Strike effect"));
  Serial.println(F("  flash (fl)  - Flash effect"));
  Serial.println(F("  comet (cm)  - Comet effect"));
  Serial.println(F("  rainbow (rb)- Rainbow cycle"));
  Serial.println(F("  stop (x)    - Stop animation"));
  Serial.println(F(""));
  Serial.println(F("Settings:"));
  Serial.println(F("  br80        - Set brightness (0-255)"));
  Serial.println(F(""));
  Serial.println(F("  status (s)  - Show status"));
  Serial.println(F("  back (b)    - Return to main menu"));
  Serial.println(F("========================================"));
  Serial.println(F(""));
}

void handleDeckLEDMenu(String cmd) {
  // Strip selection commands
  if (cmd == "left" || cmd == "l") {
    deckStripSelect = STRIP_LEFT;
    Serial.println(F(">> Selected: LEFT strip only"));
    return;
  }
  else if (cmd == "right" || cmd == "r") {
    deckStripSelect = STRIP_RIGHT;
    Serial.println(F(">> Selected: RIGHT strip only"));
    return;
  }
  else if (cmd == "both" || cmd == "bo") {
    deckStripSelect = STRIP_BOTH;
    Serial.println(F(">> Selected: BOTH strips"));
    return;
  }

  if (cmd == "on" || cmd == "1") {
    animMode = ANIM_IDLE;
    deckSetColor(currentDeckColor);
    deckShowSelected();
    Serial.print(F(">> Deck LEDs ON ("));
    Serial.print(deckColorName);
    Serial.print(F(") ["));
    Serial.print(getStripSelectName(deckStripSelect));
    Serial.println(F("]"));
  }
  else if (cmd == "off" || cmd == "0") {
    animMode = ANIM_IDLE;
    deckSetColor(C_OFF());
    deckShowSelected();
    Serial.print(F(">> Deck LEDs OFF ["));
    Serial.print(getStripSelectName(deckStripSelect));
    Serial.println(F("]"));
  }
  else if (cmd == "white" || cmd == "w") {
    currentDeckColor = C_WHITE(); deckColorName = "WHITE";
    animMode = ANIM_IDLE; deckSetColor(currentDeckColor); deckShowSelected();
    Serial.println(F(">> Color: WHITE"));
  }
  else if (cmd == "red" || cmd == "re") {
    currentDeckColor = C_RED(); deckColorName = "RED";
    animMode = ANIM_IDLE; deckSetColor(currentDeckColor); deckShowSelected();
    Serial.println(F(">> Color: RED"));
  }
  else if (cmd == "green" || cmd == "g") {
    currentDeckColor = C_GREEN(); deckColorName = "GREEN";
    animMode = ANIM_IDLE; deckSetColor(currentDeckColor); deckShowSelected();
    Serial.println(F(">> Color: GREEN"));
  }
  else if (cmd == "blue" || cmd == "bl") {
    currentDeckColor = C_BLUE(); deckColorName = "BLUE";
    animMode = ANIM_IDLE; deckSetColor(currentDeckColor); deckShowSelected();
    Serial.println(F(">> Color: BLUE"));
  }
  else if (cmd == "yellow" || cmd == "y") {
    currentDeckColor = C_YELLOW(); deckColorName = "YELLOW";
    animMode = ANIM_IDLE; deckSetColor(currentDeckColor); deckShowSelected();
    Serial.println(F(">> Color: YELLOW"));
  }
  else if (cmd == "purple" || cmd == "p") {
    currentDeckColor = C_PURPLE(); deckColorName = "PURPLE";
    animMode = ANIM_IDLE; deckSetColor(currentDeckColor); deckShowSelected();
    Serial.println(F(">> Color: PURPLE"));
  }
  else if (cmd == "orange" || cmd == "o") {
    currentDeckColor = C_ORANGE(); deckColorName = "ORANGE";
    animMode = ANIM_IDLE; deckSetColor(currentDeckColor); deckShowSelected();
    Serial.println(F(">> Color: ORANGE"));
  }
  else if (cmd == "wipe" || cmd == "wi") {
    ledAnimTarget = LED_DECK;
    startWipeAnim(currentDeckColor);
    Serial.println(F(">> Wipe animation"));
  }
  else if (cmd == "strike" || cmd == "st") {
    ledAnimTarget = LED_DECK;
    deckSetBrightnessSelected(40);
    startWipeAnim(C_RED());
    Serial.println(F(">> Strike animation"));
  }
  else if (cmd == "flash" || cmd == "fl") {
    ledAnimTarget = LED_DECK;
    startFlashAnim(currentDeckColor);
    Serial.println(F(">> Flash animation"));
  }
  else if (cmd == "comet" || cmd == "cm") {
    ledAnimTarget = LED_DECK;
    startCometAnim();
    Serial.println(F(">> Comet animation"));
  }
  else if (cmd == "rainbow" || cmd == "rb") {
    ledAnimTarget = LED_DECK;
    startRainbowAnim();
    Serial.println(F(">> Rainbow animation"));
  }
  else if (cmd == "stop" || cmd == "x") {
    animMode = ANIM_IDLE;
    deckSetBrightnessSelected(DECK_LED_BRIGHTNESS);
    deckSetColor(currentDeckColor);
    deckShowSelected();
    Serial.println(F(">> Animation stopped"));
  }
  else if (cmd.startsWith("br")) {
    int br = cmd.substring(2).toInt();
    br = constrain(br, 0, 255);
    deckSetBrightnessSelected(br);
    deckShowSelected();
    Serial.print(F(">> Brightness: "));
    Serial.println(br);
  }
  else if (cmd == "status" || cmd == "s") {
    Serial.print(F("Color: "));
    Serial.print(deckColorName);
    Serial.print(F(", Brightness: "));
    Serial.print(deckL.getBrightness());
    Serial.print(F(", Target: "));
    Serial.println(getStripSelectName(deckStripSelect));
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

// =====================================================
// SCISSOR MENU
// =====================================================

void printScissorMenu() {
  Serial.println(F(""));
  Serial.println(F("===== SCISSOR TEST (pin 7) ====="));
  Serial.print(F("  home (ho)  - Home position ("));
  Serial.print(SCISSOR_DROP_ANGLE);
  Serial.println(F(" deg)"));
  Serial.print(F("  grab (g)   - Grab/closed ("));
  Serial.print(SCISSOR_GRAB_ANGLE);
  Serial.println(F(" deg)"));
  Serial.print(F("  drop (d)   - Drop/open ("));
  Serial.print(SCISSOR_DROP_ANGLE);
  Serial.println(F(" deg)"));
  Serial.println(F("  a120       - Custom angle (0-180)"));
  Serial.print(F("  cycle (cy) - Cycle grab/drop every "));
  Serial.print(SCISSOR_CYCLE_MS / 1000.0, 1);
  Serial.println(F("s"));
  Serial.println(F("  cy,90,140  - Cycle between custom angles"));
  Serial.println(F("  stop (x)   - Stop cycling"));
  Serial.println(F("  disengage (de) - Detach servo"));
  Serial.println(F("  status (s) - Show status"));
  Serial.println(F("  back (b)   - Return to main menu"));
  Serial.println(F("================================="));
  Serial.println(F(""));
}

void handleScissorMenu(String cmd) {
  if (cmd == "home" || cmd == "ho") {
    ensureScissorAttached();
    scissorAngle = SCISSOR_DROP_ANGLE;
    ScissorsServo.write(scissorAngle);
    Serial.print(F(">> HOME ("));
    Serial.print(scissorAngle);
    Serial.println(F(")"));
  }
  else if (cmd == "grab" || cmd == "g") {
    ensureScissorAttached();
    scissorAngle = SCISSOR_GRAB_ANGLE;
    ScissorsServo.write(scissorAngle);
    Serial.print(F(">> GRAB ("));
    Serial.print(scissorAngle);
    Serial.println(F(")"));
  }
  else if (cmd == "drop" || cmd == "d") {
    ensureScissorAttached();
    scissorAngle = SCISSOR_DROP_ANGLE;
    ScissorsServo.write(scissorAngle);
    Serial.print(F(">> DROP ("));
    Serial.print(scissorAngle);
    Serial.println(F(")"));
  }
  else if (cmd.startsWith("a")) {
    ensureScissorAttached();
    int angle = cmd.substring(1).toInt();
    angle = constrain(angle, 0, 180);
    scissorAngle = angle;
    ScissorsServo.write(scissorAngle);
    Serial.print(F(">> Angle: "));
    Serial.println(scissorAngle);
  }
  else if (cmd == "cycle" || cmd == "cy" || cmd.startsWith("cycle,") || cmd.startsWith("cycle ") || cmd.startsWith("cy,") || cmd.startsWith("cy ")) {
    // Parse optional custom angles: "cycle,90,140" or "cy 90 140" etc.
    int angle1 = SCISSOR_GRAB_ANGLE;
    int angle2 = SCISSOR_DROP_ANGLE;

    // Find where command ends and arguments begin
    int argStart = -1;
    if (cmd.startsWith("cycle,") || cmd.startsWith("cycle ")) argStart = 6;
    else if (cmd.startsWith("cy,") || cmd.startsWith("cy ")) argStart = 3;

    if (argStart > 0) {
      String args = cmd.substring(argStart);
      args.trim();
      // Replace commas with spaces for uniform parsing
      args.replace(",", " ");
      args.replace("  ", " ");
      args.trim();

      int spaceIdx = args.indexOf(' ');
      if (spaceIdx > 0) {
        String arg1 = args.substring(0, spaceIdx);
        String arg2 = args.substring(spaceIdx + 1);
        arg1.trim();
        arg2.trim();
        angle1 = constrain(arg1.toInt(), 0, 180);
        angle2 = constrain(arg2.toInt(), 0, 180);
      }
    }

    ensureScissorAttached();
    scissorCycling = true;
    scissorCyclePhase = false;
    scissorCycleAngle1 = angle1;
    scissorCycleAngle2 = angle2;
    scissorAngle = scissorCycleAngle1;
    ScissorsServo.write(scissorAngle);
    scissorCycleLastMs = millis();
    Serial.print(F(">> Cycling "));
    Serial.print(scissorCycleAngle1);
    Serial.print(F("/"));
    Serial.print(scissorCycleAngle2);
    Serial.print(F(" ("));
    Serial.print(SCISSOR_CYCLE_MS / 1000.0, 1);
    Serial.println(F("s interval)..."));
  }
  else if (cmd == "stop" || cmd == "x") {
    scissorCycling = false;
    Serial.println(F(">> Cycling stopped"));
  }
  else if (cmd == "disengage" || cmd == "de") {
    scissorCycling = false;
    ScissorsServo.detach();
    Serial.println(F(">> Scissor servo DISENGAGED"));
  }
  else if (cmd == "status" || cmd == "s") {
    Serial.print(F("Scissor angle: "));
    Serial.print(scissorAngle);
    if (!ScissorsServo.attached()) {
      Serial.println(F(" (disengaged)"));
    } else if (scissorCycling) {
      Serial.println(F(" (CYCLING)"));
    } else if (scissorAngle >= 130) {
      Serial.println(F(" (GRAB)"));
    } else if (scissorAngle <= 95) {
      Serial.println(F(" (DROP)"));
    } else {
      Serial.println(F(""));
    }
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

void updateScissorCycle() {
  if (!scissorCycling) return;
  unsigned long now = millis();
  if (now - scissorCycleLastMs >= SCISSOR_CYCLE_MS) {
    scissorCycleLastMs = now;
    scissorCyclePhase = !scissorCyclePhase;
    if (scissorCyclePhase) {
      scissorAngle = scissorCycleAngle2;
    } else {
      scissorAngle = scissorCycleAngle1;
    }
    ScissorsServo.write(scissorAngle);
    Serial.print(F(">> Cycle: "));
    Serial.print(scissorAngle);
    Serial.println(F(" (x to stop)"));
  }
}

// =====================================================
// SLIDER MENU
// =====================================================

void printSliderMenu() {
  Serial.println(F(""));
  Serial.println(F("===== SLIDING DECK TEST (pin 8) ====="));
  Serial.print(F("  home (ho)    - Home / Catch position ("));
  Serial.print(SLIDER_HOME_ANGLE);
  Serial.println(F(" deg)"));
  Serial.print(F("  release (re) - Release position ("));
  Serial.print(SLIDER_RELEASE_ANGLE);
  Serial.println(F(" deg)"));
  Serial.println(F("  a150         - Custom angle (0-180)"));
  Serial.println(F("  disengage (de) - Detach servo"));
  Serial.println(F("  status (s)   - Show status"));
  Serial.println(F("  back (b)     - Return to main menu"));
  Serial.println(F("================================"));
  Serial.println(F(""));
}

void handleSliderMenu(String cmd) {
  if (cmd == "home" || cmd == "ho") {
    ensureSliderAttached();
    sliderAngle = SLIDER_HOME_ANGLE;
    SlideServo.write(sliderAngle);
    Serial.print(F(">> HOME ("));
    Serial.print(sliderAngle);
    Serial.println(F(")"));
  }
  else if (cmd == "release" || cmd == "re") {
    ensureSliderAttached();
    sliderAngle = SLIDER_RELEASE_ANGLE;
    SlideServo.write(sliderAngle);
    Serial.print(F(">> RELEASE ("));
    Serial.print(sliderAngle);
    Serial.println(F(")"));
  }
  else if (cmd.startsWith("a")) {
    ensureSliderAttached();
    int angle = cmd.substring(1).toInt();
    angle = constrain(angle, 0, 180);
    sliderAngle = angle;
    SlideServo.write(sliderAngle);
    Serial.print(F(">> Angle: "));
    Serial.println(sliderAngle);
  }
  else if (cmd == "disengage" || cmd == "de") {
    SlideServo.detach();
    Serial.println(F(">> Sliding deck servo DISENGAGED"));
  }
  else if (cmd == "status" || cmd == "s") {
    Serial.print(F("Sliding deck angle: "));
    Serial.print(sliderAngle);
    if (!SlideServo.attached()) {
      Serial.println(F(" (disengaged)"));
    } else if (sliderAngle >= 175) {
      Serial.println(F(" (HOME/CATCH)"));
    } else if (sliderAngle <= 105) {
      Serial.println(F(" (RELEASE)"));
    } else {
      Serial.println(F(""));
    }
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

// =====================================================
// RAISE MENU
// =====================================================

void printRaiseMenu() {
  Serial.println(F(""));
  Serial.println(F("===== RAISE TEST (pins 9,10) ====="));
  Serial.println(F("Presets (both servos move together):"));
  Serial.print(F("  home (ho)- Home/Up (L="));
  Serial.print(RAISE_UP_ANGLE);
  Serial.print(F(", R="));
  Serial.print(180 - RAISE_UP_ANGLE);
  Serial.println(F(")"));
  Serial.print(F("  up (u)   - Up position (L="));
  Serial.print(RAISE_UP_ANGLE);
  Serial.print(F(", R="));
  Serial.print(180 - RAISE_UP_ANGLE);
  Serial.println(F(")"));
  Serial.print(F("  down (d) - Down/Set (L="));
  Serial.print(RAISE_DOWN_ANGLE);
  Serial.print(F(", R="));
  Serial.print(180 - RAISE_DOWN_ANGLE);
  Serial.println(F(")"));
  Serial.print(F("  grab (g) - Grab (L="));
  Serial.print(RAISE_GRAB_ANGLE);
  Serial.print(F(", R="));
  Serial.print(180 - RAISE_GRAB_ANGLE);
  Serial.println(F(")"));
  Serial.print(F("  drop (dr)- Drop (L="));
  Serial.print(RAISE_DROP_ANGLE);
  Serial.print(F(", R="));
  Serial.print(180 - RAISE_DROP_ANGLE);
  Serial.println(F(")"));
  Serial.println(F(""));
  Serial.println(F("Custom angle:"));
  Serial.println(F("  a60      - Set angle (L=60, R=120)"));
  Serial.println(F(""));
  Serial.println(F("  disengage (de) - Detach servos"));
  Serial.println(F("  status (s) - Show status"));
  Serial.println(F("  back (b)   - Return to main menu"));
  Serial.println(F("==================================="));
  Serial.println(F(""));
}

void handleRaiseMenu(String cmd) {
  if (cmd == "home" || cmd == "ho") {
    ensureRaiseAttached();
    raiseLeftPos = RAISE_UP_ANGLE;
    raiseRightPos = 180 - RAISE_UP_ANGLE;
    LeftRaiseServo.write(raiseLeftPos);
    RightRaiseServo.write(raiseRightPos);
    Serial.print(F(">> HOME (L="));
    Serial.print(raiseLeftPos);
    Serial.print(F(", R="));
    Serial.print(raiseRightPos);
    Serial.println(F(")"));
  }
  else if (cmd == "up" || cmd == "u") {
    ensureRaiseAttached();
    raiseLeftPos = RAISE_UP_ANGLE;
    raiseRightPos = 180 - RAISE_UP_ANGLE;
    LeftRaiseServo.write(raiseLeftPos);
    RightRaiseServo.write(raiseRightPos);
    Serial.print(F(">> UP (L="));
    Serial.print(raiseLeftPos);
    Serial.print(F(", R="));
    Serial.print(raiseRightPos);
    Serial.println(F(")"));
  }
  else if (cmd == "down" || cmd == "d") {
    ensureRaiseAttached();
    raiseLeftPos = RAISE_DOWN_ANGLE;
    raiseRightPos = 180 - RAISE_DOWN_ANGLE;
    LeftRaiseServo.write(raiseLeftPos);
    RightRaiseServo.write(raiseRightPos);
    Serial.print(F(">> DOWN (L="));
    Serial.print(raiseLeftPos);
    Serial.print(F(", R="));
    Serial.print(raiseRightPos);
    Serial.println(F(")"));
  }
  else if (cmd == "grab" || cmd == "g") {
    ensureRaiseAttached();
    raiseLeftPos = RAISE_GRAB_ANGLE;
    raiseRightPos = 180 - RAISE_GRAB_ANGLE;
    LeftRaiseServo.write(raiseLeftPos);
    RightRaiseServo.write(raiseRightPos);
    Serial.print(F(">> GRAB (L="));
    Serial.print(raiseLeftPos);
    Serial.print(F(", R="));
    Serial.print(raiseRightPos);
    Serial.println(F(")"));
  }
  else if (cmd == "drop" || cmd == "dr") {
    ensureRaiseAttached();
    raiseLeftPos = RAISE_DROP_ANGLE;
    raiseRightPos = 180 - RAISE_DROP_ANGLE;
    LeftRaiseServo.write(raiseLeftPos);
    RightRaiseServo.write(raiseRightPos);
    Serial.print(F(">> DROP (L="));
    Serial.print(raiseLeftPos);
    Serial.print(F(", R="));
    Serial.print(raiseRightPos);
    Serial.println(F(")"));
  }
  else if (cmd.startsWith("a")) {
    ensureRaiseAttached();
    int angle = cmd.substring(1).toInt();
    angle = constrain(angle, 0, 180);
    raiseLeftPos = angle;
    raiseRightPos = 180 - angle;
    LeftRaiseServo.write(raiseLeftPos);
    RightRaiseServo.write(raiseRightPos);
    Serial.print(F(">> Angle: L="));
    Serial.print(raiseLeftPos);
    Serial.print(F(", R="));
    Serial.println(raiseRightPos);
  }
  else if (cmd == "disengage" || cmd == "de") {
    LeftRaiseServo.detach();
    RightRaiseServo.detach();
    Serial.println(F(">> Raise servos DISENGAGED"));
  }
  else if (cmd == "status" || cmd == "s") {
    Serial.print(F("Current: L="));
    Serial.print(raiseLeftPos);
    Serial.print(F(", R="));
    Serial.print(raiseRightPos);
    if (!LeftRaiseServo.attached()) {
      Serial.println(F(" (disengaged)"));
    } else {
      Serial.println(F(""));
    }
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

// =====================================================
// TURRET MENU
// =====================================================

void printTurretMenu() {
  Serial.println(F(""));
  Serial.println(F("===== TURRET TEST (pins 2,3) ====="));
  Serial.println(F("Homing:"));
  Serial.println(F("  home (ho) - Run homing sequence"));
  Serial.println(F("  adjust (adj) - Tune home adjuster value"));
  Serial.println(F(""));
  Serial.println(F("Slot positions:"));
  Serial.println(F("  1-9       - Move to slot 1-9"));
  Serial.println(F("  0         - Move to release position"));
  Serial.println(F(""));
  Serial.println(F("Manual jog:"));
  Serial.println(F("  +         - Jog forward 50 steps"));
  Serial.println(F("  -         - Jog backward 50 steps"));
  Serial.println(F("  ++        - Jog forward 200 steps"));
  Serial.println(F("  --        - Jog backward 200 steps"));
  Serial.println(F(""));
  Serial.println(F("Utility:"));
  Serial.println(F("  stop (x)  - Emergency stop"));
  Serial.println(F("  zero (z)  - Zero current position"));
#ifdef STEPPER_ENABLE_PIN
  Serial.println(F("  disengage (de) - Disable stepper"));
#endif
  Serial.println(F("  status (s)- Show status"));
  Serial.println(F("  back (b)  - Return to main menu"));
  Serial.println(F("==================================="));
  Serial.println(F(""));
}

void handleTurretMenu(String cmd) {
  // Home adjuster mode intercept
  if (homeAdjustActive && homingPhase == HOME_ADJUST_READY) {
    if (cmd == "+") {
      homeAdjustValue += 1;
      Serial.print(F(">> Adjust +1 -> "));
      Serial.print(homeAdjustValue);
      Serial.println(F("  (done to save, cancel to abort)"));
      turretMoveRelative(-1);
    }
    else if (cmd == "-") {
      homeAdjustValue -= 1;
      Serial.print(F(">> Adjust -1 -> "));
      Serial.print(homeAdjustValue);
      Serial.println(F("  (done to save, cancel to abort)"));
      turretMoveRelative(1);
    }
    else if (cmd == "++") {
      homeAdjustValue += 5;
      Serial.print(F(">> Adjust +5 -> "));
      Serial.print(homeAdjustValue);
      Serial.println(F("  (done to save, cancel to abort)"));
      turretMoveRelative(-5);
    }
    else if (cmd == "--") {
      homeAdjustValue -= 5;
      Serial.print(F(">> Adjust -5 -> "));
      Serial.print(homeAdjustValue);
      Serial.println(F("  (done to save, cancel to abort)"));
      turretMoveRelative(5);
    }
    else if (cmd == "done") {
      Serial.println(F(""));
      Serial.println(F(">> ADJUSTMENT ACCEPTED"));
      Serial.print(F(">> Final value: "));
      Serial.println(homeAdjustValue);
      Serial.println(F(">> Add this line to general_config.user.h:"));
      Serial.print(F(">>   #define TURRET_HOME_ADJUSTER "));
      Serial.println(homeAdjustValue);
      Serial.println(F(""));
      turretIsHomed = false;  // Adjuster changed, require re-home
      homeAdjustActive = false;
      homingActive = false;
      homingPhase = HOME_IDLE;
    }
    else if (cmd == "cancel") {
      Serial.println(F(">> Adjustment CANCELLED, reverted to original value"));
      homeAdjustValue = TURRET_HOME_ADJUSTER;
      homeAdjustActive = false;
      homingActive = false;
      homingPhase = HOME_IDLE;
    }
    else {
      // Check if input is a valid number
      bool isNumber = (cmd.length() > 0);
      for (unsigned int i = 0; i < cmd.length(); i++) {
        if (i == 0 && cmd.charAt(i) == '-') continue;
        if (cmd.charAt(i) < '0' || cmd.charAt(i) > '9') { isNumber = false; break; }
      }
      if (isNumber && cmd != "-") {
        int newValue = cmd.toInt();
        if (newValue < -130 || newValue > 130) {
          Serial.println(F(">> ERROR: Value must be between -130 and 130"));
        } else {
          int delta = newValue - homeAdjustValue;
          homeAdjustValue = newValue;
          Serial.print(F(">> Set to "));
          Serial.print(homeAdjustValue);
          Serial.println(F("  (done to save, cancel to abort)"));
          turretMoveRelative(-delta);
        }
      } else {
        Serial.print(F(">> Unknown adjust command: "));
        Serial.println(cmd);
      }
    }
    return;
  }

  if (cmd == "home" || cmd == "ho") {
    if (!hallMonEnabled) {
      Serial.println(F(">> ERROR: Hall sensor monitoring is disabled."));
      Serial.println(F(">> Homing requires the Hall sensor."));
      Serial.println(F(">> Re-enable it in the sensor menu first."));
    } else {
      startTurretHome();
    }
  }
  else if (cmd == "adjust" || cmd == "adj") {
    if (!hallMonEnabled) {
      Serial.println(F(">> ERROR: Hall sensor monitoring is disabled."));
      Serial.println(F(">> Homing requires the Hall sensor."));
      Serial.println(F(">> Re-enable it in the sensor menu first."));
    } else {
      homeAdjustActive = true;
      homeAdjustValue = TURRET_HOME_ADJUSTER;
      startTurretHome();
    }
  }
  else if (cmd == "0") {
    long releasePos = PinPositions[10] + TURRET_PIN10_RELEASE_OFFSET;
    Serial.print(F(">> Moving to RELEASE: "));
    Serial.print(releasePos);
    Serial.println(F(" (spring-safe speed)"));
    stepper.setMaxSpeed(TURRET_SPRING_MAXSPEED);
    stepper.setAcceleration(TURRET_SPRING_ACCEL);
    turretGoTo(releasePos);
  }
  else if (cmd.length() == 1 && cmd.charAt(0) >= '1' && cmd.charAt(0) <= '9') {
    int slot = cmd.charAt(0) - '0';
    Serial.print(F(">> Moving to slot "));
    Serial.print(slot);
    Serial.print(F(": "));
    Serial.println(PinPositions[slot]);
    stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
    stepper.setAcceleration(TURRET_NORMAL_ACCEL);
    turretGoTo(PinPositions[slot]);
  }
  else if (cmd == "+") {
    Serial.println(F(">> Jog +50"));
    turretMoveRelative(50);
  }
  else if (cmd == "-") {
    Serial.println(F(">> Jog -50"));
    turretMoveRelative(-50);
  }
  else if (cmd == "++") {
    Serial.println(F(">> Jog +200"));
    turretMoveRelative(200);
  }
  else if (cmd == "--") {
    Serial.println(F(">> Jog -200"));
    turretMoveRelative(-200);
  }
  else if (cmd == "stop" || cmd == "x") {
    stepper.stop();
    stepper.setCurrentPosition(stepper.currentPosition());
    turretMoving = false;
    homingActive = false;
    homingPhase = HOME_IDLE;
    Serial.println(F(">> EMERGENCY STOP"));
  }
  else if (cmd == "zero" || cmd == "z") {
    stepper.setCurrentPosition(0);
    turretTargetPos = 0;
    Serial.println(F(">> Position zeroed"));
  }
#ifdef STEPPER_ENABLE_PIN
  else if (cmd == "disengage" || cmd == "de") {
    stepper.stop();
    stepper.disableOutputs();
    turretMoving = false;
    homingActive = false;
    homingPhase = HOME_IDLE;
    Serial.println(F(">> Turret stepper DISENGAGED"));
  }
#endif
  else if (cmd == "status" || cmd == "s") {
    Serial.println(F(""));
    Serial.print(F("Position: "));
    Serial.println(stepper.currentPosition());
    Serial.print(F("Target: "));
    Serial.println(turretTargetPos);
    Serial.print(F("Moving: "));
    Serial.println(turretMoving ? "YES" : "NO");
    Serial.print(F("Homing: "));
    Serial.println(homingActive ? "YES" : "NO");
    Serial.println(F(""));
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

void turretGoTo(long pos) {
#ifdef STEPPER_ENABLE_PIN
  stepper.enableOutputs();
#endif
  if (turretTargetPos != pos) {
    turretTargetPos = pos;
    stepper.moveTo(turretTargetPos);
    turretMoving = true;
  }
}

void turretMoveRelative(long steps) {
  long newPos = stepper.currentPosition() + steps;
  turretGoTo(newPos);
}

void startTurretHome(long minSteps) {
  Serial.println(F(">> Starting homing sequence..."));
  homingActive = true;
  homingMinSteps = minSteps;
  stepper.setAcceleration(3000);
  stepper.setMaxSpeed(500);
  turretTargetPos = stepper.currentPosition();  // Reset so first turretGoTo() isn't suppressed
  homingStartPos = stepper.currentPosition();

  if (minSteps > 0) {
    // Caller knows we're near the magnet — advance past it first
    Serial.print(F(">> Advancing past magnet (min "));
    Serial.print(minSteps);
    Serial.println(F(" steps)..."));
    homingPhase = HOME_ADVANCE_TO_SWITCH;
  } else {
    int hallState = digitalRead(HALL_EFFECT_PIN);
    if (hallState == LOW) {
      // Hall sensor already engaged — back off first so we can re-find it precisely
      Serial.println(F(">> Magnet detected at start, backing off..."));
      homingPhase = HOME_BACKOFF;
      turretGoTo(stepper.currentPosition() - 150);
    } else {
      // Hall sensor not engaged — go straight to searching for it (clockwise)
      Serial.println(F(">> Advancing to find magnet..."));
      homingPhase = HOME_ADVANCE_TO_SWITCH;
    }
  }
}

void runHomingFSM() {
  if (!homingActive) return;

  int hallState = digitalRead(HALL_EFFECT_PIN);

  switch (homingPhase) {
    case HOME_ADVANCE_TO_SWITCH: {
      long traveled = stepper.currentPosition() - homingStartPos;
      if (traveled < 0) traveled = -traveled;
      bool pastMinSteps = (traveled >= homingMinSteps);

      if (!pastMinSteps || hallState == HIGH) {
        if (stepper.distanceToGo() == 0) {
          // This was previously + 10, and we've increased this to + 2000. This is moving the turret
          // a lot, instead of 10 steps at a time, and relying on the hall effect sensor to be
          // triggered which will stop the turret. This makes the homing process go much faster.
          turretGoTo(stepper.currentPosition() + 2000);
        }
      } else {
        Serial.println(F(">> Magnet found! Backing off..."));
        homingPhase = HOME_BACKOFF;
        turretGoTo(stepper.currentPosition() - 150);
      }
      break;
    }

    case HOME_BACKOFF:
      if (stepper.distanceToGo() == 0) {
        Serial.println(F(">> Creeping to find precise position..."));
        stepper.setMaxSpeed(100);
        homingPhase = HOME_CREEP_TO_SWITCH;
      }
      break;

    case HOME_CREEP_TO_SWITCH:
      if (hallState == HIGH) {
        if (stepper.distanceToGo() == 0) {
          turretGoTo(stepper.currentPosition() + 2);
        }
      } else {
        Serial.println(F(">> Precise position found!"));
        stepper.setCurrentPosition(homeAdjustValue);
        stepper.setMaxSpeed(500);
        stepper.setAcceleration(3000);
        turretGoTo(PinPositions[1]);
        homingPhase = HOME_SETZERO_AND_MOVE_SLOT1;
      }
      break;

    case HOME_SETZERO_AND_MOVE_SLOT1:
      if (stepper.distanceToGo() == 0) {
        homingPhase = HOME_DONE;
      }
      break;

    case HOME_DONE:
      stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
      stepper.setAcceleration(TURRET_NORMAL_ACCEL);
      Serial.println(F(">> HOMING COMPLETE"));
      Serial.print(F("   Position: "));
      Serial.println(stepper.currentPosition());
      if (homeAdjustActive) {
        homingPhase = HOME_ADJUST_READY;
        Serial.println(F(""));
        Serial.println(F(">> HOME ADJUSTER TUNING"));
        Serial.print(F(">> Current value: "));
        Serial.println(homeAdjustValue);
        Serial.println(F(">> Adjust until turret marks line up with turret plate."));
        Serial.println(F(">>   +       Adjust +1 step"));
        Serial.println(F(">>   -       Adjust -1 step"));
        Serial.println(F(">>   ++      Adjust +5 steps"));
        Serial.println(F(">>   --      Adjust -5 steps"));
        Serial.println(F(">>   <num>   Set to specific value"));
        Serial.println(F(">>   done    Accept & show config line"));
        Serial.println(F(">>   cancel  Discard changes"));
      } else {
        turretIsHomed = true;
        homingActive = false;
        homingPhase = HOME_IDLE;
      }
      break;

    case HOME_ADJUST_READY:
      // Waiting for user commands — handled in handleTurretMenu
      break;

    default:
      break;
  }
}

// =====================================================
// SEQUENCE MENU
// =====================================================

void printSequenceMenu() {
  Serial.println(F(""));
  Serial.println(F("======== SEQUENCE MENU ========"));
  Serial.println(F("  sweep (sw)        - Clear pins from lane"));
  Serial.println(F("  pindrop (pd)      - Drop pins from deck"));
  Serial.println(F("  turretload (tl)   - Load turret with 10 pins"));
  Serial.println(F("  pinpickup (pp)    - Pick up pins from lane"));
  Serial.println(F("  pinset (ps)       - Set held pins back on lane"));
  Serial.println(F("  pincycle (pc)     - Cycle pickup/set repeatedly"));
  Serial.println(F("  full (fl) [N]     - Full cycle: load/drop/pickup/set/sweep"));
  Serial.println(F("                      e.g. 'full 10' to run 10 cycles"));
  Serial.println(F("  clear (cl)        - Clear all pins (lane, deck, turret)"));
  Serial.println(F("  convtime (ct)     - Conveyor timing test"));
  Serial.println(F(""));
  Serial.println(F("  debug (de)        - Show turret load timing data"));
  Serial.println(F("  stop (x/<Enter>)  - Stop running sequence"));
  Serial.println(F("  back (b)          - Return to main menu"));
  Serial.println(F("==============================="));
  Serial.println(F(""));
}

void handleSequenceMenu(String cmd) {
  // If a prompt is active, route input to the prompt handler
  if (seqPrompt != SEQPROMPT_NONE) {
    if (cmd == "y" || cmd == "yes") {
      if (seqPrompt == SEQPROMPT_PD_PRECLEAR) {
        pinDropPreClear = true;
        seqPrompt = SEQPROMPT_PD_POSTCLEAR;
        Serial.println(F("Clear pins after dropping? (Y/n)"));
      } else if (seqPrompt == SEQPROMPT_PD_POSTCLEAR) {
        pinDropPostClear = true;
        seqPrompt = SEQPROMPT_NONE;
        startPinDrop(pinDropPreClear, pinDropPostClear);
      } else if (seqPrompt == SEQPROMPT_TL_CONFIRM) {
        seqPrompt = SEQPROMPT_NONE;
        startTurretLoad();
      } else if (seqPrompt == SEQPROMPT_TL_RESUME) {
        seqPrompt = SEQPROMPT_NONE;
        startTurretLoad();
      } else if (seqPrompt == SEQPROMPT_PP_READY) {
        seqPrompt = SEQPROMPT_NONE;
        startPinPickup();
      } else if (seqPrompt == SEQPROMPT_PS_CONFIRM) {
        seqPrompt = SEQPROMPT_NONE;
        startPinSet();
      } else if (seqPrompt == SEQPROMPT_PC_READY) {
        seqPrompt = SEQPROMPT_NONE;
        pinCycleActive = true;
        startPinPickup();
      } else if (seqPrompt == SEQPROMPT_FULL_PRECLEAR) {
        seqPrompt = SEQPROMPT_NONE;
        turretPinsLoaded = 0;
        startFullTestWithClear();
      } else if (seqPrompt == SEQPROMPT_FULL_CONFIRM) {
        seqPrompt = SEQPROMPT_NONE;
        turretPinsLoaded = 0;
        startFullTest();
      } else if (seqPrompt == SEQPROMPT_TL_TIMING) {
        seqPrompt = SEQPROMPT_NONE;
        printTurretLoadTiming();
      } else if (seqPrompt == SEQPROMPT_CT_PRECLEAR) {
        // Start clearing immediately; FSM will prompt "Ready?" when done
        seqPrompt = SEQPROMPT_NONE;
        Serial.println(F(""));
        Serial.println(F(">> Clearing all pins..."));
        convTimingActive = true;
        startClearAll();
        convTimingPhase = CTIME_PRECLEAR;
      } else if (seqPrompt == SEQPROMPT_CT_READY) {
        seqPrompt = SEQPROMPT_NONE;
        startConveyorTiming();
      } else if (seqPrompt == SEQPROMPT_CT_RAWTIMING) {
        seqPrompt = SEQPROMPT_NONE;
        printTurretLoadTiming();
      }
    } else if (cmd == "n" || cmd == "no") {
      if (seqPrompt == SEQPROMPT_PD_PRECLEAR) {
        pinDropPreClear = false;
        seqPrompt = SEQPROMPT_PD_POSTCLEAR;
        Serial.println(F("Clear pins after dropping? (Y/n)"));
      } else if (seqPrompt == SEQPROMPT_PD_POSTCLEAR) {
        pinDropPostClear = false;
        seqPrompt = SEQPROMPT_NONE;
        startPinDrop(pinDropPreClear, pinDropPostClear);
      } else if (seqPrompt == SEQPROMPT_TL_CONFIRM) {
        seqPrompt = SEQPROMPT_NONE;
        Serial.println(F(">> Turret load cancelled"));
      } else if (seqPrompt == SEQPROMPT_TL_RESUME) {
        seqPrompt = SEQPROMPT_NONE;
        turretPinsLoaded = 0;
        Serial.println(F(">> Pin count reset to 0. Turret load cancelled."));
        Serial.println(F(">> Clear the turret manually if needed, then try again."));
      } else if (seqPrompt == SEQPROMPT_PP_READY) {
        seqPrompt = SEQPROMPT_NONE;
        Serial.println(F(">> Pin pickup cancelled"));
      } else if (seqPrompt == SEQPROMPT_PS_CONFIRM) {
        seqPrompt = SEQPROMPT_NONE;
        Serial.println(F(">> Pin set cancelled"));
      } else if (seqPrompt == SEQPROMPT_PC_READY) {
        seqPrompt = SEQPROMPT_NONE;
        Serial.println(F(">> Pin cycle cancelled"));
      } else if (seqPrompt == SEQPROMPT_FULL_PRECLEAR) {
        seqPrompt = SEQPROMPT_FULL_CONFIRM;
        Serial.println(F("Verify: no pins in turret, sliding deck, scissors, or on lane."));
        Serial.println(F("Continue? (Y/n)"));
      } else if (seqPrompt == SEQPROMPT_FULL_CONFIRM) {
        seqPrompt = SEQPROMPT_NONE;
        Serial.println(F(">> Full test cancelled"));
      } else if (seqPrompt == SEQPROMPT_TL_TIMING) {
        seqPrompt = SEQPROMPT_NONE;
        Serial.println(F(">> Timing data kept. Use 'debug' to view later."));
      } else if (seqPrompt == SEQPROMPT_CT_PRECLEAR) {
        seqPrompt = SEQPROMPT_CT_READY;
        Serial.println(F(""));
        Serial.println(F("Ready to begin the conveyor timing test? (Y/n)"));
      } else if (seqPrompt == SEQPROMPT_CT_READY) {
        seqPrompt = SEQPROMPT_NONE;
        Serial.println(F(">> Conveyor timing test cancelled"));
      } else if (seqPrompt == SEQPROMPT_CT_RAWTIMING) {
        seqPrompt = SEQPROMPT_NONE;
        Serial.println(F(">> Timing data kept. Use 'debug' to view later."));
      }
    } else {
      Serial.println(F(">> Please enter 'y' or 'n'"));
    }
    return;
  }

  if (cmd == "sweep" || cmd == "sw") {
    if (anySequenceRunning()) {
      Serial.println(F(">> A sequence is already running. Stop it first (x)."));
      return;
    }
    Serial.println(F(""));
    Serial.println(F(">> Starting sweep clear..."));
    startSweepClear();
  }
  else if (cmd == "pindrop" || cmd == "pd") {
    if (anySequenceRunning()) {
      Serial.println(F(">> A sequence is already running. Stop it first (x)."));
      return;
    }
    seqPrompt = SEQPROMPT_PD_PRECLEAR;
    Serial.println(F("Clear lane before dropping? (Y/n)"));
  }
  else if (cmd == "turretload" || cmd == "tl") {
    if (anySequenceRunning()) {
      Serial.println(F(">> A sequence is already running. Stop it first (x)."));
      return;
    }
    if (turretPinsLoaded > 0) {
      Serial.print(F("System shows "));
      Serial.print(turretPinsLoaded);
      Serial.print(F(" pins in turret (at slot "));
      Serial.print(turretPinsLoaded);
      Serial.println(F(")."));
      Serial.println(F("Is this correct? Continue loading? (Y/n)"));
      seqPrompt = SEQPROMPT_TL_RESUME;
    } else {
      seqPrompt = SEQPROMPT_TL_CONFIRM;
      Serial.println(F("Verify turret is empty - no pins loaded."));
      Serial.println(F("Ready to proceed? (Y/n)"));
    }
  }
  else if (cmd == "pinpickup" || cmd == "pp") {
    if (anySequenceRunning()) {
      Serial.println(F(">> A sequence is already running. Stop it first (x)."));
      return;
    }
    seqPrompt = SEQPROMPT_PP_READY;
    Serial.println(F("Are there pins on the lane ready to pick up,"));
    Serial.println(F("and no pins already in the scissors? (Y/n)"));
  }
  else if (cmd == "pinset" || cmd == "ps") {
    if (anySequenceRunning()) {
      Serial.println(F(">> A sequence is already running. Stop it first (x)."));
      return;
    }
    seqPrompt = SEQPROMPT_PS_CONFIRM;
    Serial.println(F("Confirm no pins on the lane (scissors are holding pins)? (Y/n)"));
  }
  else if (cmd == "pincycle" || cmd == "pc") {
    if (anySequenceRunning()) {
      Serial.println(F(">> A sequence is already running. Stop it first (x)."));
      return;
    }
    seqPrompt = SEQPROMPT_PC_READY;
    Serial.println(F("Are there pins on the lane ready to pick up,"));
    Serial.println(F("and no pins already in the scissors? (Y/n)"));
  }
  else if (cmd == "full" || cmd == "fl"
           || cmd.startsWith("full ") || cmd.startsWith("fl ")) {
    if (anySequenceRunning()) {
      Serial.println(F(">> A sequence is already running. Stop it first (x)."));
      return;
    }
    // Parse optional cycle count: "full 30" or "fl 15"
    fullTestTargetCycles = 0;  // default unlimited
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx > 0) {
      int n = cmd.substring(spaceIdx + 1).toInt();
      if (n > 0) {
        fullTestTargetCycles = n;
        Serial.print(F(">> Will run "));
        Serial.print(fullTestTargetCycles);
        Serial.println(F(" cycles (x to cancel early)"));
      }
    }
    seqPrompt = SEQPROMPT_FULL_PRECLEAR;
    Serial.println(F("Run a full clear first? (Y/n)"));
  }
  else if (cmd == "clear" || cmd == "cl") {
    if (anySequenceRunning()) {
      Serial.println(F(">> A sequence is already running. Stop it first (x)."));
      return;
    }
    Serial.println(F(""));
    Serial.println(F(">> Starting CLEAR ALL sequence..."));
    startClearAll();
  }
  else if (cmd == "debug" || cmd == "de") {
    printTurretLoadTiming();
  }
  else if (cmd == "convtime" || cmd == "ct") {
    if (anySequenceRunning()) {
      Serial.println(F(">> A sequence is already running. Stop it first (x)."));
      return;
    }
    Serial.println(F(""));
    Serial.println(F("===== CONVEYOR TIMING TEST ====="));
    Serial.println(F("This test measures your conveyor belt timing and IR sensor"));
    Serial.println(F("sensitivity by loading the turret with 10 pins."));
    Serial.println(F(""));
    Serial.println(F("INSTRUCTIONS:"));
    Serial.println(F("  You will need to manually load pins at the base of the"));
    Serial.println(F("  conveyor onto consecutive conveyor doors. Have pins ready"));
    Serial.println(F("  so every door has a pin. If you miss a door, try to minimize"));
    Serial.println(F("  how many get missed - consecutive loading gives the best data."));
    Serial.println(F("================================"));
    Serial.println(F(""));
    seqPrompt = SEQPROMPT_CT_PRECLEAR;
    Serial.println(F("Run a full clear first to empty everything? (Y/n)"));
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
    Serial.println(F("Type 'help' or 'h' for menu"));
  }
}

// =====================================================
// SWEEP CLEAR SEQUENCE
// =====================================================

void startSweepClear() {
  if (sweepClearActive) {
    Serial.println(F(">> Sweep already running"));
    return;
  }
  sweepClearActive = true;

  // Check if deck is already up
  if (raiseLeftPos == RAISE_UP_ANGLE && raiseRightPos == (180 - RAISE_UP_ANGLE)) {
    // Deck already up, skip raise
    Serial.println(F("   Phase: Sweep back"));
    startSweepTo(SWEEP_BACK_ANGLE, 180 - SWEEP_BACK_ANGLE);
    sweepClearPhase = SCLEAR_SWEEP_BACK;
  } else {
    Serial.println(F("   Phase: Raise deck up"));
    ensureRaiseAttached();
    LeftRaiseServo.write(RAISE_UP_ANGLE);
    RightRaiseServo.write(180 - RAISE_UP_ANGLE);
    raiseLeftPos = RAISE_UP_ANGLE;
    raiseRightPos = 180 - RAISE_UP_ANGLE;
    sweepClearPhaseMs = millis();
    sweepClearPhase = SCLEAR_RAISE_UP;
  }
}

void runSweepClearFSM() {
  if (!sweepClearActive) return;

  unsigned long now = millis();

  switch (sweepClearPhase) {

    case SCLEAR_RAISE_UP:
      sweepClearPhase = SCLEAR_RAISE_WAIT;
      break;

    case SCLEAR_RAISE_WAIT:
      if (now - sweepClearPhaseMs >= PDROP_RAISE_SETTLE_MS) {
        Serial.println(F("   Phase: Sweep back"));
        startSweepTo(SWEEP_BACK_ANGLE, 180 - SWEEP_BACK_ANGLE);
        sweepClearPhase = SCLEAR_SWEEP_BACK;
      }
      break;

    case SCLEAR_SWEEP_BACK:
      sweepClearPhase = SCLEAR_SWEEP_WAIT;
      break;

    case SCLEAR_SWEEP_WAIT:
      if (!sweepAnimating) {
        sweepClearPhaseMs = millis();
        sweepClearPhase = SCLEAR_BACK_PAUSE;
      }
      break;

    case SCLEAR_BACK_PAUSE:
      if (millis() - sweepClearPhaseMs >= 250) {
        Serial.println(F("   Phase: Sweep to guard"));
        startSweepTo(SWEEP_GUARD_ANGLE, 180 - SWEEP_GUARD_ANGLE);
        sweepClearPhase = SCLEAR_SWEEP_GUARD;
      }
      break;

    case SCLEAR_SWEEP_GUARD:
      sweepClearPhase = SCLEAR_SWEEP_GUARD_WAIT;
      break;

    case SCLEAR_SWEEP_GUARD_WAIT:
      if (!sweepAnimating) {
        sweepClearPhase = SCLEAR_DONE;
      }
      break;

    case SCLEAR_DONE:
      sweepClearActive = false;
      sweepClearPhase = SCLEAR_IDLE;
      Serial.println(F(">> Sweep clear COMPLETE"));
      Serial.println(F(""));
      break;

    default:
      break;
  }
}

// =====================================================
// PIN DROP SEQUENCE
// =====================================================

void startPinDrop(bool preClear, bool postClear) {
  if (pinDropActive) {
    Serial.println(F(">> Pin drop already running"));
    return;
  }
  if (sweepClearActive || (turretLoadActive && !fullTestActive)) {
    Serial.println(F(">> ERROR: Another sequence is running"));
    return;
  }
  pinDropPostClear = postClear;
  pinDropActive = true;
  Serial.println(F(""));
  Serial.println(F(">> Starting pin drop sequence..."));
  if (preClear) {
    startSweepClear();
    pinDropPhase = PDROP_PRECLEAR_WAIT;
  } else {
    Serial.println(F("   Phase: Sweep to guard (no pre-clear)"));
    ensureSweepAttached();
    startSweepTo(SWEEP_GUARD_ANGLE, 180 - SWEEP_GUARD_ANGLE);
    pinDropPhase = PDROP_SWEEP_GUARD;
  }
}

void runPinDropFSM() {
  if (!pinDropActive) return;

  unsigned long now = millis();

  switch (pinDropPhase) {

    case PDROP_PRECLEAR_WAIT:
      if (!sweepClearActive) {
        ensureScissorAttached();
        ScissorsServo.write(SCISSOR_DROP_ANGLE);
        scissorAngle = SCISSOR_DROP_ANGLE;
        ensureRaiseAttached();
        ensureSliderAttached();
        pinDropPhaseStartMs = now;
        pinDropPhase = PDROP_SCISSOR_OPEN;
      }
      break;

    case PDROP_SWEEP_GUARD:
      pinDropPhase = PDROP_SWEEP_GUARD_WAIT;
      break;

    case PDROP_SWEEP_GUARD_WAIT:
      if (!sweepAnimating) {
        ensureScissorAttached();
        ScissorsServo.write(SCISSOR_DROP_ANGLE);
        scissorAngle = SCISSOR_DROP_ANGLE;
        ensureRaiseAttached();
        ensureSliderAttached();
        pinDropPhaseStartMs = now;
        pinDropPhase = PDROP_SCISSOR_OPEN;
      }
      break;

    case PDROP_SCISSOR_OPEN:
      if (now - pinDropPhaseStartMs >= PDROP_SETTLE_MS) {
        ensureRaiseAttached();
        LeftRaiseServo.write(RAISE_DOWN_ANGLE);
        RightRaiseServo.write(180 - RAISE_DOWN_ANGLE);
        raiseLeftPos = RAISE_DOWN_ANGLE;
        raiseRightPos = 180 - RAISE_DOWN_ANGLE;
        pinDropPhaseStartMs = now;
        pinDropPhase = PDROP_RAISE_DOWN;
      }
      break;

    case PDROP_RAISE_DOWN:
      if (now - pinDropPhaseStartMs >= PDROP_RAISE_SETTLE_MS) {
        SlideServo.write(SLIDER_RELEASE_ANGLE);
        sliderAngle = SLIDER_RELEASE_ANGLE;
        pinDropPhaseStartMs = now;
        pinDropPhase = PDROP_SLIDER_RELEASE;
      }
      break;

    case PDROP_SLIDER_RELEASE:
      if (now - pinDropPhaseStartMs >= PDROP_DROP_MS) {
        LeftRaiseServo.write(RAISE_UP_ANGLE);
        RightRaiseServo.write(180 - RAISE_UP_ANGLE);
        raiseLeftPos = RAISE_UP_ANGLE;
        raiseRightPos = 180 - RAISE_UP_ANGLE;
        pinDropPhaseStartMs = now;
        pinDropPhase = PDROP_RAISE_UP;
      }
      break;

    case PDROP_RAISE_UP:
      if (now - pinDropPhaseStartMs >= PDROP_RAISE_SETTLE_MS) {
        SlideServo.write(SLIDER_HOME_ANGLE);
        sliderAngle = SLIDER_HOME_ANGLE;
        pinDropPhaseStartMs = now;
        pinDropPhase = PDROP_SLIDER_HOME;
      }
      break;

    case PDROP_SLIDER_HOME:
      if (now - pinDropPhaseStartMs >= PDROP_SETTLE_MS) {
        if (pinDropPostClear) {
          Serial.println(F("   Starting post-clear..."));
          startSweepClear();
          pinDropPhase = PDROP_POSTCLEAR_WAIT;
        } else {
          pinDropPhase = PDROP_DONE;
        }
      }
      break;

    case PDROP_POSTCLEAR_WAIT:
      if (!sweepClearActive) {
        pinDropPhase = PDROP_DONE;
      }
      break;

    case PDROP_DONE:
      pinDropActive = false;
      pinDropPhase = PDROP_IDLE;
      Serial.println(F(">> Pin drop sequence COMPLETE"));
      Serial.println(F(""));
      break;

    default:
      break;
  }
}

// =====================================================
// TURRET LOAD SEQUENCE
// =====================================================

void startTurretLoad() {
  if (turretLoadActive) {
    Serial.println(F(">> Turret load already running"));
    return;
  }
  if (homingActive) {
    Serial.println(F(">> ERROR: Homing already in progress"));
    return;
  }
  if (pinDropActive) {
    Serial.println(F(">> ERROR: Pin drop sequence is running"));
    return;
  }
  if (!hallMonEnabled) {
    Serial.println(F(">> ERROR: Hall sensor monitoring is disabled"));
    Serial.println(F(">> Turret load requires the Hall sensor."));
    Serial.println(F(">> Re-enable it in the sensor menu first."));
    return;
  }

  turretLoadActive = true;
  tlLoadedCount = turretPinsLoaded;
  tlNowCatching = (turretPinsLoaded < 9) ? turretPinsLoaded + 1 : 9;
  tlQueuedPins = 0;

  // Clear timing log from previous sequence
  tlTimingLogCount = 0;
  tlTimingLogHead = 0;
  tlTimingPinCount = 0;
  tlTimingLoadNum = 0;

  Serial.println(F(""));
  // Turn on deck LEDs (white, normal brightness)
  deckL.setBrightness(DECK_LED_BRIGHTNESS);
  deckR.setBrightness(DECK_LED_BRIGHTNESS);
  deckAll(C_WHITE());
  deckShow();

  // Raise deck to UP position early so it's ready to catch pins at release.
  ensureRaiseAttached();
  raiseLeftPos = RAISE_UP_ANGLE;
  raiseRightPos = 180 - RAISE_UP_ANGLE;
  LeftRaiseServo.write(raiseLeftPos);
  RightRaiseServo.write(raiseRightPos);
  Serial.println(F("   Deck raised to UP position"));
  ConveyorOff();
  conveyorIsOn = false;

  // When pins are already in the turret (from a previous partial load), we must
  // NOT re-home. Homing rotates the turret nearly a full revolution, which would
  // jam or eject the pins already sitting in occupied slots. Instead, we skip
  // homing and move directly to the next empty slot — the turret position is
  // still valid from the homing that was done at the start of the previous load.
  if (turretPinsLoaded > 0) {
    Serial.print(F(">> Resuming turret load from "));
    Serial.print(turretPinsLoaded);
    Serial.println(F(" pins..."));
    Serial.print(F("   Skipping home (pins in turret). Moving to slot "));
    Serial.print(tlNowCatching);
    Serial.println(F("..."));
    stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
    stepper.setAcceleration(TURRET_NORMAL_ACCEL);
    turretGoTo(PinPositions[tlNowCatching]);
    turretLoadPhase = TLOAD_MOVE_TO_SLOT1;
  } else if (turretIsHomed) {
    // Already homed — skip homing and move directly to slot 1.
    Serial.println(F(">> Starting turret load (already homed)..."));
    Serial.print(F("   Moving to slot "));
    Serial.print(tlNowCatching);
    Serial.println(F("..."));
    stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
    stepper.setAcceleration(TURRET_NORMAL_ACCEL);
    turretGoTo(PinPositions[tlNowCatching]);
    turretLoadPhase = TLOAD_MOVE_TO_SLOT1;
  } else {
    Serial.println(F(">> Starting turret load sequence..."));
    Serial.println(F("   Phase: Homing turret"));
    startTurretHome();
    turretLoadPhase = TLOAD_HOMING;
  }
}

void printTurretLoadTiming() {
  if (tlTimingLogCount == 0) {
    Serial.println(F("   [TIMING] No IR events recorded."));
    return;
  }
  Serial.println(F(""));
  Serial.println(F("   ======== IR SENSOR TIMING ========"));
  int numEntries = tlTimingLogCount < TL_TIMING_MAX ? tlTimingLogCount : TL_TIMING_MAX;
  int startIdx = tlTimingLogCount <= TL_TIMING_MAX ? 0 : tlTimingLogHead;  // oldest entry
  if (tlTimingLogCount > TL_TIMING_MAX) {
    Serial.print(F("   (showing last "));
    Serial.print(TL_TIMING_MAX);
    Serial.print(F(" of "));
    Serial.print(tlTimingLogCount);
    Serial.println(F(" events)"));
  }
  for (int i = 0; i < numEntries; i++) {
    TlTimingEvent &e = tlTimingLog[(startIdx + i) % TL_TIMING_MAX];
    if (e.type == 'R') {
      Serial.print(F("   --- Turret load #"));
      Serial.print(e.pinNum);
      Serial.print(F(" released to deck (time: "));
      Serial.print(e.timeMs);
      Serial.println(F("ms) ---"));
      continue;
    }
    if (e.type == 'P') {
      Serial.print(F("   >> CAUGHT pin "));
      Serial.print(e.pinNum);
      Serial.print(F("/9 (time: "));
      Serial.print(e.timeMs);
      Serial.println(F("ms)"));
      continue;
    }
    if (e.type == 'A') {
      Serial.print(F("   >> ADVANCE to slot "));
      Serial.print(e.pinNum);
      Serial.print(F(" (time: "));
      Serial.print(e.timeMs);
      Serial.println(F("ms)"));
      continue;
    }
    Serial.print(F("   * Pin "));
    Serial.print(e.pinNum);
    if (e.type == 'D') {
      Serial.print(F(" Detected  (time: "));
      Serial.print(e.timeMs);
      Serial.print(F("ms"));
      if (e.durationMs > 0) {
        Serial.print(F(", gap: "));
        Serial.print(e.durationMs);
        Serial.print(F("ms"));
      }
      Serial.println(F(")"));
    } else {
      Serial.print(F(" Cleared   (time: "));
      Serial.print(e.timeMs);
      Serial.print(F("ms, blocked: "));
      Serial.print(e.durationMs);
      Serial.println(F("ms)"));
    }
  }
  Serial.println(F("   =================================="));
  Serial.println(F(""));
}

void runTurretLoadFSM() {
  if (!turretLoadActive) return;

  unsigned long now = millis();

  // IR debounce (runs during catching phases, including waiting for 10th pin)
  if (turretLoadPhase >= TLOAD_WAIT_CATCH && turretLoadPhase <= TLOAD_TENTH_SETTLE) {
    int raw = digitalRead(IR_SENSOR_PIN);

    // Raw IR timing (no debounce — stores every transition for later output)
    bool rawBlocked = (raw == LOW);
    if (rawBlocked != tlTimingBlocked) {
      unsigned long t = now - tlTimingStart;
      if (rawBlocked) {
        tlTimingBlocked = true;
        tlTimingBlockStart = now;
        tlTimingPinCount++;
        tlTimingLog[tlTimingLogHead].timeMs = t;
        tlTimingLog[tlTimingLogHead].durationMs = (tlTimingPinCount > 1 && tlTimingLastClear > 0) ? (now - tlTimingLastClear) : 0;
        tlTimingLog[tlTimingLogHead].pinNum = tlTimingPinCount;
        tlTimingLog[tlTimingLogHead].type = 'D';
        tlTimingLogHead = (tlTimingLogHead + 1) % TL_TIMING_MAX;
        tlTimingLogCount++;
      } else {
        tlTimingBlocked = false;
        unsigned long blocked = now - tlTimingBlockStart;
        tlTimingLastClear = now;
        tlTimingLog[tlTimingLogHead].timeMs = t;
        tlTimingLog[tlTimingLogHead].durationMs = blocked;
        tlTimingLog[tlTimingLogHead].pinNum = tlTimingPinCount;
        tlTimingLog[tlTimingLogHead].type = 'C';
        tlTimingLogHead = (tlTimingLogHead + 1) % TL_TIMING_MAX;
        tlTimingLogCount++;
      }
    }

    if (raw != tlIrLastRaw) { tlIrLastChange = now; tlIrLastRaw = raw; }
    if ((now - tlIrLastChange) > DEBOUNCE_MS) {
      if (tlIrStable != raw) tlIrStable = raw;
    }
    // ── IR re-arm logic ──────────────────────────────────────
    // Re-arm when beam has been clear long enough. Two layers
    // of protection prevent double-counting the caught pin:
    //
    //  Layer 1 – DEBOUNCE_MS (50ms): Raw noise filter (above).
    //    All observed jitter is well under 50ms:
    //      • Leading-edge bounce: 0–1ms, multiple in same ms
    //      • Trailing-edge bounce: 0–1ms, right after pin clears
    //      • Post-clear echo: 27–43ms, ~96–123ms after clear
    //    None survive the 50ms debounce window.
    //
    //  Layer 2 – TLOAD_ARM_DELAY_MS (200ms): Beam must be clear
    //    at the raw level for 200ms AND tlIrStable must be HIGH.
    //    Covers the post-clear echo window (~143ms from clear)
    //    with ~57ms margin. Echoes reset tlIrLastChange, so the
    //    200ms countdown restarts; re-arm at ~343ms after clear.
    //
    // Real-world pin timing (from conveyor test runs):
    //   • Pin blocks sensor for 203–331ms (typical ~275–310ms)
    //   • Gap between consecutive pins: 584–1005ms (typical ~730ms)
    //   • Time from catch to re-arm: ~525–560ms
    //     (clear ~300ms + debounce 50ms + arm delay 200ms)
    //   • Next pin arrives ~430ms after previous clears
    //     (730ms gap – 300ms blocked)
    //   • Re-arm happens ~170–205ms before next pin → safe margin
    //
    // Previously a third layer suppressed re-arm during the entire
    // CATCH_DELAY phase. This was removed because it caused missed
    // detections: pins arriving during the 800ms window would enter
    // and clear the sensor while re-arm was suppressed, resulting
    // in double-stacked pins in the turret.
    // ──────────────────────────────────────────────────────────
    if (tlIrStable == HIGH
        && (now - tlIrLastChange) >= TLOAD_ARM_DELAY_MS) {
      tlPinArmed = true;
    }

    // Detect pin arrival
    if (tlPinArmed && tlIrStable == LOW) {
      tlQueuedPins++;
      tlQueuedDetectMs = now;  // record when this pin was detected
      tlPinArmed = false;
      Serial.print(F("   [IR] Pin queued (phase="));
      Serial.print(turretLoadPhase);
      Serial.print(F(", q="));
      Serial.print(tlQueuedPins);
      Serial.println(F(")"));
    }
  }

  switch (turretLoadPhase) {

    case TLOAD_HOMING:
      if (!homingActive) {
        Serial.print(F("   Homing complete. Moving to slot "));
        Serial.print(tlNowCatching);
        Serial.println(F("..."));
        tlQueuedPins = 0;
        stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
        stepper.setAcceleration(TURRET_NORMAL_ACCEL);
        turretGoTo(PinPositions[tlNowCatching]);
        turretLoadPhase = TLOAD_MOVE_TO_SLOT1;
      }
      break;

    case TLOAD_MOVE_TO_SLOT1:
      if (!turretMoving) {
        Serial.print(F("   At slot "));
        Serial.print(tlNowCatching);
        Serial.println(F(". Starting pin loading..."));
        // Initialize IR state
        tlIrLastRaw = digitalRead(IR_SENSOR_PIN);
        tlIrStable = tlIrLastRaw;
        tlIrLastChange = now;
        tlQueuedPins = 0;
        // Start conveyor
        ConveyorOn();
        conveyorIsOn = true;
        Serial.println(F("   Conveyor ON - Feed pins into turret"));
        // Reset per-load timing state (log persists across loads)
        tlTimingLoadNum++;
        tlTimingStart = now;
        tlTimingBlocked = (tlIrLastRaw == LOW);
        tlTimingBlockStart = 0;
        tlTimingLastClear = 0;
        if (tlTimingBlocked) {
          tlTimingBlockStart = now;
          tlTimingPinCount = 1;
        }
        if (tlLoadedCount >= 9) {
          // Already have 9 pins, just need the 10th
          Serial.println(F("   9 pins already loaded. Waiting for 10th pin..."));
          tlPinArmed = true;
          turretLoadPhase = TLOAD_WAIT_TENTH;
        } else if (tlIrLastRaw == LOW) {
          // A pin is already at the IR sensor before loading starts. Count it
          // for the current slot and go straight to CATCH_DELAY so it has time
          // to settle into the turret. The sensor starts disarmed (tlPinArmed
          // = false) and re-arms naturally via the arm-delay mechanism once
          // the pin clears and the beam stays clear for TLOAD_ARM_DELAY_MS.
          tlPinArmed = false;
          tlLoadedCount++;
          turretPinsLoaded = tlLoadedCount;
          Serial.print(F("   Pin already at sensor - caught for slot "));
          Serial.print(tlNowCatching);
          Serial.print(F("! ("));
          Serial.print(tlLoadedCount);
          Serial.println(F("/9)"));
          tlPhaseStartMs = now;
          turretLoadPhase = TLOAD_CATCH_DELAY;
        } else {
          // Sensor clear — arm and wait for first pin
          tlPinArmed = true;
          Serial.print(F("   Waiting for pin at slot "));
          Serial.println(tlNowCatching);
          turretLoadPhase = TLOAD_WAIT_CATCH;
        }
      }
      break;

    case TLOAD_WAIT_CATCH:
      if (tlQueuedPins > 0) {
        tlQueuedPins--;
        tlLoadedCount++;
        turretPinsLoaded = tlLoadedCount;
        Serial.print(F("   Pin caught! ("));
        Serial.print(tlLoadedCount);
        Serial.println(F("/9)"));

        // Log pin-caught event
        tlTimingLog[tlTimingLogHead].timeMs = now - tlTimingStart;
        tlTimingLog[tlTimingLogHead].durationMs = 0;
        tlTimingLog[tlTimingLogHead].pinNum = tlLoadedCount;
        tlTimingLog[tlTimingLogHead].type = 'P';
        tlTimingLogHead = (tlTimingLogHead + 1) % TL_TIMING_MAX;
        tlTimingLogCount++;

        if (tlLoadedCount < 9) {
          // Catch delay timer starts from when the pin was DETECTED at the
          // sensor, not when the queue is consumed. This prevents cumulative
          // drift when pins arrive during a previous CATCH_DELAY or ADVANCING.
          // CATCH_DELAY also requires tlIrStable == HIGH (pin has cleared and
          // jitter has settled) before advancing — so even if the delay has
          // already elapsed, the turret won't advance until the pin is gone.
          tlPhaseStartMs = tlQueuedDetectMs;
          turretLoadPhase = TLOAD_CATCH_DELAY;
        } else {
          tlPhaseStartMs = tlQueuedDetectMs;
          turretLoadPhase = TLOAD_NINTH_SETTLE;
        }
      }
      break;

    case TLOAD_CATCH_DELAY:
      // Advance when BOTH conditions are met:
      //  1. Enough time has elapsed since the pin was detected (CATCH_DELAY_MS)
      //  2. The pin has cleared the sensor and jitter has settled (tlIrStable == HIGH)
      // Normally the pin clears ~300ms after detection and debounce confirms ~350ms,
      // so condition 2 is satisfied well before condition 1 (800ms). But if a pin
      // takes unusually long to clear, we wait for it rather than advancing early.
      if ((now - tlPhaseStartMs >= CATCH_DELAY_MS) && tlIrStable == HIGH) {
        tlNowCatching++;
        Serial.print(F("   Advancing to slot "));
        Serial.println(tlNowCatching);

        // Log turret-advance event
        tlTimingLog[tlTimingLogHead].timeMs = now - tlTimingStart;
        tlTimingLog[tlTimingLogHead].durationMs = 0;
        tlTimingLog[tlTimingLogHead].pinNum = tlNowCatching;
        tlTimingLog[tlTimingLogHead].type = 'A';
        tlTimingLogHead = (tlTimingLogHead + 1) % TL_TIMING_MAX;
        tlTimingLogCount++;

        stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
        stepper.setAcceleration(TURRET_NORMAL_ACCEL);
        turretGoTo(PinPositions[tlNowCatching]);
        turretLoadPhase = TLOAD_ADVANCING;
      }
      break;

    case TLOAD_ADVANCING:
      if (!turretMoving) {
        Serial.print(F("   Waiting for pin at slot "));
        Serial.println(tlNowCatching);
        turretLoadPhase = TLOAD_WAIT_CATCH;
      }
      break;

    case TLOAD_NINTH_SETTLE:
      // Wait for BOTH: settle time elapsed AND 9th pin has cleared the sensor.
      // Two failure modes without these guards:
      //  1. Slow-clearing pins (block 339-408ms > NINTH_SETTLE_MS 300ms):
      //     The 9th pin is still blocking when settle expires. A raw sensor
      //     check would misidentify it as a pre-existing 10th pin.
      //  2. Post-clear echoes (echo 55ms after pin clears): The 9th pin
      //     clears within 300ms, but a brief echo re-blocks the raw sensor.
      //     tlIrStable stays HIGH (echo too short for debounce), so a raw
      //     check would read LOW during the echo — another phantom 10th pin.
      // Fix: require tlIrStable == HIGH (handles case 1) and remove the
      // proactive raw check entirely (handles case 2). A real 10th pin
      // cannot be at the sensor this soon — minimum inter-pin gap (~584ms+)
      // far exceeds the settle window. WAIT_TENTH detects it normally via
      // the arm-delay mechanism.
      if ((now - tlPhaseStartMs >= NINTH_SETTLE_MS) && tlIrStable == HIGH) {
        Serial.println(F("   9 pins loaded. Waiting for 10th pin..."));
        tlQueuedPins = 0;
        tlPinArmed = false;
        // Re-sync debounce state so arm delay starts from clean baseline
        tlIrLastRaw = digitalRead(IR_SENSOR_PIN);
        tlIrStable = tlIrLastRaw;
        tlIrLastChange = now;
        turretLoadPhase = TLOAD_WAIT_TENTH;
      }
      break;

    case TLOAD_WAIT_TENTH:
      if (tlQueuedPins > 0) {
        tlQueuedPins--;
        Serial.println(F("   10th pin detected!"));
        if (turretLoadHoldRelease) {
          // Hold mode: stop conveyor, wait for release signal
          Serial.println(F("   Hold-release active - stopping conveyor, waiting for release..."));
          ConveyorOff();
          conveyorIsOn = false;
          turretLoadPhase = TLOAD_HOLD_RELEASE;
        } else {
          // Normal mode: prepare deck and release immediately
          ensureSliderAttached();
          SlideServo.write(SLIDER_HOME_ANGLE);
          sliderAngle = SLIDER_HOME_ANGLE;
          Serial.println(F("   Sliding deck home."));
          Serial.println(F("   Moving turret to release position..."));
          long releasePos = PinPositions[10] + TURRET_PIN10_RELEASE_OFFSET;
          stepper.setMaxSpeed(TURRET_SPRING_MAXSPEED);
          stepper.setAcceleration(TURRET_SPRING_ACCEL);
          turretGoTo(releasePos);
          tlPhaseStartMs = now;
          tlConveyorStopped = false;
          turretLoadPhase = TLOAD_TENTH_SETTLE;
        }
      }
      break;

    case TLOAD_HOLD_RELEASE: {
      if (!turretLoadHoldRelease) {
        Serial.println(F("   Hold released! Preparing deck for turret release..."));
        // Prepare deck
        ensureSliderAttached();
        SlideServo.write(SLIDER_HOME_ANGLE);
        sliderAngle = SLIDER_HOME_ANGLE;
        Serial.println(F("   Sliding deck home."));
        // Move turret to release
        Serial.println(F("   Moving turret to release position..."));
        long releasePos = PinPositions[10] + TURRET_PIN10_RELEASE_OFFSET;
        stepper.setMaxSpeed(TURRET_SPRING_MAXSPEED);
        stepper.setAcceleration(TURRET_SPRING_ACCEL);
        turretGoTo(releasePos);
        tlPhaseStartMs = now;
        tlConveyorStopped = true;  // Conveyor already stopped in WAIT_TENTH
        turretLoadPhase = TLOAD_TENTH_SETTLE;
      }
      break;
    }

    case TLOAD_TENTH_SETTLE:
      // Stop conveyor after settle delay (pin has dropped)
      if (!tlConveyorStopped && (now - tlPhaseStartMs >= CATCH_DELAY_MS)) {
        Serial.println(F("   Stopping conveyor..."));
        ConveyorOff();
        conveyorIsOn = false;
        tlConveyorStopped = true;
      }
      // Wait for turret to finish moving to release
      if (tlConveyorStopped && !turretMoving) {
        Serial.println(F("   At release position - dwelling..."));
        tlPhaseStartMs = now;
        turretLoadPhase = TLOAD_RELEASE_DWELL;
      }
      break;

    case TLOAD_RELEASE_DWELL:
      if (now - tlPhaseStartMs >= RELEASE_DWELL_MS) {
        turretPinsLoaded = 0;  // Pins released to deck
        // Log a turret-released marker in the timing buffer
        tlTimingLog[tlTimingLogHead].timeMs = now - tlTimingStart;
        tlTimingLog[tlTimingLogHead].durationMs = 0;
        tlTimingLog[tlTimingLogHead].pinNum = tlTimingLoadNum;
        tlTimingLog[tlTimingLogHead].type = 'R';
        tlTimingLogHead = (tlTimingLogHead + 1) % TL_TIMING_MAX;
        tlTimingLogCount++;
        ConveyorOff();
        conveyorIsOn = false;
        // Hall sensor is triggered at release position (near slot 10) —
        // ignore it here. Verification happens after returning to slot 1
        // and approaching the hall sensor from the slot 1 side.
        Serial.println(F("   Release complete. Returning to slot 1..."));
        if (fullTestActive) {
          turretPinsReleasedToDeck = true;
          turretLoadHoldRelease = true;  // Hold 10th pin of next batch
          tlLoadedCount = 0;
          tlNowCatching = 1;
          tlQueuedPins = 0;
          tlVerifyForFullTest = true;
        } else {
          tlVerifyForFullTest = false;
        }
        stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
        stepper.setAcceleration(TURRET_NORMAL_ACCEL);
        turretGoTo(PinPositions[1]);
        turretLoadPhase = TLOAD_VERIFY_MOVE_SLOT1;
      }
      break;

    case TLOAD_VERIFY_MOVE_SLOT1:
      // Wait for turret to reach slot 1 (long way: 10->9->...->1).
      // Hall sensor is in the gap past slot 1 — not traversed during this move.
      if (!turretMoving) {
        Serial.println(F("   At slot 1. Verifying hall sensor position..."));
        // Move quickly to 50 steps before expected hall position.
        // Approach from the slot 1 side (negative of hall) so we can
        // creep in the positive direction — same as homing — to hit
        // the same leading edge of the magnet.
        stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
        stepper.setAcceleration(TURRET_NORMAL_ACCEL);
        turretGoTo(homeAdjustValue - 50);
        turretLoadPhase = TLOAD_VERIFY_APPROACH;
      }
      break;

    case TLOAD_VERIFY_APPROACH:
      // Wait for fast approach to complete, then creep through hall zone
      if (!turretMoving) {
        stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED / 2);
        stepper.setAcceleration(TURRET_NORMAL_ACCEL);
        tlVerifyHallSeen = false;
        tlVerifyHallPos = 0;
        // Creep 100 steps in the POSITIVE direction through the hall zone.
        // Same direction as homing (which creeps +2 at a time) so we hit
        // the same leading edge of the magnet detection zone.
        // Expected trigger at homeAdjustValue (-63), centered in this range.
        turretGoTo(homeAdjustValue + 50);
        turretLoadPhase = TLOAD_VERIFY_CREEP;
      }
      break;

    case TLOAD_VERIFY_CREEP:
      // Monitor hall sensor during slow creep
      if (!tlVerifyHallSeen && digitalRead(HALL_EFFECT_PIN) == LOW) {
        tlVerifyHallSeen = true;
        tlVerifyHallPos = stepper.currentPosition();
        // Stop immediately — we have our measurement
        stepper.stop();
        stepper.setCurrentPosition(stepper.currentPosition());
        turretTargetPos = stepper.currentPosition();  // Sync so next turretGoTo() isn't suppressed
        turretMoving = false;
      }
      if (!turretMoving) {
        turretLoadPhase = TLOAD_VERIFY_EVAL;
      }
      break;

    case TLOAD_VERIFY_EVAL: {
      if (!tlVerifyHallSeen) {
        Serial.println(F("   Hall sensor NOT detected during verification. Re-homing..."));
        startTurretHome();
        turretLoadPhase = TLOAD_REHOMING;
      } else {
        long drift = abs(tlVerifyHallPos - homeAdjustValue);
        if (drift <= TURRET_VERIFY_TOLERANCE) {
          Serial.print(F("   Hall verified (drift: "));
          Serial.print(drift);
          Serial.print(F(" steps, expected "));
          Serial.print(homeAdjustValue);
          Serial.print(F(", actual "));
          Serial.print(tlVerifyHallPos);
          Serial.println(F(")"));
          // Return to slot 1 to continue
          stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
          stepper.setAcceleration(TURRET_NORMAL_ACCEL);
          turretGoTo(PinPositions[1]);
          turretLoadPhase = tlVerifyForFullTest ? TLOAD_MOVE_TO_SLOT1 : TLOAD_MOVE_SLOT1;
        } else {
          Serial.print(F("   Hall drift too large ("));
          Serial.print(drift);
          Serial.print(F(" steps, expected "));
          Serial.print(homeAdjustValue);
          Serial.print(F(", actual "));
          Serial.print(tlVerifyHallPos);
          Serial.println(F("). Re-homing..."));
          startTurretHome();
          turretLoadPhase = TLOAD_REHOMING;
        }
      }
      break;
    }

    case TLOAD_REHOMING:
      // Wait for homing FSM to complete (turret will be at slot 1 after)
      if (!homingActive) {
        Serial.println(F("   Re-homing complete."));
        turretLoadPhase = tlVerifyForFullTest ? TLOAD_MOVE_TO_SLOT1 : TLOAD_MOVE_SLOT1;
      }
      break;

    case TLOAD_MOVE_SLOT1:
      if (!turretMoving) {
        Serial.println(F("   Turret at slot 1."));
        turretLoadPhase = TLOAD_DONE;
      }
      break;

    case TLOAD_DONE:
      turretLoadActive = false;
      turretLoadPhase = TLOAD_IDLE;
      Serial.println(F(">> Turret load sequence COMPLETE"));
      Serial.println(F("   10 pins loaded and released to deck"));
      if (tlTimingLogCount > 0 && !convTimingActive) {
        seqPrompt = SEQPROMPT_TL_TIMING;
        Serial.println(F("Display IR sensor and turret timing diagnostic data? (y/N)"));
      }
      break;

    default:
      break;
  }
}

// =====================================================
// PIN PICKUP SEQUENCE
// =====================================================

void startPinPickup() {
  if (pinPickupActive || pinSetActive) {
    Serial.println(F(">> Pickup/set already running"));
    return;
  }
  if (pinDropActive || (!fullTestActive && turretLoadActive) || sweepClearActive) {
    Serial.println(F(">> ERROR: Another sequence is running"));
    return;
  }
  pinPickupActive = true;
  Serial.println(F(""));
  Serial.println(F(">> Starting pin pickup sequence..."));
  Serial.println(F("   Phase: Scissor open"));
  ensureScissorAttached();
  ensureRaiseAttached();
  ScissorsServo.write(SCISSOR_DROP_ANGLE);
  scissorAngle = SCISSOR_DROP_ANGLE;
  pinPickupPhaseStartMs = millis();
  pinPickupPhase = PPU_SCISSOR_OPEN;
}

void runPinPickupFSM() {
  if (!pinPickupActive) return;

  unsigned long now = millis();

  switch (pinPickupPhase) {

    case PPU_SCISSOR_OPEN:
      if (now - pinPickupPhaseStartMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Phase: Raise to grab position"));
        LeftRaiseServo.write(RAISE_GRAB_ANGLE);
        RightRaiseServo.write(180 - RAISE_GRAB_ANGLE);
        raiseLeftPos = RAISE_GRAB_ANGLE;
        raiseRightPos = 180 - RAISE_GRAB_ANGLE;
        pinPickupPhaseStartMs = now;
        pinPickupPhase = PPU_RAISE_GRAB;
      }
      break;

    case PPU_RAISE_GRAB:
      if (now - pinPickupPhaseStartMs >= PDROP_RAISE_SETTLE_MS) {
        Serial.println(F("   Phase: Scissor close (grab)"));
        ScissorsServo.write(SCISSOR_GRAB_ANGLE);
        scissorAngle = SCISSOR_GRAB_ANGLE;
        pinPickupPhaseStartMs = now;
        pinPickupPhase = PPU_SCISSOR_CLOSE;
      }
      break;

    case PPU_SCISSOR_CLOSE:
      if (now - pinPickupPhaseStartMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Phase: Raise up"));
        LeftRaiseServo.write(RAISE_UP_ANGLE);
        RightRaiseServo.write(180 - RAISE_UP_ANGLE);
        raiseLeftPos = RAISE_UP_ANGLE;
        raiseRightPos = 180 - RAISE_UP_ANGLE;
        pinPickupPhaseStartMs = now;
        pinPickupPhase = PPU_RAISE_UP;
      }
      break;

    case PPU_RAISE_UP:
      if (now - pinPickupPhaseStartMs >= PDROP_RAISE_SETTLE_MS) {
        pinPickupPhase = PPU_DONE;
      }
      break;

    case PPU_DONE:
      pinPickupActive = false;
      pinPickupPhase = PPU_IDLE;
      Serial.println(F(">> Pin pickup sequence COMPLETE"));
      Serial.println(F(""));
      if (pinCycleActive) {
        Serial.println(F(">> Cycling: starting pin set... (Enter or x to stop)"));
        startPinSet();
      }
      break;

    default:
      break;
  }
}

// =====================================================
// PIN SET SEQUENCE
// =====================================================

void startPinSet() {
  if (pinPickupActive || pinSetActive) {
    Serial.println(F(">> Pickup/set already running"));
    return;
  }
  if (pinDropActive || (!fullTestActive && turretLoadActive) || sweepClearActive) {
    Serial.println(F(">> ERROR: Another sequence is running"));
    return;
  }
  pinSetActive = true;
  Serial.println(F(""));
  Serial.println(F(">> Starting pin set sequence..."));
  Serial.println(F("   Phase: Raise to drop position"));
  ensureScissorAttached();
  ensureRaiseAttached();
  LeftRaiseServo.write(RAISE_DROP_ANGLE);
  RightRaiseServo.write(180 - RAISE_DROP_ANGLE);
  raiseLeftPos = RAISE_DROP_ANGLE;
  raiseRightPos = 180 - RAISE_DROP_ANGLE;
  pinSetPhaseStartMs = millis();
  pinSetPhase = PSET_RAISE_DROP;
}

void runPinSetFSM() {
  if (!pinSetActive) return;

  unsigned long now = millis();

  switch (pinSetPhase) {

    case PSET_RAISE_DROP:
      if (now - pinSetPhaseStartMs >= PDROP_RAISE_SETTLE_MS) {
        Serial.println(F("   Phase: Scissor open (release pins)"));
        ScissorsServo.write(SCISSOR_DROP_ANGLE);
        scissorAngle = SCISSOR_DROP_ANGLE;
        pinSetPhaseStartMs = now;
        pinSetPhase = PSET_SCISSOR_OPEN;
      }
      break;

    case PSET_SCISSOR_OPEN:
      if (now - pinSetPhaseStartMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Phase: Raise up"));
        LeftRaiseServo.write(RAISE_UP_ANGLE);
        RightRaiseServo.write(180 - RAISE_UP_ANGLE);
        raiseLeftPos = RAISE_UP_ANGLE;
        raiseRightPos = 180 - RAISE_UP_ANGLE;
        pinSetPhaseStartMs = now;
        pinSetPhase = PSET_RAISE_UP;
      }
      break;

    case PSET_RAISE_UP:
      if (now - pinSetPhaseStartMs >= PDROP_RAISE_SETTLE_MS) {
        pinSetPhase = PSET_DONE;
      }
      break;

    case PSET_DONE:
      pinSetActive = false;
      pinSetPhase = PSET_IDLE;
      Serial.println(F(">> Pin set sequence COMPLETE"));
      Serial.println(F(""));
      if (pinCycleActive) {
        Serial.println(F(">> Cycling: starting pin pickup... (Enter or x to stop)"));
        startPinPickup();
      }
      break;

    default:
      break;
  }
}

// =====================================================
// CLEAR ALL SEQUENCE
// =====================================================

void startClearAll() {
  clearAllActive = true;
  clearAllPhase = CLALL_CLOSE_BALL_DOOR;
  clearAllPhaseMs = millis();
  Serial.println(F("   Closing ball door..."));
  ensureBallDoorAttached();
  ballDoorAngle = BALL_DOOR_CLOSED_ANGLE;
  BallReturnServo.write(ballDoorAngle);
}

void runClearAllFSM() {
  if (!clearAllActive) return;

  unsigned long now = millis();

  switch (clearAllPhase) {

    // --- Ball door close ---
    case CLALL_CLOSE_BALL_DOOR:
      if (now - clearAllPhaseMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Ball door closed. Clearing lane..."));
        startSweepClear();
        clearAllPhase = CLALL_SWEEP1;
      }
      break;

    // --- Phase 1: Initial lane clear ---
    case CLALL_SWEEP1:
      clearAllPhase = CLALL_WAIT_SWEEP1;
      break;

    case CLALL_WAIT_SWEEP1:
      if (!sweepClearActive) {
        Serial.println(F("   Lane cleared. Lowering deck to drop position..."));
        ensureRaiseAttached();
        raiseLeftPos = RAISE_DROP_ANGLE;
        raiseRightPos = 180 - RAISE_DROP_ANGLE;
        LeftRaiseServo.write(raiseLeftPos);
        RightRaiseServo.write(raiseRightPos);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_RAISE_DROP1;
      }
      break;

    // --- Phase 2: Clear sliding deck ---
    case CLALL_RAISE_DROP1:
      if (now - clearAllPhaseMs >= PDROP_RAISE_SETTLE_MS) {
        Serial.println(F("   Deck at drop. Opening scissor..."));
        ensureScissorAttached();
        scissorAngle = SCISSOR_DROP_ANGLE;
        ScissorsServo.write(scissorAngle);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_SCISSOR_OPEN1;
      }
      break;

    case CLALL_SCISSOR_OPEN1:
      if (now - clearAllPhaseMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Scissor open. Lowering deck to set position..."));
        raiseLeftPos = RAISE_DOWN_ANGLE;
        raiseRightPos = 180 - RAISE_DOWN_ANGLE;
        LeftRaiseServo.write(raiseLeftPos);
        RightRaiseServo.write(raiseRightPos);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_RAISE_DOWN1;
      }
      break;

    case CLALL_RAISE_DOWN1:
      if (now - clearAllPhaseMs >= PDROP_RAISE_SETTLE_MS) {
        Serial.println(F("   Deck down. Releasing slider..."));
        ensureSliderAttached();
        sliderAngle = SLIDER_RELEASE_ANGLE;
        SlideServo.write(sliderAngle);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_SLIDER_RELEASE1;
      }
      break;

    case CLALL_SLIDER_RELEASE1:
      if (now - clearAllPhaseMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Slider released. Raising deck..."));
        raiseLeftPos = RAISE_UP_ANGLE;
        raiseRightPos = 180 - RAISE_UP_ANGLE;
        LeftRaiseServo.write(raiseLeftPos);
        RightRaiseServo.write(raiseRightPos);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_RAISE_UP1;
      }
      break;

    case CLALL_RAISE_UP1:
      if (now - clearAllPhaseMs >= PDROP_RAISE_SETTLE_MS) {
        Serial.println(F("   Deck up. Retracting slider..."));
        sliderAngle = SLIDER_HOME_ANGLE;
        SlideServo.write(sliderAngle);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_SLIDER_HOME1;
      }
      break;

    case CLALL_SLIDER_HOME1:
      if (now - clearAllPhaseMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Slider home. Sweeping lane..."));
        startSweepClear();
        clearAllPhase = CLALL_SWEEP2;
      }
      break;

    // --- Sweep after sliding deck dump ---
    case CLALL_SWEEP2:
      clearAllPhase = CLALL_WAIT_SWEEP2;
      break;

    case CLALL_WAIT_SWEEP2:
      if (!sweepClearActive) {
        // --- Phase 3: Clear turret ---
        if (turretIsHomed) {
          Serial.println(F("   Lane cleared. Turret already homed. Moving to release..."));
          clearAllPhase = CLALL_WAIT_HOMING;  // Skip to move-to-release
        } else {
          Serial.println(F("   Lane cleared. Homing turret..."));
          startTurretHome();
          clearAllPhase = CLALL_HOMING;
        }
      }
      break;

    case CLALL_HOMING:
      clearAllPhase = CLALL_WAIT_HOMING;
      break;

    case CLALL_WAIT_HOMING:
      if (!homingActive) {
        Serial.println(F("   Moving to release position..."));
        stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
        stepper.setAcceleration(TURRET_NORMAL_ACCEL);
        turretGoTo(PinPositions[9]);
        clearAllPhase = CLALL_FAST_TO_SLOT9;
      }
      break;

    case CLALL_FAST_TO_SLOT9:
      clearAllPhase = CLALL_WAIT_FAST_TO_SLOT9;
      break;

    case CLALL_WAIT_FAST_TO_SLOT9:
      if (!turretMoving) {
        long releasePos = PinPositions[10] + TURRET_PIN10_RELEASE_OFFSET;
        stepper.setMaxSpeed(TURRET_SPRING_MAXSPEED);
        stepper.setAcceleration(TURRET_SPRING_ACCEL);
        turretGoTo(releasePos);
        clearAllPhase = CLALL_MOVE_RELEASE;
      }
      break;

    case CLALL_MOVE_RELEASE:
      clearAllPhase = CLALL_WAIT_MOVE_RELEASE;
      break;

    case CLALL_WAIT_MOVE_RELEASE:
      if (stepper.distanceToGo() == 0) {
        Serial.println(F("   At release position. Dwelling..."));
        clearAllPhaseMs = now;
        turretMoving = false;
        turretPinsLoaded = 0;
        clearAllPhase = CLALL_RELEASE_DWELL;
      }
      break;

    case CLALL_RELEASE_DWELL:
      if (now - clearAllPhaseMs >= RELEASE_DWELL_MS) {
        Serial.println(F("   Turret released. Moving to slot 1..."));
        stepper.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
        stepper.setAcceleration(TURRET_NORMAL_ACCEL);
        turretGoTo(PinPositions[1]);
        clearAllPhase = CLALL_MOVE_SLOT1;
      }
      break;

    case CLALL_MOVE_SLOT1:
      clearAllPhase = CLALL_WAIT_MOVE_SLOT1;
      break;

    case CLALL_WAIT_MOVE_SLOT1:
      if (stepper.distanceToGo() == 0) {
        turretMoving = false;
        // Scissors already open from first pass — go straight to down/set
        Serial.println(F("   At slot 1. Lowering deck to set position..."));
        raiseLeftPos = RAISE_DOWN_ANGLE;
        raiseRightPos = 180 - RAISE_DOWN_ANGLE;
        LeftRaiseServo.write(raiseLeftPos);
        RightRaiseServo.write(raiseRightPos);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_RAISE_DOWN2;
      }
      break;

    case CLALL_RAISE_DOWN2:
      if (now - clearAllPhaseMs >= PDROP_RAISE_SETTLE_MS) {
        Serial.println(F("   Deck down. Releasing slider..."));
        sliderAngle = SLIDER_RELEASE_ANGLE;
        SlideServo.write(sliderAngle);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_SLIDER_RELEASE2;
      }
      break;

    case CLALL_SLIDER_RELEASE2:
      if (now - clearAllPhaseMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Slider released. Raising deck..."));
        raiseLeftPos = RAISE_UP_ANGLE;
        raiseRightPos = 180 - RAISE_UP_ANGLE;
        LeftRaiseServo.write(raiseLeftPos);
        RightRaiseServo.write(raiseRightPos);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_RAISE_UP2;
      }
      break;

    case CLALL_RAISE_UP2:
      if (now - clearAllPhaseMs >= PDROP_RAISE_SETTLE_MS) {
        Serial.println(F("   Deck up. Retracting slider..."));
        sliderAngle = SLIDER_HOME_ANGLE;
        SlideServo.write(sliderAngle);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_SLIDER_HOME2;
      }
      break;

    case CLALL_SLIDER_HOME2:
      if (now - clearAllPhaseMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Slider home. Final lane sweep..."));
        startSweepClear();
        clearAllPhase = CLALL_SWEEP3;
      }
      break;

    // --- Final sweep ---
    case CLALL_SWEEP3:
      clearAllPhase = CLALL_WAIT_SWEEP3;
      break;

    case CLALL_WAIT_SWEEP3:
      if (!sweepClearActive) {
        Serial.println(F("   Lane cleared. Setting scissor to open..."));
        scissorAngle = SCISSOR_DROP_ANGLE;
        ScissorsServo.write(scissorAngle);
        clearAllPhaseMs = now;
        clearAllPhase = CLALL_SCISSOR_FINAL;
      }
      break;

    case CLALL_SCISSOR_FINAL:
      if (now - clearAllPhaseMs >= PDROP_SETTLE_MS) {
        clearAllPhase = CLALL_DONE;
      }
      break;

    case CLALL_DONE:
      clearAllActive = false;
      clearAllPhase = CLALL_IDLE;
      Serial.println(F(""));
      Serial.println(F(">> CLEAR ALL complete."));
      Serial.println(F("   State: deck up, sweep guard, slider home,"));
      Serial.println(F("   scissor open, ball door closed."));
      Serial.println(F(""));
      break;

    default:
      break;
  }
}

// =====================================================
// CONVEYOR TIMING TEST
// =====================================================

void startConveyorTiming() {
  convTimingActive = true;

  // Clear timing log for fresh data
  tlTimingLogCount = 0;
  tlTimingLogHead = 0;
  tlTimingPinCount = 0;
  tlTimingLoadNum = 0;

  // Force fresh turret load
  turretPinsLoaded = 0;

  Serial.println(F(""));
  Serial.println(F(">> Starting conveyor timing test..."));
  Serial.println(F("   Starting turret load. Load pins onto consecutive conveyor doors!"));
  startTurretLoad();
  convTimingPhase = CTIME_WAIT_LOAD;
}

void runConvTimingFSM() {
  if (!convTimingActive) return;

  switch (convTimingPhase) {
    case CTIME_PRECLEAR:
      convTimingPhase = CTIME_WAIT_PRECLEAR;
      break;

    case CTIME_WAIT_PRECLEAR:
      if (!clearAllActive) {
        Serial.println(F("   Clear complete."));
        Serial.println(F(""));
        seqPrompt = SEQPROMPT_CT_READY;
        Serial.println(F("Ready to begin the conveyor timing test? (Y/n)"));
        convTimingPhase = CTIME_IDLE;
      }
      break;

    case CTIME_WAIT_LOAD:
      if (!turretLoadActive) {
        convTimingPhase = CTIME_DONE;
      }
      break;

    case CTIME_DONE:
      printConveyorTimingAnalysis();
      convTimingActive = false;
      convTimingPhase = CTIME_IDLE;
      if (tlTimingLogCount > 0) {
        seqPrompt = SEQPROMPT_CT_RAWTIMING;
        Serial.println(F("Show raw IR timing log? (y/N)"));
      }
      break;

    default:
      break;
  }
}

void printConveyorTimingAnalysis() {
  if (tlTimingLogCount == 0) {
    Serial.println(F("   No IR events recorded - cannot analyze."));
    return;
  }

  int numEntries = tlTimingLogCount < TL_TIMING_MAX ? tlTimingLogCount : TL_TIMING_MAX;
  int startIdx = tlTimingLogCount <= TL_TIMING_MAX ? 0 : tlTimingLogHead;

  // --- Identify real pins from raw IR sensor transitions ---
  // Real pins block the IR sensor for >= 100ms; bounces are < 50ms.
  // This separates real pins from jitter without relying on CAUGHT
  // markers, which fire between a pin's detect and clear events and
  // would split each physical pin's data across two groups.
  const unsigned long MIN_REAL_BLOCKED_MS = 100;

  unsigned long pinBlockedMs[10];
  unsigned long pinGapMs[10];
  unsigned long pinCycleMs[10];
  int pinBounceCount[10];
  unsigned long pinDetectMs[10];
  unsigned long pinClearMs[10];
  int realPinCount = 0;

  unsigned long lastDetectMs = 0;
  bool haveDetect = false;
  int cSinceLastReal = 0;

  for (int i = 0; i < numEntries; i++) {
    TlTimingEvent &e = tlTimingLog[(startIdx + i) % TL_TIMING_MAX];
    if (e.type == 'D') {
      lastDetectMs = e.timeMs;
      haveDetect = true;
    } else if (e.type == 'C') {
      cSinceLastReal++;
      if (e.durationMs >= MIN_REAL_BLOCKED_MS && realPinCount < 10) {
        pinDetectMs[realPinCount] = haveDetect ? lastDetectMs : (e.timeMs - e.durationMs);
        pinClearMs[realPinCount] = e.timeMs;
        pinBlockedMs[realPinCount] = e.durationMs;
        pinBounceCount[realPinCount] = cSinceLastReal - 1;
        realPinCount++;
        cSinceLastReal = 0;
      }
      haveDetect = false;
    }
  }

  // Attribute trailing bounces (after last real pin) to last real pin
  if (realPinCount > 0 && cSinceLastReal > 0) {
    pinBounceCount[realPinCount - 1] += cSinceLastReal;
  }

  // Check for trailing detect (e.g., 10th pin still at sensor when turret released)
  bool haveTrailingDetect = false;
  unsigned long trailingGapMs = 0;
  if (haveDetect && realPinCount > 0 && lastDetectMs > pinClearMs[realPinCount - 1]) {
    haveTrailingDetect = true;
    trailingGapMs = lastDetectMs - pinClearMs[realPinCount - 1];
  }

  // Compute gap and cycle per real pin
  for (int i = 0; i < realPinCount; i++) {
    if (i == 0) {
      pinGapMs[i] = 0;
      pinCycleMs[i] = 0;
    } else {
      pinGapMs[i] = pinDetectMs[i] - pinClearMs[i - 1];
      pinCycleMs[i] = pinGapMs[i] + pinBlockedMs[i];
    }
  }

  if (realPinCount == 0) {
    Serial.println(F("   No pin events detected - cannot analyze."));
    return;
  }

  // --- Pass 1: Find baseline cycle time (median) ---
  // Median is robust to outliers at both ends (anomalously short pins
  // and jams/pauses). Also collect bounce stats.
  int totalBounces = 0;
  int maxBouncesPerPin = 0;
  unsigned long sortedCycles[9];
  int cycleCount = 0;

  for (int i = 0; i < realPinCount; i++) {
    if (pinCycleMs[i] > 0) {
      sortedCycles[cycleCount++] = pinCycleMs[i];
    }
    totalBounces += pinBounceCount[i];
    if (pinBounceCount[i] > maxBouncesPerPin) maxBouncesPerPin = pinBounceCount[i];
  }
  // Insertion sort (max 9 values)
  for (int i = 1; i < cycleCount; i++) {
    unsigned long key = sortedCycles[i];
    int j = i - 1;
    while (j >= 0 && sortedCycles[j] > key) {
      sortedCycles[j + 1] = sortedCycles[j];
      j--;
    }
    sortedCycles[j + 1] = key;
  }
  unsigned long medianCycle = (cycleCount > 0) ? sortedCycles[cycleCount / 2] : 0;

  // Also compute median blocked time for stuck-pin detection
  unsigned long sortedBlocked[10];
  for (int i = 0; i < realPinCount; i++) {
    sortedBlocked[i] = pinBlockedMs[i];
  }
  for (int i = 1; i < realPinCount; i++) {
    unsigned long key = sortedBlocked[i];
    int j = i - 1;
    while (j >= 0 && sortedBlocked[j] > key) {
      sortedBlocked[j + 1] = sortedBlocked[j];
      j--;
    }
    sortedBlocked[j + 1] = key;
  }
  unsigned long medianBlocked = (realPinCount > 0) ? sortedBlocked[realPinCount / 2] : 0;

  // --- Pass 2: Classify each pin ---
  // Uses median as the baseline (robust to outliers at both ends).
  //   Jam:     cycle > 3.5x median (long pause, mechanical issue)
  //   Outlier: cycle < 0.7x median (anomalously fast, sensor edge clip)
  //   Stuck:   blocked > 3x median blocked (pin stuck at sensor)
  // Jams, outliers, and stuck pins are excluded from all averages.
  bool pinIsJam[10];
  bool pinIsOutlier[10];
  int jamCount = 0;
  int outlierCount = 0;

  for (int i = 0; i < realPinCount; i++) {
    pinIsJam[i] = false;
    pinIsOutlier[i] = false;
    if (pinCycleMs[i] > 0 && medianCycle > 0) {
      float cycleRatio = (float)pinCycleMs[i] / (float)medianCycle;
      if (cycleRatio >= 3.5) {
        pinIsJam[i] = true;
        jamCount++;
        continue;
      }
      if (cycleRatio < 0.7) {
        pinIsOutlier[i] = true;
        outlierCount++;
        continue;
      }
    }
    // Also flag stuck pins: blocked time > 3x the median blocked
    if (medianBlocked > 0 && pinBlockedMs[i] > (medianBlocked * 3)) {
      pinIsJam[i] = true;
      jamCount++;
    }
  }

  // --- Pass 3: Compute stats excluding jams and outliers ---
  unsigned long totalBlocked = 0;
  unsigned long minBlocked = 999999UL;
  unsigned long maxBlocked = 0;
  int validBlockedCount = 0;

  // Classify valid pins: consecutive (<1.6x median cycle) or skipped door
  int consecutiveCount = 0;
  unsigned long consecutiveCycleTotal = 0;
  unsigned long consecutiveGapTotal = 0;
  unsigned long consecutiveBlockedTotal = 0;
  unsigned long minConsecutiveGap = 999999UL;
  unsigned long maxConsecutiveGap = 0;
  unsigned long minConsecutiveCycle = 999999UL;
  unsigned long maxConsecutiveCycle = 0;
  int skippedDoors = 0;

  for (int i = 0; i < realPinCount; i++) {
    if (pinIsJam[i] || pinIsOutlier[i]) continue;

    totalBlocked += pinBlockedMs[i];
    if (pinBlockedMs[i] < minBlocked) minBlocked = pinBlockedMs[i];
    if (pinBlockedMs[i] > maxBlocked) maxBlocked = pinBlockedMs[i];
    validBlockedCount++;

    if (pinCycleMs[i] > 0 && medianCycle > 0) {
      float ratio = (float)pinCycleMs[i] / (float)medianCycle;
      if (ratio < 1.6) {
        consecutiveCount++;
        consecutiveCycleTotal += pinCycleMs[i];
        consecutiveGapTotal += pinGapMs[i];
        consecutiveBlockedTotal += pinBlockedMs[i];
        if (pinGapMs[i] < minConsecutiveGap) minConsecutiveGap = pinGapMs[i];
        if (pinGapMs[i] > maxConsecutiveGap) maxConsecutiveGap = pinGapMs[i];
        if (pinCycleMs[i] < minConsecutiveCycle) minConsecutiveCycle = pinCycleMs[i];
        if (pinCycleMs[i] > maxConsecutiveCycle) maxConsecutiveCycle = pinCycleMs[i];
      } else {
        // Skipped doors: ratio ~2 = 1 skip, ~3 = 2 skips
        int missed = (int)(ratio + 0.5) - 1;
        skippedDoors += missed;
      }
    }
  }

  unsigned long avgBlocked = (validBlockedCount > 0) ? (totalBlocked / validBlockedCount) : 0;
  unsigned long avgConsecutiveCycle = (consecutiveCount > 0) ? (consecutiveCycleTotal / consecutiveCount) : 0;
  unsigned long avgConsecutiveGap = (consecutiveCount > 0) ? (consecutiveGapTotal / consecutiveCount) : 0;

  // Conveyor speed calculations using avg consecutive cycle time:
  //
  // Units/sec: one chair section averages 7.5 units (alternating 7 and 8).
  //   speed = 7.5 / (avgCycleMs / 1000) = 7500 / avgCycleMs
  //
  // RPM: the full chain is 44 units. One revolution = 44 units.
  //   rev/sec = speed / 44 = 7500 / (avgCycleMs * 44)
  //   RPM = rev/sec * 60 = 450000 / (avgCycleMs * 44)
  //       = 10227.27 / avgCycleMs
  //
  // Pins/min: one pin per cycle time, assuming every door loaded.
  //   pins/min = 60000 / avgCycleMs
  //   There are 6 chairs per revolution (44 units / ~7.33 avg = 6 chairs).
  float conveyorSpeed = 0;
  float conveyorRPM = 0;
  float pinsPerMinute = 0;
  if (avgConsecutiveCycle > 0) {
    conveyorSpeed = 7500.0f / (float)avgConsecutiveCycle;
    conveyorRPM = 10227.27f / (float)avgConsecutiveCycle;
    pinsPerMinute = 60000.0f / (float)avgConsecutiveCycle;
  }

  // --- Print analysis ---
  Serial.println(F(""));
  Serial.println(F("======== CONVEYOR TIMING ANALYSIS ========"));
  Serial.print(F("Pins analyzed: "));
  Serial.println(realPinCount + (haveTrailingDetect ? 1 : 0));
  Serial.println(F(""));

  // Per-pin breakdown
  // Column widths: blocked=5 ("NNNms"), gap=6 ("NNNNms"), cycle=6, bounces=2
  // Separators: 3 spaces between columns, 4 before bounces
  Serial.println(F("Per-pin breakdown:"));
  Serial.println(F("         blocked     gap    cycle  bounces"));
  for (int i = 0; i < realPinCount; i++) {
    Serial.print(F("  Pin "));
    if (i + 1 < 10) Serial.print(F(" "));
    Serial.print(i + 1);
    Serial.print(F(":  "));
    // Blocked: 3-digit + "ms" = 5 chars (values always >= 100)
    Serial.print(pinBlockedMs[i]);
    Serial.print(F("ms"));
    // Gap: 3-space sep + right-justified 4-digit + "ms" or "   ---"
    Serial.print(F("   "));
    if (pinGapMs[i] > 0) {
      if (pinGapMs[i] < 1000) Serial.print(F(" "));
      Serial.print(pinGapMs[i]);
      Serial.print(F("ms"));
    } else {
      Serial.print(F("   ---"));
    }
    // Cycle: 3-space sep + right-justified 4-digit + "ms" or "   ---"
    Serial.print(F("   "));
    if (pinCycleMs[i] > 0) {
      if (pinCycleMs[i] < 1000) Serial.print(F(" "));
      Serial.print(pinCycleMs[i]);
      Serial.print(F("ms"));
    } else {
      Serial.print(F("   ---"));
    }
    // Bounces: 4-space sep + right-justified 2-digit
    Serial.print(F("    "));
    if (pinBounceCount[i] < 10) Serial.print(F(" "));
    Serial.print(pinBounceCount[i]);
    // Note: non-consecutive or JAM
    if (pinIsJam[i]) {
      Serial.print(F("  JAM"));
    } else if (pinIsOutlier[i]) {
      Serial.print(F("  OUTLIER"));
    } else if (pinCycleMs[i] > 0 && medianCycle > 0) {
      float ratio = (float)pinCycleMs[i] / (float)medianCycle;
      if (ratio >= 1.6) {
        int missed = (int)(ratio + 0.5) - 1;
        Serial.print(F("  +"));
        Serial.print(missed);
        Serial.print(F(" non-consec"));
      }
    }
    Serial.println();
  }
  // Trailing detect row (pin detected but not yet cleared when turret released)
  if (haveTrailingDetect) {
    Serial.print(F("  Pin "));
    int tn = realPinCount + 1;
    if (tn < 10) Serial.print(F(" "));
    Serial.print(tn);
    Serial.print(F(":  "));
    Serial.print(F("  ---"));
    Serial.print(F("   "));
    if (trailingGapMs < 1000) Serial.print(F(" "));
    Serial.print(trailingGapMs);
    Serial.print(F("ms"));
    Serial.println(F("      ---"));
  }
  if (skippedDoors > 0 || jamCount > 0 || outlierCount > 0) {
    if (skippedDoors > 0) Serial.println(F("  (+N non-consec = N empty conveyor doors before this pin)"));
    if (jamCount > 0) Serial.println(F("  (JAM = pause/jam detected, excluded from averages)"));
    if (outlierCount > 0) Serial.println(F("  (OUTLIER = anomalous cycle time, excluded from averages)"));
  }

  // Summary
  Serial.println(F(""));
  Serial.println(F("--- Summary ---"));

  Serial.print(F("  Avg blocked time:       "));
  Serial.print(avgBlocked);
  Serial.print(F("ms ("));
  Serial.print(minBlocked);
  Serial.print(F("-"));
  Serial.print(maxBlocked);
  Serial.print(F("ms)"));
  if (avgBlocked >= 150 && avgBlocked <= 350) {
    Serial.println(F("   [GOOD]"));
  } else if (avgBlocked > 350) {
    Serial.println(F("   [SLOW]"));
  } else if (avgBlocked < 150) {
    Serial.println(F("   [FAST]"));
  }

  if (avgConsecutiveCycle > 0) {
    Serial.print(F("  Avg gap time:           "));
    Serial.print(avgConsecutiveGap);
    Serial.print(F("ms ("));
    Serial.print(minConsecutiveGap);
    Serial.print(F("-"));
    Serial.print(maxConsecutiveGap);
    Serial.println(F("ms)"));

    Serial.print(F("  Avg cycle time:         "));
    Serial.print(avgConsecutiveCycle);
    Serial.print(F("ms ("));
    Serial.print(minConsecutiveCycle);
    Serial.print(F("-"));
    Serial.print(maxConsecutiveCycle);
    Serial.print(F("ms)"));
    // Show count when some valid pins were non-consecutive (skipped doors)
    if (consecutiveCount < cycleCount - jamCount - outlierCount) {
      Serial.print(F(" ["));
      Serial.print(consecutiveCount);
      Serial.print(F(" consec. pins]"));
    }
    Serial.println(F(""));

    // Cycle time assessment
    Serial.print(F("  Conveyor speed:         ~"));
    Serial.print(conveyorSpeed, 1);
    Serial.println(F(" links+doors/sec"));

    Serial.print(F("  Conveyor RPM:           ~"));
    Serial.print(conveyorRPM, 1);
    Serial.println(F(" (6 pins per revolution)"));

    Serial.print(F("  Pins per minute:        ~"));
    Serial.print(pinsPerMinute, 1);
    Serial.println(F(" (if every door loaded)"));
  }

  if (skippedDoors > 0) {
    Serial.print(F("  Skipped conveyor doors: ~"));
    Serial.println(skippedDoors);
  }

  if (jamCount > 0) {
    Serial.print(F("  Jams/pauses detected:   "));
    Serial.print(jamCount);
    Serial.println(F("  (excluded from averages)"));
  }

  if (outlierCount > 0) {
    Serial.print(F("  Timing outliers:        "));
    Serial.print(outlierCount);
    Serial.println(F("  (excluded from averages)"));
  }

  Serial.print(F("  IR bounces:             "));
  Serial.print(totalBounces);
  Serial.print(F(" total, "));
  Serial.print(maxBouncesPerPin);
  Serial.print(F(" max/pin"));
  if (maxBouncesPerPin == 0) {
    Serial.println(F("   [IDEAL]"));
  } else if (maxBouncesPerPin < 3) {
    Serial.println(F("   [GOOD]"));
  } else if (maxBouncesPerPin <= 6) {
    Serial.println(F("   [MINOR]"));
  } else {
    Serial.println(F("   [HIGH]"));
  }

  // Reference ranges
  Serial.println(F(""));
  Serial.println(F("--- Reference ---"));
  Serial.println(F("  Blocked time:  150-350ms  (pin passing IR sensor)"));
  Serial.println(F("  Cycle time:    varies by machine (motor speed, chain gearing)"));
  Serial.println(F("  Note: The conveyor chain alternates 7-unit and 8-unit chair"));
  Serial.println(F("  sections, so ~14% variation between consecutive pins is normal."));
  Serial.println(F("  Bounces:       <5 per pin is normal"));

  // Recommendations
  Serial.println(F(""));
  Serial.println(F("--- Recommendations ---"));
  bool allGood = true;

  if (avgBlocked > 350) {
    Serial.println(F("  * Pins are slow past the sensor. Check belt tension, rollers,"));
    Serial.println(F("    and motor. Or IR sensor may need repositioning."));
    allGood = false;
  }
  if (avgBlocked < 150 && avgBlocked > 0) {
    Serial.println(F("  * Pins are unusually fast past the sensor. Verify the IR"));
    Serial.println(F("    sensor is positioned correctly and pins are being detected."));
    allGood = false;
  }
  if (maxBouncesPerPin >= 7) {
    Serial.println(F("  * High IR bounce count on one or more pins. Check IR sensor"));
    Serial.println(F("    alignment, wiring, and that pins pass cleanly through the beam."));
    allGood = false;
  }
  if (outlierCount > 0) {
    Serial.println(F("  * Timing outlier(s) detected — one or more pins had an"));
    Serial.println(F("    anomalously short cycle time. This usually means a pin"));
    Serial.println(F("    clipped the edge of the IR beam. If this happens often,"));
    Serial.println(F("    check IR sensor alignment."));
    allGood = false;
  }
  if (skippedDoors > 2) {
    Serial.println(F("  * Several conveyor doors were missed. For best results,"));
    Serial.println(F("    re-run the test loading pins on every consecutive door."));
    allGood = false;
  }
  if (jamCount > 0) {
    Serial.println(F("  * Jams or pauses were detected during loading. Check for"));
    Serial.println(F("    obstructions, bent conveyor doors, or pins getting stuck."));
    Serial.println(F("    Jam data was excluded from the timing averages above."));
    allGood = false;
  }
  if (allGood) {
    Serial.println(F("  Conveyor timing and IR sensor look normal!"));
  }

  Serial.println(F("=========================================="));
  Serial.println(F(""));
}

// =====================================================
// FULL TEST SEQUENCE
// =====================================================

void startFullTestWithClear() {
  // Clear timing log from previous sequence
  tlTimingLogCount = 0;
  tlTimingLogHead = 0;
  tlTimingPinCount = 0;
  tlTimingLoadNum = 0;
  fullTestActive = true;
  fullTestCycle = 1;
  fullTestPhase = FTEST_PRECLEAR;
  fullTestPhaseMs = millis();
  Serial.println(F(""));
  Serial.println(F(">> Starting FULL TEST sequence (with clear)..."));
  deckAll(C_WHITE());
  deckShow();
  Serial.println(F("   Deck LEDs on (white)."));
  Serial.println(F("   Running clear all first..."));
  startClearAll();
}

void startFullTest() {
  // Clear timing log from previous sequence
  tlTimingLogCount = 0;
  tlTimingLogHead = 0;
  tlTimingPinCount = 0;
  tlTimingLoadNum = 0;
  fullTestActive = true;
  fullTestCycle = 1;
  fullTestPhase = FTEST_CLOSE_BALL_DOOR;
  fullTestPhaseMs = millis();
  Serial.println(F(""));
  Serial.println(F(">> Starting FULL TEST sequence..."));
  deckAll(C_WHITE());
  deckShow();
  Serial.println(F("   Deck LEDs on (white)."));
  Serial.println(F("   Closing ball door..."));
  ensureBallDoorAttached();
  ballDoorAngle = BALL_DOOR_CLOSED_ANGLE;
  BallReturnServo.write(ballDoorAngle);
}

void runFullTestFSM() {
  if (!fullTestActive) return;

  unsigned long now = millis();

  switch (fullTestPhase) {

    case FTEST_PRECLEAR:
      fullTestPhase = FTEST_WAIT_PRECLEAR;
      break;

    case FTEST_WAIT_PRECLEAR:
      if (!clearAllActive) {
        Serial.println(F("   Clear all complete. Cycle 1: Loading turret..."));
        // Ball door already closed by clear all, skip to turret load
        startTurretLoad();
        fullTestPhase = FTEST_FIRST_LOAD;
      }
      break;

    case FTEST_CLOSE_BALL_DOOR:
      if (now - fullTestPhaseMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Ball door closed. Cycle 1: Loading turret..."));
        startTurretLoad();
        fullTestPhase = FTEST_FIRST_LOAD;
      }
      break;

    case FTEST_FIRST_LOAD:
      fullTestPhase = FTEST_WAIT_FIRST_LOAD;
      break;

    case FTEST_WAIT_FIRST_LOAD:
      if (turretPinsReleasedToDeck) {
        // Turret released pins and is already loading the next batch
        Serial.println(F("   Pins released to deck. Starting pin drop..."));
        fullTestPhase = FTEST_PIN_DROP;
      }
      break;

    case FTEST_PIN_DROP:
      turretPinsReleasedToDeck = false;
      startPinDrop(false, false);
      fullTestPhase = FTEST_WAIT_DROP;
      break;

    case FTEST_WAIT_DROP:
      if (!pinDropActive) {
        Serial.println(F("   Pin drop complete."));
        fullTestPhase = FTEST_START_BG_LOAD;
      }
      break;

    case FTEST_START_BG_LOAD:
      // Turret is already loading in the background (auto-continued after
      // release dwell). Just proceed to pin pickup.
      fullTestPhase = FTEST_PICKUP;
      break;

    case FTEST_PICKUP:
      Serial.println(F("   Starting pin pickup..."));
      startPinPickup();
      fullTestPhase = FTEST_WAIT_PICKUP;
      break;

    case FTEST_WAIT_PICKUP:
      if (!pinPickupActive) {
        Serial.println(F("   Pin pickup complete. Settling..."));
        fullTestPhaseMs = now;
        fullTestPhase = FTEST_SETTLE;
      }
      break;

    case FTEST_SETTLE:
      if (now - fullTestPhaseMs >= PDROP_SETTLE_MS) {
        Serial.println(F("   Starting pin set..."));
        fullTestPhase = FTEST_SET;
      }
      break;

    case FTEST_SET:
      startPinSet();
      fullTestPhase = FTEST_WAIT_SET;
      break;

    case FTEST_WAIT_SET:
      if (!pinSetActive) {
        Serial.println(F("   Pin set complete. Starting sweep clear..."));
        fullTestPhase = FTEST_SWEEP;
      }
      break;

    case FTEST_SWEEP:
      startSweepClear();
      fullTestPhase = FTEST_WAIT_SWEEP;
      break;

    case FTEST_WAIT_SWEEP:
      if (!sweepClearActive) {
        Serial.println(F("   Sweep complete. Releasing turret hold..."));
        fullTestPhase = FTEST_RELEASE;
      }
      break;

    case FTEST_RELEASE:
      turretLoadHoldRelease = false;
      fullTestPhase = FTEST_WAIT_BG_LOAD;
      break;

    case FTEST_WAIT_BG_LOAD:
      if (turretPinsReleasedToDeck) {
        // Turret released pins and is already loading the next batch
        fullTestPhase = FTEST_LOOP;
      }
      break;

    case FTEST_LOOP:
      Serial.print(F(">> Cycle "));
      Serial.print(fullTestCycle);
      if (fullTestTargetCycles > 0) {
        Serial.print(F(" of "));
        Serial.print(fullTestTargetCycles);
      }
      Serial.println(F(" complete."));
      // Check if we've reached the target
      if (fullTestTargetCycles > 0 && fullTestCycle >= fullTestTargetCycles) {
        fullTestActive = false;
        fullTestPhase = FTEST_IDLE;
        turretLoadHoldRelease = false;
        turretPinsReleasedToDeck = false;
        // Stop background turret load (conveyor may still be running)
        if (turretLoadActive) {
          turretLoadActive = false;
          turretLoadPhase = TLOAD_IDLE;
          ConveyorOff();
          conveyorIsOn = false;
        }
        deckAll(C_OFF());
        deckShow();
        Serial.print(F(">> Full test FINISHED ("));
        Serial.print(fullTestCycle);
        Serial.println(F(" cycles, deck LEDs off)"));
        break;
      }
      fullTestCycle++;
      Serial.print(F(">> Starting cycle "));
      Serial.print(fullTestCycle);
      if (fullTestTargetCycles > 0) {
        Serial.print(F(" of "));
        Serial.print(fullTestTargetCycles);
      }
      Serial.println(F("..."));
      fullTestPhase = FTEST_PIN_DROP;
      break;

    default:
      break;
  }
}

// =====================================================
// FRAME LED MENU
// =====================================================

void printFrameLEDMenu() {
  Serial.println(F(""));
  Serial.println(F("===== FRAME LED TEST (pins 46,47) ====="));
  Serial.println(F("Individual:"));
  Serial.println(F("  1on        - LED1 (pin 46) on"));
  Serial.println(F("  1off       - LED1 (pin 46) off"));
  Serial.println(F("  2on        - LED2 (pin 47) on"));
  Serial.println(F("  2off       - LED2 (pin 47) off"));
  Serial.println(F(""));
  Serial.println(F("Both:"));
  Serial.println(F("  on         - Both LEDs on"));
  Serial.println(F("  off (0)    - Both LEDs off"));
  Serial.println(F(""));
  Serial.println(F("Animations:"));
  Serial.println(F("  blink (bl) - Both blink together"));
  Serial.println(F("  alt (a)    - Alternating blink"));
  Serial.println(F("  stop (x)   - Stop animation"));
  Serial.println(F(""));
  Serial.println(F("  status (s) - Show status"));
  Serial.println(F("  back (b)   - Return to main menu"));
  Serial.println(F("========================================="));
  Serial.println(F(""));
}

void handleFrameLEDMenu(String cmd) {
  if (cmd == "1on") {
    frameBlinkMode = FBLINK_NONE;
    digitalWrite(FRAME_LED1_PIN, HIGH);
    Serial.println(F(">> LED1 ON"));
  }
  else if (cmd == "1off") {
    frameBlinkMode = FBLINK_NONE;
    digitalWrite(FRAME_LED1_PIN, LOW);
    Serial.println(F(">> LED1 OFF"));
  }
  else if (cmd == "2on") {
    frameBlinkMode = FBLINK_NONE;
    digitalWrite(FRAME_LED2_PIN, HIGH);
    Serial.println(F(">> LED2 ON"));
  }
  else if (cmd == "2off") {
    frameBlinkMode = FBLINK_NONE;
    digitalWrite(FRAME_LED2_PIN, LOW);
    Serial.println(F(">> LED2 OFF"));
  }
  else if (cmd == "on") {
    frameBlinkMode = FBLINK_NONE;
    digitalWrite(FRAME_LED1_PIN, HIGH);
    digitalWrite(FRAME_LED2_PIN, HIGH);
    Serial.println(F(">> Both LEDs ON"));
  }
  else if (cmd == "off" || cmd == "0") {
    frameBlinkMode = FBLINK_NONE;
    digitalWrite(FRAME_LED1_PIN, LOW);
    digitalWrite(FRAME_LED2_PIN, LOW);
    Serial.println(F(">> Both LEDs OFF"));
  }
  else if (cmd == "blink" || cmd == "bl") {
    frameBlinkMode = FBLINK_BOTH;
    frameBlinkState = false;
    frameBlinkLastMs = millis();
    Serial.println(F(">> Blinking both..."));
  }
  else if (cmd == "alt" || cmd == "a") {
    frameBlinkMode = FBLINK_ALT;
    frameBlinkState = false;
    frameBlinkLastMs = millis();
    Serial.println(F(">> Alternating blink..."));
  }
  else if (cmd == "stop" || cmd == "x") {
    frameBlinkMode = FBLINK_NONE;
    digitalWrite(FRAME_LED1_PIN, LOW);
    digitalWrite(FRAME_LED2_PIN, LOW);
    Serial.println(F(">> Stopped"));
  }
  else if (cmd == "status" || cmd == "s") {
    Serial.print(F("LED1 (pin 46): "));
    Serial.println(digitalRead(FRAME_LED1_PIN) == HIGH ? "ON" : "OFF");
    Serial.print(F("LED2 (pin 47): "));
    Serial.println(digitalRead(FRAME_LED2_PIN) == HIGH ? "ON" : "OFF");
    Serial.print(F("Blink mode: "));
    if (frameBlinkMode == FBLINK_NONE) {
      Serial.println(F("NONE"));
    } else if (frameBlinkMode == FBLINK_BOTH) {
      Serial.println(F("BOTH"));
    } else {
      Serial.println(F("ALTERNATING"));
    }
  }
  else {
    Serial.print(F("Unknown: "));
    Serial.println(cmd);
  }
}

void updateFrameBlink() {
  if (frameBlinkMode == FBLINK_NONE) return;

  unsigned long now = millis();
  if (now - frameBlinkLastMs >= FRAME_BLINK_INTERVAL) {
    frameBlinkLastMs = now;
    frameBlinkState = !frameBlinkState;

    if (frameBlinkMode == FBLINK_BOTH) {
      digitalWrite(FRAME_LED1_PIN, frameBlinkState ? HIGH : LOW);
      digitalWrite(FRAME_LED2_PIN, frameBlinkState ? HIGH : LOW);
    }
    else if (frameBlinkMode == FBLINK_ALT) {
      digitalWrite(FRAME_LED1_PIN, frameBlinkState ? HIGH : LOW);
      digitalWrite(FRAME_LED2_PIN, frameBlinkState ? LOW : HIGH);
    }
  }
}

// =====================================================
// LED HELPER FUNCTIONS
// =====================================================

// Set all pixels on both deck strips
void deckAll(uint32_t col) {
  for (int i = 0; i < DECK_LED_LENGTH_L; i++) {
    deckL.setPixelColor(i, col);
  }
  for (int i = 0; i < DECK_LED_LENGTH_R; i++) {
    deckR.setPixelColor(i, col);
  }
}

// Set all pixels on both lane strips
void laneAll(uint32_t col) {
  for (int i = 0; i < LANE_LED_LENGTH_L; i++) {
    laneL.setPixelColor(i, col);
  }
  for (int i = 0; i < LANE_LED_LENGTH_R; i++) {
    laneR.setPixelColor(i, col);
  }
}

// Set color based on deck strip selection
void deckSetColor(uint32_t col) {
  if (deckStripSelect == STRIP_BOTH || deckStripSelect == STRIP_LEFT) {
    for (int i = 0; i < DECK_LED_LENGTH_L; i++) {
      deckL.setPixelColor(i, col);
    }
  }
  if (deckStripSelect == STRIP_BOTH || deckStripSelect == STRIP_RIGHT) {
    for (int i = 0; i < DECK_LED_LENGTH_R; i++) {
      deckR.setPixelColor(i, col);
    }
  }
}

// Set color based on lane strip selection
void laneSetColor(uint32_t col) {
  if (laneStripSelect == STRIP_BOTH || laneStripSelect == STRIP_LEFT) {
    for (int i = 0; i < LANE_LED_LENGTH_L; i++) {
      laneL.setPixelColor(i, col);
    }
  }
  if (laneStripSelect == STRIP_BOTH || laneStripSelect == STRIP_RIGHT) {
    for (int i = 0; i < LANE_LED_LENGTH_R; i++) {
      laneR.setPixelColor(i, col);
    }
  }
}

// Show both deck strips
void deckShow() {
  deckL.show();
  deckR.show();
}

// Show both lane strips
void laneShow() {
  laneL.show();
  laneR.show();
}

// Show selected deck strip(s)
void deckShowSelected() {
  if (deckStripSelect == STRIP_BOTH || deckStripSelect == STRIP_LEFT) {
    deckL.show();
  }
  if (deckStripSelect == STRIP_BOTH || deckStripSelect == STRIP_RIGHT) {
    deckR.show();
  }
}

// Show selected lane strip(s)
void laneShowSelected() {
  if (laneStripSelect == STRIP_BOTH || laneStripSelect == STRIP_LEFT) {
    laneL.show();
  }
  if (laneStripSelect == STRIP_BOTH || laneStripSelect == STRIP_RIGHT) {
    laneR.show();
  }
}

// Set brightness for selected deck strip(s)
void deckSetBrightnessSelected(uint8_t br) {
  if (deckStripSelect == STRIP_BOTH || deckStripSelect == STRIP_LEFT) {
    deckL.setBrightness(br);
  }
  if (deckStripSelect == STRIP_BOTH || deckStripSelect == STRIP_RIGHT) {
    deckR.setBrightness(br);
  }
}

// Set brightness for selected lane strip(s)
void laneSetBrightnessSelected(uint8_t br) {
  if (laneStripSelect == STRIP_BOTH || laneStripSelect == STRIP_LEFT) {
    laneL.setBrightness(br);
  }
  if (laneStripSelect == STRIP_BOTH || laneStripSelect == STRIP_RIGHT) {
    laneR.setBrightness(br);
  }
}

// =====================================================
// LED ANIMATION FUNCTIONS
// =====================================================

void startWipeAnim(uint32_t col) {
  animColor = col;
  animStartMs = millis();
  animLastFrameMs = 0;
  animMode = ANIM_WIPE;

  if (ledAnimTarget == LED_DECK) {
    deckSetColor(C_OFF());
    deckShowSelected();
  } else {
    laneSetColor(C_OFF());
    laneShowSelected();
  }
}

void startFlashAnim(uint32_t col) {
  animColor = col;
  flashOnPhase = true;
  flashCycles = 0;
  animStartMs = millis();
  animMode = ANIM_FLASH;

  if (ledAnimTarget == LED_DECK) {
    deckSetColor(animColor);
    deckShowSelected();
  } else {
    laneSetColor(animColor);
    laneShowSelected();
  }
}

void startCometAnim() {
  animStartMs = millis();
  animLastFrameMs = 0;
  animMode = ANIM_COMET;

  if (ledAnimTarget == LED_DECK) {
    deckSetColor(C_OFF());
    deckShowSelected();
  } else {
    laneSetColor(C_OFF());
    laneShowSelected();
  }
}

void startRainbowAnim() {
  animLastFrameMs = millis();
  rainbowOffset = 0;
  animMode = ANIM_RAINBOW;
}

void updateLEDAnimation() {
  unsigned long now = millis();

  int lenL, lenR;
  if (ledAnimTarget == LED_DECK) {
    lenL = DECK_LED_LENGTH_L;
    lenR = DECK_LED_LENGTH_R;
  } else {
    lenL = LANE_LED_LENGTH_L;
    lenR = LANE_LED_LENGTH_R;
  }
  StripSelect currentSelect = (ledAnimTarget == LED_DECK) ? deckStripSelect : laneStripSelect;

  // WIPE
  if (animMode == ANIM_WIPE) {
    unsigned long elapsed = now - animStartMs;
    if (elapsed >= WIPE_MS) {
      // Wipe done - start flash if red (strike)
      if (animColor == C_RED()) {
        startFlashAnim(C_RED());
      } else {
        animMode = ANIM_IDLE;
        if (ledAnimTarget == LED_DECK) {
          deckSetColor(animColor);
          deckShowSelected();
        } else {
          laneSetColor(animColor);
          laneShowSelected();
        }
        Serial.println(F("   Wipe complete"));
      }
    } else if (animLastFrameMs == 0 || (now - animLastFrameMs) >= WIPE_FRAME_MS) {
      animLastFrameMs = now;
      float t = (float)elapsed / (float)WIPE_MS;
      int nL = (int)(t * lenL + 0.5f);
      int nR = (int)(t * lenR + 0.5f);

      if (ledAnimTarget == LED_DECK) {
        if (currentSelect == STRIP_BOTH || currentSelect == STRIP_LEFT) {
          for (int i = 0; i < lenL; i++) {
            deckL.setPixelColor(i, (i < nL) ? animColor : C_OFF());
          }
        }
        if (currentSelect == STRIP_BOTH || currentSelect == STRIP_RIGHT) {
          for (int i = 0; i < lenR; i++) {
            deckR.setPixelColor(i, (i < nR) ? animColor : C_OFF());
          }
        }
        deckShowSelected();
      } else {
        if (currentSelect == STRIP_BOTH || currentSelect == STRIP_LEFT) {
          for (int i = 0; i < lenL; i++) {
            laneL.setPixelColor(i, (i < nL) ? animColor : C_OFF());
          }
        }
        if (currentSelect == STRIP_BOTH || currentSelect == STRIP_RIGHT) {
          for (int i = 0; i < lenR; i++) {
            laneR.setPixelColor(i, (i < nR) ? animColor : C_OFF());
          }
        }
        laneShowSelected();
      }
    }
  }

  // FLASH
  if (animMode == ANIM_FLASH) {
    if (flashOnPhase) {
      if (now - animStartMs >= FLASH_ON_MS) {
        flashOnPhase = false;
        animStartMs = now;
        if (ledAnimTarget == LED_DECK) {
          deckSetColor(C_OFF());
          deckShowSelected();
        } else {
          laneSetColor(C_OFF());
          laneShowSelected();
        }
      }
    } else {
      if (now - animStartMs >= FLASH_OFF_MS) {
        animStartMs = now;
        flashCycles++;
        if (flashCycles >= FLASH_COUNT) {
          animMode = ANIM_IDLE;
          if (ledAnimTarget == LED_DECK) {
            deckSetBrightnessSelected(DECK_LED_BRIGHTNESS);
            deckSetColor(currentDeckColor);
            deckShowSelected();
          } else {
            laneSetBrightnessSelected(LED_BRIGHTNESS_NORMAL);
            laneSetColor(currentLaneColor);
            laneShowSelected();
          }
          Serial.println(F("   Flash complete"));
        } else {
          flashOnPhase = true;
          if (ledAnimTarget == LED_DECK) {
            deckSetColor(animColor);
            deckShowSelected();
          } else {
            laneSetColor(animColor);
            laneShowSelected();
          }
        }
      }
    }
  }

  // COMET
  if (animMode == ANIM_COMET) {
    unsigned long elapsed = now - animStartMs;
    if (elapsed >= COMET_MS) {
      animMode = ANIM_IDLE;
      if (ledAnimTarget == LED_DECK) {
        deckSetColor(currentDeckColor);
        deckShowSelected();
      } else {
        laneSetColor(currentLaneColor);
        laneShowSelected();
      }
      Serial.println(F("   Comet complete"));
    } else if (now - animLastFrameMs >= COMET_FRAME_MS) {
      animLastFrameMs = now;
      float t = (float)elapsed / (float)COMET_MS;
      int maxIndexL = lenL + COMET_LEN;
      int maxIndexR = lenR + COMET_LEN;
      int headL = (int)(t * maxIndexL + 0.5f);
      int headR = (int)(t * maxIndexR + 0.5f);

      if (ledAnimTarget == LED_DECK) {
        deckSetColor(C_OFF());
        for (int k = 0; k < COMET_LEN; k++) {
          uint8_t br = 255 - (k * 60);
          uint32_t col = deckL.Color(br, br, br);
          if (currentSelect == STRIP_BOTH || currentSelect == STRIP_LEFT) {
            int idx = headL - k;
            if (idx >= 0 && idx < lenL) {
              deckL.setPixelColor(idx, col);
            }
          }
          if (currentSelect == STRIP_BOTH || currentSelect == STRIP_RIGHT) {
            int idx = headR - k;
            if (idx >= 0 && idx < lenR) {
              deckR.setPixelColor(idx, col);
            }
          }
        }
        deckShowSelected();
      } else {
        laneSetColor(C_OFF());
        for (int k = 0; k < COMET_LEN; k++) {
          uint8_t br = 255 - (k * 60);
          uint32_t col = laneL.Color(br, br, br);
          if (currentSelect == STRIP_BOTH || currentSelect == STRIP_LEFT) {
            int idx = headL - k;
            if (idx >= 0 && idx < lenL) {
              laneL.setPixelColor(idx, col);
            }
          }
          if (currentSelect == STRIP_BOTH || currentSelect == STRIP_RIGHT) {
            int idx = headR - k;
            if (idx >= 0 && idx < lenR) {
              laneR.setPixelColor(idx, col);
            }
          }
        }
        laneShowSelected();
      }
    }
  }

  // RAINBOW
  if (animMode == ANIM_RAINBOW) {
    if (now - animLastFrameMs >= RAINBOW_FRAME_MS) {
      animLastFrameMs = now;

      if (ledAnimTarget == LED_DECK) {
        if (currentSelect == STRIP_BOTH || currentSelect == STRIP_LEFT) {
          for (int i = 0; i < lenL; i++) {
            deckL.setPixelColor(i, wheel((i + rainbowOffset) & 255));
          }
        }
        if (currentSelect == STRIP_BOTH || currentSelect == STRIP_RIGHT) {
          for (int i = 0; i < lenR; i++) {
            deckR.setPixelColor(i, wheel((i + rainbowOffset) & 255));
          }
        }
        deckShowSelected();
      } else {
        if (currentSelect == STRIP_BOTH || currentSelect == STRIP_LEFT) {
          for (int i = 0; i < lenL; i++) {
            laneL.setPixelColor(i, wheel((i + rainbowOffset) & 255));
          }
        }
        if (currentSelect == STRIP_BOTH || currentSelect == STRIP_RIGHT) {
          for (int i = 0; i < lenR; i++) {
            laneR.setPixelColor(i, wheel((i + rainbowOffset) & 255));
          }
        }
        laneShowSelected();
      }
      rainbowOffset++;
    }
  }
}

uint32_t wheel(byte wheelPos) {
  wheelPos = 255 - wheelPos;
  if (wheelPos < 85) {
    return deckL.Color(255 - wheelPos * 3, 0, wheelPos * 3);
  }
  if (wheelPos < 170) {
    wheelPos -= 85;
    return deckL.Color(0, wheelPos * 3, 255 - wheelPos * 3);
  }
  wheelPos -= 170;
  return deckL.Color(wheelPos * 3, 255 - wheelPos * 3, 0);
}
