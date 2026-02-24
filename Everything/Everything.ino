// =====================================================
// Pinsetter Deck + Turret Controller
// NeoPixels serial-safe
// - Boot: quick white wipe at END of setup, then solid white
// - Strike: red wipe, quick red flashes, then back to white
// - Ball-pass comet (3–4 LEDs) ~500 ms (immediate on sensor)
// - Sequencer pause model:
//     Finish current motion (e.g., sweep tween), then PAUSE steps
//     while lane animations run; resume automatically after.
// - During animations: update ONLY lane strips (shorter interrupt),
//   and dim lane brightness for strike to reduce current spikes.
//
// NEW (Pause Mode):
// - If no BALL_SENSOR trigger for 2 minutes AND sweep is SweepUp() AND machine is idle,
//   move sweep to SweepGuard() and set all lights RED.
// - Stay paused until bowler waves finger through IR_SENSOR beam.
//   On that beam-break edge: SweepUp() and lights WHITE.
// - While paused, IR pin queueing is disabled so the "wave" doesn't enqueue pins.
// =====================================================

#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <AccelStepper.h>
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

// Current version, will be used by Scoremore to determine supported features
#define VERSION "1.2.3"

Adafruit_NeoPixel deckL(DECK_LED_LENGTH_L, DECK_PIN_L, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel deckR(DECK_LED_LENGTH_R, DECK_PIN_R, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel laneL(LANE_LED_LENGTH_L, LANE_PIN_L, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel laneR(LANE_LED_LENGTH_R, LANE_PIN_R, NEO_GRB + NEO_KHZ800);

static inline uint32_t C_WHITE(Adafruit_NeoPixel &s){ return s.Color(255,255,255); }
static inline uint32_t C_RED  (Adafruit_NeoPixel &s){ return s.Color(255,  0,  0); }
static inline uint32_t C_GREEN(Adafruit_NeoPixel &s){ return s.Color(  0,255,  0); }
static inline uint32_t C_OFF  (Adafruit_NeoPixel &s){ return s.Color(  0,  0,  0); }

// --- LED mode ---
enum LaneAnimMode { LANE_IDLE_WHITE=0, LANE_STRIKE_WIPE, LANE_STRIKE_FLASH, LANE_BALL_COMET };
LaneAnimMode laneMode = LANE_IDLE_WHITE;
static inline bool laneAnimActive(){ return laneMode==LANE_BALL_COMET||laneMode==LANE_STRIKE_WIPE||laneMode==LANE_STRIKE_FLASH; }
bool lanePauseArmed=false, lanePaused=false;

bool scoreWindowActive=false, strikePending=false, lightsShownAfterBoot=false;

unsigned long strikeWipeStartMs=0, strikeLastFrameMs=0, flashLastMs=0;
bool flashOnPhase=false; int flashCycles=0;

unsigned long ballCometStartMs=0, ballCometLastFrame=0;

// ====== FRAME STATE LEDS ======

// Helpers
void ledsBegin(); void deckAll(uint32_t col); void laneAll(uint32_t col); void ledsShowAll(); void laneShowOnly();
void laneUpdate(); void startStrikeWipe(); void startStrikeFlash(); void startBallCometImmediate();
void endAllLaneAnimsToWhite(); void startupWipeWhiteQuick();

// NEW: frame LED helpers
void updateFrameLEDs();
void frameLEDsFirstHalf();

// =============== ScoreMore mapping =================
struct PinMapping {
  int scoreMore;
  int arduino;
  bool isBowling;
};

const PinMapping pinMap[] = {
  { SM_PIN_2,        A2,                    true  },  // Bowling pin 2: we map to A2, which is unused
  { SM_PIN_3,        A3,                    true  },  // Bowling pin 3: we map to A3, which is unused
  { SM_PIN_4,        A4,                    true  },  // Bowling pin 4: we map to A4, which is unused
  { SM_PIN_5,        A5,                    true  },  // Bowling pin 5: we map to A5, which is unused
  { SM_BALL_TRIGGER, BALL_SENSOR_PIN,       false },  // Trigger sensor
  { SM_AUTO_RESET,   40,                    false },  // Auto reset trigger
  { SM_SPEED_SENSOR, BALL_SPEED_PIN,        false },  // Ball speed sensor
  { SM_SPARE_LIGHT,  42,                    false },  // Spare/strike light
  { SM_STRIKE_LIGHT, 43,                    false },  // Strike light
  { SM_FIRST_BALL,   44,                    false },  // 1st ball light - we could map this to the actual used 46 instead
  { SM_SECOND_BALL,  45,                    false },  // 2nd ball light - we could map this to the actual used 47 instead
  { SM_PIN_1,        A11,                   true  },  // Bowling pin 1: we map to A11, which is unused
  { SM_PIN_6,        A6,                    true  },  // Bowling pin 6: we map to A6, which is unused
  { SM_PIN_7,        A7,                    true  },  // Bowling pin 7: we map to A7, which is unused
  { SM_PIN_8,        A8,                    true  },  // Bowling pin 8: we map to A8, which is unused
  { SM_PIN_9,        A9,                    true  },  // Bowling pin 9: we map to A9, which is unused
  { SM_PIN_10,       A10,                   true  },  // Bowling pin 10: we map to A10, which is unused
  { SM_PINSETTER_RESET, PINSETTER_RESET_PIN,   false  },
};
const int maxPins = sizeof(pinMap) / sizeof(pinMap[0]);

Servo LeftRaiseServo, RightRaiseServo, SlideServo, ScissorsServo, LeftSweepServo, RightSweepServo, BallReturnServo;

// ======================= PAUSE MODE =======================
bool pauseMode = false;
unsigned long lastBallActivityMs = 0;

// Track sweep pose so we can reliably check "sweep is up"
enum SweepPose { SWEEP_UNKNOWN=0, SWEEP_UP, SWEEP_GUARD, SWEEP_BACK };
SweepPose sweepPoseCur = SWEEP_UNKNOWN;
SweepPose sweepPoseTarget = SWEEP_UNKNOWN;

void enterPauseMode();
void exitPauseMode();
static inline bool isReadyIdleForPause();
void setAllLightsRed();
void setAllLightsWhite();

// ======================= SEQUENCER STATE =======================
unsigned long prevStepMillis=0, prevScoreMillis=0;
int stepIndex=0;

// ---- ScoreMore ball trigger mirroring ----
const int SCOREMORE_BALL_LOGICAL_PIN = 6;
bool smBallPulseActive = false;
unsigned long smBallPulseStart = 0;
void scoremoreBallPulse_begin(){
  Serial.print("INPUT_CHANGE:"); Serial.print(SCOREMORE_BALL_LOGICAL_PIN); Serial.println(":0");
  smBallPulseActive = true;
  smBallPulseStart = millis();
}
void scoremoreBallPulse_update(){
  if(!smBallPulseActive) return;
  if(millis() - smBallPulseStart >= SCOREMORE_BALL_PULSE_MS){
    Serial.print("INPUT_CHANGE:"); Serial.print(SCOREMORE_BALL_LOGICAL_PIN); Serial.println(":1");
    smBallPulseActive = false;
  }
}

// Ball trigger
int  ballPrev=HIGH; bool ballPending=false, ballRearmed=true, waitingForBall=true;
unsigned long ballLowStartUs=0, lastBallHighMs=0;
int  throwCount=1;

// Strike light
bool strikeLightOn=false, strikeEdgeLatched=false, strikeDetected=false;

// Reset button
bool pinsetterResetRequested=false;

// ===== Fill-ball (ScoreMore logical pin 7) =====
bool autoResetFillBall = false;
bool autoResetEdgeLatched = false;
bool inFillBall = false;
bool fillBallShotInProgress = false;

// Inputs
int inputPins[maxPins], pinStates[maxPins], inputCount=0;

// ======================= TURRET / STEPPER =======================
AccelStepper stepper1(1, STEP_PIN, DIR_PIN);

bool ninthSettleActive=false, catchDelayActive=false, releaseDwellActive=false;
unsigned long ninthSettleStart=0, catchDelayStart=0, releaseDwellStart=0;

// Pin positions
const int PinPositions[]={PIN_POS_0, PIN_POS_1, PIN_POS_2, PIN_POS_3, PIN_POS_4, PIN_POS_5, PIN_POS_6, PIN_POS_7, PIN_POS_8, PIN_POS_9, PIN_POS_10};
static inline long Pin10ReleasePos(){ return PinPositions[10] + TURRET_PIN10_RELEASE_OFFSET; }

int NowCatching=1, loadedCount=0;

bool moving=false; long targetPos=0;
bool emptyTurretReturnActive = false;   // true only while EmptyTurret is returning to hall

// NEW: queued pin events while turret is busy
int queuedPinEvents = 0;

int irStableState=HIGH, irLastRead=HIGH; unsigned long irLastChange=0; bool pinEdgeArmed=true;

bool turretReleaseRequested=false, turretFillTo9Requested=false;
bool conveyorLockedByDwell=false, suspendConveyorUntilHomeDone=false;

unsigned long lastPinCatchMs = 0;

// Non-blocking homing
bool homingActive=false, postDropHomePending=false;
enum HomingPhase {
  HOME_IDLE=0, HOME_PREP_MOVE_TO_SLOT1, HOME_ADVANCE_TO_SWITCH,
  HOME_BACKOFF, HOME_CREEP_TO_SWITCH, HOME_SETZERO_AND_MOVE_SLOT1, HOME_DONE
};
HomingPhase homingPhase=HOME_IDLE;

bool dropCycleJustFinished=false;

// Deck pose flags
bool deckIsUp=false;

// authoritative deck cones
int deckConeCount=0;
bool deckHasTenReady=false;

// Background refill controller
bool backgroundRefillRequested=false;

// Force conveyors for ball return
bool forceConveyorForBallReturn=false;
bool conveyorIsOn=false;
// ======================= BALL RETURN DOOR FSM =======================
enum BallReturnState { BR_IDLE_OPEN=0, BR_IDLE_CLOSED, BR_CLOSED_WAIT_SWEEPBACK, BR_CLOSED_HOLD_AFTER_SWEEP };
BallReturnState brState=BR_IDLE_CLOSED;
unsigned long brStateStart=0;

void onBallThrownDoorClose(); void onSweepBackDoorHoldStart(); void updateBallReturnDoor();

// ======================= SWEEP TWEEN =======================
int sweepStartL=0,sweepStartR=0,sweepTargetL=0,sweepTargetR=0,sweepCurL=0,sweepCurR=0;
unsigned long sweepStartMs=0, sweepDurationMs=500; bool sweepAnimating=false;

void runSequence(); unsigned long stepDuration(int idx);
void DeckUp(); void DeckPinSet(); void DeckPinGrab(); void DeckPinDrop();
void SlidingDeckRelease(); void SlidingDeckHome(); void ScissorsGrab(); void ScissorsDrop();
void SweepGuard(); void SweepUp(); void SweepBack();
void StrikeSweepClearLane();
void checkSerial(); void handleCommand(String cmd); void checkInputChanges();
bool isTrackedInput(int pin); void removeInputPin(int pin);
int resolveArduinoPin(int scoreMorePin); int getScoreMorePin(int arduinoPin); bool isBowlingPin(int scoreMorePin);

// Turret
void runTurret(); void onPinDetected(); void goTo(long pos); void servicePinQueue();

// Non-blocking homing
void startHomeTurret(); void runHomingFSM();

// Misc demo
void EmptyTurret();

// Coordination
void startStrikeCycle(); void startTurretReleaseCycle();

// Conveyors
void updateConveyorOutput(); void ConveyorOn(); void ConveyorOff();

// Sweep tween
static int clampInt(int v,int lo,int hi);
void setSweepInstant(int leftDeg,int rightDeg);
void startSweepTo(int leftDeg,int rightDeg,unsigned long durMs=500);
void updateSweepTween(); void waitSweepDone(unsigned long timeoutMs=3000);

void PowerOnSequence(); void PrimeFullRackAndSetLaneOnce(); void pumpAll(unsigned long ms);

// ==== REFILL-LOCK ====
bool refillLockActive=false;
static inline bool refillInProgress(){ return refillLockActive && (deckConeCount<10); }

// ---- Cones-full hold ----
bool conesFullHold=false;
bool conesFullHoldArmed=false;
unsigned long conesFullHoldStartMs=0;

bool postSetResumeDelayActive = false;
unsigned long postSetResumeStart = 0;

// ---- PAUSE FOR SCOREMORE IF SCOREMORE_USER = 1 ----
bool waitForScoreMore(){
  unsigned long start = millis();

  Serial.println("WAITING_FOR_SCOREMORE");

  while (true) {
    if (Serial && Serial.available() > 0) {
      Serial.println("SCOREMORE_CONNECTED");
      return true;
    }

    // LED's only - no motion from servo's
    laneUpdate();
    delay(5);
  }
}

// ======================= SETUP =======================
void setup(){
  ledsBegin();
  pinMode(PINSETTER_RESET_PIN, INPUT_PULLUP);
  // Frame LEDs
  pinMode(FRAME_LED1_PIN, OUTPUT);
  pinMode(FRAME_LED2_PIN, OUTPUT);
  digitalWrite(FRAME_LED1_PIN, LOW);
  digitalWrite(FRAME_LED2_PIN, HIGH);  //indicate to the user that we're in setup

// =========== WAIT FOR SCOREMORE =================  
  Serial.begin(SCOREMORE_BAUD); delay(1000);

  #if SCOREMORE_USER == 1
    waitForScoreMore();   // Blocks BEFORE any servo attaches
  #endif

  Serial.println("READY");


  digitalWrite(FRAME_LED2_PIN, LOW);

  pinMode(BALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BALL_SPEED_PIN, INPUT_PULLUP);

  stepper1.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
  stepper1.setAcceleration(TURRET_NORMAL_ACCEL);

  LeftRaiseServo.attach(RAISE_LEFT_PIN);
  RightRaiseServo.attach(RAISE_RIGHT_PIN);
  ScissorsServo.attach(SCISSOR_PIN);
  
  DeckPinDrop(); delay(1000);
  ScissorsGrab(); delay(1000);
  ScissorsDrop(); delay(1000);

  BallReturnServo.attach(BALL_RETURN_PIN);
  DeckUp(); delay(1000);
  
  BallReturnClosed(); brState=BR_IDLE_CLOSED; brStateStart=millis();
  delay(1000);

  LeftSweepServo.attach(LEFT_SWEEP_PIN);
  RightSweepServo.attach(RIGHT_SWEEP_PIN);
  
  // this is a fix from @Bob D to keep the sweep from freaking out on initiaization.
  setSweepInstant(SWEEP_UP_ANGLE, 180 - SWEEP_UP_ANGLE); 
  SweepUp(); waitSweepDone(); pumpAll(1000);

  pinMode(MOTOR_RELAY_PIN, OUTPUT); ConveyorOff();
  pinMode(IR_SENSOR_PIN, INPUT);

  // ===== NEW: Initialize IR debounce & queue if beam starts blocked =====
  irLastRead    = digitalRead(IR_SENSOR_PIN);
  irStableState = irLastRead;
  irLastChange  = millis();
  if (irStableState == LOW) {
    // Beam is already blocked at boot -> treat as one pin event
    pinEdgeArmed     = false;
    queuedPinEvents += 1;
  } else {
    pinEdgeArmed = true;
  }
  // =====================================================================

  pinMode(HALL_EFFECT_PIN, INPUT_PULLUP);

  PowerOnSequence();

  lastPinCatchMs = millis();

  // Initial home (blocking only here)
  startHomeTurret();
  while(homingActive){
    runTurret(); updateConveyorOutput(); updateBallReturnDoor(); updateSweepTween(); laneUpdate(); delay(1);
  }

  NowCatching=1; goTo(PinPositions[1]); loadedCount=0;
  turretFillTo9Requested=true;

  PrimeFullRackAndSetLaneOnce();

  SweepUp(); waitSweepDone(); pumpAll(500);

  startupWipeWhiteQuick(); lightsShownAfterBoot=true;

  waitingForBall=true; throwCount=1; stepIndex=0;
  prevStepMillis=millis(); prevScoreMillis=millis();

  // NEW: initialize pause timer baseline
  lastBallActivityMs = millis();

  updateFrameLEDs();
}

// ======================= LOOP =======================
void loop(){
  unsigned long now=millis();

  updateSweepTween();
  runTurret();
  updateConveyorOutput();
  updateBallReturnDoor();
  laneUpdate();

  scoremoreBallPulse_update();

  if(refillLockActive && deckConeCount==10){ refillLockActive=false; }

  if(now - prevScoreMillis >= SCORE_INTERVAL){
    prevScoreMillis=now; checkSerial(); checkInputChanges();
  }

  // ======================= PAUSE MODE (NEW) =======================
  // If paused: a fresh IR beam-break edge resumes play.
  if(pauseMode){
    if(pinEdgeArmed && irStableState==LOW){
      pinEdgeArmed = false;     // consume this edge
      exitPauseMode();
    }
  } else {
    // Enter pause only when idle-ready, sweep is up, and we've had no ball for 2 minutes.
    if(isReadyIdleForPause() && (millis() - lastBallActivityMs >= PAUSE_IDLE_MS)){
      enterPauseMode();
    }
  }
  // ===============================================================

  // Ball trigger
  int rawBall=digitalRead(BALL_SENSOR_PIN);
  if(rawBall==LOW && ballPrev==HIGH){ ballPending=true; ballLowStartUs=micros(); }
  if(ballPending){
    if(rawBall==LOW){
      if((micros()-ballLowStartUs)>=BALL_LOW_CONFIRM_US && waitingForBall && ballRearmed){

        // If paused and a ball arrives anyway, immediately resume (fail-safe)
        if(pauseMode){
          exitPauseMode();
        }

        waitingForBall=false;
        lastBallActivityMs = millis();   // NEW: refresh idle timer on ball
        scoreWindowActive=true;

        scoremoreBallPulse_begin();

        startBallCometImmediate();
        onBallThrownDoorClose();

        lastPinCatchMs = millis();

        if (inFillBall) {
          stepIndex = 22;
          fillBallShotInProgress = true;
        } else {
          stepIndex = (throwCount==1) ? 1 : 22;
        }

        if(throwCount==1){ forceConveyorForBallReturn=true; }
        prevStepMillis=now; ballRearmed=false; lastBallHighMs=millis(); ballPending=false;
      }
    }else{ ballPending=false; }
  }
  if(!ballRearmed && (millis()-lastBallHighMs>=BALL_REARM_MS) && rawBall==HIGH){ ballRearmed=true; }
  ballPrev=rawBall;

  if(stepIndex>0){
     runSequence();
  } else {
    if(pinsetterResetRequested) {  //if the reset button has been pressed and no other sequences are queued
      stepIndex=22;       //trigger reset of the lane
      pinsetterResetRequested=false;
    }
  }
}

// ======================= SEQUENCER =======================
void runSequence(){
  unsigned long now=millis();
  unsigned long need=stepDuration(stepIndex);
  if((now - prevStepMillis < need)) return;

  if(laneAnimActive()) lanePauseArmed=true;
  if(lanePauseArmed){
    if(sweepAnimating) return;
    if(laneAnimActive()) lanePaused=true; else { lanePauseArmed=false; lanePaused=false; }
  }
  if(lanePaused) return;

  // ---------- Throw #1 ----------
  if(stepIndex==1){
    if(strikeDetected){
      scoreWindowActive=false;
      Serial.println("STRIKE@STEP1");
      ScissorsDrop();
      StrikeSweepClearLane();
      strikePending=true; startStrikeWipe();
      strikeDetected=false; strikeEdgeLatched=false; strikeLightOn=false;

      stepIndex=30; prevStepMillis=millis(); return;
    }
    SweepGuard();
  }
  else if(stepIndex==2){
    if(refillInProgress()){
      backgroundRefillRequested=true;
      if(loadedCount==9 && deckIsUp) startTurretReleaseCycle();
      prevStepMillis=millis(); return;
    }
    DeckPinGrab();
  }
  else if(stepIndex==3){ ScissorsGrab(); }
  else if(stepIndex==4){ DeckUp(); }
  else if(stepIndex==5){ SweepBack(); onSweepBackDoorHoldStart(); }
  else if(stepIndex==6){ SweepGuard(); }
  else if(stepIndex==7){ scoreWindowActive=false; DeckPinDrop(); }
  else if(stepIndex==8){ ScissorsDrop(); }
  else if(stepIndex==9){ DeckUp(); }
  else if(stepIndex==10){
    throwCount=2; SweepUp(); waitingForBall=true; 
    updateFrameLEDs();
    stepIndex=0; prevStepMillis=millis(); return;
  }

  // ---------- Throw #2 ----------
  else if(stepIndex==22){ SweepGuard(); }
  else if(stepIndex==23){ SweepBack(); onSweepBackDoorHoldStart(); }
  else if(stepIndex==24){
    scoreWindowActive=false; 
    SweepGuard();
    stepIndex=30; prevStepMillis=millis(); return;
  }

  // ---------- Reset / Set ----------
  else if(stepIndex==30){
    if(deckConeCount==10){ stepIndex=32; prevStepMillis=millis(); return; }
    startTurretReleaseCycle();
    stepIndex=31; prevStepMillis=millis(); return;
  }
  else if(stepIndex==31){
    if(!dropCycleJustFinished){ prevStepMillis=millis(); return; }
    deckHasTenReady=true;
  }
  else if(stepIndex==32){ DeckPinSet(); }
  else if(stepIndex==33){
    SlidingDeckRelease();
    deckHasTenReady=false; deckConeCount=0;
  }
  else if(stepIndex==34){
    DeckUp();

    postSetResumeDelayActive = true;
    postSetResumeStart = millis();

    refillLockActive=true;
    backgroundRefillRequested=true;
    SweepUp();
  }
  else if(stepIndex==35){
    SlidingDeckHome();
    pumpAll(400);
  }
  else if(stepIndex==36){
    forceConveyorForBallReturn=false;
    dropCycleJustFinished=false;
    deckHasTenReady=false;
    strikeDetected=false; 
    strikePending=false;

    if (fillBallShotInProgress) {
      autoResetFillBall = false;
      inFillBall = false;
      autoResetEdgeLatched = false;
      fillBallShotInProgress = false;
    }

    throwCount=1; waitingForBall=true;
    updateFrameLEDs();
    stepIndex=0; prevStepMillis=millis(); return;
  }

  prevStepMillis=millis();
  stepIndex++;
}

unsigned long stepDuration(int idx){
  if(idx==1)  return 3000;
  if(idx==22) return 3000;
  if(idx==31) return 0;
  return 1000;
}

// ======================= DECK FUNCTIONS =======================
void DeckUp() {
  LeftRaiseServo.write(RAISE_UP_ANGLE);
  RightRaiseServo.write(180 - RAISE_UP_ANGLE);
  pumpAll(DECK_EXTRA_SETTLE_MS);
  deckIsUp = true;  //don't report deck as up until we're sure it's all the way up
}

void DeckPinSet() {
  LeftRaiseServo.write(RAISE_DOWN_ANGLE);
  RightRaiseServo.write(180 - RAISE_DOWN_ANGLE);
  deckIsUp = false;  //report deck as not up right away
  pumpAll(DECK_EXTRA_SETTLE_MS);
}

void DeckPinGrab() {
  LeftRaiseServo.write(RAISE_GRAB_ANGLE);
  RightRaiseServo.write(180 - RAISE_GRAB_ANGLE);
  deckIsUp = false;    //report deck is not up right away
  pumpAll(DECK_EXTRA_SETTLE_MS);
}

void DeckPinDrop() {
  LeftRaiseServo.write(RAISE_DROP_ANGLE);
  RightRaiseServo.write(180 - RAISE_DROP_ANGLE);
  deckIsUp = false;    //report deck is not up right away
  pumpAll(DECK_EXTRA_SETTLE_MS);
}

void SlidingDeckRelease()  { SlideServo.write(SLIDER_RELEASE_ANGLE); }
void SlidingDeckHome()     { SlideServo.write(SLIDER_HOME_ANGLE); }
void ScissorsGrab()        { ScissorsServo.write(SCISSOR_GRAB_ANGLE); }
void ScissorsDrop()        { ScissorsServo.write(SCISSOR_DROP_ANGLE); }
void BallReturnClosed()    { BallReturnServo.write(BALL_DOOR_CLOSED_ANGLE); }
void BallReturnOpen()      { BallReturnServo.write(BALL_DOOR_OPEN_ANGLE); }

// NOTE: set sweepPoseTarget so pause mode can tell where we are
void SweepGuard(){ sweepPoseTarget = SWEEP_GUARD; startSweepTo(SWEEP_GUARD_ANGLE, 180-SWEEP_GUARD_ANGLE, SWEEP_TWEEN_MS); }
void SweepUp()   { sweepPoseTarget = SWEEP_UP;    startSweepTo(SWEEP_UP_ANGLE, 180-SWEEP_UP_ANGLE, SWEEP_TWEEN_MS); }
void SweepBack() { sweepPoseTarget = SWEEP_BACK;  startSweepTo(SWEEP_BACK_ANGLE, 180-SWEEP_BACK_ANGLE, SWEEP_TWEEN_MS); }

// ======================= STRIKE SWEEP HELPER =======================
void StrikeSweepClearLane(){
  SweepGuard(); waitSweepDone(); pumpAll(STRIKE_SWEEP_PAUSE_MS);
  SweepBack();  waitSweepDone(); onSweepBackDoorHoldStart(); pumpAll(STRIKE_SWEEP_PAUSE_MS);
  SweepGuard(); waitSweepDone(); pumpAll(STRIKE_SWEEP_PAUSE_MS);
}

// ======================= POWER-ON DEMO =======================
void PowerOnSequence(){
  SweepGuard(); waitSweepDone(); pumpAll(500);
  SweepBack();  waitSweepDone(); pumpAll(500);
  SweepGuard(); waitSweepDone(); pumpAll(500);

  DeckPinSet();          pumpAll(800);
  SlideServo.attach(SLIDE_PIN);
  SlidingDeckRelease();  pumpAll(800);
  DeckUp();              pumpAll(800);
  SlidingDeckHome();     pumpAll(800);

  SweepBack(); waitSweepDone(); pumpAll(400);
  SweepGuard(); waitSweepDone(); pumpAll(400);

  EmptyTurret(); pumpAll(600);

  DeckPinSet();          pumpAll(800);
  SlidingDeckRelease();  pumpAll(800);
  DeckUp();              pumpAll(800);
  SlidingDeckHome();     pumpAll(800);

  SweepBack(); waitSweepDone(); pumpAll(400);
  SweepGuard(); waitSweepDone(); pumpAll(400);
}

// ======================= PRIME (boot) =======================
void PrimeFullRackAndSetLaneOnce(){
  releaseDwellActive=false; turretReleaseRequested=false;
  while(loadedCount<9){
    runTurret(); updateConveyorOutput(); updateBallReturnDoor(); updateSweepTween(); laneUpdate();
    if(millis()-prevScoreMillis>=SCORE_INTERVAL){
      prevScoreMillis=millis(); checkSerial(); checkInputChanges();
    }
    delay(1);
  }

  startTurretReleaseCycle();
  while(!dropCycleJustFinished){
    runTurret(); updateConveyorOutput(); updateBallReturnDoor(); updateSweepTween(); laneUpdate();
    if(millis()-prevScoreMillis>=SCORE_INTERVAL){
      prevScoreMillis=millis(); checkSerial(); checkInputChanges();
    }
    delay(1);
  }
  pumpAll(200);

  DeckPinSet();         pumpAll(800);
  SlidingDeckRelease(); pumpAll(800);
  deckConeCount=0; deckHasTenReady=false;

  DeckUp();             pumpAll(600);
  SlidingDeckHome();    pumpAll(400);

  dropCycleJustFinished=false;

  BallReturnOpen(); brState=BR_IDLE_OPEN; brStateStart=millis();

  refillLockActive=true;
  backgroundRefillRequested=true;
}

// ======================= TURRET FSM =======================
void runTurret(){
  // Background refill priority
  if(backgroundRefillRequested){
    if(loadedCount<9){
      turretFillTo9Requested=true;
    }else{
      turretFillTo9Requested=false;
      if(!releaseDwellActive && !turretReleaseRequested && deckIsUp){
        turretReleaseRequested=true;
      }
    }
  }

  // Cones-full hold
  if(conesFullHoldArmed && (millis()-conesFullHoldStartMs)>=CATCH_DELAY_MS){
    conesFullHoldArmed=false;
    conesFullHold=true;
    turretFillTo9Requested=false;
    turretReleaseRequested=false;
  }

  if(conesFullHold){
    if(deckIsUp && deckConeCount<10 && !postSetResumeDelayActive){
      conesFullHold=false;
      backgroundRefillRequested=true;
      if(loadedCount==9 && !releaseDwellActive){
        turretReleaseRequested=true;
      }
    }
  }

  stepper1.run();
  if(moving && stepper1.distanceToGo()==0) moving=false;

  // Special: EmptyTurret non-blocking return-to-hall
  if (emptyTurretReturnActive) {
    if (digitalRead(HALL_EFFECT_PIN) == LOW || stepper1.distanceToGo() == 0) {
      stepper1.stop();
      emptyTurretReturnActive = false;
    }
  }

  runHomingFSM();

  // Start dwell upon arrival at release
  if(!moving && turretReleaseRequested && !releaseDwellActive && (targetPos==Pin10ReleasePos())){
    releaseDwellActive=true; releaseDwellStart=millis();
    conveyorLockedByDwell=true;
  }

  // IR debounce
  int raw=digitalRead(IR_SENSOR_PIN);
  if(raw!=irLastRead){ irLastChange=millis(); irLastRead=raw; }
  if((millis()-irLastChange)>DEBOUNCE_MS){
    if(irStableState!=raw) irStableState=raw;
  }
  if(irStableState==HIGH) pinEdgeArmed=true;

  // Queue pin hits whenever we're not in dwell/hold AND not paused (NEW)
  if(!pauseMode && !releaseDwellActive && !conesFullHold){
    if(pinEdgeArmed && irStableState==LOW){
      queuedPinEvents++;         // record that one more pin has arrived
      pinEdgeArmed = false;      // wait for beam to clear before next
    }
  }

  // Catch delay for 1..8
  if(catchDelayActive && (millis()-catchDelayStart>=CATCH_DELAY_MS)){
    catchDelayActive=false;
    if(NowCatching>=1 && NowCatching<=8){
      NowCatching++; if(NowCatching>9) NowCatching=9;
      goTo(PinPositions[NowCatching]);
    }
  }

  // 9th settle
  if(ninthSettleActive && (millis()-ninthSettleStart>=NINTH_SETTLE_MS)){
    ninthSettleActive=false;
  }

  // Finish dwell (10th dropped)
  if(releaseDwellActive && (millis()-releaseDwellStart>=RELEASE_DWELL_MS)){
    releaseDwellActive=false;

    loadedCount=0; NowCatching=1;
    startHomeTurret(); postDropHomePending=true;

    dropCycleJustFinished=true;

    deckConeCount=10; deckHasTenReady=true;

    if(backgroundRefillRequested) backgroundRefillRequested=false;

    conveyorLockedByDwell=false; suspendConveyorUntilHomeDone=true;
  }

  // After all other logic, see if it's safe to handle one queued pin
  servicePinQueue();
}

// Process queued pin events one at a time when safe
void servicePinQueue(){
  // Nothing queued?
  if (queuedPinEvents <= 0) return;

  // Don't start a new catch while in dwell, hold, or homing
  if (releaseDwellActive || conesFullHold || homingActive) return;

  // Only handle a new pin when turret is idle and not in catch delay
  if (moving || catchDelayActive) return;

  // Safe: handle exactly ONE queued pin now
  queuedPinEvents--;
  onPinDetected();
}

void onPinDetected(){
  lastPinCatchMs = millis();

  if(loadedCount<9){
    loadedCount++;
    if(NowCatching>=1 && NowCatching<=8){
      catchDelayActive=true; catchDelayStart=millis();
    }else{
      ninthSettleActive=true; ninthSettleStart=millis();
      if(deckConeCount==10){
        conesFullHoldArmed=true;
        conesFullHoldStartMs=millis();
      }
      pinEdgeArmed=false;
    }
    return;
  }

  if(loadedCount==9){
    if(deckConeCount==10){
      pinEdgeArmed=false;
      return;
    }
    if( (turretReleaseRequested || (deckIsUp && backgroundRefillRequested)) ){
      turretReleaseRequested=true; goTo(Pin10ReleasePos()); return;
    }
    pinEdgeArmed=false; return;
  }
}

// ---------- goTo with per-move speed profiles ----------
void goTo(long pos){
  if(targetPos!=pos){
    targetPos=pos;

    if(!homingActive){
      if(pos==Pin10ReleasePos()){
        stepper1.setMaxSpeed(TURRET_SPRING_MAXSPEED);
        stepper1.setAcceleration(TURRET_SPRING_ACCEL);
      }else{
        stepper1.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
        stepper1.setAcceleration(TURRET_NORMAL_ACCEL);
      }
    }

    stepper1.moveTo(targetPos);
    moving=true;
  }
}

// ---------- Non-blocking homing ----------
void startHomeTurret(){
  homingActive=true; homingPhase=HOME_PREP_MOVE_TO_SLOT1;
  ConveyorOff();
  stepper1.setAcceleration(3000); stepper1.setMaxSpeed(500);
  goTo(PinPositions[1]);
}

void runHomingFSM(){
  if(!homingActive) return;
  switch(homingPhase){
    case HOME_PREP_MOVE_TO_SLOT1:
      if(stepper1.distanceToGo()==0){ homingPhase=HOME_ADVANCE_TO_SWITCH; }
      break;
    case HOME_ADVANCE_TO_SWITCH:{
      if(digitalRead(HALL_EFFECT_PIN)==HIGH){
        if(stepper1.distanceToGo()==0){ goTo(stepper1.currentPosition()+10); }
      }else{
        homingPhase=HOME_BACKOFF; goTo(stepper1.currentPosition()-150);
      }
    }break;
    case HOME_BACKOFF:
      if(stepper1.distanceToGo()==0){
        stepper1.setMaxSpeed(100); homingPhase=HOME_CREEP_TO_SWITCH;
      }
      break;
    case HOME_CREEP_TO_SWITCH:{
      if(digitalRead(HALL_EFFECT_PIN)==HIGH){
        if(stepper1.distanceToGo()==0){ goTo(stepper1.currentPosition()+2); }
      }else{
        stepper1.setCurrentPosition(TURRET_HOME_ADJUSTER);
        stepper1.setMaxSpeed(500); stepper1.setAcceleration(3000);
        goTo(PinPositions[1]); homingPhase=HOME_SETZERO_AND_MOVE_SLOT1;
      }
    }break;
    case HOME_SETZERO_AND_MOVE_SLOT1:
      if(stepper1.distanceToGo()==0){ homingPhase=HOME_DONE; }
      break;
    case HOME_DONE: {
      homingActive=false;
      homingPhase=HOME_IDLE;

      stepper1.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
      stepper1.setAcceleration(TURRET_NORMAL_ACCEL);

      if(postDropHomePending){
        turretReleaseRequested=false;
        postDropHomePending=false;
      }

      // Sync IR debounce state to current sensor and avoid fake "new pin"
      int irNow = digitalRead(IR_SENSOR_PIN);
      irLastRead    = irNow;
      irStableState = irNow;
      if(irNow == LOW){
        pinEdgeArmed = false;   // beam currently blocked → don't arm yet
      }

      suspendConveyorUntilHomeDone=false;
    } break;
    default: break;
  }
}

// ======================= CONVEYORS =======================
void startTurretReleaseCycle(){ dropCycleJustFinished=false; turretReleaseRequested=true; }

void updateConveyorOutput(){
  if(postSetResumeDelayActive){
    if(millis() - postSetResumeStart < RESUME_AFTER_DECKUP_MS){
      ConveyorOff();
      return;
    }else{
      postSetResumeDelayActive = false;
    }
  }

  if(conveyorLockedByDwell){
    if(millis()-releaseDwellStart<RELEASE_FEED_ASSIST_MS) ConveyorOn();  else ConveyorOff();
    return;
  }
  if(suspendConveyorUntilHomeDone){ ConveyorOff(); return; }

  if(conesFullHold){ ConveyorOff(); return; }

  bool need=false;

  if(!homingActive){
    need =
      (turretFillTo9Requested && (loadedCount<9)) ||
      (ninthSettleActive) ||
      ((loadedCount==9) && deckIsUp && !releaseDwellActive &&
       (turretReleaseRequested || backgroundRefillRequested));

    if(releaseDwellActive && (millis()-releaseDwellStart<RELEASE_FEED_ASSIST_MS)) need=true;
    if(forceConveyorForBallReturn) need=true;

    if(conesFullHoldArmed) need = true;

    bool idleFrameReady = (waitingForBall && deckIsUp && deckConeCount==10);
    if(idleFrameReady) need = true;

    bool pinsOnLane     = waitingForBall;
    bool deckFull       = (deckConeCount == 10);
    bool turretHas0to9  = (loadedCount >= 0 && loadedCount <= 9);

    if(pinsOnLane && deckFull && turretHas0to9){
      if(millis() - lastPinCatchMs >= NO_CATCH_TIMEOUT_MS){
        need = false;
      }
    }
  }
  if(need) ConveyorOn(); else ConveyorOff();
}

void ConveyorOn(){  digitalWrite(MOTOR_RELAY_PIN, CONVEYOR_ACTIVE_HIGH?HIGH:LOW); conveyorIsOn=true; }
void ConveyorOff(){ digitalWrite(MOTOR_RELAY_PIN, CONVEYOR_ACTIVE_HIGH?LOW :HIGH); conveyorIsOn=false;}



// ======================= BALL RETURN DOOR CONTROL =======================
void onBallThrownDoorClose(){
  BallReturnClosed(); brState=BR_CLOSED_WAIT_SWEEPBACK; brStateStart=millis();
}
void onSweepBackDoorHoldStart(){
  if(brState==BR_CLOSED_WAIT_SWEEPBACK || brState==BR_IDLE_OPEN || brState==BR_IDLE_CLOSED){
    BallReturnClosed(); brState=BR_CLOSED_HOLD_AFTER_SWEEP; brStateStart=millis();
  }
}
void updateBallReturnDoor(){
  unsigned long now=millis();
  switch(brState){
    case BR_IDLE_OPEN: break;
    case BR_IDLE_CLOSED: break;
    case BR_CLOSED_WAIT_SWEEPBACK: break;
    case BR_CLOSED_HOLD_AFTER_SWEEP:
      if(now - brStateStart >= BR_CLOSE_AFTER_SWEEPBACK_MS){
        BallReturnOpen(); brState=BR_IDLE_OPEN; brStateStart=now;
      }
      break;
  }
}

// ======================= I/O / UTIL =======================
void startStrikeCycle(){ strikeDetected=true; strikePending=true; }

void checkSerial(){
  static String input="";
  while(Serial.available()>0){
    char c=Serial.read(); if(c=='\r') continue;
    if(c=='\n'){
      input.trim(); if(input.length()>0) handleCommand(input);
      input="";
    } else { input+=c; }
  }
}

#define SPTR_SIZE 20
char *strData = NULL;
char *sPtr[SPTR_SIZE];
size_t numberOfStr = 0;

void freeData(char **pdata){
  free(*pdata);
  *pdata=NULL;
  numberOfStr=0;
}

int separate (String &str, char **p, int size, char** pdata, char separator){
  int n=0;
  free(*pdata);
  *pdata = strdup(str.c_str());
  if(*pdata == NULL){
    Serial.println("DEBUG:separate function OUT OF MEMORY");
    return 0;
  }
  *p++ = strtok(*pdata,&separator);
  for(n=1;NULL!=(*p++=strtok(NULL,&separator)); n++){
    if (size == n) {
      break;
    }
  }
  return n;
}  

void handleCommand(String cmd){
  cmd.trim();
  if(cmd.startsWith("SET_INPUT:")){
    int scoreMorePin=cmd.substring(10).toInt();
    int pin=resolveArduinoPin(scoreMorePin);
    if(pin!=-1) {
      if(!isTrackedInput(pin) && inputCount<maxPins){
        if(isBowlingPin(scoreMorePin)) pinMode(pin, INPUT_PULLUP); else pinMode(pin, INPUT_PULLUP);
        inputPins[inputCount]=pin; pinStates[inputCount]=digitalRead(pin); inputCount++;
        Serial.print("ACK_SET_INPUT:"); Serial.println(scoreMorePin);
      } else {
        Serial.print("ACK_INPUT_ALREADY_SET:"); Serial.println(scoreMorePin);
      }
    } else {
        Serial.print("ACK_SET_INPUT_INVALID_PIN:"); Serial.println(scoreMorePin);
    }
  } else if(cmd.startsWith("SET_OUTPUT:")){
    int scoreMorePin=cmd.substring(11).toInt();
    int pin=resolveArduinoPin(scoreMorePin);
    if(pin!=-1){
      pinMode(pin, OUTPUT); removeInputPin(pin);
      Serial.print("ACK_SET_OUTPUT:"); Serial.println(scoreMorePin);
    } else {
      Serial.print("ACK_SET_OUTPUT_INVALID_PIN:"); Serial.println(scoreMorePin);
    }
  } else if(cmd.startsWith("WRITE:")){
    int firstColon=cmd.indexOf(':'), secondColon=cmd.indexOf(':', firstColon+1);
    if(firstColon>0 && secondColon>firstColon){
      int scoreMorePin=cmd.substring(firstColon+1, secondColon).toInt();
      int value=cmd.substring(secondColon+1).toInt();
      int pin=resolveArduinoPin(scoreMorePin);
      if(pin!=-1){
        digitalWrite(pin, value);
        // Strike (ScoreMore logical pin 10)
        if(scoreMorePin==SM_STRIKE_LIGHT){
          bool prev=strikeLightOn;
          strikeLightOn=(value!=0);
          if(!prev && strikeLightOn && !strikeEdgeLatched){
            startStrikeCycle(); strikeEdgeLatched=true;
          }
          if(prev && !strikeLightOn){ strikeEdgeLatched=false; }
        }
        // Fill-ball grant (ScoreMore logical pin 7)
        else if (scoreMorePin==SM_AUTO_RESET){
          bool prev = autoResetFillBall;
          bool isHigh = (value != 0);
          if(!prev && isHigh && !autoResetEdgeLatched){
            autoResetFillBall = true;
            autoResetEdgeLatched = true;
            inFillBall = true;
            for(int i=0;i<LANE_LED_LENGTH_L;i++) laneL.setPixelColor(i, C_GREEN(laneL));
            for(int i=0;i<LANE_LED_LENGTH_R;i++) laneR.setPixelColor(i, C_GREEN(laneR));
            laneShowOnly();
            Serial.println("AUTO_RESET_FILL_BALL_ARMED (GREEN ON)");
          }
          if(prev && !isHigh){
            autoResetEdgeLatched = false;
          }
        }
        Serial.print("ACK_WRITE:"); Serial.print(scoreMorePin); Serial.print(":"); Serial.println(value);
      } else {
        Serial.print("ACK_WRITE_INVALID_PIN:");Serial.println(scoreMorePin);
      } 
    }
  } else if(cmd=="RESET"){
    int strikePin=resolveArduinoPin(10);
    if(strikePin!=-1) digitalWrite(strikePin, LOW);
    strikeLightOn=false; strikeEdgeLatched=false; strikeDetected=false; strikePending=false;
    Serial.println("ACK_RESET");
  } else if(cmd=="CHECK_READY"){
    Serial.println("READY");
  } else if(cmd=="VERSION"){
    Serial.println(VERSION);
  } else if(cmd.startsWith("PINSETTER:")){
    char s[100];
    int N=separate(cmd, sPtr, SPTR_SIZE,&strData, ':');
    Serial.print("N=");Serial.println(N);
    if(N>1){
      Serial.println(sPtr[1]);
      if (strcmp(sPtr[1],"RESET")==0) {
        pinsetterResetRequested=true;
        Serial.println("ACK_PINSETTER_RESET");
      } else {
        Serial.println("ACK_UNKNOWN_PINSETTER_COMMAND");
      }
    } else {
      Serial.println("ACK_NO_PINSETTER_COMMAND_GIVEN");
    }
    freeData(&strData);
  } else {
    Serial.println("ACK_UNKNOWN_COMMAND");Serial.print("DEBUG: unknow command:");Serial.println(cmd);
  }
}

void checkInputChanges(){
  for(int i=0;i<inputCount;i++){
    int currentState=digitalRead(inputPins[i]);
    if(currentState!=pinStates[i]){
      pinStates[i]=currentState;
      int scoreMorePin=getScoreMorePin(inputPins[i]);
      if(scoreMorePin!=-1){
        Serial.print("INPUT_CHANGE:"); Serial.print(scoreMorePin); Serial.print(":"); Serial.println(currentState);
      }   
    }
  }
}

bool isTrackedInput(int pin){
  for(int i=0;i<inputCount;i++) if(inputPins[i]==pin) return true;
  return false;
}
void removeInputPin(int pin){
  for(int i=0;i<inputCount;i++) if(inputPins[i]==pin){
    for(int j=i;j<inputCount-1;j++){
      inputPins[j]=inputPins[j+1]; pinStates[j]=pinStates[j+1];
    }
    inputCount--; break;
  }
}
int resolveArduinoPin(int scoreMorePin){
  for(int i=0;i<maxPins;i++) if(pinMap[i].scoreMore==scoreMorePin) return pinMap[i].arduino;
  return -1;
}
int getScoreMorePin(int arduinoPin){
  for(int i=0;i<maxPins;i++) if(pinMap[i].arduino==arduinoPin) return pinMap[i].scoreMore;
  return -1;
}
bool isBowlingPin(int scoreMorePin){
  for(int i=0;i<maxPins;i++) if(pinMap[i].scoreMore==scoreMorePin) return pinMap[i].isBowling;
  return false;
}

// ======================= SWEEP TWEEN =======================
static int clampInt(int v,int lo,int hi){
  return (v<lo)?lo:(v>hi)?hi:v;
}

void setSweepInstant(int leftDeg,int rightDeg){
  leftDeg=clampInt(leftDeg,0,180); rightDeg=clampInt(rightDeg,0,180);
  sweepCurL=sweepStartL=sweepTargetL=leftDeg;
  sweepCurR=sweepStartR=sweepTargetR=rightDeg;
  sweepAnimating=false; LeftSweepServo.write(leftDeg); RightSweepServo.write(rightDeg);
  sweepPoseCur = sweepPoseTarget; // NEW
}

void startSweepTo(int leftDeg,int rightDeg,unsigned long durMs){
  leftDeg=clampInt(leftDeg,0,180); rightDeg=clampInt(rightDeg,0,180);
  sweepStartL=sweepCurL; sweepStartR=sweepCurR; sweepTargetL=leftDeg; sweepTargetR=rightDeg;
  sweepStartMs=millis(); sweepDurationMs=(durMs==0)?1UL:durMs;
  sweepAnimating=!((sweepStartL==sweepTargetL)&&(sweepStartR==sweepTargetR));
  if(!sweepAnimating){
    LeftSweepServo.write(sweepTargetL); RightSweepServo.write(sweepTargetR);
    sweepPoseCur = sweepPoseTarget; // NEW
  }
}

void updateSweepTween(){
  if(!sweepAnimating) return;
  unsigned long now=millis(), elapsed=now - sweepStartMs;
  if(elapsed>=sweepDurationMs){
    sweepCurL=sweepTargetL; sweepCurR=sweepTargetR;
    LeftSweepServo.write(sweepCurL); RightSweepServo.write(sweepCurR);
    sweepAnimating=false;
    sweepPoseCur = sweepPoseTarget; // NEW
    return;
  }
  float t=(float)elapsed/(float)sweepDurationMs;
  int newL = sweepStartL + (int)((sweepTargetL - sweepStartL)*t + (t>=0?0.5f:-0.5f));
  int newR = sweepStartR + (int)((sweepTargetR - sweepStartR)*t + (t>=0?0.5f:-0.5f));
  if(newL!=sweepCurL){ sweepCurL=newL; LeftSweepServo.write(sweepCurL); }
  if(newR!=sweepCurR){ sweepCurR=newR; RightSweepServo.write(sweepCurR); }
}

void waitSweepDone(unsigned long timeoutMs){
  unsigned long start=millis();
  while(sweepAnimating && (millis()-start)<timeoutMs){
    updateSweepTween(); delay(1);
  }
  if(sweepAnimating){
    sweepCurL=sweepTargetL; sweepCurR=sweepTargetR;
    LeftSweepServo.write(sweepCurL); RightSweepServo.write(sweepCurR);
    sweepAnimating=false;
    sweepPoseCur = sweepPoseTarget; // NEW
  }
}

// ======================= PUMP =======================
void pumpAll(unsigned long ms){
  unsigned long t0=millis();
  while(millis()-t0<ms){
    runTurret(); updateConveyorOutput(); updateBallReturnDoor(); updateSweepTween(); laneUpdate();
    if(millis()-prevScoreMillis>=SCORE_INTERVAL){
      prevScoreMillis=millis(); checkSerial(); checkInputChanges();
    }
    delay(1);
  }
}

// ======================= EmptyTurret (boot-only) =======================
//  1) Fast slam into hall sensor.
//  2) Fast move to pin 9.
//  3) Slow move to purge release (Pin10ReleasePos + TURRET_EMPTY_EXTRA_OFFSET).
//  4) Dwell so pins fully dump.
//  5) Start non-blocking move back toward hall; runTurret() will stop on HALL_EFFECT_PIN.
void EmptyTurret() {
  homingActive = false;
  homingPhase  = HOME_IDLE;
  moving       = false;
  emptyTurretReturnActive = false;

  // 1) Rough home to magnet
  stepper1.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
  stepper1.setAcceleration(TURRET_NORMAL_ACCEL);

  long startPos  = stepper1.currentPosition();
  long farTarget = startPos + 5000;

  stepper1.moveTo(farTarget);
  while (digitalRead(HALL_EFFECT_PIN) == HIGH && stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  stepper1.stop();
  while (stepper1.isRunning()) {
    stepper1.run();
  }

  stepper1.setCurrentPosition(TURRET_HOME_ADJUSTER);

  // 2) Fast move to pin 9
  goTo(PinPositions[9]);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  // 3) Slow spring-safe move to purge release
  long purgeReleasePos = Pin10ReleasePos() + TURRET_EMPTY_EXTRA_OFFSET;

  stepper1.setMaxSpeed(TURRET_SPRING_MAXSPEED);
  stepper1.setAcceleration(TURRET_SPRING_ACCEL);
  stepper1.moveTo(purgeReleasePos);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  // 4) Dwell at release
  pumpAll(RELEASE_DWELL_MS);

  // 5) Non-blocking move back toward hall
  stepper1.setMaxSpeed(TURRET_NORMAL_MAXSPEED);
  stepper1.setAcceleration(TURRET_NORMAL_ACCEL);

  long backTarget = stepper1.currentPosition() + 5000;
  stepper1.moveTo(backTarget);
  moving = true;
  emptyTurretReturnActive = true;
}

// ======================= LED IMPL =======================
void ledsBegin(){
  deckL.begin(); deckR.begin(); laneL.begin(); laneR.begin();
  deckL.setBrightness(DECK_LED_BRIGHTNESS);
  deckR.setBrightness(DECK_LED_BRIGHTNESS);
  laneL.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneR.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneMode=LANE_IDLE_WHITE;
}

void deckAll(uint32_t col){
  for(int i=0;i<DECK_LED_LENGTH_L;i++) deckL.setPixelColor(i,col);
  for(int i=0;i<DECK_LED_LENGTH_R;i++) deckR.setPixelColor(i,col);
}
void laneAll(uint32_t col){
  for(int i=0;i<LANE_LED_LENGTH_L;i++) laneL.setPixelColor(i,col);
  for(int i=0;i<LANE_LED_LENGTH_R;i++) laneR.setPixelColor(i,col);
}
void ledsShowAll(){ deckL.show(); deckR.show(); laneL.show(); laneR.show(); }
void laneShowOnly(){ laneL.show(); laneR.show(); }

// ---- STRIKE EFFECTS ----
void startStrikeWipe(){
  if(scoreWindowActive) return;
  lanePauseArmed=true;
  laneL.setBrightness(LED_BRIGHTNESS_STRIKE);
  laneR.setBrightness(LED_BRIGHTNESS_STRIKE);
  strikeWipeStartMs=millis(); strikeLastFrameMs=0; laneMode=LANE_STRIKE_WIPE;
}

void startStrikeFlash(){
  if(scoreWindowActive) return;
  flashOnPhase=true; flashCycles=0; flashLastMs=millis(); laneMode=LANE_STRIKE_FLASH;
  uint32_t redL=C_RED(laneL), redR=C_RED(laneR);
  for(int i=0;i<LANE_LED_LENGTH_L;i++) laneL.setPixelColor(i,redL);
  for(int i=0;i<LANE_LED_LENGTH_R;i++) laneR.setPixelColor(i,redR);
  laneShowOnly();
}

void startBallCometImmediate(){
  lanePauseArmed=true;
  laneAll(C_OFF(laneL)); laneShowOnly();
  ballCometStartMs=millis(); ballCometLastFrame=0; laneMode=LANE_BALL_COMET;
}

void endAllLaneAnimsToWhite(){
  laneAll(C_WHITE(laneL)); laneShowOnly(); laneMode=LANE_IDLE_WHITE;
  laneL.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneR.setBrightness(LED_BRIGHTNESS_NORMAL);
  lanePaused=false; lanePauseArmed=false;
}

void laneUpdate(){
  unsigned long now=millis();

  // STRIKE WIPE
  if(!scoreWindowActive && laneMode==LANE_STRIKE_WIPE){
    unsigned long elapsed=now - strikeWipeStartMs;
    if(elapsed>=STRIKE_WIPE_MS){
      startStrikeFlash();
    }else if(strikeLastFrameMs==0 || (now - strikeLastFrameMs)>=STRIKE_FRAME_MS){
      strikeLastFrameMs=now;
      float t=(float)elapsed/(float)STRIKE_WIPE_MS;
      int n1=(int)(t*LANE_LED_LENGTH_L+0.5f), n2=(int)(t*LANE_LED_LENGTH_R+0.5f);
      uint32_t redL=C_RED(laneL), redR=C_RED(laneR);
      for(int i=0;i<LANE_LED_LENGTH_L;i++)
        laneL.setPixelColor(i,(i<n1)?redL:C_WHITE(laneL));
      for(int i=0;i<LANE_LED_LENGTH_R;i++)
        laneR.setPixelColor(i,(i<n2)?redR:C_WHITE(laneR));
      laneShowOnly();
    }
  }

  // STRIKE FLASH
  if(!scoreWindowActive && laneMode==LANE_STRIKE_FLASH){
    if(flashOnPhase){
      if(now - flashLastMs >= FLASH_ON_MS){
        flashOnPhase=false; 
        flashLastMs=now;
        laneAll(C_WHITE(laneL)); 
        laneShowOnly();
      }
    }else{
      if(now - flashLastMs >= FLASH_OFF_MS){
        flashLastMs=now; 
        flashCycles++;
        if(flashCycles>=FLASH_COUNT){
          strikePending=false; 
          endAllLaneAnimsToWhite();
        }else{
          flashOnPhase=true; 
          uint32_t redL=C_RED(laneL), redR=C_RED(laneR);
          for(int i=0;i<LANE_LED_LENGTH_L;i++) laneL.setPixelColor(i,redL);
          for(int i=0;i<LANE_LED_LENGTH_R;i++) laneR.setPixelColor(i,redR);
          laneShowOnly();
        }
      }
    }
  }

  // BALL COMET
  if(laneMode==LANE_BALL_COMET){
    unsigned long elapsed=now - ballCometStartMs;
    if(elapsed>=BALL_COMET_MS){ 
      endAllLaneAnimsToWhite(); 
      return; 
    }
    if(now - ballCometLastFrame >= BALL_COMET_FRAME_MS){
      ballCometLastFrame=now;
      float t=(float)elapsed/(float)BALL_COMET_MS;
      int maxIndexL=LANE_LED_LENGTH_L+COMET_LEN, maxIndexR=LANE_LED_LENGTH_R+COMET_LEN;
      int headL=(int)(t*maxIndexL+0.5f), headR=(int)(t*maxIndexR+0.5f);

      for(int i=0;i<LANE_LED_LENGTH_L;i++) laneL.setPixelColor(i,C_OFF(laneL));
      for(int i=0;i<LANE_LED_LENGTH_R;i++) laneR.setPixelColor(i,C_OFF(laneR));

      for(int k=0;k<COMET_LEN;k++){
        int idxL=headL-k, idxR=headR-k;
        if(idxL>=0 && idxL<LANE_LED_LENGTH_L) laneL.setPixelColor(idxL,C_WHITE(laneL));
        if(idxR>=0 && idxR<LANE_LED_LENGTH_R) laneR.setPixelColor(idxR,C_WHITE(laneR));
      }
      laneShowOnly();
    }
  }
}

void startupWipeWhiteQuick(){
  deckAll(C_OFF(deckL)); laneAll(C_OFF(laneL)); ledsShowAll();
  int maxLen=LANE_LED_LENGTH_L;
  if(LANE_LED_LENGTH_R>maxLen) maxLen=LANE_LED_LENGTH_R;
  if(DECK_LED_LENGTH_L>maxLen) maxLen=DECK_LED_LENGTH_L;
  if(DECK_LED_LENGTH_R>maxLen) maxLen=DECK_LED_LENGTH_R;
  for(int i=0;i<maxLen;i++){
    if(i<DECK_LED_LENGTH_L) deckL.setPixelColor(i,C_WHITE(deckL));
    if(i<DECK_LED_LENGTH_R) deckR.setPixelColor(i,C_WHITE(deckR));
    if(i<LANE_LED_LENGTH_L) laneL.setPixelColor(i,C_WHITE(laneL));
    if(i<LANE_LED_LENGTH_R) laneR.setPixelColor(i,C_WHITE(laneR));
    ledsShowAll(); delay(STARTUP_WIPE_MS_PER_STEP);
  }
  frameLEDsFirstHalf();
}

// ======================= Frame LED helpers =======================
void updateFrameLEDs(){
  if (waitingForBall){
    if (throwCount == 1){
      digitalWrite(FRAME_LED1_PIN, HIGH);
      digitalWrite(FRAME_LED2_PIN, LOW);
    } else {
      digitalWrite(FRAME_LED1_PIN, HIGH);
      digitalWrite(FRAME_LED2_PIN, HIGH);
    }
  }
}

void frameLEDsFirstHalf(){
  digitalWrite(FRAME_LED1_PIN, HIGH);
  digitalWrite(FRAME_LED2_PIN, LOW);
}

// ======================= PAUSE MODE HELPERS (NEW) =======================
static inline bool isReadyIdleForPause(){
  if(!lightsShownAfterBoot) return false;
  if(stepIndex != 0) return false;
  if(!waitingForBall) return false;
  if(laneAnimActive()) return false;
  if(sweepAnimating) return false;
  if(sweepPoseCur != SWEEP_UP) return false;
  return true;
}

void setAllLightsRed(){
  // Force solid red everywhere (deck + lane)
  deckAll(C_RED(deckL));
  laneAll(C_RED(laneL));
  ledsShowAll();

  // Stop lane animations so laneUpdate() doesn't overwrite the solid red
  laneMode = LANE_IDLE_WHITE;
  lanePaused = false;
  lanePauseArmed = false;

  laneL.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneR.setBrightness(LED_BRIGHTNESS_NORMAL);
}

void setAllLightsWhite(){
  deckAll(C_WHITE(deckL));
  laneAll(C_WHITE(laneL));
  ledsShowAll();

  laneMode = LANE_IDLE_WHITE;
  lanePaused = false;
  lanePauseArmed = false;

  laneL.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneR.setBrightness(LED_BRIGHTNESS_NORMAL);
}

void enterPauseMode(){
  pauseMode = true;

  // Move sweep to guard and turn lights red
  SweepGuard();
  setAllLightsRed();

  // Optional: stop conveyors while paused (uncomment if desired)
  // ConveyorOff();
}

void exitPauseMode(){
  pauseMode = false;

  // Reset the idle timer so it doesn't instantly re-pause
  lastBallActivityMs = millis();

  // Resume: sweep back up + white lights
  SweepUp();
  setAllLightsWhite();
}
