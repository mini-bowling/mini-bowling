// VERSION 1.0 - original rev67 files by Danny Lum


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

// 
#define VERSION "1.1.0"

// =============== ScoreMore Serial ===============
#define SCOREMORE_BAUD 9600

// =============== LED CONFIG =====================
#define DECK_PIN_A   50 //LEFT
#define DECK_PIN_B   51 //RIGHT
#define LANE_PIN_A   52 //LEFT
#define LANE_PIN_B   53 //RIGHT

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

// --- LED mode ---
enum LaneAnimMode { LANE_IDLE_WHITE=0, LANE_STRIKE_WIPE, LANE_STRIKE_FLASH, LANE_BALL_COMET };
LaneAnimMode laneMode = LANE_IDLE_WHITE;
static inline bool laneAnimActive(){ return laneMode==LANE_BALL_COMET||laneMode==LANE_STRIKE_WIPE||laneMode==LANE_STRIKE_FLASH; }
bool lanePauseArmed=false, lanePaused=false;

bool scoreWindowActive=false, strikePending=false, lightsShownAfterBoot=false;

// --- Strike timings ---
const unsigned long STRIKE_WIPE_MS=300, FLASH_ON_MS=120, FLASH_OFF_MS=120;
const int           FLASH_COUNT=3;
const unsigned long STRIKE_FRAME_MS=15;

unsigned long strikeWipeStartMs=0, strikeLastFrameMs=0, flashLastMs=0;
bool flashOnPhase=false; int flashCycles=0;

const unsigned long STARTUP_WIPE_MS_PER_STEP=5;
const unsigned long STRIKE_SWEEP_PAUSE_MS=1000;

// ====== Ball "comet" ======
const unsigned long BALL_COMET_MS=500, BALL_COMET_FRAME_MS=15;
const int COMET_LEN=4;
unsigned long ballCometStartMs=0, ballCometLastFrame=0;

// >>> NEW: universal extra settle for slower deck motors <<<
const unsigned long DECK_EXTRA_SETTLE_MS = 500;

// ====== FRAME STATE LEDS (NEW) ======
#define FRAME_LED1 46   // First throw / first half indicator
#define FRAME_LED2 47   // Second throw / second half adds this LED

// Helpers
void ledsBegin(); void deckAll(uint32_t col); void laneAll(uint32_t col); void ledsShowAll(); void laneShowOnly();
void laneUpdate(); void startStrikeWipe(); void startStrikeFlash(); void startBallCometImmediate();
void endAllLaneAnimsToWhite(); void startupWipeWhiteQuick();

// NEW: frame LED helpers
void updateFrameLEDs();
void frameLEDsFirstHalf();

// =============== ScoreMore mapping =================
const int maxPins=18;
const int scoreMorePins[maxPins]={2,3,4,5,6,7,8,9,10,11,12,14,15,16,17,18,19,20};
const int arduinoPins [maxPins]={
  A2, //Pin 2
  A3, //Pin 3
  A4, //Pin 4
  A5, //Pin 5
  A0, //Trigger Sensor
  40, //Auto Reset Trigger
  A1, //Ball Speed Sensor
  42, //Spare / Strike Light
  43, //Strike Light
  46, //1st Ball Light
  47, //2nd Ball Light
  A1, //Pin 1
  A6, //Pin 6
  A7, //Pin 7
  A8, //Pin 8
  A9, //Pin 9
  A10,//Pin 10
  45  //clear and reset pins on lane
  }; 
const int bowlingPins[]={2,3,4,5,14,15,16,17,18,19};
const int bowlingPinCount=sizeof(bowlingPins)/sizeof(bowlingPins[0]);

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
#define BALL_SPEED_SENSOR_PIN A1
#define PINSETTER_RESET_PIN 45
#define CONVEYOR_ACTIVE_HIGH 1


//Angles of the servos (based on left servo when applicable)
#define SWEEP_BACK_ANGLE       0  // sweep servo angle for back position *HOME* (default 0)
#define SWEEP_GUARD_ANGLE     50  // sweep servo angle for guard position (default 50)
#define SWEEP_UP_ANGLE        85  // sweep servo angle for up position (default 85)
#define RAISE_UP_ANGLE       180  // raise servo angle for deck fully up  *HOME* (default 180)
#define RAISE_RELEASE_ANGLE   80  // raise servo angle for deck ready to release grabbed pins (default 80)
#define RAISE_GRAB_ANGLE      60  // raise servo angle for deck ready to grab pins before sweep (default 60)
#define RAISE_DOWN_ANGLE      20  // raise servo angle for setting pins (default 20)
#define SLIDE_FORWARD_ANGLE  180  // slide servo angle for forward position  *HOME* (default 180)
#define SLIDE_BACK_ANGLE     100  // slide servo angle for back position (default 100)
#define SCISSORS_OPEN_ANGLE   90  // scissors servo angle for open  *HOME* (default 90)
#define SCISSORS_CLOSED      140  // scissors servo angle for closed (default 140)
#define DOOR_CLOSED_ANGLE      0  // door servo angle for closed  *HOME* (default 0)
#define DOOR_OPEN_ANGLE      180  // door servo angle for open (default 180)

Servo LeftRaiseServo, RightRaiseServo, SlideServo, ScissorsServo, LeftSweepServo, RightSweepServo, BallReturnServo;

// ======================= PAUSE MODE (NEW) =======================
const unsigned long PAUSE_IDLE_MS = 300000; // 2 minutes
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
const unsigned long SCORE_INTERVAL=5;

// ---- ScoreMore ball trigger mirroring ----
const int SCOREMORE_BALL_LOGICAL_PIN = 6;
const unsigned long SCOREMORE_BALL_PULSE_MS = 150;
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
const unsigned long BALL_LOW_CONFIRM_US=1000, BALL_REARM_MS=300;
int  ballPrev=HIGH; bool ballPending=false, ballRearmed=true, waitingForBall=true;
unsigned long ballLowStartUs=0, lastBallHighMs=0;
int  throwCount=1;

// Strike light
bool strikeLightOn=false, strikeEdgeLatched=false, strikeDetected=false;

// ===== Fill-ball (ScoreMore logical pin 7) =====
bool autoResetFillBall = false;
bool autoResetEdgeLatched = false;
bool inFillBall = false;
bool fillBallShotInProgress = false;

// Inputs
int inputPins[maxPins], pinStates[maxPins], inputCount=0;

// ======================= TURRET / STEPPER =======================
AccelStepper stepper1(1,2,3);

// ---- Speed profiles ----
const float TURRET_NORMAL_MAXSPEED   = 650.0;
const float TURRET_NORMAL_ACCEL      = 3000.0;
const float TURRET_SPRING_MAXSPEED   = 300.0;
const float TURRET_SPRING_ACCEL      = 1500.0;

// Turret timing
#define DEBOUNCE_DELAY 50
const unsigned long CATCH_DELAY_MS=800, RELEASE_DWELL_MS=1000, RELEASE_FEED_ASSIST_MS=250, NINTH_SETTLE_MS=300;

bool ninthSettleActive=false, catchDelayActive=false, releaseDwellActive=false;
unsigned long ninthSettleStart=0, catchDelayStart=0, releaseDwellStart=0;

// Pin positions
const int PinPositions[]={0,-133,-267,-400,-667,-800,-933,-1200,-1333,-1467,-1600};
const int Pin10ReleaseOffset = -30;
static inline long Pin10ReleasePos(){ return PinPositions[10] + Pin10ReleaseOffset; }

// Extra-only-for-boot purge offset
const int EmptyTurretExtraOffset = -60;

//less negative means CCW
int HomeAdjuster=-63;
int NowCatching=1, loadedCount=0;

bool moving=false; long targetPos=0;
bool emptyTurretReturnActive = false;   // true only while EmptyTurret is returning to hall

// NEW: queued pin events while turret is busy
int queuedPinEvents = 0;

int irStableState=HIGH, irLastRead=HIGH; unsigned long irLastChange=0; bool pinEdgeArmed=true;

bool turretReleaseRequested=false, turretFillTo9Requested=false;
bool conveyorLockedByDwell=false, suspendConveyorUntilHomeDone=false;

// ---- Idle-stall timeout ----
const unsigned long NO_CATCH_TIMEOUT_MS = 30000;
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
const unsigned long BR_CLOSE_AFTER_SWEEPBACK_MS=5000;
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

// ---- Post-Set Resume Delay ----
const unsigned long RESUME_AFTER_DECKUP_MS = 2000;
bool postSetResumeDelayActive = false;
unsigned long postSetResumeStart = 0;

// ======================= SETUP =======================
void setup(){
  ledsBegin();
  pinMode(PINSETTER_RESET_PIN, INPUT_PULLUP);
  // Frame LEDs
  pinMode(FRAME_LED1, OUTPUT);
  pinMode(FRAME_LED2, OUTPUT);
  digitalWrite(FRAME_LED1, LOW);
  digitalWrite(FRAME_LED2, HIGH);  //indicate to the user that we're in setup

  Serial.begin(SCOREMORE_BAUD); delay(1000); Serial.println("READY");
  digitalWrite(FRAME_LED2, LOW);

  pinMode(BALL_SENSOR_PIN, INPUT_PULLUP);

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

  pinMode(MOTOR_RELAY, OUTPUT); ConveyorOff();
  pinMode(IR_SENSOR, INPUT);

  // ===== NEW: Initialize IR debounce & queue if beam starts blocked =====
  irLastRead    = digitalRead(IR_SENSOR);
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

  pinMode(HALL_EFFECT, INPUT_PULLUP);

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

  if(stepIndex>0) runSequence();
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
  RightRaiseServo.write(180-RAISE_DOWN_ANGLE);
  deckIsUp = false;  //report deck as not up right away
  pumpAll(DECK_EXTRA_SETTLE_MS);
}

void DeckPinGrab() {
  LeftRaiseServo.write(RAISE_GRAB_ANGLE);
  RightRaiseServo.write(180-RAISE_GRAB_ANGLE);
  deckIsUp = false;    //report deck is not up right away
  pumpAll(DECK_EXTRA_SETTLE_MS);
}

void DeckPinDrop() {
  LeftRaiseServo.write(RAISE_RELEASE_ANGLE);
  RightRaiseServo.write(180-RAISE_RELEASE_ANGLE);
  deckIsUp = false;    //report deck is not up right away
  pumpAll(DECK_EXTRA_SETTLE_MS);
}

void SlidingDeckRelease()  { SlideServo.write(SLIDE_BACK_ANGLE); }
void SlidingDeckHome()     { SlideServo.write(SLIDE_FORWARD_ANGLE); }
void ScissorsGrab()        { ScissorsServo.write(SCISSORS_CLOSED); }
void ScissorsDrop()        { ScissorsServo.write(SCISSORS_OPEN_ANGLE); }
void BallReturnClosed()    { BallReturnServo.write(DOOR_CLOSED_ANGLE); }
void BallReturnOpen()      { BallReturnServo.write(DOOR_OPEN_ANGLE); }

// NOTE: set sweepPoseTarget so pause mode can tell where we are
void SweepGuard(){ sweepPoseTarget = SWEEP_GUARD; startSweepTo(SWEEP_GUARD_ANGLE, 180-SWEEP_GUARD_ANGLE, 500); }
void SweepUp()   { sweepPoseTarget = SWEEP_UP;    startSweepTo(SWEEP_UP_ANGLE, 180-SWEEP_UP_ANGLE,500); }
void SweepBack() { sweepPoseTarget = SWEEP_BACK;  startSweepTo(0,180,500); }

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
    if (digitalRead(HALL_EFFECT) == LOW || stepper1.distanceToGo() == 0) {
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
  int raw=digitalRead(IR_SENSOR);
  if(raw!=irLastRead){ irLastChange=millis(); irLastRead=raw; }
  if((millis()-irLastChange)>DEBOUNCE_DELAY){
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
      if(digitalRead(HALL_EFFECT)==HIGH){
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
      if(digitalRead(HALL_EFFECT)==HIGH){
        if(stepper1.distanceToGo()==0){ goTo(stepper1.currentPosition()+2); }
      }else{
        stepper1.setCurrentPosition(HomeAdjuster);
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
      int irNow = digitalRead(IR_SENSOR);
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
void OutputNeedData(){
  char printOutput[1024];
  int charCount=sprintf(printOutput,
    "homingActive: %s\n" \
    "turretFillTo9Requested: %s\n" \
    "loadedCount: %i\n" \
    "ninthSettleActive: %s\n" \
    "deckIsUp: %s\n" \
    "releaseDwellActive: %s\n" \
    "turretReleaseRequested: %s\n" \
    "backgroundRefillRequested: %s\n" \
    "releaseDwellStart: %i\n" \
    "forceConveyorForBallReturn: %s \n" \
    "conesFullHoldArmed: %s \n" \
    "waitingForBall: %s\n" \
    "deckConeCount: %i\n", \
    homingActive ? "true" : "false",turretFillTo9Requested ? "true" : "false",loadedCount, \
    ninthSettleActive ? "true" : "false",deckIsUp ? "true" : "false",releaseDwellActive ? "true":"false", \
    turretReleaseRequested ? "true" : "false", \
    backgroundRefillRequested ? "true" : "false", releaseDwellStart, \
    forceConveyorForBallReturn ? "true" : "false",conesFullHoldArmed ? "true" : "false", \
    waitingForBall ? "true" : "false", deckConeCount);
  Serial.print(printOutput);Serial.print(":");Serial.println(charCount);
}
void ConveyorOn(){  digitalWrite(MOTOR_RELAY, CONVEYOR_ACTIVE_HIGH?HIGH:LOW); conveyorIsOn=true; }
void ConveyorOff(){ digitalWrite(MOTOR_RELAY, CONVEYOR_ACTIVE_HIGH?LOW :HIGH); conveyorIsOn=false;}

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

void handleCommand(String cmd){
  cmd.trim();

  if(cmd.startsWith("SET_INPUT:")){
    int scoreMorePin=cmd.substring(10).toInt();
    int pin=resolveArduinoPin(scoreMorePin);
/*    Serial.print("LOG: cmd, ScoremorePin, pin"); 
    Serial.print(cmd); 
    Serial.print(",");
    Serial.print(scoreMorePin);
    Serial.print(",");
    Serial.println(pin);*/
    if(pin!=-1) {
      if(!isTrackedInput(pin) && inputCount<maxPins){
        if(isBowlingPin(scoreMorePin)) pinMode(pin, INPUT_PULLUP); else pinMode(pin, INPUT);
        inputPins[inputCount]=pin; pinStates[inputCount]=digitalRead(pin); inputCount++;
        Serial.print("ACK_SET_INPUT:"); Serial.println(scoreMorePin);
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
        if(scoreMorePin==10){
          bool prev=strikeLightOn;
          strikeLightOn=(value!=0);
          if(!prev && strikeLightOn && !strikeEdgeLatched){
            startStrikeCycle(); strikeEdgeLatched=true;
          }
          if(prev && !strikeLightOn){ strikeEdgeLatched=false; }
        }

        // Fill-ball grant (ScoreMore logical pin 7)
        else if (scoreMorePin==7){
          bool prev = autoResetFillBall;
          bool isHigh = (value != 0);
          if(!prev && isHigh && !autoResetEdgeLatched){
            autoResetFillBall = true;
            autoResetEdgeLatched = true;
            inFillBall = true;

            for(int i=0;i<LANE_LEN1;i++) laneA.setPixelColor(i, C_GREEN(laneA));
            for(int i=0;i<LANE_LEN2;i++) laneB.setPixelColor(i, C_GREEN(laneB));
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
  } else if(cmd=="PINSETTER_RESET"){
    stepIndex=24;
    Serial.println("ACK_PINSETTER_RESET");
  } else Serial.println("ACK_UNKNOWN_COMMAND");
}

void checkInputChanges(){
  for(int i=0;i<inputCount;i++){
    int currentState=digitalRead(inputPins[i]);
    if(currentState!=pinStates[i]){
      pinStates[i]=currentState;
      int scoreMorePin=getScoreMorePin(inputPins[i]);
      if(scoreMorePin!=-1){
        Serial.print("INPUT_CHANGE:"); Serial.print(scoreMorePin); Serial.print(":"); Serial.println(currentState);
      } else {
        Serial.print("ACK_WRITE_INVALID_PIN:");Serial.println(scoreMorePin);
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
  for(int i=0;i<maxPins;i++) if(scoreMorePins[i]==scoreMorePin) return arduinoPins[i];
  return -1;
}
int getScoreMorePin(int arduinoPin){
  for(int i=0;i<maxPins;i++) if(arduinoPins[i]==arduinoPin) return scoreMorePins[i];
  return -1;
}
bool isBowlingPin(int scoreMorePin){
  for(int i=0;i<bowlingPinCount;i++) if(bowlingPins[i]==scoreMorePin) return true;
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
//  3) Slow move to purge release (Pin10ReleasePos + EmptyTurretExtraOffset).
//  4) Dwell so pins fully dump.
//  5) Start non-blocking move back toward hall; runTurret() will stop on HALL_EFFECT.
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
  while (digitalRead(HALL_EFFECT) == HIGH && stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  stepper1.stop();
  while (stepper1.isRunning()) {
    stepper1.run();
  }

  stepper1.setCurrentPosition(HomeAdjuster);

  // 2) Fast move to pin 9
  goTo(PinPositions[9]);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  // 3) Slow spring-safe move to purge release
  long purgeReleasePos = Pin10ReleasePos() + EmptyTurretExtraOffset;

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
  deckA.begin(); deckB.begin(); laneA.begin(); laneB.begin();
  deckA.setBrightness(LED_BRIGHTNESS_NORMAL);
  deckB.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneA.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneB.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneMode=LANE_IDLE_WHITE;
}

void deckAll(uint32_t col){
  for(int i=0;i<DECK_LEN1;i++) deckA.setPixelColor(i,col);
  for(int i=0;i<DECK_LEN2;i++) deckB.setPixelColor(i,col);
}
void laneAll(uint32_t col){
  for(int i=0;i<LANE_LEN1;i++) laneA.setPixelColor(i,col);
  for(int i=0;i<LANE_LEN2;i++) laneB.setPixelColor(i,col);
}
void ledsShowAll(){ deckA.show(); deckB.show(); laneA.show(); laneB.show(); }
void laneShowOnly(){ laneA.show(); laneB.show(); }

// ---- STRIKE EFFECTS ----
void startStrikeWipe(){
  if(scoreWindowActive) return;
  lanePauseArmed=true;
  laneA.setBrightness(LED_BRIGHTNESS_STRIKE);
  laneB.setBrightness(LED_BRIGHTNESS_STRIKE);
  strikeWipeStartMs=millis(); strikeLastFrameMs=0; laneMode=LANE_STRIKE_WIPE;
}

void startStrikeFlash(){
  if(scoreWindowActive) return;
  flashOnPhase=true; flashCycles=0; flashLastMs=millis(); laneMode=LANE_STRIKE_FLASH;
  uint32_t redA=C_RED(laneA), redB=C_RED(laneB);
  for(int i=0;i<LANE_LEN1;i++) laneA.setPixelColor(i,redA);
  for(int i=0;i<LANE_LEN2;i++) laneB.setPixelColor(i,redB);
  laneShowOnly();
}

void startBallCometImmediate(){
  lanePauseArmed=true;
  laneAll(C_OFF(laneA)); laneShowOnly();
  ballCometStartMs=millis(); ballCometLastFrame=0; laneMode=LANE_BALL_COMET;
}

void endAllLaneAnimsToWhite(){
  laneAll(C_WHITE(laneA)); laneShowOnly(); laneMode=LANE_IDLE_WHITE;
  laneA.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneB.setBrightness(LED_BRIGHTNESS_NORMAL);
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
      int n1=(int)(t*LANE_LEN1+0.5f), n2=(int)(t*LANE_LEN2+0.5f);
      uint32_t redA=C_RED(laneA), redB=C_RED(laneB);
      for(int i=0;i<LANE_LEN1;i++)
        laneA.setPixelColor(i,(i<n1)?redA:C_WHITE(laneA));
      for(int i=0;i<LANE_LEN2;i++)
        laneB.setPixelColor(i,(i<n2)?redB:C_WHITE(laneB));
      laneShowOnly();
    }
  }

  // STRIKE FLASH
  if(!scoreWindowActive && laneMode==LANE_STRIKE_FLASH){
    if(flashOnPhase){
      if(now - flashLastMs >= FLASH_ON_MS){
        flashOnPhase=false; 
        flashLastMs=now;
        laneAll(C_WHITE(laneA)); 
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
          uint32_t redA=C_RED(laneA), redB=C_RED(laneB);
          for(int i=0;i<LANE_LEN1;i++) laneA.setPixelColor(i,redA);
          for(int i=0;i<LANE_LEN2;i++) laneB.setPixelColor(i,redB);
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
      int maxIndexA=LANE_LEN1+COMET_LEN, maxIndexB=LANE_LEN2+COMET_LEN;
      int headA=(int)(t*maxIndexA+0.5f), headB=(int)(t*maxIndexB+0.5f);

      for(int i=0;i<LANE_LEN1;i++) laneA.setPixelColor(i,C_OFF(laneA));
      for(int i=0;i<LANE_LEN2;i++) laneB.setPixelColor(i,C_OFF(laneB));

      for(int k=0;k<COMET_LEN;k++){
        int idxA=headA-k, idxB=headB-k;
        if(idxA>=0 && idxA<LANE_LEN1) laneA.setPixelColor(idxA,C_WHITE(laneA));
        if(idxB>=0 && idxB<LANE_LEN2) laneB.setPixelColor(idxB,C_WHITE(laneB));
      }
      laneShowOnly();
    }
  }
}

void startupWipeWhiteQuick(){
  deckAll(C_OFF(deckA)); laneAll(C_OFF(laneA)); ledsShowAll();
  int maxLen=LANE_LEN1;
  if(LANE_LEN2>maxLen) maxLen=LANE_LEN2;
  if(DECK_LEN1>maxLen) maxLen=DECK_LEN1;
  if(DECK_LEN2>maxLen) maxLen=DECK_LEN2;
  for(int i=0;i<maxLen;i++){
    if(i<DECK_LEN1) deckA.setPixelColor(i,C_WHITE(deckA));
    if(i<DECK_LEN2) deckB.setPixelColor(i,C_WHITE(deckB));
    if(i<LANE_LEN1) laneA.setPixelColor(i,C_WHITE(laneA));
    if(i<LANE_LEN2) laneB.setPixelColor(i,C_WHITE(laneB));
    ledsShowAll(); delay(STARTUP_WIPE_MS_PER_STEP);
  }
  frameLEDsFirstHalf();
}

// ======================= Frame LED helpers =======================
void updateFrameLEDs(){
  if (waitingForBall){
    if (throwCount == 1){
      digitalWrite(FRAME_LED1, HIGH);
      digitalWrite(FRAME_LED2, LOW);
    } else {
      digitalWrite(FRAME_LED1, HIGH);
      digitalWrite(FRAME_LED2, HIGH);
    }
  }
}

void frameLEDsFirstHalf(){
  digitalWrite(FRAME_LED1, HIGH);
  digitalWrite(FRAME_LED2, LOW);
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
  deckAll(C_RED(deckA));
  laneAll(C_RED(laneA));
  ledsShowAll();

  // Stop lane animations so laneUpdate() doesn't overwrite the solid red
  laneMode = LANE_IDLE_WHITE;
  lanePaused = false;
  lanePauseArmed = false;

  laneA.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneB.setBrightness(LED_BRIGHTNESS_NORMAL);
}

void setAllLightsWhite(){
  deckAll(C_WHITE(deckA));
  laneAll(C_WHITE(laneA));
  ledsShowAll();

  laneMode = LANE_IDLE_WHITE;
  lanePaused = false;
  lanePauseArmed = false;

  laneA.setBrightness(LED_BRIGHTNESS_NORMAL);
  laneB.setBrightness(LED_BRIGHTNESS_NORMAL);
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
