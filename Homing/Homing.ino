// VERSION 1.0 - original rev2 files by Danny Lum

#define MOTOR_RELAY  4
#define IR_SENSOR 5
#define HOME_SWITCH 6

#define SCISSOR_PIN 7
#define SLIDE_PIN 8
#define RAISE_LEFT_PIN 9
#define RAISE_RIGHT_PIN 10
#define LEFT_SWEEP_PIN 11
#define RIGHT_SWEEP_PIN 12
#define BALL_RETURN_PIN 13
#define DEBOUNCE_DELAY 20 


#include <Servo.h>

//Scissors start at 90 and close at 130
//Deck slider starts at 180 and finishes at (lower number than 180)
//Left raise starts at 180, and goes down
//Right raise starts at 0, and goes down
//Sweep Servo starts at the back of the lane
//SweepLeft Starts at 0, guard position at 60, and up idle position at 100
//SweepRight starts at 180, guard position at 120, and up idle position at 80
//Ball Door starts down at 0, up position is at 180

Servo LeftRaiseServo;  // create Servo object to control a servo
Servo RightRaiseServo;  // create Servo object to control a servo
Servo SlideServo;
Servo ScissorsServo;
Servo LeftSweepServo;
Servo RightSweepServo;
Servo BallReturnServo;

int SweepGuardAngle = 50;


void setup() {
  // put your setup code here, to run once:
    LeftRaiseServo.attach(RAISE_LEFT_PIN);  // attaches the servo on pin 9 to the Servo object
    RightRaiseServo.attach(RAISE_RIGHT_PIN);  // attaches the servo on pin 9 to the Servo object
    SlideServo.attach(SLIDE_PIN);
    ScissorsServo.attach(SCISSOR_PIN);
    LeftSweepServo.attach(LEFT_SWEEP_PIN);
    RightSweepServo.attach(RIGHT_SWEEP_PIN);
    BallReturnServo.attach(BALL_RETURN_PIN);


  //moves upwards HOME
 LeftRaiseServo.write(180); //DECK UP 
 RightRaiseServo.write(0); //DECK UP 
 SlideServo.write(180); //SLIDING DECK FORWARD
 ScissorsServo.write(90); //SCISSOR OPEN
 BallReturnServo.write(0); //BALL RETURN CLOSED
LeftSweepServo.write(0); //BACK OF LANE
RightSweepServo.write(180); //BACK OF LANE

  delay(5000);




}

void loop() {

  //ScissorsServo.write(130); //SCISSORS CLOSED
  //SlideServo.write(100); //SLIDING DECK BACK
  //BallReturnServo.write(180); //BALL RETURN OPEN

//LeftRaiseServo.write(20); //DECK PIN SET
 //RightRaiseServo.write(160); //DECK PIN SET

//LeftSweepServo.write(50); //GUARD POSITION
//RightSweepServo.write(130); //GUARD POSITION


LeftSweepServo.write(85); //UP POSITION
RightSweepServo.write(95); //UP POSITION
delay(5000);


}
