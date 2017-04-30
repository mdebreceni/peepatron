// Peepatron source code 
// Mike Debreceni   2016-04-09
//  
// Additional Hardware Required
// *  Adafruit 16 channel 12 bit PWM shield for Arduino
//    ->  https://www.adafruit.com/products/1411
//    ->  Board's I2C address is default (0x40)
//    ->  Servos plugged in to outputs 4-9  
//    ->  PIVOT(4), SHOULDER(5), ELBOW(6), WRIST(7), TWIST(8), GRIPPER(9)
//
// *  Lectrobox PC Joystick shield for Arduino  (2x)
//    ->  http://www.lectrobox.com/products/arduino_pc_game_port_joystick_shield/
//    ->  Left stick:  Buttons are pins 6 (button A) and 7 (button B)
//              Analog X on A0, Y on A1
//    ->  Right stick: Buttons are pins 8 (button A) and 9 (button B)
//              Analog X on A2, Y on A3
// 
// *  2 x PC Joystick   (Logitech Wingman or any other analog PC gameport joystick)
// 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Joint.h"
#include <PCJoy.h>

#define DELAY_MILLIS 5
#define SLOW_MILLIS 20
#define INPUT_TIMEOUT_MILLIS 8000
#define SHOULDER_RESET_WAIT_MILLIS 500
#define GRIPPER_RESET_WAIT_MILLIS 300

// Initialize PWM driver using default address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

typedef enum JointID {PIVOT = 4, SHOULDER, ELBOW, WRIST, TWIST, GRIPPER} JointID;
typedef enum GripState {GRIP_STATE_OPEN = 0, GRIP_STATE_LOOSE, GRIP_STATE_TIGHT, GRIP_STATE_CLOSED} GripState;

#define PIVOT_POS_MIN 150
#define PIVOT_POS_MAX 550
#define PIVOT_POS_SANE 350

#define SHOULDER_POS_MIN 150
#define SHOULDER_POS_MAX 400
#define SHOULDER_POS_SANE 300
#define SHOULDER_TWEEN_DELAY_MILLIS 4

//#define ELBOW_POS_MIN 165
#define ELBOW_POS_MIN 350
#define ELBOW_POS_MAX 550
#define ELBOW_POS_SANE 450

//#define WRIST_POS_MIN 165
#define WRIST_POS_MIN 350
#define WRIST_POS_MAX 550
#define WRIST_POS_SANE 450

#define TWIST_POS_MIN 121
#define TWIST_POS_MAX 580
#define TWIST_POS_SANE 330

#define GRIPPER_POS_OPEN 195
#define GRIPPER_POS_CLOSED 335
#define GRIPPER_POS_SANE 200
#define GRIPPER_POS_LOOSE 280
#define GRIPPER_POS_TIGHT 330

long timeSinceLastInput;

// our servo # counter
uint8_t servonum = 0;

PCJoy *joy1;
PCJoy *joy2;

typedef struct arm_struct {
  Joint* pivot;
  Joint* shoulder;
  Joint* elbow;
  Joint* wrist;
  Joint* twist;
  Joint* gripper;
} Arm;

Arm* arm;

void setup() {
  Serial.begin(9600);
  //  Serial.println("16 channel Servo test!");

#ifdef ESP8266
  Wire.pins(2, 14);   // ESP8266 can use any two pins, such as SDA to #2 and SCL to #14
#endif

  // initialize Joystick
  joy1 = new PCJoy(A0, A1, 6, 7);
  joy2 = new PCJoy(A2, A3, 8, 9);


  // initialize joints and PWM interface
  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates


  arm = new Arm();

  // initialize pivot joint
  arm->pivot = new Joint(PIVOT, PIVOT_POS_MIN, PIVOT_POS_MAX, PIVOT_POS_SANE);
  arm->pivot->setPwmDriver(&pwm);

  // initialize shoulder joint
  arm->shoulder = new Joint(SHOULDER, SHOULDER_POS_MIN, SHOULDER_POS_MAX, SHOULDER_POS_SANE);
  arm->shoulder->setPwmDriver(&pwm);

  // initialize elbow joint
  arm->elbow = new Joint(ELBOW, ELBOW_POS_MIN, ELBOW_POS_MAX, ELBOW_POS_SANE);
  arm->elbow->setPwmDriver(&pwm);

  // initialize wrist joint
  arm->wrist = new Joint(WRIST, WRIST_POS_MIN, WRIST_POS_MAX, WRIST_POS_SANE);
  arm->wrist->setPwmDriver(&pwm);

  // initialize twist joint
  arm->twist = new Joint(TWIST, TWIST_POS_MIN, TWIST_POS_MAX, TWIST_POS_SANE);
  arm->twist->setPwmDriver(&pwm);

  // initialize gripper
  arm->gripper = new Joint(GRIPPER, GRIPPER_POS_OPEN, GRIPPER_POS_CLOSED, GRIPPER_POS_SANE);
  arm->gripper->setPwmDriver(&pwm);

  timeSinceLastInput = 0;

  setSanePos(arm);
  yield();
}

// send joint positions to arm
void setSanePos(Arm* arm) {

  // set shoulder to sane position first so arm doesn't hit anything
  arm->shoulder->setTweenDelayMillis(SHOULDER_TWEEN_DELAY_MILLIS);
  arm->shoulder->tweenPos(SHOULDER_POS_SANE);

  // set elbow, wrist pivot and twist back to sane position
  arm->wrist->tweenPos(WRIST_POS_SANE);
  arm->elbow->tweenPos(ELBOW_POS_SANE);
  arm->pivot->tweenPos(PIVOT_POS_SANE);
  arm->twist->tweenPos(TWIST_POS_SANE);


  // release any payload the gripper may be carrying
  arm->gripper->tweenPos(GRIPPER_POS_SANE);
  delay(GRIPPER_RESET_WAIT_MILLIS);
  waitForNoTrigger();
}

void waitForNoTrigger(void) {
  PCJoy_State joyStateLeft = joy1->getState();
  PCJoy_State joyStateRight = joy2->getState();
  while (getGripState(joyStateLeft, joyStateRight) != GRIP_STATE_OPEN) {
    delay(100);
    joyStateLeft = joy1->getState();
    joyStateRight = joy2->getState();

  }
}

void loop() {
  // main loop
  // collect state of each joystick
  bool gotInput = false;
  PCJoy_State joyStateLeft = joy1->getState();
  PCJoy_State joyStateRight = joy2->getState();

  // handle left stick position (shoulder joint and pivot)
  if (joyStateLeft.isConnected) {
    gotInput |= handleLeftStick(joyStateLeft);

  }

  // handle right stick position (elbow/wrist joint and twist joint
  if (joyStateRight.isConnected) {
    gotInput |= handleRightStick(joyStateRight);
  }

  // determine if we want an open, loose or tight grip
  GripState gripState = getGripState(joyStateRight, joyStateLeft);

  // Treat trigger (claw movement) as input?
  // gotInput |= (gripState != GRIP_STATE_OPEN);

  // translate grip state into a grip position
  int gripPos = getGripPosFromGripState(gripState);
  arm->gripper->setPos(gripPos);

  // overall movement speed is controlled by a short delay of DELAY_MILLIS between movements
  delay(DELAY_MILLIS);

  // check if the 'slow' button is pressed on either stick
  if (getSlowMode(joyStateRight, joyStateLeft) ) {
    // slow mode is implemented by delaying a bit between incremental movements
    delay(SLOW_MILLIS);
    timeSinceLastInput += SLOW_MILLIS;
  }

  if (gotInput) {
    timeSinceLastInput = 0;
  } else {
    timeSinceLastInput += DELAY_MILLIS;
  }

  if (timeSinceLastInput > INPUT_TIMEOUT_MILLIS) {
    setSanePos(arm);
    timeSinceLastInput = 0;
  }
}

bool getSlowMode(PCJoy_State& joyStateRight, PCJoy_State& joyStateLeft) {
  // Slow mode is enabled by pressing Button B on either or both joysticks

  bool retVal = false;
  if (joyStateLeft.bDown || joyStateRight.bDown) {
    retVal = true;
  }
  return retVal;
}


GripState getGripState(PCJoy_State& joyStateRight, PCJoy_State& joyStateLeft) {
  // Gripper date is selected using Button A on either joystick  (trigger on Logitech Wingman)
  // grip state mapping:
  // No triggers pressed =>  open
  // One trigger pressed =>  loose grip
  // Two triggers pressed =>  tight grip

  GripState gripState = GRIP_STATE_OPEN;

  if ((joyStateLeft.isConnected && joyStateLeft.aDown) || (joyStateRight.isConnected && joyStateRight.aDown)) {
    // if eithter trigger is pressed
    gripState = GRIP_STATE_LOOSE;
  }

  return gripState;
}



int getGripPosFromGripState(GripState gripState) {
  int pos = GRIPPER_POS_OPEN;
  // translate a GripState to a gripper position  (defined at top of this file)

  switch (gripState) {
    case GRIP_STATE_OPEN:
      pos = GRIPPER_POS_OPEN;
      break;
    case GRIP_STATE_LOOSE:
      pos = GRIPPER_POS_LOOSE;
      break;
    case GRIP_STATE_TIGHT:
      pos = GRIPPER_POS_TIGHT;
      break;
    case GRIP_STATE_CLOSED:
      pos = GRIPPER_POS_CLOSED;
      break;
    default:
      pos = GRIPPER_POS_SANE;
      break;
  }

  return pos;
}

bool handleRightStick(PCJoy_State& joyState) {
  bool gotInput = false;

  if (joyState.isLeft) {
    // pivot to the left (counter clockwise)
    arm->pivot->incrementPos();
    gotInput = true;

  }
  if (joyState.isRight) {
    // pivot to the right (clockwise)
    arm->pivot->decrementPos();
    gotInput = true;

  }

  if (joyState.isUp) {
    // lift arm
    arm->shoulder->decrementPos();
    gotInput = true;

  }
  if (joyState.isDown) {
    // lower arm
    arm->shoulder->incrementPos();
    gotInput = true;

  }

}

bool handleLeftStick(PCJoy_State& joyState) {
  bool gotInput = false;
  if (joyState.isLeft) {
    // twist counter-clockwise
    arm->twist->decrementPos();
    gotInput = true;
  }
  if (joyState.isRight) {
    // twist clockwise
    arm->twist->incrementPos();
    gotInput = true;

  }

  if (joyState.isUp) {
    // wrist and elbow are controlled together rather than independently
    // this will curl arm downward
    arm->wrist->incrementPos();
    arm->elbow->incrementPos();
    gotInput = true;

  }

  if (joyState.isDown) {
    // this will cause arm to straighten or curl upward
    arm->wrist->decrementPos();
    arm->elbow->decrementPos();
    gotInput = true;

  }
  return gotInput;
}

