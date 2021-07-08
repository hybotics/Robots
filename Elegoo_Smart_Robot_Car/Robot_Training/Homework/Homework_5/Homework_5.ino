#include "Arduino.h"

// Motor control signals
#define LEFT_ENABLE_PIN   5
#define RIGHT_ENABLE_PIN   6
#define IN1_PIN   7
#define IN2_PIN   8
#define IN3_PIN   9
#define IN4_PIN   11

#define ONBOARD_LED_PIN 13
#define ONBOARD_LED_BLINK_RATE_MS 500

#define BLUETOOTH_DELAY_SEC 5

//  Speed of the robot in feet/second
#define SPEED_RATE_FORWARD 2.166              // Feet/Second
#define SPEED_RATE_REVERSE 2.10               // Feet/Second

//  Speed adjustments for each side so it goes more straight

//  Minimum and maximum speeds the car is allowed to go
#define SPEED_MIN 110
#define SPEED_MAX 245

//  Starting speed and adjustment settings
#define SPEED_LEFT SPEED_MIN
#define SPEED_LEFT_ADJUST 0
#define TURN_RATE_LEFT 10.0//289.0                  // Degrees/Second
#define TURN_RATE_LEFT_ADJUST 0

#define SPEED_RIGHT SPEED_MIN
#define SPEED_RIGHT_ADJUST 0
#define TURN_RATE_RIGHT 289.0                 // Degrees/Second
#define TURN_RATE_RIGHT_ADJUST 0

#define TURN_INCREMENT_DEG 15

//  Trackers for the current speed of each side
unsigned int leftSpeedCurrent = SPEED_MIN;
unsigned int rightSpeedCurrent = SPEED_MIN;
unsigned int currentSpeed = SPEED_MIN;
int speedIncrement = TURN_INCREMENT_DEG;

/*
  Utility functions
*/

void blinkLED(uint8_t ledPin, uint16_t rate_ms) {
  digitalWrite(ledPin, HIGH);
  delay(rate_ms);
  digitalWrite(ledPin, LOW);
  delay(rate_ms);
}

/*
  Robot functions
*/

void setSpeed(unsigned int leftSpeed, unsigned int rightSpeed) {
  enableMotors();

  if (leftSpeed < SPEED_MIN or leftSpeed + SPEED_LEFT_ADJUST > SPEED_MAX or
       rightSpeed < SPEED_MIN or rightSpeed + SPEED_RIGHT_ADJUST > SPEED_MAX) {
    //  Either the left speed or right speed, or both, are out of range
    while(true) {
      blinkLED(ONBOARD_LED_PIN, ONBOARD_LED_BLINK_RATE_MS);
    }
  } else {
    //  Set the speeds
    analogWrite(LEFT_ENABLE_PIN, leftSpeed + SPEED_LEFT_ADJUST);
    leftSpeedCurrent = leftSpeed;
    analogWrite(RIGHT_ENABLE_PIN, rightSpeed + SPEED_RIGHT_ADJUST);
    rightSpeedCurrent = rightSpeed;
  }

  //  Set the speeds
  analogWrite(LEFT_ENABLE_PIN, leftSpeed + SPEED_LEFT_ADJUST);
  leftSpeedCurrent = leftSpeed;
  analogWrite(RIGHT_ENABLE_PIN, rightSpeed + SPEED_RIGHT_ADJUST);
  rightSpeedCurrent = rightSpeed;
}

void disableMotors(void) {    
  digitalWrite(LEFT_ENABLE_PIN, LOW);
  digitalWrite(RIGHT_ENABLE_PIN, LOW);
}

void enableMotors(void) {
  digitalWrite(LEFT_ENABLE_PIN, HIGH);
  digitalWrite(RIGHT_ENABLE_PIN, HIGH);
}

void brakeFull(void) {
  disableMotors();

  //analogWrite(LEFT_ENABLE_PIN, 100);
  //analogWrite(RIGHT_ENABLE_PIN, 100);

  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void forward(float distance) {
  float runTimeMS;
  
  enableMotors();
  
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  runTimeMS = distance / SPEED_RATE_FORWARD * 1000;
 
  delay(runTimeMS); 

  brakeFull();
}

void reverse(float distance) {
  float runTimeMS;
  
  enableMotors();
  
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

  runTimeMS = distance / SPEED_RATE_REVERSE * 1000;

  delay(runTimeMS);

  brakeFull();
}

void turnLeftRelative(uint16_t turnDegrees) {
  uint16_t runTimeMS;
  
  enableMotors();

  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  runTimeMS = (int) (turnDegrees / (TURN_RATE_LEFT + TURN_RATE_LEFT_ADJUST) * 1000);
  //runTimeMS = 311;
  delay(runTimeMS);

  brakeFull();
}

void turnRightRelative(uint16_t turnDegrees) {
  float runTimeMS;
  
  enableMotors();
  
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

  runTimeMS = turnDegrees / (TURN_RATE_RIGHT + TURN_RATE_RIGHT_ADJUST) * 1000.0;

  delay(runTimeMS);

  brakeFull();
}

/*
  Calibration functions
*/

void calibrateForward() {
  digitalWrite(IN1_PIN,HIGH);
  digitalWrite(IN2_PIN,LOW);
  digitalWrite(IN3_PIN,LOW);
  digitalWrite(IN4_PIN,HIGH);
  delay(5000);
  brakeFull();
}

void calibrateReverse() {
  digitalWrite(IN1_PIN,LOW);
  digitalWrite(IN2_PIN,HIGH);
  digitalWrite(IN3_PIN,HIGH);
  digitalWrite(IN4_PIN,LOW);
  delay(5000);
  brakeFull();
}
void calibrateRight() {
  digitalWrite(IN1_PIN,HIGH);
  digitalWrite(IN2_PIN,LOW);
  digitalWrite(IN3_PIN,HIGH);
  digitalWrite(IN4_PIN,LOW);
  delay(5000);
  brakeFull();
}

void calibrateLeft() {
  digitalWrite(IN1_PIN,LOW);
  digitalWrite(IN2_PIN,HIGH);
  digitalWrite(IN3_PIN,LOW);
  digitalWrite(IN4_PIN,HIGH);
  delay(5000);
  brakeFull();
}

void setup(void) {
  Serial.begin(15200);
  
  // Setup all the pins
  pinMode(LEFT_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
/*
  //  Allow Bluetooth time to connect
  Serial.println("Waiting for Bluetooth to connect");
  for (uint8_t i = 0; i < BLUETOOTH_DELAY_SEC * 2; i++) {
    blinkLED(ONBOARD_LED_PIN, 500);
  }
*/
  //  Enable the motors
  Serial.println("Enabling motors.");
  enableMotors();

  //  Make sure the motors are stopped
  Serial.println("Stopping all motors");
  brakeFull();
  
  //  Set initial motor speeds
  Serial.println("Setting initial motor speeds");
  setSpeed(SPEED_LEFT + SPEED_LEFT_ADJUST, SPEED_RIGHT + SPEED_RIGHT_ADJUST);

/*  For Robot Calibration ONLY  */
  //setSpeed(255, 255);
  //calibrateForward();
  //calibrateReverse();
  //calibrateRight();
  //calibrateLeft();
  analogWrite(LEFT_ENABLE_PIN, 255);
  analogWrite(RIGHT_ENABLE_PIN, 255);
  turnLeftRelative(90);
}

void loop(void) {    
/*  
  char command;

  blinkLED(ONBOARD_LED_PIN, ONBOARD_LED_BLINK_RATE_MS);

  if (Serial.available()) {
    //  Handle commands from remote
    command = Serial.read();

    //  Convert lower to upper case for commands
    if (command >= 97 and command <= 122) {
      command -= 32;
    }

    switch(command) {
      case 'F':
        forward(1.0);
        break;

      case 'B':
        reverse(1.0);
        break;

      case 'R':
        turnRightRelative(TURN_INCREMENT_DEG);
        break;

      case 'L':
        turnLeftRelative(TURN_INCREMENT_DEG);
        break;
    }
  } else {
    turnLeftRelative(90);
    //  Do autonomous stuff
    setSpeed(SPEED_MIN, SPEED_MIN);

    while(true) {
      //  Change speed ramping direction if neccessary
      if (speedIncrement >= 0) {
        if (currentSpeed <= 250) {
          ;
        } else if (currentSpeed >= 250) {
          speedIncrement = -30;
        }
      } else {
        if (currentSpeed >= SPEED_MIN) {
          ;
        } else if (currentSpeed <= SPEED_MIN) {
            speedIncrement = 30;
        }
      }

      leftSpeedCurrent += speedIncrement;
      rightSpeedCurrent += speedIncrement;
      currentSpeed += speedIncrement;
      setSpeed(leftSpeedCurrent, rightSpeedCurrent);
    }
  }

  delay(1000);
*/
}
