/*********************************************************************
    This is an example for our nRF51822 based Bluefruit LE modules

    Modified to drive a 3-wheeled BLE Robot Rover! by http://james.devi.to

    Pick one up today in the Adafruit shop!

    Adafruit invests time and resources providing this open source code,
        please support Adafruit and open-source hardware by purchasing
        products from Adafruit!

    MIT license, check LICENSE for more information.
    All text above, and the splash screen below must be included in
        any redistribution.

    16-Jul-2016: Modified extensively by geekguy@hybotics.org
        Added speed control - slow down is button 1, speed up is button
            2, Stop is button 4. The rover now moves until stopped.
        Motion and speed control is now done with just 4 buttons. The
            forward and reverse buttons double as speed control. Turning
            is not considered movement because the rover stays in place.

    18-Jul-2016:
        Added switch to toggle between Autonomous Operation and Manual
        Control. This was a pretty extensive modification.

    08-Aug-2016:
        Added a pivotMode to switch between pivoting and gradual turning

    27-Aug-2016:
        Removed the pibotMode switch, and replaced it with an enum TurnType
        Added the ability to control the rate of a gradual turn.
*********************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Adafruit_MotorShield.h>
// #include "utility/Adafruit_PWMServoDriver.h"
// #include <Servo.h>
#include <string.h>

#include "Little_Red_Rover.h"

//  Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//  And connect 2 DC motors to port M3 & M4 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

//  Not used, testing acceleration
//  int accelTime = 200;

//  Name your RC here
String BROADCAST_NAME = "Little Red Rover WIP";

String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//  Global variables
int velocity = 0;

float x, y;

int L_restrict = 0;
int R_restrict = 0;

unsigned long lastAccelPacket = 0;

bool isMoving = false;

bool modeToggle = false;

uint8_t leftMSpeed = MAXSPEED / 4;
uint8_t rightMSpeed = MAXSPEED / 4;
uint8_t lastLeftSpeed = leftMSpeed;
uint8_t lastRightSpeed = rightMSpeed;

enum TurnType turning = None;

bool autonomous = false;

byte lastButton = 0;
uint8_t buttonNumber = 0;
boolean buttonPressed = false;

//  A small helper
void error(const __FlashStringHelper *err) {
    Serial.print(F(">>> ERROR: "));
    Serial.println(err);
    while (true);
}

//  Function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

//  The packet buffer
extern uint8_t packetbuffer[];

char buf[60];

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void) {
#if DEBUG
    Serial.begin(115200);
    delay(5000);
    Serial.println(F("Hybotics Little Red Rover WIP"));
    Serial.println();
#endif

    AFMS.begin();  // Create with the default frequency 1.6KHz

    //    Turn on motors
    L_MOTOR->setSpeed(0);
    L_MOTOR->run(RELEASE);

    R_MOTOR->setSpeed(0);
    R_MOTOR->run(RELEASE);

    //    Initialize the module
    BLEsetup();

    //    Allow some time to place the rover on the floor
    delay(STARTDELAYMS);
}

void loop(void) {
    uint8_t len;

    //  Read new packet data
    len = readPacket(&ble, BLE_READPACKET_TIMEOUT);

    //  Buttons
    if (packetbuffer[1] == 'B') {
        buttonNumber = packetbuffer[2] - '0';
        buttonPressed = packetbuffer[3] - '0';

        if (buttonPressed and (buttonNumber == 3)) {
            autonomous = !autonomous;

            if (autonomous) {
#if DEBUG
                Serial.println(F("Autonomous Operation"));
                Serial.println(F("Stopping motors"));
#endif
                stopMotors(0);
            } else {
#if DEBUG
                Serial.println(F("Manual Control"));
#endif
            }
        }
    }

    if (autonomous) {
        //  Code for autonomous operation goes here
    } else {
        // Read from Accelerometer input
        if ( accelMode() ) {
            lastAccelPacket = millis();
            modeToggle = true;
            return;
        }

        //  Stop motors if accelerometer data is turned off (100ms timeout)
        if ( ((millis() - lastAccelPacket) > 100) and (modeToggle) ) {
            L_MOTOR->run(RELEASE);
            R_MOTOR->run(RELEASE);
            modeToggle = false;

            return;
        }

        if ( !modeToggle ) {
            buttonMode();
        }
    }
}

bool accelMode() {
    if (packetbuffer[1] == 'A') {
        x = parsefloat( packetbuffer + 2 );
        y = parsefloat( packetbuffer + 6 );

        if ( x <= -0.55 ) {
            x += 0.55;
            x *= -100.0;
            L_MOTOR->run( BACKWARD );
            R_MOTOR->run( BACKWARD );

            if ( x >= 45 ) {
                x = 45;
            }

            if ( x <= 0 ) {
                x = 0;
            }

            velocity = map( x, 0, 45, 0 , 255 );
#if DEBUG
            Serial.println(F("Accel: Moving REVERSE"));
#endif
        } else if ( x >= -0.25 ) {
            x += 0.25;
            x *= 100;
            L_MOTOR->run( FORWARD );
            R_MOTOR->run( FORWARD );

            if ( x >= 45 ) {
                x = 45;
            }

            if ( x <= 0 ) {
                x = 0;
            }

            velocity = map( x, 0, 45, 0, 255 );
#if DEBUG
            Serial.println(F("Accel: Moving FORWARD"));
#endif
        } else {
            L_MOTOR->run( RELEASE );
            R_MOTOR->run( RELEASE );
            velocity = 0;
#if DEBUG
        Serial.println(F("Accel: ALL STOP!"));
#endif
        }

        //  Account for L / R accel
        if ( y >= 0.1 ) {
            y -= 0.1;
            y *= 100;

            if ( y >= 50 ) {
                y = 50;
            }

            if ( y <= 0 ) {
                y = 0;
            }

            L_restrict = fscale( y, 0.0, 50.0, 0.0, 100.0, -4.0 );
        } else if ( y <= -0.1 ) {
            y += 0.1;
            y *= -100;

            if ( y >= 50 ) {
                y = 50;
            }

            if ( y <= 0 ) {
                y = 0;
            }

            R_restrict = fscale( y, 0.0, 50.0, 0.0, 100.0, -4.0 );
        } else {
            L_restrict = 0;
            R_restrict = 0;
        }

        float Lpercent = ( 100.0 - L_restrict ) / 100.00 ;
        float Rpercent = ( 100.0 - R_restrict ) / 100.00 ;

#if DEBUG
        Serial.print("Accel: x = ");
        Serial.print( x );
        Serial.print( ", Lpercent = " );
        Serial.print( Lpercent );
        Serial.print( ", velocity = " );
        Serial.print( velocity );
        Serial.print( ", Rpercent = " );
        Serial.println( Rpercent );
#endif

        L_MOTOR->setSpeed( velocity * Lpercent );
        R_MOTOR->setSpeed( velocity * Rpercent );

        return true;
    }

    return false;
}

bool buttonMode() {
    uint8_t speedCheck = 0;
    static unsigned long lastPress = 0;

    if (buttonPressed) {
        lastLeftSpeed = leftMSpeed;
        lastRightSpeed = rightMSpeed;
#if DEBUG
        Serial.print(F("In: isMoving = "));
        Serial.print(isMoving);
        Serial.print(F(", turning = "));

        switch (turning) {
            case None:
                Serial.print("None");
                break;

            case Left:
            Serial.print("Left");
            break;

            case Right:
                Serial.print("Right");
                break;

            case Pivot:
                Serial.print("Pivot");
                break;
        }
#endif

#if DEBUG
        Serial.print(F(", buttonPressed = "));
        Serial.print(buttonPressed);
        Serial.print(F(", buttonNumber = "));
        Serial.println(buttonNumber);
#endif

        switch (buttonNumber) {
            case 1:
#if DEBUG
                Serial.println(F("*** Unassigned"));
#endif
                break;

            case 2:
                if (turning == Pivot) {
                    turning = None;
                } else {
                    turning = Pivot;
                }
#if DEBUG
                if (turning == Pivot) {
                    Serial.println(F("*** Pivoting to turn"));
                } else {
                    Serial.println(F("*** Gradual turning"));
                }
#endif
                break;

            case 3:
#if DEBUG
                Serial.println(F("*** Autonomous/Manual Toggle"));
#endif
                break;

            case 4:
                stopMotors(0);
#if DEBUG
                Serial.println(F("*** ALL STOP!"));
#endif
                break;

            case 5:
                speedCheck = MAXSPEED - SPEEDINCR;

                if (isMoving and (turning == Pivot) and (leftMSpeed < speedCheck) and (rightMSpeed < speedCheck)) {
                    leftMSpeed += SPEEDINCR;
                    rightMSpeed += SPEEDINCR;

#if DEBUG
                    Serial.println(F("*** Speeding Up"));
#endif
                } else if ((turning == Left) and (rightMSpeed < speedCheck)) {
                    rightMSpeed += SPEEDINCR;
                } else if ((turning == Right) and (leftMSpeed < speedCheck)) {
                    leftMSpeed += SPEEDINCR;
                } else {
                    L_MOTOR->run(FORWARD);
                    R_MOTOR->run(FORWARD);

                    isMoving = true;
#if DEBUG
                    Serial.println(F("*** Moving FORWARD"));
#endif
                }
                break;

            case 6:
                speedCheck = MINSPEED + SPEEDINCR;

                if (isMoving and (turning == Pivot) and (leftMSpeed > speedCheck) and (rightMSpeed > speedCheck)) {
                    leftMSpeed -= SPEEDINCR;
                    rightMSpeed -= SPEEDINCR;

#if DEBUG
                    Serial.println(F("*** Slowing Down"));
#endif
                } else if ((turning == Left) and (rightMSpeed > speedCheck)) {
                    rightMSpeed -= SPEEDINCR;
                } else if ((turning == Right) and (leftMSpeed > speedCheck)) {
                    leftMSpeed -= SPEEDINCR;
                } else {
                    L_MOTOR->run(BACKWARD);
                    R_MOTOR->run(BACKWARD);

                    isMoving = true;
#if DEBUG
                    Serial.println(F("*** Moving in REVERSE"));
#endif
                }
                break;

            case 7:
                if (turning == Pivot) {
                    stopMotors(250);

                    L_MOTOR->run(BACKWARD);
                    R_MOTOR->run(FORWARD);
                } else {
                    speedCheck = MAXSPEED - TURNINCR;
                    turning = Left;

                    if (rightMSpeed < speedCheck) {
                        rightMSpeed += TURNINCR;
                    }
                }

                isMoving = false;

#if DEBUG
                Serial.println(F("*** Turning LEFT"));
#endif
                break;

            case 8:
                if (turning == Pivot) {
                    stopMotors(250);

                    L_MOTOR->run(FORWARD);
                    R_MOTOR->run(BACKWARD);

                    isMoving = false;
                } else {
                    speedCheck = MAXSPEED - TURNINCR;
                    turning = Right;

                    if (leftMSpeed < speedCheck) {
                        leftMSpeed += TURNINCR;
                    }

                    isMoving = true;
                }

#if DEBUG
                Serial.println(F("*** Turning RIGHT"));
#endif
                break;
        }

        lastButton = buttonNumber;
        lastPress = millis();

#if DEBUG
        Serial.print(F("Out: isMoving = "));
        Serial.println(isMoving);
        Serial.print(F(", turning = "));

        switch (turning) {
            case None:
            Serial.print("None");
            break;

        case Left:
            Serial.print("Left");
            break;

            case Right:
            Serial.print("Right");
            break;

        case Pivot:
            Serial.print("Pivot");
            break;
    }

    Serial.print(F("Speed: Left = "));
    Serial.print(leftMSpeed);
    Serial.print(F(", Right = "));
    Serial.println(rightMSpeed);
    Serial.println();
#endif

    // Don't set motor speed unless changing directon or gradual turn
    if (((turning == Pivot) and ((buttonNumber == 5) or (buttonNumber == 6))) or
        ((turning != Pivot) and ((buttonNumber == 7) or (buttonNumber == 8)))) {
#if DEBUG
      Serial.println(F("Setting motor speed"));
#endif
      L_MOTOR->setSpeed(leftMSpeed);
      R_MOTOR->setSpeed(rightMSpeed);
    }

    // Restore normal speed
    if ((turning == Left) or (turning == Right)) {
      leftMSpeed = lastLeftSpeed;
      rightMSpeed = lastRightSpeed;
    }

    return true;
}

  // if(isMoving){
  // unsigned long timeSincePress = millis() - lastPress;
  // if(timeSincePress <= accelTime){

  //   Serial.println( timeSincePress ) ;

  //   int motorSpeed = map( timeSincePress, 0, accelTime, 0, 255 );

  //   L_MOTOR->setSpeed(motorSpeed);
  //   R_MOTOR->setSpeed(motorSpeed);
  // }

  // else{
  // // full speed ahead!
  // L_MOTOR->setSpeed(255);
  // R_MOTOR->setSpeed(255);
  // }
  // }

  return false;
}

void BLEsetup() {
#if DEBUG
  Serial.print(F("Initializing the Bluefruit LE module.."));
#endif

  if ( !ble.begin(VERBOSE_MODE) ) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

#if DEBUG
  Serial.println( F("OK!") );

  // Perform a factory reset to make sure everything is in a known state
  Serial.println(F("Performing a factory reset.."));
#endif

  if (! ble.factoryReset() ) {
    error(F("Couldn't factory reset"));
  }

  // Convert the name change command to a char array
  BROADCAST_CMD.toCharArray(buf, 60);

  // Change the broadcast device name
  if (ble.sendCommandCheckOK(buf)) {
#if DEBUG
    Serial.println("Name changed");
#endif
  }

  delay(250);

  // Reset to take effect
  if (ble.sendCommandCheckOK("ATZ")) {
#if DEBUG
    Serial.println("Resetting..");
#endif
  }

  delay(250);

  // Confirm name change
  ble.sendCommandCheckOK("AT+GAPDEVNAME");

  /* Disable command echo from Bluefruit */
  ble.echo(false);

#if DEBUG
  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();
#endif

  ble.verbose(false);  // Debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

#if DEBUG
  Serial.println(F("*****************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
#endif

  ble.setMode(BLUEFRUIT_MODE_DATA);

#if DEBUG
  Serial.println(F("*****************"));
#endif
}

//Logarithmic mapping function from http://playground.arduino.cc/Main/Fscale
float fscale( float inputValue,  float originalMin, float originalMax, float newBegin, float newEnd, float curve) {
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;

  // condition curve parameter
  // limit range

  if (curve > 10) {
    curve = 10;
  }

  if (curve < -10) {
    curve = -10;
  }

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

#if DEBUG
    Serial.print(F("Curve = "));
    Serial.println(curve * 100, DEC);   // Multply by 100 to preserve resolution
    Serial.println();
#endif

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }

  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin) {
    NewRange = newEnd - newBegin;
  } else {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

#if DEBUG
    Serial.print(F("Original range = "));
    Serial.print(OriginalRange, DEC);
    Serial.print(", New range =");
    Serial.print(NewRange, DEC);
    Serial.print(", ");
    Serial.print(F("Zero Ref Curval = "));
    Serial.println(zeroRefCurVal, DEC);
    Serial.println();
#endif

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
  } else {    // invert the ranges
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}

/********************************************************************/
/*  Utility routines                        */
/********************************************************************/

/*
    Left zero pad a numeric string
*/
String leftZeroPadString (String st, uint8_t nrPlaces) {
  uint8_t i, len;
  String newStr = st;

  if (newStr.length() < nrPlaces) {
    len = st.length();

    for (i = len; i < nrPlaces; i++) {
      newStr = String("0" + newStr);
    }
  }

  return newStr;
}

/*
    Convert a pulse width in ms to inches
*/
long microsecondsToInches (long microseconds) {
  /*
    According to Parallax's datasheet for the PING))), there are
      73.746 microseconds per inch (i.e. sound travels at 1130 feet per
      second).  This gives the distance travelled by the ping, outbound
      and return, so we divide by 2 to get the distance of the obstacle.

    See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  */

  return microseconds / 74 / 2;
}

/*
    Convert a pulse width in ms to a distance in cm
*/
long microsecondsToCentimeters (long microseconds) {
  /*
    The speed of sound is 340 m/s or 29 microseconds per centimeter.

    The ping travels out and back, so to find the distance of the
      object we take half of the distance travelled.
  */

  return microseconds / 29 / 2;
}

/*
    Pulses a digital pin for a duration in ms
*/
void pulseDigital(int pin, int duration) {
  digitalWrite(pin, HIGH);      // Turn the ON by making the voltage HIGH (5V)
  delay(duration);          // Wait for duration ms
  digitalWrite(pin, LOW);       // Turn the pin OFF by making the voltage LOW (0V)
  delay(duration);          // Wait for duration ms
}

/*
  Convert a temperature in Celsius to Fahrenheit
*/
float toFahrenheit (float celsius) {
  return (celsius * 1.8) + 32;
}

/*
    Trim trailing zeros from a numeric string
*/
String trimTrailingZeros (String st) {
  uint8_t newStrLen = 0;
  String newStr = st;

  newStrLen = newStr.length();

  while (newStr.substring(newStrLen - 1) == "0") {
    newStrLen -= 1;
    newStr = newStr.substring(0, newStrLen);
  }

  return newStr;
}

void stopMotors (int ms) {
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);

  if (ms > 0) {
    delay(ms);
  }

  isMoving = false;
}

/*
  Function to read a value from a GP2Y0A21YK0F infrared distance sensor and return a
    distance value in centimeters.

  This sensor should be used with a refresh rate of 36ms or greater.

  Javier Valencia 2008

  float readIR(byte pin)

  It can return -1 if something has gone wrong.

  TODO: Make several readings over a time period, and average them
    for the final reading.

  NOTE: This code is for the older Sharp GP2D12 IR sensor, and will no
    doubt have to be adjusted to work correctly with the newer sensor.
*/
float readIR (byte sensorNr) {
  byte pin = sensorNr + IR_PIN_BASE;
  int tmp;

  tmp = analogRead(pin);

  if (tmp < 3) {
    return -1;                                  // Invalid value
  } else {
    return (6787.0 /((float)tmp - 3.0)) - 4.0;  // Distance in cm
  }
}

/*
  Ping))) Sensor

  This routine reads a PING))) ultrasonic rangefinder and returns the
    distance to the closest object in range. To do this, it sends a pulse
    to the sensor to initiate a reading, then listens for a pulse
    to return.  The length of the returning pulse is proportional to
    the distance of the object from the sensor.

  The circuit:
    * +V connection of the PING))) attached to +5V
    * GND connection of the PING))) attached to ground
    * SIG connection of the PING))) attached to digital pin 7

  http://www.arduino.cc/en/Tutorial/Ping

  Created 3 Nov 2008
    by David A. Mellis

  Modified 30-Aug-2011
    by Tom Igoe

  Modified 09-Aug-2013
    by Dale Weber

    Set units = true for cm, and false for inches
*/
int readPING (byte sensorNr, bool units=true) {
  byte pin = sensorNr + PING_PIN_BASE;
  long duration;
  int result;

  /*
    The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  */
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  /*
    The same pin is used to read the signal from the PING))): a HIGH
    pulse whose duration is the time (in microseconds) from the sending
    of the ping to the reception of its echo off of an object.
  */
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  //  Convert the duration into a distance
  if (units) {
    //  Return result in cm
    result = microsecondsToCentimeters(duration);
  } else {
    //  Return result in inches.
    result = microsecondsToInches(duration);
  }

  delay(100);

  return result;
}
