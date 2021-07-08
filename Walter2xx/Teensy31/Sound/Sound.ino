/*
  Program:      W.A.L.T.E.R. 2.0, Sound Detection and reactive behaviors
  Date:         29-Dec-2013
  Version:      0.1.2 ALPHA

  Purpose:      Added new displaySoundDirection() routine. I've started to add some
                  error handling code, as well as code to handle Wire (I2C) slave
                  operation.
                  
                Connected BotBoarduino to SSC-32, and they communicate fine. Added a
                  nice little moveServoPw() and moveServoDegrees() routines that
                  include speed and time parameters. Added servo move routines,
                  sensor display routines, modified the servo structure to include
                  max degrees (90 or 180), so the appropriate value can ge stored
                  in servo.angle for each servo. The servo move routines support
                  combination moves.

  Dependencies: Adafruit libraries:
                  LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307  

                Hybotics libraries:
                  BMP180 (modified from Adafruit's BMP085 library)
                  
  Comments:     Credit is given, where applicable, for code I did not originate.
                  This sketch started out as an Adafruit tutorial for the electret
                  microphones being used for sound detection. I've also pulled
                  code for the GP2D12 IR and PING sensors from the Arduino
                  Playground, which I have modified to suit my needs.
 
                Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/

#include <Boards.h>
#include <Wire.h>

#include <RTClib.h>

#include "Sound.h"

/*
    Initialize global variables
*/

//  Storage for sound detection sample data
Sample samples[MAX_SAMPLES];

/****************************************************************
                          Code starts here
*****************************************************************/

/*
    Pulses a digital pin for a duration in ms
*/
void pulseDigital(int pin, int duration) {
  digitalWrite(pin, HIGH);   // Turn the ON by making the voltage HIGH (5V)
  delay(duration);           // Wait for duration ms
  digitalWrite(pin, LOW);    // Turn the pin OFF by making the voltage LOW (0V)
  delay(duration);           // Wait for duration ms
}

/*
    Return a string indicating the direction of a sound
*/
String directionString(byte dir) {
  String st;
  
  switch (dir) {
    case FRONT_LEFT_SIDE:
      st = String("Front Left");
      break;
      
    case FRONT_RIGHT_SIDE:
      st = String("Front Right");
      break;
      
    case BACK_LEFT_SIDE:
      st = String("Back Left");
      break;
      
    case BACK_RIGHT_SIDE:
      st = String("Back Right");
      break;
      
    default:
      st = String("Invalid");
      break;
  }

  return st;
}

/*
    Display the data for a given sound sample
*/
void displaySoundSample (byte channel, Sample *sample) {
  String st = directionString(channel);

  Serial.print(st);
  Serial.print(" Sample = ");
  Serial.print(sample->value);
  Serial.print(", Voltage = ");
  Serial.print(sample->volts);
  Serial.println(" volts");
}

/*
    Display the direction of a sound detection
*/
void displayDirection (byte dir) {
  String st = directionString(dir);

  if ((dir != 0) && (dir != NO_SOUND_DETECTED)) {
    Serial.print("Sound detected from the ");
    Serial.print(st);    
    Serial.println(".");
  } else if (dir == NO_SOUND_DETECTED) {
    Serial.println("No sound detected.");
  }
}

/*
    Get a sound sample from the given channel
*/
Sample soundSampleOf (byte channel) {
  Sample sample;
 
  sample.value = analogRead(channel);
  sample.volts = (sample.value * MAX_VOLTS) / MAX_STEPS;

  //  Initialize the rest of the sample
  sample.signalMin = 1024;
  sample.signalMax = 0;
  sample.signalMinVolts = 0.0;
  sample.signalMaxVolts = 0.0;
  sample.peakToPeakVolts = 0.0;

  return sample;
}

/*
    Get samples of sound from four microphones
*/
void getSoundSamples (Sample samples[MAX_SAMPLES]) {
  byte channel;
  Sample ts;

  //  Start of sample window
  unsigned long startMillis= millis();

  while ((millis() - startMillis) < sampleWindow) {
    /*
        Get and process raw samples from each microphone
    */
    for (channel = 0; channel < MAX_CHAN; channel++) {
      samples[channel] = soundSampleOf(channel);
      ts = samples[channel];

      if (ts.value < 1024) {
        //  Toss out spurious readings
        if (ts.value > ts.signalMax) {
          //  Save just the max levels
          ts.signalMax = ts.value;
          ts.signalMaxVolts = ts.volts;
        } else if (ts.value < ts.signalMin) {
          //  Save just the min levels
          ts.signalMin = ts.value;
          ts.signalMinVolts = ts.volts;
        }
        
        samples[channel] = ts;
      }
    }  
  } //  End of sample collection loop
  
  //  Calculate the peak to peak voltages
  for (channel = 0; channel < MAX_CHAN; channel++) {
    ts = samples[channel];
    ts.peakToPeakVolts = abs(ts.signalMaxVolts - ts.signalMinVolts); 

    samples[channel] = ts;
  }
}

/*
    Try to detect the loudest sound from one of four directions
*/
byte detectSound (void) {
  unsigned int sampleValue;
  Sample *ts;

  /*
      Variables for sound detection
  */
  double detectionFrontVolts = 0.0, detectionBackVolts = 0.0;
  byte detectionFront = 0;
  byte detectionBack = 0;
  byte detectionResult = 0;

  int displayPeakFrontLeft, displayPeakFrontRight;
  int displayPeakBackLeft, displayPeakBackRight; 

  /*
      Turn all the sound detection LEDs off
  */
  digitalWrite(FRONT_LEFT_LED, LOW);
  digitalWrite(FRONT_RIGHT_LED, LOW);
  digitalWrite(BACK_LEFT_LED, LOW);
  digitalWrite(BACK_RIGHT_LED, LOW);

  getSoundSamples(samples);

  /*
      Calculate the FRONT detection value
  */
  detectionFrontVolts = abs(samples[FRONT_LEFT_SIDE].peakToPeakVolts - samples[FRONT_RIGHT_SIDE].peakToPeakVolts);
//  Serial.print("Front Detection value = ");
//  Serial.println(detectionFrontVolts);

  /*
      Calculate the BACK detection value
  */
  detectionBackVolts = abs(samples[BACK_LEFT_SIDE].peakToPeakVolts - samples[BACK_RIGHT_SIDE].peakToPeakVolts);
//  Serial.print("Back Detection value = ");
//  Serial.println(detectionBackVolts);

  /*
      Get our final detection result
  */
  if ((detectionFrontVolts > detectionBackVolts) && (detectionFrontVolts > DETECTION_THRESHOLD)) {
    //  Check for sound detection
    if (samples[FRONT_LEFT_SIDE].peakToPeakVolts > samples[FRONT_RIGHT_SIDE].peakToPeakVolts) {
      digitalWrite(FRONT_LEFT_LED, HIGH);
      detectionFront = FRONT_LEFT_SIDE;
    } else if (samples[FRONT_RIGHT_SIDE].peakToPeakVolts > samples[FRONT_LEFT_SIDE].peakToPeakVolts) {
      digitalWrite(FRONT_RIGHT_LED, HIGH);
      detectionFront = FRONT_RIGHT_SIDE;
    } else {
      detectionFront = NO_SOUND_DETECTED;
    }
    
    detectionResult = detectionFront;
  } else if ((detectionBackVolts > detectionFrontVolts) && (detectionBackVolts > DETECTION_THRESHOLD)) {
    //  Check for sound detection
    if (samples[BACK_LEFT_SIDE].peakToPeakVolts > samples[BACK_RIGHT_SIDE].peakToPeakVolts) {
      digitalWrite(BACK_LEFT_LED, HIGH);
      detectionBack = BACK_LEFT_SIDE;
    } else if (samples[BACK_RIGHT_SIDE].peakToPeakVolts > samples[BACK_LEFT_SIDE].peakToPeakVolts) {
      digitalWrite(BACK_RIGHT_LED, HIGH);
      detectionBack = BACK_RIGHT_SIDE;
    } else {
      detectionBack = NO_SOUND_DETECTED;
    }

    detectionResult = detectionBack;
  }
  
  return detectionResult;
}

/*
    Called when a request from an I2C (Wire) Master comes in
*/
void wireRequestEvent (void) {
  
}

//  Called when the I2C (Wire) Slave receives data from an I2C (Wire) Master
void wireReceiveData (int nrBytesRead) {

}

/*
    Process error conditions
*/
void processError (byte err) {
  Serial.print("Error: ");
  Serial.print(err);
  Serial.println("");
}

/*
    Setup routine - runs just one time
*/
void setup (void) {
  //  Start up the Wire library as a slave device at address 0xE0
  Wire.begin(NAV_I2C_ADDRESS);

  //  Register event handler
  Wire.onRequest(wireRequestEvent);
  Wire.onReceive(wireReceiveData);

  //  Initialize the hardware serial port  
  Serial.begin(115200);
  
  //  Initialize the LED pin as an output.
  pinMode(HEARTBEAT_LED, OUTPUT);
  
  //  Set the LED pins to be outputs  
  pinMode(COLOR_SENSOR_LED, OUTPUT);
  pinMode(FRONT_LEFT_LED, OUTPUT);
  pinMode(FRONT_RIGHT_LED, OUTPUT);
  pinMode(BACK_LEFT_LED, OUTPUT);
  pinMode(BACK_RIGHT_LED, OUTPUT);
 
  //  Test the LEDs
  digitalWrite(FRONT_LEFT_LED, HIGH);
  digitalWrite(FRONT_RIGHT_LED, HIGH);
  digitalWrite(BACK_LEFT_LED, HIGH);
  digitalWrite(BACK_RIGHT_LED, HIGH);

  delay(1000);

  digitalWrite(FRONT_LEFT_LED, LOW);
  digitalWrite(FRONT_RIGHT_LED, LOW);
  digitalWrite(BACK_LEFT_LED, LOW);
  digitalWrite(BACK_RIGHT_LED, LOW);
  digitalWrite(COLOR_SENSOR_LED, LOW);
}

/*
    The loop routine runs forever
*/
void loop (void) {
  byte error = 0;
  byte directionOfSound = 0;

  // Pulse the heartbeat LED
  pulseDigital(HEARTBEAT_LED, 500);

  //  Do sound detection
  directionOfSound = detectSound();
  displayDirection(directionOfSound);

  if (error != 0) {
    processError(error);
  }
}
