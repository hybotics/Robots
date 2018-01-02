/*
  Program:      W.A.L.T.E.R. 2.0, Main navigation and reactive behaviors
  Date:         28-Dec-2013
  Version:      0.1.2 ALPHA

  Purpose:      Added new displaySoundDirection() routine. I've started to add some
                  error handling code, as well as code to handle Wire (I2C) slave
                  operation.

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
#ifndef	__SOUND_H__
#define	__SOUND_H__

#define	SOUND_I2C_ADDRESS		0x51

#define	SPEAKER_OUT				05
#define	HEARTBEAT_LED       	13

//  Sensor routine settings
#define	ANALOG_PIN_BASE			00
#define	DIGITAL_PIN_BASE		12

//  Maximum sound samples stored
#define	MAX_SAMPLES				4
#define	MAX_CHAN			MAX_SAMPLES

//  Sound detection threshold in volts
#define	DETECTION_THRESHOLD		0.20

//  Sound detection locations
#define	FRONT_LEFT_SIDE			0
#define	FRONT_RIGHT_SIDE		1
#define	BACK_LEFT_SIDE			2
#define	BACK_RIGHT_SIDE			3

#define	NO_SOUND_DETECTED		0xFF

//  Digital pins - we have to avoid pin 5 because the speaker is there
#define	COLOR_SENSOR_LED		02
#define	FRONT_LEFT_LED			03
#define	FRONT_RIGHT_LED			04
#define	BACK_LEFT_LED			06
#define	BACK_RIGHT_LED			07

//  For SoftI2CMaster
#define	SOFT_I2C_SDA_PIN		08
#define	SOFT_I2C_SCL_PIN		09

#define	MAX_VOLTS				5.0
#define	MAX_STEPS				1024.0

//  Display routine constants
const int maxScale = 8;
const int redZone = 5;

//  Sample window width in mS (50 mS = 20Hz)
const int sampleWindow = 50;

//  Data stored for each sound sample
struct Sample {
	int value;
	double volts;
	unsigned int signalMin;
	unsigned int signalMax;
	double signalMinVolts;
	double signalMaxVolts;
	double peakToPeakVolts;
};

#endif
