/*
	Program:      	W.A.L.T.E.R. 2.0, Main navigation, and reactive behaviors, header file
	Date:         	12-Feb-2014
	Version:      	0.2.5 Teensy 3.1 ALPHA

	Purpose:		Added definitions for SoftwareSerial ports for the SSC-32 and RoboClaw 2x5

					Added constants for RoboClaw motor controllers

					--------------------------------------------------------------------------
					v0.1.9 ALPHA:
					Starting migration from the Arduino (BotBoarduino) to the Arduino Mega ADK

					--------------------------------------------------------------------------
					v0.2.3 Teensy 3.1 ALPHA 09-Feb-2014:
					Beginning converstion to run on the Teensy 3.1 board:

					Set header definitions for the Teensy 3.1 hardware serial ports

					Removed all references to the BMSerial and RoboClaw libraries, because they just aren't
						compatible with the Teensy 3.1 right now. I am not sure how I want to or should proceed
						with this right now.

					Fixed a problem with the Adafruit_10DOF_Unified library where it was not able to find the
						Adafruit_BMP085_Unified.h file. I switched to using my version of this library
						(Adafruit_BMP180_Unified), and everything seems OK - more testing is needed. I should
						probably rename this library to Hybotics_BMP180_Unified to show there are differences
						from the Adafruit version.

					Added the LOOP_DELAY_SECONDS define for the time to delay at the end of the
						mail loop().

					---------------------------------------------------------------------------
					v0.2.4 ALPHA 11-Feb-2014:
					Made changes to adjust header to the Teensy 3.1 hardware

					This version builds cleanly for the Teensy 3.1 - testing begins.

					---------------------------------------------------------------------------
					v0.2.5 ALPHA 12-Feb-2014:
					Changed the COLOR_SENSOR_LED from pin 4 to pin 11. Pin 4 conflicted with the
						location of the PING sensors on pins 4 to 6.

					Added the build version, date, and board to the heading announcement.
					
					Starting to test Paul's fixes to the BMSerial and RoboClaw libraries.

					---------------------------------------------------------------------------

	Dependencies:	Adafruit libraries:
                  		LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307

					Hybotics libraries:
						BMP180 (modified from Adafruit's BMP085 library)

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2D12 IR and PING sensors from the Arduino
						Playground, which I have modified to suit my needs.

					Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/
#ifndef	__NAVIGATION4_DISPLAYS_H__
#define	__NAVIGATION4_DISPLAYS_H__

#define	NAV_I2C_ADDRESS					(0x50)

#define	BUILD_VERSION					"0.2.5"
#define BUILD_DATE 						"12-Feb-2014"
#define BUILD_BOARD						"Teensy 3.1"

#define	COLOR_SENSOR_LED				11

#define	SPEAKER_OUT						12
#define	HEARTBEAT_LED       	        13

//	Display constants
#define	MAX_NUMBER_7SEG_DISPLAYS		1
#define	SEVEN_SEG_BASE_ADDR				(0x70)

#define	MATRIX_DISPLAY_ADDR				SEVEN_SEG_BASE_ADDR + MAX_NUMBER_7SEG_DISPLAYS

/*
	These settings control whether standard information is displayed
		on the seven segment and matrix displays or not, and how
		often, in minutes.
*/
#define	DISPLAY_INFORMATION				true

#define	DISPLAY_DATE_FREQ_MIN			2
#define	DISPLAY_TIME_FREQ_MIN			2
#define	DISPLAY_TEMPERATURE_FREQ_MIN	2

#define LOOP_DELAY_SECONDS 				5

/*
	Sensor settings
*/
#define	IR_PIN_BASE						0			//	Analog 6
#define	PING_PIN_BASE					4

#define	MAX_NUMBER_PING					1

#define	PING_FRONT_CENTER				0
#define	PING_FRONT_LEFT					1
#define	PING_FRONT_RIGHT				2

#define	MAX_NUMBER_IR					1

#define	IR_FRONT_CENTER					0
#define	IR_BACK_CENTER					1
#define	IR_BACK_LEFT					2
#define	IR_BACK_RIGHT					3

//	RoboClaw 2x5 Motor Controller Packet Serial constants
#define	ROBOCLAW_CONTROLLERS			1
#define	ROBOCLAW_SERIAL_BASE_ADDR		(0x80)

#define	ROBOCLAW_KP						0x00010000
#define	ROBOCLAW_KI						0x00008000
#define	ROBOCLAW_KD						0x00004000
#define	ROBOCLAW_QPPS					44000

/*
	Hardware Serial ports
*/
//	HardwareSerial: Console and Debug port
#define	SERIAL_CONSOLE_RX_PIN			0
#define	SERIAL_CONSOLE_TX_PIN			1

//	HardwareSerial2: SSC-32 Servo Controller
#define	SERIAL_SSC32_RX_PIN				9
#define	SERIAL_SSC32_TX_PIN				10

//	HardwareSerial3: RoboClaw 2x5 Motor Controller
#define	SERIAL_ROBOCLAW_RX_PIN			7
#define	SERIAL_ROBOCLAW_TX_PIN			8

/*
	Software Serial ports
*/
//	XBee
#define	SERIAL_XBEE_RX_PIN				2
#define	SERIAL_XBEE_TX_PIN				3

/*
	The following settings apply to the SSC-32 servo controller
*/
#define	SERVO_MOTOR_LEFT_PIN			4
#define	SERVO_MOTOR_LEFT_ADJUST	        0
#define	SERVO_MOTOR_LEFT_MIN			1000
#define	SERVO_MOTOR_LEFT_MAX			2000

#define	SERVO_MOTOR_LEFT

#define	SERVO_MOTOR_RIGHT_PIN	        5
#define	SERVO_MOTOR_RIGHT_ADJUST        0
#define	SERVO_MOTOR_RIGHT_MIN			1000
#define	SERVO_MOTOR_RIGHT_MAX			2000

#define	SERVO_MOTOR_NEUTRAL				1500

#define	SERVO_MAX_DEGREES				90
#define	SERVO_CENTER_MS					1500

#define	SERVO_PAN_PIN					0
#define	SERVO_PAN_ADJUST				0
#define	SERVO_PAN_LEFT_MAX				500
#define	SERVO_PAN_RIGHT_MAX				2400

#define	SERVO_TILT_PIN					1
#define	SERVO_TILT_ADJUST				-150
#define	SERVO_TILT_DOWN_MAX				500
#define	SERVO_TILT_UP_MAX				2000
/*
	End of SSC-32 definitions
*/

struct ColorSensor {
	uint16_t colorTemp;
	uint16_t lux;

	uint16_t red;
	uint16_t green;
	uint16_t blue;

	uint16_t c;
};

struct HeatSensor {
	float dieTemp;
	float objectTemp;
};

struct Motor {
	uint8_t pin;

	uint16_t pulseWidthMin;
	uint16_t pulseWidthMax;
	uint16_t pulseWidthOffset;

	uint32_t encoder;
	uint8_t encoderStatus;
	bool encoderValid;

	uint32_t speed;
	uint8_t speedStatus;
	bool speedValid;

	bool forward;

	long distance;
	bool distanceValid;
};

struct Servo {
	uint8_t pin;

	int offset;
	int msPulse;
	int angle;
	int minPulse;
	int maxPulse;
	int maxDegrees;

	int error;
};

enum MotorLocation {
	LEFT,
	RIGHT,
	FRONT,
	BACK
};

#endif
