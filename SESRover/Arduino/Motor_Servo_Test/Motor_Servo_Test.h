/*
	Program:      	SES Rover, Main navigation and reactive behaviors sketch
	Date:         	17-Apr-2014
	Version:      	0.0.3 ALPHA Lynxmotion BotBoarduino and SSC-32

	Platform:	Lynxmotion's BotBoarduino (Arduino),
				with Lynxmotion's SSC-32 Servo Controller,
				and a 3DOF (Raise/Lower, Wrist Rotate, Open/Close) Little Grip gripper

	Purpose:	To experiment with various sensor configurations, tracking objects (heat or
				color based), course following, manipulation of the environment, and to
				test code that will later be used on W.A.L.T.E.R. 2.0.

	Comments:	Credit is given, where applicable, for code I did not originate.

				Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/

#ifndef	__MOTOR_SERVO_TEST_H__
#define	__MOTOR_SERVO_TEST_H__

/*********************************************************/
/*	General settings 									 */
/*********************************************************/

#define	I2C_ADDRESS						0x50

#define	BUILD_VERSION					"0.0.3"
#define	BUILD_DATE 						"17-Apr-2014"
#define	BUILD_BOARD						"Lynxmotion's BotBoarduino with Lynxmotion's SSC-32"

#define	LOOP_DELAY_SECONDS				10

/*
	These settings control whether standard information is displayed
		on the seven segment and matrix displays or not, and how
		often, in minutes.
*/
#define	DISPLAY_INFORMATION				true

#define	DISPLAY_DATE_FREQ_MIN			15
#define	DISPLAY_TIME_FREQ_MIN			15
#define	DISPLAY_TEMPERATURE_FREQ_MIN	15

/*********************************************************
	Lynxmotion BotBoarduino (Arduino) Settings
*********************************************************/

/*
	Serial ports
*/

//	Hardware Serial: Console and Debug port
#define	SERIAL_CONSOLE_RX_PIN			0
#define	SERIAL_CONSOLE_TX_PIN			1

//	Software Serial: SSC-32 Servo Controller
#define	SERIAL_SSC32_RX_PIN				2
#define	SERIAL_SSC32_TX_PIN				3

//	Software Serial: XBee
#define	SERIAL_XBEE_RX_PIN				6
#define	SERIAL_XBEE_TX_PIN				7

//	Software Serial: RESERVED
#define	SERIAL_RESERVED_RX_PIN			8
#define	SERIAL_RESERVED_TX_PIN			9

#define	COLOR_SENSOR_LED				4
#define	SPEAKER_OUT						5
#define	HEARTBEAT_LED       	        13

/*
	Peripheral Settings - Displays, etc.
*/

//	Display constants
#define	MAX_NUMBER_7SEG_DISPLAYS		0
#define	SEVEN_SEG_BASE_ADDR				0x70

#define	MATRIX_DISPLAY_ADDR				SEVEN_SEG_BASE_ADDR + MAX_NUMBER_7SEG_DISPLAYS

/*
	Sensor settings
*/

#define	MAX_NUMBER_AREA_READINGS		9

//	Parallax PING Untrasonic sensors
#define	MAX_NUMBER_PING					1
#define	PING_PIN_BASE					10			//	Digital 10

#define	PING_FRONT_CENTER				0
#define	PING_FRONT_LEFT					1
#define	PING_FRONT_RIGHT				2

//	Sharp GP2Y0A21YK0F IR sensors
#define	MAX_NUMBER_IR					1
#define	IR_PIN_BASE						0			//	Analog 0

#define	IR_FRONT_CENTER					0
#define	IR_BACK_CENTER					1
#define	IR_BACK_LEFT					2
#define	IR_BACK_RIGHT					3

//	RoboClaw 2x5 Motor Controller Packet Serial constants
#define	ROBOCLAW_CONTROLLERS			0
#define	ROBOCLAW_SERIAL_BASE_ADDR		0x80

#define	ROBOCLAW_KP						1.0
#define	ROBOCLAW_KI						0.5
#define	ROBOCLAW_KD						0.25
#define	ROBOCLAW_QPPS					44000

/*********************************************************
	Lynxmotion SSC-32 Servo Controller Settings
*********************************************************/

#define	SERVO_MAX_DEGREES				90
#define	SERVO_CENTER_MS					1500

#define	SERVO_GRIP_LIFT_PIN				0
#define	SERVO_GRIP_LIFT_HOME			650
#define	SERVO_GRIP_LIFT_OFFSET			-90
#define	SERVO_GRIP_LIFT_MIN				500
#define	SERVO_GRIP_LIFT_MAX				2500

#define	SERVO_GRIP_WRIST_PIN			1
#define	SERVO_GRIP_WRIST_HOME			600
#define	SERVO_GRIP_WRIST_OFFSET			0
#define	SERVO_GRIP_WRIST_MIN			500
#define	SERVO_GRIP_WRIST_MAX			2500

#define	SERVO_GRIP_GRAB_PIN				2
#define	SERVO_GRIP_GRAB_HOME			2500
#define	SERVO_GRIP_GRAB_OFFSET			0
#define	SERVO_GRIP_GRAB_MIN				500
#define	SERVO_GRIP_GRAB_MAX				2500

/*
	There isn't anything on pin 3
*/

#define	SERVO_MOTOR_NEUTRAL				1500
#define	SERVO_MOTOR_MIN_SPEED			-1000
#define	SERVO_MOTOR_MAX_SPEED			1000

#define	SERVO_MOTOR_LEFT_PIN			4
#define	SERVO_MOTOR_LEFT_NEUTRAL		SERVO_MOTOR_NEUTRAL
#define	SERVO_MOTOR_LEFT_OFFSET	        0
#define	SERVO_MOTOR_LEFT_DIRECTION		false
#define	SERVO_MOTOR_LEFT_MIN			500
#define	SERVO_MOTOR_LEFT_MAX			2500

#define	SERVO_MOTOR_RIGHT_PIN	        5
#define	SERVO_MOTOR_RIGHT_NEUTRAL		SERVO_MOTOR_NEUTRAL
#define	SERVO_MOTOR_RIGHT_OFFSET		50
#define SERVO_MOTOR_RIGHT_DIRECTION		true
#define	SERVO_MOTOR_RIGHT_MIN			500
#define	SERVO_MOTOR_RIGHT_MAX			2500

#define	SERVO_PAN_PIN					6
#define	SERVO_PAN_HOME					SERVO_CENTER_MS
#define	SERVO_PAN_OFFSET				-50
#define	SERVO_PAN_LEFT_MIN				500
#define	SERVO_PAN_RIGHT_MAX				2500

#define	SERVO_TILT_PIN					7
#define	SERVO_TILT_HOME					SERVO_CENTER_MS
#define	SERVO_TILT_OFFSET				25
#define	SERVO_TILT_DOWN_MIN				500
#define	SERVO_TILT_UP_MAX				2000

/*********************************************************************/
/*	Structs for data we store about various onboard devices	*/
/*********************************************************************/

//	TC S34725 RGB Color Sensor
struct ColorSensor {
	uint16_t colorTempC;
	uint16_t lux;

	uint16_t red;
	uint16_t green;
	uint16_t blue;

	uint16_t c;
};

//	TMP006 Heat Sensor
struct HeatSensor {
	float dieTempC;
	float objectTempC;
};

//	For areaScan() readings
struct AreaDistanceReading {
	float ir;
	uint16_t ping;

	int positionDeg;
};

//	Continuous Rotation Servos - R/C PWM control mode parameters
struct ServoMotor {
	uint8_t pin;

	int offset;
	bool forward;
	uint16_t neutral;
	uint16_t minPulse;
	uint16_t maxPulse;

	uint16_t pulse;
	uint16_t error;
};

//	DC Motors - Packet Serial control mode parameters
struct Motor {
	uint32_t encoder;
	uint8_t encoderStatus;
	bool encoderValid;

	uint32_t speed;
	uint8_t speedStatus;
	bool speedValid;

	bool forward;

	long distance;
	bool distanceValid;

	uint16_t error;
};

//	Standard R/C Servos
struct Servo {
	uint8_t pin;

	int offset;
	uint16_t homePos;
	uint16_t msPulse;
	int angle;
	uint16_t minPulse;
	uint16_t maxPulse;
	uint8_t maxDegrees;

	uint16_t error;
};

enum MotorLocation {
	LEFT,
	RIGHT,
	FRONT,
	BACK
};

#endif
