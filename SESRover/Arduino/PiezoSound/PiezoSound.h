/*
	Program:   SES Rover, PiezoSound header file
	Date:      23-Apr-2014
	Version:   0.0.1 ALPHA

	Platform:	Arduino Mega 2560 R3,
					Lynxmotion's SSC-32 Servo Controller,
					and a 3DOF (Raise/Lower, Wrist Rotate, Open/Close) Little Grip gripper

	Purpose:	To experiment with various sensor configurations, tracking objects (heat or
					color based), course following, manipulation of the environment, and to
					test code that will later be used on W.A.L.T.E.R. 2.0.

	Comments:	Credit is given, where applicable, for code I did not originate.

				Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/

#ifndef	__PIEZOSOUND_H__
#define	__PIEZOSOUND_H__

/*********************************************************/
/*	General settings 									 */
/*********************************************************/

#define	I2C_ADDRESS						0x50

#define	BUILD_VERSION					"0.0.1"
#define	BUILD_DATE 						"24-Apr-2014"
#define	BUILD_BOARD						"Arduino Duemilonve"

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

/*
	Optional Sensors and Peripherals
*/
#define HAVE_COLOR_SENSOR				false
#define HAVE_HEAT_SENSOR				false
#define HAVE_DS1307_RTC					false

#define HAVE_10DOF_IMU					false

//	NOTE: These three are all contained on the 10DOF IMU board
#define	HAVE_LSM303DLHC_ACCEL			false
#define	HAVE_L3GD20_GYRO				false
#define	HAVE_BMP180_TEMP				false

#define HAVE_7SEGMENT_DISPLAYS			false

#define	DOING_MOTOR_CALIBRATION			true

/*********************************************************
	Arduino Mega R3 (Arduino) Settings
*********************************************************/

/*
	Serial ports
*/

//	Hardware Serial: Console and Debug port
#define	SERIAL_CONSOLE_RX_PIN			0
#define	SERIAL_CONSOLE_TX_PIN			1

//	Software Serial: Lynxmotion SSC-32 Servo Controller
#define	SERIAL_SSC32_RX_PIN				3
#define	SERIAL_SSC32_TX_PIN				2

//	Software Serial: XBee (ZigBee) Mesh Wireless adapter
#define	SERIAL_XBEE_RX_PIN				5
#define	SERIAL_XBEE_TX_PIN				4

//	Software Serial: RESERVED
#define	SERIAL_RESERVED_RX_PIN			7
#define	SERIAL_RESERVED_TX_PIN			6

//	Software Serial: RoboClaw 2x5 Motor Controllers
#define	SERIAL_ROBOCLAW_RX_PIN			21
#define	SERIAL_ROBOCLAW_TX_PIN			20

/*
	Peripheral Settings, for Displays, Sound, etc.
*/

//	Display constants
#define	MAX_NUMBER_7SEG_DISPLAYS		0
#define	SEVEN_SEG_BASE_ADDR				0x70

#define	MATRIX_DISPLAY_ADDR				SEVEN_SEG_BASE_ADDR + MAX_NUMBER_7SEG_DISPLAYS

/*
	Other Resources
*/
#define	SPEAKER_OUT						11 			//	Digital 9 - 11, have PWM capability
#define	COLOR_SENSOR_LED				12
#define	HEARTBEAT_LED       	        13

/*
	Sensor settings
*/
#define	MAX_NUMBER_AREA_READINGS		36

//	Parallax PING Untrasonic sensors
#define	MAX_NUMBER_PING					1
#define	PING_PIN_BASE					24			//	Digital 24
#define	PING_MIN_DISTANCE_CM			10.0 		//	In CM, which is approximately 4"

#define	PING_FRONT_CENTER				0
#define	PING_FRONT_LEFT					1
#define	PING_FRONT_RIGHT				2

//	Sharp GP2Y0A21YK0F IR sensors
#define	MAX_NUMBER_IR					1
#define	IR_PIN_BASE						6			//	Analog 6
#define IR_MIN_DISTANCE_CM				14.0 		//	In CM, which is approximately 5"

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

#define ROBOCLAW_MOTOR_LEFT_NAME		"Left Gear Motor"
#define ROBOCLAW_MOTOR_RIGHT_NAME		"Right Gear Motor"

/*********************************************************
	Sound generation constants
*********************************************************/

//	These are to support the playMelody() routine, as it is now.
#define TONE_NAMES						{ 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' }
#define TONE_FREQUENCIES				{ 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956 }
#define MAX_COUNT						24

/*********************************************************
	Lynxmotion SSC-32 Servo Controller Settings
*********************************************************/

#define	SERVO_MAX_DEGREES				90
#define	SERVO_CENTER_MS					1500

#define	SERVO_GRIP_LIFT_PIN				0
#define SERVO_GRIP_LIFT_NAME			"Grip Lift"
#define	SERVO_GRIP_LIFT_HOME			900
#define	SERVO_GRIP_LIFT_OFFSET			0
#define	SERVO_GRIP_LIFT_MIN				500
#define	SERVO_GRIP_LIFT_MAX				2500

#define	SERVO_GRIP_WRIST_PIN			1
#define SERVO_GRIP_WRIST_NAME			"Grip Wrist"
#define	SERVO_GRIP_WRIST_HOME			600
#define	SERVO_GRIP_WRIST_OFFSET			0
#define	SERVO_GRIP_WRIST_MIN			500
#define	SERVO_GRIP_WRIST_MAX			2500

#define	SERVO_GRIP_GRAB_PIN				2
#define SERVO_GRIP_GRAB_NAME			"Grip Grab"
#define	SERVO_GRIP_GRAB_HOME			2500
#define	SERVO_GRIP_GRAB_OFFSET			0
#define	SERVO_GRIP_GRAB_MIN				500
#define	SERVO_GRIP_GRAB_MAX				2500

#define	SERVO_PAN_PIN					6
#define SERVO_PAN_NAME					"Pan"
#define	SERVO_PAN_HOME					SERVO_CENTER_MS
#define	SERVO_PAN_OFFSET				-50
#define	SERVO_PAN_LEFT_MIN				500
#define	SERVO_PAN_RIGHT_MAX				2500

#define	SERVO_TILT_PIN					7
#define SERVO_TILT_NAME					"Tilt"
#define	SERVO_TILT_HOME					1400
#define	SERVO_TILT_OFFSET				0
#define	SERVO_TILT_DOWN_MIN				500
#define	SERVO_TILT_UP_MAX				2000

/*
	There isn't anything on pin 3
*/

#define	SERVO_MOTOR_MIN_SPEED			-1000
#define	SERVO_MOTOR_MAX_SPEED			1000

//	Servo Motor gears (speeds)
#define	SERVO_MOTOR_NEUTRAL				0

#define	SERVO_MOTOR_GEAR_01				50
#define	SERVO_MOTOR_GEAR_02				100
#define	SERVO_MOTOR_GEAR_03				200
#define	SERVO_MOTOR_GEAR_04				300
#define	SERVO_MOTOR_GEAR_05				400
#define	SERVO_MOTOR_GEAR_06				500
#define	SERVO_MOTOR_GEAR_07				600
#define	SERVO_MOTOR_GEAR_08				700
#define	SERVO_MOTOR_GEAR_09				800
#define	SERVO_MOTOR_GEAR_10				975

#define	SERVO_MOTOR_REVERSE_01			-50
#define	SERVO_MOTOR_REVERSE_02			-100
#define	SERVO_MOTOR_REVERSE_03			-200
#define	SERVO_MOTOR_REVERSE_04			-300
#define	SERVO_MOTOR_REVERSE_05			-400
#define	SERVO_MOTOR_REVERSE_06			-500
#define	SERVO_MOTOR_REVERSE_07			-600
#define	SERVO_MOTOR_REVERSE_08			-700
#define	SERVO_MOTOR_REVERSE_09			-800
#define	SERVO_MOTOR_REVERSE_10			-975

//	Left Servo Motor assignments
#define	SERVO_MOTOR_LEFT_PIN			4
#define SERVO_MOTOR_LEFT_NAME			"Left Servo Motor"
#define	SERVO_MOTOR_LEFT_NEUTRAL		SERVO_MOTOR_NEUTRAL

#define	SERVO_MOTOR_LEFT_OFFSET	        0
#define	SERVO_MOTOR_LEFT_SPEED_ADJ		10
#define	SERVO_MOTOR_LEFT_DIRECTION		false
#define	SERVO_MOTOR_LEFT_MIN			500
#define	SERVO_MOTOR_LEFT_MAX			2500

//	Right Servo Motor assignments
#define	SERVO_MOTOR_RIGHT_PIN	        5
#define SERVO_MOTOR_RIGHT_NAME			"Right Servo Motor"
#define	SERVO_MOTOR_RIGHT_NEUTRAL		SERVO_MOTOR_NEUTRAL

#define	SERVO_MOTOR_RIGHT_OFFSET		25
#define	SERVO_MOTOR_RIGHT_SPEED_ADJ		0
#define SERVO_MOTOR_RIGHT_DIRECTION		true
#define	SERVO_MOTOR_RIGHT_MIN			500
#define	SERVO_MOTOR_RIGHT_MAX			2500

/**********************************************************************
	Structs for data we store about various onboard devices
/*********************************************************************/

#endif
