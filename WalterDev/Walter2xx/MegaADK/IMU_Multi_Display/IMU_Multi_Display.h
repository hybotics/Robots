/*
  Program:      IMU_Multi_Display.ino - Inertial Measurement Unit testing,
                  with multiple 7 segment display support.

  Date:         20-Jan-2014
  Version:      0.1.5 ALPHA

  Purpose:      To allow experimentation and testing with various IMUs, including
                  the Adafruit 10 DOF IMU with BMP180 temperature/preasure, LMS303DLHC
                  3-axis accelerometer/3-axis Magnetometer (compass), and L3GD20
                  3-axis Gyro that I'm tinkering with now.

                This sketch will support multiple 7 segment displays using I2C
                  backpacks ( http://www.adafruit.com/products/1427 ) from Adafruit.

                These are the 1.2" four digit 7 segment displays:
                  http://www.adafruit.com/products/878 (Red)
                  http://www.adafruit.com/products/879 (Yellow)
                  http://www.adafruit.com/products/880 (Green)
                  http://www.adafruit.com/products/881 (Blue)
                  http://www.adafruit.com/products/1002 (White)

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
#ifndef	__IMU_MULTI_DISPLAY_TEST_H__
#define	__IMU_MULTI_DISPLAY_TEST_H__

#define	HARDWARE_SERIAL_RX_PIN			0
#define	HARDWARE_SERIAL_TX_PIN			1

#define	SPEAKER_OUT						5
#define	HEARTBEAT_LED       	        13

#define COLOR_SENSOR_LED                53

#define	NUMBER_DISPLAYS			1
#define	SEVEN_SEG_BASE_ADDR		0x70

#define	MATRIX_DISPLAY_ADDR		SEVEN_SEG_BASE_ADDR + NUMBER_DISPLAYS

//  Sensor routine settings
#define  ANALOG_PIN_BASE      	      6
#define  DIGITAL_PIN_BASE     	      20

//  For SoftI2CMaster
#define	SOFT_I2C_SDA_PIN		2
#define	SOFT_I2C_SCL_PIN		3

#define	MAX_VOLTS				5.0
#define	MAX_STEPS				1024.0

/*
	Sensor settings
*/
#define	IR_PIN_BASE						6			//	Analog 6
#define	PING_PIN_BASE					24

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
#define	ROBOCLAW_SERIAL_BASE_ADDR		0x80

#define	ROBOCLAW_KP						0x00010000
#define	ROBOCLAW_KI						0x00008000
#define	ROBOCLAW_KD						0x00004000
#define	ROBOCLAW_QPPS					44000

//  For SoftI2CMaster
#define	SOFT_I2C_SDA_PIN				2
#define	SOFT_I2C_SCL_PIN				3

/*
	Hardware Serial ports
*/
//	Serial1: SSC-32 Servo Controller
#define	SERIAL_SSC32_RX_PIN				19
#define	SERIAL_SSC32_TX_PIN				18

//	Serial2: RoboClaw 2x5 Motor Controller
#define	SERIAL_ROBOCLAW_RX_PIN			17
#define	SERIAL_ROBOCLAW_TX_PIN			16

//	Serial3: XBee
#define	SERIAL_XBEE_RX_PIN				15
#define	SERIAL_XBEE_TX_PIN				14

/*
	The following settings apply to the SSC-32 servo controller
*/
#define	SERVO_MOTOR_LEFT_PIN			4
#define	SERVO_MOTOR_LEFT_ADJUST	        0
#define	SERVO_MOTOR_LEFT_MIN			1000
#define	SERVO_MOTOR_LEFT_MAX			2000

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

//	Display constants
#define	SEVEN_SEG_NR_DISPLAYS			1
#define	SEVEN_SEG_BASE_ADDR				0x70

#define	MATRIX_DISPLAY_ADDR				SEVEN_SEG_BASE_ADDR + SEVEN_SEG_NR_DISPLAYS

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
	uint16_t pulseWidthAdjust;

	uint32_t encoder;
	uint32_t speed;
	bool forward;

	long distance;
	long ticks;
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
