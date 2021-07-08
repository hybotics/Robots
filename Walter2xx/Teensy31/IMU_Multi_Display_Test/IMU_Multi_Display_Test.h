/*
	Program:	IMU_Multi_Display.ino - Inertial Measurement Unit testing,
				with multiple 7 segment display support.

	Date:		07-Feb-2014
	Version: 	0.1.6t ALPHA

	Purpose:	To allow experimentation and testing with various IMUs, including
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

	Dependencies:	Adafruit libraries:
				  LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307

			Hybotics libraries:
				  BMP180 (modified from Adafruit's BMP085 library)

	Comments:	Credit is given, where applicable, for code I did not originate.
				This sketch started out as an Adafruit tutorial for the electret
				microphones being used for sound detection. I've also pulled
				code for the GP2D12 IR and PING sensors from the Arduino
				Playground, which I have modified to suit my needs.

				Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/
#ifndef		__IMU_MULTI_DISPLAY_TEST_H__
#define	__IMU_MULTI_DISPLAY_TEST_H__

#define	HARDWARE_SERIAL_RX_PIN			0
#define	HARDWARE_SERIAL_TX_PIN			1

#define	SPEAKER_OUT					5
#define	HEARTBEAT_LED				13

#define	COLOR_SENSOR_LED				4

#define	MAX_NUMBER_7SEG_DISPLAYS			1
#define	SEVEN_SEG_BASE_ADDR			0x70

#define	MATRIX_DISPLAY_ADDR			SEVEN_SEG_BASE_ADDR + MAX_NUMBER_7SEG_DISPLAYS

/*
	These settings control whether standard information is displayed
		on the seven segment and matrix displays or not, and how
		often, in minutes.
*/
#define	DISPLAY_INFORMATION			true

#define	DISPLAY_DATE_FREQ_MIN			15
#define	DISPLAY_TIME_FREQ_MIN			15
#define	DISPLAY_TEMPERATURE_FREQ_MIN		15


//  For SoftI2CMaster
#define	SOFT_I2C_SDA_PIN				2
#define	SOFT_I2C_SCL_PIN				3

/*
	Sensor settings
*/
#define	IR_PIN_BASE					6			//	Analog 6
#define	PING_PIN_BASE					24

#define	MAX_NUMBER_PING				0

#define	PING_FRONT_CENTER				0
#define	PING_FRONT_LEFT				1
#define	PING_FRONT_RIGHT				2

#define	MAX_NUMBER_IR				1

#define	IR_FRONT_CENTER				0
#define	IR_BACK_CENTER				1
#define	IR_BACK_LEFT					2
#define	IR_BACK_RIGHT				3

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

#endif
