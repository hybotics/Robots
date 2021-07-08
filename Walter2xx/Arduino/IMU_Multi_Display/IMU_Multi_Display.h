/*
  Program:      IMU_Multi_Display.ino - Inertial Measurement Unit testing, 
                  with multiple 7 segment display support.

  Date:         08-Jan-2014
  Version:      0.1.3 ALPHA

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
#ifndef	__IMU_MULTI_DISPLAY_H__
#define	__IMU_MULTI_DISPLAY_H__

#define COLOR_SENSOR_LED                4

#define	SPEAKER_OUT		        5
#define	HEARTBEAT_LED			13

#define	NUMBER_DISPLAYS			1
#define	SEVEN_SEG_BASE_ADDR		0x70

#define	MATRIX_DISPLAY_ADDR		SEVEN_SEG_BASE_ADDR + NUMBER_DISPLAYS

//  Sensor routine settings
#define  ANALOG_PIN_BASE      	      0
#define  DIGITAL_PIN_BASE     	      12

//  For SoftI2CMaster
#define	SOFT_I2C_SDA_PIN		2
#define	SOFT_I2C_SCL_PIN		3

#define	MAX_VOLTS				5.0
#define	MAX_STEPS				1024.0

#endif
