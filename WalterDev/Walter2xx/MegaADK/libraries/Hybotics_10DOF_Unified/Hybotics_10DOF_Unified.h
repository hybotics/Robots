/***************************************************************************
  This is a library for the Adafruit 10DOF Breakout

  Designed specifically to work with the Adafruit 10DOF Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __HYBOTICS_10DOF_UNIFIED_H__
#define __HYBOTICS_10DOF_UNIFIED_H__

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20.h>
#include <Hybotics_BMP180_Unified.h>
#include <Hybotics_LSM303DLHC_Unified.h>
#include <Wire.h>

/** Sensor axis */
typedef enum {
  SENSOR_AXIS_X  = (1),
  SENSOR_AXIS_Y  = (2),
  SENSOR_AXIS_Z  = (3)
} sensors_axis_t;

/* Driver for the the 10DOF breakout sensors */
class Hybotics_10DOF_Unified {
  public:
    Hybotics_10DOF_Unified(void);
    bool begin(void);

    bool  accelGetOrientation ( sensors_event_t *event, sensors_vec_t *orientation );
    bool  magTiltCompensation ( sensors_axis_t axis, sensors_event_t *mag_event, sensors_event_t *accel_event );
    bool  magGetOrientation   ( sensors_axis_t axis, sensors_event_t *event, sensors_vec_t *mag_orientation );

  private:
};

#endif
