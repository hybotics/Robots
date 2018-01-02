/* LSM303DLH Example Code
 
   LSM303 Breakout ---------- Arduino
         Vin                   5V
         GND                   GND
         SDA                   A4
         SCL                   A5
*/
#include <Wire.h>
#include <math.h>
 
#define SCALE 2  // accel full-scale, should be 2, 4, or 8
 
/* LSM303 Address definitions */
#define LSM303_MAG  0x1E                                  //  Assuming SA0 grounded
#define LSM303_ACC  0x18                                  //  Assuming SA0 grounded
 
#define X 0
#define Y 1
#define Z 2
 
/* LSM303 Register definitions */
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define HP_FILTER_RESET_A 0x25
#define REFERENCE_A 0x26
#define STATUS_REG_A 0x27
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define INT1_CFG_A 0x30
#define INT1_SOURCE_A 0x31
#define INT1_THS_A 0x32
#define INT1_DURATION_A 0x33
#define CRA_REG_M 0x00
#define CRB_REG_M 0x01
#define MR_REG_M 0x02
#define OUT_X_H_M 0x03
#define OUT_X_L_M 0x04
#define OUT_Y_H_M 0x05
#define OUT_Y_L_M 0x06
#define OUT_Z_H_M 0x07
#define OUT_Z_L_M 0x08
#define SR_REG_M 0x09
#define IRA_REG_M 0x0A
#define IRB_REG_M 0x0B
#define IRC_REG_M 0x0C
 
/* Global variables */
int accel[3];                                             //  We'll store the raw acceleration values here
int mag[3];                                               //  Raw magnetometer values stored here
float realAccel[3];                                       //  Calculated acceleration values here
 
void initLSM303(int fs) {
  LSM303_write(0x27, CTRL_REG1_A);                        //  0x27 = normal power mode, all accel axes on

  if ((fs==8)||(fs==4)) { 
    LSM303_write((0x00 | (fs-fs/2-1)<<4), CTRL_REG4_A);   //  Set full-scale
  } else {
    LSM303_write(0x00, CTRL_REG4_A);
  }

  LSM303_write(0x14, CRA_REG_M);                          //  0x14 = mag 30Hz output rate
  LSM303_write(0x00, MR_REG_M);                           //  0x00 = continouous conversion mode
}
 
void printValues(int * magArray, int * accelArray) {
  //  Print out mag and accel arrays all pretty-like
  Serial.print(accelArray[X], DEC);
  Serial.print("\t");
  Serial.print(accelArray[Y], DEC);
  Serial.print("\t");
  Serial.print(accelArray[Z], DEC);
  Serial.print("\t\t");
   
  Serial.print(magArray[X], DEC);
  Serial.print("\t");
  Serial.print(magArray[Y], DEC);
  Serial.print("\t");
  Serial.print(magArray[Z], DEC);
  Serial.println();
}
 
float getHeading(int * magValue) {
  //  See section 1.2 in app note AN3192
  float heading = 180*atan2(magValue[Y], magValue[X])/PI; //  Assume pitch, roll are 0
   
  if (heading <0) {
    heading += 360;
  }

  return heading;
}
 
float getTiltHeading(int * magValue, float * accelValue) {
  //  See appendix A in app note AN3192
  float pitch = asin(-accelValue[X]);
  float roll = asin(accelValue[Y]/cos(pitch));
   
  float xh = magValue[X] * cos(pitch) + magValue[Z] * sin(pitch);
  float yh = magValue[X] * sin(roll) * sin(pitch) + magValue[Y] * cos(roll) - magValue[Z] * sin(roll) * cos(pitch);
  float zh = -magValue[X] * cos(roll) * sin(pitch) + magValue[Y] * sin(roll) + magValue[Z] * cos(roll) * cos(pitch);
 
  float heading = 180 * atan2(yh, xh)/PI;
  if (yh >= 0)
    return heading;
  else
    return (360 + heading);
}
 
void getLSM303_mag(int * rawValues) {
  Wire.beginTransmission(LSM303_MAG);
  Wire.send(OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM303_MAG, 6);

  for (int i=0; i<3; i++) {
    rawValues[i] = (Wire.receive() << 8) | Wire.receive();
  }
}
 
void getLSM303_accel(int * rawValues) {
  rawValues[Z] = ((int)LSM303_read(OUT_X_L_A) << 8) | (LSM303_read(OUT_X_H_A));
  rawValues[X] = ((int)LSM303_read(OUT_Y_L_A) << 8) | (LSM303_read(OUT_Y_H_A));
  rawValues[Y] = ((int)LSM303_read(OUT_Z_L_A) << 8) | (LSM303_read(OUT_Z_H_A)); 
  //  Had to swap those to right the data with the proper axis
}
 
byte LSM303_read(byte address) {
  byte temp;
   
  if (address >= 0x20) {
    Wire.beginTransmission(LSM303_ACC);
  } else {
    Wire.beginTransmission(LSM303_MAG);
  }

  Wire.send(address);
   
  if (address >= 0x20) {
    Wire.requestFrom(LSM303_ACC, 1);
  } else {
    Wire.requestFrom(LSM303_MAG, 1);
  }

  while(!Wire.available())
    ;
  
  temp = Wire.receive();
  Wire.endTransmission();
   
  return temp;
}
 
void LSM303_write(byte data, byte address) {
  if (address >= 0x20) {
    Wire.beginTransmission(LSM303_ACC);
  } else {
    Wire.beginTransmission(LSM303_MAG);
  }   

  Wire.send(address);
  Wire.send(data);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);                                     //  Serial is used for debugging
  Wire.begin();                                           //  Start up I2C, required for LSM303 communication
  initLSM303(SCALE);                                      //  Initialize the LSM303, using a SCALE full-scale range
}
 
void loop() {
  getLSM303_accel(accel);                                 //  Get the acceleration values and store them in the accel array
  while(!(LSM303_read(SR_REG_M) & 0x01))
    ;                                                     //  Wait for the magnetometer readings to be ready

  getLSM303_mag(mag);                                     //  Get the magnetometer values, store them in mag

  //  PrintValues(mag, accel);                            //  Print the raw accel and mag values, good debugging
   
  for (int i=0; i<3; i++) {
    realAccel[i] = accel[i] / pow(2, 15) * SCALE;         //  Calculate real acceleration values, in units of g
  }

  //  Print both the level, and tilt-compensated headings below to compare
  Serial.print(getHeading(mag), 3);  // this only works if the sensor is level
  Serial.print("\t\t");  // print some tabs
  Serial.println(getTiltHeading(mag, realAccel), 3);      //  See how awesome tilt compensation is?!
  delay(100);                                             //  Delay for serial readability
}
