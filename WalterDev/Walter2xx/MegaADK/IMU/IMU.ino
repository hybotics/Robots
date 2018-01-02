/*
  Program:      IMU.ino - Inertial Measurement Unit testing
  Date:         08-Jan-2014
  Version:      0.1.2 ALPHA

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
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP180_Unified.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <RTClib.h>

#include "IMU.h"

/*************************************************** 
  This is a library for our I2C LED Backpacks

  Designed specifically to work with the Adafruit LED 7-Segment backpacks 
  ----> http://www.adafruit.com/products/881
  ----> http://www.adafruit.com/products/880
  ----> http://www.adafruit.com/products/879
  ----> http://www.adafruit.com/products/878

  These displays use I2C to communicate, 2 pins are required to 
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/****************************************************
  This sketch uses the 10DOF IMU from Adafruit, which has a
    BMP180 Temperature/Barometric pressure sensor, a
    LMS303 Three-axis accelerometer, and a
    L3GD20 Gyro
    
    Adafruit product http://www.adafruit.com/products/1604
*****************************************************/

// Enable one of these two #includes and comment out the other.
// Conditional #include doesn't work due to Arduino IDE shenanigans.
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
//#include <TinyWireM.h> // Enable this line if using Adafruit Trinket, Gemma, etc.

Adafruit_8x8matrix matrix8x8 = Adafruit_8x8matrix();

Adafruit_BMP180_Unified bmp180 = Adafruit_BMP180_Unified(10085);
RTC_DS1307 rtc;

/*
    Global variables
*/

//  Support for multiple 7 segment displays
Adafruit_7segment sevenSeg[NUMBER_DISPLAYS];

boolean displayDate = true;
uint8_t dateDisplayFreq = 15;              //  How often to display the date, in minutes
uint8_t minuteCount = 0;                   //  Count the minutes

static const uint8_t PROGMEM
  hpa_bmp[] = {
    B10001110,
    B10001001,
    B11101110,
    B10101000,
    B00000100,
    B00001010,
    B00011111,
    B00010001
  },
    
  c_bmp[] = {
    B01110000,
    B10001000,
    B10000000,
    B10001000,
    B01110000,
    B00000000,
    B00000000,
    B00000000
  },
    
  f_bmp[] = {
    B11111000,
    B10000000,
    B11100000,
    B10000000,
    B10000000,
    B00000000,
    B00000000,
    B00000000
  },
    
  m_bmp[] = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B11101110,
    B10111010,
    B10010010,
    B10000010
  },

  date_bmp[] = {
    B10110110,
    B01001001,
    B01001001,
    B00000100,
    B00000100,
    B01111100,
    B10000100,
    B01111100
  },
  
  year_bmp[] = {
    B00000000,
    B10001000,
    B10001000,
    B01110000,
    B00101011,
    B00101100,
    B00101000,
    B00000000
  },
  
  am_bmp[] = {
    B01110000,
    B10001010,
    B10001010,
    B01110100,
    B00110110,
    B01001001,
    B01001001,
    B01001001
  },
 
  pm_bmp[] = {
    B01111100,
    B10000010,
    B11111100,
    B10000000,
    B10110110,
    B01001001,
    B01001001,
    B01001001
  },
  
  allon_bmp[] = {
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111
  };

/*
    Left zero pad a numeric string
*/
String leftZeroPadString (String st, uint8_t nrPlaces) {
  uint8_t i, len;
  String newStr = st;
  
  if (newStr.length() < nrPlaces) {
    len = st.length();
  
    for (i = len; i < nrPlaces; i++) {
      newStr = String("0" + newStr);
    }
  }

  return newStr;
}

/*
    Trim trailing zeros from a numeric string
*/
String trimTrailingZeros (String st) {
  uint8_t newStrLen = 0;
  String newStr = st;

  newStrLen = newStr.length();

  while (newStr.substring(newStrLen - 1) == "0") {
    newStrLen -= 1;
    newStr = newStr.substring(0, newStrLen);
  }

  return newStr;
}
  
/*
	Write a floating point value to the 7-Segment display
*/

/*
boolean writeFloat (double value, uint8_t decimal = 2, uint8_t nrDisplayDigits = 4, boolean noblank = false) {
  boolean exitStatus = true;

  long integerPart = 0, decimalPart = 0;
  double sign = 1.0, newValue = 0.0;
  uint8_t digitCount = 1, temp = 0, dotPosition = 0, totalDigits = 0;
  uint8_t intPartLen = 0, decPartLen = 0, valueLen = 0, nrDigits = 0; 
  boolean decimalPoint = false;
  String decPartStr, intPartStr, valueStr;
  
  //  Store the sign
  if (value < 0) {
    sign = -1.0;
  } else {
    sign = 1.0;
  }

  newValue = abs(value);

  valueStr = String(newValue, DEC);  
  valueLen = valueStr.length();
  dotPosition = valueStr.indexOf(".");

  intPartStr = valueStr.substring(0, dotPosition);
  intPartLen = intPartStr.length();
//  integerPart = int(value);

  decPartStr = trimTrailingZeros(valueStr.substring(dotPosition + 1));
  decPartLen = decPartStr.length();
//  decimalPart = value - integerPart;
  
  Serial.print("(writeFloat) Integer part string = '");
  Serial.print(intPartStr);
  Serial.print("' (");
  Serial.print(intPartLen);
  Serial.println(")");

  Serial.print("Decimal part string = '");
  Serial.print(decPartStr);
  Serial.print("' (");
  Serial.print(decPartLen);
  Serial.println(")");

  Serial.print("sign = ");
  Serial.println(sign);

  Serial.print("(writeFloat) value string = '");
  Serial.print(valueStr);
  Serial.print("' (");
  Serial.print(valueLen);
  Serial.println(")");

  //  Find out how many digits we have to display
  totalDigits = intPartLen + decPartLen;
  
  if (sign < 0) {
    totalDigits += 1;
  }

  //  Check to be sure we can display the entire value
  if ((totalDigits > nrDisplayDigits) || (totalDigits > (NUMBER_DISPLAYS * 4))) {
    exitStatus = false;
  }

  if (exitStatus) {
    temp = value / 100;
 
    Serial.print("(writeNumber) value = ");
    Serial.print(value);
    Serial.print(", temp = ");
    Serial.println(temp);

    //	Set first digit of the integer portion
    if ((noblank) or (temp > 9)) {
      decimalPoint = ((digitCount) == decimal);
      sevenSeg.writeDigitNum(0, int(temp / 10), decimalPoint);  //  Tens
    } else {
      sevenSeg.clear();
    }

    //	Set the second digit of the integer portion
    digitCount += 1;
    decimalPoint = ((digitCount) == decimal);
    sevenSeg.writeDigitNum(1, temp % 10, decimalPoint);         //  Ones

    //	Set the first digit of the decimal portion
    temp = value % 100;
    digitCount += 1;
    decimalPoint = ((digitCount) == decimal);
    sevenSeg.writeDigitNum(3, int(temp / 10), decimalPoint);    //  Tens

    //	Set the second digit of the decimal portion
    digitCount += 1;
    decimalPoint = ((digitCount) == decimal);
    sevenSeg.writeDigitNum(4, temp % 10, decimalPoint);         //  Ones
  }
  
  return exitStatus;
}
*/

/*
	Write a number (integer or floating point) to the 7-Segment display
*/
void writeNumber (uint16_t value, uint8_t decimal = 2, boolean noblank = false) {
  uint8_t digitCount = 1, temp = 0;
  boolean decimalPoint = false;

  temp = value / 100;
/*  
  Serial.print("(writeNumber) value = ");
  Serial.print(value);
  Serial.print(", temp = ");
  Serial.println(temp);
*/

  //	Set first digit of the integer portion
  if ((noblank) or (temp > 9)) {
/*    
    Serial.print("(writeNumber) digit = ");
    Serial.println(digit);
*/

    decimalPoint = ((digitCount) == decimal);
    sevenSeg[0].writeDigitNum(0, int(temp / 10), decimalPoint);  //  Tens
  } else {
    sevenSeg[0].clear();
  }

  //	Set the second digit of the integer portion
  digitCount += 1;
  decimalPoint = ((digitCount) == decimal);
  sevenSeg[0].writeDigitNum(1, temp % 10, decimalPoint);         //  Ones

  //	Set the first digit of the decimal portion
  temp = value % 100;
  digitCount += 1;
  decimalPoint = ((digitCount) == decimal);
  sevenSeg[0].writeDigitNum(3, int(temp / 10), decimalPoint);    //  Tens

  //	Set the second digit of the decimal portion
  digitCount += 1;
  decimalPoint = ((digitCount) == decimal);
  sevenSeg[0].writeDigitNum(4, temp % 10, decimalPoint);         //  Ones
}

float tempFahrenheit (float celsius) {
  return (celsius * 1.8) + 32;
}

void setup() {
  uint8_t nrDisp;
  
  //  Initialize serial port communication
  Serial.begin(115200);
  Serial.println("IMU Time/Temperature Test");

  /*
      Multiple 7 segment displays will be supported.
  */

  //  Initialize the 7-Segment display(s)
  for (nrDisp = 0; nrDisp < NUMBER_DISPLAYS; nrDisp++) {
    sevenSeg[nrDisp] = Adafruit_7segment();
    sevenSeg[nrDisp].begin(SEVEN_SEG_ADDR_BASE + nrDisp);
    sevenSeg[nrDisp].setBrightness(1);
    sevenSeg[nrDisp].drawColon(false);
  }
/*  
  sevenSeg.begin(SEVEN_SEG_ADDR_BASE);
  sevenSeg.setBrightness(1);
  sevenSeg.drawColon(false);
*/  
  matrix8x8.begin(MATRIX_DISPLAY_ADDR);
  matrix8x8.setBrightness(1);
  matrix8x8.setRotation(3);

  Serial.println("Testing all displays..");

  //  Test all the displays
  for (nrDisp = 0; nrDisp < NUMBER_DISPLAYS; nrDisp++) {
    sevenSeg[nrDisp].print(8888);
    sevenSeg[nrDisp].drawColon(true);
  }
/*  
  sevenSeg.print(8888);
  sevenSeg.drawColon(true);
*/
  matrix8x8.drawBitmap(0, 0, allon_bmp, 8, 8, LED_ON);
  sevenSeg[0].writeDisplay();
  matrix8x8.writeDisplay();
 
  delay(2000);

  //  Initialize the BMP180 sensor
  if(!bmp180.begin()) {
    //  There was a problem detecting the BMP180 ... check your connections
    Serial.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

/*  
  writeFloat(163.90127, 2, 4, false);
  writeFloat(-23.0159, 2, 4, false);
*/
}

void loop() {
  boolean amTime;
  uint8_t displayNr = 0;
  DateTime now = rtc.now();
  float celsius, fahrenheit, altitude;
  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  sensors_event_t bmp180event;
  
  uint8_t hour = now.hour(), nrDisplays = 0;
  uint16_t displayInt;

  String displayString;
  String timeString;
  String currMonth = leftZeroPadString(String(now.month()), 2);
  String currDay = leftZeroPadString(String(now.day()), 2);
  String currYear = leftZeroPadString(String(now.year()), 4);
  String currMinute = leftZeroPadString(String(now.minute()), 2);
  String currSecond = leftZeroPadString(String(now.second()), 2);
  
  //  Clear the displays
  for (displayNr = 0; displayNr < NUMBER_DISPLAYS; displayNr ++) {
    sevenSeg[displayNr].clear();
    sevenSeg[displayNr].drawColon(false);
  }

  matrix8x8.clear();

/*
  Serial.print("Month = ");
  Serial.print(now.month());
  Serial.print(", Day = ");
  Serial.print(now.day());
  Serial.print(", Year = ");
  Serial.println(now.year());
*/

/*  
  Serial.print("displayInt = ");
  Serial.println(displayInt);
*/  

  //  Display the date, if it's time
  if (displayDate) {
    displayString = String(currMonth + currDay);

    displayInt = (now.month() * 100) + now.day();  

    //  Month and day
    writeNumber(displayInt, 0, true);
    matrix8x8.drawBitmap(0, 0, date_bmp, 8, 8, LED_ON);

    sevenSeg[0].writeDisplay();
    matrix8x8.writeDisplay();

    delay(5000);

    sevenSeg[0].clear();
    matrix8x8.clear();  

    //  Year
    writeNumber(now.year(), 0, false);
    matrix8x8.drawBitmap(0, 0, year_bmp, 8, 8, LED_ON);

    sevenSeg[0].writeDisplay();
    matrix8x8.writeDisplay();

    delay(5000);
    
    minuteCount = 0;
  }
  
  sevenSeg[0].clear();
  matrix8x8.clear();  

  if (hour > 12) {
    amTime = false;
    hour = hour - 12;
  } else {
    amTime = true;
  }
  
  displayInt = (hour * 100) + now.minute();  
  timeString = leftZeroPadString(String((hour * 100) + now.minute()), 4);

  //  Display the current time on the 7 segment display
  writeNumber(displayInt, 0, false);
  sevenSeg[0].drawColon(true);
  
  matrix8x8.clear();
  
  if (amTime) {
    matrix8x8.drawBitmap(0, 0, am_bmp, 8, 8, LED_ON);
  } else {
    matrix8x8.drawBitmap(0, 0, pm_bmp, 8, 8, LED_ON);
  }
  
  sevenSeg[0].writeDisplay();
  matrix8x8.writeDisplay();
  
  delay(45000);

  sevenSeg[0].drawColon(false);

/*
  //  Display the current time
  Serial.print("Current date: ");
  Serial.print(currMonth);
  Serial.print('/');
  Serial.print(currDay);
  Serial.print('/');
  Serial.println(currYear);
  
  Serial.print("Current time: (");
  Serial.print(timeString);
  Serial.print(") ");
  Serial.print(currHour);
  Serial.print(':');
  Serial.print(currMinute);
  Serial.print(':');
  Serial.print(currSecond);
  Serial.print(" ");
  Serial.println(amPM);
  Serial.println();
*/

  /* Get a new sensor event */ 
  bmp180.getEvent(&bmp180event);
  
  //  Display the barometric pressure in hPa
  if (bmp180event.pressure) {
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */

    //  First we get the current temperature from the BMP180 in celsius and fahrenheit
    bmp180.getTemperature(&celsius);
    fahrenheit = tempFahrenheit(celsius);

    //   Convert the atmospheric pressure, SLP and temp to altitude in meters
    altitude = bmp180.pressureToAltitude(seaLevelPressure, bmp180event.pressure, celsius); 
/*
    //  Display the barometric pressure in hPa
    sevenSeg.print(event.pressure);
    sevenSeg.writeDisplay();

    matrix8x8.clear();
    matrix8x8.drawBitmap(0, 0, hpa_bmp, 8, 8, LED_ON);
    matrix8x8.writeDisplay();

    delay(5000);
*/

    //  Display the temperature in Fahrenheit
    writeNumber(int(fahrenheit * 100), 2, false);
    sevenSeg[0].writeDisplay();

    matrix8x8.clear();
    matrix8x8.drawBitmap(0, 0, f_bmp, 8, 8, LED_ON);
    matrix8x8.writeDisplay();

    delay(7500);

    //  Display the temperature in Celsius
    writeNumber(int(celsius * 100), 2, false);
    sevenSeg[0].writeDisplay();

    matrix8x8.clear();
    matrix8x8.drawBitmap(0, 0, c_bmp, 8, 8, LED_ON);
    matrix8x8.writeDisplay();

/*    
    delay(5000);
    
    //  Display altitude in meters
    sevenSeg.print(altitude);
    sevenSeg.writeDisplay();

    matrix8x8.clear();
    matrix8x8.drawBitmap(0, 0, m_bmp, 8, 8, LED_ON);
    matrix8x8.writeDisplay();

    for (int8_t x=7; x>=-36; x--) {
      matrix8x8.clear();
      matrix8x8.setCursor(x,0);
      matrix8x8.print("m");
      matrix8x8.writeDisplay();
      delay(100);
    }

    //  Display atmospheric pressue in hPa
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
    
    //  Display the temperature in Celsius and Fahrenheit
    Serial.print("Temperature: ");
    Serial.print(celsius);
    Serial.print(" C, ");
    Serial.print(fahrenheit);
    Serial.println(" F");
    
    //  Display our altitude in meters
    Serial.print("Altitude:    ");
    Serial.print(altitude); 
    Serial.println(" m");
    Serial.println("");
*/
    minuteCount += 1;
    displayDate = (minuteCount == dateDisplayFreq);
    
    Serial.println();
    delay(7500);
  }
}
