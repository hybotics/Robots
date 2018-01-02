// Date and time functions using a DS1307 RTC connected via I2C and Wire lib

#include <Wire.h>
#include "RTClib.h"

RTC_DS1307 rtc;

void setup () {
  Serial.begin(115200);
#ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif
  rtc.begin();

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
  } else {
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  }
}

void loop () {
    DateTime now = rtc.now();
//    uint8_t realMinute = &now.minute + 9;

    Serial.print("__DATE__ is ");
    Serial.print(__DATE__);
    Serial.print(", __TIME__ is ");
    Serial.println(__TIME__);

    Serial.print("Current time: ");
    Serial.print(now.month());
    Serial.print('/');
    Serial.print(now.day());
    Serial.print('/');
    Serial.print(now.year());
    Serial.print(' ');
    Serial.print(now.hour());
    Serial.print(':');
    Serial.print(now.minute());
    Serial.print(':');
    Serial.print(now.second());
    Serial.println();
    
    Serial.print(" since midnight 1/1/1970 = ");
    Serial.print(now.unixtime());
    Serial.print("s = ");
    Serial.print(now.unixtime() / 86400L);
    Serial.println("d");
    
    // calculate a date which is 7 days and 30 seconds into the future
    DateTime future (now.unixtime() + 7 * 86400L + 30);
    
    Serial.print(" now + 7d + 30s: ");
    Serial.print(future.month());
    Serial.print('/');
    Serial.print(future.day());
    Serial.print('/');
    Serial.print(future.year());
    Serial.print(' ');
    Serial.print(future.hour());
    Serial.print(':');
    Serial.print(future.minute());
    Serial.print(':');
    Serial.print(future.second());
    Serial.println();
    
    Serial.println();
    delay(3000);
}
