#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303DLHC_Unified.h>
     
/* Assign a unique ID to these sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

float AccelMinX, AccelMaxX;
float AccelMinY, AccelMaxY;
float AccelMinZ, AccelMaxZ;

float MagMinX, MagMaxX;
float MagMinY, MagMaxY;
float MagMinZ, MagMaxZ;
     
long lastDisplayTime;
     
void setup(void) {
    Serial.begin(9600);
    Serial.println("LSM303 Calibration");
    Serial.println("");

    /* Initialise the accelerometer */
    if (!accel.begin()) {
        /* There was a problem detecting the LMS303 Accelerometer... check your connections */
        Serial.println("Ooops, no LSM303 Accelerometer detected ... Check your wiring!");
        while(1);
    }

    /* Initialise the magnetometer */
    if (!mag.begin()) {
        /* There was a problem detecting the LSM303 Magnetometer (Compass) ... check your connections */
        Serial.println("Ooops, no LSM303 Magnetometer (Compass) detected ... Check your wiring!");
        while(1);
    }

    lastDisplayTime = millis();
}
     
void loop(void) {
    /* Get a new sensor event */
    sensors_event_t accelEvent;
    sensors_event_t magEvent;

    accel.getEvent(&accelEvent);
    mag.getEvent(&magEvent);

    if (accelEvent.acceleration.x < AccelMinX) {
        AccelMinX = accelEvent.acceleration.x;
    }

    if (accelEvent.acceleration.x > AccelMaxX) {
        AccelMaxX = accelEvent.acceleration.x;
    }

    if (accelEvent.acceleration.y < AccelMinY) {
        AccelMinY = accelEvent.acceleration.y;
    }

    if (accelEvent.acceleration.y > AccelMaxY) {
        AccelMaxY = accelEvent.acceleration.y;
    }
     
    if (accelEvent.acceleration.z < AccelMinZ) {
        AccelMinZ = accelEvent.acceleration.z;
    }

    if (accelEvent.acceleration.z > AccelMaxZ) {
        AccelMaxZ = accelEvent.acceleration.z;
    }

    if (magEvent.magnetic.x < MagMinX) {
        MagMinX = magEvent.magnetic.x;
    }

    if (magEvent.magnetic.x > MagMaxX) {
        MagMaxX = magEvent.magnetic.x;
    }

    if (magEvent.magnetic.y < MagMinY) {
        MagMinY = magEvent.magnetic.y;
    }

    if (magEvent.magnetic.y > MagMaxY) {
        MagMaxY = magEvent.magnetic.y;
    }

    if (magEvent.magnetic.z < MagMinZ) {
        MagMinZ = magEvent.magnetic.z;
    }

    if (magEvent.magnetic.z > MagMaxZ) {
        MagMaxZ = magEvent.magnetic.z;
    }

    if ((millis() - lastDisplayTime) > 1000) {          //  Display once/second
        Serial.print("Accel Minimums: X = ");
        Serial.print(AccelMinX);
        Serial.print(", Y = ");
        Serial.print(AccelMinY);
        Serial.print(", Z = ");
        Serial.println(AccelMinZ);
        Serial.println();

        Serial.print("Accel Maximums: X = ");
        Serial.print(AccelMaxX);
        Serial.print(", Y = ");
        Serial.print(AccelMaxY);
        Serial.print(", Z = ");
        Serial.println(AccelMaxZ);
        Serial.println();

        Serial.print("Mag Minimums: X = ");
        Serial.print(MagMinX);
        Serial.print(", Y = ");
        Serial.print(MagMinY);
        Serial.print(", Z = ");
        Serial.println(MagMinZ);
        Serial.println();

        Serial.print("Mag Maximums: X = ");
        Serial.print(MagMaxX);
        Serial.print(", Y = ");
        Serial.print(MagMaxY);
        Serial.print(", Z = ");
        Serial.println(MagMaxZ);
        Serial.println();
        Serial.println();

        lastDisplayTime = millis();
    }
}
