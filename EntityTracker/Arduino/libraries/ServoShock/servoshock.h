/*
 * servoshock.h
 * Rev. 1.0 Beta
 *
 * Created: 2/23/2014
 * Author: Mike
 *
To use this ServoShock library:
1. Setup the SPI bus.  Do not use a clock speed faster than 1MHz or the ServoShock can't keep up.
2. Create an instance of ServoShock. The input is the slave select pin number.
3. In the loop, modify the output packet if desired.  Access the values by using <Servoshock_Instance>.outPacket.<parameter>.  (See struct "SPI_OUTPUT_PACKET" below)
4. In the loop, call <Servoshock_Instance>.update() to perform a SPI transfer.  Wait at least 10 ms between calls.
5. In the loop, access the updated controller state by using <Servoshock_Instance>.inPacket.<parameter> (See struct "SPI_INPUT_PACKET" below).


///////////////////////USAGE EXAMPLE////////////////////
#include <SPI.h>
#include "servoshock.h"

const int slaveSelect = 10;  // set pin 10 as the slave select for the ServoShock.  Set jumper JP2 on the Shield to D10.
ServoShock Servoshock1(slaveSelect);

void setup() {
  //initialize SPI:
  digitalWrite(slaveSelect,HIGH);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin(); 
  
  //initialize UART for serial monitor
  Serial.begin(115200);
}

void loop() {
  //Set output parameters
  Servoshock1.outPacket.overrideLED = 1;
  Servoshock1.outPacket.LEDValue = 0b1111;
  Servoshock1.outPacket.overrideRumbleL = 1;
  Servoshock1.outPacket.rumbleL = 0xFF;
  
  //update controller state
  Servoshock1.update(); 
  
  //do something with updated inputs
  Serial.print(Servoshock1.inPacket.lStickX);  Serial.print(' ');
  Serial.print(Servoshock1.inPacket.lStickY);
  Serial.print("\n\r");
  
  //ServoShock polls the controller at 100Hz max; go do something else in the meantime.
  delay(50);
}
////////////////////////////////////////////////////////////
 */ 


#ifndef SERVOSHOCK_H_
#define SERVOSHOCK_H_

#define INPUT_PACKET_SIZE 59
#define OUTPUT_PACKET_SIZE 25


/************************************************************************************************
struct SPI_OUTPUT_PACKET
*************************************************************************************************
structure to hold SPI output packet (master to ServoShock)
************************************************************************************************/
typedef struct  __attribute__((packed)) _OUTPUT_FIELDS
{
  union
  {
    struct
    {
      //feedback overrides
      unsigned overrideLED :1; //byte 0
      unsigned overrideRumbleL :1;
      unsigned overrideRumbleH :1;
      unsigned :5;
      unsigned char LEDValue; //byte 1
      unsigned char rumbleL;
      unsigned char rumbleH; //byte3
      //output overrides
      unsigned overrideSelect:1; //byte4
      unsigned overrideLStickPress:1;
      unsigned overrideRStickPress:1;
      unsigned overrideStart:1;
      unsigned overrideDPadUp:1;
      unsigned overrideDPadRight:1;
      unsigned overrideDPadDown:1;
      unsigned overrideDPadLeft:1;
      unsigned overrideLTriggerPress:1; //byte 5
      unsigned overrideRTriggerPress:1;
      unsigned overrideLBumper:1;
      unsigned overrideRBumper:1;
      unsigned overrideTriangle:1;
      unsigned overrideCircle:1;
      unsigned overrideCross:1;
      unsigned overrideSquare:1;
      unsigned overridePsButton:1; // byte 6
      unsigned overrideLStickX:1;
      unsigned overrideLStickY:1;
      unsigned overrideRStickX:1;
      unsigned overrideRStickY:1;
      unsigned overrideLTriggerAnalog:1;
      unsigned overrideRTriggerAnalog:1;
      unsigned overrideAccelX:1;
      unsigned overrideAccelY:1; //byte 7
      unsigned overrideAccelZ:1;
      unsigned overrideGyroZ:1;
      unsigned :5; //padding for byte 7
      unsigned select:1; //byte 8
      unsigned lStickPress:1;
      unsigned rStickPress:1;
      unsigned start:1;
      unsigned dPadUp:1;
      unsigned dPadRight:1;
      unsigned dPadDown:1;
      unsigned dPadLeft:1;
      unsigned lTriggerPress:1; //byte 9
      unsigned rTriggerPress:1;
      unsigned lBumper:1;
      unsigned rBumper:1;
      unsigned triangle:1;
      unsigned circle:1;
      unsigned cross:1;
      unsigned square:1;
      unsigned psButton:1; //byte 10
      unsigned :7; //padding for byte 10
      unsigned char lStickX; //byte 11
      unsigned char lStickY;
      unsigned char rStickX;
      unsigned char rStickY;
      unsigned char lTriggerAnalog;
      unsigned char rTriggerAnalog;
      unsigned int accelX;
      unsigned int accelY;
      unsigned int accelZ;
      unsigned int gyroZ;
    };
    byte array[OUTPUT_PACKET_SIZE];
  };
}SPI_OUTPUT_PACKET;


/************************************************************************************************
struct SPI_INPUT_PACKET
*************************************************************************************************
structure to hold SPI input packet (ServoShock to master)
************************************************************************************************/
typedef struct  __attribute__((packed)) _INPUT_FIELDS
{
  union
  {
    struct
    {
      unsigned :8; //old buffer data, discard.  //byte 0
      unsigned :8; //old buffer data, discard.
      unsigned :8; //old buffer data, discard.
      unsigned :8; //old buffer data, discard.
      unsigned :8; //old buffer data, discard.
      unsigned :8; //old buffer data, discard.
      unsigned :8; //old buffer data, discard.
      unsigned :8; //old buffer data, discard.
      //start of packet data
      unsigned char packetType; //byte 8
      unsigned char empty; //not sure what this byte does, but if it's 0xFF, the packet is empty garbage
      unsigned select:1;  //byte 10 start
      unsigned lStickPress:1;
      unsigned rStickPress:1;
      unsigned start:1;
      unsigned dPadUp:1;
      unsigned dPadRight:1;
      unsigned dPadDown:1;
      unsigned dPadLeft:1;
      unsigned lTriggerPress:1;   //byte 11 start
      unsigned rTriggerPress:1;
      unsigned lBumper:1;
      unsigned rBumper:1;
      unsigned triangle:1;
      unsigned circle:1;
      unsigned cross:1;
      unsigned square:1;
      unsigned psButton:1;   //byte 12 start
      unsigned :7;
      unsigned :8;
      unsigned char lStickX;  //byte 14
      unsigned char lStickY;
      unsigned char rStickX;
      unsigned char rStickY;
      unsigned :8; //padding reserved for using 10-bit stick mode
      unsigned :8; //padding reserved for using 10-bit stick mode
      unsigned :8; //padding reserved for using 10-bit stick mode
      unsigned :8; //padding reserved for using 10-bit stick mode
      unsigned char dPadUpPressure;  //byte 22
      unsigned char dPadRightPressure;
      unsigned char dPadDownPressure;
      unsigned char dPadLeftPressure;
      unsigned char lTriggerAnalog;
      unsigned char rTriggerAnalog;
      unsigned char lBumperPressure;
      unsigned char rBumperPressure;
      unsigned char trianglePressure;
      unsigned char circlePressure;
      unsigned char crossPressure;
      unsigned char squarePressure;
      unsigned :8;
      unsigned :8;
      unsigned :8;
      unsigned char usbCable; ///0x02 = plugged, 0x03 = unplugged
      unsigned char battery; //0x05 = fully charged, 0x01 = dead
      unsigned char rumbleStatus; //0x10 = usb, rumble on. 0x12 = usb, rumble off. 0x14 = bluetooth, rumble on. 0x16 bluetooth, rumble off.
      unsigned :8;  //not sure about this one, turns to 0xFF when RSSI is first updated.
      unsigned char RSSI;  //looks like a signal strength indicator, not sure though.  Updated every 10 secs.  //byte 41
      unsigned :8;
      unsigned :8;
      unsigned :8;
      unsigned :8;
      unsigned :8;
      unsigned :8;
      unsigned :8;
      unsigned int accelX;
      unsigned int accelY;
      unsigned int accelZ;
      unsigned int gyroZ;
      unsigned char rumbleL;
      unsigned char rumbleH;
      unsigned char LEDValue;
      };
    unsigned char array[INPUT_PACKET_SIZE];
  };
}SPI_INPUT_PACKET;
            
/************************************************************************************************
class ServoShock
*************************************************************************************************
Create an instance of ServoShock.

Input: Slave Select digital I/O pin number

Usage Example: 
ServoShock servoshock1(10);
************************************************************************************************/
class ServoShock
{
	public:
		ServoShock(int slaveSelectPin);
    void update(void);
    SPI_OUTPUT_PACKET outPacket;
    SPI_INPUT_PACKET inPacket;

	private:
  int SSPin;
  unsigned int byteSwap(unsigned int value);
};

ServoShock::ServoShock(int slaveSelectPin)
{
  pinMode(slaveSelectPin, OUTPUT);
  SSPin = slaveSelectPin;
};

/************************************************************************************************
function ServoShock::update
*************************************************************************************************
Performs the SPI packet transfer to read from and write to the ServoShock module.  Also converts
the 16-bit parameters (accels and gyro) between little endian (Arduino) to big endian (DualShock)

Usage Example:
update();
************************************************************************************************/
void ServoShock::update(void)
{
  //switch little-endian accel/gyro figures to big endian in output packet
  outPacket.accelX = byteSwap(outPacket.accelX);
  outPacket.accelY = byteSwap(outPacket.accelY);
  outPacket.accelZ = byteSwap(outPacket.accelZ);
  outPacket.gyroZ = byteSwap(outPacket.gyroZ);
  
  
  digitalWrite(SSPin, LOW);
  for (unsigned int i = 0; i<INPUT_PACKET_SIZE; i++)
  {
    if (i < OUTPUT_PACKET_SIZE)
    {
      inPacket.array[i] = SPI.transfer(outPacket.array[i]);
    }
    else
    {
      inPacket.array[i] = SPI.transfer(0xCC); //padding bytes
    }
  }
  digitalWrite(SSPin, HIGH);
  
  //switch big-endian accel/gyro figures to little endian in input packet
  
  inPacket.accelX = byteSwap(inPacket.accelX);
  inPacket.accelY = byteSwap(inPacket.accelY);
  inPacket.accelZ = byteSwap(inPacket.accelZ);
  inPacket.gyroZ = byteSwap(inPacket.gyroZ);
}

/************************************************************************************************
function ServoShock::byteSwap
*************************************************************************************************
Swaps bytes on 16-bit integers.

Usage Example:
swappedInteger = byteSwap(integer);
************************************************************************************************/
unsigned int ServoShock::byteSwap(unsigned int value)
{
  return (value << 8) | (value >> 8 );
}

#endif /* SERVOSHOCK_H_ */
