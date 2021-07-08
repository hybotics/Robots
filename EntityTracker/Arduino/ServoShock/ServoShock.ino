/*
This example reads the Dualshock 3 controller state from the ServoShock controller and outputs it to the serial console. 
It will also override the LED and rumble motors with SPI commands.

Set the serial baud rate to 115200 bps.
Make sure your terminal window is wide enough to avoid wrap-around to make easier to see the output.
*/

// include the SPI library:
#include <SPI.h>
#include "servoshock.h"


// set the slave select pin for the ServoShock.  Set jumper JP2 on the Shield to D10 if using digital output 10.
const int slaveSelect = 10;
ServoShock Servoshock1(slaveSelect);  //create instance of ServoShock

void setup() {
  //initialize SPI:
  digitalWrite(slaveSelect,HIGH);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin(); 
  
  Serial.begin(115200);  //initialize serial monitor
}


void loop() {

  Servoshock1.update(); 
  
  Servoshock1.outPacket.overrideLED = 1;
  Servoshock1.outPacket.LEDValue = 0b1111;  //bit mask forLEDs
  Servoshock1.outPacket.overrideRumbleL = 1;
  Servoshock1.outPacket.rumbleL = 0; //Rumble motor, set from 0-255.
  Servoshock1.outPacket.overrideRumbleH = 1;
  Servoshock1.outPacket.rumbleH = 0; //Rumble motor, set from 0-255.

  SerialWriteDec(Servoshock1.inPacket.lStickX);  Serial.print(' ');
  SerialWriteDec(Servoshock1.inPacket.lStickY);  Serial.print(' ');
  SerialWriteDec(Servoshock1.inPacket.rStickX);  Serial.print(' ');
  SerialWriteDec(Servoshock1.inPacket.rStickY);  Serial.print(' ');
  SerialWriteDec(Servoshock1.inPacket.lTriggerAnalog);  Serial.print(' ');
  SerialWriteDec(Servoshock1.inPacket.rTriggerAnalog);  Serial.print(' ');
  SerialWriteBin(Servoshock1.inPacket.array[10]); //buttons
  SerialWriteBin(Servoshock1.inPacket.array[11]); //buttons
  SerialWriteBin(Servoshock1.inPacket.array[12]);  Serial.print(' ');
  SerialWriteIntDec(Servoshock1.inPacket.accelX);  Serial.print(' ');
  SerialWriteIntDec(Servoshock1.inPacket.accelY);  Serial.print(' ');
  SerialWriteIntDec(Servoshock1.inPacket.accelZ);  Serial.print(' ');
  SerialWriteIntDec(Servoshock1.inPacket.gyroZ);
  Serial.print("\n\r");
  
  /*
  for (int i = 0; i<INPUT_PACKET_SIZE; i++)
  {
    SerialWriteHex(Servoshock1.inPacket.array[i]);
    Serial.print(' ');
  }
  */
  
  //ServoShock polls the controller at 100Hz max; go do something else in the meantime.
  delay(50);
}

//Subroutine to print packet data.
void SerialWriteHex(unsigned char byte)
{
  if (byte < 0x10)
  {
    Serial.print('0');
  }
  Serial.print(byte,HEX);
}

void SerialWriteIntHex(unsigned int uinteger)
{
  if (uinteger < 0x1000)
  {
    Serial.print('0');
  }
  if (uinteger<0x0100)
  {
    Serial.print('0');
  }
  if (uinteger<0x010)
  {
    Serial.print('0');
  }
  Serial.print(uinteger,HEX);
}

void SerialWriteDec(unsigned char byte)
{
  if (byte < 100)
  {
    Serial.print('0');
  }
  if (byte < 10)
  {
    Serial.print('0');
  }
  Serial.print(byte,DEC);
}

void SerialWriteIntDec(unsigned int uinteger)
{
  if (uinteger < 10000)
  {
    Serial.print('0');
  }
  if (uinteger < 1000)
  {
    Serial.print('0');
  }
  if (uinteger < 100)
  {
    Serial.print('0');
  }
  if (uinteger < 10)
  {
    Serial.print('0');
  }
  Serial.print(uinteger,DEC);
}


void SerialWriteBin(unsigned char byte)
{
  unsigned char mask = 0b10000000;
  for (unsigned char i=0; i<8 ; i++)
  {
    if (mask & byte) {
        Serial.print('1');
    }
    else {
        Serial.print('0');
    }
    mask = mask>>1;
  }
}      
