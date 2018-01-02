#include "BMSerial.h"
#include "RoboClaw.h"
#include "PacketSerialEncoderSpeedAccelDist_Walter.h"

BMSerial terminal(SERIAL_CONSOLE_RX_PIN, SERIAL_CONSOLE_TX_PIN, 10000);

//Arduino Mega and Leonardo chips only support some pins for receiving data back from the RoboClaw
//This is because only some pins of these boards support PCINT interrupts or are UART receivers.
//Mega: 0,10,11,12,13,14,15,17,19,50,51,52,53,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
//Leonardo: 0,8,9,10,11

//Arduino Due currently does not support SoftwareSerial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

RoboClaw roboClaw(SERIAL_ROBOCLAW_RX_PIN, SERIAL_ROBOCLAW_TX_PIN, 10000);

/*
    Pulses a digital pin for a duration in ms
*/
void pulseDigital(int pin, int duration) {
  digitalWrite(pin, HIGH);      // Turn the ON by making the voltage HIGH (5V)
  delay(duration);          // Wait for duration ms
  digitalWrite(pin, LOW);       // Turn the pin OFF by making the voltage LOW (0V)
  delay(duration);          // Wait for duration ms
}

void displayspeed (void) {
  uint8_t status;
  bool valid;

  terminal.println(F("Displaying speed.."));
  
  uint32_t enc1 = roboClaw.ReadEncM1(ROBOCLAW_SERIAL_BASE_ADDR, &status, &valid);

  if (valid) {
    terminal.print("Encoder1: ");
    terminal.print(enc1, DEC);
    terminal.print(", Status: ");
    terminal.print(status, HEX);
    terminal.print(" ");
  }

  uint32_t enc2 = roboClaw.ReadEncM2(ROBOCLAW_SERIAL_BASE_ADDR, &status, &valid);

  if (valid) {
    terminal.print("Encoder2: ");
    terminal.print(enc2, DEC);
    terminal.print(", Status: ");
    terminal.print(status, HEX);
    terminal.print(" ");
  }
  
  uint32_t speed1 = roboClaw.ReadSpeedM1(ROBOCLAW_SERIAL_BASE_ADDR, &status, &valid);

  if (valid) {
    terminal.print("Speed1: ");
    terminal.print(speed1, DEC);
    terminal.print(" ");
  }

  uint32_t speed2 = roboClaw.ReadSpeedM2(ROBOCLAW_SERIAL_BASE_ADDR, &status, &valid);

  if (valid) {
    terminal.print("Speed2: ");
    terminal.print(speed2, DEC);
    terminal.println();
  }
}

void setup (void) {
  terminal.begin(9600);
  roboClaw.begin(38400);

  //  Initialize the LED pin as an output.
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW);
  
  //  Set the RoboClaw motor constants
  roboClaw.SetM1VelocityPID(ROBOCLAW_SERIAL_BASE_ADDR, ROBOCLAW_KD, ROBOCLAW_KP, ROBOCLAW_KI, ROBOCLAW_QPPS);
  roboClaw.SetM2VelocityPID(ROBOCLAW_SERIAL_BASE_ADDR, ROBOCLAW_KD, ROBOCLAW_KP, ROBOCLAW_KI, ROBOCLAW_QPPS);
}

void loop() {
  char version[32];

  roboClaw.SpeedAccelDistanceM1(ROBOCLAW_SERIAL_BASE_ADDR, 12000, 12000, 48000);

  uint8_t depth1, depth2;

  // Pulse the heartbeat LED
  pulseDigital(HEARTBEAT_LED, 500);

  if (roboClaw.ReadVersion(ROBOCLAW_SERIAL_BASE_ADDR, version)) {
    terminal.print(F("RoboClaw 2x5 Version is "));
    terminal.println(version);
    terminal.println();
  }

  do {
    displayspeed();
    roboClaw.ReadBuffers(ROBOCLAW_SERIAL_BASE_ADDR, depth1, depth2);
  } while(depth1);

  roboClaw.SpeedAccelDistanceM1(ROBOCLAW_SERIAL_BASE_ADDR, 12000, -12000, 48000);

  do {
    displayspeed();
    roboClaw.ReadBuffers(ROBOCLAW_SERIAL_BASE_ADDR, depth1, depth2);
  } while(depth1);
}
