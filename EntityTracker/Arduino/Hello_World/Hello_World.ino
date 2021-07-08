void setup (void) {
  //  Initialize serial communications at a 9600 baud rate
  Serial.begin(9600);
}

void loop (void) {
  //  Send 'Hello, world!' over the serial port
  Serial.println("Hello, world!");

  //  Wait 100 milliseconds so we don't drive ourselves crazy
  delay(100);
}

