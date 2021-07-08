//  Import the Serial library
import processing.serial.*;

//  The Serial port object
Serial myPort;
String val;

//  Our port to contact the the Arduino
String device = "/dev/ttyACM0";

//  Since we're doing serial handshaking, 
//    we need to check if we've heard from the microcontroller
boolean firstContact = false;

void serialEvent (Serial myPort) {
  //  Put the incoming data into a String - 
  //    the '\n' is our end delimiter indicating the end of a complete packet
  val = myPort.readStringUntil('\n');

  //  Make sure our data isn't empty before continuing
  if (val != null) {
    //trim whitespace and formatting characters (like carriage return)
    val = trim(val);
    println(val);

    //  Look for our 'A' string to start the handshake
    //    if it's there, clear the buffer, and send a request for data
    if (firstContact == false) {
      if (val.equals("A")) {
        myPort.clear();
        firstContact = true;
        myPort.write("A");
        println("contact");
      }
    } else {
      //  If we've already established contact, keep getting and parsing data
      println(val);

      if (mousePressed == true) {
        //  If we clicked in the window
        myPort.write('1');        //send a 1
        println("1");
      }

      //  When you've parsed the data you have, ask for more:
      myPort.write("A");
    }
  }
}

void draw() {
  //  We can leave the draw method empty, 
  //    because all our programming happens in the serialEvent (see below)
}

void setup() {
  //  Make our canvas 200 x 200 pixels big
  size(200, 200);

  //  Initialize your serial port and set the baud rate to 9600
  myPort = new Serial(this, device, 9600);
  myPort.bufferUntil('\n'); 
}

