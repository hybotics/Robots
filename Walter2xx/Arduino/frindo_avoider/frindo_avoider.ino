
// arduino "avoider" sketch   version 1.0   J.Luke (RobotBits.co.uk)
//
// A simple sketch to help you get started with an Arduino powered frindo robot
//
// This example has been written for the frindo open source robotics platform and uses
// two bi-directional motors controlled by an Arduino and a RobotShield
//
// This sketch uses the RobotShield library which includes a series of functions that perform 
// the different types of movement for your robot including:
//
//     forward, reverse, stop, rotate clock-wise, rotate counter clock-wise, 
//     turn_left and turn_right 
//
// To program the movement of your robot simply call the functions as needed
// from the main loop
//
// This example is a 3 sensor "avoider" application. The robot will move forward continuously 
// avoiding obstacles etc. This application uses 3 infrared sensors (forward, left & right) connected
// to Analogue 0, 1 & 2. Although these are analogue sensors, the code below has a threshold detector 
// for each sensor creating simple "bump sensors"
//
// For support, please use the frindo support forum at www.frindo.org



#include <RobotShield.h> // include the RobotShield library

RobotShield rs;          // create an instance of the RobotShield class called "rs"


// define the pins used
int lock = 5;            // lock is a disable input - program will not run if lock is in place
                         // the lock is a single wire (pin to ground)
                         // NOTE: DO NOT use pins 0 or 1 for the lock as this will stop
                         // you being able to program your robot!

// define the analogue pins used by IR Sensor
int FrontBump = 0;       // Front IR sensor connected to pin 0
int RightBump = 1;       // Right-hand IR sensor (as viewed from rear) connected to pin 1
int LeftBump = 2;        // Left-hand IR sensor (as viewed from rear) connected to pin 2
int RearBump = 3;        // Rear IR sensor connected to pin 4

// define variables used
int locked = HIGH;

int bump;
int left;
int right;
int rear;

int frontTrigger = 200;    // define the threshold at which bump events are detected
int sideTrigger = 200;
int rearTrigger = 200;


void setup() {       
              
              pinMode(lock, INPUT);           // set lock pin as input
              digitalWrite(lock, HIGH);       // turn on pullup resistors
              
              Serial.begin(9600);             // set up serial port
             }



int front_bump() {                            // check for front bump, if bump detected return 1
  
            bump = analogRead(FrontBump);      // read front IR sensor
            
            if(bump > frontTrigger){                 // look for bump             
                return 1;
                } 
            else {  
                return 0;
                }                
            }
            
            
int left_bump() {                            // check for left-hand bump, if bump detected return 1
  
            left = analogRead(LeftBump);      // read left-hand IR sensor
            
            if(left > sideTrigger){                 // look for bump             
                return 1;
                } 
            else {  
                return 0;
                }                
            }
            
            
int right_bump() {                            // check for right-hand bump, if bump detected return 1
  
            right = analogRead(RightBump);      // read right-hand IR sensor
            
            if(right > sideTrigger){                 // look for bump             
                return 1;
                } 
            else {  
                return 0;
                }                
            }
 
int rear_bump() {                            // check for rear bump sensor, if bump detected return 1
  
            rear = analogRead(RearBump);      // read rear IR sensor
            
            if(rear > rearTrigger){                 // look for bump             
                return 1;
                } 
            else {  
                return 0;
                }                
            }           
            
            
void loop() { 
  
            Serial.println("Stopped!... waiting!");

            locked = digitalRead (lock);  
            
            while(locked == LOW){                    // while lock is in place stop and wait for lock to be removed
                                                     // this section includes some useful debug tools - use a serial port link
                                                     // to connect and view/set the bump sensor thresholds 
                  rs.stop();
                  locked = digitalRead (lock);
                  
                  bump = analogRead(FrontBump);      // read front IR sensor
                  left = analogRead(LeftBump);       // read left-hand IR sensor
                  right = analogRead(RightBump);     // read right-hand IR sensor             
                  
                  Serial.print("Front level: ");
                  Serial.println(bump);
    
                  if (bump > frontTrigger){
                      Serial.println("Front bump detected!");
                      Serial.println("");
                  }
                  
                  Serial.print("Left Level: ");
                  Serial.println(left);
    
                  if (left > sideTrigger){
                      Serial.println("Left bump detected!");
                      Serial.println("");
                  }
                  
                  Serial.print("Right Level: ");
                  Serial.println(right);
                  
                  if (right > sideTrigger){
                      Serial.println("Right bump detected!");
                      Serial.println("");
                  }
                  
                  delay(1000);
                  Serial.println("");
            }

            
            while(!front_bump()){      // while there is no bump keep going forward (about 10cm with GPD120)
            
                if(!left_bump() && !right_bump()) { 
                    Serial.println("NO bump detected - move forward");
                    rs.forward(500, 200);                 // move forward for 500 mS at speed 200 (200/255ths of full speed)   
                }
                
                else if(left_bump() && !right_bump())  {
                    Serial.println("LEFT bump detected - turn right");
                    rs.rot_cw(100, 200);                 // turn right for 100 mS at speed 200 (200/255ths of full speed)      
                }
                
                else if(!left_bump() && right_bump())  {
                    Serial.println("RIGHT bump detected - turn left");
                    rs.rot_ccw(100, 200);                 // turn left for 100 mS at speed 200 (200/255ths of full speed)       
                }
                
                else if(left_bump() && right_bump())  {
                    Serial.println("BOTH bumps detected - rotating");
                    rs.rot_cw(100, 200);                 // rotate clockwise for 100 mS at speed 200 (200/255ths of full speed) 
                }    
            }
       
            Serial.println("FRONT bump detected - rotating");
            rs.rot_cw(100, 200);                 // rotate clockwise for 100 mS at speed 200 (200/255ths of full speed) 
                    
      } 
