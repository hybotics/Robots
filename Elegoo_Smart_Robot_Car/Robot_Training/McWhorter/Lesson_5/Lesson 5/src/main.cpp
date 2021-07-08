#include <Arduino.h>
int ENA=5; 
int ENB=6; 
int IN1=7; 
int IN2=8; 
int IN3=9; 
int IN4=11; 
float d; 
int degRot; 


/*
  Robot functions
*/

void stopCar(void) { 
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW); 
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,LOW); 
} 

void forward(float d){ 
  float t; 

  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW); 
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH); 

  t=d/2.45*1000;

  delay(t); 

  stopCar(); 
} 

void backward(float d) { 
  float t; 

  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,HIGH); 
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,LOW); 

  t=d/2.45*1000;

  delay(t); 

  stopCar(); 
} 

void turnRight(int deg) { 
  float t;
 
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,LOW);

  t=deg/345.*1000.; 

  Serial.println(deg); 

  delay(t);
  
  stopCar(); 
} 

void turnLeft(float deg) { 
  float t; 

  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH); 

  t=deg/345.*1000.; 

  Serial.println(deg); 

  delay(t);

  stopCar(); 
} 

/*
  Calibration functions
*/

void calF(void) { 
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW); 
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH); 

  delay(5000);

  stopCar(); 
} 

void calB(void) { 
  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,LOW); 

  delay(5000); 

  stopCar(); 
} 

void calR(void) { 
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW); 
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,LOW); 

  delay(5000);

  stopCar(); 
} 

void calL(){ 
  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,HIGH); 
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH); 

  delay(5000); 

  stopCar(); 
}

void setup(void) { 
  Serial.begin(9600); 

  pinMode(ENA,OUTPUT); 
  pinMode(ENB,OUTPUT); 
  pinMode(IN1,OUTPUT); 
  pinMode(IN2,OUTPUT); 
  pinMode(IN3,OUTPUT); 
  pinMode(IN4,OUTPUT); 

  digitalWrite(ENA,HIGH); 
  digitalWrite(ENB,HIGH); 
} 

void loop(void) { 
  forward(10); 
  turnRight(180); 
  forward(10); 
  turnLeft(180); 

  while(true) { }
}
