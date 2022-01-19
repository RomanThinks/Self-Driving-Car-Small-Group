#include "Adafruit_TCS34725.h"
#include <Wire.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

const byte interruptPin = 2;  //Pin used to trigger interrupt

const int leftPin  = 0;       //Left Motor
const int rightPin = 1;       //Left Motor

const int dirA = 12;          //decides to turn on Left Motor
const int dirB = 13;          //decides to turn on Right Motor
const int speedA = 3;         //Pin supplying the value for speed
const int speedB = 11;        //Pin supplying the value for speed

const int forward  = HIGH;
const int backward = LOW;

// the vehicle that is moving forward at a constant speed and will stop when obstacle is detected
// & continue to move when the obstacle disappears.
void isr1() {
  digitalWrite(dirA, LOW);
  digitalWrite(dirB, LOW);
  analogWrite(speedA, 0);
  analogWrite(speedB, 0);
}

void setup() {
  // Interrupt setup
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr1, LOW);
  uint16_t r;
  // Direction setup:
  pinMode(dirA,OUTPUT);
  pinMode(dirB,OUTPUT); 
  // Speed setup:
  pinMode(speedA,INPUT); 
  pinMode(speedB,INPUT); 
  //Serial Monitor setup:
  Serial.begin(9600); 
}

bool colorData() {
  uint16_t r, g, b, c;              
  tcs.getRawData(&r, &g, &b, &c);

  //Printing for testing purposes:
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  if (r > 600 && r < 1500 && g < 500 && b < 500) { //logic if red is detected
    return false;
  }
  else
    return true;                                    //red is not detected and everything can continue normally
}

void Forward() {
  digitalWrite(dirA, HIGH);
  digitalWrite(dirB, HIGH);
  analogWrite(speedA, 130);   //right motor speed
  analogWrite(speedB, 130);   //left motor speed
}

void Left() {  
  digitalWrite(dirA, HIGH);
  digitalWrite(dirB, HIGH);
  analogWrite(speedA, 100); 
  analogWrite(speedB, 0);
}

// will make car turn right
void Right() { 
  digitalWrite(dirA, HIGH);
  digitalWrite(dirB, HIGH);
  analogWrite(speedA, 0); 
  analogWrite(speedB, 100);  
}

void Stop() {
  analogWrite(speedA, 0);
  analogWrite(speedB, 0);
}

// Detects black line: 
// if left sensor detects white, car has moved off track in the left direction ==> move right
// if right sensor detects line, car has moved off track in the right direction ==> move left
//if neither, keep moving forward
void lineDetection() {

  Forward();
    
  int LValue  = analogRead(leftPin);      //both values are constantly read at each iteration
  int RValue = analogRead(rightPin);

  while(LValue < 750) {                                        
    Serial.println("Left Value is low, Turning Right");
    Right();
    LValue  = analogRead(leftPin);
    RValue = analogRead(rightPin);

  }
  
  while(RValue < 750) {
    Serial.println("Right Value is low, Turning Left");
    Left();
    LValue  = analogRead(leftPin);
    RValue = analogRead(rightPin);

  }

  //testing purposes
  Serial.print("Left Sensor: ");  Serial.print(LValue); Serial.print(" ");
  Serial.print("Right Sensor: "); Serial.print(RValue); Serial.println(" ");
  
  }

void loop() {

  bool color = colorData();   //colorData() returns true or false

  if(color) {
    lineDetection();
  }

  else {
    Serial.println("RED........STOP.......3 SECONDS......");
    Stop();
    delay(3000);             //3 second stop 
    Forward();
  }
}
