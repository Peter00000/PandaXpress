/************* Program variables ************/
int state = 1;
float photoresistorSignal;
float duration_left, duration_top, duration_right, duration_side,
      inches_left, inches_right, inches_top;   //ultrasonic reading
float boulder_length, boulder_height, boulder_area, voltage;
int obstacleCount = 0;

/**************** Constants *****************/
const int markerNum = 113;    //update this when receive new marker
const float pi      = 3.14159;
const int repeatNo  = 5;      //for multiple ultrasonic read
const int interval  = 200;    //ms wait
const long upper    = 1200;   //filter out unwanted data in ultrasonic reading
const long lower    = 150;    //only apply when averaging multiple trials
const float arm_width   = 13.75;   //inches between left and right ultrasonic sensor
const float arm_height  = 13;   //inches between top ultrasonic and ground
  float blackCutOff = .25;   // this should be tested with the boulder
  float greenCutOff = .45;
const int k = 200;              //constant relate delta_angle to delta_PWM
const int maxCount = 3;
/************** Pin Variables ***************/
int in3 = 2;  int in4 = 3;  
int enb = 4;  int ena = 5; //ena: right wheel   enb: left wheel
int in1 = 6;  int in2 = 7;
int LED = 9;  
int RF_TX = 10; 
int RF_RX = 11;
int trig_left_top = 12;
int trig_right_side = 13;
int photoresist_pin     = A0; //analog pins
int ultrasound_left_pin = A1;
int ultrasound_top_pin  = A2;
int ultrasound_right_pin= A3;
int ultrasound_side_pin = A4;

/********* Initiate RF GPS System ***********/
#include <SoftwareSerial.h>
#include "enes100.h"
SoftwareSerial mySerial(RF_RX, RF_TX);
enes100::RfClient<SoftwareSerial> rf(&mySerial); 
enes100::Marker marker;

/*************** Main Code ******************/
/** Setup (4/14/2016 Austin)
 * initiate all system variables */
void setup() {
  Serial.begin(9600);
  RFSetup();
  motorSetup();
  ultrasonicSetup();
}

/** loop  (4/14/2016  Austin)
 * main loop that navigate robot through different stages
 * use the variable 'state' to track stages of navigation */
void loop() {
  RFLoop();
  switch(state) {
    case 0: break;
    case 1: //OSV is in landing zone
      turnRight(-pi/2);
      if(marker.y> 0.4) 
            driveForwardYDirection(0.3, marker.x, false);
      turnLeft(0);
      driveForwardXDirection(.7, 0.3);
      state = 2; 
      break; //this shoss at edge of landing zone
    case 2:  
      turnLeft(pi/2);
      driveForwardYDirectionSensor(); //scans for an opening between obsticles
      turnRight(0);
      state = 3;
      break;
    case 3://OSV can now move forward because no obstacles are in the way
      if (marker.y != 1) {
        if (marker.y > 1) {
          turnRight(-pi/2);
          driveForwardYDirection(1,1.5,false); //need to update for the new control code
          turnLeft(0);
            
        } else {
          turnLeft(pi/2);
          driveForwardYDirection(1,1.5,true);
          turnRight(0);
        }
      }
      driveForwardXDirection(3.1,1);
      state = 4;
      break;
    case 4://OSV is now at the x value of the Terrain Site
      turnLeft(pi/2);
      driveForwardYDirection(1.3,3.1,true);
      turnRight(0);
      //move forward until hit boulder
      state = 5;
      break;  
    case 5:
      motorStraight();
      delay(1200);
      motorControl(0,0); //stop the motor running before measure
      allSensors();     //mission code
  }
}

/************** Setup Code ******************/
/** RF Setup  (4/17/2016 Yichao)
 * Setup the RF communication with GPS system */
void RFSetup() {
   mySerial.begin(9600); //this establishes serial communication with
                        //something other than serial monitor, in this
                        //case RF communication with mission control

  pinMode(RF_RX, INPUT); //since pin 8 is RX, it receives as an input
  pinMode(RF_TX, OUTPUT); //since pin 9 is TX, it transmits as an output
  
  rf.resetServer();
  rf.sendMessage("\nTeam PandaXpress is Connected\n"); //sent to mission control
  Serial.println("Team PandaXpress is Connected"); //sent to student's serial monitor
  delay(500);
}

/** motorSetup  (Paulo) */ 
void motorSetup() {
  pinMode(ena,OUTPUT);    pinMode(enb,OUTPUT);
  pinMode(in1,OUTPUT);    pinMode(in3,OUTPUT);
  pinMode(in2,OUTPUT);    pinMode(in4,OUTPUT);
}

/** ultrasonicSetup (4/28/2016 Stephen) */
void ultrasonicSetup() {
  pinMode(trig_left_top, OUTPUT);
  pinMode(trig_right_side, OUTPUT);
  pinMode(LED, OUTPUT);
}

/************** Motion Code *****************/
void motorStraight() { 
  digitalWrite(in1,HIGH);     digitalWrite(in3,LOW);
  digitalWrite(in2,LOW);      digitalWrite(in4,HIGH);
  analogWrite(ena,255);       analogWrite(enb,255);
}

void motorBack() { 
  digitalWrite(in1,LOW);      digitalWrite(in3,HIGH);
  digitalWrite(in2,HIGH);     digitalWrite(in4,LOW);
  analogWrite(ena,255);       analogWrite(enb,255);
}

void motorControl(int PWM_left, int PWM_right){
  digitalWrite(in1,HIGH);     digitalWrite(in3,LOW);
  digitalWrite(in2,LOW);      digitalWrite(in4,HIGH);
  analogWrite(ena,PWM_right); analogWrite(enb,PWM_left);
}

void motorTurnRight() {
  digitalWrite(in1, LOW);   digitalWrite(in3, LOW);
  digitalWrite(in2, HIGH);  digitalWrite(in4, HIGH);
  analogWrite(ena,170);     analogWrite(enb,170);
}

void motorTurnLeft() {
  digitalWrite(in1, HIGH);  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);   digitalWrite(in4, LOW);
  analogWrite(ena,170);     analogWrite(enb,170); //the left wheel will not run below 150
}

/** control (4/29/2016 Yichao) 
* change the wheel power supply based on the difference in theta */
void control(float theta_ref, float theta_desired){
      float delta_theta   = marker.theta - theta_desired; //calculate difference between current and desired angle
      int delta_PWM     = (int)(abs(delta_theta*k));  //calculate speed difference based on angle differnece
      if(delta_PWM>127){
            if(delta_theta > theta_ref){
                  motorTurnRight(); return;}
            else{ motorTurnLeft(); return;}}
      if (delta_theta > theta_ref) 
            motorControl(255,255-delta_PWM);
      else  motorControl(255-delta_PWM,255);
}

/** driveForwardXDirection (4/14/2016 Austin) */ 
void driveForwardXDirection(float x_goal, float y_ref) {
  while (marker.x < x_goal) {
    RFLoop();
    float error = marker.y-y_ref;
    float theta = -0.5*atan(error);
    control(0, theta);
  }
  motorControl(0,0); //stop the motor when the destination is reached
}

/** driveForwardYDirection (4/14/2016 Austin) */ 
void driveForwardYDirection(float y_goal, float x_ref, bool positive_dir) {
  float tolerance = .01;
  float error, y_ref;
  double theta;
  if (positive_dir){ //moving up the field
      while (marker.y - y_goal < -(tolerance)) {
            RFLoop();
            error = marker.y-y_ref;
            theta = -1/2*atan(error) + pi/2;
            control(pi/2, theta);
      }
  } else { //moving down the field
      while(marker.y - y_goal > tolerance){
            RFLoop();
            error = marker.y-y_ref;
            theta = -1/2*atan(error) -pi/2;
            control(-pi/2, theta);
      }
  }
  motorControl(0,0); //stop the motor when the destination is reached
}

void driveForwardYDirectionSensor() {
  bool obstacle;
  while (true) {
    motorStraight();
    obstacle = senseObstacle();
    if(!obstacle) obstacleCount++;
    else obstacleCount = 0;
    if(obstacleCount > maxCount) break;
  } 
  //motorBack();
  //delay(1000);
}

/** turnLeft (4/14/2016 Austin)
 * Turn the OSV to the desired orientation to the left */ 
void turnLeft(float orientation) {
  float tolerance = 0.05;
  while (marker.theta - orientation < -(tolerance)) {
    RFLoop();
    motorTurnLeft();
  }
  motorControl(0,0); //stop the motor when the angle is reached
}

/** turnRight (4/14/2016 Austin)
 * Turn the OSV to the desired orientation to the right */ 
void turnRight(float orientation) {
  float tolerance = 0.05;
  while (marker.theta - orientation > tolerance) {
    RFLoop();
    motorTurnRight();
  }
  motorControl(0,0); //stop the motor when the angle is reached
}

/************** Sensor Code ******************/

/** senseObstacle (4/14/2016 Austin)
 * Determines if the obsticle exist based on the ultrasonic reading
 * Caution: need to modify for better path finding, does not remember past values */ 
bool senseObstacle() {
  float inches_side;
  duration_side   = ping(trig_right_side,ultrasound_side_pin);
  inches_side     = microsecondsToInches(duration_side);
  if (inches_side < 12) {return true;}
  else {return false;}
}

/**RFLoop (Keystone)
 * Read and print the x, y, theta orientation */ 
void RFLoop() {
    if(rf.receiveMarker(&marker, markerNum))
  {
    //rf.sendMessage("\nPX: read data");
//     Serial.print("PX: RF Data");
//     Serial.print(marker.x); Serial.print("|");
//     Serial.print(marker.y); Serial.print("|");
//     Serial.println(marker.theta);
      analogWrite(LED, 0); //turn the LED off when it is receiving the signal
  }
  else
  {
    //rf.sendMessage("\nPX: Marker not register");
    analogWrite(LED, 235);  //turn the LED on when ping fails
    Serial.println("PX: Marker is not registering");
    //delay(300);
  }
}

/** ping (4/28/2019 Yichao Peng)
 * Single ping for a given ultrasonic sensor */
long ping(int trig_pin, int echo_pin){
  digitalWrite(trig_pin, LOW); 
  delayMicroseconds(2); // Short LOW pulse beforehand ensures a clean HIGH pulse:
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(5); // Ping triggered by a HIGH pulse of 2 or more microseconds.
  digitalWrite(trig_pin, LOW);
  return pulseIn(echo_pin, HIGH); // Reads the pulse duration of the echo in milliseconds
}
 
/** MultiplePing (Yichao)
* For taking multiple pings and average them
* Filters out number outside the range of expected values, return 0 */
float multiplePing(int trig_pin, int echo_pin){
  float duration = 0;
  int trialNum = 0;
  for(int i=0; i<repeatNo; i++){
    long d = ping(trig_pin,echo_pin);
    if(d > lower && d < upper){
        duration = duration + d;
        trialNum++;
    }
    delay(interval);
  }
  if(trialNum == 0){ return 0;}
  else  {return duration /1.0 / trialNum; }
}

// 73.746 microseconds per inch (i.e. sound travels at 1130 feet per seconds
// See: http://www.parallax.com/dl/docs/prod/acc/28015-PI...
float microsecondsToInches(long microseconds)
      {return microseconds / 73.746 / 2.0;}

// The speed of sound is 340 m/s or 29 microseconds per centimeter.
float microsecondsToCentimeters(long microseconds)
      {return microseconds / 29.41 / 2.0;}

/** colorSensor (4/28/2016 Stephen)
 * determine color of boulder and return value*/
void colorSensor(){
  analogWrite(LED, 235);   //90% duty cycle for 4.5V
  delay(400);
  photoresistorSignal = analogRead(photoresist_pin); //take the reading at analog input pin
  voltage= (5.0*photoresistorSignal)/1023;           //convert the signal to a scale of 0 to 5 V
  Serial.println(voltage);
  if(voltage > blackCutOff && voltage < greenCutOff ){
      rf.sendMessage("Green");
  } else if(voltage < blackCutOff){
      rf.sendMessage("Black");
  }
  analogWrite(LED,0);
}

/** allSensors (4/28/2016 Stephen) 
* conduct mission measurement*/
void allSensors(){
  duration_left   = multiplePing(trig_left_top,ultrasound_left_pin); //average multiple ping
  duration_top    = multiplePing(trig_left_top,ultrasound_top_pin);
  duration_right  = multiplePing(trig_right_side,ultrasound_side_pin);
  
  inches_left     = microsecondsToInches(duration_left); // convert the time into a distance
  inches_top      = microsecondsToInches(duration_top);
  inches_right    = microsecondsToInches(duration_right);
  
  boulder_length  = arm_width - (inches_left + inches_right); //calculate length
  boulder_height  = arm_height - inches_top;                  //calculate height
  boulder_area    = boulder_length * boulder_height;          //calculate area
  
  rf.sendMessage("\nLength: ");     //send the message back to mission control
  rf.sendMessage(boulder_length);
  rf.sendMessage("in");
  rf.sendMessage("\nHeight: ");
  rf.sendMessage(boulder_height);  
  rf.sendMessage("in");
  rf.sendMessage("\nSurface Area: ");
  rf.sendMessage(boulder_area);
  rf.sendMessage("in^2");
  rf.sendMessage("\nColor: ");        
  colorSensor();
}
