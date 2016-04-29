/************* Program variables ************/
int state = 1;
float photoresistorSignal;
float duration_left, duration_top, duration_right, 
      inches_left, inches_right, inches_top;   //ultrasonic reading
bool is_boulder_green;
float boulder_length, boulder_height, boulder_area, voltage;

/**************** Constants *****************/
const int markerNumber = 113; //update this when receive new marker
const float pi      = 3.14159;
const int repeatNo  = 5;      //for multiple ultrasonic read
const int interval  = 200;    //ms wait
const long upper    = 1200;   //filter out unwanted data in ultrasonic reading
const long lower    = 150;    //only apply when averaging multiple trials
const float arm_width   = 13;   //inches between left and right ultrasonic sensor
const float arm_height  = 15;   //inches between top ultrasonic and ground
const float color_cutoff = 3.7; //color cutoff voltage
const int k = 150;              //constant relate delta_angle to delta_PWM

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
      turnRight(0);
      driveForwardXDirection(0.62,marker.y);
      state = 2; break; //this should be in the control code
    case 2: //OSV is at edge of landing zone
      bool obstacle;
      obstacle = senseObstacle();
      //turnLeft(pi/2);
      if(obstacle == true){ //need to recode this part
        if(marker.y > 1){
          turnRight(-pi/2);
          driveForwardYDirection(1,0.62, false);
          turnLeft(0);
          break;
        } else {
          turnLeft(pi/2);
          driveForwardYDirection(1,0.62,true);
          turnRight(0);
          break;
        }
      } else {
        //turnRight(0);
        state = 3;
        break;
      }
    case 3://OSV can now move forward because no obstacles are in the way
      if (marker.y != 1) {
        if (marker.y > 1) {
          turnRight(-pi/2);
          driveForwardYDirection(1,0.62,false); //need to update for the new control code
          turnLeft(0);
            
        } else {
          turnLeft(pi/2);
          driveForwardYDirection(1,0.62,true);
          turnRight(0);
        }
      }
      driveForwardXDirection(3.2,1);
      state = 4;
      break;
    case 4://OSV is now at the x value of the Terrain Site
      turnLeft(pi/2);
      driveForwardYDirection(1.5,3.2,true);
      turnRight(0);
      //move forward until hit boulder
      //call allSensors();
      state = 0;
      break;  
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
  //delay(1000);
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
  digitalWrite(in1,HIGH);    digitalWrite(in3,LOW);
  digitalWrite(in2,LOW);   digitalWrite(in4,HIGH);
  analogWrite(ena,255);     analogWrite(enb,255);
}

void motorControl(int PWM_left, int PWM_right){
  digitalWrite(in1,HIGH);    digitalWrite(in3,LOW);
  digitalWrite(in2,LOW);   digitalWrite(in4,HIGH);
  analogWrite(ena,PWM_right);     analogWrite(enb,PWM_left);
}

void motorTurnRight() {
  digitalWrite(in1, LOW);   digitalWrite(in3, LOW);
  digitalWrite(in2, HIGH);  digitalWrite(in4, HIGH);
  analogWrite(ena,150);     analogWrite(enb,150);
}

void motorTurnLeft() {
  digitalWrite(in1, HIGH);  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);   digitalWrite(in4, LOW);
  analogWrite(ena,150);     analogWrite(enb,150); //the left wheel will not run below 150
}

/** driveForwardXDirection (4/14/2016 Austin)
 * Caution: used for fakebot only, does not move robot */ 
void driveForwardXDirection(float x_goal, float y_ref) {
  while (marker.x < x_goal) {
    RFLoop();
    float error = marker.y-y_ref;
    float theta = -1/2*atan(error);
    control(0, theta);
  }
}

void control(float theta_ref, float theta_desired){
      float delta_theta   = marker.theta - theta_desired; //calculate difference between current and desired angle
      int delta_PWM     = (int)(abs(delta_theta*k));  //calculate speed difference based on angle differnece
      if(delta_PWM>255)       delta_PWM = 255;
      if (delta_theta > theta_ref) 
            motorControl(255,255-delta_PWM);
      else  motorControl(255-delta_PWM,255);
      delay(300);
}

/** controlX (4/28/2016 Mitchell) */
// void controlX(int xRefIn, int xTermIn) {
//   digitalWrite(in1,LOW);    digitalWrite(in3,HIGH);
//   digitalWrite(in2,HIGH);   digitalWrite(in4,LOW);
  
//   if (x.marker < xterm) {
//     float delta_y       = marker.y-yref;
//     float theta_desired = -1/2*arctan(delta_y);
//     float delta_theta   = marker.theta - theta_desired;
//     int delta_PWM     = (int)(abs(delta_theta*k));
//     if(delta_PWM>255)   deltaPWM = 255;
//     if (delta_theta > 0) {
//       analogWrite(enb, 255);                   //these are ena and enb
//       analogWrite(ena, 255-delta_PWM); 
//     } else {
//       analogWrite(ena, 255);
//       analogWrite(enb, 255-delta_PWM);
//     }
//     delay(400);
//     }
//   else state++;
// }

/** driveForwardYDirection (4/14/2016 Austin) */ 
void driveForwardYDirection(float y_goal, float x_ref, bool positive_dir) {
  float tolerance = .05;
  float error = marker.y-y_ref;
  float theta = -1/2*arctan(error);
  if (positive_dir){ //moving up the field
        while (marker.y - y_goal < -(tolerance)) {
              RFLoop();
              theta = theta + pi/2;
              control(pi/2, theta);
        }
  } else { //moving down the field
        while(marker.y - y_goal > tolerance){
              RFLoop();
              theta = theta + 3*pi/2;
              control(3*pi/2, theta);
        }
  }
}

/** controlY (4/28/2016 Mitchell) */
// void controlY(int xref, int yterm) {
//   if (y.marker < yterm) {
//     float delta_x       = marker.x-xref;
//     float theta_desired = -1/2*arctan(delta_x);
//     float delta_theta   = marker.theta-pi/2 - theta_desired;
//     int delta_PWM       = (int)(abs(delta_theta*k));
//     if(delta_PWM>255)   deltaPWM = 255;
//     if (delta_theta > 0) {
//       analogWrite(enb, 255);
//       analogWrite(ena, 255-delta_PWM); //right_wheel pin4? and leftwheel pin 5?
//     } else {
//       analogWrite(ena, 255);
//       analogWrite(enb, 255-delta_PWM);
//     }
//     delay(400);
//   }
//   else state++;
// }

/** turnLeft (4/14/2016 Austin)
 * Turn the OSV to the desired orientation to the left */ 
void turnLeft(float orientation) {
  float tolerance = (pi/12);
  while (marker.theta - orientation < -(tolerance)) {
    RFLoop();
    motorTurnLeft();
    delay(200);
  }
}

/** turnRight (4/14/2016 Austin)
 * Turn the OSV to the desired orientation to the right */ 
void turnRight(float orientation) {
  float tolerance = (pi/12);
  while (marker.theta - orientation > tolerance) {
    RFLoop();
    motorTurnRight();
    delay(200);
  }
}

/************** Sensor Code ******************/

/** senseObstacle (4/14/2016 Austin)
 * Determines if the obsticle exist based on the ultrasonic reading
 * Caution: need to modify for better path finding, does not remember past values */ 
bool senseObstacle() {
  if (inches != 0) {return true;}
  else {return false;}
}

/**RFLoop (Keystone)
 * Read and print the x, y, theta orientation */ 
void RFLoop() {
    if(rf.receiveMarker(&marker, markerNumber))
  {
    //rf.sendMessage("\nMarker is successfully being read\n");
    Serial.println("Marker value [x,y,theta] = ["+marker.x+","+marker.y+","+marker.theta+"]");
    //rf.sendMessage("Panda Xpress is Reading Data");
  }
  else
  {
    rf.sendMessage("\nMarker is not registering\n");
    Serial.println("Marker is not registering");
    //delay(500);
  }
}

/** ping (4/28/2019 Yichao Peng)
 * Single ping for a given ultrasonic sensor
 */
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

float microsecondsToInches(long microseconds)
{  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per seconds
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PI...
  return microseconds / 73.746 / 2.0;
}

float microsecondsToCentimeters(long microseconds)
{  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  return microseconds / 29.41 / 2.0;
}

void colorSensor(){
  analogWrite(LED, 235);   //90% duty cycle for 4.5V
  delay(400);
  photoresistorSignal = analogRead(photoresist_pin); //take the reading at analog input pin
  voltage= (5.0*photoresistorSignal)/1023;           //convert the signal to a scale of 0 to 5 V
  Serial.println(voltage);
  if(voltage < colorCutoff ){                        //3.70 volt is the estimate for when you put your finger over the light sensor.
   rf.sendMessage("Green");                  // if the voltage reading is less than 3.70, the LED stays off
  } else {
   rf.sendMessage("Black");                  // if the voltage reading is more than 3.70, the LED stays on
  }
  analogWrite(LED,0);
}

void allSensors(){
  duration_left   = multiplePing(trig_left_top,ultrasound_left_pin); //average multiple ping
  duration_top    = multiplePing(trig_left_top,ultrasound_top_pin);
  duration_right  = multiplePing(trig_right_side,ultrasound_side_pin);
  
  inches_left     = microsecondsToInches(duration_left); // convert the time into a distance
  inches_top      = microsecondsToInches(duration_top);
  inches_right    = microsecondsToInches(duration_right);
  
  boulder_length  = arm_width - (inches_left + inches_right); //calculate length
  boulder_height  = arm_height - inches_top;                  //calculate height
  boulder_area    = distance * height;                        //calculate area
  
  rf.sendMessage("\nLength: " + boulder_length + "in");
  rf.sendMessage("\nHeight: " + boulder_height + "in");
  rf.sendMessage("\nSurface Area: " + boulder_area + "in^2");
  rf.sendMessage("\nColor: ");        colorSensor();
}
