/************* Program variables ************/
int state = 1;
float duration, inches;       //ultrasonic reading
bool is_boulder_green;
float boulder_length, boulder_height, boulder_area; 
//
/**************** Constants *****************/
const int markerNumber = 113; //update this when receive new marker
const float pi      = 3.14159;
const int repeatNo  = 5;      //for multiple ultrasonic read
const int interval  = 200;    //ms wait
const long upper    = 1200;   //filter out unwanted data in ultrasonic reading
const long lower    = 150;    //only apply when averaging multiple trials

/************** Pin Variables ***************/
int in3 = 2;  int in4 = 3; //Kaitlin, Maria, please verify the digital pin asssignments
int enb = 4;  //left and top
int ena = 5; //right and side
int in1 = 6;  int in2 = 7;
int LED = 9;  
int RF_TX = 10;
int RF_RX = 11;
int photoresist_pin     = 0;
int ultrasound_left_pin = 1;
int ultrasound_top_pin  = 2;
int ultrasound_right_pin= 3;
int ultrasound_side_pin = 4;

/********* Fakebot testing pin var **********/
// int ena = 5;  int enb = 4; //need better name for the motor pins
// int in1 = 6;  int in3 = 2; 
// int in2 = 7;  int in4 = 3;
// int trig = 3;       //attach pin 3 to Trig
// int echo = 4;       //attach pin 4 to Echo
// int trig2 = 5;      //attach pin 5 to Trig on second sensor
// int echo2 = 6;      //attach pin 6 to Echo on second sensor 
// int green = 10;
// int red = 11;

/********* Initiate RF GPS System ***********/
#include <SoftwareSerial.h>
#include "enes100.h"
SoftwareSerial mySerial(RF_RX, RF_TX); //RX is pin 8, TX is pin 9
enes100::RfClient<SoftwareSerial> rf(&mySerial); 
enes100::Marker marker;

/*************** Main Code ******************/

/** Setup
 * created 4/14/2016    by Austin
 * initiate all system variables */
void setup() {
  Serial.begin(9600);
  RFSetup();
  motorSetup();
  singleSensorSetup();
  pinMode(green, OUTPUT);
  pinMode(red, OUTPUT);
}

/** loop
 * created 4/14/2016    by Austin
 * main loop that navigate robot through different stages
 * use the variable 'state' to track stages of navigation
 */
void loop() {
  RFLoop();
  switch(state) {
    case 0:
      break;
    case 1: //OSV is in landing zone
      turnRight(0);
      driveForwardXDirection(1);
      state = 2;
      break;
    case 2: //OSV is at edge of landing zone
      boolean obstacle;
      obstacle = senseObstacle();
      //turnLeft(pi/2);
      if(obstacle == true){
        if(marker.y > 1){
          turnRight(-pi/2);
          driveForwardYDirection(1);
          turnLeft(0);
          break;
        } else {
          turnLeft(pi/2);
          driveForwardYDirection(1);
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
          driveForwardYDirection(1);
          turnLeft(0);
            
        } else {
          turnLeft(pi/2);
          driveForwardYDirection(1);
          turnRight(0);
        }
      }
      driveForwardXDirection(3.5);
      state = 4;
      break;
    case 4://OSV is now at the x value of the Terrain Site
      turnLeft(pi/2);
      driveForwardYDirection(1.5);
      turnRight(0);
      state = 0;
      break;  
  }

}

/************** Setup Code ******************/
/** RFSetup
 * Created no date (Keystone)
 * Modified 4/17/2016 (Yichao Peng)
 * The program setup the RF communication with GPS system
 */
void RFSetup() {
   mySerial.begin(9600); //this establishes serial communication with
                        //something other than serial monitor, in this
                        //case RF communication with mission control
  //Serial.begin(9600); //already called by main program

  pinMode(RF_RX, INPUT); //since pin 8 is RX, it receives as an input
  pinMode(RF_TX, OUTPUT); //since pin 9 is TX, it transmits as an output
  
  rf.resetServer();
  rf.sendMessage("\nTeam PandaXpress is Connected\n"); //sent to mission control
  Serial.println("Team PandaXpress is Connected"); //sent to student's serial monitor
  //delay(1000);
}

/** singleSensorSetup
 * setting up ultasonic sensor
 */ 
void singleSensorSetup() {
  pinMode(trig, OUTPUT);
  pinMode(A1,INPUT);    //this should not be hard coded
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
}

/** motorSetup
 * Created (Paulo)
 * Modified 4/14/2016 (Austin)
 * setting up ultasonic sensor
 */ 
void motorSetup() {
  pinMode(ena,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(enb,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
}

/************** Motion Code *****************/
void motorStraight() {
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  analogWrite(ena,255);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  analogWrite(enb,255);
}

void motorTurnRight() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena,100);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enb,100);
}

void motorTurnLeft() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena,100);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enb,100);
}

/** driveForwardXDirection
 * Created 4/14/2016 (Austin)
 * Caution: used for fakebot only, does not move robot
 */ 
void driveForwardXDirection(float xValue) {
  while (marker.x < xValue) {
    motorStraight();
    RFLoop();
    //digitalWrite(green, HIGH);
    //digitalWrite(red, HIGH);
  }
}

/** driveForwardYDirection
 * Created 4/14/2016 (Austin)
 * Caution: use for fakebot only, does not move robot
 */ 
void driveForwardYDirection(float yValue) {
  float tolerance = .05;
  while (marker.y - yValue < -(tolerance) || marker.y - yValue > tolerance) {
    motorStraight();
    RFLoop();
    //digitalWrite(green, HIGH);
    //digitalWrite(red, HIGH);
  }
}

/** turnLeft
 * Created 4/14/2016 (Austin)
 * Turn the OSV to the desired orientation to the left
 * Caution: modify code for real robot
 */ 
void turnLeft(float orientation) {
  float tolerance = (pi/12);
  while (marker.theta - orientation < -(tolerance)) {
    motorTurnLeft();
    RFLoop();
    //digitalWrite(green, LOW);
    //digitalWrite(red, HIGH);
  }
}

/** turnRight
 * Created 4/14/2016 (Austin)
 * Turn the OSV to the desired orientation to the right
 * Caution: modify code for real robot
 */ 
void turnRight(float orientation) {
  float tolerance = (pi/12);
  while (marker.theta - orientation > tolerance) {
    motorTurnRight();
    RFLoop();
    //digitalWrite(green, HIGH);
    //digitalWrite(red, LOW);
  }
}

/************** Sensor Code ******************/

/** senseObstacle
 * Created 4/14/2016 (Austin)
 * Modified 
 * Determines if the obsticle exist based on the ultrasonic reading
 * Caution: need to modify for better path finding, does not remember past values
 */ 
boolean senseObstacle() {
  boolean obstacle;
  if (inches != 0) {
    obstacle = true;
  } else {
    obstacle = false;
  }
}

/**RFLoop
 * Created no date (Keystone)
 * Read and print the x, y, theta orientation
 * Caution: need to modify to return value or save value in global variable
 */ 
void RFLoop() {
    if(rf.receiveMarker(&marker, markerNumber)) //see if marker 104 is received
  {
    rf.sendMessage("\nMarker is successfully being read\n");
    Serial.print("x value of marker is ");
    Serial.println(marker.x);
    Serial.print("y value of marker is ");
    Serial.println(marker.y);
    Serial.print("the orientation is ");
    Serial.println(marker.theta);
    //delay(500);   //got rid of the delay
    rf.sendMessage("Panda Xpress Rocks");
  }
  else
  {
    rf.sendMessage("\nMarker is not registering\n");
    Serial.println("Marker is not registering");
    //delay(500);
  }
}

/** singleSensorLoop
 * Calls display distance value from multiple sensor calls
 * Caution: do not use when robot is moving
 */ 
void singleSensorLoop() {
  duration = multiplePing(repeatNo,interval,lower,upper); //average multiple ping
  // convert the time into a distance
  inches = microsecondsToInches(duration);
  Serial.print(inches);
  Serial.print(" ");
  Serial.print("in");
  Serial.println();
}

/** multiplePing (for stationary measurement)
 * Created 4/4/2016 (Yichao Peng)
 * Average multiple pings to get more accuracy
 * Used for measuring boulders - removed extreme values from calculation
 */ 
float multiplePing(int trialNo, int delayTime, long lowerLimit, long upperLimit) {
  float duration = 0;
  int trialNum = 0;
  for(int i=0; i<trialNo; i++){
    long d = ultrasoundPing();
    if(d > lowerLimit && d < upperLimit){
        duration = duration + d;
        trialNum++;
    }
    delay(delayTime);
  }
  if(trialNum == 0){ return 0;
  }else{ return duration /1.0 / trialNum;
  }
}

/** ultrasoundPing (single ping for on the move)
 * (http://www.instructables.com/id/Easy-ultrasonic-4-pin-sensor-monitoring-hc-sr04/)
 * Code triggers the ultrasonic sensor and reads the return raw duration in milliseconds
 * Caution: only can read one sensor per round
 * Caution: hard coded for A! now
 */ 
long ultrasoundPing() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  return pulseIn(A1, HIGH);
}

/** microsecondToInches
 * (http://www.instructables.com/id/Easy-ultrasonic-4-pin-sensor-monitoring-hc-sr04/)
 * Return number of inches to the object
 */ 
float microsecondsToInches(long microseconds) {
  return microseconds / 73.746 / 2.0;
}
