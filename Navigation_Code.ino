/************* Program variables ************/
int state = 1;
float photoresistorSignal;
float duration, inches;       //ultrasonic reading
bool is_boulder_green;
float boulder_length, boulder_height, boulder_area, voltage;
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
int trig_left_top = 12;
int trig_right_side = 13;
int photoresist_pin     = A0; //analog pins
int ultrasound_left_pin = A1;
int ultrasound_top_pin  = A2;
int ultrasound_right_pin= A3;
int ultrasound_side_pin = A4;

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
  ultrasonicSetup();
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
<<<<<<< HEAD
/** RfFSetup
=======

>>>>>>> origin/master
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

void ultrasoniceSetup() {
pinMode(trig_left_top, OUTPUT);
pinMode(trig_right_side, OUTPUT);
Serial.begin(9600);
pinMode(LED, OUTPUT);
}

 //Gives off one single ping
long ultrasoundPing(){
  // Ping triggered by a HIGH pulse of 2 or more microseconds.
  // Short LOW pulse beforehand ensures a clean HIGH pulse:
  digitalWrite(trig_left_top, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_left_top, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig_left_top, LOW);
  
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  return pulseIn(ultrasound_left_pin, HIGH);
}
//Gives off one single ping
long ultrasoundPing2(){
  // Ping triggered by a HIGH pulse of 2 or more microseconds.
  // Short LOW pulse beforehand ensures a clean HIGH pulse:
  digitalWrite(trig_right_side, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_right_side, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig_right_side, LOW);
  
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  return pulseIn(ultrasound_right_pin, HIGH);
}

long ultrasoundPing3(){
  // Ping triggered by a HIGH pulse of 2 or more microseconds.
  // Short LOW pulse beforehand ensures a clean HIGH pulse:
  digitalWrite(trig_left_top, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_left_top, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig_left_top, LOW);
  
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  return pulseIn(ultrasound_top_pin, HIGH);
}

long ultrasoundPing4() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  return pulseIn(A4, HIGH);
}

float microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second). This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PI...
  return microseconds / 73.746 / 2.0;
}
float microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29.41 / 2.0;
}



float multiplePing(int trialNo, int delayTime, long lowerLimit, long upperLimit){
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

//For taking multiple pings and average them
//Filters out number outside the range of expected values, return 0

//For taking multiple pings and average them
//Filters out number outside the range of expected values, return 0
float multiplePing2(int trialNo, int delayTime, long lowerLimit, long upperLimit){
  float duration2 = 0;
  int trialNum = 0;
  for(int i=0; i<trialNo; i++){
    long d = ultrasoundPing2();
    if(d > lowerLimit && d < upperLimit){
        duration2 = duration2 + d;
        trialNum++;
    }
    delay(delayTime);
  }
  if(trialNum == 0){ return 0;
  }else{ return duration2 /1.0 / trialNum;
  }
}

float multiplePing3(int trialNo, int delayTime, long lowerLimit, long upperLimit){
  float duration = 0;
  int trialNum = 0;
  for(int i=0; i<trialNo; i++){
    long d = ultrasoundPing3();
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

/** multiplePing (for stationary measurement)
 * Created 4/4/2016 (Yichao Peng)
 * Average multiple pings to get more accuracy
 * Used for measuring boulders - removed extreme values from calculation
 */ 
float multiplePing4(int trialNo, int delayTime, long lowerLimit, long upperLimit) {
  float duration = 0;
  int trialNum = 0;
  for(int i=0; i<trialNo; i++){
    long d = ultrasoundPing4();
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


void colorSensor(){
 digitalWrite(LED, HIGH);
 photoresistorSignal = analogRead(photoresist_pin);                     //take the reading at analog input pin A1 and call it the "signal"
 voltage= (5.0*photoresistorSignal)/1023;                 //convert the signal to a scale of 0 to 5 V
 Serial.println(voltage);                   //print the signal reading on a scale of 0 to 1023 to the serial monitor
 if(voltage < 3.7 ){                        //3.70 volt is the estimate for when you put your finger over the light sensor.
   Serial.println("Green");                  // if the voltage reading is less than 3.70, the LED stays off
 } else {
   Serial.println("Light");                  // if the voltage reading is more than 3.70, the LED stays on
 }
}

void allSensors(){
  // establish variables for duration of the ping,
// and the distance result in inches and centimeters:
float duration, inches, cm, duration2, inches2, inchesTop, duration3;
duration = multiplePing(repeatNo,interval,lower,upper); //average multiple ping
duration2 = multiplePing2(repeatNo,interval,lower,upper);
duration3 = multiplePing3(repeatNo,interval,lower,upper);
// convert the time into a distance
inches = microsecondsToInches(duration);
inches2 = microsecondsToInches(duration2);
inchesTop = microsecondsToInches(duration3);
float distance = 13 - (inches + inches2);
float height = 15 - inchesTop;
float surfaceArea = distance * height;
Serial.print("Length: ");
Serial.print(distance);
Serial.print("in");
Serial.println();
Serial.print("Height: ");
Serial.print(height);
Serial.print("in");
Serial.println();
Serial.print("Surface Area: ");
Serial.print(surfaceArea);
Serial.println();
Serial.print("Color: ");

colorSensor();
Serial.println();
}
<<<<<<< HEAD






 



=======
// Control Code 
void controlX(int xRefIn, int xTermIn) {
  if (x.marker < xterm) {
    delta_y = y.marker-yref;
    theta_desired = -1/2*arctan(delta_y);
    delta_theta = marker.theta - theta_desired;
    if (delta_theta > 0) {
      analogout(left_wheel, 255);
      delta_PWM = int(abs(delta_theta*k));
      anologout(right_wheel, 255-delta_PWM); //right_wheel pin4? and leftwheel pin 5?
    } else {
      analogout(right_wheel, 255);
      delta_PWM = int(abs(delta_theta*k));
      analogout(left_wheel, 255-delta_PWM);
    }
    delta(500);
    }
}
  
  
  void controlY(int yRefIn, int yTermIn) {
  if (y.marker < yterm) {
    delta_x = x.marker-xref;
    theta_desired = -1/2*arctan(delta_x);
    delta_theta = marker.theta-pi/2 - theta_desired;
    if (delta_theta > 0) {
      analogout(left_wheel, 255);
      delta_PWM = int(abs(delta_theta*k));
      anologout(right_wheel, 255-delta_PWM); //right_wheel pin4? and leftwheel pin 5?
    } else {
      analogout(right_wheel, 255);
      delta_PWM = int(abs(delta_theta*k));
      analogout(left_wheel, 255-delta_PWM);
    }
    delta(500);
    }
}
  
  }
}
>>>>>>> pr/5
