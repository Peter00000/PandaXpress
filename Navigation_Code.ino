/************* Program variables ************/
int state = 1;
float photoresistorSignal;
float duration, duration2, duration3, inches, inches2, inchesTop;   //ultrasonic reading
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
const float arm_width   = 13;   //inches between left and right ultrasonic sensor
const float arm_height  = 15;   //inches between top ultrasonic and ground
const float color_cutoff = 3.7; //color cutoff voltage
const int k = 150;              //constant relate delta_angle to delta_PWM

/************** Pin Variables ***************/
int in3 = 2;  int in4 = 3;  
int enb = 4;  int ena = 5; 
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
          driveForwardYDirection(1); //need to update for the new control code
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
  digitalWrite(in1,LOW);    digitalWrite(in3,HIGH);
  digitalWrite(in2,HIGH);   digitalWrite(in4,LOW);
  analogWrite(ena,255);     analogWrite(enb,255);
}

void motorTurnRight() {
  digitalWrite(in1, LOW);   digitalWrite(in3, LOW);
  digitalWrite(in2, HIGH);  digitalWrite(in4, HIGH);
  analogWrite(ena,100);     analogWrite(enb,100);
}

void motorTurnLeft() {
  digitalWrite(in1, HIGH);  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);   digitalWrite(in4, LOW);
  analogWrite(ena,100);     analogWrite(enb,100);
}

/** driveForwardXDirection (4/14/2016 Austin)
 * Caution: used for fakebot only, does not move robot */ 
void driveForwardXDirection(float xValue) {
  while (marker.x < xValue) {
    motorStraight();
    RFLoop();
  }
}

/** controlX (4/28/2016 Mitchell) */
void controlX(int xRefIn, int xTermIn) {
  if (x.marker < xterm) {
    delta_y       = marker.y-yref;
    theta_desired = -1/2*arctan(delta_y);
    delta_theta   = marker.theta - theta_desired;
    delta_PWM     = int(abs(delta_theta*k));
    if (delta_theta > 0) {
      analogout(left_wheel, 255);
      anologout(right_wheel, 255-delta_PWM); 
    } else {
      analogout(right_wheel, 255);
      analogout(left_wheel, 255-delta_PWM);
    }
    delay(400);
    }
}

/** driveForwardYDirection (4/14/2016 Austin)
 * Caution: use for fakebot only, does not move robot */ 
void driveForwardYDirection(float yValue) {
  float tolerance = .05;
  while (marker.y - yValue < -(tolerance) || marker.y - yValue > tolerance) {
    motorStraight();
    RFLoop();
  }
}

/** controlY (4/28/2016 Mitchell) */
void controlY(int yRefIn, int yTermIn) {
  if (y.marker < yterm) {
    delta_x       = marker.x-xref;
    theta_desired = -1/2*arctan(delta_x);
    delta_theta   = marker.theta-pi/2 - theta_desired;
    delta_PWM     = int(abs(delta_theta*k));
    if (delta_theta > 0) {
      analogout(left_wheel, 255);
      anologout(right_wheel, 255-delta_PWM); //right_wheel pin4? and leftwheel pin 5?
    } else {
      analogout(right_wheel, 255);
      analogout(left_wheel, 255-delta_PWM);
    }
    delay(400);
  }
}

/** turnLeft (4/14/2016 Austin)
 * Turn the OSV to the desired orientation to the left */ 
void turnLeft(float orientation) {
  float tolerance = (pi/12);
  while (marker.theta - orientation < -(tolerance)) {
    RFLoop();
    motorTurnLeft();
  }
}

/** turnRight (4/14/2016 Austin)
 * Turn the OSV to the desired orientation to the right */ 
void turnRight(float orientation) {
  float tolerance = (pi/12);
  while (marker.theta - orientation > tolerance) {
    RFLoop();
    motorTurnRight();
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
    rf.sendMessage("\nMarker is successfully being read\n");
    // Serial.print("x value of marker is ");
    // Serial.println(marker.x);
    // Serial.print("y value of marker is ");
    // Serial.println(marker.y);
    // Serial.print("the orientation is ");
    // Serial.println(marker.theta);
    Serial.println("Marker value successfully being read.");
    //delay(500);   //got rid of the delay
    rf.sendMessage("Panda Xpress is Reading Data");
  }
  else
  {
    rf.sendMessage("\nMarker is not registering\n");
    Serial.println("Marker is not registering");
    //delay(500);
  }
}

 //Gives off one single ping from left sensor
long ultrasoundPing(){
  // Ping triggered by a HIGH pulse of 2 or more microseconds.
  // Short LOW pulse beforehand ensures a clean HIGH pulse:
  digitalWrite(trig_left_top, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_left_top, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig_left_top, LOW);
  return pulseIn(ultrasound_left_pin, HIGH); // Reads the pulse duration of the echo in milliseconds
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
  return pulseIn(ultrasound_right_pin, HIGH); // Reads the pulse duration of the echo in milliseconds
}

long ultrasoundPing3(){
  // Ping triggered by a HIGH pulse of 2 or more microseconds.
  // Short LOW pulse beforehand ensures a clean HIGH pulse:
  digitalWrite(trig_left_top, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_left_top, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig_left_top, LOW);
  return pulseIn(ultrasound_top_pin, HIGH); // Reads the pulse duration of the echo in milliseconds
}

long ultrasoundPing4() {
  digitalWrite(trig_right_side, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_right_side, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig_right_side, LOW);
  return pulseIn(ultrasound_side_pin, HIGH); // Reads the pulse duration of the echo in milliseconds
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

/** MultiplePing (Yichao)
 * For taking multiple pings and average them
 * Filters out number outside the range of expected values, return 0 */
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
  if(trialNum == 0){ return 0;}
  else  {return duration /1.0 / trialNum; }
}

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
 analogWrite(LED, 235);   //90% duty cycle for 4.5V
 photoresistorSignal = analogRead(photoresist_pin); //take the reading at analog input pin
 voltage= (5.0*photoresistorSignal)/1023;           //convert the signal to a scale of 0 to 5 V
 Serial.println(voltage);
 if(voltage < colorCutoff ){                        //3.70 volt is the estimate for when you put your finger over the light sensor.
   rf.sendMessage("Green");                  // if the voltage reading is less than 3.70, the LED stays off
 } else {
   rf.sendMessage("Black");                  // if the voltage reading is more than 3.70, the LED stays on
 }
}

void allSensors(){
  duration = multiplePing(repeatNo,interval,lower,upper); //average multiple ping
  duration2 = multiplePing2(repeatNo,interval,lower,upper);
  duration3 = multiplePing3(repeatNo,interval,lower,upper);
  
  inches = microsecondsToInches(duration); // convert the time into a distance
  inches2 = microsecondsToInches(duration2);
  inchesTop = microsecondsToInches(duration3);
  
  float distance = arm_width - (inches + inches2);   //calculate length
  float height = arm_height - inchesTop;              //calculate height
  float surfaceArea = distance * height;      //calculate area
  
  rf.sendMessage("\nLength: " + distance + "in");
  rf.sendMessage("\nHeight: " + height + "in");
  rf.sendMessage("\nSurface Area: " + surfaceArea + "in^2");
  rf.sendMessage("\nColor: ");        colorSensor();
}
