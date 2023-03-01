#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Servo headServo; 

//Switches
const boolean HEAD_DEBUG = false;

//Head Servo timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 500; //500 = half second movements

//head servo constants
const int HEAD_SERVO_PIN = 20;
const int NUM_HEAD_POSITIONS = 3;
//const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {135, 120, 105, 90, 75, 60, 45};
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {135, 90, 45};


//head servo data 
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;



//Sensor part
//Initialize Ultrasonic
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;

//Ultrasonic Max Distance;
const float MAX_DISTANCE = 100.0;

//Determine normalization factor based on MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE/ 100;
const float STOP_DISTANCE = 5;

//Ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 50; //time to wait between checking US sensor

//Motor Timing 
unsigned long motorCm;
unsigned long motorPm; 
const unsigned long MOTOR_PERIOD = 20; //time to wait between adjusting motor speed 

//current US distance reading
float distance = 0;

void setup() {
  Serial.begin(57600);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  
  //initialize head position to start
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(90);
  //headServo.write(90);  //test for being dead center 

  //start delay
  delay(100);
  buzzer.play("c32");

}

void loop() {
  moveHead();
  //delay(1000);
  usReadCm();
}

void moveHead(){
  headCm = millis();
  if(headCm > headPm + HEAD_MOVEMENT_PERIOD){

    if(HEAD_DEBUG) {
      Serial.print(currentHeadPosition);
      Serial.print(" - ");
      Serial.println(HEAD_POSITIONS[currentHeadPosition]);
    }

    //position head to current position in array
    headServo.write( HEAD_POSITIONS[currentHeadPosition] );

    //Set next head position
    // moves servo to next head position and changes direction when needed

    if(headDirectionClockwise){
      if(currentHeadPosition >= (NUM_HEAD_POSITIONS -1)) {
         headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition --;
      }
      else{
        currentHeadPosition ++;
      }
    }
    else {
      if(currentHeadPosition <= 0){
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition++;
      }
      else {
        currentHeadPosition --; 
      }
    }

    //reset previous millis
    headPm = headCm;
  }
}

void usReadCm(){
  usCm = millis();
  if(usCm > usPm + US_PERIOD){
     
    //clear trigpin (set low)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    //Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);    

    //Reads the ECHO_PIN returns sound wave travel time in microseconds
    //duration = 38000 microseconds - allows for reading up to max distance supported by sensor
    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
    //calculate distance
    distance = duration * 0.034 / 2; //time of flight eq: speed of sound wave / 2

    //apply limits
    if(distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if(distance == 0) distance = MAX_DISTANCE;

    //displays distance on Serial Monitor
    Serial.print(" Distance: ");
    Serial.print(distance);
    Serial.println(" cm ");

    //update prevmillis
    usPm = usCm;

  }
}








