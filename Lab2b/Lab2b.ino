#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;

//Initialize Ultrasonic
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;

//Ultrasonic Max Distance
const float MAX_DISTANCE = 100.0;

//Determine normalization factor based on MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE/ 100;
const float STOP_DISTANCE = 5;

//Motor Constants
const float MOTOR_BASE_SPEED = 200.0;
const int MOTOR_MIN_SPEED = 30;
//determine normalization factor based on MOTOR_BASE_SPEED
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

// Motor comensation (swap r/l)
const float L_MOTOR_FACTOR = 1.0;
const float R_MOTOR_FACTOR = 1.0;
const float L_MOTOR_FACTOR_THRESHOLD = 150;
const float R_MOTOR_FACTOR_THRESHOLD = 150;

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
  // put your setup code here, to run once:

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  delay(1000);
  buzzer.play("c32");

}

void loop() {
  // put your main code here, to run repeatedly:

  //Update current distance 
  usReadCm();

  //Update motor speed 
  setMotors();
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


void setMotors(){
    motorCm = millis();
    if(motorCm > motorPm + MOTOR_PERIOD){

      //Start with MOTOR_BASE_SPEED
      float leftSpeed = MOTOR_BASE_SPEED;
      float rightSpeed = MOTOR_BASE_SPEED;

      //check to see if most current distance measurement is less than /equal to MAX_DISTANCE
      if(distance < MAX_DISTANCE){
        //determine magnitude of distance by difference (short distance = high magnitude)
        //divide by DISTANCE_FACTOR to ensure uniform response as MAX_DISTANCE changes
        // this maps the distance range (1 - MAX_RANGE) to 0-100 for the magnitude 
        float magnitude = (float)(MAX_DISTANCE - distance) / DISTANCE_FACTOR;
        //ex 1: MAX_DISTANCE = 80, distance = 40: 80-40 = 40/.8 = 50 (mid range)
        //ex 2: MAX_DISTANCE = 160, distance = 40: 160-40 = 120 / 1.6 = 75 (top 1/4)

        leftSpeed = MOTOR_BASE_SPEED - (magnitude  * MOTOR_FACTOR);
        rightSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
      }      

    // add in motor comp
    if (leftSpeed <= L_MOTOR_FACTOR_THRESHOLD){
      leftSpeed *= L_MOTOR_FACTOR;   
    }
    if (rightSpeed <= R_MOTOR_FACTOR_THRESHOLD){
      rightSpeed *= R_MOTOR_FACTOR;   
    }


     //lower limit check 
     if(leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
     if(rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;

     //check stop distance 
     if(distance <= STOP_DISTANCE) leftSpeed = 0;
     if(distance <= STOP_DISTANCE) rightSpeed = 0;

     Serial.print("Left: ");
     Serial.print(leftSpeed);
     Serial.print(" Right: ");
     Serial.print(rightSpeed);

     motors.setSpeeds(-leftSpeed, -rightSpeed);

     motorPm = motorCm;
    }
  }


