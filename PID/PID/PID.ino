#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

const float MOTOR_BASE_SPEED = 300.0;
const int MOTOR_MIN_SPEED = 30;

unsigned long motorCm;
unsigned long motorPm; 

Buzzer buzzer;
Servo headServo;
Motors motors;

unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 240;

const int HEAD_SERVO_PIN = 20;
const int NUM_HEAD_POSITIONS = 3;

// flag is used to make sure we do not use the US while the servo is moving
boolean flag;
// front flag is set true if we have an object within 30 cm
boolean frontFlag;

// the first HEAD_POSITIONS are used when nothing is in within 30 cm
// once something is in 30 cm we change to 2 and then back 2 one when we get around the object
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {90, 5, 5};
const int HEAD_POSITIONS2[NUM_HEAD_POSITIONS] = {55, 5, 5};

boolean headDirectionClockwise = true;
// we start with the us to the right
int currentHeadPosition = 1;

// Code 2a
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;
int distance = 0;
int fdistance = 0;

const int MAX_DISTANCE = 200;

unsigned long currentMillis;
unsigned long previousMillis;
const unsigned long US_PERIOD = 100;

// desired x position
double desiredState = (double) 20;
double desiredFrontState = (double) 20;

// wall PID numbers 5 0 .5
double kp = 5;
double ki = 0;
double kd = .5;
// front PID numbers 100 0 12
double fkp = 100;
double fki = 0;
double fkd = 12; 

double kiTotal = 0.0;
double fkiTotal = 0.0;
double previousError = 0.0;
double fpreviousError = 0.0;


void setup(){
  Serial.begin(57600);

  // Servo
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(40);

  // US
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  // start delay
  delay(3000);
  buzzer.play("c32");
  

}
void loop() {
  moveHead();

  usReadCm();

  // if nothing is in front then I care about the wall
  if (!frontFlag){
    pid(); 
  }
  // if something is in front move till it is not there anymore
  else if(frontFlag){
    frontpid();
  }
}

void pid(){
    double error = desiredState - distance;

    double proportional = kp * error;

    // tying to pervent windup
    kiTotal += error;
    if (kiTotal > 50){
      kiTotal = 0;
    }
    
    double integral = ki * kiTotal;
    
    float derivative = kd * (error - previousError);
    
    previousError = error;

    float pidResult = proportional + integral + derivative;
    
    // got this code from chat GPT and is used to apply speed from the pidResult from a base speed of 100
    double leftSpeed = 100 + pidResult;
    double rightSpeed = 100 - pidResult;
    if (leftSpeed < 0)
      leftSpeed = 110;
    if (rightSpeed < 0)
      rightSpeed = 110;
    if (leftSpeed > 200)
      leftSpeed = 160;
    if (rightSpeed > 200)
      rightSpeed = 160;
    
    motors.setSpeeds(-leftSpeed, -rightSpeed);
}
void frontpid(){
    double error = desiredFrontState - fdistance;

    double proportional = kp * error;

    // tying to pervent windup
    fkiTotal += error;
    if (fkiTotal > 50){
      fkiTotal = 50;
    }
    
    double integral = ki * fkiTotal;
    
    float derivative = kd * (error - fpreviousError);
    
    fpreviousError = error;

    float pidResult = proportional + integral + derivative;

    // got this code from chat GPT and is used to apply speed from the pidResult from a base speed of 100 
    // the right has a speed of 25 because I could not get it to turn fast but im am not sure if I can change that
    double leftSpeed = 100 + pidResult;
    double rightSpeed = 25 - pidResult;

    if (leftSpeed < -50)
      leftSpeed = -50;
    if (rightSpeed < -50)
      rightSpeed = -50;
    if (leftSpeed > 200)
      leftSpeed = 200;
    if (rightSpeed > 200)
      rightSpeed = 200;
    if (leftSpeed > rightSpeed){
      motors.setSpeeds(-leftSpeed, -rightSpeed);
    }
}

void moveHead(){
  // the flag is to change the HEAD_POSITIONS array
  if(frontFlag){
    headCm = millis();
    if (headCm > headPm + HEAD_MOVEMENT_PERIOD){

      // position head to the current position in the array
      headServo.write(HEAD_POSITIONS2[currentHeadPosition]);

      /**
        * Set next head position
        * Moves servo to the next head position and changes dirction when needed
        */
      if(headDirectionClockwise){
        if(currentHeadPosition >= (NUM_HEAD_POSITIONS - 1)){
          headDirectionClockwise = !headDirectionClockwise;
          currentHeadPosition--;
        }
        else {
          currentHeadPosition++;
        }
      }
      else {
        if(currentHeadPosition <= 0){
          headDirectionClockwise = !headDirectionClockwise;
          currentHeadPosition++;
        }
        else {
          currentHeadPosition--;
        }
      }      
      headPm = headCm;
      flag = false;
    }
  }
  else{
    headCm = millis();
    if (headCm > headPm + HEAD_MOVEMENT_PERIOD){

      // position head to the current position in the array
      headServo.write(HEAD_POSITIONS[currentHeadPosition]);

      /**
        * Set next head position
        * Moves servo to the next head position and changes dirction when needed
        */
      if(headDirectionClockwise){
        if(currentHeadPosition >= (NUM_HEAD_POSITIONS - 1)){
          headDirectionClockwise = !headDirectionClockwise;
          currentHeadPosition--;
        }
        else {
          currentHeadPosition++;
        }
      }
      else {
        if(currentHeadPosition <= 0){
          headDirectionClockwise = !headDirectionClockwise;
          currentHeadPosition++;
        }
        else {
          currentHeadPosition--;
        }
      }      
      headPm = headCm;
      flag = false;
    }
  }
}

void usReadCm(){
  // I have no idea why I need to look at the currentHeadPosition after it points to the front for it to work but it kinda works
  if((currentHeadPosition == 1) && headDirectionClockwise){
    currentMillis = millis();
    if(currentMillis > previousMillis + US_PERIOD && !flag){ // got rid of flag

      //clears the TRIG_PIN (set low)
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);

      // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
      // note the duration (38000 microseconds) that will allow for reading up max distance supporteed by the sensor
      long duration = pulseIn(ECHO_PIN, HIGH, 38000);
      // calc disitace
      distance = duration *0.034 / 2; // Time of fligh eq: speed of sound /2

      // apply limits
      if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
      if (distance == 0) distance = MAX_DISTANCE;

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      //update prevmils
      previousMillis = currentMillis;
      flag = true;
    }
  }
   if((currentHeadPosition == 2) && headDirectionClockwise){
    currentMillis = millis();
    if(currentMillis > previousMillis + US_PERIOD && !flag){ // got rid of flag

      //clears the TRIG_PIN (set low)
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);

      // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
      // note the duration (38000 microseconds) that will allow for reading up max distance supporteed by the sensor
      long duration = pulseIn(ECHO_PIN, HIGH, 38000);
      // calc disitace
      fdistance = duration *0.034 / 2; // Time of fligh eq: speed of sound /2

      // apply limits
      if (fdistance > MAX_DISTANCE) fdistance = MAX_DISTANCE;
      if (fdistance == 0) fdistance = MAX_DISTANCE;

      Serial.print("Front Distance: ");
      Serial.print(fdistance);
      Serial.println(" cm");

      if (fdistance > 25){
        frontFlag = false;
      }
      else{
        frontFlag = true;
      }

      //update prevmils
      previousMillis = currentMillis;
      flag = true;
    }
  }
  else{
    currentMillis = millis();
    if(currentMillis > previousMillis + US_PERIOD && !flag){ // got rid of flag

      //clears the TRIG_PIN (set low)
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);

      // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
      // note the duration (38000 microseconds) that will allow for reading up max distance supporteed by the sensor
      long duration = pulseIn(ECHO_PIN, HIGH, 38000);
      // calc disitace
      distance = duration *0.034 / 2; // Time of fligh eq: speed of sound /2

      // apply limits
      if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
      if (distance == 0) distance = MAX_DISTANCE;

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      //update prevmils
      previousMillis = currentMillis;
      flag = true;
    }
  }
}