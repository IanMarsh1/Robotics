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
const unsigned long HEAD_MOVEMENT_PERIOD = 1000;

const int HEAD_SERVO_PIN = 20;
const int NUM_HEAD_POSITIONS = 2;

boolean flag;


const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {25, 90};

boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

// Code 2a
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;
int distance = 0;


const int MAX_DISTANCE = 200;


unsigned long currentMillis;
unsigned long previousMillis;
const unsigned long US_PERIOD = 100;


// desired x position
double desiredState = (double) 20;

double desiredFrontState = (double) 0.0;

double kp = 4;
double ki = 0;
double kd = .4;
// track sep?
double frontKp = 0; // check
double kiTotal = 0.0;
double previousError = 0.0;



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

  pid(); 
  
}



void pid(){
  if(HEAD_POSITIONS[currentHeadPosition] == 90){
    double error = desiredState - distance;

    double proportional = kp * error;

    kiTotal += error;
    if (kiTotal > 50){
      kiTotal = 50;
    }
    
    double integral = ki * kiTotal;
    
    float derivative = kd * (error - previousError);
    
    previousError = error;

    float pidResult = proportional + integral + derivative;
    
    Serial.println(pidResult);
    
    double leftSpeed = 100 + pidResult;
    double rightSpeed = 100 - pidResult;
    if (leftSpeed < 0)
      leftSpeed = 100;
    if (rightSpeed < 0)
      rightSpeed = 100;
    if (leftSpeed > 200)
      leftSpeed = 200;
    if (rightSpeed > 200)
      rightSpeed = 200;
    
    motors.setSpeeds(-leftSpeed, -rightSpeed);
  }
  else{
    Serial.print("poop");
  }
}











void moveHead(){
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

void usReadCm(){
  currentMillis = millis();
  if(currentMillis > previousMillis + US_PERIOD && !flag){

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

    //Serial.print("Distance: ");
    //Serial.print(distance);
    //Serial.println(" cm");

    /*
    Serial.println(currentMillis);
    Serial.println(millis());
    */

    //update prevmils
    previousMillis = currentMillis;
    flag = true;
  }

}


