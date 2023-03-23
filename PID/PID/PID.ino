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
const unsigned long HEAD_MOVEMENT_PERIOD = 100;

const int HEAD_SERVO_PIN = 20;
const int NUM_HEAD_POSITIONS = 1;

boolean flag;


const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {25};

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

double kp = 50;
double ki = 0;
double kd = 0;
// track sep?
double frontKp = 0; // check
double kiTotal = 0.0;
double priorError = 0.0;








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

  //motors.setSpeeds(25, 25);

  // How are we going to track samples at various angles?
  // what about infinity? readings from walls beyond max distance?
  // get the error
  // How will angles apply????? (forgeting the left turn for a moment, what
  // about readings at angles other than 90??)
  double error = desiredState - distance;//desiredState - currentReading;
  double speed = error * kp;
  if (error == 0){
    motors.setSpeeds(speed, speed);
  }
  else if (error > 0){
    motors.setSpeeds(speed- 5, speed);
  }
  else{
    motors.setSpeeds(speed, speed- 5);
  }
  // determine error for left turn (on coming wall) if separate
  // determine how much time has passed since last cycle
  // Kp get the proportional correction
  // double kpResult = kp * error * timePassed;
  // version without time
  // double kpResult = kp * error;
  // Ki get integral correction
  // first add the current error to the sum
  kiTotal += error;
  // Perhaps we should apply limits on Ki?
  // (Remember the section on preventing intregral windup?)
  // next get the KiRelsult by applying our tuning constant to the total
  double kiResult = ki * kiTotal;
  // Kd: derivative
  // Remember derivative is the differece of the error and the
  // priorError, divided by time passed between readings.
  // remember order of operations! Don't divide the priorError and add error!
  // sum P, I and D together
  // Apply the sum to the plant (motors)
  // remember one will will be +pidSum, the other -pidSum
  // pidSum will be either positive or negative which will
  // determine direction
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

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    /*
    Serial.println(currentMillis);
    Serial.println(millis());
    */

    //update prevmils
    previousMillis = currentMillis;
    flag = true;
  }
}


