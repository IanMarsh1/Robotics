#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Motors motors;
Buzzer buzzer;
Servo headServo; 


//init encoders
unsigned long encodersCurrent, encodersPrevious;
const unsigned long ENCODER_PERIOD = 20;

//init motors
float leftSpeed = 0.0f;
float rightSpeed = 0.0f;
const float BASE_SPEED = 65.0f;
const float MIN_SPEED = 40.0f;
const float MAX_SPEED = 110.0f;

//normalize motor speed
//TODO maybe adjust to straighten out???
const float MOTOR_FACTOR = BASE_SPEED / 97.0f;

//init time intervals 
unsigned long Time1;
unsigned long Time2;
const unsigned long MOTOR_PERIOD = 20;

//init wheel rotations
long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

//measured wheel rotations
float sL = 0.0f;
float sR = 0.0f;

//prev measured wheel rotations
float prevSL = 0.0f;
float prevSR = 0.0f;

//Delta Distance Traveled
float sL_Delta = 0.0f;
float sR_Delta = 0.0f;

//coordinate pair
float x = 0.0f;
float y = 0.0f;

//theta
float theta = 3.14159f;

//delta Positions of last 2 
//float xDelta = 0.0f;
//float yDelta = 0.0f;
float DeltaTheta = 0.0f;
//delta of distance of last 2
//float DeltaPos = 0.0f;

//init spirit airlines bot info
const float CLICKS_PER_ROTATION = 12.0f;
const float GEAR_RATIO = 75.81f;
const float WHEEL_CIRCUMFRENCE = 10.0531f;

//init location 
const float SLOW_DOWN_POINT = 20.0f;
const float divisor = 8.5f;

//init current goal (Mostly from Gormanly)
int currentGoal = 0;
//amount of goals 
const int NUMBER_OF_GOALS = 1;
//goals
float xGoals[NUMBER_OF_GOALS] = { 120.0f };
float yGoals[NUMBER_OF_GOALS] = { -60.0f };
float xGoal = xGoals[currentGoal];
float yGoal = yGoals[currentGoal];

//error calculations
float goalPrecision = 1.0f;
float startGoalDistance = sqrt(sq(xGoal - x) + sq(y - yGoal));
float currentGoalDistance = startGoalDistance;
// change speed based on distance
float distanceFactor = startGoalDistance / 2.0f;


//PID data
const float KP = 100.0f; //proportion
float PIDfix = 0.0f;
//PID error
float currentError = 0.0f;
float errorMagnitude = 0.0f;


//Head Servo timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 250; //500 = half second movements
const boolean HEAD_DEBUG = false;


//head servo constants
const int HEAD_SERVO_PIN = 20;
const int NUM_HEAD_POSITIONS = 7;
//const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {145, 130, 115, 90, 65, 50, 35};

//Head positions for my bot - my bots a little special 
//  range of servo before hitting wires: 40 -- 160 *
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {160, 138.2, 116.6, 95, 76.7, 58.4, 40};  
//const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {135, 90, 45};

//head servo data 
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

const float HEAD_MAGNITUDES[NUM_HEAD_POSITIONS] = {0.1, 0.25, 0.5, 1.0, 0.25, 0.5, 0.1};
//const int HEAD_MAGNITUDES[NUM_HEAD_POSITIONS] = {1, 2, 3, 4, 3, 2, 1};


//Sensor part
//Initialize Ultrasonic
const int ECHO_PIN = 18;
const int TRIG_PIN = 12;

//Ultrasonic Max Distance;
const float MAX_DISTANCE = 100.0;

//Determine normalization factor based on MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE/ 100;
const float STOP_DISTANCE = 5;
float distance = 0;

//Ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 50; //time to wait between checking US sensor


//setup
void setup() {
  Serial.begin(9600);
  delay(1000);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(40);
}

//loop where code runs
void loop(){
  if (currentGoal < NUMBER_OF_GOALS){
    checkEncoders();
    setMotors(PIDfix);  //Motors to make it to goal
    moveHead();
    usReadCm();
    if(distance <= 30){
      adjustMotors();     //Obstacle avoidance motors 
      setMotors(PIDfix);
    }
    GoalStatus();
  }
  if(currentGoal == NUMBER_OF_GOALS){
    motors.setSpeeds(0,0);
  }

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
    Serial.println (" cm ");

    Serial.print("  Servo Pos ");
    Serial.println(currentHeadPosition);

    //update prevmillis
    usPm = usCm;

  }
}


void adjustMotors(){
  //use distance and currentHeadPos

  //currHeadPos = num 0 -> 6
  float adjustment = distance * HEAD_MAGNITUDES[currentHeadPosition];

  // <3 means object is to the left of the bot 
  if(currentHeadPosition < 3){
    leftSpeed -= adjustment;
    rightSpeed += adjustment;
  }  
  else{ //otherwise, object is right or FRONT of the bot
    leftSpeed += adjustment;
    rightSpeed -= adjustment;
  }

  //motors.setSpeeds(-leftSpeed, -rightSpeed);
}

//moves spirit airlines bot (from previous lab)
void checkEncoders(){
  encodersCurrent = millis();

  if (encodersCurrent > encodersPrevious + ENCODER_PERIOD){

    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    prevSL = sL;
    prevSR = sR;

    sL += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);
    sR += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);

    sL_Delta = sL - prevSL;
    sR_Delta = sR - prevSR;

    prevLeft = countsLeft;
    prevRight = countsRight;

    encodersCurrent = encodersPrevious;

    UpdatePosition();
  }
}

//update motor speed (from previous lab)
void setMotors(float controllerOutput){
  Time1 = millis();

  if (Time1 > Time2 + MOTOR_PERIOD){
    float targetDistance = (startGoalDistance - currentGoalDistance);
    leftSpeed = ThetaController(controllerOutput, 1, targetDistance);
    rightSpeed = ThetaController(controllerOutput, -1, targetDistance);

    motors.setSpeeds(-leftSpeed, -rightSpeed);
    Time2 = Time1;
  }
}

//updates coordinate pair and their Changes
void UpdatePosition(){
  // calculate trig functions needed for position update
  float sinThetaDelta = sin(theta + DeltaTheta / 2.0f);
  float cosThetaDelta = cos(theta + DeltaTheta / 2.0f);

  // update the current location
  x += (sL_Delta + sR_Delta) / 2.0f * cosThetaDelta;
  y += (sL_Delta + sR_Delta) / 2.0f * sinThetaDelta;
  theta += (sR_Delta - sL_Delta) / divisor;
  currentGoalDistance = sqrt(sq(xGoal - x) + sq(y - yGoal));

  // send position data to PID controller to get a correction
  PIDfixer();
}


//fix pid
void PIDfixer(){
  currentError = theta - atan2(yGoal - y, xGoal - x);
  currentError = atan2(sin(currentError), cos(currentError));
  //currentError = atan2(sin(currentError), cos(currentError)) - theta;
  PIDfix = KP * currentError;
}

//check goal status 
void GoalStatus(){
  
  bool Completed = false;

  //check if current goal is met 
  if ((xGoal - goalPrecision <= x && xGoal + goalPrecision >= x) 
      && (yGoal - goalPrecision <= y 
        && yGoal + goalPrecision >= y)){
      Completed = true;
      motors.setSpeeds(0, 0);
      if(currentGoal == 0){
        buzzer.play("c32");
      }else if(currentGoal == 1){
        buzzer.play("c32");
      }
       // TODO MAKE THIS different for each goal 
      delay(1000);
  }

// check for completion
  if (Completed){
    // increment goals
    currentGoal++;
    if (currentGoal == NUMBER_OF_GOALS){
      // stop at final
      motors.setSpeeds(0, 0);
      ledGreen(1);
      buzzer.play("g32");
      delay(1000);
    }
    else{
      // set next goal
      xGoal = xGoals[currentGoal];
      yGoal = yGoals[currentGoal];
      
      // update start distance
      startGoalDistance = sqrt(sq(xGoal - x) + sq(y - yGoal));
    }
  }
}

//theta control
float ThetaController(float thetaCorrection, int direction, float targetDistance){
  // update error on distance
  errorMagnitude = targetDistance / distanceFactor;
  errorMagnitude = atan2(sin(errorMagnitude), cos(errorMagnitude)); //gormanly todo

 // Calculate theta corrected speed
  float wheelSpeed = BASE_SPEED + errorMagnitude * distanceFactor + thetaCorrection * direction;

  // Apply speed limits
  wheelSpeed = max(wheelSpeed, MIN_SPEED);
  wheelSpeed = min(wheelSpeed, MAX_SPEED);

  return wheelSpeed;
}