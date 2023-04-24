#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Encoders encoders;
Motors motors;
Buzzer buzzer;

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
float theta = 0.0f;

//delta Positions of last 2 
float xDelta = 0.0f;
float yDelta = 0.0f;
float DeltaTheta = 0.0f;
//delta of distance of last 2
float DeltaPos = 0.0f;

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
const int NUMBER_OF_GOALS = 3;
//goals
float xGoals[NUMBER_OF_GOALS] = { 30.0f, 30.0f, 0.0f };
float yGoals[NUMBER_OF_GOALS] = { 30.0f, 60.0f, 0.0f };
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

//setup
void setup() {
  Serial.begin(9600);
  delay(1000);
}

//loop where code runs
void loop(){
    if (currentGoal < NUMBER_OF_GOALS){
      checkEncoders();
      setMotors(PIDfix);
      GoalStatus();
    }
    if(currentGoal == NUMBER_OF_GOALS){
      motors.setSpeeds(0,0);
    }
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

    motors.setSpeeds(leftSpeed, rightSpeed);
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