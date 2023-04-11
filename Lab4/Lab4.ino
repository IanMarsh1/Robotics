#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;

const float MOTOR_BASE_SPEED = 100.0;
const int MOTOR_MIN_SPEED = 30;
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;  //miliseconds - how often we check encoders

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

const int CLICKS_PER_ROTATION = 12;  //Rotation of DC motor
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;  //3.2 cm
const int WHEEL_CIRCUMFERENCE = 10.0531;

float Sl = 0.0F;
float Sr = 0.0F;

unsigned long motorCm;
unsigned long motorPm;

// goals
const int NUMBER_OF_GOALS = 3;
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};
int curGoal = 0;

const float pi = 3.14159;

const float DIST_PER_TICK = 3.2*pi / 909.72; //3.2cm diameter wheel with 909.72 CPR
const int baseRobot = 8.5;

//PID Constants
double kp = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay(3000);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  currentMillis = millis();
  if(currentMillis > prevMillis + PERIOD){
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);


    float curTheta = 0;

    float deltaTheta = ((Sr - Sl) / baseRobot);      
    float deltaX = cos(curTheta + (deltaTheta / 2)); 
    float deltaY = sin(curTheta + (deltaTheta / 2)); 
    float deltaS = (Sr + Sl)/2;

    curTheta += deltaTheta;

    Serial.println(curTheta);
    
    float goalTheta = atan2(yGoals[curGoal] - deltaY, xGoals[curGoal] - deltaX); //goal theta


    if(curGoal < NUMBER_OF_GOALS){
      curGoal++;
    }


    double error = goalTheta - curTheta;
    double proportional = kp * error;

    double leftSpeed = MOTOR_BASE_SPEED + proportional;
    double rightSpeed = MOTOR_BASE_SPEED - proportional;
    if (leftSpeed < 0)
      leftSpeed = 0;
    if (rightSpeed < 0)
      rightSpeed = 0;
    if (leftSpeed > 200)
      leftSpeed = 200;
    if (rightSpeed > 200)
      rightSpeed = 200;
    
    motors.setSpeeds(-leftSpeed, -rightSpeed);

  } 
}

void distance(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1;
  float dy = y2 - y1;
  return sqrt(dx*dx + dy*dy);
}


