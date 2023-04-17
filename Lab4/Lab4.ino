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
double kp = 70;

float deltaTheta = 0.0;
float curTheta = 0.0;
float deltaX = 0.0;
float deltaY = 0.0;
float deltaS = 0.0;
float goalTheta = 0.0;
float currentX = 0.0;
float currentY = 0.0;
float dis = 0;
bool flag = true;
bool debug = true;


void setup() {
  Serial.begin(57600);
  delay(3000);
  buzzer.play("c32");
}

void loop() {
  currentMillis = millis();
  if(currentMillis > prevMillis + PERIOD){
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    float Sl = ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    float Sr = ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    deltaS = ((Sr + Sl)/2);
    deltaTheta = ((Sl - Sr) / baseRobot); 
    curTheta += deltaTheta;
    
    deltaX = deltaS * (cos(curTheta + (deltaTheta / 2))); 
    deltaY = deltaS * (sin(curTheta + (deltaTheta / 2))); 
    currentX += -(deltaX);
    currentY += -(deltaY);

    goalTheta = atan2(yGoals[curGoal] - currentY, xGoals[curGoal] - currentX); 
    double error = goalTheta - curTheta;

    dis = distance(currentX, currentY, xGoals[curGoal], yGoals[curGoal]);

    if (debug){
      Serial.print("Distance: ");
      Serial.println(dis);
      
      Serial.print("Delta S: ");
      Serial.print(deltaS);
      Serial.print(" --- ");

      Serial.print("x: ");
      Serial.print(currentX);
      Serial.print(" --- ");

      Serial.print("y: ");
      Serial.print(currentY);
      Serial.print(" --- ");

      Serial.print("curTheta: ");
      Serial.print(curTheta);
      Serial.print(" --- ");

      Serial.print("goalTheta: ");
      Serial.print(goalTheta);
      Serial.print(" --- ");

      Serial.print("deltaTheta: ");
      Serial.print(deltaTheta);
      Serial.print(" --- ");

      Serial.print("error: ");
      Serial.print(error);
      Serial.print(" --- ");
    }

    // PID - Only using proportional
    double proportional = kp * error;

    double leftSpeed = MOTOR_BASE_SPEED + proportional;
    double rightSpeed = MOTOR_BASE_SPEED - proportional;

    // if we are not .5 cm within the goal we keep moving
    if (dis > .5){
      if (leftSpeed < 0)
        leftSpeed = 0;
      if (rightSpeed < 0)
        rightSpeed = 0;
      if (leftSpeed > 200)
        leftSpeed = 200;
      if (rightSpeed > 200)
        rightSpeed = 200;
      motors.setSpeeds(-rightSpeed, -leftSpeed);
    }


    else{
      //motors.setSpeeds(0, 0);
      //delay(3000);

      // check to make sure there is still more goals to go
      if (curGoal <= 1){
        buzzer.play("c32");
        curGoal = curGoal + 1;
      }
      // if we are done 
      else if(flag){
        motors.setSpeeds(0, 0);
        buzzer.play(F("l8 cdefgab>c"));
        flag = false;
        delay(3000);
      }
    }

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;
  } 
}


float distance(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1;
  float dy = y2 - y1;
  return sqrt(dx*dx + dy*dy);
}


