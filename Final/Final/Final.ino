#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo;

// Goals
const int NUMBER_OF_GOALS = 1;
float xGoals[] = {430};
float yGoals[] = {-110};
int currentGoal = 0;
bool LocMin = true;

// PID Values
float kp = 35;
float ki = .25;
float kd = 0.025;
float kiTotal = 0;
float error = 0;
float prevError = 0;

// Servo 
float headPositions[] = {145, 110, 90, 70, 35};
//float headPositions[] = {90, 90, 90, 90, 90};
float headWeights[] = {.25, .2, .15, .25, .25}; // lower is more
int NUM_HEAD_POSITIONS = 5;

int currentHeadPosition = 0;
bool headDirectionClockwise = true;
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 275;
bool flag = false;

// Pins
const int TRIG_PIN = 12;	 
const int ECHO_PIN = 18;
const int SERVO_PIN = 20; 

// Hardware Constants
const float baseRobot = 8.5; //b constant
const unsigned long PERIOD = 30; 
const int CLICKS_PER_ROTATION = 12;		
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;

// Timing
unsigned long currentMillis;
unsigned long prevMillis;

// Speed control variables
const float MOTOR_BASE_SPEED = 75.0;
const float MOTOR_MIN_SPEED = 30;
const int MAX_SPEED = 200;

// Location variables
const float pi = 3.14159;
float currentX = 0.0f;
float currentY = 0.0f; 
float curTheta = 0;

// Ultrasonic variables
unsigned long USCurrentMills;
unsigned long USPrevMills;
const unsigned int US_PERIOD = 10;
float USDistance = 0;
float MAX_DISTANCE = 30; // if it is more the 30 cm then we do not check
float firstDistance[] = {0, 0, 0, 0, 0}; // keep track of the most recent pint
float secondDistance[] = {0, 0, 0, 0, 0}; // keep track of the last and get the avg

void setup(){
	headServo.attach(SERVO_PIN);

	pinMode(ECHO_PIN, INPUT);
	pinMode(TRIG_PIN, OUTPUT);

	headServo.write(headPositions[currentHeadPosition]);

  Serial.begin(57600);
  delay(3000);
  buzzer.play("C32");

  // Invert motors - chat
	motors.flipLeftMotor(true);
	motors.flipRightMotor(true);
}

void loop(){
  //Go until we run out of goals
	if (currentGoal < NUMBER_OF_GOALS){
		moveHead();
		usReadCm();
		localization();
		potentialFields();
	}
}

void moveHead(){
	headCm = millis();

  // Make servo move back and forth
	if (headCm > headPm + HEAD_MOVEMENT_PERIOD){
    if (headDirectionClockwise){
      if (currentHeadPosition >= NUM_HEAD_POSITIONS - 1){ 
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition--;
      }
      else{
        currentHeadPosition++;
      }
    }

    else{
      if (currentHeadPosition <= 0){
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition++;
      }
      else{
        currentHeadPosition--;
      }
    }

    headServo.write(headPositions[currentHeadPosition]);

    headPm = headCm;
  }
}

void usReadCm(){
	USCurrentMills = millis();

	if ((USCurrentMills > USPrevMills + US_PERIOD) && !flag){
      
    //clears the TRIG_PIN (set low)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    USDistance = duration * 0.034 / 2; // Time of flight, speed of sound / 2

    // apply limits - if it is to far or too close we do not care
    if(USDistance > MAX_DISTANCE) USDistance = MAX_DISTANCE;
    if(USDistance == 0) USDistance = MAX_DISTANCE;

    // Save distance
    float adjDis = USDistance - MAX_DISTANCE;
    setDistance(-adjDis);

    //Serial.println(USDistance);

    flag = false;
    USPrevMills = USCurrentMills;
  }
}

void localization(){
	currentMillis = millis();

	if (currentMillis > prevMillis + PERIOD){

    // Inverted and (-) because we drive backwards - got this from chat
    long countLeft = -encoders.getCountsAndResetRight();
    long countRight = -encoders.getCountsAndResetLeft();

    // Calc the distance taken from the last reading
    float Sl = (countLeft / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    float Sr = (countRight / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    // calc the distance from the middle of the robot and get the angle
    float deltaS  = (Sl + Sr) / 2;
    float changeTheta = (Sr - Sl) / baseRobot;

    // Get change in x and y
    float deltaX = deltaS * cos(curTheta + (changeTheta / 2));
    float deltaY = deltaS * sin(curTheta + (changeTheta / 2));

    // update totals
    curTheta += changeTheta;
    currentX += deltaX;
    currentY += deltaY;

    prevMillis = currentMillis;
  }
}

void potentialFields(){

  //Calc distance to goal
	float distance = distanceCalc(currentX, currentY, xGoals[currentGoal], yGoals[currentGoal]);

  // Have not hit goal yet
	if (distance > .8){
    float goalTheta = atan2(yGoals[currentGoal] - currentY, xGoals[currentGoal] - currentX);
    float error = goalTheta - curTheta;
    error = atan2(sin(error), cos(error)); // got from class prez

    // PID calc
    float kpResult = kp * error;
    kiTotal += error;
    
    // I wind-up
    if (kiTotal > 50){
      kiTotal = 50;    
    }
    else if (kiTotal < -50){
      kiTotal = -50;
    } 

    float kiResult = ki * kiTotal;
    float kdResult = kd * (error - prevError);
    prevError = error;

    error = kpResult + kiResult + kdResult;

    // Initialize vars for PF
    float forceSum = 0;
    float leftPush = 0;
    float rightPush = 0;
    float centerDistance = 0;

    /* Because we keep track of the last two points per every angle we 
     * get the some of the distances and add left and right push accordingly
     * base on the anlge
     */
    for (int i = 0; i < NUM_HEAD_POSITIONS; i++){
      float dis = weightedDistance(i);
      forceSum += unWeightedDistance(i);

      //Objects to the left of robot
      if (headPositions[i] < 90){
        rightPush += dis;
      }
      //Objects to the right of robot
      else if (headPositions[i] > 90){
        leftPush += dis;
      }
      else{
        centerDistance = dis;
      }
    }
    // If there is an object in front of us then we need to decide to go
    // left or right. If there is more push one way then we add the front
    // force to that - chat was used to help us
    if (rightPush >= leftPush){
      rightPush += centerDistance;
    } 
    else{
      leftPush += centerDistance;
    } 

    // Sum of push values
    float push = leftPush - rightPush;
    float leftSpeed = MOTOR_BASE_SPEED  - error + push;
    float rightSpeed = MOTOR_BASE_SPEED  + error + -(push);

    // If we're really close
    // turn up pid just in case and slow the bot down
    if (distance < 5){
      kp = 75;
      leftSpeed /= 1.5;
      rightSpeed /= 1.5;
    }

    // Enforce max and min speed limits
    if (leftSpeed < MOTOR_MIN_SPEED)
      leftSpeed = MOTOR_MIN_SPEED;
    if (rightSpeed < MOTOR_MIN_SPEED)
      rightSpeed = MOTOR_MIN_SPEED;
    if (leftSpeed > MAX_SPEED)
      leftSpeed = MAX_SPEED;
    if (rightSpeed > MAX_SPEED)
      rightSpeed = MAX_SPEED;

    // This is my attempt at sloving the local min problem
    // if there is a lot of obj close then it will turn
    // in a random angle and randmom speed to hopefully get
    // it out of the spot it is stuck in - used chat
    if (forceSum >= 63 && LocMin) {
      buzzer.play("C32");
      int randomAngle = random(-30, 30); // generate a random angle between -30 and 30 degrees
      float randomVelocity = random(50, 100); // generate a random velocity between 50 and 100

      leftSpeed = randomVelocity + randomAngle;
      rightSpeed = randomVelocity - randomAngle;
    }

    motors.setSpeeds(rightSpeed, leftSpeed);
  }

  else {
    motors.setSpeeds(0, 0);
    buzzer.play(F("l8 cdefgab>c"));
    delay(3000);
		currentGoal++;
    kp = 50; // just in case there are more points after this one
  }
  
}

// Distance formula
float distanceCalc(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1;
  float dy = y2 - y1;
  return sqrt(dx*dx + dy*dy);
}

// Helper function to set values in the array for distance
void setDistance(float distance){
		firstDistance[currentHeadPosition] = secondDistance[currentHeadPosition];
		secondDistance[currentHeadPosition] = distance;
}

// Apply distance weights to the US distance - got some help from chat
float weightedDistance(int index){
	float avgDistance = (firstDistance[index] + secondDistance[index]) / 2;
  Serial.print("dis: ");
  Serial.println(avgDistance);
  Serial.print("wei: ");
  Serial.println(avgDistance * headWeights[index]);
	return avgDistance / headWeights[index];
}

// Apply distance weights to the US distance - got some help from chat
float unWeightedDistance(int index){
	float avgDistance = (firstDistance[index] + secondDistance[index]) / 2;
	return avgDistance;
}