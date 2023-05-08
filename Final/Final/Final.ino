#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

float servoPositions[] = {140, 110, 90, 70, 40};
float servoWeights[] = {5, 6, 7, 6, 5};
float firstDistance[] = {0, 0, 0, 0, 0};
float secondDistance[] = {0, 0, 0, 0, 0};
int NUM_HEAD_POSITIONS = 5;

int currentHeadPosition = 0;
boolean headDirectionClockwise = true;
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 240;
boolean flag = false;

float kp = 50;
float ki = 0.00005;
float kd = 0.00001;
float kiTotal = 0;
float error = 0;
float priorError = 0;

// Hardware
Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo;

// Pins
const int TRIG_PIN = 12;	 
const int ECHO_PIN = 18;
const int SERVO_PIN = 20; 

// goals
const int NUMBER_OF_GOALS = 1;
float xGoals[] = {90};
float yGoals[] = {90};
int currentGoal = 0;

// Encoder configurations
const float DISTANCE_BETWEEN_WHEELS = 8.5; // 8.5cm
const unsigned long PERIOD = 30;   //  miliseconds
const int CLICKS_PER_ROTATION = 12;		   // DC motor 1 rotation = 12 state changes marked by encoders
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;

// Encoder variables
unsigned long currentMillis;
unsigned long prevMillis;

// Speed control variables
const float MOTOR_BASE_SPEED = 75.0;
const float MOTOR_MIN_SPEED = 30;
const int MAX_SPEED = 200;

float Sr = 0.0;
float Sl = 0.0;

// Location variables
const float pi = 3.14159;
float currentX = 0.0F; // in CM
float currentY = 0.0F; // in CM
float curTheta = pi/2;

// Ultrasonic variables
unsigned long USCurrentMills;
unsigned long USPrevMills;
const unsigned int ULTRASONIC_PERIOD = 10;
float ultraDistance = 0;
float MAX_DISTANCE = 30;
const float MAX_DISTANCE_TEMP = 2;

void setup(){
	// Invert motors
	motors.flipLeftMotor(true);
	motors.flipRightMotor(true);

	// Attach servo pin to servo
	headServo.attach(SERVO_PIN);
  //headServo.write(90);
	// Set ultrasonic pins
	pinMode(ECHO_PIN, INPUT);
	pinMode(TRIG_PIN, OUTPUT);

	// Send servo to first position
	headServo.write(servoPositions[currentHeadPosition]);

  Serial.begin(57600);
  delay(3000);
  buzzer.play("C32");
}

void loop(){
	// Run funtions until all goals have been reached
	if (currentGoal < NUMBER_OF_GOALS){ // Correctly count number of elements in array.
		moveHead();
		ultraReadCm();
		localization();
		potentialFields();
	}
}

void moveHead(){
	headCm = millis();
	// Move servo every set period of time
	if (headCm > headPm + HEAD_MOVEMENT_PERIOD){
    // Decide next position and switch direction if reached either end
    if (headDirectionClockwise){
      if (currentHeadPosition >= NUM_HEAD_POSITIONS - 1){ // Correctly count number of elements in array
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition--;
      }

      else{
        // Continue clockwise
        currentHeadPosition++;
      }
    }
    else{
      if (currentHeadPosition <= 0){
        // Counterclockwise and reached end
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition++;
      }
      else{
        // Continue counterclockwise
        currentHeadPosition--;
      }
    }
    // Move to new position
    headServo.write(servoPositions[currentHeadPosition]);

    headPm = headCm;
  }
}

void ultraReadCm(){
	USCurrentMills = millis();
	// Check sensors only after being moved to a new position, and give long enough for servo to reach position.
	// Then make sure enough time has passed between ultrasonic reads
	if ((USCurrentMills > USPrevMills + ULTRASONIC_PERIOD) && !flag){
    // Clear trig
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Set trig high for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read echo for flight time in microseconds
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    ultraDistance = duration * 0.034 / 2; // Time of flight, speed of sound divided by 2.

    //apply limits
    if(ultraDistance > MAX_DISTANCE) ultraDistance = MAX_DISTANCE;
    if(ultraDistance == 0) ultraDistance = MAX_DISTANCE;

    // Save distance to object (associate distance with position measured at)
    setDistance(MAX_DISTANCE - ultraDistance);

    flag = false;
    USPrevMills = USCurrentMills;
  }
}

void localization(){
	// Updates encoder information once every specified period of time
	currentMillis = millis();
	if (currentMillis > prevMillis + PERIOD){

    // Inverted and reversed because going "backwards"
    long countLeft = -encoders.getCountsAndResetRight();
    long countRight = -encoders.getCountsAndResetLeft();

    // new measurements since previous
    float Sl = (countLeft / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    float Sr = (countRight / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    float deltaS  = (Sl + Sr) / 2;
    float changeTheta = (Sr - Sl) / DISTANCE_BETWEEN_WHEELS;

    float deltaX = deltaS  * cos(curTheta + (changeTheta / 2));
    float deltaY = deltaS  * sin(curTheta + (changeTheta / 2));

    curTheta += changeTheta;
    currentX += deltaX;
    currentY += deltaY;

    prevMillis = currentMillis;
  }
}

void potentialFields(){
		// Distance from goal. Goal is zero, and distances further away from the goal get larger.
	float distance = distanceCalc(currentX, currentY, xGoals[currentGoal], yGoals[currentGoal]);

	// Reached close enough to goal
	if (distance < .5){
    motors.setSpeeds(0, 0);
    buzzer.play(F("l8 cdefgab>c"));
    delay(3000);
		currentGoal++;
		kp = 45;
		return;
	}

	// Calculate rotation error
	float goalTheta = atan2(yGoals[currentGoal] - currentY, xGoals[currentGoal] - currentX);
	float error = goalTheta - curTheta;
	error = atan2(sin(error), cos(error));

  float kpResult = kp * error;
  kiTotal += error;
  if (kiTotal > 50) kiTotal = 50;
  else if (kiTotal < -50) kiTotal = -50;
  float kiResult = ki * kiTotal;
  float kdResult = kd * (error - priorError);
  priorError = error;

	error = kpResult + kiResult + kdResult;

	float leftPush = 0;
	float rightPush = 0;
	float centerAdjustedDistance = 0;
	// For each servo angle
	for (int i = 0; i < NUM_HEAD_POSITIONS; i++){
		float adjustedDistance = getAdjustedDistance(i);

		if (servoPositions[i] < 90){
			rightPush += adjustedDistance;
		}
		else if (servoPositions[i] > 90){
			leftPush += adjustedDistance;
		}
		else{
			centerAdjustedDistance = adjustedDistance;
		}
	}
	if (rightPush > leftPush) rightPush += centerAdjustedDistance;
	else leftPush += centerAdjustedDistance;

  float push = leftPush - rightPush;
	float leftSpeed = MOTOR_BASE_SPEED  - error + push;
	float rightSpeed = MOTOR_BASE_SPEED  + error + -(push);

	if (distance < 10){
		kp = 100;
		MAX_DISTANCE = MAX_DISTANCE_TEMP;
		if (distance / 5 < 1){
			leftSpeed *= distance / 5;
			rightSpeed *= distance / 5;
		}
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

	motors.setSpeeds(rightSpeed, leftSpeed);
}

// use chat a bit for some of the code but I did not change this at all
float distanceCalc(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1;
  float dy = y2 - y1;
  return sqrt(dx*dx + dy*dy);
}

void setDistance(float distance){
		firstDistance[currentHeadPosition] = secondDistance[currentHeadPosition];
		secondDistance[currentHeadPosition] = distance;
}

float getAdjustedDistance(int index){
	float averageDistance = (firstDistance[index] + secondDistance[index]) / 2;
	return averageDistance * servoWeights[index];
}