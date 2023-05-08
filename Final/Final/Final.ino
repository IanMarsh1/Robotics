#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

/**
 *Custom class to keep track of servo positions and the distance measured at them.
 */
class ultrasonicReading
{
private:
	double position;
	double weight;
	// Distances are inverted, bigger number means closer to an obstacle.
	double firstDistance;
	double secondDistance;
	double thirdDistance;

public:
	// Constructor
	ultrasonicReading(double position, double weight)
	{
		this->position = position;
		this->weight = weight;
		firstDistance = 0;
		secondDistance = 0;
		thirdDistance = 0;
	}
	// Bigger number means closer to an obstacle
	void setDistance(double distance)
	{
		if (distance < 0)
			distance = 0;
		// 3 distances to take average of them. Oldest distance overwritten by newer distances.
		firstDistance = secondDistance;
		secondDistance = thirdDistance;
		thirdDistance = distance;
	}
	double getPosition()
	{
		return position;
	}
	// Average the 3 most recent measured distances and multiply by the weight.
	double getAdjustedDistance()
	{
		double averageDistance = (firstDistance + secondDistance + thirdDistance) / 3;
		return averageDistance * weight;
	}
};

double kp = 20;
double ki = 0.00005;
double kd = 0.00001;
double kiTotal = 0;
double error = 0;
double priorError = 0;

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
double xGoals[] = {60};
double yGoals[] = {60};
int currentGoal = 0;

// Encoder configurations
const float DISTANCE_BETWEEN_WHEELS = 8.5; // 8.5cm
const unsigned long ENCODER_PERIOD = 30;   //  miliseconds
const int CLICKS_PER_ROTATION = 12;		   // DC motor 1 rotation = 12 state changes marked by encoders
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;
const float CM_PER_INCH = 2.54;

// Encoder variables
unsigned long encoderCm;
unsigned long encoderPm;

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
float theta = pi/2;

// Servo variables
ultrasonicReading SERVO_POSITIONS[] = {
	ultrasonicReading(140, 5),
	ultrasonicReading(110, 6),
	ultrasonicReading(90, 7),
	ultrasonicReading(70, 6),
	ultrasonicReading(40, 5)};

int NUM_HEAD_POSITIONS = 5; 
int currentServoPosition = 0;
boolean headDirectionClockwise = true;
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 250;
boolean flag = false;

// Ultrasonic variables
unsigned long USCurrentMills;
unsigned long USPrevMills;
const unsigned int ULTRASONIC_PERIOD = 10;
double ultraDistance = 0;
double MAX_DISTANCE = 30;
const double MAX_DISTANCE_TEMP = 5;

void setup(){
	// Invert motors
	motors.flipLeftMotor(true);
	motors.flipRightMotor(true);

	// Attach servo pin to servo
	headServo.attach(SERVO_PIN);
  headServo.write(90);
	// Set ultrasonic pins
	pinMode(ECHO_PIN, INPUT);
	pinMode(TRIG_PIN, OUTPUT);

	// Send servo to first position
	headServo.write(SERVO_POSITIONS[currentServoPosition].getPosition());

  Serial.begin(57600);
  delay(3000);
  buzzer.play("C32");
}

void loop(){
	// Run funtions until all goals have been reached
	if (currentGoal < sizeof(xGoals) / sizeof(xGoals[0])) // Correctly count number of elements in array.
	{
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
      if (currentServoPosition >= NUM_HEAD_POSITIONS - 1){ // Correctly count number of elements in array
        headDirectionClockwise = !headDirectionClockwise;
        currentServoPosition--;
      }

      else{
        // Continue clockwise
        currentServoPosition++;
      }
    }
    else{
      if (currentServoPosition <= 0){
        // Counterclockwise and reached end
        headDirectionClockwise = !headDirectionClockwise;
        currentServoPosition++;
      }
      else{
        // Continue counterclockwise
        currentServoPosition--;
      }
    }
    // Move to new position
    headServo.write(SERVO_POSITIONS[currentServoPosition].getPosition());

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
    SERVO_POSITIONS[currentServoPosition].setDistance(MAX_DISTANCE - ultraDistance);

    flag = false;
    USPrevMills = USCurrentMills;
  }
}

void localization(){
	// Updates encoder information once every specified period of time
	encoderCm = millis();
	if (encoderCm < encoderPm + ENCODER_PERIOD)
		return;

	// Inverted and reversed because going "backwards"
	long countLeft = -encoders.getCountsAndResetRight();
	long countRight = -encoders.getCountsAndResetLeft();

	// new measurements since previous
	float Sl = (countLeft / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
	float Sr = (countRight / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

	float deltaS  = (Sl + Sr) / 2;					   // distance the centerpoint went
	float changeTheta = (Sr - Sl) / DISTANCE_BETWEEN_WHEELS; // new rotation, change in theta

	float deltaX = deltaS  * cos(theta + (changeTheta / 2));
	float deltaY = deltaS  * sin(theta + (changeTheta / 2));

	theta = theta + changeTheta;
	currentX = currentX + deltaX;
	currentY = currentY + deltaY;

	encoderPm = encoderCm;
}

void potentialFields(){
		// Distance from goal. Goal is zero, and distances further away from the goal get larger.
	float distance = distanceCalc(currentX, currentY, xGoals[currentGoal], yGoals[currentGoal]);

	// Reached close enough to goal
	if (distance < .5)
	{
		motors.setSpeeds(0, 0);
		buzzer.play("C32");
		delay(300);
		currentGoal++;
		kp = 45;
		return;
	}

	// Calculate rotation error
	float idealRotation = atan2(yGoals[currentGoal] - currentY, xGoals[currentGoal] - currentX);
	float rotationError = idealRotation - theta;
	rotationError = atan2(sin(rotationError), cos(rotationError));
  error = rotationError;
  double kpResult = kp * error;
  kiTotal += error;
  if (kiTotal > 50){
    kiTotal = 50;
  }
  else if (kiTotal < -50){
    kiTotal = -50;
  }
  double kiResult = ki * kiTotal;
  double kdResult = kd * (error - priorError);
  priorError = error;

	rotationError = kpResult + kiResult + kdResult;

	// ---- Potential fields of obsticles ----
	double leftAdjustment = 0;
	double rightAdjustment = 0;
	double centerAdjustedDistance = 0;
	// For each servo angle
	for (int i = 0; i < sizeof(SERVO_POSITIONS) / sizeof(SERVO_POSITIONS[0]); i++)
	{
		// Get recorded distance, adjusted with the particular angle's weight
		double adjustedDistance = SERVO_POSITIONS[i].getAdjustedDistance();
		if (SERVO_POSITIONS[i].getPosition() < 90)
		{
			// Object to right, speed up right
			rightAdjustment += adjustedDistance;
		}
		else if (SERVO_POSITIONS[i].getPosition() > 90)
		{
			// Object to left, speed up left
			leftAdjustment += adjustedDistance;
		}
		else
		{
			// Center 90° angle, decide after
			centerAdjustedDistance = adjustedDistance;
		}
	}
	// Determine which way the measurement at 90° is applied (Turn away from side with more things)
	if (rightAdjustment > leftAdjustment)
		rightAdjustment += centerAdjustedDistance;
	else
		leftAdjustment += centerAdjustedDistance;

	// Sum up all factors to get wheel speeds
	float leftSpeed = MOTOR_BASE_SPEED  - rotationError + leftAdjustment - rightAdjustment;
	float rightSpeed = MOTOR_BASE_SPEED  + rotationError + rightAdjustment - leftAdjustment;

	// Close to target.  Ignore a lot of sensor data, increase goal-seeking PID.
	if (distance < 10)
	{
		kp = 100;
		MAX_DISTANCE = MAX_DISTANCE_TEMP;
		// Slow down near target using quadratic-calculated distance. (The if prevents speeding up)
		if (distance / 5 < 1)
		{
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

