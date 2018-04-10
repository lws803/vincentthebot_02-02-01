/* This source code defines the behaviour of the team's mBot movement along 
 * the maze's path. Considerations were well thought out and in place to 
 * ensure the smooth and effective maneuver around the maze.
 * These considerations include:
 * 	-	Effective straight, parallel movement along the path
 * 	-	Precise turnings at junctions
 * 	-	Well designed algorithms to detect type of challenges and solving the
 * 		challenge with precise answers which determines the next course of 
 *		action
 *  - Printing of sensor values and additional information for easier debugging.
 */

/* Library imports */
#include <MeMCore.h>
#include <MeLightSensor.h>

/* MACROS */
#define NO_ACTION 3 // No action
#define FRONT 2 // Front direction
#define LEFT 1 // Left direction
#define RIGHT 0 // Right direction
#define LIGHT_CHALLENGE 0 // Light challenge
#define SOUND_CHALLENGE 1 // Sound challenge
#define LEFT_SENSOR A0 // Defines pin of left IR sensor
#define RIGHT_SENSOR A1 // Defines pin of right IR sensor
#define SOUND_SENSOR A3 // Defines pin of sound sensor
#define LEFT_MOTOR_SPEED 177 // Default speed for left wheel
#define RIGHT_MOTOR_SPEED 157 // Default speed for right wheel
#define LEFT_MOTOR_TURN_SPEED 177 // Turning speed for left wheel
#define RIGHT_MOTOR_TURN_SPEED 157 // Turning speed for right wheel
#define LEFT_SPEED_INCREMENT 50 // Increment speed for left wheel
#define RIGHT_SPEED_INCREMENT 75 // Increment speed for right wheel
#define TURN_DELAY 370 // Time taken for a turn
#define ADJUSTMENT_DELAY 75 // Delay during left and right adjustment
#define CALIBRATE_DELAY 200 // Delay before calibration begins

/* Global variables */
// IR Sensors
double LEFT_SENSOR_THRESHOLD;
double RIGHT_SENSOR_THRESHOLD;
// Environment sensors(light, sound, line)
// Values below are calibrated to actual scenario.
MeLineFollower lineFinder(PORT_2); 
MeLightSensor lightSensor(PORT_6);
double SOUND_THRESHOLD_MILD = 1.3;
double SOUND_THRESHOLD_LOUD = 2.15;
double LIGHT_THRESHOLD_DARK = 1.3;
double LIGHT_THRESHOLD_LOW = 2.6;
double LIGHT_THRESHOLD_MEDIUM = 3.9;
// Motors
MeDCMotor leftMotor(M1); 
MeDCMotor rightMotor(M2);
uint8_t leftWheelDefSpeed = LEFT_MOTOR_SPEED;
uint8_t rightWheelDefSpeed = RIGHT_MOTOR_SPEED;
uint8_t leftWheelTurnSpeed = LEFT_MOTOR_TURN_SPEED;
uint8_t rightWheelTurnSpeed = RIGHT_MOTOR_TURN_SPEED;
uint8_t leftWheelSpeedIncre = LEFT_SPEED_INCREMENT;
uint8_t rightWheelSpeedIncre = RIGHT_SPEED_INCREMENT;
// LED
MeRGBLed led(7);
// Buzzer
MeBuzzer buzzer;

// Main SETUP
void setup() {
	Serial.begin(9600);
	calibrateSystem();
}

// Main LOOP
void loop() {
	if ((millis() > 3000) && (millis() < 5000)) {
		led.setColor(0, 0, 0, 0);
		led.show();	
	}
	if (readLine()) {
    // Brief delay for mBot to be centralized.
		delay(10);
		stopSystem();
    // 1 second delay for more reliable sensor readings.
		delay(1000);
		challengeTurn();
	}
	parallelMove();
}

/* This function calibrates the IR sensors once system is turned on
 * While there is flickering of blue light, it means that calibration is still ongoing.
 * We take the average of 40 calibrated readings to minimise any possible random error.
 * If the mBot is connected to the laptop, it will print out the summation of calibrated values per interval.
 * Once calibration is done, the average of the calibrated values represents our respective threshold values
 * and a green light is displayed, signalling that the mBot is ready to start the maze.
*/
void calibrateSystem(void) {
	long i, totalLeft = 0	, totalRight = 0;
	int blueColor = 40, greenColor = 40;
	double calibrateLoop = 40, thresholdScale = 0.875;

	delay(CALIBRATE_DELAY);
	for (i = 1; i <= calibrateLoop; i++) {
		if (i % 3 == 0) {
			led.setColor(0, 0, 0, blueColor);
			led.show();
		}
		else {
			led.setColor(0, 0, 0, 0);
			led.show();
		}

		totalLeft += readIR(LEFT_SENSOR);
		totalRight += readIR(RIGHT_SENSOR);
		Serial.print("total left = ");
		Serial.println(totalLeft);
		Serial.print("total right = ");
		Serial.println(totalRight);
		led.setColor(0, 0, 0, 0);
		led.show();
	}
	// Set thresholds with calibrated values
	LEFT_SENSOR_THRESHOLD = (((double) totalLeft / calibrateLoop) * thresholdScale);
	RIGHT_SENSOR_THRESHOLD = (((double) totalRight / calibrateLoop) * thresholdScale);
	Serial.print("Left sensor threshold = ");
	Serial.println(LEFT_SENSOR_THRESHOLD);
	Serial.print("Right sensor threshold = ");
	Serial.println(RIGHT_SENSOR_THRESHOLD);
	Serial.println("Calibration done!");
	led.setColor(0, 0, greenColor, 0);
	led.show();
}

/* Parallel Move
 * This function adjusts the mBot's pathing to ensure that the mBot is centralized.
 * When the mBot deviates to the left, the algorithm kicks in to shift it rightwards to the middle
 * and vice versa. At every instance, the mBot checks for the line to ensure that the mBot is always prompt in
 * reacting with minimal delay.
*/
void parallelMove(void) {
	double leftPosition = readIR(LEFT_SENSOR);
	double rightPosition = readIR(RIGHT_SENSOR);
	
	// Check if black line is encountered. If true, exits function.
	if(readLine()) {
		return;
	}
	// Moves the mBot forward
	startSystem();
	
	// Determine if adjustments are required.
	// Adjust to right
	if (leftPosition < LEFT_SENSOR_THRESHOLD) {
		Serial.println("Adjusting right!");
		// Adjust to right
		while (leftPosition < LEFT_SENSOR_THRESHOLD) {
			// Check if black line is encountered. If true, exits function.
			if(readLine()) {
				return;
			}
			forwardLeftWheel(leftWheelDefSpeed + leftWheelSpeedIncre);
			delay(ADJUSTMENT_DELAY);
			leftPosition = readIR(LEFT_SENSOR);
		}
		forwardLeftWheel(leftWheelDefSpeed - leftWheelSpeedIncre);
		delay(ADJUSTMENT_DELAY);
		forwardLeftWheel(leftWheelDefSpeed);
	}
	// Adjust to left
	if (rightPosition < RIGHT_SENSOR_THRESHOLD) {
		Serial.println("Adjusting left!");
		while (rightPosition < RIGHT_SENSOR_THRESHOLD) {
			// Check if black line is encountered
			if(readLine()) {
				return;
			}
			forwardRightWheel(rightWheelDefSpeed + rightWheelSpeedIncre);
			delay(ADJUSTMENT_DELAY);
			rightPosition = readIR(RIGHT_SENSOR);
		}
		forwardRightWheel(rightWheelDefSpeed - rightWheelSpeedIncre);
		delay(ADJUSTMENT_DELAY);
		forwardRightWheel(rightWheelDefSpeed);
	}
}

/* IR sensor
 * Works by reading the voltage from input pin. Cast across a range from 0 to 1024
 * Pre-cond: An int value for the input pin is specified
 * Post-cond: Returns an int value from 0 to 1024 
 */
int readIR(int pin) {
	int irValue = analogRead(pin);
	return irValue;
}

/* Sound sensor
 * This reads the analog values from the microphone installed on the system
 * Pre-cond: An int value for the input pin is specified
 * Post-cond: Returns an int value from 0 to 1024 
 */
double readSound(int pin) {
	int soundValue = analogRead(pin);
	return soundValue;
}

/* Line Sensor
 * Returns true if a black line can be detected at the current position
 */
int readLine(void) {
	if (lineFinder.readSensors() == S1_IN_S2_IN) {
		return 1;
	}
	return 0;
}

// Rolls left wheel forward
void forwardLeftWheel(uint8_t speed) {
	leftMotor.run(-speed);
}

// Rolls left wheel backward
void backwardLeftWheel(uint8_t speed) {
	leftMotor.run(speed);
}

// Rolls right wheel forward
void forwardRightWheel(uint8_t speed) {
	rightMotor.run(speed);
}

// Rolls right wheel backward
void backwardRightWheel(uint8_t speed) {
	rightMotor.run(-speed);
}

// Stop left wheel
void stopLeftWheel(void) {
	leftMotor.stop();
}

// Stop right wheel
void stopRightWheel(void) {
	rightMotor.stop();
}

// Start the system
void startSystem(void) {
	forwardLeftWheel(leftWheelDefSpeed);
	forwardRightWheel(rightWheelDefSpeed);
}

// Stop the system
void stopSystem(void) {
	leftMotor.stop();
	rightMotor.stop();
}

/* Executes a 90 degree left spot turn
 * Pre-cond: System stopped moving
 */
void leftSpotTurn(void) {
	backwardLeftWheel(leftWheelTurnSpeed);
	forwardRightWheel(rightWheelTurnSpeed);
	delay(TURN_DELAY);
}

/* Executes a 90 degree right spot turn
 * Pre-cond: System stopped moving
 */
void rightSpotTurn(void) {
	backwardRightWheel(rightWheelTurnSpeed);
	forwardLeftWheel(leftWheelTurnSpeed);
	delay(TURN_DELAY);
}

/* Determines the challenge encountered and solves it. 
 * Takes in a value from detectChallenge.
 * If it is neither sound or light challenge, it signifies the end of maze and exit the function afterwards.
 * Otherwise, the functions, whether computeLightChallenge or computeSoundChallenge, will return the turn direction.
 * The mBot will turn accordingly before exiting this function.
 */
void challengeTurn(void) {
	int direction, challenge;

	// Determine which challenge is encountered
	challenge = detectChallenge();
	if (challenge == LIGHT_CHALLENGE) {
		direction = computeLightChallenge();
	}
	else if (challenge == SOUND_CHALLENGE) {
		direction = computeSoundChallenge();
	}
	else {
    Serial.println("End of maze!");
    endCourse();
    return;
	}
	
	// Turn the system based on the direction returned by challenge
	if (direction == LEFT) {
		Serial.println("Turning left!");
		leftSpotTurn();
	}
	else if (direction == RIGHT) {
		Serial.println("Turning right!");
		rightSpotTurn();
	}
	else if (direction == FRONT) {
		Serial.println("Going forward!");
		startSystem();
	}
}

// Determine the type of challenge
// Returns the corresponding value to challengeTurn function.
int detectChallenge(void) {
	int challenge, lightVoltageValue;
	uint16_t lightSensorValue;
	double soundVoltageValue, soundSensorValue;
	double maxAnalogVal = 1023.0, maxVoltage = 5.0;

  // Take light and sound sensor readings.
	lightSensorValue = lightSensor.read();
	lightVoltageValue = (((double) lightSensorValue / maxAnalogVal) * maxVoltage);
	soundSensorValue = readSound(SOUND_SENSOR);
	soundVoltageValue = ((soundSensorValue / maxAnalogVal) * maxVoltage);

	// If low light and no sound, no action required
	if (lightVoltageValue < LIGHT_THRESHOLD_DARK && soundVoltageValue < SOUND_THRESHOLD_MILD) {
			return NO_ACTION;
	}

	/* If surrounding light level is sufficiently high, it is a light challenge.
	 * Otherwise, it is a sound challenge.
	 */
	if (lightVoltageValue >= LIGHT_THRESHOLD_DARK) {
		challenge = LIGHT_CHALLENGE;
	}
	else {
		challenge = SOUND_CHALLENGE;
	}
	return challenge;
}

// Solve light challenge and return the corresponding direction
int computeLightChallenge(void) {
	int direction;
	uint16_t lightSensorValue;
	double lightVoltageValue;
	double maxAnalogVal = 1023.0, maxVoltage = 5.0;

	// Determine the turn direction according to surrounding light level
	lightSensorValue = lightSensor.read();
	lightVoltageValue = ((double)lightSensorValue / maxAnalogVal) * maxVoltage;
	Serial.print("Light voltage = ");
	Serial.println(lightVoltageValue);
  if (lightVoltageValue < LIGHT_THRESHOLD_LOW) {
		direction = RIGHT;
	}
	else if (lightVoltageValue < LIGHT_THRESHOLD_MEDIUM) {
		direction = LEFT;
	}
	else {
		direction = FRONT;
	}

	return direction;
}

// Solve sound challenge and return the corresponding direction
int computeSoundChallenge(void) {
	int direction;
	double soundVoltageValue, soundSensorValue;
	double maxAnalogVal = 1023.0, maxVoltage = 5.0;

	soundSensorValue = readSound(SOUND_SENSOR);
	soundVoltageValue = ((double)soundSensorValue / maxAnalogVal) * maxVoltage;
	Serial.print("Sound voltage = ");
	Serial.println(soundVoltageValue);
  //Turns right at mild. Turns left at loud.
  if (soundVoltageValue < SOUND_THRESHOLD_LOUD) {
		direction = RIGHT;
	}
	else {
		direction = LEFT;
	}
	return direction;
}

// Executes the end of maze sequence
// A buzzer tone signifies that the mBot successfully reached the end.
void endCourse(void) {
	int soundFreq = 523, soundDuration = 12000;
	stopSystem();
	buzzer.tone (soundFreq, soundDuration);
	delay(soundDuration);
}
