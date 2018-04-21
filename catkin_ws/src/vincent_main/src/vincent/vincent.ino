#include "serialize.h"
#include <math.h>
#include <Wire.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "packet.h"
#include "constants.h"
#include "A4990MotorShield.h"

// Motor wires
// YELLOW to RED
// ORANGE to BLACK
// GREEN to BLACK
// BLUE to RED

typedef enum
{
	STOP=0,
	FORWARD=1,
	BACKWARD=2,
	LEFT=3,
	RIGHT=4,
	FORWARD_IR=5,
	BACKWARD_IR=6
} TDirection;


volatile TDirection dir = STOP;
/*
 * Vincent's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.
#define COUNTS_PER_REV 140
#define COUNTS_PER_REV_LEFT 195
#define COUNTS_PER_REV_RIGHT 197

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC
#define WHEEL_CIRC 20.4

// LEFT and RIGHT wheel PWM DIFFERENCE
#define WHEEL_DIFF_FOR 0 
#define WHEEL_DIFF_BAC 0

// Motor control pins. You need to adjust these till
// Vincent moves in the correct direction
#define LF 6   // Left forward pin -> PD6
#define LR 5   // Left reverse pin -> PD5
#define RF 11  // Right forward pin -> PB3
#define RR 10  // Right reverse pin -> PB2

// Determine if left/right adjustments are needed
#define NEED_ADJUST_LEFT  50
#define NEED_ADJUST_RIGHT 50

// Vincent's length and breadth in cm
#define VINCENT_LENGTH 17.5
#define VINCENT_BREADTH 11

// Magnetometer
#define DEG_PER_RAD (180.0/3.14159265358979)
#define MAG_address 0x0E

#define MOTOR_CONST_LEFT 0 
#define MOTOR_CONST_RIGHT 3

// Vincent's diagonal. We compute and store this once
// since it is expensive to compute and really doesn't change
float vincentDiagonal = 0.0;

// Vincent's turning circumference, calculated once
float vincentCirc = 0.0;

/*
 *    Vincent's State Variables
 */

// declare motor 1 and motor 2 object from motor library
A4990MotorShield motors;
 
 
// Store the ticks from Vincent's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Vincent's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have moved a
// commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

// Variables to keep track of current speed
float currentLeftSpeed;
float currentRightSpeed;
float speedConstant;

// Variable to store bearings
float heading;
float curBearing;
float destBearing;

// Variables to track autonomous states
volatile bool AUTONOMOUS_FLAG = false  ;

// Store current Vincent mode (default as remote)
bool isAuto = false;

/* 
 *
 *  ALL function prototypes
 *
 */
// Handle ongoing communications
TResult readPacket(TPacket *packet);
void sendStatus();
void sendMessage(const char *message);
void sendBadPacket();
void sendBadChecksum();
void sendBadCommand();
void sendBadResponse();
void sendOK();
void sendMoveOK();
void sendOKAuto();
void sendStopOK();
void sendReady();

void sendResponse(TPacket *packet);
void handleCommand(TPacket *command);
void waitForHello();
void handlePacket(TPacket *packet);

// Set up interrupts
void enablePullups();
void leftISR();
void rightISR();
void setupEINT();

// Set up serial communications
void setupSerial();
void startSerial();
int readSerial(char *buffer);
void writeSerial(const char *buffer, int len);

// Start the motors
void setupMotors();
void startMotors();


int pwmVal(float speed);
void forward(float dist, float speed);
void reverse(float dist, float speed);
unsigned long computeDeltaTicks(float ang);
void left(float ang, float speed);
void right(float ang, float speed);
void adjustLeft(float increment);
void adjustRight(float increment);
void stop();

// Handle the statistics
void clearCounters();
void clearOneCounter(int which);
void initializeState();

// MAG3110
void beginMAG();
void MAG(int*, int*, int*);
void getHeading();
float getBearing();
void leftMAG();
void rightMAG();
bool turn = false;;

// IR Sensor
bool hasLeftObstacle();
bool hasRightObstacle();
void setupIR();

// Debugging
void lightRed();

/*
 *
 * Setup Arduino
 *
 */
void setup() {

	// Setup PD4 as output pin for red led lighting
	cli();
	setupEINT();
	setupSerial();
	startSerial();
	setupMotors();
	startMotors();
	enablePullups();
	initializeState();
	sei();
	setupIR();

	// Compute Vincent's diagonal and circumference
	vincentDiagonal = sqrt((VINCENT_LENGTH * VINCENT_LENGTH) 
			+ (VINCENT_BREADTH * VINCENT_BREADTH));

	vincentCirc = PI * vincentDiagonal;


	//Initialize I2C communication
	Wire.begin();

	//Start I2C transmission from the MAG3110

	Wire.beginTransmission(MAG_address);

	//Select control register-1
	Wire.write((byte)0x10);
	//Set active mode enabled
	Wire.write((byte)0x01);
	Wire.endTransmission();
	

}

void normalizeSpeed() {
	motors.setSpeeds(currentLeftSpeed, currentRightSpeed);
}

/*
 *
 * Continuous loop
 *
 */
void loop() {

	
	// Check when Vincent can stop moving forward/backward after
	// it is given a fixed distance to move forward/backward
	if (deltaDist > 0) {
		if (dir == FORWARD) {
			// Check when to stop after given distance
			if (forwardDist >= newDist) {
				deltaDist = 0;
				newDist = 0;
				stop();
			}
		}
		else if (dir == FORWARD_IR) {
			
			// check right and left obstacles
			if (hasLeftObstacle() && !hasRightObstacle()) {
				adjustRight(NEED_ADJUST_RIGHT);
			}

			else if (hasRightObstacle() && !hasLeftObstacle()) {
				adjustLeft(NEED_ADJUST_LEFT);
			}
			
			else if (!hasLeftObstacle() && !hasRightObstacle()) {
				normalizeSpeed();				
			}
			
			// Check when to stop after given distance
			if (forwardDist >= newDist) {
				deltaDist = 0;
				newDist = 0;
				stop();
			}	
		}
		else if (dir == BACKWARD) {
			if (reverseDist > newDist) {
				deltaDist = 0;
				newDist = 0;
				stop();
			}
		}
		else if (dir == STOP) {
			deltaDist = 0;
			newDist = 0;
			stop();
		}
	}
	
	 // Turning with magnetometer measurement
	 if (turn) {
		if (dir == LEFT || dir == RIGHT) {
			int cur;
			int upBound = destBearing + 3;
			int lowBound = destBearing - 3;
			if (upBound >= 360) upBound -= 360.0;
			if (lowBound <= 0) lowBound += 360.0;
			if (lowBound > upBound) {
				while(1) {
					cur = getBearing();
					if (cur <= upBound || cur >= lowBound) break;
				}
			} else {
				while(1) {
					cur = getBearing();
					if (cur <= upBound && cur >= lowBound) break;
				}
			}			
		} 
		
		curBearing = destBearing = 0;
		turn = false;
		stop(); 
	}

	// Retrieve packets from RasPi and handle them
	TPacket recvPacket; // This holds commands from the Pi
	TResult result = readPacket(&recvPacket);

	if(result == PACKET_OK) {
		handlePacket(&recvPacket);
	} else {
		if(result == PACKET_BAD) 
		{
			sendBadPacket();
		}
		else {
			if(result == PACKET_CHECKSUM_BAD) {
				sendBadChecksum();
			}
		}
	}
}

/*
 * 
 * Vincent Communication Routines.
 * 
 */

TResult readPacket(TPacket *packet)
{
	// Reads in data from the serial port and
	// deserializes it.Returns deserialized
	// data in "packet".
	// We would need to declare the deserialize function

	char buffer[PACKET_SIZE];
	int len;

	len = readSerial(buffer);

	if(len == 0)
		return PACKET_INCOMPLETE;
	else
		return deserialize(buffer, len, packet);

}

void sendStatus()
{
	// Implement code to send back a packet containing key
	// information like leftTicks, rightTicks, leftRevs, rightRevs
	// forwardDist and reverseDist
	// Use the params array to store this information, and set the
	// packetType and command files accordingly, then use sendResponse
	// to send out the packet. See sendMessage on how to use sendResponse.
	//
	TPacket statusPacket;
	statusPacket.packetType = PACKET_TYPE_RESPONSE;
	statusPacket.command = RESP_STATUS;

	statusPacket.params[0] = leftForwardTicks;
	statusPacket.params[1] = rightForwardTicks;
	statusPacket.params[2] = leftReverseTicks;
	statusPacket.params[3] = rightReverseTicks;
	statusPacket.params[4] = leftForwardTicksTurns;
	statusPacket.params[5] = rightForwardTicksTurns;
	statusPacket.params[6] = leftReverseTicksTurns;
	statusPacket.params[7] = rightReverseTicksTurns;
	statusPacket.params[8] = forwardDist;
	statusPacket.params[9] = reverseDist;

	sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
	// Sends text messages back to the Pi. Useful
	// for debugging.

	TPacket messagePacket;
	messagePacket.packetType=PACKET_TYPE_MESSAGE;
	strncpy(messagePacket.data, message, MAX_STR_LEN);
	sendResponse(&messagePacket);
}

void sendBadPacket()
{
	// Tell the Pi that it sent us a packet with a bad
	// magic number.

	TPacket badPacket;
	badPacket.packetType = PACKET_TYPE_ERROR;
	badPacket.command = RESP_BAD_PACKET;
	sendResponse(&badPacket);

}

void sendBadChecksum()
{
	// Tell the Pi that it sent us a packet with a bad
	// checksum.

	TPacket badChecksum;
	badChecksum.packetType = PACKET_TYPE_ERROR;
	badChecksum.command = RESP_BAD_CHECKSUM;
	sendResponse(&badChecksum);  
}

void sendBadCommand()
{
	// Tell the Pi that we don't understand its
	// command sent to us.

	TPacket badCommand;
	badCommand.packetType=PACKET_TYPE_ERROR;
	badCommand.command=RESP_BAD_COMMAND;
	sendResponse(&badCommand);

}

void sendBadResponse()
{
	TPacket badResponse;
	badResponse.packetType = PACKET_TYPE_ERROR;
	badResponse.command = RESP_BAD_RESPONSE;
	sendResponse(&badResponse);
}

void sendOK()
{
	TPacket okPacket;
	okPacket.packetType = PACKET_TYPE_RESPONSE;
	okPacket.command = RESP_OK;
	sendResponse(&okPacket);  
}

void sendOKAuto() {
	TPacket autoOKPacket;
	autoOKPacket.packetType = PACKET_TYPE_RESPONSE;
	autoOKPacket.command = RESP_OK_AUTO;
	sendResponse(&autoOKPacket);
}

void sendStopOK() {
	TPacket stopPacket;
	stopPacket.packetType = PACKET_TYPE_RESPONSE;
	stopPacket.command = RESP_STOP;
	sendResponse(&stopPacket);
}

void sendMoveOK() {
	TPacket movePacket;
	movePacket.packetType = PACKET_TYPE_RESPONSE;
	movePacket.command = RESP_MOVE;
	sendResponse(&movePacket);
}

void sendReady() {
	TPacket readyPacket;
	readyPacket.packetType = PACKET_TYPE_RESPONSE;
	readyPacket.command = RESP_READY;
	sendResponse(&readyPacket);
}

void sendHeading() {
    TPacket headingPacket;
    headingPacket.packetType = PACKET_TYPE_RESPONSE;
    headingPacket.command = RESP_HEADING;
	headingPacket.params[0] = heading;
    sendResponse(&headingPacket);
}

void sendResponse(TPacket *packet) {
	// Takes a packet, serializes it then sends it out
	// over the serial port.
	char buffer[PACKET_SIZE];
	int len;

	len = serialize(buffer, packet, sizeof(TPacket));
	writeSerial(buffer, len);
}

void handleCommand(TPacket *command)
{
	switch(command->command)
	{
		// For forward/reverse commands, param[0] = distance, param[1] = speed.
		// For turn left/right commands, param[0] = angle, param[1] = speed;
		// For adjust left/right commands, param[0] = increment;
		case COMMAND_FORWARD:
			sendOK();
			forward((float) command->params[0], (float) command->params[1]);
			break;
		case COMMAND_FORWARD_IR:
			sendOK();
			forwardIR(command->params[0], command->params[1]);
			break;
		case COMMAND_REVERSE:
			sendOK();
			reverse((float) command->params[0], (float) command->params[1]);
			break;
		case COMMAND_REVERSE_IR:
			sendOK();
			reverseIR(command->params[0], command->params[1]);
			break;	
		case COMMAND_TURN_LEFT:
			sendOK();
			leftMAG(((float) command->params[0]), ((float) command->params[1]));
			break;
		case COMMAND_TURN_RIGHT:
			sendOK();
			rightMAG((float) command->params[0], (float) command->params[1]);
			break;
		case COMMAND_STOP:
			sendOK();
			stop();
			break;
		case COMMAND_GET_STATS:
			sendStatus();
			sendReady();
			break;
		case COMMAND_CLEAR_STATS:
			sendOK();
			clearOneCounter(command->params[0]);
			sendReady();
			break;
		case COMMAND_AUTO_MODE:
			isAuto = true;
			sendOKAuto();
			sendReady();
			break;
		case COMMAND_REMOTE_MODE:
			isAuto = false;
			sendOK();
			break;
		case COMMAND_GET_HEADING:
			sendOK();
			getHeading();
			sendHeading();
			sendReady();
			break;
		default:
			sendBadCommand();
	}
}

void waitForHello()
{
	int exit=0;

	while(!exit)
	{
		TPacket hello;
		TResult result;

		do
		{
			result = readPacket(&hello);
		} while (result == PACKET_INCOMPLETE);

		if(result == PACKET_OK)
		{
			if(hello.packetType == PACKET_TYPE_HELLO)
			{


				sendOK();
				exit=1;
			}
			else
				sendBadResponse();
		}
		else
			if(result == PACKET_BAD)
			{
				sendBadPacket();
			}
			else
				if(result == PACKET_CHECKSUM_BAD)
					sendBadChecksum();
	} // !exit
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
			handleCommand(packet);
			break;

		case PACKET_TYPE_RESPONSE:
			break;

		case PACKET_TYPE_ERROR:
			break;

		case PACKET_TYPE_MESSAGE:
			break;

		case PACKET_TYPE_HELLO:
			break;

		case PACKET_TYPE_AUTO:
			AUTONOMOUS_FLAG = true;
			break;
	}
}

/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
	// Use bare-metal to enable the pull-up resistors on pins
	// 2 and 3. These are pins PD2 and PD3 respectively.
	// We set bits 2 and 3 in DDRD to 0 to make them inputs. 
	DDRD &= 0b11110011;
	PIND |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{ 
	if (dir == FORWARD || dir == FORWARD_IR) {
		leftForwardTicks++;
		forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV_LEFT * WHEEL_CIRC);
	}
	else if (dir == BACKWARD || dir == BACKWARD_IR) {
		leftReverseTicks++;
		reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV_LEFT * WHEEL_CIRC);
	}
	else if (dir == RIGHT) {
		leftForwardTicksTurns++;
	}
}

void rightISR()
{
	if (dir == FORWARD || dir == FORWARD_IR) {
		rightForwardTicks++;
		forwardDist = (unsigned long) ((float) rightForwardTicks / COUNTS_PER_REV_RIGHT * WHEEL_CIRC);
	} else if (dir == BACKWARD || dir == BACKWARD_IR) {
		rightReverseTicks++;
		reverseDist = (unsigned long) ((float) rightReverseTicks / COUNTS_PER_REV_RIGHT * WHEEL_CIRC);
	} else if (dir == LEFT) {
		rightForwardTicksTurns++;
	}  
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
	EICRA = 0x0A;
	EIMSK |= 0b00000011;

}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
// TODO: Implement adjustments during interrupts
ISR (INT0_vect) {
	leftISR();
}

ISR (INT1_vect) {
	rightISR();
}

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.

void setBaud(unsigned long baudRate)
{
    unsigned int b;
    b = (unsigned int) round(F_CPU / (16.0 * baudRate))- 1; // round up/ down because its an integer    
    UBRR0H = (unsigned char) (b >> 8);
    UBRR0L = (unsigned char) b;
}

void setupSerial()
{
    cli();
    UCSR0A = 0;
    UCSR0C = 0b00000110;  
}

void startSerial()
{
    setBaud (9600);
    UCSR0B = 0b00011000; // Disable receive and transmit complete interrupts
    sei();
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.
char dataRecv; 

int readSerial(char *buffer)
{

	int count=0;
    while ((UCSR0A & 0b10000000) == 0); 
        buffer[count++] = UDR0;


	return count;
}


void writeSerial(const char *buffer, int len)
{
    int i = 0;
    while(i++ < len) {
        while ((UCSR0A & 0b00100000) == 0); 
        UDR0 = buffer[i];   
    }
}

/*
 * Vincent's motor drivers.
 * 
 */

// Start the PWM for Vincent's motors.
// We will implement this later. For now it is
// blank.
void startMotors(){

	// verify the ports
	TCCR0B = 0b0000011;	// 64 prescalar value
	TCCR2B = 0b0000100;	// 64 prescalar value
}

// Convert percentages to PWM values
int pwmVal(float speed) 
{
	if(speed < 0.0)
		speed = 0;

	if(speed > 100.0)
		speed = 100.0;

	return (int) ((speed / 100.0) * 255.0);
}

// Move Vincent forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Vincent will
// continue moving forward indefinitely.
void forward(float dist, float speed) 
{
	// Set the direction of travel
	dir = FORWARD;

	// Set current speed
	currentLeftSpeed = speed;
	currentRightSpeed = speed;

	// it is now moving
	sendMoveOK();

	// Compute the new total distance given the input
	if (dist > 0) deltaDist = dist;
	else deltaDist = 9999999;
	
	newDist = forwardDist + deltaDist;

	// LF = Left forward pin, LR = Left reverse pin
	// RF = Right forward pin, RR = Right reverse pin
	// This will be replaced later with bare-metal code.
	
	motors.setSpeeds(speed - MOTOR_CONST_LEFT, speed - MOTOR_CONST_RIGHT);
}

void forwardIR(int dist, int speed) 
{
	// Set the direction of travel
	dir = FORWARD_IR;

	// Set current speed
	currentLeftSpeed = speed - MOTOR_CONST_LEFT;
	currentRightSpeed = speed - MOTOR_CONST_RIGHT;

	// it is now moving
	sendMoveOK();
	
	// Compute the new total distance given the input
	if (dist > 0) deltaDist = dist;
	else deltaDist = 9999999;
	
	newDist = forwardDist + deltaDist;
	
	motors.setSpeeds(currentLeftSpeed, currentRightSpeed);
	
}

// Reverse Vincent "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Vincent will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
	// Set the direction of travel
	dir = BACKWARD;

	// Set current speed
	currentRightSpeed = speed;
	currentLeftSpeed = speed;
	
	// it is now moving
	sendMoveOK();
	
	
	// Compute the new total distance given the input
	if (dist > 0) deltaDist = dist;
	else deltaDist = 9999999;
	newDist = reverseDist + deltaDist;

	motors.setSpeeds(-speed + MOTOR_CONST_LEFT, -speed + MOTOR_CONST_RIGHT);
	
	
	// LF = Left forward pin, LR = Left reverse pin
	// RF = Right forward pin, RR = Right reverse pin
	// This will be replaced later with bare-metal code.
	
}

void reverseIR(int dist, int speed)
{
	// Set the direction of travel
	dir = BACKWARD_IR;

	// Set current speed
	currentLeftSpeed = speed - MOTOR_CONST_LEFT;
	currentRightSpeed = speed - MOTOR_CONST_RIGHT;
	
	// it is now moving
	sendMoveOK();

	// Compute the new total distance given the input
	if (dist > 0) deltaDist = dist;
	else deltaDist = 9999999;
	newDist = reverseDist + deltaDist;

	motors.setSpeeds(-currentLeftSpeed, -currentRightSpeed);
}

// Function to estimate number of wheel ticks needed
// to turn an angle
unsigned long computeDeltaTicks(float ang) {
	if (dir == LEFT) {
		unsigned long ticks = (unsigned long) ((ang * 
				vincentCirc * COUNTS_PER_REV_LEFT) / (360.0 * WHEEL_CIRC));
	}
	else if (dir == RIGHT) {
		unsigned long ticks = (unsigned long) ((ang * 
				vincentCirc * COUNTS_PER_REV_RIGHT) / (360.0 * WHEEL_CIRC));


	return ticks;
}

// Turn Vincent left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn left indefinitely.
void left(float ang, float speed)
{
	// Set the direction of travel
	dir = LEFT;

	// it is now moving
	sendMoveOK();

	// Compute the new total ticks needed to left turn
	if(ang == 0) deltaTicks=99999999; 
	else deltaTicks=computeDeltaTicks(ang); 
	targetTicks = rightForwardTicksTurns + deltaTicks;

	// left motor does nothing, right motor moves
	motors.setSpeeds(-speed, speed);  // spot turn
}

// Turn Vincent right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn right indefinitely.
void right(float ang, float speed)
{
	// Set the direction of travel
	dir = RIGHT;

	// it is now moving
	sendMoveOK();

	// Compute the new total ticks needed to right turn
	if(ang == 0) deltaTicks=99999999; 
	else deltaTicks=computeDeltaTicks(ang); 
	targetTicks = leftForwardTicksTurns + deltaTicks;

	motors.setSpeeds(speed, -speed);
}

void leftMAG (float ang, float speed) {
  // Set the direction of travel
  dir = LEFT;
  turn = true;

  sendMoveOK();
  curBearing = getBearing();
  destBearing = curBearing - ang;
  if (destBearing < 0) destBearing += 360;

  motors.setSpeeds(0, speed);
}

void rightMAG (float ang, float speed) {
  // Set the direction of travel
  dir = RIGHT;
  turn = true;

  sendMoveOK();
  curBearing = getBearing();
  destBearing = curBearing + ang;
  if (destBearing > 360) destBearing -= 360;

  motors.setSpeeds(speed, -speed);
}


// Adjust Vincent left given degree of adjust
// TODO: Figure out the way to compute the degree of adjustment
void adjustLeft(float increment) 
{	
	// Adjust the motors speed
	motors.setSpeeds(currentLeftSpeed, currentRightSpeed + increment);
	
	//currentRightSpeed += increment;
}

// Adjust Vincent right given degree of adjust
// TODO: Figure out the way to compute the degree of adjustment
void adjustRight(float increment) 
{
	// Adjust the motors speed
	motors.setSpeeds(currentLeftSpeed + increment, currentRightSpeed);
	
	//currentLeftSpeed += increment;
}

// Stop Vincent. To replace with bare-metal code later.
void stop()
{
	dir = STOP;
	turn = false;
	//setSpeeds(0, 0);
	motors.setSpeeds(0,0);

	sendStopOK();
	sendReady();
}

/* End of vincent motor code
 *
*/

/*
 * Vincent's setup and run codes
 * 
 */
int x, y, z;
static int xmax = -1024, xmin = 1024, ymax = -1024, ymin = 1024, zmax = -1024, zmin = 1024;

void MAG(int* xR, int* yR, int* zR) {
  Wire.beginTransmission(MAG_address);
  Wire.write((byte)0x01);
  Wire.endTransmission();

  //delayMicroseconds(2);

  // Read out data using multiple byte read mode
  Wire.requestFrom(MAG_address, 6);
  while( Wire.available() != 6 ) {}

  // Combine registers
  uint16_t values[3];
  for(uint8_t idx = 0; idx <= 2; idx++)
  {
    values[idx]  = Wire.read() << 8;  // MSB
    values[idx] |= Wire.read();     // LSB
  }

  // Put data into referenced variables
  x = (int) values[0];
  y = (int) values[1];
  z = (int) values[2];
  
  xmax = max(xmax, x);
  xmin = min(xmin, x);
  ymax = max(ymax, y);
  ymin = min(ymin, y);
  zmax = max(zmax, z);
  zmin = min(zmin, z);

  *xR = x - (xmax + xmin) / 2;
  *yR = y - (ymax + ymin) / 2;
  *zR = z - (zmax + zmin) / 2;
}

void getHeading() {
  int x, y, z;
  MAG(&x, &y, &z);
  heading = atan2(-y, x) * DEG_PER_RAD + 180;
}

float getBearing() {
  int x, y, z;
  MAG(&x, &y, &z);
  return atan2(-y, x) * DEG_PER_RAD + 180;
}

// Clears all our counters
void clearCounters()
{
	leftForwardTicks = 0; 
	rightForwardTicks = 0;
	leftReverseTicks = 0; 
	rightReverseTicks = 0;
	leftForwardTicksTurns = 0; 
	rightForwardTicksTurns = 0;
	leftReverseTicksTurns = 0; 
	rightReverseTicksTurns = 0;
	leftRevs=0;
	rightRevs=0;
	forwardDist=0;
	reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
	clearCounters();
}
// Initialize Vincent's internal states

void initializeState()
{
	clearCounters();
}

void setupIR() {
	// pin 12 for leftIR -> (PB4)
	// pin 13 for rightIR -> (PB5)
	DDRB &= 0b11001111;	// set as input
}

bool hasLeftObstacle() {
	if (PINB & 0b00010000)	return false;
	else	return true;
}

bool hasRightObstacle() {
	if (PINB & 0b00100000)	return false;
	else	return true;
}


// Light up red led for debugging
void lightRed() 
{
	PORTD |= 0b00010000;
	delay(500);
	PORTD &= 0b11101111;
	delay(500);
}
