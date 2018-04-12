#include <serialize.h>
#include <math.h>
#include <Wire.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "packet.h"
#include "constants.h"
#include <A4990MotorShield.h>
//#include "Motors.h"


/*
 * TODO:
 * 1. Convert AnalogWrite to bare metal programming (if we have time)
 * 2. Stack Nav, make sure it doesn't give bad checksum
 * 3. Calibrate with new motors
 * 4. Magnetometer XOR Ticks
 * 
 */

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
	RIGHT=4
} TDirection;

////////////////////////////////////////////////////////
/*
const unsigned char _MTR1DIR = 7;	// PORTD (PD7)
const unsigned char _MTR2DIR = 8;	// PORTB (PB0)	
const unsigned char _MTR1PWM = 9;	// PORTB (PB1)
const unsigned char _MTR2PWM = 10;	// PORTB (PB2)

volatile int currSpeed;

void initMotors() {
	// initialize pin states 
	
	// write to _MTR1PWM
	PORTB &= 0b11111101;	// write low
	DDRB |= 0b00000010;
	PORTB &= 0b11111101;	// write low
	
	// write to _MTR2PWM
	PORTB &= 0b11111011;	// write low
	DDRB |= 0b00000100;
	PORTB &= 0b11111011;	// write low
	
	// write to _MTR1DIR
	PORTD &= 0b01111111;	// write low
	DDRD |= 0b10000000;
	PORTD &= 0b01111111;	// write low
	
	// write to _MTR2DIR
	PORTB &= 0b11111110;	// write low
	DDRB |= 0b00000001;
	PORTB &= 0b11111110;	// write low
	
	// enable interrupts for timer1
	TIMSK1 |= 0b110;
	
	#ifdef MOTOR_USE_20KHZ_PWM
		// timer 1 configuration
		// prescaler: clockI/O / 1
		// outputs enabled
		// phase-correct PWM
		// top of 400
		//
		// PWM frequency calculation
		// 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
		TCCR1A = 0b10100000;
		TCCR1B = 0b00010001;
		TCNT0 = 0;
		ICR1 = 400; 
	#endif
}


void setM1Speed(int speed) {
	init();
	currSpeed = speed;
	
	if (speed < 0) 
		speed = -speed;
	else if (speed > 400)
		speed = 400;
	
	#ifdef	MOTOR_USE_20KHZ_PWM
		OCR1A = speed;
	#endif
	
	TCCR1A |= 0b10000001;	// sets timer 1
	PORTB |= 0b00000010;	// write high
	PORTD |= 0b10000000;	// write high
	TCCR1B |= 0b00000001;	// starts timer 1
	
	if (speed > 255)
		OCR1B = 255;
	else (speed >= 0)
		OCR1B = speed;
}

void setM2Speed(int speed) {
	init();
	currSpeed = speed;
	
	if (speed < 0) 
		speed = -speed;
	else if (speed > 400)
		speed = 400;
	
	
	//#ifdef	MOTOR_USE_20KHZ_PWM
		//OCR1B = speed;//
	//#endif//
	
	TCCR1A = 0b10000001;	// sets timer 1
	PORTB &= 0b00000001;	// write low
	PORTB &= 0b00000100;	// write low
	TCCR1B |= 0b00000001;	// starts timer 1`
	
	OCR1A = speed;
}

void setSpeeds(int m1Speed, int m2Speed) {
	setM1Speed(m1Speed);
	setM2Speed(m2Speed);
}
*/
////////////////////////////////////////////////


volatile TDirection dir = STOP;
/*
 * Vincent's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.
#define COUNTS_PER_REV 140

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
//ISR (INT0_vect);
//ISR (INT1_vect);

// Set up serial communications
void setupSerial();
void startSerial();
int readSerial(char *buffer);
void writeSerial(const char *buffer, int len);

// Start the motors
void setupMotors();
void startMotors();

/*
   void right_motor_forward(void);
   void right_motor_reverse(void);
   void left_motor_forward(void);
   void left_motor_reverse(void);
 */

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
	// DDRD |= 0b00010000;

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
			if (forwardDist > newDist) {
				deltaDist = 0;
				newDist = 0;
				stop();
			}
			
			// check right and left obstacles
			if (hasLeftObstacle() && !hasRightObstacle()) {
				adjustRight(NEED_ADJUST_RIGHT);
				sendMessage("adjusting right!");
			}

			else if (hasRightObstacle() && !hasLeftObstacle()) {
				adjustLeft(NEED_ADJUST_LEFT);
				sendMessage("adjusting left!");
			}
			
			else if (!hasLeftObstacle() && !hasRightObstacle()) {
				normalizeSpeed();				
				sendMessage("normalizing!");
			}
			
		}
		else if (dir == BACKWARD) {
			if (reverseDist > newDist) {
				deltaDist = 0;
				newDist = 0;
				stop();
			}
			
			// check right and left obstacles
			if (hasLeftObstacle()) {
				adjustRight(NEED_ADJUST_RIGHT);
			} else if (hasRightObstacle()) {
				adjustLeft(NEED_ADJUST_LEFT);
			}
		}
		else if (dir == STOP) {
			deltaDist = 0;
			newDist = 0;
			stop();
		}
	}
	
	
	
	/*
	// Check when Vincent can stop turning left/right after
	// it is given a fixed angle to turn left/right
	if (deltaTicks > 0) {
		if (dir == LEFT) {
			if (rightForwardTicksTurns >= targetTicks) {
				deltaTicks = 0;
				targetTicks = 0;
				stop();
			}
		}
		else if (dir == RIGHT) {
			if (leftForwardTicksTurns >= targetTicks) {
				deltaTicks = 0;
				targetTicks = 0;
				stop();
			}
		}
		else if (dir == STOP) {
			deltaTicks = 0;
			targetTicks = 0;
			stop();
		}
	}*/

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

	// Handle packets differently if autonomous or remote
	// 
	// TODO: Do we really need to handle packets this way during 
	// autonomous mode? Is it needed?
	/*
	   if (isAuto) {
	   if (result == PACKET_AUTO_OK) {
	   handlePacket(&recvPacket);
	   } else {
	   if (result == PACKET_BAD) {
	   sendBadPacket();
	   } else {
	   if (result == PACKET_CHECKSUM_BAD)
	   sendBadChecksum();
	   }
	   }

	   } else {
	   if(result == PACKET_OK)
	   handlePacket(&recvPacket);
	   else
	   if(result == PACKET_BAD)
	   {
	   sendBadPacket();
	   }
	   else
	   if(result == PACKET_CHECKSUM_BAD)
	   sendBadChecksum();
	   }
	 */

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
		case COMMAND_REVERSE:
			sendOK();
			reverse((float) command->params[0], (float) command->params[1]);
			break;
		case COMMAND_TURN_LEFT:
			sendOK();
			//left((float) command->params[0], (float) command->params[1]);
			leftMAG((float) command->params[0], (float) command->params[1]);
			break;
		case COMMAND_TURN_RIGHT:
			sendOK();
			//right((float) command->params[0], (float) command->params[1]);
			rightMAG((float) command->params[0], (float) command->params[1]);
			break;
		case COMMAND_ADJUST_LEFT:
			sendOK();
			adjustLeft((float) command->params[0]);
			break;
		case COMMAND_ADJUST_RIGHT:
			sendOK();
			adjustRight((float) command->params[0]);
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
			sendReady();
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
	if (dir == FORWARD) {
		leftForwardTicks++;
		forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
	}
	else if (dir == BACKWARD) {
		leftReverseTicks++;
		reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
	}
	else if (dir == RIGHT) {
		//rightReverseTicksTurns++;
		leftForwardTicksTurns++;
	}
	//sendMessage("left\n");
	//Serial.print("DIST: ");
	//Serial.println(forwardDist);
}

void rightISR()
{
	if (dir == FORWARD) {
		rightForwardTicks++;
		forwardDist = (unsigned long) ((float) rightForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
	} else if (dir == BACKWARD) {
		rightReverseTicks++;
		reverseDist = (unsigned long) ((float) rightReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
	} else if (dir == LEFT) {
		//rightReverseTicksTurns++;
		rightForwardTicksTurns++;
	}  

	//Serial.print("RIGHT: ");
	//Serial.println((double)rightTicks/COUNTS_PER_REV * WHEEL_CIRC);

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

	// Check for adjustments
	//if (getAdjustReadings() == NEED_ADJUST_LEFT) adjustLeft(100);
	//else adjustRight(100);
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
void setupSerial()
{
	// To replace later with bare-metal.
	Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
	// Empty for now. To be replaced with bare-metal code
	// later on.

}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

	int count=0;

	while(Serial.available())
		buffer[count++] = Serial.read();

	return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
	Serial.write((const uint8_t*) (buffer), len);
}

/*
 * Vincent's motor drivers.
 * 
 */

// Set up Vincent's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
	/* Our motor set up is:  
	 *    A1IN - Pin 5, PD5, OC0B
	 *    A2IN - Pin 6, PD6, OC0A
	 *    B1IN - Pin 10, PB2, OC1B	
	 *    B2In - Pin 11, PB3, OC2A
	 */
	 
	 // 

}

// Start the PWM for Vincent's motors.
// We will implement this later. For now it is
// blank.
void startMotors(){

	// verify the ports
	TCCR0B = 0b0000011;	// 64 prescalar value
	TCCR2B = 0b0000100;	// 64 prescalar value
}


// Specific ports need to be verified in this part

/*
   void right_motor_forward(void) {
   TCCR0A = 0b10000001;
   PORTD &= 0b11011111;
   }

   void right_motor_reverse(void) {
   TCCR0A = 0b00100001;
   PORTD &= 0b10111111;
   }

   void left_motor_forward(void) {
   TCCR2A = 0b10000001;
   PORTD &= 0b11011111;
   }

   void left_motor_reverse(void) {
   TCCR2A = 0b00100001;
   PORTD &= 0b11011111;
   }
 */


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

	//int leftVal = pwmVal(speed);
	//int rightVal = leftVal + WHEEL_DIFF_FOR;

	// Compute the new total distance given the input
	if (dist > 0) deltaDist = dist;
	else deltaDist = 9999999;
	
	newDist = forwardDist + deltaDist;

	// LF = Left forward pin, LR = Left reverse pin
	// RF = Right forward pin, RR = Right reverse pin
	// This will be replaced later with bare-metal code.
	
	/*
	analogWrite(LF, leftVal);
	analogWrite(RF, rightVal);
	analogWrite(LR,0, 0);*/
	
	motors.setSpeeds(speed - MOTOR_CONST_LEFT, speed - MOTOR_CONST_RIGHT);
	//motors.setSpeeds(speed - MOTOR_CONST_LEFT, speed - MOTOR_CONST_RIGHT);
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
	
	
	//int rightVal = pwmVal(speed);
	//int leftVal = rightVal - WHEEL_DIFF_BAC;
	
	
	//int leftVal = pwmVal(speed);
	//int rightVal = leftVal + WHEEL_DIFF_BAC;

	// Compute the new total distance given the input
	if (dist > 0) deltaDist = dist;
	else deltaDist = 9999999;
	newDist = reverseDist + deltaDist;

	//setSpeeds(-speed + MOTOR_CONST_LEFT, -speed + MOTOR_CONST_RIGHT);
	motors.setSpeeds(-speed + MOTOR_CONST_LEFT, -speed + MOTOR_CONST_RIGHT);
	
	
	// LF = Left forward pin, LR = Left reverse pin
	// RF = Right forward pin, RR = Right reverse pin
	// This will be replaced later with bare-metal code.
	
	/*analogWrite(LR, leftVal);
	analogWrite(RR, leftVal);
	analogWrite(LF, 0);
	analogWrite(RF, 0);
	*/
	
}

// Function to estimate number of wheel ticks needed
// to turn an angle
unsigned long computeDeltaTicks(float ang) {
	unsigned long ticks = (unsigned long) ((ang * 
				vincentCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

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

	/*
	 * int rightVal = pwmVal(speed);
	 int leftVal = rightVal - WHEEL_DIFF_FOR;
	 * 
	 */

	//int leftVal = pwmVal(speed);
	//int rightVal = leftVal + WHEEL_DIFF_FOR;  

	// it is now moving
	sendMoveOK();

	// Compute the new total ticks needed to left turn
	if(ang == 0) deltaTicks=99999999; 
	else deltaTicks=computeDeltaTicks(ang); 
	targetTicks = rightForwardTicksTurns + deltaTicks;

	// To turn left we reverse the left wheel and move
	// the right wheel forward.
	//analogWrite(LR, 0);
	//analogWrite(RF, rightVal);
	//analogWrite(LF, 0);
	//analogWrite(RR, 0);
	
	// left motor does nothing, right motor moves
	//setSpeeds(0, speed);
	motors.setSpeeds(-speed, speed);  // spot turn
	//motors.setSpeeds(0, speed);
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

	//int leftVal = pwmVal(speed);
	//int rightVal = leftVal + WHEEL_DIFF_FOR;

	// it is now moving
	sendMoveOK();

	// Compute the new total ticks needed to right turn
	if(ang == 0) deltaTicks=99999999; 
	else deltaTicks=computeDeltaTicks(ang); 
	targetTicks = leftForwardTicksTurns + deltaTicks;

	// To turn right we reverse the right wheel and move
	// the left wheel forward.
	//analogWrite(RR, 0);
	//analogWrite(LF, leftVal);
	//analogWrite(LR, 0);
	//analogWrite(RF, 0);
	
	//setSpeeds(speed, 0);
	//motors.setSpeeds(speed, 0); spot turn
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

  motors.setSpeeds(-speed, speed);
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

void MAG(int* xR, int* yR, int* zR) {
  int x, y, z;
  static int xmax = -1024, xmin = 1024, ymax = -1024, ymin = 1024, zmax = -1024, zmin = 1024;
  
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
