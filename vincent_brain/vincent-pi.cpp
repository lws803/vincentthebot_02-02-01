#include <stdio.h>
#include <iostream>
#include <utility>
#include <tuple>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"

using namespace std;

#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

// Defined constants to indicate directions, distance and speed
#define DEFAULT_SPEED 100 // Default move/turn speed
#define GRID_UNIT_DISTANCE 20 // Assume grid unit distance to be wheel circumference
// Defined constants for movement command type
#define MOVE_COMMAND 0 // For forward/backward
#define TURN_COMMAND 1 // For turning

/*
 * Global Variables
 */
int exitFlag=0;
sem_t _xmitSema;
// Keep track of autonomous mode routine
volatile bool AUTONOMOUS_FLAG = false;
volatile bool AUTO_RECEIVE_OK = false;
volatile bool AUTO_RECEIVE_BAD = false;
float currentHeading = 0;
float nextHeading = 0;
int gridSteps = 0;

/*
 * Structures definitions
 */
// LIDAR raw data to commands Pair
// arg 1: Turn command (arg1: direction, arg2: angle)
// arg 2: Forward/backward command (arg1: direction, arg2: distance)
typedef pair< pair<string, float> , pair<string, float> > rawDataCommandPair;
// Execution command tuple
// arg 1: Command type --> TURN or MOVE
// arg 2: Direction
// arg 3: Angle/Distance
typedef tuple<int, string, float> commandTuple;

/*
 * Function prototypes
 */
// Handling packets and errors
void handleError(TResult error);
void handleStatus(TPacket *packet);
void handleResponse(TPacket *packet);
void handleErrorResponse(TPacket *packet);
void handleMessage(TPacket *packet);
void handlePacket(TPacket *packet);
void sendPacket(TPacket *packet);
void *receiveThread(void *p);
// Process MANUAL commands
void printInstructions(float *currHead, float *nextHead, int *steps, rawDataCommandPair *pair);
commandTuple executeUserCommand();
void flushInput();
float getParams(TPacket *commandPacket);
float sendCommand(char command);
// Process raw data from LIDAR
rawDataCommandPair processRawData(float currentHeading, 
	float nextHeading, int gridSteps);

/*
 * Main program
 */
int main()
{
	// Connect to the Arduino
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

	// Sleep for two seconds
	printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
	sleep(2);
	printf("DONE\n");

	// Spawn receiver thread
	pthread_t recv;

	pthread_create(&recv, NULL, receiveThread, NULL);

	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);
	
	// Autonomous packet
	TPacket autoPacket;
	autoPacket.packetType = PACKET_TYPE_AUTO;

	while(!exitFlag)
	{
		if (AUTONOMOUS_FLAG) {
			// Continuously process autonomous commands
			// Inform Arduino incoming autonomous packet
			sendPacket(&autoPacket);
			if (AUTO_RECEIVE_OK) {
				// Process the next command
				//processCommands(currentHeading, nextHeading, gridSteps);
				//getLidarData(&currentHeading, &nextHeading, &gridSteps);
			}
			else {
				// Re-process the current failed command
				//processCommands(currentHeading, nextHeading, gridSteps);
			}
		}
		else {
			// Retrieve raw data from LIDAR
			//getLidarData(&currentHeading, &nextHeading, &gridSteps);
			// Process raw data from LIDAR
			rawDataCommandPair cmdPair = 
				processRawData(currentHeading, nextHeading, gridSteps);
			commandTuple inputCmd;
			
			// Print the raw data and required command to standard
			// output
			printInstructions(&currentHeading, &nextHeading, &gridSteps, &cmdPair);
			
			// TODO: UI mismatch. Reponse is received after printing
			// the prompt message. Hence, response message gets printed
			// AFTER prompt message. FIX THIS.
			
			// We only need to process one command, forward/backward
			if (get<1>(get<0>(cmdPair)) == 0) {
				// Get the forward/backward input
				inputCmd = executeUserCommand();

			}
			// We need to process both turn and forward/backward
			else {
				// Get the turn input
				inputCmd = executeUserCommand();
				// Get the forward/backward input
				inputCmd = executeUserCommand();
			}
			
			// Invert the input commands for future back tracking
			//invertCommands(inputCmd);
			
		}
	}

	printf("Closing connection to Arduino.\n");
	endSerial();
	
	return 0;
}

void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n");
			break;

		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n");
			break;

		default:
			printf("ERROR: UNKNOWN ERROR\n");
	}
}

void handleStatus(TPacket *packet)
{
	printf("\n ------- VINCENT STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", packet->params[0]);
	printf("Right Forward Ticks:\t\t%d\n", packet->params[1]);
	printf("Left Reverse Ticks:\t\t%d\n", packet->params[2]);
	printf("Right Reverse Ticks:\t\t%d\n", packet->params[3]);
	printf("Left Forward Ticks Turns:\t%d\n", packet->params[4]);
	printf("Right Forward Ticks Turns:\t%d\n", packet->params[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", packet->params[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", packet->params[7]);
	printf("Forward Distance:\t\t%d\n", packet->params[8]);
	printf("Reverse Distance:\t\t%d\n", packet->params[9]);
	printf("\n---------------------------------------\n\n");

}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			printf("Command OK\n");
			break;

		case RESP_STATUS:
			handleStatus(packet);
			break;

		case RESP_OK_AUTO:
			printf("AUTONOMOUS Command OK");
			AUTO_RECEIVE_OK = true;
			break;
			
		case RESP_BAD_AUTO:
			printf("AUTONOMOUS Command BAD");
			AUTO_RECEIVE_BAD = false;
			break;

		default:
			printf("Arduino is confused\n");
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			printf("Arduino received bad magic number\n");
		break;

		case RESP_BAD_CHECKSUM:
			printf("Arduino received bad checksum\n");
		break;

		case RESP_BAD_COMMAND:
			printf("Arduino received bad command\n");
		break;

		case RESP_BAD_RESPONSE:
			printf("Arduino received unexpected response\n");
		break;

		default:
			printf("Arduino reports a weird error\n");
	}
}

void handleMessage(TPacket *packet)
{
	printf("Message from Vincent: %s\n", packet->data);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	}
}

void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				}
		}
	}
}

// This prints the current positional state of Vincent and the required
// commands for next movement
void printInstructions(float *currHead, float *nextHead, int *steps, rawDataCommandPair *pair) {
	printf("******************************\n");
	printf("Current Heading		: %f\n", *currHead);
	printf("Next Heading		: %f\n", *nextHead);
	printf("Grid steps required	: %d\n", *steps);
	printf("******************************\n");
	printf("Movement commands required:\n");
	// Vincent needs to turn first
	if ((pair->first).second != 0) {
		if ((pair->first).first == "r") printf("TURN RIGHT by ");
		else printf("TURN LEFT by ");
		
		printf("%0.2f degree\n", (pair->first).second);
	}
	
	if ((pair->second).first == "f") printf("FORWARD by ");
	else printf("BACKWARD by ");
	printf("%0.2f CM\n", (pair->second).second);
	printf("******************************\n\n");
}

// This simply executes the command 
commandTuple executeUserCommand() {	
	commandTuple cmdTup;			
	char ch;
	
	printf("******************************\n");
	printf("Commands:\n");
	printf("f ---- forward\n");
	printf("b ---- reverse\n");
	printf("l ---- turn left\n"); 
	printf("r ---- turn right\n");
	printf("s ---- stop\n");
	printf("c ---- clear stats\n");
	printf("g ---- get stats\n");
	printf("a ---- autonomous mode\n");
	printf("q ---- exit\n");
	printf("******************************\n");
	printf("Input:	");
	scanf("%c", &ch);
	printf("\n");
	
	// Purge extraneous characters from input stream
	flushInput();
	
	float value = sendCommand(ch);
	
	if (ch == 'f' || ch == 'F' ||ch == 'b' || ch == 'B') {
		get<0>(cmdTup) = MOVE_COMMAND;
		get<2>(cmdTup) = value;
	}
	else if (ch == 'l' || ch == 'L' ||ch == 'r' || ch == 'R') {
		get<0>(cmdTup) = TURN_COMMAND;
		get<2>(cmdTup) = value;
	}
	get<1>(cmdTup).push_back(ch);
	
	return cmdTup;
}

/* This takes in the raw data command pair and does TWO things:
 * 
 * 1. Invert the commands for backtracking in future
 * 2. Push the inverted commands into the backtracking stack
 */

void flushInput()
{
	char c;
	while((c = getchar()) != '\n' && c != EOF);
}

float getParams(TPacket *commandPacket)
{
	int value;
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d %d", &value, &commandPacket->params[1]);
	flushInput();
	
	commandPacket->params[0] = value;
	
	return (float)value;
}

float sendCommand(char command)
{
	TPacket commandPacket;
	commandPacket.packetType = PACKET_TYPE_COMMAND;

	float value;

	switch(command)
	{
		case 'f':
		case 'F':
			value = getParams(&commandPacket);
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			break;

		case 'b':
		case 'B':
			value = getParams(&commandPacket);
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;

		case 'l':
		case 'L':
			value = getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'r':
		case 'R':
			value = getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			sendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			break;
		
		case 'a':
		case 'A':
			AUTONOMOUS_FLAG = true;
			//getLidarData(&currentHeading, &nextHeading, &gridSteps);
			printf("Switching to AUTONOMOUS mode...");
			break;

		case 'q':
		case 'Q':
			exitFlag=1;
			break;

		default:
			printf("Bad command\n");

	}
	
	return value;
}

/*
 * Retrieve navigation raw data from RPiLidar and process them
 * to become movement commands readable by Arduino
 * 
 * Current heading reports the current direction Vincent is facing,
 * where 0 <= currentHeading <= 359
 * 
 * Next heading determines the direction of travel for Vincent, where
 * 0 <= nextHeading <= 359
 * 
 * Grid steps is the number of steps, with reference to the GRID in the
 * navigation map, Vincent is required to move. This determines the 
 *
 * If both headings and grid steps are non-zero, Vincent has to spot
 * turn to the next heading direction THEN moves forward by grid steps
 * This means there are effectively two commands
 * If only gridSteps is non-zero, Vincent only moves forward/backward 
 * 
 * Thus, we either return 1 or 2 movement commands. We wrap them in the
 * defined rawDataCommandPair
 */
rawDataCommandPair processRawData(float currentHeading, float nextHeading, 
	int gridSteps) {
	// Raw data to command pair
	rawDataCommandPair cmdPair;
		
	// First, Vincent determines the movement distance
	float moveDistance = 0;
	if (gridSteps > 0) {
		 moveDistance = (float)gridSteps * GRID_UNIT_DISTANCE; 
	}
	
	// Next, Vincent needs to determine the direction of based on the
	// given heading
	float turnAngle = nextHeading-currentHeading;
	string direction;
	
	// We can use -180 < x <= 180 (degrees) 
	// with negative going left, positive going right 
	// Xavier:	Implemented Matthew's method and corrected sign error
	//			Change 180 degre turn angle into the case where Vincent
	//			will ground turn instead of reverse.
	if (turnAngle > 180)
		turnAngle -= 360;
	else if (turnAngle < -180)
		turnAngle += 360;
	
	if (turnAngle > 0) {
		direction = "r";
	} 
	else if (turnAngle < 0) {
		direction = "l";
		turnAngle = -turnAngle;
	} 
	else if (turnAngle == -180 || turnAngle == 180) {
		direction = "f";
	} 
	else {
		direction = "f";
	}
	
	cmdPair = make_pair(make_pair(direction, turnAngle), make_pair(direction, moveDistance));
	
	return cmdPair;
}
