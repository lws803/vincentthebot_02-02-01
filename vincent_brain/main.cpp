#include <stdio.h>
#include <iostream>
#include <fstream>
#include <utility>
#include <tuple>
#include <deque>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"

using namespace std;

#define PORT_NAME					"/dev/ttyACM0"
#define BAUD_RATE					B9600

// Defined constants to indicate directions, distance and speed
#define DEFAULT_SPEED				100 // Default move/turn speed
#define DEFAULT_LEFT_TURN_SPEED		100 // Default left turn speed
#define DEFAULT_RIGHT_TURN_SPEED	100 // Default right turn speed
#define GRID_UNIT_DISTANCE			20 // Assume grid unit distance to be wheel circumference
// Defined constants for movement command type
#define MOVE_COMMAND 				0 // For forward/backward
#define TURN_COMMAND 				1 // For turning
#define NON_MOVEMENT_COMMAND 		2 // For non movement commands
#define END_COMMAND 				3 // Vincent has reached the end point
#define UNDO_COMMAND 				4 // Command to specify undo action
// Max retries for failed autonomous command
#define MAX_AUTO_FAIL 				5

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
 * Global Variables
 */
int exitFlag=0;
sem_t _xmitSema;
// Command ready flag
volatile bool READY_FLAG = true;
// General movement flags
volatile bool isMoving = false;
// Keep track of autonomous mode routine
volatile bool AUTONOMOUS_FLAG = false;
volatile bool AUTO_RECEIVE_OK = false;
float currentHeading = 0;
float nextHeading = 0;
int gridSteps = 0;
// Backtracking variables
deque<commandTuple> backStack;
// Output file for realtime reading
//ofstream outputFile;

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
void printCommandList();
commandTuple executeUserCommand();
void flushInput();
float getParams(TPacket *commandPacket);
float sendCommand(char command);
void invertCommand(commandTuple *tup);
void pushCmdToStack(commandTuple *tup);
void pushToStack();
void popFromStack();
void printCmdStack();
void processCommand(commandTuple cmdTup);
void setEndPoint(TPacket *commandPacket);
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
	// Falied autonomous command counter
	int autoFailedCount = 0;
	
	// Create a new screen output file
	//outputFile.open("output.txt");

	while(!exitFlag)
	{	
		/*
		// Wait for Arduino to send the ready status to indicate ready
		// to receive the next command
		bool rdyMsg = false;
		while (READY_FLAG == false) {
			if (!rdyMsg && isMoving) {
				printf("Vincent not ready, standby..\n");
				rdyMsg = true;
			}
		}
		*/

		/* 
		 * TWO different modes of movement
		 * AUTONOMOUS
		 * 	Vincent moves semi autonomously by reading commands
		 * 	from the command stack.
		 * 	One command, one movement. 
		 * 	Vincent only reads the next command when Arduino sends
		 * 	back a "ready" packet back.
		 * 
		 * REMOTE
		 * 	Vincent moves based on the command input by operator.
		 * 	Vincent only reads the next command when Arduino sends
		 * 	back a "ready" packet back.
		 */
		if (AUTONOMOUS_FLAG) {
			// Check if the backtracking command stack is empty
			// If it is, stop autonomous mode
			if (backStack.empty() || autoFailedCount == MAX_AUTO_FAIL) {
				AUTONOMOUS_FLAG = false;
				continue;
			}
			
			// Continuously process autonomous commands
			// Inform Arduino incoming autonomous packet
			sendPacket(&autoPacket);
			if (AUTO_RECEIVE_OK) {
				// Process the next command
				backStack.pop_back();
				processCommand(backStack.back());
				autoFailedCount = 0;
			}
			else {
				// Re-process the current failed command
				processCommand(backStack.back());
				autoFailedCount++;
			}
		}
		else {
			// Retrieve raw data from LIDAR
			//getLidarData(&currentHeading, &nextHeading, &gridSteps);
			// Process raw data from LIDAR
			rawDataCommandPair cmdPair = 
				processRawData(currentHeading, nextHeading, gridSteps);
			commandTuple inputCmd;
			
			printf("HEADING: %f\n", currentHeading);
			
			// If the turn angle is zero, we know we only needs to move
			// forward/backward
			if (get<1>(get<0>(cmdPair)) == 0) {
				
				// Place the input request in loop for undo ability
				do {
					printCommandList();
					// Get the forward/backward input
					inputCmd = executeUserCommand();
				} while (get<0>(inputCmd) == UNDO_COMMAND);
				
				// All movement commands get pushed to the stack
				if (get<0>(inputCmd) != NON_MOVEMENT_COMMAND) {
					// Invert the input commands for future back tracking
					invertCommand(&inputCmd);
					pushCmdToStack(&inputCmd);
				}

			}
			// We need to process both turn and forward/backward
			else {
				// Place the input request in loop for undo ability
				do {
					printCommandList();
					// Get the forward/backward input
					inputCmd = executeUserCommand();
				} while (get<0>(inputCmd) == UNDO_COMMAND);
				
				// Push command to stack
				if (get<0>(inputCmd) != NON_MOVEMENT_COMMAND) {
					// Invert the input commands for future back tracking
					invertCommand(&inputCmd);
					pushCmdToStack(&inputCmd);
				}
				
				// Place the input request in loop for undo ability
				do {
					printCommandList();
					// Get the forward/backward input
					inputCmd = executeUserCommand();
				} while (get<0>(inputCmd) == UNDO_COMMAND);
				
				// Push command to stack
				if (get<0>(inputCmd) != NON_MOVEMENT_COMMAND) {
					// Invert the input commands for future back tracking
					invertCommand(&inputCmd);
					pushCmdToStack(&inputCmd);
				}
			}
		}
	}

	//outputFile.close();
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
			printf("AUTONOMOUS Command OK\n");
			AUTO_RECEIVE_OK = true;
			break;
			
		case RESP_BAD_AUTO:
			printf("AUTONOMOUS Command BAD\n");
			AUTO_RECEIVE_OK = false;
			break;
			
		case RESP_MOVE:
			printf("Vincent started moving\n");
			isMoving = true;
			break;
			
		case RESP_STOP:
			printf("Vincent stopped moving\n");
			isMoving = false;
			break;
		
		case RESP_READY:
			printf("Vincent ready for next command\n");
			READY_FLAG = true;
			break;
			
		case RESP_HEADING:
			currentHeading = packet->params[0];
			READY_FLAG = true;
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
	printf("****************************************\n");
	printf("Current Heading		: %f\n", *currHead);
	printf("Next Heading		: %f\n", *nextHead);
	printf("Grid steps required	: %d\n", *steps);
	printf("****************************************\n");
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
	printf("****************************************\n\n");
}

void printCommandList() {
	printf("******************************\n");
	printf("Commands:\n");
	printf("f ---- forward\n");
	printf("b ---- reverse\n");
	printf("l ---- turn left\n"); 
	printf("r ---- turn right\n");
	printf("s ---- stop\n");
	printf("e ---- indicate end point is reached\n");
	printf("c ---- clear stats\n");
	printf("g ---- get stats\n");
	printf("h ---- get current heading\n");
	printf("a ---- autonomous mode\n");
	printf("p ---- print the command stack\n");
	printf("o ---- pop most recent command from stack\n");
	printf("i ---- push a custom command into the stack\n");
	printf("q ---- exit\n");
	printf("******************************\n");
}

// This simply executes the command 
commandTuple executeUserCommand() {	
	commandTuple cmdTup;			
	char ch;
	float value = 0;
	
	// Request user for command
	printf("Input: ");
	scanf("%c", &ch);
	printf("\n\n");
	
	// Purge extraneous characters from input stream
	flushInput();
	
	// Check if input conflicts with Arduino
	// If Arduino is not ready and user tries to input a movement
	// command that is not "Stop", we prevent the command from sending
	if (!(READY_FLAG || ch == 's')) {
		printf("Arduino is not ready! Stop Vincent first.\n\n");
		get<0>(cmdTup) = UNDO_COMMAND;
		return cmdTup;
	}
	
	// Process the character input and send to Arduino
	value = sendCommand(ch);
	
	// Undo option in the case where user wants to change command
	if (value < 0) {
		get<0>(cmdTup) = UNDO_COMMAND;
		return cmdTup;
	}
	
	// Build the command tuple
	if (ch == 'f' || ch == 'F' ||ch == 'b' || ch == 'B') {
		get<0>(cmdTup) = MOVE_COMMAND;
		get<1>(cmdTup).push_back(ch);
	}
	else if (ch == 'l' || ch == 'L' ||ch == 'r' || ch == 'R') {
		get<0>(cmdTup) = TURN_COMMAND;
		get<1>(cmdTup).push_back(ch);	
	}
	else {
		get<0>(cmdTup) = NON_MOVEMENT_COMMAND;
	}
	get<2>(cmdTup) = value;
	
	return cmdTup;
}

void flushInput()
{
	char c;
	while((c = getchar()) != '\n' && c != EOF);
}

float getParams(TPacket *commandPacket)
{
	int value, power;
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	printf("TO UNDO: Input negative for any of the values.\n");
	scanf("%d %d", &value, &power);
	flushInput();
	printf("\n\n");
	
	// Undo option
	if (value < 0 || power < 0) {
		return -1;
	}
	
	// Build the command packet with given input
	commandPacket->params[1] = power;
	commandPacket->params[0] = value;
	
	return (float)value;
}

float sendCommand(char command) {
	TPacket commandPacket;
	commandPacket.packetType = PACKET_TYPE_COMMAND;

	float value = 0;

	switch(command)
	{
		case 'f':
		case 'F':
			value = getParams(&commandPacket);
			if (value < 0) return value;
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			READY_FLAG = false;
			break;

		case 'b':
		case 'B':
			value = getParams(&commandPacket);
			if (value < 0) return value;
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			READY_FLAG = false;
			break;

		case 'l':
		case 'L':
			value = getParams(&commandPacket);
			if (value < 0) return value;
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			READY_FLAG = false;
			break;

		case 'r':
		case 'R':
			value = getParams(&commandPacket);
			if (value < 0) return value;
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			READY_FLAG = false;
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			READY_FLAG = false;
			break;

		case 'e':
		case 'E':
			setEndPoint(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			sendPacket(&commandPacket);
			READY_FLAG = false;
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			READY_FLAG = false;
			break;
			
		case 'h':
		case 'H':
			commandPacket.command = COMMAND_GET_HEADING;
			sendPacket(&commandPacket);
			READY_FLAG = false;
			break;
		
		case 'a':
		case 'A':
			AUTONOMOUS_FLAG = true;
			READY_FLAG = false;
			//getLidarData(&currentHeading, &nextHeading, &gridSteps);
			printf("Switching to AUTONOMOUS mode...\n");
			printf("To stop any time, input 's'\n");
			commandPacket.command = COMMAND_AUTO_MODE;
			sendPacket(&commandPacket);
			sleep(2);  // Wait a little while..
			break;

		case 'p':
		case 'P':
			printCmdStack();
			break;
			
		case 'o':
		case 'O':
			popFromStack();
			break;
			
		case 'i':
		case 'I':
			pushToStack();
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

// Invert the input command for back tracking in future
void invertCommand(commandTuple *tup) {
	if (get<0>(*tup) == TURN_COMMAND) {
		if (get<1>(*tup) == "l" || get<1>(*tup) == "L") 
			get<1>(*tup) = "r";
		else 
			get<1>(*tup) = "l";
	}
}

// Call this to push a command tuple into the back track command
// stack
void pushCmdToStack(commandTuple *tup) {
	backStack.push_back(*tup);
}

// Call this to push in a custom command into the stack
void pushToStack() {
	commandTuple tup;
	char ch;
	float value;
	
	printf("Give a direction (f,b,l,r): ");
	scanf("%c", &ch);
	// Purge extraneous characters from input stream
	flushInput();
	
	printf("Give a value (distance for forward/backward, turn degree for left/right): ");
	scanf("%f", &value);
	flushInput();
	
	// Build the command tuple
	if (ch == 'l' || ch == 'L' || ch == 'r' || ch == 'R') {
		get<0>(tup) = TURN_COMMAND;
	}
	else if (ch == 'f' || ch == 'F' || ch == 'b' || ch == 'B') {
		get<0>(tup) = MOVE_COMMAND;
	}
	get<1>(tup) = ch;
	get<2>(tup) = value;
	
	// Push the command to stack
	pushCmdToStack(&tup);
}

// Call this to pop the most top command in the back track command stack
void popFromStack() {
	printf("CURRENT STACK\n");
	printCmdStack();
	printf("Now we pop the most recent command..\n");
	backStack.pop_back();
	printCmdStack();
}

// Prints the back tracking command stack
void printCmdStack() {
	
	printf("==================================================\n");
	printf("\t\tMOVEMENT STACK\n");
	printf("==================================================\n");
	if (backStack.empty()) printf("\t\tSTACK IS EMPTY!\n");
	for (auto it = backStack.rbegin(); it != backStack.rend(); it++) {
		if (get<1>(*it) == "f") {
			printf("FORWARD - - -|| - - - %0.2f CM\n", get<2>(*it));
		}
		else if (get<1>(*it) == "b") {
			printf("BACKWARD - - || - - - %0.2f CM\n", get<2>(*it));
		}
		else if (get<1>(*it) == "l") {
			printf("LEFT - - - - || - - - %0.2f DEGREE\n", get<2>(*it));
		}
		else {
			printf("RIGHT - - - -|| - - - %0.2f DEGREE\n", get<2>(*it));
		}
	}
	
	printf("==================================================\n\n");
	
	
	/* TODO: Experimental: append screen output to another file instead
	 * for realtime reading. Less cluttering on main terminal.
	outputFile << "==================================================\n";
	for (auto it = backStack.rbegin(); it != backStack.rend(); it++) {
		if (get<1>(*it) == "f") {
			outputFile << "FORWARD - - - || - - - " << get<2>(*it) << " CM\n";
		}
		else if (get<1>(*it) == "b") {
			outputFile << "BACKWARD - - -|| - - - " << get<2>(*it) << " CM\n";
		}
		else if (get<1>(*it) == "l") {
			outputFile << "LEFT - - - - -|| - - - " << get<2>(*it) << " DEGREE\n";
		}
		else {
			outputFile << "RIGHT - - - - || - - - " << get<2>(*it) << " DEGREE\n";
		}
	}
	
	outputFile << "==================================================\n\n";
	outputFile.close();
	outputFile.open("output.txt");
	*/
}

// Process and execute command from a commandTuple object
void processCommand(commandTuple cmdTup) {
	TPacket commandPacket;
	commandPacket.packetType = PACKET_TYPE_COMMAND;
	
	switch(get<1>(cmdTup)[0]) {
		case 'f':
		case 'F':
			commandPacket.command = COMMAND_FORWARD;
			commandPacket.params[1] = DEFAULT_SPEED;
			break;
		case 'b':
		case 'B':
			commandPacket.command = COMMAND_REVERSE;
			commandPacket.params[1] = DEFAULT_SPEED;
			break;
		case 'l':
		case 'L':
			commandPacket.command = COMMAND_TURN_LEFT;
			commandPacket.params[1] = DEFAULT_LEFT_TURN_SPEED;
			break;
		case 'r':
		case 'R':
			commandPacket.command = COMMAND_TURN_RIGHT;
			commandPacket.params[1] = DEFAULT_RIGHT_TURN_SPEED;
			break;
		default:
			printf("Invalid command from stack");
	}
	
	// Set the distance/angle value in command
	commandPacket.params[0] = get<2>(cmdTup);
	
	// Send the command to Arduino
	sendPacket(&commandPacket);
	
	READY_FLAG = false;
}

void setEndPoint(TPacket *commandPacket) {
	// Ensure Vincent fully comes to a stop
	commandPacket->packetType = PACKET_TYPE_COMMAND;
	commandPacket->command = COMMAND_STOP;
	sendPacket(commandPacket);
	
	// Wait for Arduino to be ready
	READY_FLAG = false;
	while (!READY_FLAG) {}
	
	// Rotate Vincent 180 degree to the back
	TPacket endPacket;
	endPacket.packetType = PACKET_TYPE_COMMAND;
	endPacket.command = COMMAND_TURN_RIGHT;
	endPacket.params[0] = 180;
	endPacket.params[1] = DEFAULT_RIGHT_TURN_SPEED;
	sendPacket(&endPacket);
	isMoving = true;
	READY_FLAG = false;
	
	// Insert a dummy command into the stack
	commandTuple cmdTup;
	get<0>(cmdTup) = MOVE_COMMAND;
	get<1>(cmdTup).push_back('f');
	get<2>(cmdTup) = 0;
	pushCmdToStack(&cmdTup);
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
rawDataCommandPair processRawData(float currentHeading, 
	float nextHeading, int gridSteps) {
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
