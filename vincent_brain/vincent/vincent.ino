#include <serialize.h>
#include <math.h>
#include <Wire.h>
#include "packet.h"
#include "constants.h"

/*
 * TODO:
 * 1. Convert AnalogWrite to bare metal programming (if we have time)
 * 2. Stack Nav, make sure it doesn't give bad checksum
 * 3. Calibrate with new motors
 * 4. Magnetometer XOR Ticks
 * 
 */


typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;

volatile TDirection dir = STOP;
/*
 * Vincent's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.
#define COUNTS_PER_REV 95

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance travelled 
// by taking revs * WHEEL_CIRC
#define WHEEL_CIRC 20.4

// LEFT and RIGHT wheel PWM DIFFERENCE
#define WHEEL_DIFF_FOR -35 
#define WHEEL_DIFF_BAC -35

// Motor control pins. You need to adjust these till
// Vincent moves in the correct direction
#define LF 6   // Left forward pin -> PD6
#define LR 5   // Left reverse pin -> PD5
#define RF 11  // Right forward pin -> PB3
#define RR 10  // Right reverse pin -> PB2

// Determine if left/right adjustments are needed
#define NEED_ADJUST_LEFT 0 
#define NEED_ADJUST_RIGHT 1

// PI, for calculating turn circumference 
#define PI 3.1415923

// Vincent's length and breadth in cm
#define VINCENT_LENGTH 17.5
#define VINCENT_BREADTH 11

#define MAG_address 0x0E

// Vincent's diagonal. We compute and store this once
// since it is expensive to compute and really doesn't change
float vincentDiagonal = 0.0;

// Vincent's turning circumference, calculated once
float vincentCirc = 0.0;

/*
 *    Vincent's State Variables
 */

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

// Forward and backward distance travelled
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
float currentSpeed;

// Variable to store heading
double heading;

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
ISR (INT0_vect);
ISR (INT1_vect);

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
int getAdjustReadings();
void stop();

// Handle the statistics
void clearCounters();
void clearOneCounter(int which);
void initializeState();

//MAG3110
void MAG(int*, int*, int*);

// Debugging
void lightRed();

/*
 *
 * Setup Arduino
 *
 */
void setup() {

  // Setup PD4 as output pin for red led lighting
  DDRD |= 0b00010000;

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
  
  // Compute Vincent's diagonal and circumference
  vincentDiagonal = sqrt((VINCENT_LENGTH * VINCENT_LENGTH) 
    + (VINCENT_BREADTH * VINCENT_BREADTH));

  vincentCirc = PI * vincentDiagonal;
  Wire.beginTransmission(MAG_address);
  Wire.write((byte)0x02);
  Wire.write((byte)0x00);
  Wire.endTransmission();
}

/*
 *
 * Continuous loop
 *
 */
void loop() {
  /* BROKEN
  int MAG_x, MAG_y, MAG_z;
  MAG(&MAG_x, &MAG_y, &MAG_z);
  heading = atan2((double)MAG_y,(double)MAG_x);
  */
  
  // Check when Vincent can stop moving forward/backward after
  // it is given a fixed distance to move forward/backward
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
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
    
  // Check when Vincent can stop turning left/right after
  // it is given a fixed angle to turn left/right
  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (rightForwardTicksTurns >= targetTicks) 
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

void sendResponse(TPacket *packet)
{
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
      left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
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
    reverseDist = (unsigned long) ((float) leftReverseTicks /COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == RIGHT) {
    //rightReverseTicksTurns++;
    leftForwardTicksTurns++;
  }
  
  //Serial.print("DIST: ");
  //Serial.println(forwardDist);
}

void rightISR()
{
  if (dir == FORWARD) rightForwardTicks++;
  else if (dir == BACKWARD) rightReverseTicks++;
  else if (dir == LEFT) {
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
   *    B2In - pIN 11, PB3, OC2A
   */
   
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
  currentSpeed = speed;
  
  // it is now moving
  sendMoveOK();

/*
 * int leftVal = pwmVal(speed);
 * int rightVal = leftVal + WHEEL_DIFF_FOR;
 */
  
  int rightVal = pwmVal(speed);
  int leftVal = rightVal - WHEEL_DIFF_FOR;

  // Compute the new total distance given the input
  if (dist > 0) deltaDist = dist;
  else deltaDist = 9999999;
  newDist = forwardDist + deltaDist;

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.

  
  /* 
	OCR0A = leftVal;
  */
  
  
  analogWrite(LF, leftVal);
  analogWrite(RF, rightVal);
  analogWrite(LR,0);
  analogWrite(RR, 0);
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
  currentSpeed = speed;
  
   // it is now moving
  sendMoveOK();

  /*
 * int leftVal = pwmVal(speed);
 * int rightVal = leftVal + WHEEL_DIFF_BAC;
 */
 
  int rightVal = pwmVal(speed);
  int leftVal = rightVal - WHEEL_DIFF_BAC;
  
  // Compute the new total distance given the input
  if (dist > 0) deltaDist = dist;
  else deltaDist = 9999999;
  newDist = reverseDist + deltaDist;

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, leftVal);
  analogWrite(RR, rightVal);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
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
    
  int leftVal = pwmVal(speed);
  int rightVal = leftVal + WHEEL_DIFF_FOR;  
  
  // it is now moving
  sendMoveOK();

  // Compute the new total ticks needed to left turn
  if(ang == 0) deltaTicks=99999999; 
  else deltaTicks=computeDeltaTicks(ang); 
  targetTicks = rightForwardTicksTurns + deltaTicks;

  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(LR, 0);
  analogWrite(RF, rightVal);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
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
  
  int leftVal = pwmVal(speed);
  int rightVal = leftVal + WHEEL_DIFF_FOR;
  
  // it is now moving
  sendMoveOK();

  // Compute the new total ticks needed to right turn
  if(ang == 0) deltaTicks=99999999; 
  else deltaTicks=computeDeltaTicks(ang); 
  targetTicks = leftForwardTicksTurns + deltaTicks;

  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, 0);
  analogWrite(LF, leftVal);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Adjust Vincent left given degree of adjust
// TODO: Figure out the way to compute the degree of adjustment
void adjustLeft(float increment) 
{
  // Set the direction of travel
  dir = LEFT;
  
  // Compute the PWM values for Left-Forward and Right-Forward
  // wheel directions. The Right-Forward is greater than
  // Left-Forward with the difference depended on degree of
  // adjustment
  int leftVal = pwmVal(currentSpeed);
  int rightVal = pwmVal(currentSpeed + increment);
  
  analogWrite(LF, leftVal);
  analogWrite(RF, rightVal);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

// Adjust Vincent right given degree of adjust
// TODO: Figure out the way to compute the degree of adjustment
void adjustRight(float increment) 
{
  // Set the direction of travel
  dir = RIGHT;
  
  // Compute the PWM values for Left-Forward and Right-Forward
  // wheel directions. The Left-Forward is greater than
  // Right-Forward with the difference depended on degree of
  // adjustment
  int rightVal = pwmVal(currentSpeed);
  int leftVal = pwmVal(currentSpeed + increment);
  
  // Write the values to motors
  analogWrite(RF, rightVal);
  analogWrite(LF, leftVal);
  analogWrite(RR, 0);
  analogWrite(LR, 0);
}

// Determine if Vincent requires left/right adjustment by checking
// external sensor readings
// (RIGHT NOW USING IR SENSOR)
int getAdjustReadings()
{
  return NEED_ADJUST_LEFT;
}

// Stop Vincent. To replace with bare-metal code later.
void stop()
{
  dir = STOP;

  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
  
	sendStopOK();
	sendReady();
}

/*
 * Vincent's setup and run codes
 * 
 */


void MAG(int *x, int *y, int *z) {
  //Tell the MAg3110 where to begin reading data
  Wire.beginTransmission(MAG_address);
  Wire.write((byte)0x03); //select register 3, X MSB register
  Wire.endTransmission();
   
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(MAG_address, 6);
  if(6<=Wire.available()){
    *x = Wire.read() << 8; //X msb
    *x |= Wire.read();     //X lsb
    *z = Wire.read() << 8; //Z msb
    *z |= Wire.read();     //Z lsb
    *y = Wire.read() << 8; //Y msb
    *y |= Wire.read();     //Y lsb
  }
  else{   //return 0 value when data is unavailable or component is unplugged or malfuntioning
    *x=0;
    *y=0;
    *z=0;
  }
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

// Light up red led for debugging
void lightRed() 
{
  PORTD |= 0b00010000;
  delay(500);
  PORTD &= 0b11101111;
  delay(500);
}
