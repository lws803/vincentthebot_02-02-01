#ifndef __CONSTANTS_INC__
#define __CONSTANTS_INC__

/* 
 *  This file containts all the packet types, commands
 *  and status constants
 *  
 */

// Packet types
typedef enum
{
  PACKET_TYPE_COMMAND = 0,
  PACKET_TYPE_RESPONSE = 1,
  PACKET_TYPE_ERROR = 2,
  PACKET_TYPE_MESSAGE = 3,
  PACKET_TYPE_HELLO = 4,
  PACKET_TYPE_AUTO = 5,
  PACKET_TYPE_REMOTE = 6
} TPacketType;

// Response types. This goes into the command field
typedef enum
{
  RESP_OK = 0,
  RESP_STATUS=1,
  RESP_BAD_PACKET = 2,
  RESP_BAD_CHECKSUM = 3,
  RESP_BAD_COMMAND = 4,
  RESP_BAD_RESPONSE = 5,
  RESP_OK_AUTO = 6,
  RESP_BAD_AUTO = 7,
  RESP_STOP = 8,
  RESP_MOVE = 9,
  RESP_READY = 10,
  RESP_HEADING = 11
  
} TResponseType;


// Commands
// For direction commands, param[0] = distance in cm to move
// param[1] = speed
typedef enum
{

  COMMAND_FORWARD = 'a',
  COMMAND_FORWARD_IR = 'b',
  COMMAND_REVERSE = 'c',
  COMMAND_REVERSE_IR = 'd',
  COMMAND_TURN_LEFT = 'e',
  COMMAND_TURN_RIGHT = 'f',
  COMMAND_STOP = 'g',
  COMMAND_GET_STATS = 'h',
  COMMAND_CLEAR_STATS = 'i',
  COMMAND_AUTO_MODE = 'j',
  COMMAND_REMOTE_MODE = 'k',
  COMMAND_GET_HEADING = 'l'

/*
  COMMAND_FORWARD = 0,
  COMMAND_FORWARD_IR = 1,
  COMMAND_REVERSE = 2,
  COMMAND_REVERSE_IR = 3,
  COMMAND_TURN_LEFT = 4,
  COMMAND_TURN_RIGHT = 5,
  COMMAND_STOP = 6,
  COMMAND_GET_STATS = 7,
  COMMAND_CLEAR_STATS = 8,
  COMMAND_AUTO_MODE = 9,
  COMMAND_REMOTE_MODE = 10,
  COMMAND_GET_HEADING = 11
*/
} TCommandType;
#endif

