#ifndef SERIAL_CONFIG
#define SERIAL_CONFIG


/*
 *	Bytes used for communication
 *  Everything that is not alphanumeric bytes for display should be defined here
 */

//Byte used to yield channel since we need to communicate in both ways
#define YIELD_CHANNEL 	0x01

//Button replacements for serial
#define SERIAL_UP_HIGH 		0x02
#define SERIAL_UP_LOW	 	0x03
#define SERIAL_DOWN_HIGH	0x04
#define SERIAL_DOWN_LOW		0x05
#define SERIAL_PROGRAM_HIGH 0x06
#define SERIAL_PROGRAM_LOW 	0x07


#endif