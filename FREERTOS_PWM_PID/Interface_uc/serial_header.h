#ifndef SERIAL_CONFIG
#define SERIAL_CONFIG


/*
 *	Bytes used for communication
 *  Everything that is not alphanumeric bytes for display should be defined here
 */

//Byte used to yield channel since we need to communicate in both ways
#define YIELD_CHANNEL 	    0x01

//Button replacements for serial
#define SERIAL_UP0      		    0x02
#define SERIAL_DOWN0     	      0x03
#define SERIAL_PROGRAM_START    0x04
#define SERIAL_PROGRAM_RECORD   0x05
#define SERIAL_STOP             0x06

#define UP              B00000001       //Pin 14
#define DOWN            B00000100       //Pin 16
#define PROGRAM_BUTTON  B00010000       //Pin 18

//Caso se use digitalRead()
//#define UP                  14
//#define DOWN                16
//#define PROGRAM_BUTTON      18




//Button states
#define START     0
#define NEWPRESS  1
#define HOLD      2
#define PRESS     3
#define UNPRESSED 4

#define LDELAY0 500                 //Delay to repeat command on longPress
#define DEBOUNCING_DELAY 50

#endif
