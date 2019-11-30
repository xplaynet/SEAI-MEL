#ifndef HEADER_CONFIG
#define HEADER_CONFIG


/*
 * Here you will find anything that you might want to change without touching the code like:
 * PIN Locations
 * PWM constraints
 * PID values
 * Change between different different running modes
 */


/*
 * PINS
 */

//Buttons
//PINC
#define UP              B00000001       //Pin 14
#define DOWN            B00000100       //Pin 16
#define PROGRAM_BUTTON  B00010000       //Pin 18
//#define UP                  14
//#define DOWN                16
//#define PROGRAM_BUTTON      18

//POWER CIRCUIT
#define PWM                   10 //OC1B, Do not change
#define TAC                   3  //Do not change
//todo current detection pins


/*
 * Button configurations
 */

#define LDELAY  500   //Time, in milliseconds, between speed change triggers when longpressing a button
#define RPMSTEP 100   //Increment or decrement desiredRPM by this amount for each trigger

/*
 * Timeouts to detect tachometer faillure, in FreeRTOS tickerates. Needs to always be >15 milliseconds
 */

#define NORMAL_FAIL_READS     500  / portTICK_PERIOD_MS       //for 500 milliseconds 
#define SLOWSTART_FAIL_READS  2000 / portTICK_PERIOD_MS       //SLOWSTART_FAIL_READS must always be bigger than or equal to NORMAL_FAIL_READS

/*
 * PID configurations
 */

#define MIN_DUTY_CYCLE    2             //Min duty cycle in percentage. Must be >2
#define MAX_DUTY_CYCLE    100           //Max duty cycle
#define MIN_RPM           600           //Must be >600
#define MAX_RPM           5000          

//xunning
#define PID_KP            0.01
#define PID_KI            0.20
#define PID_KD            0

/*
 * PWM configurations
 */
#define PWM_FREQ 20                 //in Khz  


        

















#endif