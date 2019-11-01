#include <Arduino_FreeRTOS.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//#include <Rotary.h>
#include <PID_v1.h>

#define PWM 10
#define UP 5
#define DOWN 4

unsigned int RPM;                   // real rpm variable
unsigned int count;                 // tacho pulses count variable
unsigned int cycle = 0;
unsigned int lastcount = 0;         // additional tacho pulses count variable
unsigned long lastcounttime = 0;
unsigned long lastflash;
unsigned long lastpiddelay = 0;
unsigned long previousMillis = 0;
unsigned long lastDebounceTime = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long timeout1 = 0;
unsigned long timeout2 = 0;

const int sampleRate = 1;           // Variable that determines how fast our PID loop
const int debounceDelay = 50;       // the debounce time; increase if the output flickers

//PWM duty-cycle. 400 -> 100%
const int minoutputlimit = 50;      // limit of PID output
const int maxoutputlimit = 400;     // limit of PID output
const int minrpm = 300;           // min RPM of the range 1
const int maxrpm = 5000;          // max RPM of the range 1


volatile unsigned int desiredRPM = 0;               //RPM selected by operator

byte lastButtonState = HIGH;
byte lastButtonState2 = HIGH;  // the previous reading from the input pin
byte downstate = HIGH;
byte upstate = HIGH;

bool loopflag = false;              // flag for soft start
bool startflag = false;             // flag for motor start delay
bool runflag = false;               // flag for motor running state

double Setpoint, Input, Output;       // define PID variables
double sKp = 0.1, sKi = 0.2, sKd = 0; // PID tuning parameters for starting motor
double rKp = 0.25, rKi = 1, rKd = 0;  // PID tuning parameters for runnig motor

//LiquidCrystal_I2C lcd(0x26, 16, 2);   // set the LCD address to 0x26 for a 16 chars and 2 line display
//Rotary r = Rotary(12, 11);            // define rottary encoder and pins
PID myPID(&Input, &Output, &Setpoint, sKp, sKi, sKd, DIRECT); // define PID variables and parameters


//define function to count RPM on interrupt
void tacho();

void TaskButtonReader(void *pvParameters);
void updatePID();
void TaskSerialPrinter(void *pvParameters);


// define two tasks for Blink & AnalogRead
//void TaskBlink( void *pvParameters );
//void TaskAnalogRead( void *pvParameters );


// the setup function runs once when you press reset or power the board
void setup() {

  pinMode(UP, INPUT);               // Increase RPM
  pinMode(DOWN, INPUT);             // Decrease RPM
  digitalWrite(UP, HIGH);           // turn on pullup resistors
  digitalWrite(DOWN, HIGH);         // turn on pullup resistors

  Input = 20;                        // assign initial value for PID
  Setpoint = 20;                     // assign initial value for PID

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(minoutputlimit, maxoutputlimit);
  myPID.SetSampleTime(sampleRate);    // Sets the sample rate

  // set up Timer1 Phase and Frequency Correct PWM Mode
  pinMode(PWM, OUTPUT);                            //PWM PIN for OC1B
  //reset registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  //Set bits for Phase and Frequency Correct 20khz PWM
  TCCR1A = _BV(COM1B1) | _BV(WGM10);                //Set compare mode and bit 0 for mode selection
  TCCR1B = _BV(CS10) | _BV(WGM13);                  //Prescaler 1, bit 3 for mode selection
  OCR1A = 400;                                      //FreqPWM = (16 000 000) / (2 * 1 * 400) = 20 000
  OCR1B = 0;                                        //Register to compare for cycle

  // set up tacho sensor interrupt IRQ1 on pin3
  attachInterrupt(1, tacho, FALLING);

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  xTaskCreate(
    TaskButtonReader
    ,  (const portCHAR *)"BReader"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskSerialPrinter
    ,  (const portCHAR *)"BReader"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskButtonReader(void *pvParameters)
{
  (void) pvParameters;


  /*
    Button Reader
    Reads button state from assigned ports

    It includes code to deal with bouncing inputs and avoids problems with long pressing buttons.
  */

  byte lastStateUp = HIGH;
  byte lastStateDown = HIGH;
  byte StateUp = HIGH;
  byte StateDown = HIGH;

  unsigned long timeoutUp = 0;
  unsigned long timeoutDown = 0;
  unsigned long debounceUp = 0;
  unsigned long debounceDown = 0;

  int reading;
  unsigned long tempRPM;
  unsigned long timer;

  for (;;) {
    tempRPM = desiredRPM;
    timer = millis();

    reading = digitalRead(UP); // read the state of the switch into a local variable:
    if (reading != lastStateUp) {  // If the switch changed, due to noise or pressing
      debounceUp = timer;     // reset the debouncing timer
    }
    else if ((timer - debounceUp) > debounceDelay) {
      if (timer - timeoutUp > 500) {
        if (reading == LOW)tempRPM += 100;
        timeoutUp = timer;
        StateUp = reading;
      }
    }
    lastStateUp = reading;


    reading = digitalRead(DOWN); // read the state of the switch into a local variable:
    if (reading != lastStateDown) {  // If the switch changed, due to noise or pressing
      debounceDown = timer;     // reset the debouncing timer
    }
    else if ((timer - debounceDown) > debounceDelay) {
      if (timer - timeoutDown > 500) {
        if (reading == LOW && tempRPM >= 100)tempRPM -= 100;
        timeoutDown = timer;
        StateDown = reading;
      }
    }
    lastStateDown = reading;
    if (tempRPM > maxrpm) tempRPM = maxrpm;
    else if (tempRPM < minrpm) tempRPM = 0;

    desiredRPM = tempRPM;
    vTaskDelay(3);
  }

}
//void updatePID();
void TaskSerialPrinter(void *pvParameters) {

  (void) pvParameters;

  unsigned long timer;
  for (;;) {
    timer = millis();
    Serial.print(timer);
    Serial.print("\ndesiredRPM:");
    Serial.print(desiredRPM);
    Serial.print("\n");
    vTaskDelay( 20 );
  }

}


void tacho() {
  count++;
  unsigned long time = micros() - lastflash;
  float time_in_sec  = ((float)time) / 1000000;
  float prerpm = 60 / time_in_sec;
  RPM = prerpm;
  lastflash = micros();
}
