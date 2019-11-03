#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//#include <Rotary.h>
#include <PID_v1.h>
#include <EEPROM.h>

#define DACPWM 11
#define PWM 10 //OC1B
#define UP 14
#define DOWN 17
#define TAC 3

#define TASKNUMBER 3
#define PROGSIZE 30

//in miliseconds
#define DDELAY 50    //reading delay to avoid jumping between button states
#define LDELAY 500   //Long press delays

#define RPMSTEP 100

//For automated control writing to EPPROM
#define STARTADDRESS 2
#define SIGNATURE 0xFACC

//Shared variables
volatile unsigned int RPM;                   // real rpm variable
volatile unsigned int desiredRPM = 0;        //RPM selected by operator

volatile bool loopflag = false;              // flag for soft start
volatile bool runflag = false;               // flag for motor running state

SemaphoreHandle_t semDAC = NULL;

unsigned int progRecord[PROGSIZE][2];
volatile unsigned int progCursor = 0;


unsigned int count;                 // tacho pulses count variable
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
//These variable are also shared but READ-ONLY so there is no concurrency problems
const int minoutputlimit = 50;      // limit of PID output
const int maxoutputlimit = 400;     // limit of PID output
const int minrpm = 300;           // min RPM of the range 1
const int maxrpm = 5000;          // max RPM of the range 1

//define function to count RPM on interrupt
void tacho();

//define tasks
void TaskButtonReader(void *pvParameters);
void TaskUpdatePID(void *pvParameters);
void TaskSerialPrinter(void *pvParameters);

TaskHandle_t xHandle = NULL;

//To use for message passing between ButtonTask and create program Task
struct progSlice
{
  unsigned int desiredRPM;
  unsigned int cycle;
};

QueueHandle_t progQueue;


//define functions
unsigned short readButtonSM(unsigned short int *state, int button, unsigned long *timer, byte *lastRead);

// the setup function runs once when you press reset or power the board
void setup() {

  pinMode(UP, INPUT);               // Increase RPM
  pinMode(DOWN, INPUT);             // Decrease RPM
  digitalWrite(UP, HIGH);           // turn on pullup resistors
  digitalWrite(DOWN, HIGH);         // turn on pullup resistors


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

  //Set up Timer2 for DAC with fast PWM
  pinMode(DACPWM, OUTPUT);
  //reset registers
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  //freqPWM = 62 500
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  OCR2A = 0;


  // set up tacho sensor interrupt IRQ1 on pin3
  attachInterrupt(digitalPinToInterrupt(TAC), tacho, FALLING);

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  progQueue = xQueueCreate(15, sizeof(struct progSlice));
  semDAC = xSemaphoreCreateMutex();

  xTaskCreate(
    TaskButtonReader
    ,  (const portCHAR *)"BReader"   // A name just for humans
    ,  164  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &xHandle );

  xTaskCreate(
    TaskSerialPrinter
    ,  (const portCHAR *)"SerialPrinter"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskUpdatePID
    ,  (const portCHAR *)"PIDcontrol"   // A name just for humans
    ,  256  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

void EEPROMWrite16(unsigned int address, unsigned int value) {
  EEPROM.write(address, (value >> 8) & 0xFF);
  EEPROM.write(address + 1, value & 0xFF);
}

unsigned int EEPROMRead16(unsigned int address) {
  unsigned int x;
  x = EEPROM.read(address);
  x = (x << 8);
  x |= EEPROM.read(address + 1) & 0xFF;

  return x;
}


unsigned short readButtonSM(unsigned short *state, int button, unsigned long *timer, byte *lastRead) {

  unsigned long ltimer;
  byte reading;


  ltimer = millis();

  //Filter bouncing erros to avoid jumping erroneously and quickly through the state machine
  if ((ltimer - *timer <= DDELAY)) {
    reading = *lastRead;
  } else {
    reading = digitalRead(button);
    if (reading != *lastRead) {
      *timer = ltimer;
      *lastRead = reading;
    }
  }

  //State machine to process input
  switch (*state) {
    case 0: if (!reading) {
        *state = 1;
      }
      break;
    case 1: if (reading) {
        *state = 0;
      } else {
        *state = 2;
      }
      break;
    case 2: if (reading) *state = 0;
      else if (ltimer - *timer > LDELAY) {
        *state = 1;
        *timer = ltimer;
      }
      break;
    default:
      *state = 0;
      break;
  }
  return *state;

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

    It includes code to deal with bouncing inputs and deals with long presses.
  */

  struct progSlice progTemp;

  byte lastStateUp = HIGH;
  byte lastStateDown = HIGH;
  unsigned short StateUp = 0;
  unsigned short StateDown = 0;
  unsigned short tempState = 0;

  unsigned long timerUp = 0;
  unsigned long timerDown = 0;

  int reading;
  long tempRPM;                   //desiredRPM is a shared variable so we need something to store
  //it so we can modify freely locally and at the end modify desiredRPM

  for (;;) {
    debugDAC(uxTaskPriorityGet(NULL), 1);

    tempRPM = 0;


  
    //Once a button is beeing pressed the other is ignored
    if (StateUp == 0) {
      tempState = StateDown;
      reading =  readButtonSM(&StateDown, DOWN, &timerDown, &lastStateDown);
      if (reading != 0) {
        if (reading == 1) {
          tempRPM -= RPMSTEP;                         //on state/reading == 1 update desiredRPM
          if (tempState == 0) progTemp.cycle = OCR1B; //if this is a new button press prepare progTemp to send
        }
      } else if (tempState != 0) {
        progTemp.desiredRPM = desiredRPM;                 //if the button was just unpressed update progTemp
        xQueueSend(progQueue, (void *)&progTemp, (TickType_t) 2);  //Then put in the Queue for another Task to process it
      }

    }

    if(StateDown == 0){
    tempState = StateUp;
    reading =  readButtonSM(&StateUp, UP, &timerUp, &lastStateUp);
    if (reading != 0) {
        if (reading == 1) {
          tempRPM += RPMSTEP;                         //on state/reading == 1 update desiredRPM
          if (tempState == 0) progTemp.cycle = OCR1B; //if this is a new button press prepare progTemp to send
        }
      } else if (tempState != 0) {
        progTemp.desiredRPM = desiredRPM;                 //if the button was just unpressed update progTemp
        xQueueSend(progQueue, (void *)&progTemp, (TickType_t) 2);  //Then put in the Queue for another Task to process it
      }

    }


    //Make adjustments to tempRPM to avoid getting out of bounds. the min rpm thing might be kinda useless
    tempRPM += desiredRPM;
    if (tempRPM > maxrpm) tempRPM = maxrpm;
    else if (tempRPM == 100) tempRPM = 300;
    else if (tempRPM == 200) tempRPM = 0;
    else if (tempRPM < 0) tempRPM = 0;

    //Set flags for PID, maybe separate task when automated control is added
    if (tempRPM > 0) {
      if (!runflag) {
        if (RPM < minrpm)loopflag = true; //If cold starting set slow start. Cold starting is defined if motor is below minimum RPM
        runflag = true;
      }
    } else if (!tempRPM) runflag = 0 ;

    desiredRPM = tempRPM;
    debugDAC(uxTaskPriorityGet(NULL), 0);
    vTaskDelay(4);
  }

}
void TaskUpdatePID(void *pvParameters) {

  (void) pvParameters;

  double Setpoint, Input, Output;       // define PID variables
  double sKp = 0.1, sKi = 0.2, sKd = 0; // PID tuning parameters for starting motor
  double rKp = 0.25, rKi = 1, rKd = 0;  // PID tuning parameters for runnig motor

  bool localSlow, localRun;

  unsigned int cycle = 0;

  //Setup PID
  PID myPID(&Input, &Output, &Setpoint, sKp, sKi, sKd, DIRECT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(minoutputlimit, maxoutputlimit);
  myPID.SetSampleTime(sampleRate);    // Sets the sample rate

  //Initialize things
  Input = 20;
  Setpoint = 20;

  for (;;) {
    debugDAC(uxTaskPriorityGet(NULL), 1);
    localSlow = loopflag;
    localRun = runflag;

    if (localRun) {
      //soft start
      if (localSlow == true) {
        myPID.SetTunings(sKp, sKi, sKd);        // Set the PID gain constants and start
        if (RPM > minrpm) {
          lastcounttime = millis();
          lastpiddelay = millis();
          localRun = true;
          localSlow = false;
        }
      }

      // normal motor running state
      else if (localSlow == false) {
        unsigned long piddelay = millis();
        if ((piddelay - lastpiddelay) > 1000) {     // delay to switch PID values. Prevents hard start
          myPID.SetTunings(rKp, rKi, rKd);          // Set the PID gain constants and start
          lastpiddelay = millis();
        }
      }
      Input = RPM;
      Setpoint = desiredRPM;
      myPID.Compute();
      //dimming = map(Output, minoutputlimit, maxoutputlimit, maxoutputlimit, minoutputlimit); // reverse the output
      cycle = constrain(Output, minoutputlimit, maxoutputlimit);     // check that dimming is in 20-625 range
    }
    else cycle = 0;

    OCR1B = cycle;
    debugDAC(uxTaskPriorityGet(NULL), 0);
    vTaskDelay(2);
  }



}
void TaskSerialPrinter(void *pvParameters) {

  (void) pvParameters;

  struct progSlice progTemp;
  unsigned long timer;
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle );
  for (;;) {
    debugDAC(uxTaskPriorityGet(NULL), 1);
    timer = millis();
    Serial.print(timer);
    Serial.print("\ndesiredRPM:");
    Serial.print(desiredRPM);
    Serial.print("\nCycle:");
    Serial.print(OCR1B);
    Serial.print("\n");
    if( xQueueReceive(progQueue, &(progTemp), (TickType_t) 0)){
      Serial.print(progTemp.desiredRPM);
      Serial.print("|");
      Serial.print(progTemp.cycle);
      Serial.print("\n");
    }
    
    debugDAC(uxTaskPriorityGet(NULL), 0);
    vTaskDelay( 2000 / portTICK_PERIOD_MS );
  }

}

//My very simple try at having a task tracer. to use with 22uF capacitor, 3.3kOhm resistor, pin 11, osciloscope
void debugDAC(int i, int state) {

  static unsigned int values[TASKNUMBER];

  if (semDAC != NULL) {
    if (xSemaphoreTake(semDAC, (TickType_t) 2) == pdTRUE) {

      //update TASK state in array
      values[i] = state;

      //Update PWM output
      for (i = 0; i < TASKNUMBER ; i++) {
        if (values[i] != 0) {
          OCR2A = (256 / TASKNUMBER) * (i + 1);
          break;
        }
      }
      if (i == TASKNUMBER) OCR2A = 0;
      xSemaphoreGive(semDAC);
    }
  }


}

void tacho() {
  count++;
  unsigned long timer = micros() - lastflash;
  float time_in_sec  = ((float)timer) / 1000000;
  float prerpm = 60 / time_in_sec;
  RPM = prerpm;
  lastflash = timer;
}
