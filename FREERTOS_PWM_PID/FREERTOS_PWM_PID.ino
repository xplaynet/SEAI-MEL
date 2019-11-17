#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//#include <Rotary.h>
#include <PID_v1.h>
#include <EEPROM.h>

#define DEV 1

#define DACPWM 11
#define PWM 10 //OC1B
//#define UP 14
//#define DOWN 17
#define TAC 3
//#define PROGRAM_BUTTON 19

#define UP B00000001
#define DOWN B00000100
#define PROGRAM_BUTTON B00010000

#define TASKNUMBER 3
#define PROGSIZE 10

//in miliseconds
#define DDELAY 50    //reading delay to avoid jumping between button states
#define LDELAY 500   //Long press delays

#define RPMSTEP 100

//For automated control and writing/reading EPPROM
#define STARTADDRESS 2
#define SIGNATURE 0xFACC
#define AUTOTIMEOUT 30000 / portTICK_PERIOD_MS  // This should give us 30 second timer
#define RPM_DELAY_AUTO 1000                     //Delay to try and wait for system stabilization before checking cycle
#define CYCLE_DELAY_AUTO 1000
#define LONGPRESS 3                             //number of cycles until its considered long press

//Button states
#define START     0
#define NEWPRESS  1
#define HOLD      2
#define PRESS     3
#define UNPRESSED 4

//Program record states
#define NEWSTEP     0
#define HOLDSTEP    1
#define CORRECTSTEP 2
#define ENDSTEP     3

//AVOID BIG MISTAKES STUFF
#define MAXFAILREADS 10


//Shared variables
volatile unsigned int RPM = 0;                 // real rpm variable
volatile unsigned int desiredRPM = 0;        //RPM selected by operator

volatile bool loopflag = false;              // flag for soft start
volatile bool runflag = false;               // flag for motor running state

SemaphoreHandle_t semDAC = NULL;

unsigned int progRecord[PROGSIZE * 2];

//Tachometer things, measures wave cycle.
volatile unsigned long period = 0;
volatile unsigned int count;                 // tacho pulses count variable

unsigned long lastcounttime = 0;
unsigned long lastflash;
unsigned long lastpiddelay = 0;


const int sampleRate = 1;           // Variable that determines how fast our PID loop | Task delays already take care of that so we can set this no minimum
const int debounceDelay = 50;       // the debounce time; increase if the output flickers

//PWM duty-cycle. 400 -> 100%
//These variable are also shared but READ-ONLY so there is no concurrency problems
const int minoutputlimit = 5;      // limit of PID output
const int maxoutputlimit = 400;     // limit of PID output
const int minrpm = 300;             // min RPM
const int maxrpm = 5000;            // max RPM
const int runningrpm = 1000;


//const byte PROGRAM_BUTTON = B00100000;
//const byte DOWN = B00010000;
//const byte UP = B00000001;

//define function to count RPM on interrupt
void tacho();

//define tasks
void TaskButtonReader(void *pvParameters);
void TaskUpdatePID(void *pvParameters);
void TaskSerialPrinter(void *pvParameters);
void TaskCreateProgram(void *pvParameters);
void TaskRunProgram(void *pvParameters);

//Handles for intertask things.
TaskHandle_t xHandle = NULL;
TaskHandle_t Handle_Create = NULL;
TaskHandle_t Handle_Run = NULL;
TaskHandle_t Handle_PID = NULL;
TaskHandle_t Handle_Button = NULL;


volatile bool CreateF = false;
volatile bool RunF = false;
//To use for message passing between ButtonTask and create program Task
struct progSlice
{
  unsigned int desiredRPM;
  unsigned int cycle;
};

QueueHandle_t progQueue;


//define functions
void readButtonSM(unsigned short int *state, const byte reading, unsigned long *timer);

// the setup function runs once when you press reset or power the board
void setup() {

  DDRC = 0;
  PORTC = 0 ^ (UP|DOWN|PROGRAM_BUTTON); 
//  pinMode(UP, INPUT);               // Increase RPM
//  pinMode(DOWN, INPUT);             // Decrease RPM
//  pinMode(PROGRAM_BUTTON, INPUT);   // It's in the name
  pinMode(TAC, INPUT);
//  digitalWrite(UP, HIGH);           // turn on pullup resistor
//  digitalWrite(DOWN, HIGH);         // turn on pullup resistor
//  digitalWrite(PROGRAM_BUTTON, HIGH);         // turn on pullup resistor
  digitalWrite(TAC, HIGH);

  // set up Timer1 Phase and Frequency Correct PWM Mode
  pinMode(PWM, OUTPUT);                            //PWM PIN for OC1B
  //reset registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  //Set bits for Phase and Frequency Correct 20khz PWM
  TCCR1A = _BV(COM1B1) | _BV(WGM10);                //Set compare mode and bit 0 for mode selection
  TCCR1B = _BV(CS10) | _BV(WGM13);                  //Prescaler 1, bit 3 for mode selection
  OCR1A = maxoutputlimit;                           //FreqPWM = (16 000 000) / (2 * 1 * 400) = 20 000
  OCR1B = 0;                                        //Register to compare for cycle

  //Set up Timer2 for DAC with fast PWM
  pinMode(DACPWM, OUTPUT);
  //reset registers
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  //freqPWM = ??? it's lower than 62 500. I just changed CS bits to lower the frequency but didn't bother checking the prescaler because it doesn't matter.
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20) | _BV(CS21) | _BV(CS22) ;
  OCR2A = 123;


  // set up tacho sensor interrupt IRQ1 on pin3
  attachInterrupt(digitalPinToInterrupt(TAC), tacho, FALLING);

  // initialize serial communication at x bits per second:
  Serial.begin(57600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

#ifdef DEV
  EEPROMWrite16(0, 0);
#endif

  progQueue = xQueueCreate(15, sizeof(struct progSlice));
  semDAC = xSemaphoreCreateMutex();

  xTaskCreate(
    TaskButtonReader
    ,  (const portCHAR *)"BReader"   // A name just for humans
    ,  80  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &Handle_Button );
  Serial.print(1);
      xTaskCreate(
        TaskSerialPrinter
        ,  (const portCHAR *)"SerialP"   // A name just for humans
        ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL );

  xTaskCreate(
    TaskUpdatePID
    ,  (const portCHAR *)"PIDctrl"   // A name just for humans
    ,  98  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &Handle_PID );
  Serial.print(2);
  //  xTaskCreate(
  //    TaskProgramButton
  //    ,  (const portCHAR *)"PButton"   // A name just for humans
  //    ,  100  // This stack size can be checked & adjusted by reading the Stack Highwater
  //    ,  NULL
  //    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  //    ,  NULL );
  Serial.print(3);
  xTaskCreate(
    TaskCreateProgram
    ,  (const portCHAR *)"PMAKE"   // A name just for humans
    ,  110  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &Handle_Create );

  Serial.print(4);
  xTaskCreate(
    TaskRunProgram
    ,  (const portCHAR *)"PRUN"   // A name just for humans
    ,  70  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &Handle_Run );
  Serial.print(5);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

//Helper functions to write and read 2byte(16bit) variables. Each EEPROM cell only stores 1byte
void EEPROMWrite16(unsigned int address, unsigned int value) {

#ifdef DEV
  progRecord[address] = value;
  Serial.print("DEV");
#endif
#ifndef DEV
  address += address;
  EEPROM.write(address, (value >> 8) & 0xFF);
  EEPROM.write(address + 1, value & 0xFF);
#endif
}

unsigned int EEPROMRead16(unsigned int address) {
#ifdef DEV
  Serial.print("DEVR");
  return progRecord[address];

#endif
#ifndef DEV
  unsigned int x;
  address += address;
  x = EEPROM.read(address);
  x = (x << 8);
  x |= EEPROM.read(address + 1) & 0xFF;

  return x;
#endif
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


  byte buttons = 0;
  //  bool lastStateUp = HIGH;
  //  bool lastStateDown = HIGH;
  //  bool lastProgB = HIGH;
  unsigned short StateUp = 0;
  unsigned short StateDown = 0;
  unsigned short StateProgB = 0;


  unsigned int timerUp = 0;
  unsigned int timerDown = 0;
  unsigned int timerProgB = 0;

  unsigned short upCount = 0;
  unsigned short downCount = 0;
  unsigned short progCount = 0;


  long tempRPM;                   //desiredRPM is a shared variable so we need something to store
  //it so we can modify freely locally and at the end modify desiredRPM



  for (;;) {
    //debugDAC(uxTaskPriorityGet(NULL), 1);

    tempRPM = 0;
    buttons = PINC;


    //MANUAL SPEED CONTROL BUTTONS
    //Once a button is beeing pressed the other is ignored
    if (StateUp == START) {
      readButtonSM(&StateDown,buttons & DOWN, &timerDown);
      if (StateDown & 0x01) tempRPM -= RPMSTEP;                   //Update RPM on new button press or periodically when button is held
    }

    //Same thing as the other button but for increasing RPM
    if (StateDown == START) {
      readButtonSM(&StateUp,buttons & UP, &timerUp);
      if (StateUp & 0x01) tempRPM += RPMSTEP;
    }
    if (CreateF) {
      if (StateDown == NEWPRESS || StateUp == NEWPRESS) progTemp.cycle = OCR1B;   //On new press, update progTemp with current duty-cycle
      if (StateUp == UNPRESSED || StateDown == UNPRESSED) {
        progTemp.desiredRPM = desiredRPM;                                       //if the button was just unpressed update progTemp with new RPM
        xQueueSend(progQueue, (void *)&progTemp, (TickType_t) 2);               //Then put in the Queue for another Task to process it
        vTaskResume(Handle_Create);                                             //Wake up task to do it
      }
    }

    if (tempRPM != 0) {
      if (RunF)vTaskSuspend(Handle_Run);
      //Make adjustments to tempRPM to avoid getting out of bounds. the min rpm thing might be kinda useless
      tempRPM += desiredRPM;
      if (tempRPM > maxrpm) tempRPM = maxrpm;
      else if (tempRPM == 100) tempRPM = 300;
      else if (tempRPM == 200) tempRPM = 0;
      else if (tempRPM < 0) tempRPM = 0;



      desiredRPM = tempRPM;
    }
    //AUTOMATED PROGRAM CONTROL BUTTON

    readButtonSM(&StateProgB, buttons & PROGRAM_BUTTON, &timerProgB);
    //oof ok.
    //On short press, run stored program if it exists. If there is already a program running just make sure it's actually running
    //On long press, start Task to create a program, there should be no active create program tasks or run program tasks. The motor speed must be set to 0 to make sure the program starts on idle.
    if (StateProgB == NEWPRESS) progCount = 0;
    else if (StateProgB == PRESS) progCount++;
    else if (StateProgB == UNPRESSED) {
      if (progCount < LONGPRESS) {
        //check if there is available complete program
        if (EEPROMRead16(0) == SIGNATURE && !CreateF) {
          Serial.print("\nRun\n");
          //Delete Create Task in case it exists in waiting
          vTaskResume(Handle_Run);
          if (!RunF) RunF = true;
        }
      } else {
        //Launch Task to create a program on long press. The system should be Idle to start this task
        if (!CreateF && !RunF && desiredRPM == 0) {
          Serial.print("\nCreate\n");
          vTaskResume(Handle_Create);
          CreateF = true;

        }
      }
      count = 0;
    }
 


    //debugDAC(uxTaskPriorityGet(NULL), 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

}

void TaskCreateProgram( void *pvParameters) {

  (void) pvParameters;

  unsigned int address;                     //Keep track of address beeing written
  unsigned int previousRPM;                 //save previousRPM
  unsigned int tempCycle;                   //Keep Cycle when decresing RPM in case the machine is ending its work
  unsigned int i;

  unsigned short state;
  struct progSlice progTemp;
  //Serial.print("\nStarting2 Create\n");


  for (;;) {

    //Wait for Create Program button Press. Ignore this suspend once it started until it sopped recording
    //Reset all local variables
    if (!CreateF) {
      vTaskSuspend(NULL);
      EEPROMWrite16(0, 0);
      state=0;
      previousRPM =0;
      tempCycle =0;
      address=1;
    }

    vTaskSuspend(NULL);
    //debugDAC(uxTaskPriorityGet(NULL), 1);
    if ( xQueueReceive(progQueue, &(progTemp), (TickType_t) 0)) {
      if (previousRPM == progTemp.desiredRPM) continue; //Check there was an actuall change to the RPM
      if (progTemp.desiredRPM == 0) state = ENDSTEP;
      else {
        switch (state) {
          case NEWSTEP: if (progTemp.desiredRPM < previousRPM) state = HOLDSTEP;
            break;

          case 1: if (progTemp.desiredRPM > previousRPM) state = CORRECTSTEP;
            break;

          case 2: if (progTemp.desiredRPM < previousRPM) state = HOLDSTEP;
            else if (progTemp.desiredRPM > previousRPM) state = NEWSTEP;
            break;

          default: state = 4;
        }
      }

      //This should be used to figure out, when debugging, the stack amount that should be made available to tasks

      //Add new step to program
      if (state == NEWSTEP) {
        tempCycle = 0;
        previousRPM = progTemp.desiredRPM;
        //WRITE TO EEPROM
        //WHILE IN TEST DO PRINTS
        //Serial.print("\ndesired RPM: "); Serial.print(previousRPM); Serial.print("\t cycle: "); Serial.print(progTemp.cycle);
        EEPROMWrite16(address, progTemp.desiredRPM);
        EEPROMWrite16(address + 1, progTemp.cycle);
        address += 2;
      }
      //When speed is decreased keep some information.
      else if (state == HOLDSTEP) {
        //Serial.print("\nHELD"); Serial.print(progTemp.cycle);
        if (!tempCycle) tempCycle = progTemp.cycle;
        previousRPM = progTemp.desiredRPM;
      }
      //if after speed is decreased there is an increase, correct previous RPM value in program. It is assumed the operator overshot the desired value.
      else if (state == CORRECTSTEP) {
        //Serial.print("\ndesired RPM: "); Serial.print(previousRPM);
        EEPROMWrite16(address - 2, previousRPM);
        previousRPM = progTemp.desiredRPM;
        EEPROMWrite16(address, progTemp.desiredRPM);
        EEPROMWrite16(address + 1, progTemp.cycle);
        address += 2;

        //Serial.print("\ndesired RPM: "); Serial.print(previousRPM); Serial.print("\t cycle: "); Serial.print(progTemp.cycle);
      }

      //desired RPM has hit 0 so the work is done
      //Finish writing the program to EEPROM and use signture to let other taks know there is a program loaded
      //Program persists after resets
      else if (state == ENDSTEP) {
        EEPROMWrite16(address, progTemp.desiredRPM);
        if (tempCycle) EEPROMWrite16(address + 1, tempCycle);
        else EEPROMWrite16(address + 1, progTemp.cycle);
        //Serial.print("\nFinishing");
        EEPROMWrite16(0, SIGNATURE);
        //Serial.print("\n"); Serial.print(EEPROMRead16(0));
        for (i = 1; i < (PROGSIZE * 2) - 2 ; i += 2) {
          //Serial.print("\nn: "); Serial.print(i - 1); Serial.print("\nsetRPM: "); Serial.print(EEPROMRead16(i)); Serial.print("\t cycle: "); Serial.print(EEPROMRead16(i + 1));
          if (EEPROMRead16(i) == 0 ) {
            EEPROMWrite16(0, SIGNATURE);
            break;
          }
        }
        CreateF = false;
      }
      //else  Serial.print("\nStill Alive");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);





    //debugDAC(uxTaskPriorityGet(NULL), 0);
  }


}



void TaskRunProgram(void *pvParameters) {

  (void) pvParameters;

  bool rpmlock = false;
  bool cyclelock = false;

  unsigned int Pcycle = 0;
  unsigned int Ccycle = 0;
  unsigned int Crpm = 0;
  unsigned int address = 1;

  unsigned long rpmTimer = 0;
  unsigned long cycleTimer = 0;
  unsigned long cycleTimeout = 0;

  unsigned int readCycle = 0;

  for (;;) {


    //If both rpm and cycle conditions are met, Advance program
    if (!rpmlock && !cyclelock) {


      //On Task launch/ Wait for program button to start task. Detect if program ended and wait here again
      if (Crpm == 0) {
        RunF = false;
        vTaskSuspend( NULL );
        address = 1;

      }


      Crpm = EEPROMRead16(address);
      Ccycle = EEPROMRead16(address + 1);
      //Initiate locks for RPM and Cycle check
      rpmlock = true;
      cyclelock = true;

      //Update desiredRPM
      desiredRPM = Crpm;



      //Setup things for detecting the stuff
      rpmTimer = 0;
      cycleTimer = 0;
      Pcycle = maxoutputlimit + 1;
      address += 2;
    }

    //Wait for system to reach target RPM and then wait a bit to try and stabilize
    if (RPM >= Crpm && rpmlock ) {
      if (!rpmTimer) rpmTimer = millis();
      else if (millis() - rpmTimer > RPM_DELAY_AUTO) {
        rpmlock = false;
        Serial.print("\nunlocked rpm\n");
      }
    }


    if (!rpmlock) {
      readCycle = OCR2B;

      //Wait for system to reach target duty-cycle or stay on the same cycle or higher for too long;
      if (readCycle < Pcycle) {
        Pcycle = readCycle;
        cycleTimeout = millis();
      }
      if (readCycle <= Ccycle || (millis() - cycleTimeout) > AUTOTIMEOUT) {
        if (!cycleTimer) cycleTimer = millis();
        else if ( (millis() - cycleTimer) > CYCLE_DELAY_AUTO) {
          Serial.print("\nunl cycle\n");
          cyclelock = false;
        }
      }
    }
    vTaskDelay(6);
  }
}
void TaskUpdatePID(void *pvParameters) {

  (void) pvParameters;

  double Setpoint, Input, Output;       // define PID variables

  const double sKp = 0.25, sKi = 0.4, sKd = 0; // PID tuning parameters for starting motor
  const double rKp = 0.6, rKi = 1, rKd = 0;  // PID tuning parameters for runnig motor


  bool localSlow = false;
  bool localRun = false;

  unsigned int cycle = 0;
  unsigned int localCount = 0;

  unsigned int localPeriod = 0;

  //Setup PID
  PID myPID(&Input, &Output, &Setpoint, sKp, sKi, sKd, DIRECT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(minoutputlimit, maxoutputlimit);
  myPID.SetSampleTime(sampleRate);    // Sets the sample rate

  //Initialize things
  Input = 20;
  Setpoint = 20;
  //unsigned long timeru = 0;
  float time_in_sec = 0;

  unsigned short tachoTimeoutCounter = 0;


  for (;;) {
    //debugDAC(uxTaskPriorityGet(NULL), 1);


    Setpoint = desiredRPM;
    localPeriod = period;

    //Check if there's a new reading on tachometer, filter unreasonable values
    if (count != localCount && localPeriod > 4000) {
      Input = localPeriod;

      Input = ((float)1 / (float)Input) * 1000000;
      Input = 7.5 * Input;
      localCount = count;

      tachoTimeoutCounter = 0;


      //If tachometer isnt reading, if desiredRPM > 0 wait a bit, if not solved initiate motor stop
    } else if (Setpoint != 0) {
      if ((tachoTimeoutCounter < MAXFAILREADS)) tachoTimeoutCounter++;
      else if (tachoTimeoutCounter >= MAXFAILREADS) {
        Input = maxrpm * 2; //Setup warning
        Setpoint = 0;
      }
    } else Input = 0;


    RPM = Input;


    if (Setpoint > 0) {
      if (Input < runningrpm)localSlow = true; //If cold starting set slow start. Cold starting is defined if motor is below minimum RPM
      localRun = true;

    } else localRun = false ;


    if (localRun) {
      //soft start
      if (localSlow == true) {
        myPID.SetTunings(sKp, sKi, sKd);        // Set the PID gain constants and start
        if (Input > runningrpm) {
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
      myPID.Compute();
      //dimming = map(Output, minoutputlimit, maxoutputlimit, maxoutputlimit, minoutputlimit); // reverse the output
      cycle = constrain(Output, minoutputlimit, maxoutputlimit);     // check that dimming is in range
    }
    else cycle = 0;
    //Might need some work done here

    OCR1B = cycle;
    //debugDAC(uxTaskPriorityGet(NULL), 0);
    //Serial.print(RPM);Serial.print("\n");
    vTaskDelay(1);
  }

}


void TaskSerialPrinter(void *pvParameters) {

  (void) pvParameters;

  struct progSlice progTemp;
  unsigned long timer;
  UBaseType_t uxHighWaterMark;
  
  for (;;) {
    //vTaskSuspend(NULL);
    //debugDAC(uxTaskPriorityGet(NULL), 1);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( Handle_Create );
    Serial.print("\nWaterMark: ");
    Serial.print(uxHighWaterMark);
    Serial.print("\ndesiredRPM: ");
    Serial.print(desiredRPM);
    Serial.print("\nReadRPM: ");
    Serial.print(RPM);
    Serial.print("\nCycle: ");
    Serial.print(OCR1B);
    Serial.print("\n");
    //    if ( xQueueReceive(progQueue, &(progTemp), (TickType_t) 0)) {
    //      Serial.print(progTemp.desiredRPM);
    //      Serial.print(" | ");
    //      Serial.print(progTemp.cycle);
    //      Serial.print("\n");
    //    }
  
    //debugDAC(uxTaskPriorityGet(NULL), 0);
    vTaskDelay( 2000 / portTICK_PERIOD_MS );
  }
}


//My very simple try at having a task tracer. to use with 22uF capacitor, 3.3kOhm resistor op amp low pass filter, pin 11, osciloscope
//void debugDAC(int i, int state) {
//
//  static unsigned int values[TASKNUMBER];
//
//  if (semDAC != NULL) {
//    if (xSemaphoreTake(semDAC, (TickType_t) 2) == pdTRUE) {
//
//      //update TASK state in array
//      values[i] = state;
//
//      //Update PWM output
//      for (i = 0; i < TASKNUMBER ; i++) {
//        if (values[i] != 0) {
//          OCR2A = (256 / TASKNUMBER) * (i + 1);
//          break;
//        }
//      }
//      if (i == TASKNUMBER) OCR2A = 0;
//      xSemaphoreGive(semDAC);
//    }
//  }
//
//
//}

//State machine used for reading simple push buttons. state is used as function output.
void readButtonSM(unsigned short *state, const byte reading, unsigned int *timer) {

  unsigned int ltimer;



  ltimer = millis();


  //State machine to process input
  switch (*state) {
    case START: if (!reading) *state = NEWPRESS;
      break;

    case NEWPRESS: if (reading) *state = UNPRESSED;
      else{
        *state = HOLD;
        *timer = ltimer;
      }
      break;

    case HOLD: if (reading) *state = UNPRESSED;
      else if (ltimer - *timer > LDELAY) {
        *state = PRESS;
        *timer = ltimer;
      }
      break;

    case PRESS: if (reading)  *state = UNPRESSED;
      else  *state = HOLD;
      break;

    case UNPRESSED: if (!reading) *state = NEWPRESS;
      else *state = START;
      break;

    default:
      *state = START;
      break;
  }

}

unsigned long lastInt = 0;
void tacho() {
  count++;
  period = micros() - lastInt;
  lastInt = micros();
  //  unsigned long timer = micros() - lastflash;
  //  float time_in_sec  = ((float)timer) / 1000000;
  //  float prerpm = 60 / time_in_sec;
  //  RPM = prerpm;
  //  lastflash = timer;
}
