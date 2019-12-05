#include "serial_header.h"






void readButtonSM(unsigned short *state, const byte reading, unsigned int *timer);

void setup() {
  // put your setup code here, to run once:

  //Setup PINS de input para os botões //Alexandre

  //Setup Serial. 9600 Baud, parity Even, 8 bits, 2 stop bits //Alexandre
  //Tenho código de outros trabalhos para fazer setup de comunicação série e mesmo envio/leitura também se quiseres.

  //Setup display //Pedro Coelho

}

//  unsigned short StateUp = 0;
//  unsigned short StateDown = 0;
//  unsigned short StateProgB = 0;
//
//
//  unsigned int timerUp = 0;
//  unsigned int timerDown = 0;
//  unsigned int timerProgB = 0;
//
//  byte buttons;
//
// short tempPress;

void loop() {
  // put your main code here, to run repeatedly:


  //Ler pins de input e aplicar máquina de estados, com exemplo
//  buttons = PINC; //Ler banco de PINs C para byte button, também podes usar tipo digitalRead(PinNumber) para cada botão se preferires
//  tempPress = 0;// decrementar e/ou incrementar tempPress dependendo dos botões. 
//  readButtonSM(&StateDown, buttons & DOWN, &timerDown);
//  if (StateDown & 0x01)  tempPress -= 1; //a MASK 0x01 detecta estados NEWPRESS e PRESS

  
  //Enviar Bytes descritos no serial_header de mudança de velocidade, dependendo de tempPress

  //Enviar YIELD_CHANNEL

  //Receber e processar respostas até receber YIELD_CHANNEL 
  //A resposta por agora vai ser só actualizar a variável para o display




  //Actualizar display //Pedro Coelho
 
  

}



//estado NEWPRESS e PRESS indicam actualização 
void readButtonSM(unsigned short *state, const byte reading, unsigned int *timer) {

  unsigned int ltimer;

  
  ltimer = millis();

  //Return if debouncing period is not over
  if(ltimer - *timer < DEBOUNCING_DELAY) return;

  //State machine to process input
  switch (*state) {
    case START: if (!reading) *state = NEWPRESS;
      break;

    case NEWPRESS: if (reading) *state = UNPRESSED;
      else {
        *state = HOLD;
        *timer = ltimer;
      }
      break;

    case HOLD: if (reading) *state = UNPRESSED;
      else if (ltimer - *timer > LDELAY0) {
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
