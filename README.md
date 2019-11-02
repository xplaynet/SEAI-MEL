# SEAI-MEL

Adaptação de Projecto de controlo de mototores com triac para PWM

Original:
https://create.arduino.cc/projecthub/saulius-bandzevicius/arduino-based-universal-ac-motor-speed-controller-a4ceaf

**Table of Contents**

[TOC]

### Descrição
Este projecto tem como objectivo implementar um programa de controlo de motor com PWM num arduino UNO a correr uma kernel de sistemas operativos de tempo real *FreeRTOS*.

### Funcionalidades

- Geração de onda PWM a 20kHz com duty-cycle variável.
- Task periódica de leitura de inputs que permite ao operador definir a velocidade desejada do motor;
- Conversão de impulsos periódicos proveninentes de um tacómetro, com recurso a interrupções, em RPMs do motor;
- Implementação de um PID em software;
- Task periódica responsável por correr o PID. Actualiza os seus inputs, requesita um novo cálculo do output e actualiza o duty-cycle da onda PWM com o mesmo;

### Porquê RTOS?
Apesar de estas funcionalidade poderem ser facilmente implementadas num programa simples, cíclico, sem recurso a uma kernel, optou-se por usar a kernel *FreeRTOS* .

Esta kernel foi escolhida essencialmente por duas vantagens. Um melhor controlo do tempo de processamento do CPU e, facilidade de integrar novas funcionalidades ou mesmo modificar já existentes.

A primeira vantagem deve-se à possibilidade da prioritizar tasks e definir a frequência com que estas se repetem. Neste projecto, em que o output do PID e consequentemente o duty-cycle da onda PWM, têm de actualizar frequentemente para evitar grandes deslizamentos de RPM do motor, é importante que a task do PID seja capaz de executar periodicamente sem ser bloqueada por outros processos a correr no CPU, com a preempção destes se necessário.

Para a segunda vantagem, visto ser possível separar as diferentes funcionalides pelas suas próprias tasks, o desenvolvimento de novas tasks pode ser feito independente do resto do código. Apenas é necessário ter alguns cuidados com os recursos partilhados entre tasks.

### Lista de materias

#### Arduino UNO
Foi escolhido como o microcontrolador a usar no sistema devido à sua versatibilidade, disponibilade de documentação online e experiência com o mesmo em projectos anteriores.

#### PHOTOTRANSISTOR OPTOCOUPLER 6N136
Este elemento foi usado pois necessitamos de separar electricamente o arduino do resto do circuito de drive ao motor de alta tensão.
O 6N136 foi escolhido em especifico pelo seu baixo preço, disponibilidade nos laboratórios da feup e por permitir uma resolução de 2% duty-cycle para ondas PWM 20Khz.
Mais barato que o circuito integrado escolhido, e disponivel nos laboratórios, só o 4N33, mas devido aos seus elevados temps de transição de estado não é possível usá-lo neste projecto.
 

### Esquemática

![](https://github.com/FranciscoGilCC/SEAI-MEL/blob/master/images/Schematic.png)