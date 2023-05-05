/*    |*    |*
*    Authors: Caio
*    Readme:
*        Nova versão excluiu a biblioteca MCPWM(escala de 0-100 duty) e adicionou a biblioteca LEDC(escala 10bits).
*        Com a LEDC poderemos trocar mais vezes o indice do vetor (sem sampler hold) porque para mais vezes, o programa leva mais tempo e aí é possivel atrasar = abaixar a freq final mais ainda.        
*        A nova Tabela foi gerada de 0 a 1000 no .py
*    Fontes: https://portal.vidadesilicio.com.br/controle-de-potencia-via-pwm-esp32/
*    
*    *|
*/
#include <stdio.h>
#include <math.h>
#include "esp_system.h"
#include "esp_attr.h"
#include "Arduino.h"
#include "driver/mcpwm.h"
#include "driver/adc.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/dac.h"
#include "Wire.h"

/*    |*    |*
*    GPIOS
*    Resolução do pwm 10 bits = 2^10 = 1024
*
*    *|
*/
#define GPIO_DAC0 25
#define GPIO1 19  
#define RES_1024 10
#define LEN 1000
hw_timer_t * timer = NULL;

int freq_switch = 20000;
int indexTB = 0;
int waveFormTB[LEN] = { 500 , 531 , 562 , 593 , 624 , 654 , 684 , 712 , 740 , 767 , 793 , 818 , 842 , 864
, 885 , 904 , 922 , 938 , 952 , 964 , 975 , 984 , 991 , 996 , 999, 1000,999,996
, 991 , 984 , 975 , 964 , 952 , 938 , 922, 904 , 885,864,842,818,793,767
,740 , 712 , 684 , 654 , 624 , 593 , 562 , 531 , 500 , 469 , 438 , 407 , 376, 346
,316 , 288 , 260 , 233 , 207 ,182, 158 , 136 , 115 , 96, 78, 62, 48, 36
, 25, 16 , 9 , 4 , 1 , 0 , 1 ,4 , 9, 16, 25, 36, 48, 62
, 78, 96, 115, 136, 158, 182, 207, 233, 260, 288, 316, 346, 376, 407
, 438, 469};
bool pass = false;

/*TIMER*/                                 
void readINA10k (){
  pass= true;
}

void setup()
{
  Serial.begin(9600);
  pinMode(GPIO1, OUTPUT);

  /*TIMER*/                               
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &readINA10k, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);

  /*LEDC*/
  ledcAttachPin(GPIO1, 0);
  ledcSetup(0, freq_switch, RES_1024); 
}

void loop()
{ /*100kHz loop*/

  if (pass){
    ledcWrite(0 , waveFormTB[indexTB]);
    indexTB++;
    if (indexTB == 100)
      indexTB = 0;
    pass = false;
  }
}