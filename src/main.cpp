// #include <Arduino.h>
// #include "Wire.h"
// #include <Arduino.h>
// #include <stdio.h>
// #include "esp_system.h"
// #include "esp_attr.h"
// #include "driver/mcpwm.h"
// #include "soc/mcpwm_reg.h"
// #include "soc/mcpwm_struct.h"
// #include "driver/adc.h"
// #include "driver/dac.h"
// #include "cos_dac.h"

// // Definição do sensor de corrente e tensão.

// #define CANALDAC25 25
// #define GPIO1 12  // pra IN1 Ponte H
// #define GPIO2 16  // pra IN2 Ponte H
// mcpwm_config_t pwm_config;


// void setup(){

//   // Open serial communications and wait for port to open:
//   Serial.begin(115200);

//   //PWM
//   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO1);
//   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO2);
//   pwm_config.frequency = 20000;                                                //frequencia do pwm F1 (1Khz) (acima da audivel )
//   pwm_config.cmpr_a = 95;
//   pwm_config.cmpr_b = 2;
//   pwm_config.counter_mode = MCPWM_UP_COUNTER;
//   pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

//   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
//   mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 10, 10);  //Enable deadtime on PWM0A and PWM0B with red = (656)*100ns & fed = (67)*100ns on PWM0A and PWM0B generated from PWM0A


//   adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
//   adc1_config_width(ADC_WIDTH_BIT_10);

// }

// void loop()
// {
//   // //Serial.print(micros()); para encontrar  a amostragem do esp
//   //   //Serial.print(','); para encontrar  a amostragem do esp
//   //   //measureCurrent();
//   //   int d = (int)(100*adc1_get_raw(ADC1_CHANNEL_6)*0.00097);// mult por 100 do pwm e o 0.00097  é 1 divido por 1023 do adc
//   //   pwm_config.cmpr_a = d;
//   //   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
//   //   //dacWrite(CANALDAC,(int)(255*d*0.01));
//   //   dacWrite(CANALDAC25,(int)(1.7*ina219_0.getCurrent_mA()));
//   //   //dacWrite(CANALDAC25,(int)(127));
//   //   Serial.print(1.7*(int)(ina219_0.getCurrent_mA()));
//   //   //Serial.print(micros()); para encontrar  a amostragem do esp
//   //   Serial.print("\n"); //para encontrar  a amostragem do esp
//   delayMicroseconds(100);                                                 // frequencia de 1 amostra F2 ( 1/100u = 10KHz)
// }

/*    |*    |*
*    Equipe Capivara
*    Authors: Caio
*/
#include <Arduino.h>
#include "Wire.h"
#include <Arduino.h>
#include <stdio.h>
#include "esp_system.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/adc.h"
#include "driver/dac.h"

#define GPIOPWM 16
#define GPIODAC 26
#define LEN_TB 20
mcpwm_config_t pwm_config;
hw_timer_t * timer = NULL;
/*    
 *      WaveForm tabel gerada no .py
 *      Indice para varrer a WaveFormTB
 *      Flag para executar timer
 *      Frequencia de chaveamento do pwm
 *      Divisor de frequencia, para d=1 => Fsin= Ftimer/len 
 */
int WaveTB[LEN_TB]={ 50,  65,  79,  90,  97, 100,  97,  90,  79,  65,  50,  35,  21,  10,   3,   0,   3,  10, 21,  35};
int indexTB = 0;
bool pass = false;
int freqSWitch = 20000;
int peso = 1;
int d = 1;

void readINA10k (){
  pass= true;
}

void setup(){

  // Open serial communications and wait for port to open:
  Serial.begin(115200);

   /*TIMER*/
  timer = timerBegin(0, 800, true);
  timerAttachInterrupt(timer, &readINA10k, true);
  timerAlarmWrite(timer, 10000000 * (1.0/freqSWitch), true);
  timerAlarmEnable(timer);
  //PWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIOPWM);
  pwm_config.frequency = 20000;                                                //frequencia do pwm F1 (1Khz) (acima da audivel )
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 10, 10);  //Enable deadtime on PWM0A and PWM0B with red = (656)*100ns & fed = (67)*100ns on PWM0A and PWM0B generated from PWM0A

  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_width(ADC_WIDTH_BIT_10);
}

void loop()
{
  if(pass){
    int d=WaveTB[indexTB];
    // pwm_config.cmpr_a = WaveTB[indexTB];
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    dacWrite(GPIODAC,(int)(d*0.01*255));
    indexTB++;
    if(indexTB==20)
      indexTB=0;
  }
  //delayMicroseconds(50);                                              // frequencia de 1 amostra F2 ( 1/100u = 10KHz)
}
