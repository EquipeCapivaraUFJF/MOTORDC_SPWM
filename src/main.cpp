#include <Arduino.h>
#include "Wire.h"
#include <stdio.h>
#include "esp_system.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/adc.h"
#include "driver/dac.h"

// // /*    |*    |*
// // *    Title: MOTORDC_SPWM (sinusoidal pwm)
// // *    Authors: Caio
// // *
// // *   |*
// // */
// // /*
// // ─ ─ ─ ─ ─ ─ ▄ ▌ ▐ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀▀ ▀ ▀▀ ▀ ▀▀ ▀ ▀▌
// // ─ ─ ─ ▄ ▄ █ █ ▌ █ ░   ░      # Equipe Capivara #                         ░░ ░  ░  ▐
// // ▄ ▄ ▄ ▌ ▐ █ █ ▌ █ ░  ░ ░░                                          ░░░ ░ ░ ░     ▐
// // █ █ █ █ █ █ █ ▌ █ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄▀ ▀ ▀▀ ▀ ▀▀ ▀ ▀▀ ▀ ▀ ▌===       ---------
// // ▀ (@) ▀ ▀ ▀ ▀ ▀ ▀ ▀ (@)(@) ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ ▀ (@) ▀ ▘                    (@)
// // */



// Definição do sensor de corrente e tensão.
// #define FREQsen 2
#define CANALDAC26 26
#define PWM_EN_L 19
#define PWM_EN_R 18

#define UP 85
#define DOWN 65
#define LEN 50
#define GPIO1 14  // pra IN1 Ponte H
#define GPIO2 16  // pra IN2 Ponte H
mcpwm_config_t pwm_config;

void setup(){

  pinMode(CANALDAC26, OUTPUT);
  pinMode(PWM_EN_R, OUTPUT);
  pinMode(PWM_EN_L, OUTPUT);

  digitalWrite(PWM_EN_L, HIGH);
  digitalWrite(PWM_EN_R, HIGH);

  Serial.begin(115200);
  
  /*SETUP VAR*/
  int freq_pwm = 20000;

  /*PWM*/
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO2);
  pwm_config.frequency = freq_pwm;                                                //frequencia do pwm F1 (20Khz) (acima da audivel )
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 0, 0);  //Enable deadtime on PWM0A and PWM0B with red = (656)*100ns & fed = (67)*100ns on PWM0A and PWM0B generated from PWM0A
}

void loop()
{

  float d = analogRead(34)*0.0002442;

  /*Descomente para fazer o DEBUG do duty-cycle vindo do gerador*/
  // Serial.print(d*100);
  // Serial.print("\n");

  dacWrite(CANALDAC26, int(d *255));
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, int(d*100));
  delayMicroseconds(50);
   
}