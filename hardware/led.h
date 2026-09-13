#ifndef LED_H
#define LED_H

#include "ti_msp_dl_config.h"

/* id: 0 = LED0(PB21), 1 = LED1(PB2), 2 = LED2(PB3)  —— 见 led.c */
void led_on(uint8_t id);
void led_off(uint8_t id);

#endif