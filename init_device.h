#ifndef INIT_DEVICE_H

#include "common.h"

//- Funciton prototypes
void init_gpio(void);
void init_dac(void);
void init_adc(void);
void init_timer(void);
void init_exti_interrupts(void);

#define INIT_DEVICE_H
#endif
