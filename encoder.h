#ifndef ENCODER_H

#include "common.h"

void encoder_device_counterclockwise(void);
void encoder_device_clockwise(void);
void encoder_device_output(void);

#define ENC_CH_MASK   0x3
#define ENC_CH_A      0x1
#define ENC_CH_B      0x2

#define ENCODER_H
#endif
