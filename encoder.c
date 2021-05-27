#include "encoder.h"

// The encoder state emulated by the user button ISR
static volatile uint8_t encoder_device;
//~ Encoder Emulation
void encoder_device_counterclockwise(void) {
    // Emulate Anti-Clockwise rotation of the encoder_device
    switch (encoder_device & ENC_CH_MASK) {
        case 0x0: {
            encoder_device = 0x2;
        } break;
        
        case 0x1: {
            encoder_device = 0x0;
        } break;
        
        case 0x2: {
            encoder_device = 0x3;
        } break;
        
        case 0x3: {
            encoder_device = 0x1;
        } break;
    }
}

void encoder_device_clockwise(void) {
    // Emulate Clockwise rotation of the encoder_device
    switch (encoder_device & ENC_CH_MASK) {
        case 0x0: {
            encoder_device = 0x1;
        } break;
        
        case 0x1: {
            encoder_device = 0x3;
        } break;
        
        case 0x2: {
            encoder_device = 0x0;
        } break;
        
        case 0x3: {
            encoder_device = 0x2;
        } break;
    }
}

void encoder_device_output(void) {
    // Write to PA[7:6] the encoder state to emulate the 
    // Encoder
    GPIOA->BSRRH = ((~encoder_device) & ENC_CH_MASK) << 6;
    GPIOA->BSRRL = (encoder_device & ENC_CH_MASK) << 6;
}
