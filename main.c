#include <stdint.h>
#include "stm32f3xx.h" // Device header

#include "common.h"
#include "init_device.h"

#define CLOCK_WISE_EMULATION 1

// This value will hold the encoder_device state in
// the lest significant 2 bits.

#define ENC_C_MASK   0x3
#define ENC_CA       0x1
#define ENC_CB       0x2
// The encoder state emulated by the user button ISR
volatile uint8_t encoder_device;
// The encoder state read
volatile uint8_t encoder_state;
volatile uint8_t encoder_counter;

//~ Program (main)
int main(void) {
    init_gpio();
    init_exti_interrupts();
    
    //-Main program loop
    // Display the ADC value on the LEDs
    while (1) {
        // Clear the LEDs that are not part of the 4-bit count
        GPIOE->BSRRH = ((~encoder_counter) & 0xF) << 11;
        // Set the LEDs that are in the 4bit count
        GPIOE->BSRRL = (encoder_counter & 0xF) << 11;
    }
}

//~ Interrupt Service Routines
void encoder_device_counterclockwise() {
    // Emulate Clockwise rotation of the encoder_device
    switch (encoder_device & ENC_C_MASK) {
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

void encoder_device_clockwise() {
    // Emulate Anti-Clockwise rotation of the encoder_device
    switch (encoder_device & ENC_C_MASK) {
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

void EXTI0_IRQHandler() {
    // Check the IRQ source is the USR button
    if (EXTI->PR & EXTI_PR_PR0) {
        // Clear the pending IRQ bit 
        // this is a wc_w1 register meaning we write 1 to clear the bit.
        EXTI->PR |= EXTI_PR_PR0;

#if CLOCK_WISE_EMULATION
        encoder_device_clockwise();
#else
        encoder_device_counterclockwise();
#endif
        // Output the encoder_device state on PE.8 and PE.9
        GPIOE->BSRRH = (~encoder_device & 0x3) << 8;
        GPIOE->BSRRL = (encoder_device & 0x3) << 8;
    }
}

/*
 * encoder_device Channel A: PE.8 -> PA.1
 * encoder_device Channel B: PE.9 -> PA.3
 */
void EXTI1_IRQHandler() {
    // Triggers when encoder_device channel A changes
    // if the interrupt source is PA.1
    if (EXTI->PR & EXTI_PR_PR1) {
        // Clear the pending IRQ bit 
        EXTI->PR |= EXTI_PR_PR1;
        
        // If the previous state was 0b00 or 0b11
        if ((encoder_state & ENC_C_MASK) == 0x0 ||
            (encoder_state & ENC_C_MASK) == 0x3) {
            encoder_counter++;
        } else {
            encoder_counter--;
        }
        // Clear the previous state of the encoder channels
        encoder_state &= ~(ENC_C_MASK);
        // Write the new state of the encoder channels
        encoder_state |= encoder_device & ENC_C_MASK;
    }
}

void EXTI3_IRQHandler() {
    // Triggers when encoder_device channel B changes
    
    // if the interrupt source is PA.3
    if (EXTI->PR & EXTI_PR_PR3) {
        // Clear the pending IRQ bit 
        EXTI->PR |= EXTI_PR_PR3;

        // If the previous state was 0b00 or 0b11
        if ((encoder_state & ENC_C_MASK) == 0x0 ||
            (encoder_state & ENC_C_MASK) == 0x3) {
            encoder_counter--;
        } else {
            encoder_counter++;
        }
        
        // Clear the previous state of the encoder channels
        encoder_state &= ~(ENC_C_MASK);
        // Write the new state of the encoder channels
        encoder_state |= encoder_device & ENC_C_MASK;
    }
}
