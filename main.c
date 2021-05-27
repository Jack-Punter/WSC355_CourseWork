#include <stdint.h>
#include "stm32f3xx.h" // Device header

#include "common.h"
#include "init_device.h"
#include "encoder.h"

#define LED_STATE_MASK (0x7 << 8)
// These states values are picked to align with the LED
// bits that represent them.
#define STATE_POT_TEST (1 << 8)
#define STATE_ENCODER_TEST (1 << 9)
#define STATE_COMBINED_TEST (1 << 10)

volatile uint16_t global_prog_state = STATE_ENCODER_TEST;

volatile uint16_t enc_timer_count;
volatile uint16_t pot_timer_count;
volatile int8_t timer_delta = 1;

volatile uint8_t adc_idx = 0;
volatile uint16_t adc_values[5];

// This value will hold the encoder_device state in
// the lest significant 2 bits.

// The encoder state read
volatile uint8_t encoder_state;
// The counter for the encoder
volatile uint8_t encoder_counter;

uint16_t average_adc_value(void) {
    return (adc_values[0] + adc_values[1] + adc_values[2] +
            adc_values[3] + adc_values[4]) / 5;
}

uint16_t combined_output(void) {
    uint16_t adc_average_5bit = (int)((average_adc_value() / 256.0f) * 32);
    uint16_t enc_5bit = (int)((encoder_counter / 128.0f) * 32);
    uint16_t result = (adc_average_5bit + enc_5bit) / 2;
    return result;
}

//~ Program (main)
int main(void) {
    init_gpio();
    init_dac();
    init_adc();
    init_timer();
    init_exti_interrupts();


    // Clear all the LEDs
    GPIOE->BSRRH = 0xFF00;
    // Set the LEDs to the current state
    GPIOE->BSRRL = global_prog_state;
   
    //-Main program loop
    // Display the ADC value on the LEDs
    while (1) {
        uint8_t five_bit_pos = 0;
        
        ADC1->CR |= ADC_CR_ADSTART; // Start a conversion using ADC
        // Wait for conversion to complete
        WAIT_FOR_BIT_SET(ADC1->ISR, ADC_ISR_EOC);
        adc_values[adc_idx] = ADC1->DR;
        
        switch (global_prog_state) {
            case STATE_POT_TEST: {
                five_bit_pos = (int)(((float)adc_values[adc_idx] / 256.0f) * 32);
            } break;
            
            case STATE_ENCODER_TEST: {
                // counter / 128 (90 degrees of encoder) * 32 to map into 5 bit range
                five_bit_pos = (int)(((float)encoder_counter / 128.0f) * 32);
            } break;
            
            case STATE_COMBINED_TEST: {
                five_bit_pos = combined_output();
            } break;
        }
        GPIOE->BSRRH = 0xF800;
        GPIOE->BSRRL = five_bit_pos << 11;
        
        adc_idx = (adc_idx + 1) % ArrayCount(adc_values);
    }
}


//~ Interrupt Service Routines
void TIM3_IRQHandler() {
    // If the interrupt source is a timer 'Update' interrupt
    if ((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF) {        
        // Reset the Update interrupt flag in the status register
        TIM3->SR &= ~TIM_SR_UIF;
        

        pot_timer_count += timer_delta;
        
        // 3640 / 113 ~= 32
        if (pot_timer_count % 32 == 0) {
            enc_timer_count += timer_delta;
            if (timer_delta == 1) {
                encoder_device_clockwise();
            } else if (enc_timer_count != 0) {
                encoder_device_counterclockwise();
            }
        }

        // Clamp pot to range of input
        // 3640 = 8/9 * 4095
        if (pot_timer_count == 3640) {
            timer_delta = -1;
            enc_timer_count = 113;
        } else if (pot_timer_count == 0) {
            timer_delta = 1;
        }
        
        // Write the new state to the pins emulating the encoder
        encoder_device_output();
        // output the pot value
        DAC1->DHR12R1 = pot_timer_count;
    }
}

// Triggers when the User Button is pressed
// Change output mode
void EXTI0_IRQHandler() {
    // Check the IRQ source is the USR button
    if (EXTI->PR & EXTI_PR_PR0) {
        // Clear the pending IRQ bit 
        // this is a wc_w1 register meaning we write 1 to clear the bit.
        EXTI->PR |= EXTI_PR_PR0;
        
        switch(global_prog_state) {
            case STATE_POT_TEST: {
                global_prog_state = STATE_ENCODER_TEST;
            } break;
            
            case STATE_ENCODER_TEST: {
                global_prog_state = STATE_COMBINED_TEST;
            } break;
            
            case STATE_COMBINED_TEST: {
                global_prog_state = STATE_POT_TEST;
            } break;
        }
        
        // Clear the LEDs representing the current state
        GPIOE->BSRRH = LED_STATE_MASK;
        // Set the LEDs to the current state
        GPIOE->BSRRL = global_prog_state;
    }
}

/*
 * encoder_device Channel A: PA.4 -> PA.1
 * encoder_device Channel B: PA.5 -> PA.3
 */
// Triggers when encoder_device channel A changes
void EXTI1_IRQHandler() {

    // if the interrupt source is PA.1
    if (EXTI->PR & EXTI_PR_PR1) {
        // Clear the pending IRQ bit 
        EXTI->PR |= EXTI_PR_PR1;
        
        // If the previous state was 0b00 or 0b11
        if ((encoder_state & ENC_CH_MASK) == 0x0 ||
            (encoder_state & ENC_CH_MASK) == 0x3)
        {
            encoder_counter++;
        } else {
            encoder_counter--;
        }
        
        // Update the saved encoder state by toggling the
        // Channel A bit
        encoder_state ^= ENC_CH_A;
    }
}

// Triggers when encoder_device channel B changes
void EXTI3_IRQHandler() {
    // if the interrupt source is PA.3
    if (EXTI->PR & EXTI_PR_PR3) {
        // Clear the pending IRQ bit 
        EXTI->PR |= EXTI_PR_PR3;

        // If the previous state was 0b00 or 0b11
        if ((encoder_state & ENC_CH_MASK) == 0x0 ||
            (encoder_state & ENC_CH_MASK) == 0x3) {
            encoder_counter--;
        } else {
            encoder_counter++;
        }
        
        // Update the saved encoder state by toggling the
        // Channel A bit
        encoder_state ^= ENC_CH_B;
    }
}
