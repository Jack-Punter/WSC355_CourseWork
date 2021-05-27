#include <stdint.h>
#include "stm32f3xx.h" // Device header

#include "common.h"
#include "init_device.h"


//~ Program (main)
int main(void) {
    init_gpio();
    //init_timer();
    //init_dac();
    //init_adc();
    //init_opamp();
    init_pwm_timer();
    
    //-Main program loop
    // Display the ADC value on the LEDs
    volatile uint8_t adc_value = 0;
    while (1) {
        ADC1->CR |= ADC_CR_ADSTART;
        
        // Wait until the 'end of conversion', EOC,  bit is set in the ISR register
        // while((ADC1->ISR & ADC_ISR_EOC) == 0))
        WAIT_FOR_BIT_SET(ADC1->ISR, ADC_ISR_EOC);
        
        //adc_value = ADC1->DR;
        // Display the ADC value on the LEDs
        
        //GPIOE->BSRRH = (uint16_t)(~adc_value << 8);
        //GPIOE->BSRRL = (uint16_t)(adc_value << 8);
    }
}

//~ Interrupt Service Routines
/*
 We are only using 8 bits 0-255
 The DAC uses a 12-bit input meaning we can use values 0-4095
 The DAC's output is linear from 0-3.3v, therefore
 the maximum output using 8 bits = 255 / 4095 * 3.3v which is approx 0.2v
*/

volatile uint8_t count;
void TIM3_IRQHandler() {
    // If the interrupt source is a timer 'Update' interrupt
    if ((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF) {
        count++;
        
        // Write count to the DAC
        // (This will act as a sawtooth wave)
        // DAC1->DHR12R2 = count;
        
        // Reset the Update interrupt flag in the status register
        // TIM3->SR &= ~TIM_SR_UIF;
    }
}

