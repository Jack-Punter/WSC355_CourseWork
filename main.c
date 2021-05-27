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
    init_exti_interrupts();
    
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
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

const uint16_t PWM_Cycle[] = {
    0,    // 0%
    250,  // 25%
    500,  // 50%
    750,  // 75%
    1000, // 100%
};
void EXTI0_IRQHandler() {
    // Check the IRQ source
    if (EXTI->PR & EXTI_PR_PR0) {
        // Clear the pending IRQ bit 
        // this is a wc_w1 register meaning we write 1 to clear the bit.
        EXTI->PR |= EXTI_PR_PR0;
        
        count++;
        TIM1->CCR1 = PWM_Cycle[count % ArrayCount(PWM_Cycle)];
    }
}

