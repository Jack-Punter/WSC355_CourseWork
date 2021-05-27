#include "init_device.h"
#include "stm32f3xx.h" // Device header

//~ Initialisation code
// GPIO Configuration
void init_gpio(void) {
    // Enable clock on GPIO port A and E
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    
    // set E.9,11 & 13 to Alternative funcition mode for PWM
    GPIOE->MODER  |= 0x08880000;
    // Set the Alternative function of E.9, 11 & 13 to AF2
    // TIM_CH1, TIM_CH2 & TIM_CH3 respectively
    GPIOE->AFR[1] |= 0x00202020;
    
}

void init_pwm_timer(void) {
    // Enable to clock connection to the TIM1 device
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    
    // PSC = 79, ARR = 999 for 100Hz period
    // Povides a resolution of, 100% / (999 + 1) = 0.1%
    // incremenets to the duty cycle
    TIM1->PSC = 79;
    TIM1->ARR = 999;
    
    // Configure the timer to PWM mode using the Capture/Compare mode register
    // This sets standard PWM mode for channels 1, 2 and 3.
    TIM1->CCMR1 |= 0x00006060;
    TIM1->CCMR2 |= 0x00000060;
    //Set the CCR for the channel 1 to be 10%
    TIM1->CCR1 = 100;
    
    //Set the CCR for the channel 2 to be 50%
    TIM1->CCR2 = 500;
    
    //Set the CCR for the channel 3 to be 90%
    TIM1->CCR3 = 900;
    
    // Enable output mode in the Break and Dead Time Register (BDTR)
    TIM1->BDTR |= TIM_BDTR_MOE;
    // Enable the Channel 1 output of the timer in the
    // Capture/Compare enable register
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E ;
    
    // Enable the the timer
    TIM1->CR1 |= TIM_CR1_CEN;
}
