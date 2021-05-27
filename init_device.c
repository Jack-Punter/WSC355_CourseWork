#include "init_device.h"
#include "stm32f3xx.h" // Device header

// GPIO Configuration
void init_gpio(void) {
    // Enable clock on GPIO port A and E
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN| RCC_AHBENR_GPIOEEN;
    
    // No changes need to be made to the GPIOA MODER register
    // as the reset state for PA.0 is Input mode
    
    // set E.15 to output and E.9,11 & 13 to Alternative funcition mode for PWM
    GPIOE->MODER  |= 0x48880000;
    // Set the Alternative function of E.9, 11 & 13 to AF2
    // TIM_CH1, TIM_CH2 & TIM_CH3 respectively
    GPIOE->AFR[1] |= 0x00202020;
    
} 

/* Timer Configuration: Derivation of PSC and ARR registers
 *          Time=1s   Sysclk=8Mhz
 *             Time = ((PSC + 1) / Sysclk) * (ARR+1)
 *    Time * Sysclk =(PSC+1)(ARR+1)
 *             8MHz = (PSC + 1)(ARR + 1)
 *              PSC = 399
 * 8MHz / (399 + 1) = ARR+1 = 20000
 *              ARR = 20000 - 1 = 19999
 *        PSC = 399   ARR = 19999
 */
void init_timer(void) {
    // Enable clock to timer 3
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    TIM3->PSC = 399;
    TIM3->ARR = 1999;
    // Set DIER Register to watch for 'Update' Interrupt Enable
    TIM3->DIER |= TIM_DIER_UIE;
    // Enable Timer 3 interrupt request in NVIC
    NVIC_EnableIRQ(TIM3_IRQn);
    
    // Start the timer
    TIM3->CR1 |= TIM_CR1_CEN;
}

// DAC Configuration
void init_dac(void) {
    // Enable the clock connection to the DAC
    RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
    
    // Disable the "buffer" functionality on DAC channel 2 (DAC_CR_BOFF2)
    // Enable DAC channel 2 (DAC_CR_EN2)
    DAC1->CR |= (DAC_CR_BOFF2 | DAC_CR_EN2);
}

// ADC Configuration
void init_adc(void) {
    // The GPIO pin input for this device, PA.2 (ADC1_IN3),
    // is configured in the init_gpio function to keep gpio configuration
    // in one place
    
    // Write 0b00 then 0b01 to the ADVREGEN bit in the control register
    // To enable the ADC Voltage Regulator:
    ADC1->CR &= ~ADC_CR_ADVREGEN;
    ADC1->CR |= ADC_CR_ADVREGEN_0;
    // Loop 100 times to delay for approx 10us
    for(volatile int i = 0; i < 100; i++);
    
    // Set calibration mode to single-ended
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    // Start Calibration
    ADC1->CR |= ADC_CR_ADCAL;
    
    // loop until the ADC is finished calibrating.
    // Signalled  by the ADCAL bit being cleared in the control register
    WAIT_FOR_BIT_CLR(ADC1->CR, ADC_CR_ADCAL);
    
    // Enable the clock connections
    RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV2;
    RCC->AHBENR |= RCC_AHBENR_ADC12EN;
    ADC1_2_COMMON->CCR |= 0x00010000;
    
    // Configure resolution to 8-bit
    ADC1->CFGR |= ADC_CFGR_RES_1;
    // Set right hand alignment
    ADC1->CFGR &= ~ADC_CFGR_ALIGN;
    // Set non-continuous operation
    ADC1->CFGR &= ~ADC_CFGR_CONT;
    
    // We are only using 1 input (channel 3)
    // SQ1 is in bits [10:6] of the SQR1 register
    //             channel 3
    ADC1->SQR1 |= (0x3 << 6);
    // Set the sequence length to 1 Conversion (clear the bits)
    ADC1->SQR1 &= (~ADC_SQR1_L);
    
    // Specify that we will use 7.5 cycle sample time 
    ADC1->SMPR1 = (0x3 << 3);
    
    // Enable the ADC
    ADC1->CR |= ADC_CR_ADEN;
    // Wait for the ADC to be ready
    WAIT_FOR_BIT_SET(ADC1->ISR, ADC_ISR_ADRD);
}

// OpAmp Configuration
void init_opamp() {
    // Enable the clock connection to the opamp
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    // The GPIO configuration for this device is already 
    // done in the init_gpio function, we are using PA.5
    // (DAC channel2) as the input and PA.2 (ADC1_IN3)
    
    // configure PA.5 as Non-inverting input (0x4)
    // Enabale PGA mode in the VM_SEL (0x40)
    OPAMP1->CSR |= 0x00000044;
    // Enable the OpAmp
    OPAMP1->CSR |= 0x00000001;
    
    // Clear PGA_GAIN [17:14]
    OPAMP1->CSR &= ~(0xF << 14);
    // Set the OpAmp gain to 16 in PGA_GAIN
    // 16 * 0.2v = 3.2v which matches
    OPAMP1->CSR |= (0x3 << 14);
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
    
    // Enable the timer
    TIM1->CR1 |= TIM_CR1_CEN;
}

void init_exti_interrupts(void) {
    // GPIOA Enabled in the init_gpio function
    
    // Enable the clock connection to the interrupt controller
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    // Enable an interrupt to be generated using the EXTI_IMR register
    // Unmask EXTI0
    EXTI->IMR |= EXTI_IMR_MR0;
    
    // Set trigger to be rising edge
    EXTI->RTSR |= EXTI_RTSR_TR0;
    
    // Configure multiplexing to set PA.0 to generate an interrupt EXTI0;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
    
    // enable the IRQ
    NVIC_EnableIRQ(EXTI0_IRQn);
}
