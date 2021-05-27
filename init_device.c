#include "common.h"
#include "init_device.h"


// GPIO Configuration
void init_gpio(void) {
    // Enable clock on GPIO ports A and E
    RCC->AHBENR |=  RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOEEN;
    
    // We are using PA as input pins which is the reset
    // state of the MODER regster
    // Set PA.6 & PA.7 as output mode
    GPIOA->MODER |= 0x00005000;
    // Set PA.2 & PA.4 as analog
    GPIOA->MODER |= 0x00000330;

    // Set PE[15:8] and PE[1:0] as output 
    // PE[1:0] is for eumulated encoder output
    GPIOE->MODER  |= 0x55550005;
    //GPIOA->PUPDR  |= 0x0000000A;
}

/* Timer Configuration: Derivation of PSC and ARR registers for 226hz timer
 *       Time = 1/226   Sysclk = 8Mhz
 *               Time = (PSC + 1) / Sysclk * (ARR + 1)
 *      Time * Sysclk = (PSC + 1)(ARR + 1)
 *              35398 = (PSC + 1)(ARR + 1)
 *                PSC = 1608
 *     35398 / (1609) = ARR + 1 = 22
 *           ARR = 22 - 1 = 21
 *         PSC = 1608   ARR=21
 */
void init_timer(void) {
    // Enable clock to timer 3
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    TIM3->PSC = 60;
    TIM3->ARR = 17;
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

    // Disable the "buffer" functionality on DAC channel 1 (DAC_CR_BOFF1)
    // Enable DAC channel 1 (DAC_CR_EN1)
    DAC1->CR |= (DAC_CR_BOFF1 | DAC_CR_EN1);
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

void init_exti_interrupts(void) {
    // GPIOA Enabled in the init_gpio function
    
    // Enable the clock connection to the interrupt controller
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    //- User Button Interrupt Configuration
    // Enable an interrupt to be generated using the EXTI_IMR register
    // Unmask EXTI0
    EXTI->IMR |= EXTI_IMR_MR0;
    // Set trigger on rising edge
    EXTI->RTSR |= EXTI_RTSR_TR0;
    // Configure multiplexing to set PA.0 (User button)
    // to generate an interrupt EXTI0;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

    //- Emulated Encoder input pins.
    // Remove the mask for the EXTI1 and 3 interrupt sources
    EXTI->IMR |= EXTI_IMR_MR1 | EXTI_IMR_MR3;
    // Configure both falling and rising edge iterrupt trigger
    // for both pins
    EXTI->RTSR |= EXTI_RTSR_TR1 | EXTI_RTSR_TR3;
    EXTI->FTSR |= EXTI_FTSR_TR1 | EXTI_FTSR_TR3;
    // Configure multiplexing to set PA.1 and PA.3 as the sources
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA | SYSCFG_EXTICR1_EXTI3_PA;
    
    // Enable the IRQs
    // enable the IRQ
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    
    NVIC_SetPriority(EXTI1_IRQn, 0x0F00);
    NVIC_SetPriority(EXTI3_IRQn, 0x0F00);
}
