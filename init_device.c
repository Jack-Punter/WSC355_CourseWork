#include "init_device.h"
#include "stm32f3xx.h" // Device header

// GPIO Configuration
void init_gpio(void) {
    // Enable clock on GPIO ports A and E
    RCC->AHBENR |=  RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOEEN;
    
    // We are using PA as input pins which is the reset
    // state of the MODER regster
    
    // Set PE.8 and PE.9 as output for encoder ouptut
    GPIOE->MODER  |= 0x00050000;
    
    // Set PE[14:11] as output for encoder ouptut
    GPIOE->MODER  |= 0x15400000;
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
}
