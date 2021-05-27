/* 
	Pete Hubbard 2019
	Loughborough University
	WSC055 Lab 1
	V.2.0
	
	The following 'c' code presents an outline for you to adapt during the laboratory
	
*/

#include "stm32f3xx.h"                  // Device header

void delay(int a); // prototype for delay function

volatile unsigned char count;

int main(void) {
    // Enable clock on GPIO port E
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;

    // GPIOE is a structure defined in stm32f303xc.h file
    // Define settings for each output pin using GPIOE structure
    
    // Set pins E.8 and E.12 as output
    GPIOE->MODER  |=   0x55550000;
    // Set E.8 to open-drain (others are 0, push-pull)
    // GPIOE->OTYPER |=   0x00000100;  
    // Set Pin E.8 to Pull up
    // GPIOE->PUPDR  |= 0x00010000;
    
    // Configure Timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 399;
    TIM3->ARR = 19999;
    // Set DIER Register to watch for 'Update' Interrupt Enable
    TIM3->DIER |= TIM_DIER_UIE;
    // Enable Timer 3 interrupt request in NVIC
    NVIC_EnableIRQ(TIM3_IRQn);

    // Start the timer
    TIM3->CR1 |= TIM_CR1_CEN;

    // Main programme loop - Use the 8 LEDs as a counter displaying the 8 bits of count
    while (1) {/* }
        // Turn off the LEDs that are not part of the current count
        GPIOE->BSRRH = (uint16_t)(~(count) << 8);
        // Turn on the LEDs that are part of the current count
        GPIOE->BSRRL = (uint16_t)(count << 8);
    */}
}


// Timer ISR
void TIM3_IRQHandler() {
    // If the interrupt source is a timer 'Update' interrupt
    if ((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF) {
        count++;
        // Turn off the LEDs that are not part of the current count
        GPIOE->BSRRH = (uint16_t)(~(count) << 8);
        // Turn on the LEDs that are part of the current count
        GPIOE->BSRRL = (uint16_t)(count << 8);
        
        // Reset the Update interrupt flag in the status register
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

// Delay function to occupy processor
void delay (int a) {
    volatile int i,j;
    
    for (i=0 ; i < a ; i++) {
        j++;
    }

    return;
}
