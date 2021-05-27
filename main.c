#include <stdint.h>
#include "stm32f3xx.h" // Device header

//-
// TODO
// Configure OPAMP1 (Input PA.5 Output to PA.2)
// ADC INput already on PA.2
//-

//~ Defines and prototypes
#define WAIT_FOR_BIT_SET(r, bit) while(((r) & (bit)) == 0)
#define WAIT_FOR_BIT_CLR(r, bit) while(((r) & (bit)) == (bit))

//- Funciton prototypes
void init_gpio(void);
void init_timer(void);
void init_dac(void);
void init_adc(void);
void init_opamp(void);

//~ Program (main)
int main(void) {
    init_gpio();
    init_timer();
    init_dac();
    init_adc();
    init_opamp();
    
    //-Main program loop
    // Display the ADC value on the LEDs
    volatile uint8_t adc_value = 0;
    while (1) {
        ADC1->CR |= ADC_CR_ADSTART;
        
        // Wait until the 'end of conversion', EOC,  bit is set in the ISR register
        // while((ADC1->ISR & ADC_ISR_EOC) == 0))
        WAIT_FOR_BIT_SET(ADC1->ISR, ADC_ISR_EOC);
        
        adc_value = ADC1->DR;
#if 1
        // Display the ADC value on the LEDs
        GPIOE->BSRRH = (uint16_t)(~(adc_value) << 8);
        GPIOE->BSRRL = (uint16_t)(adc_value << 8);
#endif
    }
}


//~ Initialisation code
// GPIO Configuration
void init_gpio(void) {
    // Enable clock on GPIO port A and E
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOEEN;
    
    // Set pins A.5 and A.2 as Analog
    // DAC1 channel2 and ADC1_IN3 respectively
    GPIOA->MODER  |=   0x00000C30;
    
    // Set pins E.15:8 to output mode so that we can use the LEDs
    GPIOE->MODER  |=   0x55550000;
}

//- Timer Configuration 
// Derivation of PSC and ARR registers
//          Time=1s   Sysclk=8Mhz
//             Time = ((PSC + 1) / Sysclk) * (ARR+1)
//    Time * Sysclk =(PSC+1)(ARR+1)
//             8MHz = (PSC + 1)(ARR + 1)
//              PSC = 399
// 8MHz / (399 + 1) = ARR+1 = 20000
//              ARR = 20000 - 1 = 19999
//        PSC = 399   ARR = 19999
//-
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
        DAC1->DHR12R2 = count;
        
        // Reset the Update interrupt flag in the status register
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

