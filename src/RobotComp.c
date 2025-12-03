/************************************************
 * Team 18: A. Jacobs L. DeLussy
 * Robot competiton!!! 12/5/2025
 * NucleoF466RE CMSIS STM32F4xx
 */
#include "stm32f4xx.h"
#include "UART2.h"
#include <stdio.h>
#include "SSD_Array.h"
#include <stdlib.h>
#include <stdbool.h>

#define Freq  16000000UL // 16 MHz  
#define BAUD  115200
#define IR_SENSOR1    0 // PC0
#define IR_SEONSOR2  1 // PC1
#define IR_SENSOR3 2 // PC2
#define IR_SENSOR4  3 // PC3
#define IR_PORT   GPIOC
#define SERVO3_PIN    (8) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO3_PORT   (GPIOC)
#define SERVO1_PIN    (9) // Assuming servo motor 4 control pin is connected to GPIOC pin 8
#define SERVO1_PORT   (GPIOC)
#define BTN_PIN 13 // PC13
#define BTN_PORT GPIOC
#define USART2_TX_PIN 2 // PA2
#define USART2_RX_PIN 3 // PA3
#define USART2_GPIO_PORT GPIOA
#define ANALOG_PIN 1
#define ANALOG_PORT GPIOA
#define ADC_CHANNEL 1 // ADC Channel for PA1
#define ADC_SAMPLES 16 // Number of samples for averaging

volatile uint8_t digitSelect = 0;
bool pause = false;
//bool Passed_first_Line = false;
volatile int IRSensorReading = 0;
int IR_Reading_Binary = 0;
int pulse_width_2 = 1500; // Default pulse width for servos
int pulse_width_1 = 1500; // Default pulse width for servos
int phase = 0; //0 = line following mode 1 = Ultrasonic radar mode

void PWM_Output_PC6_PC8_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; // Enable clock for TIM8
    TIM8->PSC = 15; // 16 MHz / 16 = ~1 MHz (verify timer clock)
    TIM8->ARR = 19999; // Period for 50 Hz
    TIM8->CCR3 = 1500; // Duty cycle (1.5 ms center pulse)
    TIM8->CCR4 = 1500; // Duty cycle (1.5 ms center pulse)

    // configure channel1 for PWM mode 1 with preload
    TIM8->CCMR1 &= ~(0xFFU); // clear OC1M + OC1PE fields
    TIM8->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos); // PWM mode 1 (OC1M = 110)
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE; // Preload enable

    TIM8->CCMR2 &= ~(0xFFU); // clear OC4M + OC4PE fields

    TIM8->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos); // PWM mode 1 (OC4M = 110)
    TIM8->CCMR2 |= TIM_CCMR2_OC3PE; // Preload enable

    TIM8->CCMR2 |= (6U << TIM_CCMR2_OC4M_Pos); // PWM mode 1 (OC4M = 110)
    TIM8->CCMR2 |= TIM_CCMR2_OC4PE; // Preload enable

    TIM8->CCER &= ~TIM_CCER_CC3P; // active high (optional)
    TIM8->CCER |= TIM_CCER_CC3E; // Enable CH3 output (PC8)
    
    TIM8->CCER &= ~TIM_CCER_CC4P; // active high (optional)
    TIM8->CCER |= TIM_CCER_CC4E; // Enable CH4 output (PC9)
    
    TIM8->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
    

    TIM8->EGR = TIM_EGR_UG; // Generate update event to load PSC/ARR/CCR
    TIM8->BDTR |= TIM_BDTR_MOE; // Main output enable
    TIM8->CR1 |= TIM_CR1_CEN; // Enable timer

    GPIOC->MODER &= ~(0x3 << (8 * 2)); // clear mode bits for PC8
	GPIOC->MODER |=  (0x2 << (8 * 2)); // Alternate function
	GPIOC->AFR[1] &= ~(0xF << (0 * 4)); // clear AF bits for PC8
	GPIOC->AFR[1] |=  (0x3 << (0 * 4)); // AF3 (TIM8) for PC8
    
    GPIOC->MODER &= ~(0x3 << (9 * 2)); // Clear mode bits for PC9
    GPIOC->MODER |=  (0x2 << (9 * 2)); // Set PC9 to Alternate Function mode
    GPIOC->AFR[1] &= ~(0xF << (1 * 4));
    GPIOC->AFR[1] |=  (0x3 << (1 * 4)); // Set AF3 (TIM8) for PC9
}

// Configure TIM2 for 0.5 millisecond interrupt (update ssd)
void TIM2_init(void){
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 15; // Prescaler: (16MHz/(15+1) = 1MHz, 1usec period), divides clock frequency which lets it operate at a slower rate
    TIM2->ARR = 499; // Auto-reload, when : 500 counts for .5ms interrupt 
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 2); // Set priority for TIM2
    TIM2->CR1 = TIM_CR1_CEN; // Enable TIM2
}

void btn_init(void){
	// Enable button interrrupts
	//enable GPIOC
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;// Enable GPIOC

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock for EXTI
    BTN_PORT->MODER &= ~(0x3 << (BTN_PIN *2)); //set button as input

    EXTI->IMR |= (1<< BTN_PIN); // Unmask interrupt for button pin so the board does something when interrupt happens
    EXTI->FTSR |= (1 << BTN_PIN); // Trigger on falling edge (button press)

    SYSCFG->EXTICR[3] &= ~(0xF << (1*4)); //clear EXTI13 bits; EXTIRC[3] is where group w/ 15 exists
    SYSCFG->EXTICR[3] |= (0x2 << (1*4)); // Map EXTI13 to GPIOC pin 13
    NVIC_SetPriority(EXTI15_10_IRQn, 0); // Set interrupt to highest priority; sets any interrputs in group 10-15 to top priority
	NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable EXTI line[15:10] interrupts in NVIC
}

void Sensor_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
    IR_PORT->MODER &= ~((0x3 << (IR_SENSOR1 * 2)) | (0x3 << (IR_SEONSOR2 * 2)) | (0x3 << (IR_SENSOR3 * 2)) | (0x3 << (IR_SENSOR4 * 2))); // Set PC0-PC3 as input
    IR_PORT->PUPDR |= ((0x3 << (IR_SENSOR1 * 2)) | (0x3 << (IR_SEONSOR2 * 2)) | (0x3 << (IR_SENSOR3 * 2)) | (0x3 << (IR_SENSOR4 * 2))); //no pull-up/pull-down
}

void TIM2_IRQHandler(void){
    if(TIM2->SR & TIM_SR_UIF){ //check if update interrupt flag is set
        TIM2->SR &= ~TIM_SR_UIF; //clear the interrupt flag
        digitSelect = (digitSelect + 1) % 4; //cycle through digits 0-3
        To_Binary();
        SSD_update(digitSelect, IR_Reading_Binary, 0);
    }
}

void To_Binary(void){
    read_IRSensor();
    if(IRSensorReading == 15){
        IR_Reading_Binary = 0;
    } else if(IRSensorReading == 0){
        IR_Reading_Binary = 1 + (1*10) + (1*100) + (1*1000); //1111
    }else if(IRSensorReading == 9){
        IR_Reading_Binary = 0 + (1*10) + (1*100) + (0*1000); //0110
    }
}

void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & (1<< BTN_PIN)){ //check if interrupt is from button pin
        EXTI->PR |= (1<< BTN_PIN); //clear pending interrupt flag by writing 1 to it
		pause = !pause; // Toggle pause state
	}
}

void read_IRSensor(void){
    IRSensorReading = GPIOC->IDR & 0x0F; // Read PC0-PC3
}

void IR_cases(int sensor){
    read_IRSensor();
    if(!pause){
    switch(sensor){
        case 15:
                pulse_width_1 = 1500; 
                pulse_width_2 = 1500; 
                pause = true;
            break;
        case 14: // slight right
        //for (int i = 0; i < 1333333; i++); //potential problem
                pulse_width_2 = 1550; 
                pulse_width_1 = 1490; 
        break;
        case 12: // slight left
            pulse_width_2 = 1550; 
            pulse_width_1 = 1500;
        break;
        case 9:
                pulse_width_1 = 1435.20; 
                pulse_width_2 = 1550; 
            break;
        case 8: // left turn
                pulse_width_1 = 1475.20; 
                pulse_width_2 = 1550; 
            break;
        case 3: // slight right
                pulse_width_2 = 1500; 
                pulse_width_1 = 1450; 
            break;
        case 1: // right turn
            //for (int i = 0; i < 1333333; i++); //delay about 0.1s //potential problem
            pulse_width_1 = 1510; 
            pulse_width_2 = 1430; 
        break;
        case 0:
            read_IRSensor();
            pulse_width_1 = 1435.20; 
            pulse_width_2 = 1550;
            break;
        default:
               break;
    }
}else{
    pulse_width_1 = 1500; 
    pulse_width_2 = 1500; 
}
}

void SysTick_Handler(void){
    read_IRSensor();
    IR_cases(IRSensorReading);
    TIM8->CCR3 = pulse_width_1; // Update servo 3 pulse width
    TIM8->CCR4 = pulse_width_2; // Update servo 4 pulse width

}

int main(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SysTick_Config(1000000);

    SSD_init();
	UART2_Init();
	btn_init();
	PWM_Output_PC6_PC8_Init();
    //ADC1_Init();
    TIM2_init();
    Sensor_Init();
	while (1) {
	}
}