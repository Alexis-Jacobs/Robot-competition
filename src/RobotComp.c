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
#define SERVO3_PIN (6) // PC6
#define SERVO3_PORT (GPIOC)
#define SERVO2_PIN    (8) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO2_PORT   (GPIOC)
#define SERVO1_PIN    (9) // Assuming servo motor 4 control pin is connected to GPIOC pin 8
#define SERVO1_PORT   (GPIOC)
#define ENCODER_PIN   (7) // Assuming servo motor encoder is connected to GPIOC pin 7
#define ENCODER_PORT  (GPIOC)
#define TRIG_PORT GPIOA
#define TRIG_PIN  4
#define ECHO_PORT GPIOB
#define ECHO_PIN  0
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
int right_distance = 0;
int left_distance = 0;

volatile uint32_t sysTickStart = 0; //echo goes low to high
volatile uint32_t EchoStart = 0; //echo goes low to high
volatile uint32_t EchoEnd = 0; //echo goes high to low
volatile uint32_t EchoWidth = 0; //timing from rise to fall, captured
float distance = 0; //distance in cm
bool unit = true; //true = cm, false = inches
int didgit = 0; // for ssd display
int angle = 0; //for servo control
volatile uint32_t pulse_width = 0; //servo pulse width
int time = 0;
int time_ms = 0;

#define RADAR_IDLE        0
#define RADAR_TRIGGER     1
#define RADAR_WAIT_HIGH   2
#define RADAR_WAIT_LOW    3
#define RADAR_SCAN_RIGHT  4
#define RADAR_SCAN_LEFT   5
#define RADAR_DECIDE      6
#define PARK   7
int radar_state = RADAR_IDLE;

uint32_t t0;

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

// Configure TIM2 for 0.1 milisecond interrupt (update ssd)
void TIM2_init(void){
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 15; // Prescaler: (16MHz/(15+1) = 1MHz, 1usec period), divides clock frequency which lets it operate at a slower rate
    TIM2->ARR = 99; // Auto-reload, when : 100000 counts for 0.1s interrupt 
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 2); // Set priority for TIM2
    TIM2->CR1 = TIM_CR1_CEN; // Enable TIM2
}

void TIM5_INIT(void){
    //Configure TIM5 for 10microsecond pulses
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable TIM2 clock
    TIM5->PSC = 15; // Prescaler: (16MHz/(15+1) = 1MHz, 1usec period), divides clock frequency which lets it operate at a slower rate
    TIM5->ARR = 0xFFFFFFFF; // Auto-reload, when : 1000 counts for 1ms interrupt
    TIM5->EGR |= TIM_EGR_UG; // Enable update interrupt
    TIM5->CR1 = TIM_CR1_CEN; // Enable TIM5
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

void PWM_Output_PC6_Init(void) {
    // Enable GPIOC and TIM3 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    // Set PC6 to alternate function (AF2 for TIM3_CH1)
    GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2));
    GPIOC->MODER |= (0x2 << (SERVO3_PIN * 2)); // Alternate function
    GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4));
    GPIOC->AFR[0] |= (0x2 << (SERVO3_PIN * 4)); // AF2 = TIM3
    // Configure TIM3 for PWM output on CH1 (PC6)
    TIM3->PSC = (Freq/1000000) - 1; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
    TIM3->ARR = 19999; // 20 ms period (50 Hz)
    TIM3->CCR1 = 1500; // 1.5 ms pulse width (neutral position); 90 degrees
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M);
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // Conficgures TIM3 CH1 as PWM mode 1 (makes sure output is waveform)
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload for CH1
    TIM3->CCER |= TIM_CCER_CC1E; // Enable CH1 output (PC6)
    TIM3->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
    TIM3->EGR = TIM_EGR_UG; // Generate update event
    TIM3->CR1 |= TIM_CR1_CEN; // Enable timer
}

void servo_angle_set(int angle) {
	pulse_width = 1500 - (500 * (angle/45.0)); // calculate pulse width for angle (-45 to 45 degrees) 500us per 45 degrees
	TIM3->CCR1 = pulse_width;
}

void TIM2_IRQHandler(void){
    if(TIM2->SR & TIM_SR_UIF){ //check if update interrupt flag is set
        TIM2->SR &= ~TIM_SR_UIF; //clear the interrupt flag
        digitSelect = (digitSelect + 1) % 4; //cycle through digits 0-3
        if (pause == false){
        time_ms ++;
        if (time_ms >= 10){
            time_ms = 0;
            time ++;
        }
        SSD_update(digitSelect, time/100, 3);
    } else if(pause == true && time == 0){
        SSD_update(digitSelect, 0, 3);
    }else{
        SSD_update(digitSelect, time/100, 3);
    }
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
/*
void radar_update(void){
    switch(radar_state){
        case RADAR_TRIGGER:
            TRIG_PORT->ODR |= (1 << TRIG_PIN); //set trigger high
            t0 = TIM5->CNT;
            radar_state = RADAR_WAIT_HIGH;
            break;
        
        case RADAR_WAIT_HIGH:
            if((TIM5->CNT - t0) >= 10){
                TRIG_PORT->ODR &= ~(1 << TRIG_PIN); //set trigger low
            }
            if(ECHO_PORT->IDR & (1<< ECHO_PIN)){ //wait for echo to go high
                EchoStart = TIM5->CNT;
                radar_state = RADAR_WAIT_LOW;
            }
            break;

        case RADAR_WAIT_LOW:
            if(!(ECHO_PORT->IDR & (1<< ECHO_PIN))){ //wait for echo to go low
                EchoEnd = TIM5->CNT;
                //calculate echo width
                if(EchoEnd >= EchoStart){
                    EchoWidth = EchoEnd - EchoStart;
                } else { //handle timer overflow
                    EchoWidth = (0xFFFFFFFF - EchoStart) + EchoEnd;
                }

                servo_angle_set(-45);

                left_distance = EchoWidth * 0.01715; //distance in cm
                
                t0= TIM5->CNT;

                radar_state = RADAR_SCAN_RIGHT;
            }
            break;
        case RADAR_SCAN_RIGHT:
            if((TIM5->CNT - t0) >= 500000){ //wait 500ms
                servo_angle_set(45);
                right_distance = EchoWidth * 0.01715; //distance in cm
                t0 = TIM5->CNT;
                radar_state = RADAR_DECIDE;
            }
            break;
        case RADAR_DECIDE:
        t0 = TIM5->CNT;
            if(right_distance > left_distance && (TIM5->CNT) - t0 <= 500000){
                //turn right
                pulse_width_1 = 1510; 
                pulse_width_2 = 1430; 
            } else if (left_distance > right_distance && (TIM5->CNT) - t0 <= 500000){
                //turn left
                pulse_width_1 = 1475.20; 
                pulse_width_2 = 1550; 
            } else {
                //go straight
                pulse_width_1 = 1500; 
                pulse_width_2 = 1500; 
                radar_state = PARK;
            }
        break;
        case PARK:
        read_IRSensor();
            if(IRSensorReading != 15){
                pulse_width_1 = 1500; 
                pulse_width_2 = 1500;
                radar_state = RADAR_IDLE;
            }
        break;
    }
}
*/
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
                //phase = 1;
                //radar_state = RADAR_TRIGGER;
            break;
        case 14: //left
                pulse_width_1 = 1500; 
                pulse_width_2 = 1570; 
        break;
        case 12: // slight left
            pulse_width_1 = 1500;
            pulse_width_2 = 1570; 
        break;
        case 9:
                pulse_width_1 = 1415.20; 
                pulse_width_2 = 1570; 
            break;
        case 8: // left turn
                pulse_width_1 = 1475.20; 
                pulse_width_2 = 1570; 
            break;
        case 7: // slight right
                pulse_width_1 = 1420; 
                pulse_width_2 = 1500; 
            break;
        case 3: // slight right
                pulse_width_1 = 1420; 
                pulse_width_2 = 1500; 
            break;
        case 1: // right turn
            pulse_width_1 = 1410; 
            pulse_width_2 = 1500; 
        break;
        case 0:
            read_IRSensor();
            pulse_width_1 = 1415.20; 
            pulse_width_2 = 1570;
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
    //if (phase == 0) {// Line following mode
        read_IRSensor();
        IR_cases(IRSensorReading);
    /*} else if (phase == 1) {// Ultrasonic radar mode
        radar_update();
    }*/
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
    PWM_Output_PC6_Init();
    TIM2_init();
    TIM5_INIT();
    Sensor_Init();
	while (1) {
        //radar_update();
	}
}



/************************************************
 * Team 18: A. Jacobs L. DeLussy
 * Robot competiton!!! 12/5/2025
 * NucleoF466RE CMSIS STM32F4xx
 */
/*#include "stm32f4xx.h"
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
#define SERVO3_PIN (6) // PC6
#define SERVO3_PORT (GPIOC)
#define SERVO2_PIN    (8) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO2_PORT   (GPIOC)
#define SERVO1_PIN    (9) // Assuming servo motor 4 control pin is connected to GPIOC pin 8
#define SERVO1_PORT   (GPIOC)
#define ENCODER_PIN   (7) // Assuming servo motor encoder is connected to GPIOC pin 7
#define ENCODER_PORT  (GPIOC)
#define TRIG_PORT GPIOA
#define TRIG_PIN  4
#define ECHO_PORT GPIOB
#define ECHO_PIN  0
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
int right_distance = 0;
int left_distance = 0;

volatile uint32_t sysTickStart = 0; //echo goes low to high
volatile uint32_t EchoStart = 0; //echo goes low to high
volatile uint32_t EchoEnd = 0; //echo goes high to low
volatile uint32_t EchoWidth = 0; //timing from rise to fall, captured
float distance = 0; //distance in cm
bool unit = true; //true = cm, false = inches
int didgit = 0; // for ssd display
int angle = 0; //for servo control
volatile uint32_t pulse_width = 0; //servo pulse width
int time = 0;
int time_ms = 0;

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

// Configure TIM2 for 0.1 milisecond interrupt (update ssd)
void TIM2_init(void){
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 15; // Prescaler: (16MHz/(15+1) = 1MHz, 1usec period), divides clock frequency which lets it operate at a slower rate
    TIM2->ARR = 99; // Auto-reload, when : 100000 counts for 0.1s interrupt 
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 2); // Set priority for TIM2
    TIM2->CR1 = TIM_CR1_CEN; // Enable TIM2
}

void TIM5_INIT(void){
    //Configure TIM5 for 10microsecond pulses
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable TIM2 clock
    TIM5->PSC = 15; // Prescaler: (16MHz/(15+1) = 1MHz, 1usec period), divides clock frequency which lets it operate at a slower rate
    TIM5->ARR = 0xFFFFFFFF; // Auto-reload, when : 1000 counts for 1ms interrupt
    TIM5->EGR |= TIM_EGR_UG; // Enable update interrupt
    TIM5->CR1 = TIM_CR1_CEN; // Enable TIM5
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

void PWM_Output_PC6_Init(void) {
    // Enable GPIOC and TIM3 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    // Set PC6 to alternate function (AF2 for TIM3_CH1)
    GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2));
    GPIOC->MODER |= (0x2 << (SERVO3_PIN * 2)); // Alternate function
    GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4));
    GPIOC->AFR[0] |= (0x2 << (SERVO3_PIN * 4)); // AF2 = TIM3
    // Configure TIM3 for PWM output on CH1 (PC6)
    TIM3->PSC = (Freq/1000000) - 1; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
    TIM3->ARR = 19999; // 20 ms period (50 Hz)
    TIM3->CCR1 = 1500; // 1.5 ms pulse width (neutral position); 90 degrees
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M);
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // Conficgures TIM3 CH1 as PWM mode 1 (makes sure output is waveform)
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload for CH1
    TIM3->CCER |= TIM_CCER_CC1E; // Enable CH1 output (PC6)
    TIM3->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
    TIM3->EGR = TIM_EGR_UG; // Generate update event
    TIM3->CR1 |= TIM_CR1_CEN; // Enable timer
}

void servo_angle_set(int angle) {
	pulse_width = 1500 - (500 * (angle/45.0)); // calculate pulse width for angle (-45 to 45 degrees) 500us per 45 degrees
	TIM3->CCR1 = pulse_width;
}

void TIM2_IRQHandler(void){
    if(TIM2->SR & TIM_SR_UIF){ //check if update interrupt flag is set
        TIM2->SR &= ~TIM_SR_UIF; //clear the interrupt flag
        if (pause == false){
        digitSelect = (digitSelect + 1) % 4; //cycle through digits 0-3
        time_ms ++;
        if (time_ms >= 10){
            time_ms = 0;
            time ++;
        }
    }
    int seconds = time / 1000;
    SSD_update(digitSelect, seconds, 3);
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

void check_distance(int sensor){
    read_IRSensor();
    switch(sensor){
        case 15:
            if(right_distance > left_distance){
                //turn right
                pulse_width_1 = 1510; 
                pulse_width_2 = 1430; 
               for(int i = 0; i < 2500000; i++); //delay about 0.2s
                pulse_width_1 = 1435.20; 
                pulse_width_2 = 1550; 
            }else{
                //turn left
                 pulse_width_1 = 1475.20; 
                pulse_width_2 = 1550;
               for(int i = 0; i < 2500000; i++); //delay about 0.2s
                pulse_width_1 = 1435.20; 
                pulse_width_2 = 1550; 
            }
            case 0:
                pulse_width_1 = 1500; 
                pulse_width_2 = 1500;
            break;
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
                phase = 1;
            break;
        case 14: // slight right
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
    if (phase == 0) {// Line following mode
        read_IRSensor();
        IR_cases(IRSensorReading);
    } else if (phase == 1) {
    servo_angle_set(45);
    right_distance = EchoWidth * 0.01715; //distance in cm
    servo_angle_set(-45);
    left_distance = EchoWidth * 0.01715; //distance in cm
    check_distance(IRSensorReading);
    }
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
    PWM_Output_PC6_Init();
    TIM2_init();
    TIM5_INIT();
    Sensor_Init();
	while (1) {
        if (phase == 1){
            GPIOA ->ODR |= (1 << TRIG_PIN); //set trigger high
            sysTickStart = TIM5->CNT;
            while((TIM5->CNT - sysTickStart) < 10);
                GPIOA->ODR &= ~(1 << TRIG_PIN); //set trigger low
   
            //send trigger pulse
            while(!(ECHO_PORT->IDR & (1<< ECHO_PIN))); //wait for echo to go high
                EchoStart = TIM5->CNT;
            while((ECHO_PORT->IDR & (1<< ECHO_PIN))); //wait for echo to go low or timeout
                EchoEnd = TIM5->CNT;

            //calculate echo width
            if(EchoEnd >= EchoStart){
                EchoWidth = EchoEnd - EchoStart;
            } else { //handle timer overflow
                EchoWidth = (0xFFFFFFFF - EchoStart) + EchoEnd;
            }
        }
	}
}*/