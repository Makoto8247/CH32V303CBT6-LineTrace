/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 USART Print debugging routine:
 USART1_Tx(PA9).
 This example demonstrates using USART1(PA9) as a print debug port output.

*/

#include "debug.h"



void GPIO_Sencer_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin =
            GPIO_Pin_0 |
            GPIO_Pin_1 |
            GPIO_Pin_2 |
            GPIO_Pin_3 |
            GPIO_Pin_4 |
            GPIO_Pin_5 |
            GPIO_Pin_6 |
            GPIO_Pin_7;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

#define LED_PIN 0x2000
void GPIO_LED_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/************** Motor Pin **********************
 * TB6612FNG
 * MAX IOut : 1A
 * MOTOR1 == MOTOR2 / 3
 *
 * STBY -> GPIOA0
 * PWMA -> GPIOA1
 * PWMB -> GPIOA2
 * AIN1 -> GPIOA3
 * AIN2 -> GPIOA4
 * BIN1 -> GPIOA5
 * BIN2 -> GPIOA6
 */

#define STBY_PIN    0x0001
#define PWMA_PIN    0x0002
#define PWMB_PIN    0x0004
#define AIN1_PIN    0x0008
#define AIN2_PIN    0x0010
#define BIN1_PIN    0x0020
#define BIN2_PIN    0x0040
#define MOTOR_ARR   1440 - 1

void GPIO_Motor_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Output
    GPIO_InitStructure.GPIO_Pin =
            STBY_PIN |
            AIN1_PIN |
            AIN2_PIN |
            BIN1_PIN |
            BIN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PWM setting
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =
            PWMA_PIN |
            PWMB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = MOTOR_ARR; // 100kHz
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 720 - 1; // 50%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);


    GPIO_SetBits(GPIOA, STBY_PIN);
}




void Motor_Forward()
{
    GPIOA -> BSHR =
            STBY_PIN |
            AIN2_PIN |
            BIN2_PIN ;
    GPIOA -> BCR  =
            AIN1_PIN |
            BIN1_PIN;

}

void Motor_Back()
{
    GPIOA -> BSHR =
            STBY_PIN |
            AIN1_PIN |
            BIN1_PIN |
            PWMA_PIN |
            PWMB_PIN;
    GPIOA -> BCR  =
            AIN2_PIN |
            BIN2_PIN;
}

void Motor_Brake()
{
    GPIOA -> BSHR =
                STBY_PIN |
                AIN1_PIN |
                AIN2_PIN |
                BIN1_PIN |
                BIN2_PIN ;
}

/************** Line Angle **********************
 *
 * This function expresses the degree of curvature of the line in degrees.
 * Returns a value of -1 ~ 180, with 0 representing the left, and 180 to the right.
 * -1 indicates that all sensors are responding.
 *
 */

#define IS_0SENCER  0b00000001 // LEFT
#define IS_1SENCER  0b00000010
#define IS_2SENCER  0b00000100
#define IS_3SENCER  0b00001000
#define IS_4SENCER  0b00010000
#define IS_5SENCER  0b00100000
#define IS_6SENCER  0b01000000
#define IS_7SENCER  0b10000000 // RIGHT
int16_t Line_Angle()
{
    uint8_t getSencer = ~(GPIO_ReadInputData(GPIOB) & 0x00FF);
    printf("getSencer : %x\r\n", getSencer);
    if(getSencer == 0x00) return -1;
    if(getSencer == 0xFF) return 90;
    if((getSencer & IS_7SENCER) == IS_7SENCER) return 180;
    if((getSencer & IS_0SENCER) == IS_0SENCER) return 0;
    if((getSencer & IS_6SENCER) == IS_6SENCER) return 150;
    if((getSencer & IS_1SENCER) == IS_1SENCER) return 30;
    if((getSencer & IS_5SENCER) == IS_5SENCER) return 120;
    if((getSencer & IS_2SENCER) == IS_2SENCER) return 60;
    if((getSencer & IS_4SENCER) == IS_4SENCER) return 100;
    if((getSencer & IS_3SENCER) == IS_3SENCER) return 80;
    return 90;
}

#define RIGHT   1
#define LEFT    0
#define SCALE_FACTOR    100
#define PWM_PULSE_MAX   (MOTOR_ARR * SCALE_FACTOR) * 80 / SCALE_FACTOR

uint16_t Angle_To_Pulse(int16_t angle, uint8_t dir) {
    if(angle < 0) angle = 90;
    uint16_t pulse = (angle * PWM_PULSE_MAX * SCALE_FACTOR) / (180 * SCALE_FACTOR);
    if(dir == RIGHT) pulse = PWM_PULSE_MAX - pulse;

    printf("pulse : %d \r\n", pulse);
    return pulse;
}

void Motor_Pulse(uint16_t motor1_pulse, uint16_t motor2_pulse) {
    TIM_SetCompare2(
            TIM2,
            motor2_pulse
    );
    TIM_SetCompare3(
            TIM2,
            motor1_pulse
    );
}


int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

	GPIO_Sencer_INIT();
	GPIO_Motor_INIT();
	GPIO_LED_INIT();

	int16_t angle;
	uint16_t pulse_r;
	uint16_t pulse_l;

	while(1)
    {
	    GPIO_SetBits(GPIOC, LED_PIN);

	    angle = Line_Angle();
	    pulse_r = Angle_To_Pulse(angle, RIGHT);
	    pulse_l = Angle_To_Pulse(angle, LEFT);

	    printf("Line Angle: %d\r\n", angle);

	    Motor_Pulse(pulse_l, pulse_r);
	    if(angle < 0) {
	        Motor_Brake();
	    }
	    else Motor_Forward();

	    Delay_Ms(20);
	}
}

