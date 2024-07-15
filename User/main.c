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
            BIN2_PIN |
            PWMA_PIN |
            PWMB_PIN;
    GPIOA -> BCR  =
            AIN1_PIN |
            BIN1_PIN;

}

void Motor_Back()
{
    GPIO_SetBits(GPIOA, BIN1_PIN || AIN1_PIN);
    GPIO_ResetBits(GPIOA, AIN2_PIN || PWMB_PIN);
}

void Motor_Right()
{
    GPIO_SetBits(GPIOA, AIN2_PIN || BIN1_PIN || PWMB_PIN);
    GPIO_ResetBits(GPIOA, AIN1_PIN);
}

void Motor_Left()
{
    GPIO_SetBits(GPIOA, AIN2_PIN || PWMB_PIN || AIN1_PIN);
    GPIO_ResetBits(GPIOA, BIN1_PIN);
}

void Motor_TurnRight()
{
    GPIO_SetBits(GPIOA, BIN1_PIN || PWMB_PIN);
    GPIO_ResetBits(GPIOA, AIN2_PIN || AIN1_PIN);
}

void Motor_TurnLeft()
{
    GPIO_SetBits(GPIOA, AIN2_PIN || AIN1_PIN);
    GPIO_ResetBits(GPIOA, BIN1_PIN || PWMB_PIN);
}

void Motor_Brake()
{
    GPIOA -> BSHR =
                STBY_PIN |
                AIN1_PIN |
                AIN2_PIN |
                BIN1_PIN |
                BIN2_PIN |
                PWMA_PIN |
                PWMB_PIN;
}

/************** Line Angle **********************
 *
 * This function expresses the degree of curvature of the line in degrees.
 * Returns a value of -1 ~ 180, with 0 representing the left, and 180 to the right.
 * -1 indicates that all sensors are responding.
 *
 */

int Line_Angle()
{
    uint8_t getSencer = GPIO_ReadInputData(GPIOB) & 0x00FF;
    switch(getSencer)
    {
        case 0b00000000:
        case 0b00011000:
            return 90;
            break;
        // Left
        case 0b00010000:
            return 85;
            break;
        case 0b00110000:
        case 0b11110000:
            return 0;
            break;
        // Right
        case 0b00001000:
            return 95;
            break;
        case 0b00001111:
            return 180;
            break;
        default:
            return -1;
            break;
    }

}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();

	GPIO_Sencer_INIT();
	GPIO_Motor_INIT();
	GPIO_LED_INIT();

	while(1)
    {

	    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0) Motor_Forward();
	    else Motor_Brake();

	}
}

