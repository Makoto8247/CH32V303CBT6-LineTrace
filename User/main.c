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
            GPIO_Pin_0 ||
            GPIO_Pin_1 ||
            GPIO_Pin_2 ||
            GPIO_Pin_3 ||
            GPIO_Pin_4 ||
            GPIO_Pin_5 ||
            GPIO_Pin_6 ||
            GPIO_Pin_7;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/************** Motor Pin **********************
 * MODE -> GPIOA0
 * ERR  -> GPIOA1 (INPUT) // H = NoErr, L = Err
 * IN1B -> GPIOA2
 * IN2B -> GPIOA3
 * IN1A -> GPIOA4
 * IN2A -> GPIOA5
 * STBY -> GPIOA6
 *
 */

#define MODE_PIN    0x0001
#define ERR_PIN     0x0002
#define IN1B_PIN    0x0004
#define IN2B_PIN    0x0008
#define IN1A_PIN    0x0010
#define IN2A_PIN    0x0020
#define STBY_PIN    0x0040

void GPIO_Motor_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Output
    GPIO_InitStructure.GPIO_Pin =
            MODE_PIN ||
            IN1B_PIN ||
            IN2B_PIN ||
            IN1A_PIN ||
            IN2A_PIN ||
            STBY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Input
    GPIO_InitStructure.GPIO_Pin = ERR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Mode -> IN
    GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
}

void Motor_Forward()
{
    GPIO_SetBits(GPIOA, IN1A_PIN || IN1B_PIN);
    GPIO_ResetBits(GPIOA, IN2A_PIN || IN2B_PIN);
}

void Motor_Brake()
{
    GPIO_SetBits(GPIOA,
            IN1A_PIN ||
            IN1B_PIN ||
            IN2A_PIN ||
            IN2B_PIN
    );
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);	
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
	printf("This is printf example\r\n");

	GPIO_Sencer_INIT();
	GPIO_Motor_INIT();

	while(1)
    {
	    // Skip if there is an error
	    if(!GPIO_ReadInputDataBit(GPIOA, ERR_PIN))
	    {
	        GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
	        continue;
	    }

	    GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_SET);

	    Motor_Forward();

	}
}

