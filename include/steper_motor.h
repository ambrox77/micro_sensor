/*
 * steper_motor.h
 *
 *  Created on: 12.07.2018
 *      Author: MCV
 */

#ifndef STEPER_MOTOR_H_
#define STEPER_MOTOR_H_

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "delay.h"
#include "diag/Trace.h"
#include "stm32f4xx_tim.h"
#include "stdio.h"


#define DIR GPIO_Pin_0	//GPIOE
#define STEP GPIO_Pin_1
#define EN GPIO_Pin_2
#define MS1 GPIO_Pin_3
#define MS2 GPIO_Pin_4
#define MS3 GPIO_Pin_5
#define BUTTON GPIO_Pin_0 //GPIOA

#define TIM_PERIOD 72
#define TIM_PRESC 1000

#define TRUE 1
#define FALSE 0


void TM_PIN_Init(void);
void PIN_LOG_Inite(void);
void TIMER_Init(void);
void TIM3_IRQHandler(void);

uint16_t step_count=0;
uint8_t enable=0;

void PIN_LOG_Inite(void){
	GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Set pins */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

    //Sterowanie pó³nokrokowe (1/2):
    GPIO_SetBits(GPIOE, EN);
    GPIO_ResetBits(GPIOE, MS1| MS2 | MS3);
    GPIO_SetBits(GPIOE, MS1);
    //GPIO_ResetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);

    GPIO_InitStruct.GPIO_Pin = BUTTON;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void TM_PIN_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);


    /* Set pins */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Alternating functions for pins */
   // GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_TIM1);



}

void TIMER_Init(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;

    // TIM4 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//Compute the prescaler value
	//PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 21000000) -1;


	TIM_Cmd(TIM3, DISABLE);
	// Time Base Configuration
	TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD; // 1MHz
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_PRESC; // 1MHz/1k = 1ms
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// PWM1 Mode configuration: Channel1
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM_PERIOD/2; //duty cycle = 50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	TIM_ARRPreloadConfig(TIM3, ENABLE);


	// TIM4 enable counter
	TIM_Cmd(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	//Initialize NVIC
	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStruct);


	//Enable RX interrupt
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

//obs³uga przerwania portu szeregowego USART3
void TIM3_IRQHandler(void)
{
	 if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	    {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		if(enable==1)GPIO_ToggleBits(GPIOE, GPIO_Pin_1);
		if(step_count<0xffff)step_count++;
	    }
}


#endif /* STEPER_MOTOR_H_ */
