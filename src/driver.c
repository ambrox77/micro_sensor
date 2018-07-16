/*
 * driver.c
 *
 *  Created on: 16.07.2018
 *      Author: MCV
 */


#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "delay.h"
#include "stdio.h"
#include "driver.h"
#include "phot_meas.h"
#include <math.h>
#include "coil_driver.h"
#include "uart.h"

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

#define DRV_USTEP 20 // 100 microns / 5 - dla 200 imp / obrot


typedef struct{
	uint16_t coil_pos;
	uint16_t step_pos;
	uint16_t step_cnt;
	uint16_t ustep;
}drv_data_T;


float Sine12bit_B[4096]={0};
uint16_t Triangular[4096]={0};

drv_data_T drv_data;

static void DRV_SendData(void);
static uint8_t DRV_Meas_OLD(void);
static void DRV_Meas_NEW(uint8_t licz_probki, uint8_t dokladnosc_pom);
static void DRV_Step(uint16_t delay);
static void DRV_StepperInit(void);
static void DRV_TimerInit(void);
void TIM3_IRQHandler(void);

uint16_t step_count=0;
uint8_t enable=0;

//--------------------- External functions ----------------------------------------------------------------//
void DRV_Init(void)
{
	uint16_t probka=0,point=0;

	DRV_StepperInit();
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED4);

	for(uint16_t p=0; p<4096; p++){
		Sine12bit_B[p] = round(0.75*(2048.0*sin((p*2*M_PI)/4095.0)+2048.0));
	}

	for(uint16_t p=0; p<4096; p++){
		Triangular[p] = probka++;
		if(probka==2048){
			do {
				Triangular[p] = 0.75*(probka--)+500.0;
			}while(probka != 0);
		}
	}


	while(1){
		Delay_ms(1);
		DAC_SetVoltage(I2C2,Sine12bit_B[point++]);
		if(point == 4095) point=0;
	}
	STM_EVAL_LEDToggle(LED5);

	//DAC_SetVoltage(I2C1,2000);

	// Zatrzymanie programu po resecie a¿ do nacisniecia przycisku
	while(GPIO_ReadInputDataBit(GPIOA, BUTTON) == 0);


	// Enable dla silnika:
	GPIO_ResetBits(GPIOE, DIR);
	GPIO_ResetBits(GPIOE, MS1 | MS2 | MS3);
	GPIO_ResetBits(GPIOE, EN); //zezwolenie pracy
}
void DRV_Main(void)
{
	STM_EVAL_LEDToggle(LED4);


	if(drv_data.step_cnt == 15) 	//1.5mm
	{
		GPIO_ToggleBits(GPIOE, DIR);
	}


	DRV_Step(400);
	DRV_Meas_OLD();

	//DRV_Meas_NEW(2, 2);
	DRV_SendData();
}

//--------------------------- Internal: --------------------------------------------------------------//
static void DRV_Step(uint16_t delay)
{

	for(uint32_t i=0;i<DRV_USTEP;i++){	// 200uKrokow/obr
				GPIO_ToggleBits(GPIOE, STEP);
				Delay_ms(1);
				GPIO_ToggleBits(GPIOE, STEP);
				Delay_ms(1);
	}

	drv_data.step_cnt++;

	Delay_ms(delay);
}

static uint8_t DRV_Meas_OLD(void)
{

	phot_measures_T pomiary;
	uint16_t delta=0,sumaAC=0,sumaBD=0,porow=0,flaga=0;

	uint16_t drv_meas_cnt=0;

	// inkrementuje tablice i sprawdza kiedy jest w ognisku poczym pozostaje w nim
	do {
			STM_EVAL_LEDToggle(LED6);

			PHOT_GetMeas(&pomiary);
			sumaAC = pomiary.photo_A+pomiary.photo_C;
			sumaBD = pomiary.photo_B+pomiary.photo_D;
			delta = abs(sumaAC-sumaBD);

//			USART_Send_Int(sumaAC);
//			USART_Send_String("  ");
//			USART_Send_Int(sumaBD);
//			USART_Send_String("  ");
			USART_Send_Int(delta);
			USART_Send_String(" \n\r ");
			if(delta <= 5){
				USART_Send_String("A \n\r ");
				//if (porow != Sine12bit_B[drv_meas_cnt]){ 	//sprawdz czy probka jest ró¿na z poprzedni¹

					drv_data.coil_pos = Sine12bit_B[drv_meas_cnt];			// sumaAC - sumaBD
					drv_data.step_pos = drv_data.step_cnt*DRV_USTEP;		// rozdzielczosc pomiarowa
				//	return 1;
				//}
				//porow = Sine12bit_B[drv_meas_cnt];
				flaga=1;

			}
			else if (sumaAC > sumaBD){
				USART_Send_Int(drv_meas_cnt);
				USART_Send_String("B \n\r ");
				DAC_SetVoltage(I2C1,Sine12bit_B[drv_meas_cnt++]);
				if(drv_meas_cnt==4095)drv_meas_cnt=0;
			}

			else if (sumaAC < sumaBD){
				USART_Send_Int(drv_meas_cnt);
				USART_Send_String("C \n\r ");
				DAC_SetVoltage(I2C1,Sine12bit_B[drv_meas_cnt++]);
				if(drv_meas_cnt==4095)drv_meas_cnt=0;
			}
			Delay_ms(1);
	} while(flaga == 0);
	flaga=0;


	return 0;
}

//static void DRV_Meas_NEW(uint8_t licz_probki, uint8_t dokladnosc_pom)
//{
//	phot_measures_T pomiary;
//	// inkrementuje tablice i sprawdza kiedy jest w ognisku poczym pozostaje w nim
//	 	 while(1){
//			uint16_t sumaAC = pomiary.photo_A+pomiary.photo_C;
//			uint16_t sumaBD = pomiary.photo_B+pomiary.photo_D;
//			if ((sumaAC > sumaBD) && (sumaAC-sumaBD > dokladnosc_pom)){
//					STM_EVAL_LEDOn(LED5);
//					STM_EVAL_LEDOff(LED6);
//					if(i==4095)i=0;
//					DAC_SetVoltage(I2C1,Sine12bit_B[i++]);
//			}
//			else if(sumaAC-sumaBD <= dokladnosc_pom){
//				STM_EVAL_LEDToggle(LED4);
//				if (porow == Sine12bit_B[i]) licznik++; //sprawdz czy probka jest rowna z poprzedni¹ i zwiêksz licznik
//				porow = Sine12bit_B[i];
//
//				if (licznik+1 == licz_probki){
//
//					bufor[0] = Sine12bit_B[i];			// sumaAC - sumaBD
//					bufor[1] = krok_pomiar*uK;		// rozdzielczosc pomiarowa
//					licznik = 0;
//					return;
//				}
//
//
//				DAC_SetVoltage(I2C1,Sine12bit_B[i++]);
//			}
//			else if ((sumaAC < sumaBD) && (sumaAC-sumaBD > dokladnosc_pom)){
//				STM_EVAL_LEDOn(LED6);
//				STM_EVAL_LEDOff(LED5);
//				if(i==0)i=64;
//				DAC_SetVoltage(I2C1,Sine12bit_B[i--]);
//			}
//
//		}
//}

static void DRV_SendData(void)
{
	char buf[30];
	sprintf(buf,"%d,%d,%d\n\r",drv_data.step_cnt,drv_data.coil_pos,drv_data.step_pos);
	//sprintf(buf,"Raw: %d, Odl: %d.%dmm Krok: %d\n\r",bufor[0],bufor[1]/100,bufor[1]%100,krok_pomiar);
	USART_Send(buf);
}

//------------ Silnik krokowy -----------------------------------------------------------------------------//

static void DRV_StepperInit(void){
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


    //
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}


static void DRV_TimerInit(void){
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
