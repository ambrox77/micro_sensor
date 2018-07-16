#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "delay.h"
#include "diag/Trace.h"
#include "stm32f4xx_i2c.h"
#include "math.h"
#include "uart.h"
#include "multi_adc.h"
#include "i2c_fun.h"
#include "steper_motor.h"

typedef enum{
	A = 0,
	C = 1,
	B = 2,
	D = 3
}fotodiode;

#define CORE_I2C_ADDRESS 0x00
#define CODEC_I2C_ADDRESS (0x60 << 1)

#define I2C_SCL_PIN		GPIO_Pin_10  //port B
#define I2C_SDA_PIN		GPIO_Pin_11  //port B
#define CODEC_I2C I2C1


uint16_t krok_pomiar = 0;
volatile uint32_t wartosc = 0xffff;
volatile int32_t zmienna = 0xffff;
uint16_t porow = 0;
uint8_t flaga = 0;
uint8_t licznik = 0;

static uint16_t i = 0;
float Sine12bit_B[4096]={0};
uint16_t Triangular[4096]={0};
uint16_t probka;
uint32_t uK;
uint16_t bufor[10];
uint16_t step_um;

extern uint16_t ADC1ConvertedValue[5];//Stores converted vals

void step( uint16_t step_um, uint16_t delay);
void meas(void);
void Send_meas(void);
void meas_probki(uint8_t licz_probki, uint8_t dokladnosc_pom);

void main(void)
{
	SystemInit();
	DelayInit();
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED4);



	for(uint16_t p=0; p<4096; p++){
		Sine12bit_B[p] = round(0.75*(2048*sin(((probka++)*2*M_PI)/4095)+2048)+500);
		if(probka==4095) probka=0;
	}

	for(uint16_t p=0; p<4096; p++){
		Triangular[p] = probka++;
		if(probka==2048){
			do {
				Triangular[p] = 0.75*(probka--)+500;
			}while(probka != 0);
		}
	}

	//*** inicjalizacja uart ***
	IniteUART(230400);

	//*** inicjalizacja dac ***
	DAC1_Crtl_Init();
	DAC2_Crtl_Init();

	//*** inicjalizacja adc_multi ***
	IniteADC_multi();

	//*** inicjalizacja steper_motor ***
	PIN_LOG_Inite();
	GPIO_ResetBits(GPIOE, DIR);

//	DAC_SetVoltage(I2C2,1600);
//	while(1){
//		Delay_ms(10);
//		DAC_SetVoltage(I2C1,Sine12bit_B[i++]);
//	}

	do{
//		DAC_SetVoltage(I2C2,Sine12bit_B[i++]);
//		if(i==4095)i=0;
		//Delay_ms(2);
		wartosc = (ADC1ConvertedValue[A]+ADC1ConvertedValue[C])-(ADC1ConvertedValue[B]+ADC1ConvertedValue[D]);
		if (porow != wartosc){
			USART_Send_Int(Sine12bit_B[i]), USART_Send("\n\r");
			return;
		}
		porow = wartosc;
	} while(wartosc > 2);

	while(GPIO_ReadInputDataBit(GPIOA, BUTTON) == 0);
	STM_EVAL_LEDOn(LED4);

	while(1)
	{
		GPIO_ResetBits(GPIOE, MS1 | MS2 | MS3);
		GPIO_ResetBits(GPIOE, EN); //zezwolenie pracy


		if(krok_pomiar == 15) 	//1.5mm
		{
			GPIO_ToggleBits(GPIOE, DIR);
			bufor[0] = 0;
		}

		step(100,600); 		// krok co 100um
		meas();
		//meas_probki(2, 2);
		Send_meas();

		//USART_Send_MultiInt(ADC1ConvertedValue[A], ADC1ConvertedValue[B], ADC1ConvertedValue[C], ADC1ConvertedValue[D]);
		//USART_Send_Int(sumaAC);
		//Delay_ms(2);

	}
}

void step( uint16_t step_um, uint16_t delay){

	uK = step_um/5; // 0.01mm
	for(uint32_t i=0;i<uK;i++){		// 200uKrokow/obr
				GPIO_ToggleBits(GPIOE, STEP);
				Delay_ms(1);
				GPIO_ToggleBits(GPIOE, STEP);
				Delay_ms(1);
	}
	krok_pomiar++;
	Delay_ms(delay);
}

void meas(void){
	// inkrementuje tablice i sprawdza kiedy jest w ognisku poczym pozostaje w nim
	do {
			zmienna = (ADC1ConvertedValue[A]+ADC1ConvertedValue[C])-(ADC1ConvertedValue[B]+ADC1ConvertedValue[D]);
			uint16_t sumaAC = ADC1ConvertedValue[A]+ADC1ConvertedValue[C];
			uint16_t sumaBD = ADC1ConvertedValue[B]+ADC1ConvertedValue[D];
			if ((sumaAC > sumaBD) && (zmienna>3)){
					STM_EVAL_LEDOn(LED5);
					STM_EVAL_LEDOff(LED6);
					if(i==4095)i=0;
					DAC_SetVoltage(I2C1,Sine12bit_B[i++]);
			}
			else if(zmienna <= 3){
				STM_EVAL_LEDToggle(LED4);
				if (porow != Sine12bit_B[i]){ 	//sprawdz czy probka jest ró¿na z poprzedni¹

					bufor[0] = Sine12bit_B[i];			// sumaAC - sumaBD
					bufor[1] = krok_pomiar*uK;		// rozdzielczosc pomiarowa
					return;
				}
				porow = Sine12bit_B[i];
				flaga++;

			}
			else if ((sumaAC < sumaBD) && (zmienna>3)){
				STM_EVAL_LEDOn(LED6);
				STM_EVAL_LEDOff(LED5);
				if(i==0)i=64;
				DAC_SetVoltage(I2C1,Sine12bit_B[i--]);
			}

		} while((flaga%2) == FALSE);
}

void meas_probki(uint8_t licz_probki, uint8_t dokladnosc_pom){
	// inkrementuje tablice i sprawdza kiedy jest w ognisku poczym pozostaje w nim
	 	 while(1){
			uint16_t sumaAC = ADC1ConvertedValue[A]+ADC1ConvertedValue[C];
			uint16_t sumaBD = ADC1ConvertedValue[B]+ADC1ConvertedValue[D];
			if ((sumaAC > sumaBD) && (sumaAC-sumaBD > dokladnosc_pom)){
					STM_EVAL_LEDOn(LED5);
					STM_EVAL_LEDOff(LED6);
					if(i==4095)i=0;
					DAC_SetVoltage(I2C1,Sine12bit_B[i++]);
			}
			else if(sumaAC-sumaBD <= dokladnosc_pom){
				STM_EVAL_LEDToggle(LED4);
				if (porow == Sine12bit_B[i]) licznik++; //sprawdz czy probka jest rowna z poprzedni¹ i zwiêksz licznik
				porow = Sine12bit_B[i];

				if (licznik+1 == licz_probki){

					bufor[0] = Sine12bit_B[i];			// sumaAC - sumaBD
					bufor[1] = krok_pomiar*uK;		// rozdzielczosc pomiarowa
					licznik = 0;
					return;
				}


				DAC_SetVoltage(I2C1,Sine12bit_B[i++]);
			}
			else if ((sumaAC < sumaBD) && (sumaAC-sumaBD > dokladnosc_pom)){
				STM_EVAL_LEDOn(LED6);
				STM_EVAL_LEDOff(LED5);
				if(i==0)i=64;
				DAC_SetVoltage(I2C1,Sine12bit_B[i--]);
			}

		}
}

void Send_meas(void){
	char buf[30];
	sprintf(buf,"%d,%d,%d\n\r",bufor[0],bufor[1],krok_pomiar);
	//sprintf(buf,"Raw: %d, Odl: %d.%dmm Krok: %d\n\r",bufor[0],bufor[1]/100,bufor[1]%100,krok_pomiar);
	USART_Send(buf);
}

