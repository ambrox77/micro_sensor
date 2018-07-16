/*
 * uart.c
 *
 *  Created on: 15.07.2018
 *      Author: MCV
 */

#include <uart.h>

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "string.h"
#include "stm32f4xx_conf.h"

#define ROZMIAR_BUF_RX_TX 128

char bufor_rx_USART2[ROZMIAR_BUF_RX_TX];  //bufor odbiorczy USARTA1
char bufor_tx_USART2[ROZMIAR_BUF_RX_TX];  //bufor transmisji USART1
//wskaŸnik do pocz¹tku bufora odbiorczego USART1
char *p_bufor_rx_USART2;
//bajt okreœlaj¹cy ca³kowity rozmiar bufora odbiorczego USART1
int  bufor_rx_USART2_rozmiar;
/* wskaŸnik bie¿¹cej pozycji w buf.odbiorczym dla danych zapisywanych w buf. Odpowiednik *p_WE */
char *p_rx_USART2_in;
/* wskaŸnik bie¿¹cej pozycji w buf.odbiorczym dla danych odczytywanych z buf. Odpowiednik *p_WY */
char *p_rx_USART2_out;
char *p_bufor_tx_USART2;  //wskaŸnik do pocz¹tku buforu transmisji USART1
int  bufor_tx_USART2_rozmiar;  //bajt okreœlaj¹cy ca³kowity rozmiar bufora transmisji UART1
/* wskaŸnik bie¿¹cej pozycji w buf.nadawczym dla danych zapisywanych w buf. Odpowiednik *p_WE */
char *p_tx_USART2_in;
/* wskaŸnik bie¿¹cej pozycji w buf.nadawczym dla danych odczytywanych z buf. Odpowiednik *p_WY */
char *p_tx_USART2_out;


//void USART_Send_Doable(float d_value){
//	//char bufor[100];
//	uint8_t size = sizeof(double);
//	char size_char[size];
//	//uint16_t value = snpintf(size_char, size, "%f", d_value);
//	//printf("i = %d\)
//	USART_Send_Int(value);
//}


void USART_Send_String(char * txt)
    {
        while( *txt ) Send_Char(*txt++);
    }

void USART_Send_Int(int16_t value)
    {
        char bufor[100];
        USART_Send_String( itoa(value, bufor, 10) );
    }
void USART_Send_MultiInt(int16_t value1, int16_t value2, int16_t value3, int16_t value4)
    {
        char bufor[100];
        USART_Send_String( itoa(value1, bufor, 10) ), USART_Send("\t");
        USART_Send_String( itoa(value2, bufor, 10) ), USART_Send("\t");
        USART_Send_String( itoa(value3, bufor, 10) ), USART_Send("\t");
        USART_Send_String( itoa(value4, bufor, 10) ), USART_Send("\n");
    }

void USART_Send(volatile char *s)
{

	while(*s){
		// wait until data register is empty
		while( !(USART2->SR & 0x00000040) );
		USART_SendData(USART2, *s);
		s++;
	}
}

void Send_Char(char c)
 {
    //Sprawdza czy bufor nadawczy jest pusty
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
 }


void IniteUART(uint32_t baudrate){
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	GPIO_InitTypeDef 	GPIO_InitStruct;

	//Enable clock for USART2 peripheral
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	// Enable clock for GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// Initialize pins as alternating function
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/**
	 * Set Baudrate to value you pass to function
	 * Disable Hardware Flow control
	 * Set Mode To TX and RX, so USART will work in full-duplex mode
	 * Disable parity bit
	 * Set 1 stop bit
	 * Set Data bits to 8
	 *
	 * Initialize USART2
	 * Activate USART2
	 */
	USART_StructInit(&USART_InitStruct);
	USART_InitStruct.USART_BaudRate = baudrate;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStruct);
	USART_Cmd(USART2, ENABLE);

	/**
	 * Set Channel to USART2
	 * Set Channel Cmd to enable. That will enable USART2 channel in NVIC
	 * Set Both priorities to 0. This means high priority
	 *
	 * Initialize NVIC
	 */
	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);


	//Enable RX interrupt
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

}

//obs³uga przerwania portu szeregowego USART2
void USART_2_IRQHandler(void)
{
//obs³uga przerwania USART2
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
    	//przepisanie odebranego bajtu z rejestru USART2 do buforu ko³owego
        *p_rx_USART2_in = USART_ReceiveData(USART2);
        p_rx_USART2_in++;
        if (p_rx_USART2_in >=(p_bufor_rx_USART2 +bufor_rx_USART2_rozmiar))
        {
        	//przewiniêcie wskaŸnika na pocz¹tek
            p_rx_USART2_in =p_bufor_rx_USART2;
        }
    }
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {
    	//przepisanie kolejnego bajtu do wys³ania z bufora ko³owego do rejestru USART1
        if (p_tx_USART2_in != p_tx_USART2_out)
        {
        	//w buforze transmisji s¹ znaki do wys³ania uart-em
            USART_SendData(USART2, *p_tx_USART2_out);
            p_tx_USART2_out++;
            if (p_tx_USART2_out >= (p_bufor_tx_USART2 + bufor_tx_USART2_rozmiar))
            {
                p_tx_USART2_out =p_bufor_tx_USART2;
            }
        }
        else
        {
        	//ca³a zawartoœæ buforu ko³owego zosta³a wys³ana, wy³¹czenie zezwolenia na przerwanie transmisji
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);// disable tx interrupt
        }
    }
}
