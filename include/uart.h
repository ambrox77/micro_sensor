/*
 * uart.h
 *
 *  Created on: 27.06.2018
 *      Author: MCV
 */

#ifndef UART_TXT_
#define UART_TXT_



#include "stdio.h"


void IniteUART(uint32_t baudrate);
void Send_Char(char);
void USART_Send(volatile char *s);
void USART_Send_Int(int16_t value);
void USART_Send_String(char * txt);
void USART_Send_MultiInt(int16_t value1, int16_t value2, int16_t value3, int16_t value4);
//void USART_Send_Doable(float d_value);




#endif /* UART_TXT_ */
