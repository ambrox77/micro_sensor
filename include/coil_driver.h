/*
 * i2c_fun.h
 *
 *  Created on: 27.06.2018
 *      Author: MCV
 */

#ifndef I2C_FUN_TXT_
#define I2C_FUN_TXT_


#include "stm32f4xx.h"



uint8_t DAC_SetVoltage(I2C_TypeDef* I2Cx,uint16_t volt);
void DAC1_Crtl_Init(void);
void DAC2_Crtl_Init(void);






#endif /* I2C_FUN_TXT_ */
