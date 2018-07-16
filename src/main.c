
#include <phot_meas.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "delay.h"
#include "uart.h"
#include "coil_driver.h"
#include "driver.h"


void main(void)
{
	SystemInit();
	DelayInit();

	//*** inicjalizacja uart ***
	IniteUART(230400);

	//*** inicjalizacja dac ***
	DAC1_Crtl_Init();
	//DAC2_Crtl_Init();

	//*** inicjalizacja adc_multi ***
	PHOT_Init();

	//*** inicjalizacja steper_motor ***
	DRV_Init();


	while(1)
	{
		DRV_Main();
	}
}



