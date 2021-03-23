/* This code toggles the onboard led when an external button is pressed (Button is connected in PA10)
 *
 *
 *  Created on: Feb 7, 2021
 *      Author: Eduardo Garcia
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "stm32f103.h"
#include "stm32f103_gpio_driver.h"
#define		GPIOC_IDR		(APB2_BASEADDR + 0x1000 + 0x0008)

void delay(void){
	for(uint32_t i = 0; i < 100000; i++);
}

int main(void)
{
    /* Configura el GPIO asociado al LED */
	GPIO_Handle_t	gpioled;
	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.PinMode = GPIO_MODE_PP;
	gpioled.GPIO_PinConfig.PinNumber = 5;
	gpioled.GPIO_PinConfig.PinSpeed = GPIO_MODE_OUT_LS;
	GPIO_PeriCLKControl(GPIOA,ENABLE);
	GPIO_Init(&gpioled);

	/* Configura el GPIO asociado al Boton*/
	GPIO_Handle_t	gpiobtn;
	gpiobtn.pGPIOx = GPIOC;
	gpiobtn.GPIO_PinConfig.PinMode = GPIO_MODE_IN_PU_PD;
	gpiobtn.GPIO_PinConfig.PinNumber = 13;
	gpiobtn.GPIO_PinConfig.PinSpeed = GPIO_MODE_INPUT;
	gpiobtn.GPIO_PinConfig.PinResistor = GPIO_PU;       /*Activando Pull up resistor*/
	GPIO_PeriCLKControl(GPIOC,ENABLE);
	GPIO_Init(&gpiobtn);

	uint32_t boton = 0;
	volatile uint32_t *Puerto_C = (uint32_t*) GPIOC_IDR;

	while(1){
		while(1){
			boton = ((*Puerto_C)>>13)&1;
			if ( !boton ) break;
		}

		GPIO_WriteToOutputPin(GPIOA, 5, 1);
		delay();
	}

	return 0;
}
