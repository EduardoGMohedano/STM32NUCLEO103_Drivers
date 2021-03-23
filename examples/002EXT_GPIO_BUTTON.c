/* This code toggles the onboard led when an external button is pressed (Button is connected in PA10)
 * GPIO_external_btn.c
 *
 *  Created on: Dec 1, 2020
 *      Author: egarc
 */


#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "stm32f103.h"
#include "stm32f103_gpio_driver.h"


void delay(void){
	for(uint32_t i = 0; i < 100000; i++);
}

int main(void)
{
    /* Led config */
	GPIO_Handle_t	gpioled;
	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.PinMode = GPIO_MODE_PP;
	gpioled.GPIO_PinConfig.PinNumber = 5;
	gpioled.GPIO_PinConfig.PinSpeed = GPIO_MODE_OUT_LS;
	GPIO_PeriCLKControl(GPIOA,ENABLE);
	GPIO_Init(&gpioled);

	/* Button config */
	GPIO_Handle_t	gpiobtn;
	gpiobtn.pGPIOx = GPIOA;
	gpiobtn.GPIO_PinConfig.PinMode = GPIO_MODE_IN_PU_PD;
	gpiobtn.GPIO_PinConfig.PinNumber = 10;
	gpiobtn.GPIO_PinConfig.PinSpeed = GPIO_MODE_INPUT;
	gpiobtn.GPIO_PinConfig.PinResistor = GPIO_PU;       /*Pull up resistor activated*/
	GPIO_Init(&gpiobtn);



	while(1){
		//GPIO_WriteToOutputPin(GPIOA, 5, 1);
		if( GPIO_ReadFromInputPin(GPIOA, 10) == 0x00 ){
			delay();
			GPIO_ToggleOutputPin(GPIOA, 5);

		}

	}
}
