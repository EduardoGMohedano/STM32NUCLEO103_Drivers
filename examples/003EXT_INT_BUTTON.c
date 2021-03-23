/*
 * 003EXT_INT_BUTTON.c
 * The following code makes onboard led to blink using an external button (an interruption is used) connected in PA10
 * EXTERNAL_INT_BTN.c
 *
 *  Created on: 6 dic. 2020
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

void EXTI15_10_IRQHandler(void){
	GPIO_IRQHandling(10);
	GPIO_ToggleOutputPin(GPIOA, 5);
	delay();

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

	/*External interrupt configuration*/
	GPIO_IRQ_Mode(GPIOA, 10,FALLING_EDGE);
	GPIO_IRQConfig(IRQ_NO_EXTI15_10,1,ENABLE);

	while(1){

	}
}
