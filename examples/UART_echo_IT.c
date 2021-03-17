/* This code is aimed to perform an echo effect by sending data back of already received data
 * through UART1 by using an Interrupt mechanism. Therefore Main loop does nothing.
 * UART_echo_IT.c
 *
 *  Created on: 27 feb. 2021
 *  Author: Eduardo Garcia
 *  Email: egarciamohedano@gmail.com
 */
#include <stdint.h>
#include "stm32f103.h"
#include "stm32f103_gpio_driver.h"
#include "stm32f103_uart_driver.h"

/*
 * IRQ Handler for received data through UART1
 */
void USART1_IRQHandler(void){
	char datos;
	UART_IRQHandling(IRQ_NO_UART1);
	datos = UART_Read(UART1);
	UART_Write_Char(UART1, datos);
}

int main(void)
{
    	/* Configure GPIO pin TX , using PA9 */
	GPIO_Handle_t	gpioTX;
	gpioTX.pGPIOx = GPIOA;
	gpioTX.GPIO_PinConfig.PinMode = GPIO_MODE_AF_PP;
	gpioTX.GPIO_PinConfig.PinNumber = 9;
	gpioTX.GPIO_PinConfig.PinSpeed = GPIO_MODE_OUT_LS;
	GPIO_PeriCLKControl(GPIOA,ENABLE);
	GPIO_Init(&gpioTX);

	/* Configure GPIO pin RX, using PA10*/
	GPIO_Handle_t	gpioRX;
	gpioRX.pGPIOx = GPIOA;
	gpioRX.GPIO_PinConfig.PinMode = GPIO_MODE_AF_PP;
	gpioRX.GPIO_PinConfig.PinNumber = 9;
	gpioRX.GPIO_PinConfig.PinSpeed = GPIO_MODE_OUT_LS;
	GPIO_Init(&gpioRX);

	//Initialize AFIO clock
	AFIO_PCLK_EN();

	//Configure UART1 peripheral settings	
	UART_Handle_t UART_Con;
	UART_Con.pUARTx = UART1;
	UART_Con.UART_Config.Baud_Rate = 19200;
	UART_Con.UART_Config.Mode = UART_MODE_TX_RX;
	UART_Con.UART_Config.Stop_Bits = UART_STOP_BITS_2;
	UART_Con.UART_Config.Word_Length = UART_DATA_LENGTH_7BITS;
	UART_Con.UART_Config.Parity = UART_PARITY_EVEN;
	UART_PeriCLKControl(UART1, ENABLE);
	UART_Init(&UART_Con);


	/*External interrupt configuration*/
	UART_IRQ_Mode(UART1, UART_IRQ_RX_COMPLETED, ENABLE);
	UART_IRQConfig(IRQ_NO_UART1, 1 , ENABLE);

	while(1){
		//Wait for an Interrupt event
	}
	return 0;
}

