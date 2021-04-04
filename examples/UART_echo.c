/* This code is aimed to perform an echo effect by sending data back of already data received through UART1
 * this example works by polling data in the Infinite While loop 
 * UART_echo.c
 *
 *  Created on: 07 March 2021
 *  Author: Eduardo Garcia
 *  Email: egarciamohedano@gmail.com
 */
#include <stdint.h>
#include "stm32f103.h"
#include "stm32f103_gpio_driver.h"
#include "stm32f103_uart_driver.h"

int main(void)
{
   	 /* Configure GPIO pin as TX, using PA9 */
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
	gpioRX.GPIO_PinConfig.PinMode = GPIO_MODE_IN_PU_PD;
	gpioRX.GPIO_PinConfig.PinNumber = 10;
	gpioRX.GPIO_PinConfig.PinSpeed = GPIO_MODE_INPUT;
	gpioRX.GPIO_PinConfig.PinResistor = GPIO_PU;
	GPIO_Init(&gpioRX);

	//Initialize AFIO clock
	AFIO_PCLK_EN();

	UART_Handle_t UART_Con;
	UART_Con.pUARTx = UART1;
	UART_Con.UART_Config.Baud_Rate = 19200;
	UART_Con.UART_Config.Mode = UART_MODE_TX_RX;
	UART_Con.UART_Config.Stop_Bits = UART_STOP_BITS_2;
	UART_Con.UART_Config.Word_Length = UART_DATA_LENGTH_7BITS;
	UART_Con.UART_Config.Parity = UART_PARITY_EVEN;
	UART_Con.UART_Config.HWFlow_Control = UART_HW_FLOW_CTRL_NONE;
	UART_PeriCLKControl(UART1, ENABLE);
	UART_Init(&UART_Con);
	
	//Datos will be used to poll data in UART
	static char datos;
	
	while(1){
		datos = UART_Read(UART1);
		UART_Write_Char(UART1, datos); //Sending back every received character
	}
	return 0;
}

