/*
 * stm32f103_gpio_driver.h
 *
 *  Created on: Dec 1, 2020
 *      Author: egarc
 */

#ifndef INC_STM32F103_GPIO_DRIVER_H_
#define INC_STM32F103_GPIO_DRIVER_H_


#include "stm32f103.h"


/*
 *
 */
typedef struct{
	uint8_t		PinNumber;		//Use 0 to 15 to indicate pin number
	uint8_t		PinMode;		//Use @GPIO_INPUT_CONFIG or @GPIO_OUTPUT CONFIG to set up pin number as input or output
	uint8_t		PinSpeed;		//Use @GPIO_SPEED
	uint8_t		PinResistor;
}GPIO_PinConfig_t;


/*
 * This is a handle structure for a GPIO Pin
 */
typedef struct{
	GPIO_RegDef_t		*pGPIOx;
	GPIO_PinConfig_t	GPIO_PinConfig;
}GPIO_Handle_t;


/*@GPIO_INPUT_CONFIG
 * GPIO pin input CONFIG possible modes
 */
#define		GPIO_MODE_ANALOG	0
#define		GPIO_MODE_FLOAT		1
#define		GPIO_MODE_IN_PU_PD	2

/*@GPIO_OUTPUT CONFIG
 * GPIO pin output CONFIG possible modes
 */
#define		GPIO_MODE_PP		0x00
#define		GPIO_MODE_OD		0x01
#define		GPIO_MODE_AF_PP		0x02
#define		GPIO_MODE_AF_OD		0x03

/*@GPIO_SPEED
 * GPIO pin possible modes
 */
#define		GPIO_MODE_INPUT		0
#define		GPIO_MODE_OUT_MS	1		//OUTPUT MODE 10MHZ
#define		GPIO_MODE_OUT_LS	2		//OUTPUT MODE 2MHZ
#define		GPIO_MODE_OUT_HS	3		//OUTPUT MODE 50MHZ

/*@GPIO PULL UP/PULL DOWN RESISTOR
 * GPIO PULL UP PULL DOWN RESISTOR
 */
#define		GPIO_PD		0		//PULL DOWN
#define		GPIO_PU		1		//PULL UP



/*****************************************************************************
 *                    APIs supported by this driver
 *****************************************************************************/

/*
 * GPIO INIT AND DE_INIT
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/*
 * Peripheral clock control
 */
void GPIO_PeriCLKControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);


/*
 * Read and write
 */
uint8_t 	GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);		//Can i make this func return a bool value
uint16_t 	GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void 		GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void 		GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value);
void 		GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/*
 * IRQ handler and its configuration
 */
void GPIO_IRQ_Mode(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber,uint8_t Edge);		//This function is used to configure Rising or Falling edge Interrupt Trigger
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F103_GPIO_DRIVER_H_ */
