/*
 * stm32f103_i2c_driver.h
 *
 *  Created on: 18 mar. 2021
 *      Author: egarc
 */

#ifndef INC_STM32F103_I2C_DRIVER_H_
#define INC_STM32F103_I2C_DRIVER_H_

#include "stm32f103.h"

typedef struct{
	uint8_t 	I2C_ACK;
	uint8_t		I2C_ADDMODE;
	uint8_t		I2C_MODE;
	uint32_t	I2C_SPEED;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
}I2C_Handle_t;

/*
 * Macros to define I2C Interrupt sources
 */
#define		I2C_IRQ_START_BIT_SENT
#define		I2C_IRQ_ADDRESS						//IF MASTER IT MEANS SENT, IF SLAVE ADDRESS IS MATCHED
#define 	I2C_IRQ_10_BIT_HEADER_SENT
#define 	I2C_IRQ_STOP_RECEIVED
#define 	I2C_IRQ_BYTE_TRANSFER_FINISHED
#define 	I2C_IRQ_RX_BUFFER_NOT_EMPTY
#define 	I2C_IRQ_TX_BUFFER_EMPTY

/*
 * Init and Deinit functions for a I2C Peripheral
 */
void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_RegDef_t* pI2Cx);
void I2C_PeriCLKControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);

/*
 * @brief	This function is used to enable the Peripheral
 */
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);

/*
 * Functions to read and write through the given I2C Peripheral
 */
void I2C_Write_Char(I2C_RegDef_t* pI2Cx, char data);
void I2C_Write_String(I2C_RegDef_t* pI2Cx, char* data_buffer, uint8_t size);
char I2C_Read(I2C_RegDef_t* pI2Cx);

/*
 * Functions to set Interrupts configuration and handling
 */
void I2C_IRQ_Mode(I2C_RegDef_t* pI2Cx, uint8_t Mode, uint8_t EnorDi);		//This function is used to configure Rising or Falling edge Interrupt Trigger
void I2C_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi);
void I2C_IRQHandling(uint8_t IRQNumber);

#endif /* INC_STM32F103_I2C_DRIVER_H_ */
