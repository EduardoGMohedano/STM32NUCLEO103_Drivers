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
	uint8_t 	I2C_ACK_CTRL;
	uint8_t		I2C_FM_DUTY_CYCLE;
	uint8_t		I2C_ADDRESS;		//Own device address if mcu is selected as slave
	uint8_t		I2C_SPEED_MODE;
	uint32_t	I2C_SPEED;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
}I2C_Handle_t;

/*
 * @I2C Speed Mode macros
 */
#define 	I2C_SPEED_SM		0		//To select this mode Peripheral clock must be at least 2Mhz, SCL up to 100KHz
#define 	I2C_SPEED_FM		1		//To select this mode Peripheral clock must be at least 4Mhz, SCL up to 400KHz

/*
 * @I2C Acknowledge control
 */
#define 	I2C_ACK_ENABLE		1
#define 	I2C_ACK_DISABLE		0

/*
 * I2C FM duty cycle
 */
#define		I2C_FM_DUTY_2		0
#define		I2C_FM_DUTY_16_9	1

/*
 * Macros to define I2C Interrupt sources
 */
#define		I2C_IRQ_START_BIT_SENT			0
#define		I2C_IRQ_ADDRESS					1	//If mcu is Master it means send data, if mcu is slave address is matched
#define 	I2C_IRQ_10_BIT_HEADER_SENT		2
#define 	I2C_IRQ_STOP_RECEIVED			3
#define 	I2C_IRQ_BYTE_TRANSFER_FINISHED	4
#define 	I2C_IRQ_RX_BUFFER_NOT_EMPTY		5
#define 	I2C_IRQ_TX_BUFFER_EMPTY			6

/*
 * Macro to get CCR value to be loaded
 */
#define		I2C_GET_CCR_SM(FPher,Fi2c)				((FPher)/(Fi2c*2))
#define		I2C_GET_CCR_FM_DUTY_2(FPher,Fi2c)		((FPher)/(Fi2c*3))
#define		I2C_GET_CCR_FM_DUTY_16_9(FPher,Fi2c)	((FPher)/(Fi2c*25))

/*
 * Macros to define Maximum Rise time as per Standard, expresed in nanoseconds
 */
#define		I2C_MAX_RISE_TIME_SM	1000
#define		I2C_MAX_RISE_TIME_FM	300

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
void I2C_Master_SendData(I2C_RegDef_t* pI2Cx, char* data_buffer, uint8_t size, uint8_t SlaveAddr);
void I2C_Master_ReceiveData(I2C_Handle_t* pI2CHandle, char* data_buffer, uint8_t size, uint8_t SlaveAddr);
char I2C_Read(I2C_RegDef_t* pI2Cx);

/*
 * Functions to implement Writing sequences of data
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t* pI2Cx);

/*
 * Functions to set Interrupts configuration and handling
 */
void I2C_IRQ_Mode(I2C_RegDef_t* pI2Cx, uint8_t Mode, uint8_t EnorDi);		//This function is used to configure Rising or Falling edge Interrupt Trigger
void I2C_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi);
void I2C_IRQHandling(uint8_t IRQNumber);

#endif /* INC_STM32F103_I2C_DRIVER_H_ */
