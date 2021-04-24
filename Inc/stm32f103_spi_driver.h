/*
 * stm32f103_spi_driver.h
 *
 *  Created on: 12 dic. 2020
 *      Author: egarc
 */

#ifndef INC_STM32F103_SPI_DRIVER_H_
#define INC_STM32F103_SPI_DRIVER_H_

#include "stm32f103.h"

/*
 * This structure is used to set the SPI settings
 */
typedef struct{
	uint8_t SPI_Baud_Rate;
	uint8_t	SPI_Data_Frame_Format;	//Buffer size
	uint8_t	SPI_Frame_Format;		//Sending order MSB or LSB first
	uint8_t SPI_Mode;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * This is a handle structure for a SPI Peripheral
 */
typedef struct{
	SPI_RegDef_t	*pSPIx;
	SPI_Config_t	SPI_PinConfig;
}SPI_Handle_t;


/*
 * Data Frame Format values
 */
#define 	Data_Format_8_bits		0
#define 	Data_Format_16_bits		1

/*
 * Frame format (choose whether MSB or LSB will be sent first)
 */
#define 	MSB_First		0
#define 	LSB_First		1

/*
 * @SPI Device configuration Mode
 */
#define 	SPI_Slave		0
#define 	SPI_Master		1

/*
 * @SPI Bus config, either Full or Half duplex
 */
#define 	SPI_MODE_FD		0
#define 	SPI_MODE_HD		1

/*
 * Clock Polarity
 */
#define 	SPI_CPOL_LOW		0
#define 	SPI_CPOL_HIGH		1

/*
 * Clock Phase
 */
#define 	SPI_CPHA_LOW	0		// The first clock transition is the first data capture edge
#define 	SPI_CPHA_HIGH	1

/*
 *@SPI Clock Modes
 */
#define 	SPI_MODE1		1
#define 	SPI_MODE2		2
#define 	SPI_MODE3		3
#define 	SPI_MODE4		4

/*
 * @SPI Software Slave Management mode (Hardware or Software)
 */
#define		SPI_SSM_HW		0
#define		SPI_SSM_SW		1

/*
 *@SPI clock speed division
 */
#define 	SPI_PCLK_DIV2		0
#define 	SPI_PCLK_DIV4		1
#define 	SPI_PCLK_DIV8		2
#define 	SPI_PCLK_DIV16		3
#define 	SPI_PCLK_DIV32		4
#define 	SPI_PCLK_DIV64		5
#define 	SPI_PCLK_DIV128		6
#define 	SPI_PCLK_DIV256		7

/*
 * Macros to enable or disable the SPI (this is different from enabling the Peripheral clock)
 */
#define		SPI1_ENABLE()				SPI1->SPI_CR1 |= (1<<6)
#define		SPI2_ENABLE()				SPI2->SPI_CR1 |= (1<<6)
#define		SPI3_ENABLE()				SPI3->SPI_CR1 |= (1<<6)

#define		SPI1_DISNABLE()				SPI1->SPI_CR1 &= ~(1<<6)
#define		SPI2_DISNABLE()				SPI2->SPI_CR1 &= ~(1<<6)
#define		SPI3_DISNABLE()				SPI3->SPI_CR1 &= ~(1<<6)

/*
 * SPI Definitions for Interrupt sources (Mode)
 */
#define		SPI_IRQ_TX_BUFFER_EMPTY			(7)		//Interrupt when the transmit buffer is empty
#define		SPI_IRQ_RX_BUFFER_NOEMPTY		(6)		//Interrupt when the receiver buffer is not empty which means data can be read
#define		SPI_IRQ_ERROR					(5)		//Interrupt for either Overrun o Underrun error *It is needed to check for their respective flags
#define 	SPI_IRQ_TX_DMA					(1)		//Interrupt fro TX DMA buffer


/*****************************************************************************
 *                    APIs supported by this driver
 *****************************************************************************/

/*
 * SPI init and deinit functions
 */
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);

/*
 * Peripheral clock control
 */
void SPI_PeriCLKControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);

/*
 * Peripheral control
 */
void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);

/*
 * SPI Internal Slave Select pin function control
 */
void SPI_SSI_Config(SPI_RegDef_t* pSPIx, uint8_t EnorDi);

/*
 * SPI Functions to write and read from buffer
 */
void 		SPI_Write_Char(SPI_RegDef_t* pSPIx, uint32_t data);
void 		SPI_Write_String(SPI_RegDef_t* pSPIx, uint8_t* data, uint32_t size);
uint16_t	SPI_ReadChar(SPI_RegDef_t* pSPIx);
void		SPI_ReadData(SPI_RegDef_t* pSPIx, uint8_t* data, uint32_t size);  /*Data are declared as pointers*/

/*
 * IRQ handler and its configuration
 */
void SPI_IRQ_Mode(SPI_RegDef_t* pSPIx, uint8_t Mode, uint8_t EnorDi);		//This function is used to configure Rising or Falling edge Interrupt Trigger
void SPI_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi);
void SPI_IRQHandling(uint8_t IRQNumber);

#endif /* INC_STM32F103_SPI_DRIVER_H_ */
