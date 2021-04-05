/*
 * stm32f103_uart_driver.h
 *
 *  Created on: 16 feb. 2021
 *      Author: egarc
 */

#ifndef INC_STM32F103_UART_DRIVER_H_
#define INC_STM32F103_UART_DRIVER_H_

#include "stm32f103.h"

/*
 * This structure is used to set the UART settings
 */
typedef struct{
	uint8_t		Word_Length;		//Use it to send either 8 or 9 bits
	uint8_t		Stop_Bits;			//Used to configure stop bits it can be 0.5, 1, 1.5 and 2 bits. Being 1 bit the default mode
	uint32_t	Baud_Rate;			//Used to set Baud Rate for Data Transmission
	uint8_t		Mode;				//Used to enable Transmitter, Receiver or both modes
	uint8_t		Parity;
	uint8_t		HWFlow_Control;
	uint32_t	External_clk;		//Set the variable in case an external Crystal is being used
}UART_Config_t;

/*
 * This is a handle structure for a UART Peripheral
 */
typedef struct{
	UART_RegDef_t		*pUARTx;
	UART_Config_t		UART_Config;
}UART_Handle_t;


/*
 * Definitions to handle UART Configurations
 */

#define		UART_STOP_BITS_0_5			(1)
#define		UART_STOP_BITS_1			(0)
#define		UART_STOP_BITS_1_5			(3)
#define		UART_STOP_BITS_2			(2)				//0.5 and 1.5 Stop bits are not available for UART4 & UART5

#define		UART_DATA_LENGTH_8BITS		(0)
#define		UART_DATA_LENGTH_9BITS		(1)
#define 	UART_DATA_LENGTH_7BITS		(2)				//ONLY AVAILABLE IF PARITY CONTROL ENABLED

#define		UART_MODE_RX				(1)
#define		UART_MODE_TX				(2)
#define		UART_MODE_TX_RX				(3)				//Macros to enable or disable Transmitter and Receiver

#define		UART_PARITY_EVEN			(0)
#define		UART_PARITY_ODD				(1)
#define		UART_PARITY_NONE			(2)

#define 	UART_HW_FLOW_CTRL_NONE		(0)
#define 	UART_HW_FLOW_CTS			(2)
#define 	UART_HW_FLOW_RTS			(1)
#define 	UART_HW_FLOW_CTS_RTS		(3)				//Macros for defining Hardware Flow Control this mode is
														//available only for UART1, 2 and 3

/*
 * UART Definitions for interruption sources
 */
#define 	UART_IRQ_TX_DR_EMPTY		(7)			//Transmit data register empty
#define 	UART_IRQ_TX_COMPLETED		(6)			//Transmission is complete
#define 	UART_IRQ_RX_COMPLETED		(5)			//Received data is readyy to be read
#define 	UART_IRQ_IDLE				(4)			//Idle is detected


/*
 * Definitons to Enable or Disable Peripheral Clock
 */
#define		UART1_ENABLE()				(UART1->UART_CR1 |= (1<<13))
#define		UART2_ENABLE()				(UART2->UART_CR1 |= (1<<13))
#define		UART3_ENABLE()				(UART3->UART_CR1 |= (1<<13))
#define		UART4_ENABLE()				(UART4->UART_CR1 |= (1<<13))
#define		UART5_ENABLE()				(UART5->UART_CR1 |= (1<<13))

#define		UART1_DISABLE()				(UART1->UART_CR1 &= ~(1<<13))
#define		UART2_DISABLE()				(UART2->UART_CR1 &= ~(1<<13))
#define		UART3_DISABLE()				(UART3->UART_CR1 &= ~(1<<13))
#define		UART4_DISABLE()				(UART4->UART_CR1 &= ~(1<<13))
#define		UART5_DISABLE()				(UART5->UART_CR1 &= ~(1<<13))


/*
 * Init and Deinit functions for a UART Peripheral
 */
void UART_Init(UART_Handle_t* pUARTHandle);
void UART_DeInit(UART_RegDef_t* pUARTx);
void UART_PeriCLKControl(UART_RegDef_t* pUARTx, uint8_t EnorDi);

/*
 * Functions to read and write through the given UART Peripheral
 */
void UART_Write_Char(UART_RegDef_t* pUARTx, char data);
void UART_Write_String(UART_RegDef_t* pUARTx, char* data_buffer, uint8_t size);
char UART_Read(UART_RegDef_t* pUARTx);

/*
 * Functions to set Interrupts configuration and handling
 */
void UART_IRQ_Mode(UART_RegDef_t* pUARTx, uint8_t Mode, uint8_t EnorDi);		//This function is used to configure Rising or Falling edge Interrupt Trigger
void UART_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi);
void UART_IRQHandling(uint8_t IRQNumber);


#endif /* INC_STM32F103_UART_DRIVER_H_ */
