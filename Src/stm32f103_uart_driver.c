/*
 * stm32f103_uart_driver.c
 *
 *  Created on: 16 feb. 2021
 *      Author: egarc
 */

#include "stm32f103_uart_driver.h"
#include <math.h>

#define		HIGH_SPEED_INTERNAL		(0)
#define		HIGH_SPEED_EXTERNAL		(1)
#define		PHASE_LOCKED_LOOP		(2)

#define		DIV_AHB_PREESCALER(sysclk)	( (((RCC->RCC_CFGR>>4)&0xF) == 0) ? sysclk :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 1) ? sysclk :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 2) ? sysclk :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 3) ? sysclk :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 4) ? sysclk :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 5) ? sysclk :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 6) ? sysclk :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 7) ? sysclk :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 8) ? sysclk/2 :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 9) ? sysclk/4 :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 10) ? sysclk/8 :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 11) ? sysclk/16 :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 12) ? sysclk/64 :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 13) ? sysclk/128 :\
										  (((RCC->RCC_CFGR>>4)&0xF) == 14) ? sysclk/256 : sysclk/512)

#define		DIV_APB2_PREESCALER(hclk)	( (((RCC->RCC_CFGR>>11)&0x7) == 0) ? hclk :\
										  (((RCC->RCC_CFGR>>11)&0x7) == 1) ? hclk :\
										  (((RCC->RCC_CFGR>>11)&0x7) == 2) ? hclk :\
										  (((RCC->RCC_CFGR>>11)&0x7) == 3) ? hclk :\
										  (((RCC->RCC_CFGR>>11)&0x7) == 4) ? hclk/2 :\
										  (((RCC->RCC_CFGR>>11)&0x7) == 5) ? hclk/4 :\
										  (((RCC->RCC_CFGR>>11)&0x7) == 6) ? hclk/8 : hclk/16)

#define		DIV_APB1_PREESCALER(hclk)	( (((RCC->RCC_CFGR>>8)&0x7) == 0) ? hclk :\
										  (((RCC->RCC_CFGR>>8)&0x7) == 1) ? hclk :\
										  (((RCC->RCC_CFGR>>8)&0x7) == 2) ? hclk :\
										  (((RCC->RCC_CFGR>>8)&0x7) == 3) ? hclk :\
										  (((RCC->RCC_CFGR>>8)&0x7) == 4) ? hclk/2 :\
										  (((RCC->RCC_CFGR>>8)&0x7) == 5) ? hclk/4 :\
										  (((RCC->RCC_CFGR>>8)&0x7) == 6) ? hclk/8 : hclk/16)

#define		DIV_PREDIV1(exclk)			( (((RCC->RCC_CFGR>>0)&0xF) == 0) ? exclk :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 1) ? exclk/2 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 2) ? exclk/3 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 3) ? exclk/4 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 4) ? exclk/5 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 5) ? exclk/6 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 6) ? exclk/7 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 7) ? exclk/8 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 8) ? exclk/9 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 9) ? exclk/10 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 10) ? exclk/11 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 11) ? exclk/12 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 12) ? exclk/13 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 13) ? exclk/14 :\
										  (((RCC->RCC_CFGR>>0)&0xF) == 14) ? exclk/15 : exclk/16)

#define		PLL_MULTIPLICATION(exclk)	( (((RCC->RCC_CFGR>>18)&0xF) == 0) ? exclk*2 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 1) ? exclk*3 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 2) ? exclk*4 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 3) ? exclk*5 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 4) ? exclk*6 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 5) ? exclk*7 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 6) ? exclk*8 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 7) ? exclk*9 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 8) ? exclk*10 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 9) ? exclk*11 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 10) ? exclk*12 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 11) ? exclk*13 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 12) ? exclk*14 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 13) ? exclk*15 :\
										  (((RCC->RCC_CFGR>>18)&0xF) == 14) ? exclk*16 : exclk*16)

extern uint8_t	CLK_SRC = 0;


/***************************************************
 * @fn			-	UART_INIT
 *
 * @brief		-	Initializes a UART port given the certain config
 *
 * @param[in]	-	*pUARTHandle	Contains both, the Peripheral number to be used and how to be configured
 *
 * @return		-	none
 *
 * @Note		- 	Be aware that UART4 & UART5 cannot be used in Synchronous mode
 */
void UART_Init(UART_Handle_t* pUARTHandle){
	pUARTHandle->pUARTx->UART_CR1 |= (1<<13);			//Enabling USART module
	if(pUARTHandle->UART_Config.Parity != UART_PARITY_NONE){
		pUARTHandle->pUARTx->UART_CR1 |= (1<<10); 																//Enabling parity control
		pUARTHandle->UART_Config.Word_Length = (pUARTHandle->UART_Config.Word_Length == UART_DATA_LENGTH_7BITS) ? 0 : 1;	//Modifying Word length in case Parity control enabled
		if( pUARTHandle->UART_Config.Parity == UART_PARITY_ODD)	pUARTHandle->pUARTx->UART_CR1 |= (1<<9);		//Enabled odd parity
		else pUARTHandle->pUARTx->UART_CR1 &= ~(1<<9);															//Enabled even parity
	}
	else{
		pUARTHandle->pUARTx->UART_CR1 &= ~(1<<10); 													//Disabling parity control
	}

	pUARTHandle->pUARTx->UART_CR1 |= (pUARTHandle->UART_Config.Word_Length << 12);
	pUARTHandle->pUARTx->UART_CR2 |= (pUARTHandle->UART_Config.Stop_Bits << 12);
	uint8_t clk_type = ((RCC->RCC_CFGR>>2)&0x3);	//Selected system clock source
	uint32_t Fclk = 0,hclk = 0;
	//extern uint8_t CLK_SRC;		//In case to use a HSE i.e a crystal, this variable must be set in main code to get proper Baud Rate Config
	switch( clk_type ){
		case HIGH_SPEED_INTERNAL:
			Fclk = 8000000;		//This is the only option
		break;

		case HIGH_SPEED_EXTERNAL:
			Fclk = CLK_SRC;
		break;

		case PHASE_LOCKED_LOOP:
			if ( ((RCC->RCC_CFGR>>16)&1) ){
				//Apply predivision macro to HSE clock
				Fclk = DIV_PREDIV1(CLK_SRC);
			}
			else{
				//High Speed Internal RC 8 Mhz clock
				Fclk = 4000000;
			}
			//PLL MULTIPLICATOR
			Fclk = PLL_MULTIPLICATION(Fclk);
		break;
	}
	hclk = DIV_AHB_PREESCALER(Fclk);
	//Apply the proper APB1 or APB2 clock division for each peripheral
	if( pUARTHandle->pUARTx == UART1 ){
		hclk = DIV_APB2_PREESCALER(hclk);		//Only UART1 hangs on APB2 Bus
	}
	else{
		hclk = DIV_APB1_PREESCALER(hclk);		//The rest of UARTx hangs on APB1 Bus
	}
	float USART_DIV_MANTISSA_f = ((float)hclk)/(16*(pUARTHandle->UART_Config.Baud_Rate));
	uint32_t USART_DIV_FRACTION = 16*(USART_DIV_MANTISSA_f - floor(USART_DIV_MANTISSA_f));
	uint32_t USART_DIV_MANTISSA = (uint32_t) USART_DIV_MANTISSA_f;
	if( USART_DIV_FRACTION >= 16 ){
		USART_DIV_MANTISSA += USART_DIV_FRACTION - 16; //Carry value
		USART_DIV_FRACTION -=16;						//Update fraction part
	}

	pUARTHandle->pUARTx->UART_BRR = USART_DIV_FRACTION;
	pUARTHandle->pUARTx->UART_BRR |= (USART_DIV_MANTISSA << 4);

	//Activate the TX or RX modes
	switch( pUARTHandle->UART_Config.Mode ){
		case 1:
			pUARTHandle->pUARTx->UART_CR1 |= (1<<2);
		break;

		case 2:
			pUARTHandle->pUARTx->UART_CR1 |= (1<<3);
		break;

		case 3:
			pUARTHandle->pUARTx->UART_CR1 |= (3<<2);
		break;

		default:
		break;
	}

}


/***************************************************
 * @fn			-	UART_DEINIT
 *
 * @brief		-	Deinitializes a UART peripheral
 *
 * @param[in]	-	*pUARTx	Contains the Peripheral number to be reset
 * @return		-	none
 */
void UART_DeInit(UART_RegDef_t* pUARTx){
	//Apply the proper reset on respective Bus, either APB1 or APB2 on which the peripheral is hanging
	if(pUARTx == UART1)	RCC->RCC_APB2RSTR &= ~(1<<14);	//Only UART1 hangs on APB2 Bus
	if(pUARTx == UART2) RCC->RCC_APB1RSTR &= ~(1<<17);
	if(pUARTx == UART3) RCC->RCC_APB1RSTR &= ~(1<<18);
	if(pUARTx == UART4) RCC->RCC_APB1RSTR &= ~(1<<19);
	if(pUARTx == UART5) RCC->RCC_APB1RSTR &= ~(1<<20);

}

/***************************************************
 * @fn			-	UART_PeriCLKControl
 *
 * @brief		-	Enables or disables a UART peripheral
 *
 * @param[in]	-	*pUARTx	Contains the Peripheral number to be used
 * @param[in]	-	EnorDi	Flag to enable or disable Peripheral
 * @return		-	none
 */
void UART_PeriCLKControl(UART_RegDef_t* pUARTx, uint8_t EnorDi){
		if(EnorDi){
			if(pUARTx == UART1) UART1_PCLK_EN();
			if(pUARTx == UART2) UART2_PCLK_EN();
			if(pUARTx == UART3) UART3_PCLK_EN();
			if(pUARTx == UART4) UART4_PCLK_EN();
			if(pUARTx == UART5) UART5_PCLK_EN();
		}
		else{
			if(pUARTx == UART1) UART1_PCLK_DIS();
			if(pUARTx == UART2) UART2_PCLK_DIS();
			if(pUARTx == UART3) UART3_PCLK_DIS();
			if(pUARTx == UART4) UART4_PCLK_DIS();
			if(pUARTx == UART5) UART5_PCLK_DIS();
		}
}

/***************************************************
 * @fn			-	UART_Write_Char
 *
 * @brief		-	Writes only one character to the Data Buffer
 *
 * @param[in]	-	*pUARTx	Contains the UART Peripheral to be used
 * @param[in]	-	data	Character to be sent
 * @return		-	none
 */
void UART_Write_Char(UART_RegDef_t* pUARTx, char data){
	pUARTx->UART_DR = data;
}

/***************************************************
 * @fn			-	UART_Write_String
 *
 * @brief		-	Writes an array of data to the UART Data Buffer
 *
 * @param[in]	-	*pUARTx	Contains the UART Peripheral to be used
 * @param[in]	-	data	Array of data to be sent
 * @param[in]	-	size	Size of data Array
 * @return		-	none
 */
void UART_Write_String(UART_RegDef_t* pUARTx, char* data_buffer, uint8_t size){
	for(int i = 0; i < size; i++){
		while( ((pUARTx->UART_SR>>6)&1) == 0 );	//Wait until previous transmission is complete (useful when sending a data buffer)
		UART_Write_Char(pUARTx, *(data_buffer+i));
	}
}

/***************************************************
 * @fn			-	UART_Read
 *
 * @brief		-	Reads from the UART Data Buffer
 *
 * @param[in]	-	*pUARTx	Contains the UART Peripheral to be used
 * @return		-	Data read from the buffer
 */
char UART_Read(UART_RegDef_t* pUARTx){
	char data = 0;
	while( ((pUARTx->UART_SR>>5)&1) == 0 );	//Wait until data is ready to be read
	data = pUARTx->UART_DR;
	return data;
}

/***************************************************
 * @fn			-	UART_IRQ_Mode
 *
 * @brief		-	Configures the respective UART Interruption
 *
 * @param[in]	-	*pUARTx	Contains the UART Peripheral to be used
 * @param[in]	-	Mode It is the respective interrupt source to be enabled
 * @param[in]	-	EnorDi Flag to enable or disable the interrupt
 * @return		-
 */
void UART_IRQ_Mode(UART_RegDef_t* pUARTx, uint8_t Mode, uint8_t EnorDi){
	if(EnorDi){
		pUARTx->UART_CR1 |= (1<<Mode);
		return;
	}
	pUARTx->UART_CR1 &= ~(1<<Mode);
}


/***************************************************
 * @fn			-	UART_IRQ_Config
 *
 * @brief		-	Configures the respective UART Interruption in the NVIC by enabling global interrupt and setting priority
 *
 * @param[in]	-	IRQNumber	Contains the UART Peripheral irq number
 * @param[in]	-	IRQ Priority It is the priority to be set
 * @param[in]	-	EnorDi Flag to enable or disable the interrupt priority
 * @return		-
 */
void UART_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi){
	uint8_t reg_pos = IRQNumber/32;
	uint32_t irq_no = IRQNumber - (32*reg_pos);
	//ENABLING OR DISABLING INTERRUPT NUMBER
	if (EnorDi == 1){
		NVIC->iser[reg_pos] = (1<<irq_no);
	}
	else{
		NVIC->icer[reg_pos] = (1<<irq_no);
	}
	//Setting priority number
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift = (8 * iprx_section) + 4;
	NVIC_PRIOR->ipr[iprx] |= (((uint32_t)IRQPriority) << shift);
}


/***************************************************
 * @fn			-	UART_IRQHandling
 *
 * @brief		-	This function clears the Pending Status register of the global IRQ interruption
 *
 * @param[in]	-	IRQNumber
 * @return		-	none
 *
 * @Note		- 	none
 */
void UART_IRQHandling(uint8_t IRQNumber){
	//Clear the pending register bit for the interruption
	uint8_t reg_pos = IRQNumber/32;
	uint32_t irq_no = IRQNumber - (32*reg_pos);
	if( (NVIC->icpr[reg_pos]>>irq_no) & 1 ){
		NVIC->icpr[reg_pos] |= (1<<irq_no);
	}
}
