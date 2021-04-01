/*
 * stm32f103_gpio.c
 *
 *  Created on: Dec 1, 2020
 *      Author: egarc
 */


#include "stm32f103_gpio_driver.h"

/*
 * GPIO INIT AND DE_INIT
 */


/***************************************************
 * @fn			-	GPIO_INIT
 *
 * @brief		-	Initializes a GPIO port and a pin config type
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle){
	uint32_t temp = 0;
	if( pGPIOHandle->GPIO_PinConfig.PinNumber <= 7 ){
		//1. and 2. Configure mode pin and speed
		pGPIOHandle->pGPIOx->CRL &= ~(0x3 <<((4*pGPIOHandle->GPIO_PinConfig.PinNumber)+2));
		temp |= (pGPIOHandle->GPIO_PinConfig.PinMode) <<( (4*pGPIOHandle->GPIO_PinConfig.PinNumber)+2 );
		temp |= (pGPIOHandle->GPIO_PinConfig.PinSpeed) <<( 4*pGPIOHandle->GPIO_PinConfig.PinNumber );
		pGPIOHandle->pGPIOx->CRL |= temp;
	}
	else{
		//1. and 2. Configure mode pin and speed
		pGPIOHandle->pGPIOx->CRH &= ~(0x3 <<((4*(pGPIOHandle->GPIO_PinConfig.PinNumber-8))+2));
		temp |= (pGPIOHandle->GPIO_PinConfig.PinMode) <<( (4*(pGPIOHandle->GPIO_PinConfig.PinNumber-8))+2 );
		temp |= (pGPIOHandle->GPIO_PinConfig.PinSpeed) <<( 4*(pGPIOHandle->GPIO_PinConfig.PinNumber-8) );
		pGPIOHandle->pGPIOx->CRH |= temp;
	}
	temp = 0;
	//3. CONFIGURE PULL UP PULL DOWN REG
	temp = (pGPIOHandle->GPIO_PinConfig.PinResistor) << pGPIOHandle->GPIO_PinConfig.PinNumber;
	pGPIOHandle->pGPIOx->ODR |= temp;

}

/***************************************************
 * @fn			-	GPIO_DE-INIT
 *
 * @brief		-	Reset a GPIO port
 *
 * @param[in]	-	GPIO REG DEFINITION
 *
 * @return		-	none
 *
 * @Note		- 	none
 */

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx){
	if( pGPIOx == GPIOA ){
			GPIOA_REG_RESET();
	}
	else if ( pGPIOx == GPIOB ){
		GPIOB_REG_RESET();
	}
	else if ( pGPIOx == GPIOC ){
		GPIOC_REG_RESET();
	}
	else if ( pGPIOx == GPIOD ){
		GPIOD_REG_RESET();
	}
	else if ( pGPIOx == GPIOE ){
		GPIOE_REG_RESET();
	}
	else if ( pGPIOx == GPIOF ){
		GPIOF_REG_RESET();
	}
	else if ( pGPIOx == GPIOG ){
		GPIOG_REG_RESET();
	}
}



/*
 * Peripheral clock control
 */
/***************************************************
 * @fn			-	GPIO_PeriCLKControl
 *
 * @brief		-	This function enables or disables peripheral clock for a certain GPIO port
 *
 * @param[in]	-	GPIO_Register definition pointer
 * @param[in]	-	EnorDi flag to enable or disable peripheral clock
 * @param[in]	-
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void GPIO_PeriCLKControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi){
	if(EnorDi){
		if( pGPIOx == GPIOA ){
			GPIOA_PCLK_EN();
		}
		else if ( pGPIOx == GPIOB ){
			GPIOB_PCLK_EN();
		}
		else if ( pGPIOx == GPIOC ){
			GPIOC_PCLK_EN();
		}
		else if ( pGPIOx == GPIOD ){
			GPIOD_PCLK_EN();
		}
		else if ( pGPIOx == GPIOE ){
			GPIOE_PCLK_EN();
		}
		else if ( pGPIOx == GPIOF ){
			GPIOF_PCLK_EN();
		}
		else if ( pGPIOx == GPIOG ){
			GPIOG_PCLK_EN();
		}

	}
	else{
		if( pGPIOx == GPIOA ){
			GPIOA_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOB ){
			GPIOB_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOC ){
			GPIOC_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOD ){
			GPIOD_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOE ){
			GPIOE_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOF ){
			GPIOF_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOG ){
			GPIOG_PCLK_DIS();
		}

	}

}


/***************************************************
 * @fn			-	GPIO_ReadFromInputPin
 *
 * @brief		-	This function return the state of a certain inpu pin of a GPIO port
 *
 * @param[in]	-	GPIO_Register definition pointer
 * @param[in]	-	Pin Number

 * @return		-	Pin Number State
 *
 * @Note		- 	none
 */
uint8_t 	GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber){
	//Can i make this func return a bool value
	return (uint8_t) ((pGPIOx->IDR >> PinNumber) & 1);

}


/***************************************************
 * @fn			-	GPIO_ReadFromInputPort
 *
 * @brief		-	This function return the state of a GPIO port
 *
 * @param[in]	-	GPIO_Register definition pointer

 * @return		-	Pins Port State
 *
 * @Note		- 	none
 */
uint16_t 	GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx){
	return (uint16_t) pGPIOx->IDR;
}


/***************************************************
 * @fn			-	WriteToOutputPin
 *
 * @brief		-	This function writes value to a certain input pin of a GPIO port
 *
 * @param[in]	-	GPIO_Register definition
 * @param[in]	-	Pin Number
 * @param[in]	-	Value

 * @return		-	none
 *
 * @Note		- 	none
 */
void 		GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value){
		pGPIOx->ODR |= 1 << PinNumber;
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/***************************************************
 * @fn			-	WriteToOutputPort
 *
 * @brief		-	This function writes value to a GPIO port
 *
 * @param[in]	-	GPIO_Register definition
 * @param[in]	-	Value

 *
 * @Note		- 	none
 */
void 		GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}


/***************************************************
 * @fn			-	ToggleOutputPin
 *
 * @brief		-	This function toggles value to a certain input pin of a GPIO port
 *
 * @param[in]	-	GPIO_Register definition
 * @param[in]	-	Pin Number
 * @return		-	none
 *
 * @Note		- 	none
 */
void 		GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}



/*
 * IRQ handler and its configuration
 */

/***************************************************
 * @fn			-	IRQ_Mode
 *
 * @brief		-	This function configures edge detection for a certain input pin of a GPIO port to generate an EXT Interrupt
 *
 * @param[in]	-	GPIO_Register definition
 * @param[in]	-	Pin Number
 * * @param[in]	-	Falling or rising edge
 * @return		-	none
 *
 * @Note		- 	none
 */
void GPIO_IRQ_Mode(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber,uint8_t Edge){
	//1. Enable rising or falling edge detection
	if(Edge){
		EXTI->EXTI_RTSR |=   1 << PinNumber;
		EXTI->EXTI_FTSR &= ~(1 << PinNumber);
	}
	else{
		EXTI->EXTI_FTSR |=   1 << PinNumber;
		EXTI->EXTI_RTSR &= ~(1 << PinNumber);
	}
	//2. Configure the respective port for the EXT line
	uint8_t temp1  = PinNumber/4;
	uint8_t temp2 = PinNumber%4;
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOx);
	AFIO_PCLK_EN();
	AFIO->AFIO_EXTICR[temp1] = portcode << ( temp2 * 4);
	//3. EXTI X line interrupt is not masked, it means enabling the EXT line interrupt
	EXTI->EXTI_IMR |= 1 << PinNumber;
}


/***************************************************
 * @fn			-	IRQ_Config
 *
 * @brief		-	This function sets IRQ Priority for an Interrupt
 *
 * @param[in]	-	IRQ Number
 * @param[in]	-	IRQ Priority
 * @param[in]	-	EnorDi Enables/Disables the IRQ
 * @return		-	none
 *
 * @Note		- 	none
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi){
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
 * @fn			-	GPIO_IRQ_Config
 *
 * @brief		-	This function clears the Pending Status register of the EXTERNAL IRQ INTERRUPTION
 *
 * @param[in]	-	Pin Number
 * @return		-	none
 *
 * @Note		- 	none
 */
void GPIO_IRQHandling(uint8_t PinNumber){
	//Clear the pending register bit for the interruption
	if( EXTI->EXTI_PR & (1<<PinNumber) ){
		EXTI->EXTI_PR |= (1<<PinNumber);
	}

}
