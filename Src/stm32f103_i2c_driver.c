/*
 * stm32f103_i2c_driver.c
 *
 *  Created on: 18 mar. 2021
 *      Author: egarc
 */

#include "stm32f103_i2c_driver.h"

/***************************************************
 * @fn			-	I2C_Init
 *
 * @brief		-	This function sets peripheral settings for I2C
 *
 * @param[in]	-	pI2CHandler definition pointer
 *
 * @return		-	none
 * @Note		- 	none
 */
void I2C_Init(I2C_Handle_t* pI2CHandle){
	//Configure Standard or Fast Mode
	pI2CHandle->pI2Cx->I2C_CCR |= (pI2CHandle->I2C_Config.I2C_SPEED_MODE << 15);

	//Configure SCL speed
	uint8_t clk_type = ((RCC->RCC_CFGR>>2)&0x3);	//Selected system clock source
	uint32_t Fclk = 0,hclk = 0;
	switch( clk_type ){
		case HIGH_SPEED_INTERNAL:
			Fclk = 8000000;		//This is the only option
		break;

		case HIGH_SPEED_EXTERNAL:
			Fclk = 0;			//Fill this value if using an external clock
		break;

		case PHASE_LOCKED_LOOP:
			if ( ((RCC->RCC_CFGR>>16)&1) ){
				//Apply predivision macro to HSE clock
				Fclk = 0;  //Fill this value if using an external clock  DIV_PREDIV1(External clock freq value)
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
	//Apply the proper APB1 clock division peripheral
	hclk = DIV_APB1_PREESCALER(hclk);		//The I2C peripheral hangs on APB1 Bus
	pI2CHandle->pI2Cx->I2C_CR2 |= ((hclk/1000000) & 0x0000001F );	//Getting the clock freq number in Mhz and loading value of clock
	uint16_t CCR = 0;
	//Standard mode
	if( pI2CHandle->I2C_Config.I2C_SPEED_MODE == I2C_SPEED_SM ){
		CCR = I2C_GET_CCR_SM(hclk,pI2CHandle->I2C_Config.I2C_SPEED);
	}
	//Fast mode
	if( pI2CHandle->I2C_Config.I2C_FM_DUTY_CYCLE == I2C_FM_DUTY_2 ){
		CCR = I2C_GET_CCR_FM_DUTY_2(hclk,pI2CHandle->I2C_Config.I2C_SPEED);
	}
	else{
		CCR = I2C_GET_CCR_FM_DUTY_16_9(hclk,pI2CHandle->I2C_Config.I2C_SPEED);
	}
	pI2CHandle->pI2Cx->I2C_CCR |= (CCR & 0x000000FFF);								//Load Value to CCR
	pI2CHandle->pI2Cx->I2C_CCR |= (pI2CHandle->I2C_Config.I2C_FM_DUTY_CYCLE <<14);	//Duty Cycle

	//Configure the device address in case it is slave
	pI2CHandle->pI2Cx->I2C_OAR1 |= (pI2CHandle->I2C_Config.I2C_ADDRESS << 1);
	pI2CHandle->pI2Cx->I2C_OAR1 |= (1 << 14);

	//Enable the acknowledge
	pI2CHandle->pI2Cx->I2C_CR1 |= (pI2CHandle->I2C_Config.I2C_ACK_CTRL << 10);

	//Configure the rise time of pins
	uint8_t TRISE = 0;
	if(pI2CHandle->I2C_Config.I2C_SPEED_MODE){	//Fast mode
		TRISE = (uint8_t) ((float)(hclk/1000000000)*I2C_MAX_RISE_TIME_FM);
	}
	else{	//Standard mode
		TRISE = (uint8_t) ((float)(hclk/1000000000)*I2C_MAX_RISE_TIME_SM);
	}
	pI2CHandle->pI2Cx->I2C_TRISE |= (TRISE & 0x3F);	//Load value
}

/***************************************************
 * @fn			-	I2C_DeInit
 *
 * @brief		-	This function disables peripheral clock for I2C
 *
 * @param[in]	-	I2C_Register definition pointer
 *
 * @return		-	none
 * @Note		- 	none
 */
void I2C_DeInit(I2C_RegDef_t* pI2Cx){
	if(pI2Cx == I2C1){
		RCC->RCC_APB1RSTR |= (1<<21);		//Reset all Peripheral registers
	}
	else if(pI2Cx == I2C2){
		RCC->RCC_APB1RSTR |= (1<<22);
	}

}

/***************************************************
 * @fn			-	I2C_PeriCLKControl
 *
 * @brief		-	This function enables or disables peripheral clock for I2C
 *
 * @param[in]	-	I2C_Register definition pointer
 * @param[in]	-	EnorDi flag to enable or disable peripheral clock
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void I2C_PeriCLKControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi){
	if(EnorDi){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
	}
	else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DIS();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_DIS();
		}
	}
}

/*
 * @brief	This function is used to enable the Peripheral
 */
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi){
	if(EnorDi){
		pI2Cx->I2C_CR1 |= (1<<0);
		return;
	}
	pI2Cx->I2C_CR1 &= ~(1<<0);
}

/***************************************************
 * @fn			-	I2C_Master_SendData
 *
 * @brief		-	Send a buffer of data through I2C
 *
 * @param[in]	-	*pI2Cx	Contains the I2C Peripheral to be used
 * @param[in]	-	data It is the buffer which contains data to be sent
 * @param[in]	-	size It is the size of the buffer
 * @param[in]	-	SlaveAddr	It is the slave address to send data to
 * @return		-
 */
void I2C_Master_SendData(I2C_RegDef_t* pI2Cx, char* data_buffer, uint8_t size, uint8_t SlaveAddr){
	//Start condition
	pI2Cx->I2C_CR1 |= (1<<8);
	while( !(pI2Cx->I2C_SR1&1) );	//Wait until Start condition was generated
	//Address phase
	pI2Cx->I2C_DR |= ((SlaveAddr<<1)&0xFE);	//Shift Slave Address to insert at 0th position the r/w bit (write)
	I2C_ClearADDRFlag(pI2Cx);		//Clearing Address flag

	//Send data until size becomes 0
	while(size > 0){
		while( !((pI2Cx->I2C_SR1>>7)&1) );
		pI2Cx->I2C_DR = *data_buffer;
		data_buffer++;
		size--;
	}
	while( !((pI2Cx->I2C_SR1>>7)&1) );		//Wait until TXE is 1
	while( !((pI2Cx->I2C_SR1>>2)&1) );		//Wait until BTF is 1

	//Stop condition
	pI2Cx->I2C_CR1 |= (1<<9);
}


/***************************************************
 * @fn			-	I2C_IRQ_Mode
 *
 * @brief		-	Configures the respective I2C Interruption
 *
 * @param[in]	-	*pI2Cx	Contains the I2C Peripheral to be used
 * @param[in]	-	Mode It is the respective interrupt source to be enabled
 * @param[in]	-	EnorDi Flag to enable or disable the interrupt
 * @return		-
 */
void I2C_IRQ_Mode(I2C_RegDef_t* pI2Cx, uint8_t Mode, uint8_t EnorDi){
	if(EnorDi){
		pI2Cx->I2C_CR2 |= (1<<9);
		if( (Mode == I2C_IRQ_RX_BUFFER_NOT_EMPTY) || (Mode == I2C_IRQ_TX_BUFFER_EMPTY) ){
			pI2Cx->I2C_CR2 |= (1<<10);
		}
		return;
	}
	pI2Cx->I2C_CR2 &= ~(1<<9);
	if( (Mode == I2C_IRQ_RX_BUFFER_NOT_EMPTY) || (Mode == I2C_IRQ_TX_BUFFER_EMPTY) ){
		pI2Cx->I2C_CR2 &= ~(1<<10);
	}
}


/***************************************************
 * @fn			-	I2C_IRQ_Config
 *
 * @brief		-	Configures the respective I2C Interruption in the NVIC by enabling global interrupt and setting priority
 *
 * @param[in]	-	IRQNumber	Contains the I2C Peripheral irq number
 * @param[in]	-	IRQ Priority It is the priority to be set
 * @param[in]	-	EnorDi Flag to enable or disable the interrupt priority
 * @return		-
 */
void I2C_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi){
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


static void I2C_ClearADDRFlag(I2C_RegDef_t* pI2Cx){
	uint32_t dummy_data = pI2Cx->I2C_SR1;
	dummy_data = pI2Cx->I2C_SR2;
	(void)dummy_data;
}
