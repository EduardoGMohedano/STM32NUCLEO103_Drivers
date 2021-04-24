/*
 * stm32f103_spi_driver.c
 *
 *  Created on: 13 dic. 2020
 *      Author: egarc
 */

#include "stm32f103_spi_driver.h"



/***************************************************
 * @fn			-	SPI_INIT
 *
 * @brief		-	Initializes a SPI port and a load its config
 *
 * @param[in]	-	SPI_Handle_t contains port number and its configurations
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void SPI_Init(SPI_Handle_t* pSPIHandle){
	//I might need to reset the whole register or peripheral in order to really write the correct settings using OR operator
	pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL);		//CPOL SETTING
	pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA);		//CPHA SETTING
	pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPI_PinConfig.SPI_Data_Frame_Format << SPI_CR1_DFF);	//Buffer size
	pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPI_PinConfig.SPI_Frame_Format << SPI_CR1_DFF);	//Data Frame Format MSB or LSB first
	//By default the SPI will be handled by hardware (Chip select must be physically wired)
	if ( 0 == pSPIHandle->SPI_PinConfig.SPI_SSM ){
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << 2);		//SSOE enables pin NSS as output used when in master mode
	}
	else{
		pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPI_PinConfig.SPI_SSM << 9);		//Enabling Software slave management
	}

	//Settings for SPI configured as a Master
	if(pSPIHandle->SPI_PinConfig.SPI_Mode == SPI_Master){
		pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPI_PinConfig.SPI_Baud_Rate << SPI_CR1_BR);	//BAUD RATE SETTING
		pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPI_PinConfig.SPI_Mode <<  SPI_CR1_MODE);		//Master mode

	}
	else{
		pSPIHandle->pSPIx->SPI_CR1 &= ~(~pSPIHandle->SPI_PinConfig.SPI_Mode << SPI_CR1_MODE);		//slave mode
	}
}

/***************************************************
 * @fn			-	SPI_DEINIT
 *
 * @brief		-	De-Initializes a SPI port
 *
 * @param[in]	-	SPI_Handle_t contains port number and its configurations
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void SPI_DeInit(SPI_RegDef_t* pSPIx){
	if( pSPIx == SPI1 ){
		SPI1_REG_RESET();
	}
	else if ( pSPIx == SPI2 ){
		SPI2_REG_RESET();
	}
	else if ( pSPIx == SPI3 ){
		SPI3_REG_RESET();
	}
}


/***************************************************
 * @fn			-	SPI_PeriCLKControl
 *
 * @brief		-	This function enables or disables peripheral clock for a SPI
 *
 * @param[in]	-	SPI_Register definition pointer
 * @param[in]	-	EnorDi flag to enable or disable peripheral clock
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void SPI_PeriCLKControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi){
	if(EnorDi){
		if( pSPIx == SPI1 ){
			SPI1_PCLK_EN();
		}
		else if ( pSPIx == SPI2 ){
			SPI2_PCLK_EN();
		}
		else if ( pSPIx == SPI3 ){
			SPI3_PCLK_EN();
		}
	}
	else{
		if( pSPIx == SPI1 ){
			SPI1_PCLK_DIS();
		}
		else if ( pSPIx == SPI2 ){
			SPI2_PCLK_DIS();
		}
		else if ( pSPIx == SPI3 ){
			SPI3_PCLK_DIS();
		}
	}
}

/***************************************************
 * @fn			-	SPI_PeripheralControl
 *
 * @brief		-	This function enables and disables SPI peripheral
 *
 * @param[in]	-	SPIx Periheral
 * @param[in]	-	EnorDi flag to enable or disable peripheral
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi){
	if(EnorDi){
		pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_ENABLE );		//Enable the peripheral
		return;
	}
	pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_ENABLE );
}

/***************************************************
 * @fn			-	SPI_SSI_Config
 *
 * @brief		-	This function sets pin state for internal Slave Management
 *
 * @param[in]	-	SPI_Register definition pointer
 * @param[in]	-	EnorDi flag to enable or disable peripheral clock
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void SPI_SSI_Config(SPI_RegDef_t* pSPIx, uint8_t EnorDi){
	//Settings for SPI SSI pin (internal)
	if(EnorDi){
		pSPIx->SPI_CR1 |= ( 1 << 8);
	}
	else{
		pSPIx->SPI_CR1 &= ~(1 << 8);
	}
}


/***************************************************
 * @fn			-	SPI_Write_Char
 *
 * @brief		-	This function writes one character to the Buffer indirectly by writing to the DATA REGISTER
 *
 * @param[in]	-	SPI_Register definition pointer
 * @param[in]	-	data in uint32 type
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void 	SPI_Write_Char(SPI_RegDef_t* pSPIx, uint32_t data){
		while( ((pSPIx->SPI_SR>>SPI_SR_TXE)&1) == 0x0 );	//Blocking function to wait until Buffer is empty and avoid corrupting data
		if( (pSPIx->SPI_CR1>>SPI_CR1_DFF)&1   ){
			pSPIx->SPI_DR = (uint16_t) data;
		}
		else{
			pSPIx->SPI_DR = (uint8_t) data;
		}
}

/***************************************************
 * @fn			-	SPI_Write_String
 *
 * @brief		-	This function writes data to the Buffer indirectly by writing to the DATA REGISTER
 *
 * @param[in]	-	SPI_Register definition pointer
 * @param[in]	-	data in 32 bits type
 * @param[in]	-	size is the actual data buffer size
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void 	SPI_Write_String(SPI_RegDef_t* pSPIx, uint8_t* data, uint32_t size){

	while(size > 0){
		while( ((pSPIx->SPI_SR>>SPI_SR_TXE)&1) == 0x0 );	//Blocking function to wait until Buffer is empty and avoid corrupting data
		if( (pSPIx->SPI_CR1>>SPI_CR1_DFF)&1   ){
			pSPIx->SPI_DR = *((uint16_t*)data);
			size-=2;
			(uint16_t*)data++;
		}
		else{
			pSPIx->SPI_DR = *data;
			size--;
			data++;
		}
	}
}

/***************************************************
 * @fn			-	SPI_ReadChar
 *
 * @brief		-	This function reads data from the DATA REGISTER
 *
 * @param[in]	-	SPI_Register definition pointer to read from
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
uint16_t	SPI_ReadChar(SPI_RegDef_t* pSPIx){
	while( ! (pSPIx->SPI_SR&1) );	//Blocking function to wait until Buffer is not empty and avoid corrupting received data
	return pSPIx->SPI_DR;
}

/***************************************************
 * @fn			-	SPI_ReadData
 *
 * @brief		-	This function reads data from the DATA REGISTER
 *
 * @param[in]	-	SPI_Register definition pointer
 * @param[in]	-	data in 32 bits type
 * @param[in]	-	len is the actual length of the data, either 8 or 16 bits
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void	SPI_ReadData(SPI_RegDef_t* pSPIx, uint8_t* data, uint32_t size){
	while(size > 0){
		while( ((pSPIx->SPI_SR>>SPI_SR_RXNE)&1) == 0x0 );	//Blocking function to wait until Buffer is not empty and avoid corrupting received data
		if( (pSPIx->SPI_CR1>>SPI_CR1_DFF)&1  ){
			*((uint16_t*)data) = pSPIx->SPI_DR ;
			size-=2;
			(uint16_t*)data++;
		}
		else{
			*data = pSPIx->SPI_DR;
			size--;
			data++;
		}
	}
}

/***************************************************
 * @fn			-	SPI_IRQ_Mode
 *
 * @brief		-	This function sets the respective Interruption source for SPI
 *
 * @param[in]	-	SPI_Register definition pointer
 * @param[in]	-	Mode to indicate the possible SPI Interrupt
 * @param[in]	-	EnorDi to enable or disable Interruption source
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void SPI_IRQ_Mode(SPI_RegDef_t* pSPIx, uint8_t Mode, uint8_t EnorDi){
	if(EnorDi){
			pSPIx->SPI_CR2 |= (1 << Mode);
			return;
	}
	pSPIx->SPI_CR2 &= ~(1 << Mode);
}

/***************************************************
 * @fn			-	SPI_IRQ_Config
 *
 * @brief		-	Configures the respective SPI Interruption in the NVIC by enabling global interrupt and setting priority
 *
 * @param[in]	-	SPI IRQ interruption number
 * @param[in]	-	IRQ Priority to be set
 * @param[in]	-	EnorDi Flag to enable or disable the interrupt priority
 *
 * @return		-	none
 *
 * @Note		- 	none
 */
void SPI_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi){
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
 * @fn			-	SPI_IRQHandling
 *
 * @brief		-	This function clears the Pending Status register of the global IRQ interruption
 *
 * @param[in]	-	IRQNumber
 * @return		-	none
 *
 * @Note		- 	none
 */
void SPI_IRQHandling(uint8_t IRQNumber){
	//Clear the pending register bit for the interruption
	uint8_t reg_pos = IRQNumber/32;
	uint32_t irq_no = IRQNumber - (32*reg_pos);
	if( (NVIC->icpr[reg_pos]>>irq_no) & 1 ){
		NVIC->icpr[reg_pos] |= (1<<irq_no);
	}
}
