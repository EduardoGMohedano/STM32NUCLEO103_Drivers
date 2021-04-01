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
