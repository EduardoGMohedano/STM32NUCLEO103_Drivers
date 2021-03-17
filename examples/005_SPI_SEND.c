/*
 * 005_SPI_SEND.c
 * This file is used to test sending data over the SPI1 using low level driver
 * A "Hola!" will be sent by SPI which is configured as Master
 *
 *  Created on: March 10th 2021
 *      Author: egarc
 */


#include <stdint.h>
#include<string.h>
#include "stm32f103.h"
#include "stm32f103_gpio_driver.h"
#include "stm32f103_spi_driver.h"

void config_spi1_pins(void);
void delay_ms(uint32_t time);

int main(void)
{
    config_spi1_pins();
    SPI_Handle_t spi_master;
    spi_master.pSPIx = SPI1;
    spi_master.SPI_PinConfig.SPI_Baud_Rate = SPI_PCLK_DIV128;
    spi_master.SPI_PinConfig.SPI_CPHA = SECOND_CLK_TRANSITION;
    spi_master.SPI_PinConfig.SPI_CPOL = SPI_CPOL_HIGH;
    spi_master.SPI_PinConfig.SPI_Data_Frame_Format = Data_Format_8_bits;
    spi_master.SPI_PinConfig.SPI_Frame_Format = MSB_First;
    spi_master.SPI_PinConfig.SPI_SSM = SPI_SSM_HW;
    spi_master.SPI_PinConfig.SPI_Mode = SPI_Master;
    SPI_PeriCLKControl(SPI1,ENABLE);
    SPI_Init(&spi_master);

	while(1){
		SPI_PeripheralControl(SPI1, ENABLE);
		SPI_Write_Char(SPI1, (uint32_t) 'H');
		SPI_Write_Char(SPI1, (uint32_t) 'o');
		SPI_Write_Char(SPI1, (uint32_t) 'l');
		SPI_Write_Char(SPI1, (uint32_t) 'a');
		SPI_Write_Char(SPI1, (uint32_t) '\n');

		//Confirming spi is not busy
		while( (SPI1->SPI_SR>>7)&1 );
		SPI_PeripheralControl(SPI1, DISABLE);
		delay_ms(1000000);
	}
}

void config_spi1_pins(){
	/* MOSI PIN CONFIG PA7 NO_REMAP*/
	GPIO_Handle_t	mosi_pin;
	mosi_pin.pGPIOx = GPIOA;
	mosi_pin.GPIO_PinConfig.PinMode = GPIO_MODE_AF_PP;
	mosi_pin.GPIO_PinConfig.PinNumber = 7;
	mosi_pin.GPIO_PinConfig.PinSpeed = GPIO_MODE_OUT_HS;
	GPIO_PeriCLKControl(GPIOA,ENABLE);
	GPIO_Init(&mosi_pin);

	/*SCLK PIN CONFIG PA5 NO_REMAP*/
	GPIO_Handle_t	sck_pin;
	sck_pin.pGPIOx = GPIOA;
	sck_pin.GPIO_PinConfig.PinMode = GPIO_MODE_AF_PP;
	sck_pin.GPIO_PinConfig.PinNumber = 5;
	sck_pin.GPIO_PinConfig.PinSpeed = GPIO_MODE_OUT_HS;
	//sck_pin.GPIO_PinConfig.PinResistor = GPIO_PU;       /*Pull up resistor activated*/
	GPIO_Init(&sck_pin);

	/*CSS PIN CONFIG PA4 NO_REMAP*/
	GPIO_Handle_t	nss_pin;
	nss_pin.pGPIOx = GPIOA;
	nss_pin.GPIO_PinConfig.PinMode = GPIO_MODE_AF_PP;
	nss_pin.GPIO_PinConfig.PinNumber = 4;
	nss_pin.GPIO_PinConfig.PinSpeed = GPIO_MODE_OUT_HS;
	//sck_pin.GPIO_PinConfig.PinResistor = GPIO_PU;       ///Pull up resistor activated
	GPIO_Init(&nss_pin);

	AFIO_PCLK_EN();
}

void delay_ms(uint32_t time){
	while(time > 0){
		time--;
	}
}
