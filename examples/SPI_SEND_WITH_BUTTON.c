/*
 * SPI_SEND_WITH_BUTTON.c
 * This file is used to test sending data over the SPI1 using low level driver
 * A "Hello world!" text will be sent by Master when a button is pressed and if the slave acknowledges the command to do so.
 * The board is configured as Master
 *
 *  Created on: 22 mar. 2021
 *      Author: egarc
 */

#include <stdint.h>
#include <string.h>
#include "stm32f103.h"
#include "stm32f103_gpio_driver.h"
#include "stm32f103_spi_driver.h"

#define  command		'W'

void config_spi1_pins(void);
void delay_ms(uint32_t time);

int main(void)
{

	while( ! (RCC->RCC_CR>>1)&1 );	//Wait until internal clock is in steady state

	/* Button config */
	GPIO_Handle_t	gpiobtn;
	gpiobtn.pGPIOx = GPIOC;
	gpiobtn.GPIO_PinConfig.PinMode = GPIO_MODE_IN_PU_PD;
	gpiobtn.GPIO_PinConfig.PinNumber = 13;
	gpiobtn.GPIO_PinConfig.PinSpeed = GPIO_MODE_INPUT;
	gpiobtn.GPIO_PinConfig.PinResistor = GPIO_PU;       /*Pull up resistor activated*/
	GPIO_PeriCLKControl(GPIOC,ENABLE);
	GPIO_Init(&gpiobtn);


    config_spi1_pins();
    SPI_Handle_t spi_master;
    spi_master.pSPIx = SPI1;
    spi_master.SPI_PinConfig.SPI_Baud_Rate = SPI_PCLK_DIV256;
    spi_master.SPI_PinConfig.SPI_CPHA = SPI_CPHA_HIGH;
    spi_master.SPI_PinConfig.SPI_CPOL = SPI_CPOL_HIGH;
    spi_master.SPI_PinConfig.SPI_Data_Frame_Format = Data_Format_8_bits;	//Later try using 16 bits
    spi_master.SPI_PinConfig.SPI_Frame_Format = MSB_First;
    spi_master.SPI_PinConfig.SPI_SSM = SPI_SSM_HW;
    spi_master.SPI_PinConfig.SPI_Mode = SPI_Master;
    SPI_PeriCLKControl(SPI1,ENABLE);
    SPI_Init(&spi_master);
    char *data = "Hello world\n";
    uint8_t dummy_data = 'd';
    uint8_t received_data;

	while(1)
	{
		//Wait until button is pressed
		while ( GPIO_ReadFromInputPin(GPIOC, 13) );

		//debounced effect
		delay_ms(250000);

		SPI_PeripheralControl(SPI1, ENABLE);
		//Sending command to the slave
		SPI_Write_Char(SPI1, (uint8_t) command);
		SPI_Write_Char(SPI1, (uint8_t) command);

		//Send dummy data to read from slave
		SPI_Write_Char(SPI1, (uint8_t) dummy_data);

		//Read data to Acknowledge Slave received the command
		SPI_ReadData(SPI1,&received_data,1);

		//Send back received data
		SPI_Write_Char(SPI1, (uint8_t) received_data);

		if(received_data == 0xF5){
			SPI_Write_String(SPI1, (uint8_t*) data, strlen(data));
		}

		//Confirming spi is not busy
		while( (SPI1->SPI_SR>>7)&1 );
		SPI_PeripheralControl(SPI1, DISABLE);
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

	/* MISO PIN CONFIG PA6 NO_REMAP*/
	GPIO_Handle_t	miso_pin;
	miso_pin.pGPIOx = GPIOA;
	miso_pin.GPIO_PinConfig.PinMode = GPIO_MODE_IN_PU_PD;
	miso_pin.GPIO_PinConfig.PinNumber = 6;
	miso_pin.GPIO_PinConfig.PinSpeed = GPIO_MODE_INPUT;
	miso_pin.GPIO_PinConfig.PinResistor = GPIO_PU;       /*Pull up resistor activated*/
	GPIO_Init(&miso_pin);

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
