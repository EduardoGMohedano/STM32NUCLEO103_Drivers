/*
 * SPI_RX_TX_IT.c
 *	This example will fetch data from an arduino board( used as slave), STM32 board will be the master and reads the data based on interrupt
 *	APIs. The STM32 board reading will be triggered by an external interrupt pin.
 *  Created on: 25 abr. 2021
 *      Author: egarc
 */


#include <stdint.h>
#include <string.h>
#include "stm32f103.h"
#include "stm32f103_gpio_driver.h"
#include "stm32f103_spi_driver.h"

void config_spi1_pins(void);
void config_spi1(void);
void config_interrupt_pin(void);

uint8_t DataAvailable = 0, rcvStop;
uint8_t dummy_data = 'd';
uint8_t received_data;
SPI_Handle_t spi_master;
uint8_t RcvBuff[200];

int main(void)
{

	while( ! (RCC->RCC_CR>>1)&1 );	//Wait until internal clock is in steady state

	//Configure pin to generate interupt on falling edge
	config_interrupt_pin();

    config_spi1_pins();

    config_spi1();

    SPI_IRQConfig(IRQ_NO_SPI1, 1, ENABLE);

    while(1)
	{
		rcvStop = 0;

		while(! DataAvailable);		//wait until data is available

		GPIO_IRQConfig(IRQ_NO_EXTI15_10,1,DISABLE);

		SPI_PeripheralControl(SPI1, ENABLE);

		//Execute commands to fetch data
		while( !rcvStop ){
			while( SPI_Write_StringIT(&spi_master, &dummy_data, 1) == SPI_BUSY_IN_TX );
			while( SPI_ReadDataIT(&spi_master, &received_data, 1) == SPI_BUSY_IN_RX );

		}

		//Confirming spi is not busy
		while( (SPI1->SPI_SR>>7)&1 );

		SPI_PeripheralControl(SPI1, DISABLE);

		DataAvailable = 0;

		GPIO_IRQConfig(IRQ_NO_EXTI15_10,1,ENABLE);
	}
}

void config_interrupt_pin(void){
	/* External pin interrupt config */
	GPIO_Handle_t	IT_pin;
	IT_pin.pGPIOx = GPIOA;
	IT_pin.GPIO_PinConfig.PinMode = GPIO_MODE_IN_PU_PD;
	IT_pin.GPIO_PinConfig.PinNumber = 10;
	IT_pin.GPIO_PinConfig.PinSpeed = GPIO_MODE_INPUT;
	IT_pin.GPIO_PinConfig.PinResistor = GPIO_PU;       /*Pull up resistor activated*/
	GPIO_Init(&IT_pin);

	/*External interrupt configuration*/
	GPIO_IRQ_Mode(GPIOA, 10,FALLING_EDGE);
	GPIO_IRQConfig(IRQ_NO_EXTI15_10,1,ENABLE);
}

void config_spi1_pins(void){
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


void config_spi1(void){

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

	SPI_IRQ_Mode(SPI1, SPI_IRQ_TX_BUFFER_EMPTY, ENABLE);
	SPI_IRQ_Mode(SPI1, SPI_IRQ_RX_BUFFER_NOEMPTY, ENABLE);
}

void EXTI15_10_IRQHandler(void){
	GPIO_IRQHandling(10);
	DataAvailable = 1;
}

void SPI_ApplicationEventCallback(SPI_Handle_t* pSPIHandle,uint8_t AppEv){
	static uint32_t i = 0;
	if(AppEv == SPI_EVENT_RX_CMPLT){
		RcvBuff[i++] = received_data;
		if( received_data == '\0' || (i == 200) ){
			rcvStop = 1;
			RcvBuff[i-1] = '\0' ;
			i = 0;
		}
	}
}

void SPI1_IRQHandler(void){
	SPI_IRQHandling(&spi_master);
}
