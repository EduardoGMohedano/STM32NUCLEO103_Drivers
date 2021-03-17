/*
 * stm32f103.h
 *
 *  Created on: Dec 1, 2020
 *      Author: egarc
 */

#ifndef INC_STM32F103_H_
#define INC_STM32F103_H_


#include	<stdint.h>
#define		__vo		volatile

/*Defining main memories and peripherals base addresses*/
#define		FLASH_BASEADDR		0x08000000UL
#define		SRAM_BASEADDR		0x20000000UL
#define 	ROM_BASEADDR		0x1FFFF000UL		//System memory in datasheet
#define 	NVIC_BASEADDR	0xE000E100UL


/*
 * AHB and APB1 AND APB2 buses peripheral base addresses
 */
#define		AHB_BASEADDR		0x40018000UL
#define		APB2_BASEADDR		0x40010000UL
#define 	APB1_BASEADDR		0x40000000UL

/*
 * Peripherals base addresses hanging on APB2 bus
 */
#define		AFIO_BASEADDR		(APB2_BASEADDR + 0x0000)
#define		EXTI_BASEADDR		(APB2_BASEADDR + 0x0400)
#define		GPIOA_BASEADDR		(APB2_BASEADDR + 0x0800)
#define		GPIOB_BASEADDR		(APB2_BASEADDR + 0x0C00)
#define		GPIOC_BASEADDR		(APB2_BASEADDR + 0x1000)
#define		GPIOD_BASEADDR		(APB2_BASEADDR + 0x1400)
#define		GPIOE_BASEADDR		(APB2_BASEADDR + 0x1800)
#define		GPIOF_BASEADDR		(APB2_BASEADDR + 0x1C00)
#define		GPIOG_BASEADDR		(APB2_BASEADDR + 0x2000)
#define		ADC1_BASEADDR		(APB2_BASEADDR + 0x2400)
#define		ADC2_BASEADDR		(APB2_BASEADDR + 0x2800)
#define		TIM1_BASEADDR		(APB2_BASEADDR + 0x2C00)
#define		SPI1_BASEADDR		(APB2_BASEADDR + 0x3000)
#define		TIM8_BASEADDR		(APB2_BASEADDR + 0x3400)
#define		USART1_BASEADDR		(APB2_BASEADDR + 0x3800)
#define		ADC3_BASEADDR		(APB2_BASEADDR + 0x3C00)
#define		TIM9_BASEADDR		(APB2_BASEADDR + 0x4C00)
#define		TIM10_BASEADDR		(APB2_BASEADDR + 0x5000)
#define		TIM11_BASEADDR		(APB2_BASEADDR + 0x5400)

/*
 * Peripherals base addresses hanging on APB1 bus
 */
#define		TIM2_BASEADDR		(APB1_BASEADDR + 0x0000)
#define		TIM3_BASEADDR		(APB1_BASEADDR + 0x0400)
#define		TIM4_BASEADDR		(APB1_BASEADDR + 0x0800)
#define		TIM5_BASEADDR		(APB1_BASEADDR + 0x0C00)
#define		TIM6_BASEADDR		(APB1_BASEADDR + 0x1000)
#define		TIM7_BASEADDR		(APB1_BASEADDR + 0x1400)
#define		TIM12_BASEADDR		(APB1_BASEADDR + 0x1800)
#define		TIM13_BASEADDR		(APB1_BASEADDR + 0x1C00)
#define		TIM14_BASEADDR		(APB1_BASEADDR + 0x2000)
#define		RTC_BASEADDR		(APB1_BASEADDR + 0x2800)
#define		WWDOG_BASEADDR		(APB1_BASEADDR + 0x2C00)
#define		IWDG_BASEADDR		(APB1_BASEADDR + 0x3000)
#define		SPI2_BASEADDR		(APB1_BASEADDR + 0x3800)
#define		SPI3_BASEADDR		(APB1_BASEADDR + 0x3C00)
#define		USART2_BASEADDR		(APB1_BASEADDR + 0x4400)
#define		USART3_BASEADDR		(APB1_BASEADDR + 0x4800)
#define		UART4_BASEADDR		(APB1_BASEADDR + 0x4C00)
#define		UART5_BASEADDR		(APB1_BASEADDR + 0x5000)
#define		I2C1_BASEADDR		(APB1_BASEADDR + 0x5400)
#define		I2C2_BASEADDR		(APB1_BASEADDR + 0x5800)
#define		USB_BASEADDR		(APB1_BASEADDR + 0x5C00)
#define		USB_CAN_BASEADDR	(APB1_BASEADDR + 0x6000)
#define		bxCAN1_BASEADDR		(APB1_BASEADDR + 0x6400)
#define		bxCAN2_BASEADDR		(APB1_BASEADDR + 0x6800)
#define		BKP_REG_BASEADDR	(APB1_BASEADDR + 0x6C00)
#define		PWR_CTRL_BASEADDR	(APB1_BASEADDR + 0x7000)
#define		DAC_BASEADDR		(APB1_BASEADDR + 0x7400)

/*
 * Peripherals base addresses hanging on AHB bus
 */
#define		SDIO_BASEADDR			(AHB_BASEADDR + 0x0000)
#define		DMA1_BASEADDR			(AHB_BASEADDR + 0x8000)
#define		DMA2_BASEADDR			(AHB_BASEADDR + 0x8400)
#define		RCC_BASEADDR			(AHB_BASEADDR + 0x9000UL)
#define		FLASH_INTERFACE_BASEADDR (AHB_BASEADDR + 0xA000)
#define		CRC_BASEADDR			(AHB_BASEADDR + 0xB000)
#define		ETHERNET_BASEADDR		(AHB_BASEADDR + 0x10000)
#define		USB_OTG_BASEADDR		(AHB_BASEADDR + 0xFFE8000)
#define		FSMC_BASEADDR			(AHB_BASEADDR + 0x5FFE8000)


/*********************************************Peripheral register definition structures*******************************************/

typedef struct{
	__vo uint32_t	CRL;			/*GPIO PORT MODE REGISTERS		ADDRESS OFFSET: 0X00*/
	__vo uint32_t	CRH;
	__vo uint32_t	IDR;
	__vo uint32_t	ODR;
	__vo uint32_t	BSRR;
	__vo uint32_t	BRR;
	__vo uint32_t	LCKR;
}GPIO_RegDef_t;


typedef struct{
	__vo uint32_t	RCC_CR;			/*RCC REGISTERS					ADDRESS OFFSET: 0X00*/
	__vo uint32_t	RCC_CFGR;
	__vo uint32_t	RCC_CIR;
	__vo uint32_t	RCC_APB2RSTR;
	__vo uint32_t	RCC_APB1RSTR;
	__vo uint32_t	RCC_AHBENR;
	__vo uint32_t	RCC_APB2ENR;
	__vo uint32_t	RCC_APB1ENR;
	__vo uint32_t	RCC_BDCR;
	__vo uint32_t	RCC_CSR;
}RCC_RegDef_t;

//Structure for SPI
typedef struct{
	__vo uint32_t	SPI_CR1;			/*SPI REGISTERS					ADDRESS OFFSET: 0X00*/
	__vo uint32_t	SPI_CR2;
	__vo uint32_t	SPI_SR;
	__vo uint32_t	SPI_DR;
	__vo uint32_t	SPI_CRCPR;
	__vo uint32_t	SPI_RXCRCR;
	__vo uint32_t	SPI_TXCRCR;
}SPI_RegDef_t;

//Structure for USART
typedef struct{
	__vo uint32_t	UART_SR;			/*USART REGISTERS				ADDRESS OFFSET: 0X00*/
	__vo uint32_t	UART_DR;
	__vo uint32_t	UART_BRR;
	__vo uint32_t	UART_CR1;
	__vo uint32_t	UART_CR2;
	__vo uint32_t	UART_CR3;
	__vo uint32_t	UART_GTPR;
}UART_RegDef_t;


/*
 * Peripheral definitions (peripheral base address typecasted to GPIO_RegDef_t)
 */
#define		GPIOA		((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define		GPIOB		((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define		GPIOC		((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define		GPIOD		((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define		GPIOE		((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define		GPIOF		((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define		GPIOG		((GPIO_RegDef_t*) GPIOG_BASEADDR)

/*
 * RCC definition typecasted
 */
#define		RCC			((RCC_RegDef_t*) RCC_BASEADDR)
//uint8_t		CLK_SRC = 0;

/*
 * Peripheral macros to enable GPIO clock
 */
#define		GPIOA_PCLK_EN()			RCC->RCC_APB2ENR |= (1<<2)
#define		GPIOB_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<3) )
#define		GPIOC_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<4) )
#define		GPIOD_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<5) )
#define		GPIOE_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<6) )
#define		GPIOF_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<7) )
#define		GPIOG_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<8) )

/*
 * Peripheral macros to disable GPIO clock
 */
#define		GPIOA_PCLK_DIS()			(RCC->RCC_APB2ENR &= ~(1<<2) )
#define		GPIOB_PCLK_DIS()			(RCC->RCC_APB2ENR &= ~(1<<3) )
#define		GPIOC_PCLK_DIS()			(RCC->RCC_APB2ENR &= ~(1<<4) )
#define		GPIOD_PCLK_DIS()			(RCC->RCC_APB2ENR &= ~(1<<5) )
#define		GPIOE_PCLK_DIS()			(RCC->RCC_APB2ENR &= ~(1<<6) )
#define		GPIOF_PCLK_DIS()			(RCC->RCC_APB2ENR &= ~(1<<7) )
#define		GPIOG_PCLK_DIS()			(RCC->RCC_APB2ENR &= ~(1<<8) )

/*
 * Peripheral macros to reset GPIO registers
 */
#define		GPIOA_REG_RESET()			(RCC->RCC_APB2RSTR |= (1<<2) )
#define		GPIOB_REG_RESET()			(RCC->RCC_APB2RSTR |= (1<<3) )
#define		GPIOC_REG_RESET()			(RCC->RCC_APB2RSTR |= (1<<4) )
#define		GPIOD_REG_RESET()			(RCC->RCC_APB2RSTR |= (1<<5) )
#define		GPIOE_REG_RESET()			(RCC->RCC_APB2RSTR |= (1<<6) )
#define		GPIOF_REG_RESET()			(RCC->RCC_APB2RSTR |= (1<<7) )
#define		GPIOG_REG_RESET()			(RCC->RCC_APB2RSTR |= (1<<8) )


/*
 * SPI Peripheral definitions (peripheral base address typecasted to SPI_RegDef_t)
 */
#define	SPI1	((SPI_RegDef_t*) SPI1_BASEADDR)
#define	SPI2	((SPI_RegDef_t*) SPI2_BASEADDR)
#define	SPI3	((SPI_RegDef_t*) SPI3_BASEADDR)


/*
 * Peripheral macros to enable SPI clock
 */
#define		SPI1_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<12) )
#define		SPI2_PCLK_EN()			(RCC->RCC_APB1ENR |= (1<<14) )
#define		SPI3_PCLK_EN()			(RCC->RCC_APB1ENR |= (1<<15) )


/*
 * Peripheral macros to disable SPI clock
 */
#define		SPI1_PCLK_DIS()			(RCC->RCC_APB2ENR &= ~(1<<12) )
#define		SPI2_PCLK_DIS()			(RCC->RCC_APB1ENR &= ~(1<<14) )
#define		SPI3_PCLK_DIS()			(RCC->RCC_APB1ENR &= ~(1<<15) )


/*
 * Peripheral macros to reset SPI Registers
 */
#define		SPI1_REG_RESET()			(RCC->RCC_APB2RSTR |= (1<<12) )
#define		SPI2_REG_RESET()			(RCC->RCC_APB1RSTR |= (1<<14) )
#define		SPI3_REG_RESET()			(RCC->RCC_APB1RSTR |= (1<<15) )


/*
 * Peripheral Macros to enable and disable alternate function IO clock enable
 */
#define		AFIO_PCLK_EN()			RCC->RCC_APB2ENR |=  (1<<0)
#define		AFIO_PCLK_DIS()			RCC->RCC_APB2ENR &= ~(1<<0)


/*
 * UART Peripheral definitions (peripheral base address typecasted to USART_RegDef_t)
 */
#define		UART1		((UART_RegDef_t*) USART1_BASEADDR)
#define		UART2		((UART_RegDef_t*) USART2_BASEADDR)
#define		UART3		((UART_RegDef_t*) USART3_BASEADDR)
#define		UART4		((UART_RegDef_t*) UART4_BASEADDR)
#define		UART5		((UART_RegDef_t*) UART5_BASEADDR)

/*
 * Peripheral macros to enable and disable USART peripherals
 */
#define		UART1_PCLK_EN()			RCC->RCC_APB2ENR |= (1<<14)
#define		UART2_PCLK_EN()			RCC->RCC_APB1ENR |= (1<<17)
#define		UART3_PCLK_EN()			RCC->RCC_APB1ENR |= (1<<18)
#define		UART4_PCLK_EN()			RCC->RCC_APB1ENR |= (1<<19)
#define		UART5_PCLK_EN()			RCC->RCC_APB1ENR |= (1<<20)
#define		UART1_PCLK_DIS()		RCC->RCC_APB2ENR &= ~(1<<14)
#define		UART2_PCLK_DIS()		RCC->RCC_APB1ENR &= ~(1<<17)
#define		UART3_PCLK_DIS()		RCC->RCC_APB1ENR &= ~(1<<18)
#define		UART4_PCLK_DIS()		RCC->RCC_APB1ENR &= ~(1<<19)
#define		UART5_PCLK_DIS()		RCC->RCC_APB1ENR &= ~(1<<20)



/*
 * EXTI Peripheral definitions (peripheral base address typecasted to EXTI_RegDef_t)
 */
#define	EXTI	((EXTI_RegDef_t*) EXTI_BASEADDR)


/********************************************Struct definition to access EXTI**********************************************/

typedef struct{
	__vo uint32_t	EXTI_IMR;			/*EXTERNAL INTERRUPT REGISTERS		ADDRESS OFFSET: 0X00*/
	__vo uint32_t	EXTI_EMR;
	__vo uint32_t	EXTI_RTSR;
	__vo uint32_t	EXTI_FTSR;
	__vo uint32_t	EXTI_SWIER;
	__vo uint32_t	EXTI_PR;
}EXTI_RegDef_t;

#define	EXTI	((EXTI_RegDef_t*) EXTI_BASEADDR)



/****************************************Struct definition to access Alternate function****************************************/
typedef struct{
	__vo uint32_t	AFIO_EVCR;			/*Alternate FUNCTION REGISTERS		ADDRESS OFFSET: 0X00*/
	__vo uint32_t	AFIO_MAPR;
	__vo uint32_t	AFIO_EXTICR[4];
	__vo uint32_t	AFIO_MAPR2;
}AFIO_RegDef_t;

#define		AFIO		((AFIO_RegDef_t*) AFIO_BASEADDR)

#define 	AFIO_PCLK_EN()				RCC->RCC_APB2ENR |=  (1<<0)
#define 	AFIO_PCLK_DIS()				RCC->RCC_APB2ENR &= ~(1<<0)



/******************************************Struct definition to access NVIC CONTROLLER*******************************************/
typedef struct{
	__vo uint32_t iser[3];	/* 00 */
	__vo uint32_t icer[3];	/* 0c */
	__vo uint32_t ispr[3];	/*Interrupt Set Pending reg*/
	__vo uint32_t icpr[3];	/*Interrupt Clear Pending reg*/
}NVIC_RegDef_t;

typedef struct{
	__vo uint32_t ipr[21];	/* Interrupt priority register*/
}NVIC_Prior_RegDef_t;

#define NVIC			((NVIC_RegDef_t *) NVIC_BASEADDR)
#define NVIC_PRIOR		((NVIC_Prior_RegDef_t *) (NVIC_BASEADDR + 0x300))

/*
 * Returns port code (between 0 and 7) for a given GPIO base address(x)
 */

#define		GPIO_BASEADDR_TO_CODE(x)		( (x==GPIOA)?0:\
										  	  (x==GPIOB)?1:\
										  	  (x==GPIOC)?2:\
										      (x==GPIOD)?3:\
										      (x==GPIOE)?4:\
										      (x==GPIOF)?5:\
										      (x==GPIOG)?6:0 )



/*
 * IRQ NUMBERS of STM32F103 MCU
 * TO DO: COMPLETE THE LIST FOR THE REST OF PERIPHERALS
 */
#define	IRQ_NO_EXTI0		6
#define	IRQ_NO_EXTI1		7
#define	IRQ_NO_EXTI2		8
#define	IRQ_NO_EXTI3		9
#define	IRQ_NO_EXTI4		10
#define	IRQ_NO_EXTI9_5		23
#define	IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_UART1		37
#define IRQ_NO_UART2		38
#define IRQ_NO_UART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53


/*
 * Bit position definitions for SPI PERIPHERAL
 */

#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MODE		2
#define SPI_CR1_BR			3
#define SPI_CR1_ENABLE		6
#define SPI_CR1_DFF			7
#define SPI_CR1_BUFFER_S	11
#define SPI_SR_TXE			1	//Transmit Buffer empty flag


/*
 * Generic macros
 */


#define		ENABLE			1
#define 	DISABLE			0
#define		SET				ENABLE
#define 	RESET			DISABLE
#define 	GPIO_PIN_SET	SET
#define		GPIO_PIN_RESET	RESET
#define 	RISING_EDGE		1
#define		FALLING_EDGE	0


#endif /* INC_STM32F103_H_ */
