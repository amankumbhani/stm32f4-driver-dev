#ifndef STM32F411RE_H
#define STM32F411RE_H

#include <stdint.h>
#include <stdbool.h>

#define HSI														((uint32_t)16000000U)			/** 16 MHz */
#define HSE														((uint32_t)8000000U)			/** 8 MHz */

#define FLASH													(uint32_t *)(0x08000000U)
#define SYSTEM_MEMORY											(uint32_t *)(0x1FFF0000U)
#define OTP_AREA												(uint32_t *)(0x1FFF7800U)
#define OPTION_BYTES											(uint32_t *)(0x1FFFC000U)

#define NVIC_ISER0												(uint32_t *)(0xE000E100U)
#define NVIC_ISER1												(uint32_t *)(0xE000E104U)
#define NVIC_ISER2												(uint32_t *)(0xE000E108U)
#define NVIC_ISER3												(uint32_t *)(0xE000E10CU)
#define NVIC_ISER4												(uint32_t *)(0xE000E110U)
#define NVIC_ISER5												(uint32_t *)(0xE000E114U)
#define NVIC_ISER6												(uint32_t *)(0xE000E118U)
#define NVIC_ISER7												(uint32_t *)(0xE000E11CU)

#define NVIC_ICER0												(uint32_t *)(0xE000E180U)
#define NVIC_ICER1												(uint32_t *)(0xE000E184U)
#define NVIC_ICER2												(uint32_t *)(0xE000E188U)
#define NVIC_ICER3												(uint32_t *)(0xE000E18CU)
#define NVIC_ICER4												(uint32_t *)(0xE000E190U)

#define NVIC_ISPR0												(uint32_t *)(0xE000E200U)
#define NVIC_ISPR1												(uint32_t *)(0xE000E204U)
#define NVIC_ISPR2												(uint32_t *)(0xE000E208U)
#define NVIC_ISPR3												(uint32_t *)(0xE000E20CU)
#define NVIC_ISPR4												(uint32_t *)(0xE000E210U)

#define NVIC_ICPR0												(uint32_t *)(0xE000E280U)
#define NVIC_ICPR1												(uint32_t *)(0xE000E284U)
#define NVIC_ICPR2												(uint32_t *)(0xE000E288U)
#define NVIC_ICPR3												(uint32_t *)(0xE000E28CU)
#define NVIC_ICPR4												(uint32_t *)(0xE000E290U)

#define NVIC_IPR0												(uint32_t *)(0xE000E400U)
#define NVIC_IPR1												(uint32_t *)(0xE000E404U)
#define NVIC_IPR2												(uint32_t *)(0xE000E408U)
#define NVIC_IPR3												(uint32_t *)(0xE000E40CU)

#define STIR													(uint32_t *)(0xE000EF00U)

typedef enum
{
	E_SUCCESS,
	E_FAIL
} HAL_Status;

typedef enum
{
	E_DI,
	E_EN
} HAL_LogicLevel;

/**
 * \brief GPIO Configuration Registers
 */
typedef struct
{
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSSR;
	uint32_t LCKR;
	uint32_t AFR[2];
} GPIOx_t;

typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SR;
	uint32_t DR;
	uint32_t CRCPR;
	uint32_t RXCRCR;
	uint32_t TXCRCR;
	uint32_t I2SCFGR;
	uint32_t I2SPR;
} SPIx_t;

/**
 * \brief RCC Configuration Registers
 */
typedef struct
{
	uint32_t CR;
	uint32_t PLLCFGR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t RESERVED0[2];
	uint32_t APB1RSTR;
	uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t RESERVED2[2];
	uint32_t APB1ENR;
	uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t RESERVED4[2];
	uint32_t APB1LPENR;
	uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t RESERVED6[2];
	uint32_t SSCGR;
	uint32_t PLLI2SCFGR;
	uint32_t RESERVED7;
	uint32_t DCKCFGR;
} RCC_t;

typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t OAR1;
	uint32_t OAR2;
	uint32_t DR;
	uint32_t SR1;
	uint32_t SR2;
	uint32_t CCR;
	uint32_t TRISE;
	uint32_t FLTR;
} I2Cx_t;

typedef struct
{
	uint32_t SR;
	uint32_t DR;
	uint32_t BRR;
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t GTPR;
} USARTx_t;

typedef struct
{
	uint32_t IMR;
	uint32_t EMR;
	uint32_t RSTR;
	uint32_t FTSR;
	uint32_t SWIER;
	uint32_t PR;
} EXTI_t;

typedef struct
{
	uint32_t MEMRMP;
	uint32_t PMC;
	uint32_t EXTICR[4];
	uint32_t CMPCR;
} SYSCFG_t;

#define CORTEXM4_INTERNAL_PERIPHERALS_BASEADDR					(0xE0000000U)
#define AHB2_BASEADDR											(0x50000000U)
#define AHB1_BASEADDR											(0x40020000U)
#define APB2_BASEADDR											(0x40007400U)
#define APB1_BASEADDR											(0x40000000U)

#define USB_OTG_FS_OFFSET										(0x00000000U)
#define USB_OTG_FS_BASEADDR										(AHB2_BASEADDR + USB_OTG_FS_OFFSET)

#define GPIOA_OFFSET											(0x0000U)
#define GPIOB_OFFSET											(0x0400U)
#define GPIOC_OFFSET											(0x0800U)
#define GPIOD_OFFSET											(0x0C00U)
#define GPIOE_OFFSET											(0x1000U)
#define GPIOH_OFFSET											(0x1C00U)
#define CRC_OFFSET												(0x3000U)
#define RCC_OFFSET												(0x3800U)
#define FLASH_INTERFACE_OFFSET									(0x3C00U)
#define DMA1_OFFSET												(0x6000U)
#define DMA2_OFFSET												(0x6400U)

#define GPIOA_BASEADDR											(AHB1_BASEADDR + GPIOA_OFFSET)
#define GPIOB_BASEADDR											(AHB1_BASEADDR + GPIOB_OFFSET)
#define GPIOC_BASEADDR											(AHB1_BASEADDR + GPIOC_OFFSET)
#define GPIOD_BASEADDR											(AHB1_BASEADDR + GPIOD_OFFSET)
#define GPIOE_BASEADDR											(AHB1_BASEADDR + GPIOE_OFFSET)
#define GPIOH_BASEADDR											(AHB1_BASEADDR + GPIOH_OFFSET)

#define RCC_BASEADDR											(AHB1_BASEADDR + RCC_OFFSET)
#define FLASH_INTERFACE_BASEADDR								(AHB1_BASEADDR + FLASH_INTERFACE_OFFSET)
#define DMA1_BASEADDR											(AHB1_BASEADDR + DMA1_OFFSET)
#define DMA2_BASEADDR											(AHB1_BASEADDR + DMA2_OFFSET)

#define TIM1_OFFSET												(0x8C00U)
#define USART1_OFFSET											(0x9C00U)
#define USART6_OFFSET											(0xCD00U)
#define ADC1_OFFSET												(0xAC00U)
#define SDIO_OFFSET												(0xB800U)
#define SPI1_OFFSET												(0xBC00U)
#define SPI4_OFFSET												(0xC000U)
#define SYSCFG_OFFSET											(0xC400U)
#define EXTI_OFFSET												(0xC800U)
#define TIM9_OFFSET												(0xCC00U)
#define TIM10_OFFSET											(0xD000U)
#define TIM11_OFFSET											(0xD400U)
#define SPI5_OFFSET												(0xDC00U)

#define TIM1_BASEADDR											(APB2_BASEADDR + TIM1_OFFSET)
#define USART1_BASEADDR											(APB2_BASEADDR + USART1_OFFSET)
#define USART6_BASEADDR											(APB2_BASEADDR + USART6_OFFSET)
#define ADC1_BASEADDR											(APB2_BASEADDR + ADC1_OFFSET)
#define SDIO_BASEADDR											(APB2_BASEADDR + SDIO_OFFSET)
#define SPI1_BASEADDR											(APB2_BASEADDR + SPI1_OFFSET)
#define SPI4_BASEADDR											(APB2_BASEADDR + SPI4_OFFSET)
#define SYSCFG_BASEADDR											(APB2_BASEADDR + SYSCFG_OFFSET)
#define EXTI_BASEADDR											(APB2_BASEADDR + EXTI_OFFSET)
#define TIM9_BASEADDR											(APB2_BASEADDR + TIM9_OFFSET)
#define TIM10_BASEADDR											(APB2_BASEADDR + TIM10_OFFSET)
#define TIM11_BASEADDR											(APB2_BASEADDR + TIM11_OFFSET)
#define SPI5_BASEADDR											(APB2_BASEADDR + SPI5_OFFSET)

#define TIM2_OFFSET												(0x0000U)
#define TIM3_OFFSET												(0x0400U)
#define TIM4_OFFSET												(0x0800U)
#define TIM5_OFFSET												(0x0C00U)
#define RTC_BKP_OFFSET											(0x2800U)
#define WWDG_OFFSET												(0x2C00U)
#define IWDG_OFFSET												(0x3000U)
#define I2S2EXT_OFFSET											(0x3400U)
#define SPI2_OFFSET												(0x3800U)
#define SPI3_OFFSET												(0x3C00U)
#define I2S3EXT_OFFSET											(0x4000U)
#define USART2_OFFSET											(0x4400U)
#define I2C1_OFFSET												(0x5400U)
#define I2C2_OFFSET												(0x5800U)
#define I2C3_OFFSET												(0x5C00U)
#define PWR_OFFSET												(0x7000U)

#define TIM2_BASEADDR											(APB1_BASEADDR + TIM2_OFFSET)
#define TIM3_BASEADDR											(APB1_BASEADDR + TIM3_OFFSET)
#define TIM4_BASEADDR											(APB1_BASEADDR + TIM4_OFFSET)
#define TIM5_BASEADDR											(APB1_BASEADDR + TIM5_OFFSET)
#define RTC_BKP_BASEADDR										(APB1_BASEADDR + RTC_BKP_OFFSET)
#define WWDG_BASEADDR											(APB1_BASEADDR + WWDG_OFFSET)
#define IWDG_BASEADDR											(APB1_BASEADDR + IWDG_OFFSET)
#define I2S2EXT_BASEADDR										(APB1_BASEADDR + I2S2EXT_OFFSET)
#define SPI2_BASEADDR											(APB1_BASEADDR + SPI2_OFFSET)
#define SPI3_BASEADDR											(APB1_BASEADDR + SPI3_OFFSET)
#define I2S3EXT_BASEADDR										(APB1_BASEADDR + I2S3EXT_OFFSET)
#define USART2_BASEADDR											(APB1_BASEADDR + USART2_OFFSET)
#define I2C1_BASEADDR											(APB1_BASEADDR + I2C1_OFFSET)
#define I2C2_BASEADDR											(APB1_BASEADDR + I2C2_OFFSET)
#define I2C3_BASEADDR											(APB1_BASEADDR + I2C3_OFFSET)
#define PWR_BASEADDR											(APB1_BASEADDR + PWR_OFFSET)

#define GPIOA													((GPIOx_t *)GPIOA_BASEADDR)
#define GPIOB													((GPIOx_t *)GPIOB_BASEADDR)
#define GPIOC													((GPIOx_t *)GPIOC_BASEADDR)
#define GPIOD													((GPIOx_t *)GPIOD_BASEADDR)
#define GPIOE													((GPIOx_t *)GPIOE_BASEADDR)
#define GPIOH													((GPIOx_t *)GPIOH_BASEADDR)

#define SPI1													((SPIx_t *)SPI1_BASEADDR)
#define SPI2													((SPIx_t *)SPI2_BASEADDR)
#define SPI3													((SPIx_t *)SPI3_BASEADDR)

#define I2C1													((I2Cx_t *)I2C1_BASEADDR)
#define I2C2													((I2Cx_t *)I2C2_BASEADDR)
#define I2C3													((I2Cx_t *)I2C3_BASEADDR)

#define USART1													((USARTx_t *)USART1_BASEADDR)
#define USART2													((USARTx_t *)USART2_BASEADDR)

#define RCC														((RCC_t *)RCC_BASEADDR)

#define EXTI													((EXTI_t *)EXTI_BASEADDR)

#define SYSCFG													((SYSCFG_t *)SYSCFG_BASEADDR)


/************************* BIT DEFINITIONS FOR SPI PERIPHERAL *************************/
#define BIDIMODE												(15U)
#define BIDIOE													(14U)
#define CRCEN													(13U)
#define CRCNEXT													(12U)
#define DFF														(11U)
#define RXONLY													(10U)
#define SSM														(9U)
#define SSI														(8U)
#define LSBFIRST												(7U)
#define SPE														(6U)
#define BAUDRATECONTROL											(3U)
#define MSTR													(2U)
#define CPOL													(1U)
#define CPHA													(0U)

#define SPI_SR_TXE_MASK											(1 << 1U)
#define SPI_SR_BUSY_MASK										(1 << 7U)
#define SPI_SR_RXNE_MASK										(1 << 0U)

#define SPI_CR1_DFF_MASK										(1 << DFF)


/************************* BIT DEFINITIONS FOR RCC *************************/
#define RCC_CFGR_PPRE1_MASK										(0x1C00U)		/** PRESCALER FOR APB1 */
#define RCC_CFGR_HPRE_MASK										(0x00F0U)		/** PRESCALER FOR AHB */
#define RCC_CFGR_SWS_MASK										(0xCU)		    /** SYSTEM CLOCK SWITCH */
#define RCC_PLLCFGR_PLLSRC										(1 << 22U) 		/** MAIN PLL & AUDIO PLL ENTRY CLOCK SOURCE */
#define RCC_PLLCFGR_PLLM										(0x3FU)			/** DIVISION FACTOR FOR MAIN PLL INPUT CLOCK */
#define RCC_PLLCFGR_PLLP										(0x30000U) 		/** MAIN PLL DIVISION FACTOR FOR MAIN SYSTEM CLOCK */
#define RCC_PLLCFGR_PLLN										(0x7FC0U)		/** MAIN PLL MULTIPLICATION FACTOR FOR VCO */


/************************* BIT DEFINITIONS FOR I2C *************************/
#define I2C_CR1_START_BIT_POS									(8U)		/** START BIT POSITION IN I2C CR1 REG */
#define I2C_CR1_START_BIT_MASK									(1 << 8)
#define I2C_CR1_STOP_BIT_POS									(9U)		/** STOP BIT POSITION IN I2C CR1 REG */
#define I2C_CR1_STOP_BIT_MASK									(1 << 9)
#define I2C_SR1_TXE_BIT_POS										(7U)
#define I2C_SR1_TXE_BIT_MASK									(1 << 7)
#define I2C_SR1_RXNE_BIT_POS									(6U)
#define I2C_SR1_RXNE_BIT_MASK									(1 << 6)
#define I2C_SR1_BTF_BIT_POS										(2U)
#define I2C_SR1_BTF_BIT_MASK									(1 << 2)
#define I2C_SR1_ADDR_BIT_POS									(1U)
#define I2C_SR1_ADDR_BIT_MASK									(1 << 1)
#define I2C_SR1_START_BIT_POS									(0U)
#define I2C_SR1_START_BIT_MASK									(1 << 0)


/************************* BIT DEFINITIONS FOR USART *************************/
#define USART_CR1_UE_BIT_POS									(13U)
#define USART_CR1_UE_BIT_MASK									(1U << USART_CR1_UE_BIT_POS)
#define USART_CR1_M_BIT_POS										(12U)
#define USART_CR1_M_BIT_MASK									(1U << USART_CR1_M_BIT_POS)
#define USART_CR1_PS_BIT_POS									(9U)
#define USART_CR1_PS_BIT_MASK									(1 << USART_CR1_PS_BIT_POS)
#define USART_CR1_PCE_BIT_POS									(10U)
#define USART_CR1_PCE_BIT_MASK									(1 << USART_CR1_PCE_BIT_POS)
#define USART_CR1_RE_BIT_POS									(2U)
#define USART_CR1_RE_BIT_MASK									(1 << USART_CR1_RE_BIT_POS)
#define USART_CR1_TE_BIT_POS									(3U)
#define USART_CR1_TE_BIT_MASK									(1 << USART_CR1_TE_BIT_POS)
#define USART_CR2_STOP_BIT_POS									(12U)
#define USART_CR2_STOP_BIT_MASK									(1 << USART_CR2_STOP_BIT_POS)
#define USART_SR_TXE_BIT_POS									(7U)
#define USART_SR_TXE_BIT_MASK									(1 << USART_SR_TXE_BIT_POS)
#define USART_SR_RXNE_BIT_POS									(5U)
#define USART_SR_RXNE_BIT_MASK									(1 << USART_SR_RXNE_BIT_POS)
#define USART_SR_TC_BIT_POS										(6U)
#define USART_SR_TC_BIT_MASK									(1 << USART_SR_TC_BIT_POS)
#define USART_CR1_OVR8_BIT_POS									(15U)
#define USART_CR1_OVR8_BIT_MASK									(1 << USART_CR1_OVR8_BIT_POS)

/**
 * \brief This function calculates the output of the PLL clock
 * \return Value of the output clock
 */
uint32_t RCC_GetPLLOutputClock(void);

/**
 * \brief This function calculates the PLK1 clock
 * \return Value of the PLK1 clock
 */
uint32_t RCC_GetPLK1Clock(void);

#endif
