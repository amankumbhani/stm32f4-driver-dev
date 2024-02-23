#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "stm32f411re.h"

#define SPI1_PERIPH_CLK_EN()				(RCC->APB2ENR |= (1U << 12U))
#define SPI1_PERIPH_CLK_DI()				(RCC->APB2ENR &= ~(1U << 12U))

#define SPI2_PERIPH_CLK_EN()				(RCC->APB1ENR |= (1U << 14U))
#define SPI2_PERIPH_CLK_DI()				(RCC->APB1ENR &= ~(1U << 14U))

#define SPI3_PERIPH_CLK_EN()				(RCC->APB1ENR |= (1U << 15U))
#define SPI3_PERIPH_CLK_DI()				(RCC->APB1ENR &= ~(1U << 15U))

typedef enum
{
	E_SPI_MODE_0,
	E_SPI_MODE_1,
	E_SPI_MODE_2,
	E_SPI_MODE_3,
} SPI_Mode_e;

typedef enum
{
	E_SPI_CLK_DIV_2 = 0,
	E_SPI_CLK_DIV_4,
	E_SPI_CLK_DIV_8,
	E_SPI_CLK_DIV_16,
	E_SPI_CLK_DIV_32,
	E_SPI_CLK_DIV_64,
	E_SPI_CLK_DIV_128,
	E_SPI_CLK_DIV_256,
} SPI_ClockSpeed_e;

typedef enum
{
	E_SPI_SLAVE = 0,
	E_SPI_MASTER,
} SPI_DeviceConfig_e;

typedef enum
{
	E_SPI_FULL_DUPLEX,
	E_SPI_HALF_DUPLEX,
	E_SPI_SIMPLEX
} SPI_BusConfig_e;

typedef enum
{
	E_8BIT_DATA,
	E_16BIT_DATA,
} SPI_DataFormat_e;

typedef enum
{
	E_NSS_SSM_DI,
	E_NSS_SSM_EN,
} SPI_NssConfig_e;

/**
 * API requirements for SPI
 *
 * SPI configuration
 * 	SPI Mode (master or slave)
 * 	SPI Bus Config (full duplex, half duplex, simplex)
 * 	SPI Data Frame Format (8 bit or 16 bit data)
 * 	SPI CHPA (clock phase)
 * 	SPI CPOL (clock polarity)
 * 	SPI SSM (slave select mode - hardware or software)
 * 	SPI Speed (speed of communication FPCLK / 2 == MAX)
 * SPI TX
 * SPI RX
 * SPI interrupt configuration
 */

typedef struct
{
	SPI_DeviceConfig_e e_deviceConfig; 				/** Master or slave */
	SPI_Mode_e e_commMode;							/** Communication mode 0, 1, 2, 3 */
	SPI_BusConfig_e e_busConfig;					/** Bus configuration full duplex, half duplex or simplex */
	SPI_DataFormat_e e_dataFormat;					/** Data format 8 bit or 16 bit data */
	SPI_ClockSpeed_e e_clockSpeed;					/** Baud rate for communication */
	SPI_NssConfig_e e_nssConfig;
} SPI_Configuration;

typedef struct
{
	SPIx_t * SPIx;
	SPI_Configuration spiConfiguration;
} SPI_Info_t;

/**
 * \brief A function to initialise SPI communication
 */
void SPIInitialize(SPI_Info_t * spiConfigInfo);

/**
 * \brief A function to send data over SPI
 */
void SPISendData(SPIx_t SPIx, uint8_t * dataBuf, uint8_t dataLen);

/**
 * \brief A fucntion used to enable the SPI peripheral
 */
void SPIEnablePeripheral(SPIx_t SPIx, HAL_LogicLevel enDi);

/**
 * \brief A function that enables the peripheral clock for the input SPI
 */
void SPIEnablePeriphClk(SPIx_t * SPIx, HAL_LogicLevel enDi);

/**
 * \brief A function that enables / disables an IRQ
 */
void SPIConfigureIRQn(uint8_t IRQn, HAL_LogicLevel enDi);

/**
 * \brief A function that sets the interrupt priority for a given IRQ number
 */
void SPISetIRQPriority(uint8_t IRQn, uint8_t priorityNumber);

/**
 * \brief A function that handles interrupts
 */
void SPIIRQHandler(uint8_t pinNumber);

/**
 * \brief A function that enables SSI
 */
void SPISSIConfig(SPIx_t SPIx, HAL_LogicLevel enDi);
#endif
