#include "spi_driver.h"

void SPIInitialize(SPI_Info_t * spiConfigInfo)
{
	// TODO: Replace all the below numbers with bit macros
	uint16_t tempReg = 0;

	/** Configure the device as a master or a slave */
	tempReg |= (spiConfigInfo->spiConfiguration.e_deviceConfig << 2U);

	/** Configure the data format */
	tempReg |= (spiConfigInfo->spiConfiguration.e_dataFormat << 11U);

	/** Configure the bus configuration, i.e., full, half, simplex */
	if (spiConfigInfo->spiConfiguration.e_busConfig == E_SPI_FULL_DUPLEX)
	{
		/** Clear BIDIMODE */
		tempReg &= ~(1U << 15U);
	}
	else if (spiConfigInfo->spiConfiguration.e_busConfig == E_SPI_HALF_DUPLEX)
	{
		/** Set BIDIMODE */
		tempReg |= (1U << 15U);
	}
	else if (spiConfigInfo->spiConfiguration.e_busConfig == E_SPI_SIMPLEX)
	{
		/** Clear BIDIMODE */
		tempReg &= ~(1U << 15U);
		/** Set RXONLY */
		tempReg |= (1U << 10U);
	}
	else
	{
		// TODO: Handle invalid bus configuration
	}

	/** Configure the baudrate */
	tempReg |= (spiConfigInfo->spiConfiguration.e_clockSpeed << 3U);

	/** Configure the CHPA & CPOL derived from SPI mode */
	tempReg &= ~(1 << 0U);
	tempReg &= ~(1 << 1U);

	switch (spiConfigInfo->spiConfiguration.e_commMode)
	{
		case E_SPI_MODE_0:
			/** CPOL = 0, CPHA = 0 */
			break;
		case E_SPI_MODE_1:
			/** CPOL = 0, CPHA = 1 */
			tempReg |= (1U << 0U);
			break;
		case E_SPI_MODE_2:
			/** CPOL = 1, CPHA = 0 */
			tempReg |= (1U << 1U);
			break;
		case E_SPI_MODE_3:
			/** CPOL = 1, CPHA = 1 */
			tempReg |= (1U << 0U);
			tempReg |= (1U << 1U);
			break;
		default:
			// TODO: Handle error cases
			break;
	}

	spiConfigInfo->SPIx->CR1 = tempReg;
}

void SPIEnablePeriphClk(SPIx_t * SPIx, HAL_LogicLevel enDi)
{
	if (enDi == E_EN)
	{
		if (SPIx == SPI1)
		{
			SPI1_PERIPH_CLK_EN();
		}
		else if (SPIx == SPI2)
		{
			SPI2_PERIPH_CLK_EN();
		}
		else if (SPIx == SPI3)
		{
			SPI3_PERIPH_CLK_EN();
		}
		else
		{
			// TODO: Add handling for invalid SPI base address
		}
	}
	else
	{
		if (SPIx == SPI1)
		{
			SPI1_PERIPH_CLK_DI();
		}
		else if (SPIx == SPI2)
		{
			SPI2_PERIPH_CLK_DI();
		}
		else if (SPIx == SPI3)
		{
			SPI3_PERIPH_CLK_DI();
		}
		else
		{
			// TODO: Add handling for invalid SPI base address
		}
	}
}

void SPISendData(SPIx_t SPIx, uint8_t * dataBuf, uint8_t dataLen)
{
	while (dataLen > 0)
	{
		/** Wait here until the TX buffer is not empty */
		while (!(SPIx->SR & SPI_SR_TXE_MASK));

		/** Check the data frame format settings, i.e., 8 bit or 16 bit data */
		if (SPIx->CR1 & SPI_CR1_DFF_MASK)
		{
			/** 16 bit data transfer */
			SPIx->DR = *((uint16_t)dataBuf);
			dataLen--;
			dataLen--;
			(uint16_t *)dataBuf++;
		}
		else
		{
			/** 8 bit data transfer */
			SPIx->DR = *dataBuf;
			dataBuf++;
			dataLen--;
		}
	}
}

void SPISSIConfig(SPIx_t SPIx, HAL_LogicLevel enDi)
{
	if (enDi == E_EN)
	{
		SPIx->CR1 |= (1 << SSI);
	}
	else
	{
		SPIx->CR1 &= ~(1 << SSI);
	}
}

void SPIEnablePeripheral(SPIx_t SPIx, HAL_LogicLevel enDi)
{
	if (enDi == E_EN)
	{
		SPIx->CR1 |= (1 << SPE);
	}
	else
	{
		SPIx->CR1 &= ~(1 << SPE);
	}
}

