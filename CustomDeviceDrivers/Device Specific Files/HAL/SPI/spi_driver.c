#include "spi_driver.h"

static void SPITXEHandle(SPI_Comm_Handle_t *spiCommHandle);
static void SPIRXNEHandle(SPI_Comm_Handle_t *spiCommHandle);

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

void SPISendDataIT(SPI_Comm_Handle_t *spiCommHandle, uint8_t *pTxBuf, uint8_t len)
{
	if (spiCommHandle->dataPacket.txState_e != E_TX_STATE_BUSY)
	{
		spiCommHandle->dataPacket.pTxBuf = pTxBuf;
		spiCommHandle->dataPacket.txLen = len;
		spiCommHandle->dataPacket.txState_e = E_TX_STATE_BUSY;

		/** Enable interrupt for TXE */
		spiCommHandle->SPIx->CR2 |= (1 << 7U);
	}
}

void SPIReceiveDataIT(SPI_Comm_Handle_t *spiCommHandle, uint8_t *pRxBuf, uint8_t len)
{
	if (spiCommHandle->dataPacket.rxState_e != E_RX_STATE_BUSY)
	{
		spiCommHandle->dataPacket.pRxBuf = pRxBuf;
		spiCommHandle->dataPacket.rxLen = len;
		spiCommHandle->dataPacket.rxState_e = E_RX_STATE_BUSY;

		/** Enable interrupt for RXNE */
		spiCommHandle->SPIx->CR2 |= (1 << 6U);
	}
}

void SPIIRQHandler(SPI_Comm_Handle_t *spiCommHandle)
{
	/** Check why the interrupt has occured */
	uint8_t temp1, temp2;
	temp1 = (spiCommHandle->SPIx->SR & SPI_SR_TXE_MASK);
	temp2 = (spiCommHandle->SPIx->SR & SPI_SR_RXNE_MASK);

	if (temp1)
	{
		/** Interrupt caused as TX buffer is empty */
		SPITXEHandle();
	}
	if (temp2)
	{
		/** Interrupt caused as RX buffer is not empty */
		SPIRXNEHandle();
	}
}

static void SPITXEHandle(SPI_Comm_Handle_t *spiCommHandle)
{
	/** Check the data frame format settings, i.e., 8 bit or 16 bit data */
	if (spiCommHandle->SPIx->CR1 & SPI_CR1_DFF_MASK)
	{
		/** 16 bit data transfer */
		spiCommHandle->SPIx->DR = *((uint16_t*)spiCommHandle->dataPacket.pTxBuf);
		spiCommHandle->dataPacket.txLen--;
		spiCommHandle->dataPacket.txLen--;
		(uint16_t *)spiCommHandle->dataPacket.pTxBuf++;
	}
	else
	{
		/** 8 bit data transfer */
		spiCommHandle->SPIx->DR = *spiCommHandle->dataPacket.pTxBuf;
		spiCommHandle->dataPacket.txLen--;
		spiCommHandle->dataPacket.pTxBuf++;
	}
	if (!spiCommHandle->dataPacket.txLen)
	{
		/** Disable interrutps */
		spiCommHandle->SPIx->CR2 &= ~(1 << 7U);
	}
}

static void SPIRXNEHandle(SPI_Comm_Handle_t *spiCommHandle)
{
	if ((spiCommHandle->SPIx->CR1 & SPI_CR1_DFF_MASK))
	{
		/** 16 bit data transfer */
		*((uint16_t *)spiCommHandle->dataPacket.pRxBuf) = (uint16_t)spiCommHandle->SPIx->DR;
		(uint16_t *)spiCommHandle->dataPacket.pRxBuf++;
		/** Read two bytes of data */
		spiCommHandle->dataPacket.rxLen--;
		spiCommHandle->dataPacket.rxLen--;
	}
	else
	{
		/** 8 bit data transfer */
		*spiCommHandle->dataPacket.pRxBuf = spiCommHandle->SPIx->DR;
		spiCommHandle->dataPacket.rxLen--;
		spiCommHandle->dataPacket.pRxBuf++;
	}
	if (!spiCommHandle->dataPacket.rxLen)
	{
		/** Disable interrupts */
		spiCommHandle->SPIx->CR2 &= ~(1 << 6U);
	}
}

void SPISendData(SPIx_t *SPIx, uint8_t * dataBuf, uint8_t dataLen)
{
	while (dataLen > 0)
	{
		/** Wait here until the TX buffer is not empty */
		while (!(SPIx->SR & SPI_SR_TXE_MASK));

		/** Check the data frame format settings, i.e., 8 bit or 16 bit data */
		if (SPIx->CR1 & SPI_CR1_DFF_MASK)
		{
			/** 16 bit data transfer */
			SPIx->DR = *((uint16_t*)dataBuf);
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

void SPIReceiveData(SPIx_t *SPIx, uint8_t *destBuf, uint8_t dataLen)
{
	if (dataLen != 0)
	{
		while (dataLen != 0)
		{
			/** Wait for RXNE to be 1, i.e., there is some data to receive */
			while (!(SPIx->SR & SPI_SR_RXNE_MASK ))
			{
				if ((SPIx->CR1 & SPI_CR1_DFF_MASK))
				{
					/** 16 bit data transfer */
					*((uint16_t *)destBuf) = (uint16_t)SPIx->DR;
					(uint16_t *)destBuf++;
					/** Read two bytes of data */
					dataLen--;
					dataLen--;
				}
				else
				{
					/** 8 bit data transfer */
					*destBuf = SPIx->DR;
					dataLen--;
					destBuf++;
				}
			}
		}
	}
	else
	{
		/** Nothing to send! */
	}
}

void SPISetIRQPriority(uint8_t IRQn, uint8_t priorityNumber)
{
	uint8_t prxRegister = IRQn / 4;
	uint8_t prxSection = IRQn % 4;

	uint8_t shiftAmount = (prxSection * 8U) + (8 - 4); // TODO: Make a macro for 4, as it is the number of priority bits implemented in this MCU

	*(NVIC_IPR0 + (prxRegister * 4)) |= (1 << shiftAmount);
}

void SPIConfigureIRQn(uint8_t IRQn, HAL_LogicLevel enDi)
{
	if (enDi == E_EN)
	{
		if (IRQn <= 31)
		{
			*NVIC_ISER1 |= (1 << IRQn);
		}
		else if (IRQn > 31 && IRQn < 64)
		{
			*NVIC_ISER1 |= (1 << IRQn % 32U);
		}
		else if (IRQn >= 64 && IRQn < 96)
		{
			*NVIC_ISER2 |= (1 << IRQn % 32U);
		}
	}
	else
	{
		if (IRQn <= 31)
		{
			*NVIC_ISER0 &= ~(1 << IRQn % 32U);
		}
		else if (IRQn > 31 && IRQn < 64)
		{
			*NVIC_ISER1 &= ~(1 << IRQn % 32U);
		}
		else if (IRQn >= 64 && IRQn < 96)
		{
			*NVIC_ISER2 &= ~(1 << IRQn % 32U);
		}
	}
}

void SPISSIConfig(SPIx_t *SPIx, HAL_LogicLevel enDi)
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

void SPIEnablePeripheral(SPIx_t *SPIx, HAL_LogicLevel enDi)
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

