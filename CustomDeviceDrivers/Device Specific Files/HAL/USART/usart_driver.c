#include "usart_driver.h"

void USARTPeripheralEnable(USARTx_t * USARTx)
{
	USARTx->CR1 |= (1 << USART_CR1_UE_BIT_POS);
}

void USARTPeripheralDisable(USARTx_t * USARTx)
{
	USARTx->CR1 &= ~(1 << USART_CR1_UE_BIT_POS);
}

void USARTTransmitEnable(USARTx_t * USARTx, HAL_LogicLevel enDi)
{
	USARTx->CR1 |= (1 << USART_CR1_TE_BIT_POS);
}

void USARTReceiveEnable(USARTx_t * USARTx, HAL_LogicLevel enDi)
{
	USARTx->CR1 |= (1 << USART_CR1_RE_BIT_POS);
}

bool USARTGetFlagStatus(USARTx_t * USARTx, uint16_t mask)
{
	return (USARTx->SR & mask);
}

void USARTInitialize(USART_Info_t * usartInfo)
{
	uint32_t tempReg = 0U;

	/** Enable the peripheral clock for the USART peripheral */
	USARTPeripheralClkEnable(usartInfo->USARTx);

	/*********************** CONFIGURE CR1 ***********************/
	/** Enable the RX / TX engines based on the mode */
	if (usartInfo->usartConfig.e_mode == E_USART_TX_ONLY)
	{
		tempReg |= (1U << USART_CR1_TE_BIT_POS);
	}
	else if (usartInfo->usartConfig.e_mode == E_USART_RX_ONLY)
	{
		tempReg |= (1U << USART_CR1_RE_BIT_POS);
	}
	else
	{
		tempReg |= ((1U << USART_CR1_RE_BIT_POS) | (1U << USART_CR1_TE_BIT_POS));
	}

	/** Word length configuration */
	tempReg |= (usartInfo->usartConfig.e_wordLen << USART_CR1_M_BIT_POS);

	/** Parity bits configuration */
	if (usartInfo->usartConfig.e_parityType != E_USART_DISABLE_PARITY)
	{
		tempReg |= (1 << USART_CR1_PCE_BIT_POS);
		tempReg |= (usartInfo->usartConfig.e_parityType << USART_CR1_PS_BIT_POS);
	}
	else
	{
		tempReg &= ~(1 << USART_CR1_PCE_BIT_POS);
	}
	usartInfo->USARTx->CR1 = tempReg;

	/*********************** CONFIGURE CR2 ***********************/
	/** Stop bit configuration */
	tempReg = 0u;

	tempReg |= (usartInfo->usartConfig.e_stopBit << USART_CR2_STOP_BIT_POS);
	usartInfo->USARTx->CR2 = tempReg;

	/*********************** CONFIGURE CR3 ***********************/
	tempReg = 0u;
	// TODO: Add configurability for enabling HW flow control


	/** Setting the baudrate */
	USARTSetBaudRate(usartInfo, usartInfo->usartConfig.baudRate);

	USARTPeripheralEnable(usartInfo->USARTx);
}

void USARTSetBaudRate(USART_Info_t *usartInfo, uint32_t baudRate)
{
	uint32_t pClk = RCC_GetPLK1Clock();
	uint32_t usartDiv = 0u;
	uint32_t tempReg = 0u;

	if (usartInfo->USARTx->CR1 & USART_CR1_OVR8_BIT_MASK)
	{
		/** Oversampling by 8 */
		usartDiv = ((pClk * 25) / (2 * baudRate));
	}
	else
	{
		/** Oversampling by 16 */
		usartDiv = ((pClk * 25) / (4 * baudRate));
	}

	uint16_t mantissa = usartDiv / 100;
	tempReg |= (mantissa << 4);

	uint32_t fraction = usartDiv - (mantissa * 100);
	if (usartInfo->USARTx->CR1 & USART_CR1_OVR8_BIT_MASK)
	{
		/** Oversampling by 8 */
		fraction = ((fraction * 8) & ((uint8_t)0x07));
	}
	else
	{
		/** Oversampling by 16 */
		fraction = ((fraction * 16) & ((uint8_t)0x0F));
	}
	fraction = (fraction + 50) / 100;

	tempReg |= (fraction << 0U);

	usartInfo->USARTx->BRR = tempReg;
}

void USARTPeripheralClkEnable(USARTx_t * USARTx)
{
	if (USARTx == USART1)
	{

	}
	else if (USARTx == USART2)
	{
		RCC->APB1ENR |= (1 << 17U);
	}
	else
	{
		/** TODO: Implement the other peripherals */
	}
}

void USARTPeripheralClkDisable(USARTx_t * USARTx)
{

}

void USARTSendData(USART_Info_t * usartInfo, uint8_t *txBuf, uint32_t txLen)
{
	uint16_t * pDataPtr = 0;

	while (txLen > 0)
	{
		/** Wait till the TXE flag is set */
		while (!USARTGetFlagStatus(usartInfo->USARTx, USART_SR_TXE_BIT_MASK));

		/** Check if 8-bit or 9-bit data transfer is selected */
		if (usartInfo->usartConfig.e_wordLen == E_USART_9_DATA_BITS)
		{
			/** If 9-bit data transfer is enabled, mask 9 bits from the buffer */
			pDataPtr = (uint16_t *)txBuf;
			usartInfo->USARTx->DR = (*pDataPtr & (uint16_t)0x01FF);
			if (usartInfo->usartConfig.e_parityType == E_USART_DISABLE_PARITY)
			{
				txBuf++;
				txBuf++;
				txLen--;
				txLen--;
			}
			else
			{
				txBuf++;
				txLen--;
			}
		}
		else
		{
			usartInfo->USARTx->DR = (*txBuf & (uint8_t)0xFF);
			txBuf++;
			txLen--;
		}
	}
	while (!USARTGetFlagStatus(usartInfo->USARTx, USART_SR_TC_BIT_MASK));
}

void USARTReceiveData(USART_Info_t * usartInfo, uint8_t *rxBuf, uint32_t rxLen)
{

}
