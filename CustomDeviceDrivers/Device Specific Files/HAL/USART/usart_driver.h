#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include "stm32f411re.h"

/**
 * USART settings (Transmit)
 * 1. Program the M bit in the USART_CR1 register to define the word length
 * 2. Program the number of stop bits in USART_CR2 register
 * 3. Select the baudrate USART_BRR
 * 4. Set the TE bit in USART_CR1 to enable the transmit block
 * 5. Enable the USART by writing the UE bit in USART_CR1
 * 6. Now if TXE flag is set, then write data byte to send in USART_DR
 * 7. Wait until TC is set
 *
 * (Reception)
 * 1. Program the M bit in the USART_CR1 register to define the word length
 * 2. Program the number of stop bits in USART_CR2 register
 * 3. Select the baudrate USART_BRR
 * 4. Enable the USART by writing the UE bit in USART_CR1
 * 5. Set the RE bit in USART_CR1 which enables receiver block of the peripheral
 * 6. Wait until RXNE & then read USART_DR
 */
typedef enum
{
	E_USART_1_STOP_BIT,
	E_USART_0_5_STOP_BIT,
	E_USART_2_STOP_BIT,
	E_USART_1_5_STOP_BIT
} USART_StopBitSettings_e;

typedef enum
{
	E_USART_EVEN_PARITY,
	E_USART_ODD_PARITY,
	E_USART_DISABLE_PARITY
} USART_ParitySettings_e;

typedef enum
{
	E_USART_8_DATA_BITS,
	E_USART_9_DATA_BITS
} USART_WordLen_e;

typedef enum
{
	E_USART_TX_ONLY,
	E_USART_RX_ONLY,
	E_USART_RXTX
} USART_Mode_e;

typedef struct
{
	USART_Mode_e e_mode;
	uint32_t baudRate;
	USART_StopBitSettings_e e_stopBit;
	USART_ParitySettings_e e_parityType;
	USART_WordLen_e e_wordLen;
} USART_Configuration;

typedef struct
{
	USARTx_t * USARTx;
	USART_Configuration usartConfig;
} USART_Info_t;

/**
 * \brief A function that enables the USART peripheral
 * \param[in] Base address of the USART peripheral to be enabled
 */
void USARTPeripheralEnable(USARTx_t * USARTx);

/**
 * \brief A function that disables the USART peripheral
 * \param[in] Base address of the USART peripheral to be disabled
 */
void USARTPeripheralDisable(USARTx_t * USARTx);

/**
 * \brief A function to enable or disable transmission
 * \param[in] Base address of the USART peripheral
 * \param[in] enDi Enable or disable the transmission
 */
void USARTTransmitEnable(USARTx_t * USARTx, HAL_LogicLevel enDi);

/**
 * \brief A function to enable or disable reception
 * \param[in] Base address of the USART peripheral
 * \param[in] enDi Enable or disable the reception
 */
void USARTReceiveEnable(USARTx_t * USARTx, HAL_LogicLevel enDi);

/**
 * \brief A function to enable the peripheral clock for USART
 */
void USARTPeripheralClkEnable(USARTx_t * USARTx);

/**
 * \brief A function to disable the peripheral clock for USART
 */
void USARTPeripheralClkDisable(USARTx_t * USARTx);

/**
 * \brief A function to send data via USART
 * \param[in] txBuf A pointer to the buffer to be sent
 * \param[in] txLen Data bytes to be sent
 */
void USARTSendData(USART_Info_t * usartInfo, uint8_t *txBuf, uint32_t txLen);

/**
 * \brief A function to receive data via USART
 * \param[in] txBuf A pointer to the buffer in which data will be copied
 * \param[in] txLen Data bytes to be received
 */
void USARTReceiveData(USART_Info_t * usartInfo, uint8_t *rxBuf, uint32_t rxLen);

/**
 * \brief Initialise the USART peripheral
 * \param[in] Base address of the USART peripheral
 */
void USARTInitialize(USART_Info_t * usartInfo);

/**
 * \brief A function that sets the baudrate
 * \param[in] USARTx Base address of USART peripheral
 * \param[in] baudRate The baudrate to be set
 */
void USARTSetBaudRate(USART_Info_t *usartInfo, uint32_t baudRate);

#endif
