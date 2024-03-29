#include "i2c_driver.h"

void I2CInitialize(I2C_Info_t * i2cInfo)
{
	uint8_t tempReg = 0u;
	uint16_t CCRVal = 0u;

	// TODO: Enable the clock for the I2C peripheral

	// FIXME: This must be done only when PE=1
	/** Ack control enable / disable settings */
	tempReg = (i2cInfo->i2cConfiguration.e_ackControl << 10);
	i2cInfo->I2Cx->CR1 |= tempReg;

	tempReg = 0;

	/** Peripheral clock configuration settings */
	tempReg = RCC_GetPLK1Clock() / 1000000;
	i2cInfo->I2Cx->CR2 |= tempReg;

	/** I2C baudrate configuration settings */
	// TODO: Incorporate duty settings to be able to match speeds between 400 & 100 khz
	tempReg = 0;
	if (i2cInfo->i2cConfiguration.e_i2cBaudrate == E_I2C_BAUDRATE_100KHZ)
	{
		/** 100 KHz */
		tempReg &= ~(1 << 15);
		tempReg &= ~(1 << 14);
		CCRVal = RCC_GetPLK1Clock() / (2 * 400000);
	}
	else
	{
		/** 400 KHz */
		tempReg |= (1 << 15);
		tempReg |= (1 << 14);

		CCRVal = RCC_GetPLK1Clock() / (25 * 400000);
	}
	tempReg |= CCRVal;
	i2cInfo->I2Cx->CCR |= tempReg;

	/** Slave address configuration */
	i2cInfo->I2Cx->OAR1 |= (i2cInfo->i2cConfiguration.slaveAddress << 1);

	/** 7-bit slave addressing */
	i2cInfo->I2Cx->OAR1 &= ~(1 << 15U);

	/** 14 bit must always be kept 1 by software */
	i2cInfo->I2Cx->OAR1 |= (1 << 14U);
}

void I2CPeripheralEnable(I2Cx_t * I2Cx)
{
	I2Cx->CR1 |= (1 << 0);
}

void I2CPeripheralDisable(I2Cx_t * I2Cx)
{
	I2Cx->CR1 &= ~(1 << 0);
}

// TODO: Incorporate repeated start condition instead of STOP START
void I2CMasterSendData(I2Cx_t * I2Cx, uint8_t i2cSlaveAddress, uint8_t *dataBuf, uint8_t dataLen)
{
	/** Set the start bit to 1 */
	I2Cx->CR1 |= (1 << I2C_CR1_START_BIT_POS);

	/** Check for SB=1 */
	while (!(I2Cx->SR1 & I2C_SR1_START_BIT_MASK));

	/** Clear by reading SR1 followed by writing DR register with address << 1 with read/write bit */
	uint8_t slaveAddr = i2cSlaveAddress << 1U;
	slaveAddr &= ~(1);
	I2Cx->DR = slaveAddr;

	/** Check if ADDR=1 */
	while (!(I2Cx->SR1 & I2C_SR1_ADDR_BIT_MASK));

	/** Clear by reading SR1 followed by reading SR2 */
	uint32_t statusReg2 = (I2Cx->SR2);
	(void)statusReg2;

	/** Check if TXE=1 & write DR with data */
	while (!(I2Cx->SR1 & I2C_SR1_TXE_BIT_MASK));

	/** Check if TXE=1 & write DR with next data */
	while (dataLen > 0)
	{
		while (!(I2Cx->SR1 & I2C_SR1_TXE_BIT_MASK));
		I2Cx->DR = *dataBuf;
		dataBuf++;
		dataLen--;
	}

	/** Check if TXE=1 & BTF=1 when dataLen == 0 */
	while ((!(I2Cx->SR1 & I2C_SR1_TXE_BIT_MASK)) && (!(I2Cx->SR1 & I2C_SR1_BTF_BIT_MASK)));

	/** Set the stop bit to 1 */
	I2Cx->CR1 |= (1U << I2C_CR1_STOP_BIT_POS);

	// TODO; Configure the TRISE time
}

// TODO: Incorporate repeated start condition instead of STOP START
void I2CMasterReceiveData(I2Cx_t * I2Cx, uint8_t i2cSlaveAddress, uint8_t *rxBuf, uint8_t rxLen)
{
	/** Set the start bit to 1 */
	I2Cx->CR1 |= (1 << I2C_CR1_START_BIT_POS);

	/** Check for SB=1 */
	while (!(I2Cx->SR1 & I2C_SR1_START_BIT_MASK));

	/** Clear by reading SR1 followed by writing DR register with address << 1 with read/write bit */
	uint8_t slaveAddr = i2cSlaveAddress << 1U;
	slaveAddr |= (1);
	I2Cx->DR = slaveAddr;

	/** Check if ADDR=1 */
	while (!(I2Cx->SR1 & I2C_SR1_ADDR_BIT_MASK));

	if (rxLen == 1)
	{
		/** Make the ACK bit 0 */
		I2Cx->CR1 &= ~(1 << 10);

		/** Clear ADDR flag */
		/** Clear by reading SR1 followed by reading SR2 */
		uint32_t statusReg2 = (I2Cx->SR2);
		(void)statusReg2;

		while (!(I2Cx->SR1 & I2C_SR1_RXNE_BIT_MASK));

		/** Generate the STOP condition */
		I2Cx->CR1 |= (1U << I2C_CR1_STOP_BIT_POS);

		*rxBuf = I2Cx->DR;
		rxBuf++;
		rxLen--;
	}
	else
	{
		/** Clear ADDR flag */
		/** Clear by reading SR1 followed by reading SR2 */
		uint32_t statusReg2 = (I2Cx->SR2);
		(void)statusReg2;

		/** Start receiving the data */
		while (rxLen > 0)
		{
			while (!(I2Cx->SR1 & I2C_SR1_RXNE_BIT_MASK));

			if (rxLen == 2)
			{
				/** Make the ACK bit 0 */
				I2Cx->CR1 &= ~(1 << 10);

				/** Generate the STOP condition */
				I2Cx->CR1 |= (1U << I2C_CR1_STOP_BIT_POS);
			}

			*rxBuf = I2Cx->DR;
			rxBuf++;
			rxLen--;
		}
	}
	/** Re-enable Ack control, if ACK is enabled */
	// TODO: Incorporate ACK control user configurable parameter to re-enable or disable this
	I2Cx->CR1 |= (1 << 10);
}
