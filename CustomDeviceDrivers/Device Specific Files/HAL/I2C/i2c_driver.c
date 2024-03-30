#include "i2c_driver.h"

static void I2CManageAckSettings(I2Cx_t *I2Cx, HAL_LogicLevel enDI);
static void I2CGenerateStartCondition(I2Cx_t *I2Cx);
static void I2CGenerateStopCondition(I2Cx_t *I2Cx);
static bool I2CGetFlagStatusReg1(I2Cx_t *I2Cx, uint16_t mask);
static bool I2CGetFlagStatusReg2(I2Cx_t *I2Cx, uint16_t mask);

void I2CInitialize(I2C_Info_t * i2cInfo)
{
	uint32_t tempReg = 0u;
	uint16_t CCRVal = 0u;

	I2C1_PERIPH_CLK_EN();

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
		CCRVal = RCC_GetPLK1Clock() / (2 * 100000);
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

	I2CPeripheralEnable(i2cInfo->I2Cx);

	I2CManageAckSettings(i2cInfo->I2Cx, E_EN);
}

void I2CManageAckSettings(I2Cx_t * I2Cx, HAL_LogicLevel enDi)
{
	if (enDi == E_EN)
	{
		/** Enable ACKs */
		I2Cx->CR1 |= (1 << 10);
	}
	else
	{
		/** Disable ACKs */
		I2Cx->CR1 &= ~(1 << 10);
	}
}

void I2CPeripheralEnable(I2Cx_t * I2Cx)
{
	I2Cx->CR1 |= (1 << 0);
}

void I2CPeripheralDisable(I2Cx_t * I2Cx)
{
	I2Cx->CR1 &= ~(1 << 0);
}

static void I2CGenerateStartCondition(I2Cx_t * I2Cx)
{
	/** Set the start bit to 1 */
	I2Cx->CR1 |= (1 << I2C_CR1_START_BIT_POS);
}

static void I2CGenerateStopCondition(I2Cx_t *I2Cx)
{
	/** Set the stop bit to 1 */
	I2Cx->CR1 |= (1U << I2C_CR1_STOP_BIT_POS);
}

static bool I2CGetFlagStatusReg1(I2Cx_t *I2Cx, uint16_t mask)
{
	return (I2Cx->SR1 & mask);
}

static bool I2CGetFlagStatusReg2(I2Cx_t *I2Cx, uint16_t mask)
{
	return (I2Cx->SR1 & mask);
}

static void I2CSendSlaveAddress(I2Cx_t *I2Cx, uint8_t slaveAddress, I2C_OperationType_e operationType)
{
	uint8_t slaveAddr = slaveAddress << 1U;
	if (operationType == E_I2C_READ_OP)
	{
		slaveAddr |= (1);
		I2Cx->DR = slaveAddr;
	}
	else if (operationType == E_I2C_WRITE_OP)
	{
		slaveAddr &= ~(1);
		I2Cx->DR = slaveAddr;
	}
	else
	{
		/** Invalid operation type */
	}
}

// TODO: Incorporate repeated start condition instead of STOP START
void I2CMasterSendData(I2Cx_t * I2Cx, uint8_t i2cSlaveAddress, uint8_t *dataBuf, uint8_t dataLen)
{
	I2CGenerateStartCondition(I2Cx);

	/** Check for SB=1 */
	while (!I2CGetFlagStatusReg1(I2Cx, I2C_SR1_START_BIT_MASK));

	/** Clear by reading SR1 followed by writing DR register with address << 1 with read/write bit */
	I2CSendSlaveAddress(I2Cx, i2cSlaveAddress, E_I2C_WRITE_OP);

	/** Check if ADDR=1 */
	while (!I2CGetFlagStatusReg1(I2Cx, I2C_SR1_ADDR_BIT_MASK));

	/** Clear by reading SR1 followed by reading SR2 */
	uint32_t statusReg2 = (I2Cx->SR2);
	(void)statusReg2;

	/** Check if TXE=1 & write DR with data */
	while (!(I2CGetFlagStatusReg1(I2Cx, I2C_SR1_TXE_BIT_MASK)));

	/** Check if TXE=1 & write DR with next data */
	while (dataLen > 0)
	{
		while (!(I2CGetFlagStatusReg1(I2Cx, I2C_SR1_TXE_BIT_MASK)));
		I2Cx->DR = *dataBuf;
		dataBuf++;
		dataLen--;
	}

	/** Check if TXE=1 & BTF=1 when dataLen == 0 */
	while ((!(I2CGetFlagStatusReg1(I2Cx, I2C_SR1_TXE_BIT_MASK))) && (!(I2CGetFlagStatusReg1(I2Cx, I2C_SR1_BTF_BIT_MASK))));

	I2CGenerateStopCondition(I2Cx);

	// TODO; Configure the TRISE time
}

// TODO: Incorporate repeated start condition instead of STOP START
void I2CMasterReceiveData(I2Cx_t * I2Cx, uint8_t i2cSlaveAddress, uint8_t *rxBuf, uint8_t rxLen)
{
	I2CGenerateStartCondition(I2Cx);

	/** Check for SB=1 */
	while (!I2CGetFlagStatusReg1(I2Cx, I2C_SR1_START_BIT_MASK));

	/** Clear by reading SR1 followed by writing DR register with address << 1 with read/write bit */
	I2CSendSlaveAddress(I2Cx, i2cSlaveAddress, E_I2C_READ_OP);

	/** Check if ADDR=1 */
	while (!I2CGetFlagStatusReg1(I2Cx, I2C_SR1_ADDR_BIT_MASK));

	if (rxLen == 1)
	{
		/** Make the ACK bit 0 */
		I2CManageAckSettings(I2Cx, E_DI);

		/** Clear ADDR flag by reading SR1 followed by reading SR2 */
		uint32_t statusReg2 = (I2Cx->SR2);
		(void)statusReg2;

		while (!I2CGetFlagStatusReg1(I2Cx, I2C_SR1_RXNE_BIT_MASK));

		/** Generate the STOP condition */
		I2CGenerateStopCondition(I2Cx);

		*rxBuf = I2Cx->DR;
		rxBuf++;
		rxLen--;
	}
	else
	{
		/** Clear ADDR flag by reading SR1 followed by reading SR2 */
		uint32_t statusReg2 = (I2Cx->SR2);
		(void)statusReg2;

		/** Start receiving the data */
		while (rxLen > 0)
		{
			while (!I2CGetFlagStatusReg1(I2Cx, I2C_SR1_RXNE_BIT_MASK));

			if (rxLen == 2)
			{
				/** Make the ACK bit 0 */
				I2CManageAckSettings(I2Cx, E_DI);

				/** Generate the STOP condition */
				I2CGenerateStopCondition(I2Cx);
			}

			*rxBuf = I2Cx->DR;
			rxBuf++;
			rxLen--;
		}
	}
	/** Re-enable Ack control, if ACK is enabled */
	I2CManageAckSettings(I2Cx, E_EN);
	// TODO: Incorporate ACK control user configurable parameter to re-enable or disable this
}
