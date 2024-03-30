#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "stm32f411re.h"

#define I2C1_PERIPH_CLK_EN()				(RCC->APB1ENR |= (1U << 21U))
#define I2C1_PERIPH_CLK_DI()				(RCC->APB1ENR &= ~(1U << 21U))


typedef enum
{
	E_I2C_NO_ACK_RETURN,
	E_I2C_ACK_RETURN
} I2C_AckControl_e;

typedef enum
{
	E_I2C_BAUDRATE_100KHZ,
	E_I2C_BAUDRATE_400KHZ
} I2C_BaudRate_e;

typedef enum
{
	E_I2C_DUTY_CYCLE_0,
	E_I2C_DUTY_CYCLE_1
} I2C_DutyCycle_e;

typedef struct
{
	I2C_AckControl_e e_ackControl;
	I2C_BaudRate_e e_i2cBaudrate;
	I2C_DutyCycle_e e_i2cDutyCycle;
	uint8_t slaveAddress;
} I2C_Configuration;

typedef struct
{
	I2Cx_t *I2Cx;
	I2C_Configuration i2cConfiguration;
} I2C_Info_t;

/**
 * APIs required
 * 1. Configure I2C (i2c init function)
 * 2. Enable / disable peripheral
 * 3. Enable / disable peripheral clock
 * 4. Send data blocking
 * 5. Receive data blocking
 * 6. Send data IT
 * 7. Receive data IT
 * 8. IT interrupt handling (event & errors)
 */

/**
 * \brief A function that initialises the I2C peripheral
 */
void I2CInitialize(I2C_Info_t * i2cInfo);

/**
 * \brief A function that enables the I2C peripheral
 */
void I2CPeripheralEnable(I2Cx_t * I2Cx);

/**
 * \brief A function that disables the I2C peripheral
 */
void I2CPeripheralDisable(I2Cx_t * I2Cx);

/**
 * \brief A function to send data over I2C
 * param[in] I2Cx - I2C peripheral being used
 * param[in] i2cSlaveAddress - Slave address of the I2C slave device intended to communicate with
 * param[in] dataBuf - Pointer to the data to be sent
 * param[in] dataLen - Number of bytes to be sent
 */
void I2CMasterSendData(I2Cx_t * I2Cx, uint8_t i2cSlaveAddress, uint8_t *dataBuf, uint8_t dataLen);

/**
 * \brief A function to receive data over I2C
 */
void I2CMasterReceiveData(I2Cx_t * I2Cx, uint8_t slaveAddress, uint8_t *rxBuf, uint8_t rxLen);

#endif
