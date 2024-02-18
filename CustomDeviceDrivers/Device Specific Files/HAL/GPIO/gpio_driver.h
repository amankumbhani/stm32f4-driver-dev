#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32f411re.h"

#define GPIO_BASEADDR_TO_PORT_CODE(x) 	(x == GPIOA) ? 0 : \
										(x == GPIOB) ? 1 : \
										(x == GPIOC) ? 2 : \
										(x == GPIOD) ? 3 : \
										(x == GPIOE) ? 4 : \
										(x == GPIOH) ? 7 : -1

#define GPIO_PIN_0						((uint16_t) 0x0001U)
#define GPIO_PIN_1						((uint16_t) 0x0002U)
#define GPIO_PIN_2						((uint16_t) 0x0004U)
#define GPIO_PIN_3						((uint16_t) 0x0008U)
#define GPIO_PIN_4						((uint16_t) 0x0010U)
#define GPIO_PIN_5						((uint16_t) 0x0020U)
#define GPIO_PIN_6						((uint16_t) 0x0040U)
#define GPIO_PIN_7						((uint16_t) 0x0080U)
#define GPIO_PIN_8						((uint16_t) 0x0100U)
#define GPIO_PIN_9						((uint16_t) 0x0200U)
#define GPIO_PIN_10						((uint16_t) 0x0400U)
#define GPIO_PIN_11						((uint16_t) 0x0800U)
#define GPIO_PIN_12						((uint16_t) 0x1000U)
#define GPIO_PIN_13						((uint16_t) 0x2000U)
#define GPIO_PIN_14						((uint16_t) 0x4000U)
#define GPIO_PIN_15						((uint16_t) 0x8000U)
#define GPIO_ALL_PINS					((uint16_t) 0xFFFFU)

enum
{
	E_GPIO_PIN_0 = 0,
	E_GPIO_PIN_1,
	E_GPIO_PIN_2,
	E_GPIO_PIN_3,
	E_GPIO_PIN_4,
	E_GPIO_PIN_5,
	E_GPIO_PIN_6,
	E_GPIO_PIN_7,
	E_GPIO_PIN_8,
	E_GPIO_PIN_9,
	E_GPIO_PIN_10,
	E_GPIO_PIN_11,
	E_GPIO_PIN_12,
	E_GPIO_PIN_13,
	E_GPIO_PIN_14,
	E_GPIO_PIN_15,
};

typedef enum
{
	E_GPIO_INPUT = 0,
	E_GPIO_OUTPUT,
	E_GPIO_ALTERNATE,
	E_GPIO_ANALOG,
	E_GPIO_IT_FALLING,
	E_GPIO_IT_RISING,
} GPIO_Dir_e;

typedef enum
{
	E_GPIO_PUSHPULL = 0,
	E_GPIO_OPENDRAIN
} GPIO_Type_e;

typedef enum
{
	E_GPIO_LOW_SPEED = 0,
	E_GPIO_MEDIUM_SPEED,
	E_GPIO_FAST_SPEED,
	E_GPIO_HIGH_SPEED
} GPIO_Speed_e;

typedef enum
{
	E_GPIO_NO_PULLUP = 0,
	E_GPIO_PULLUP,
	E_GPIO_PULLDOWN,
	E_GPIO_RESERVED
} GPIO_PullUpDownConf_e;

typedef enum
{
	E_GPIO_LOW,
	E_GPIO_HIGH
} GPIO_OutputState_e;

typedef struct
{
	/** Pin number to be configured */
	uint16_t pinNumber;

	/** GPIO type */
	GPIO_Type_e type;

	/** Direction of the GPIO */
	GPIO_Dir_e mode;

	/** Speed of the GPIO */
	GPIO_Speed_e speed;

	/** Push pull configuration of the GPIO */
	GPIO_PullUpDownConf_e pullUpDownConf;

	/** Alternate functionality of the GPIO */
	// TODO: Add this functionality
} GPIO_PinConfiguration;

typedef struct
{
	/** Pointer to the base address of the GPIO port to be configured */
	GPIOx_t * GPIOx;
	GPIO_PinConfiguration pinConfiguration;
} GPIO_Info_t;

/**
 * \brief A function used to initialise a GPIO pin
 * \return SUCCESS if initialization succeeds, fail otherwise
 */
HAL_Status GPIOInitialize(GPIO_Info_t * gpioConfig);

/**
 * \brief A function that writes a value to the given GPIO pin within a port
 */
void GPIOWritePin(GPIOx_t * GPIOx, uint16_t pinNumber, GPIO_OutputState_e logicLevel);

/**
 * \brief A function that enables the peripheral clock for the input GPIO
 */
void GPIOEnablePeriphClk(GPIOx_t * GPIOx, HAL_LogicLevel enDi);

/**
 * \brief A function that enables / disables an IRQ
 */
void GPIOConfigureIRQn(uint8_t IRQn, HAL_LogicLevel enDi);

/**
 * \brief A function that sets the interrupt priority for a given IRQ number
 */
void GPIOSetIRQPriority(uint8_t IRQn, uint8_t priorityNumber);

/**
 * \brief A function that handles interrupts
 */
void GPIOIRQHandler(uint8_t pinNumber);
#endif
