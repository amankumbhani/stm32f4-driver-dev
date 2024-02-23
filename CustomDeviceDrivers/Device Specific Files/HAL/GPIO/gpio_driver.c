#include "gpio_driver.h"

HAL_Status GPIOInitialize(GPIO_Info_t * gpioConfig)
{
	HAL_Status lRetVal_e = E_FAIL;

	/** Configure the mode of the GPIO, i.e., input, output, alternate or analog */
	if (gpioConfig->pinConfiguration.mode <= E_GPIO_ANALOG)
	{
		gpioConfig->GPIOx->MODER |= (gpioConfig->pinConfiguration.mode << 2 * gpioConfig->pinConfiguration.pinNumber);
	}
	else
	{
		if (gpioConfig->pinConfiguration.mode == E_GPIO_IT_RISING)
		{
			/** Configure the RTSR register for rising edge triggers */
			EXTI->RSTR |= (1u << gpioConfig->pinConfiguration.pinNumber);
			EXTI->FTSR &= ~(1u << gpioConfig->pinConfiguration.pinNumber);
		}
		else if (gpioConfig->pinConfiguration.mode == E_GPIO_IT_FALLING)
		{
			/** Configure the FTSR for falling edge triggers */
			EXTI->FTSR |= (1u << gpioConfig->pinConfiguration.pinNumber);
			EXTI->RSTR &= ~(1u << gpioConfig->pinConfiguration.pinNumber);
		}

		/** Enable SYSCFG clock */
		// TODO: Do this properly within a macro function
		RCC->APB2ENR |= (1u << 14u);

		/** Configure the SYSCFG_EXTI register to select the input GPIO to the corresponding EXTI line */
		uint8_t gpioPort = GPIO_BASEADDR_TO_PORT_CODE(gpioConfig->GPIOx);

		SYSCFG->EXTICR[(gpioConfig->pinConfiguration.pinNumber / 4)] |= (gpioPort << ((gpioConfig->pinConfiguration.pinNumber % 4) * 4));

		/** Enable the interrupt (unmask it) */
		EXTI->IMR |= (1u << gpioConfig->pinConfiguration.pinNumber);
	}

	/** Configure the alternate functionality required on the GPIO pin */
	gpioConfig->GPIOx->AFR[ gpioConfig->pinConfiguration.pinNumber / 8U] |= gpioConfig->pinConfiguration.alternateFunc << (gpioConfig->pinConfiguration.pinNumber % (uint16_t)8);

	/** Configure the speed of the GPIO */
	gpioConfig->GPIOx->OSPEEDR |= (gpioConfig->pinConfiguration.speed << 2 * gpioConfig->pinConfiguration.pinNumber);

	/** Configure the type of the GPIO, i.e., open drain or push pull */
	gpioConfig->GPIOx->OTYPER |= (gpioConfig->pinConfiguration.type << gpioConfig->pinConfiguration.pinNumber);

	/** Configure the pull ups of the GPIO */
	gpioConfig->GPIOx->PUPDR |= (gpioConfig->pinConfiguration.pullUpDownConf << 2 * gpioConfig->pinConfiguration.pinNumber);

	return lRetVal_e;
}

void GPIOWritePin(GPIOx_t * GPIOx, uint16_t pinNumber, GPIO_OutputState_e logicLevel)
{
	if (logicLevel == 1)
	{
		GPIOx->ODR |= (1U << pinNumber);
	}
	else
	{
		GPIOx->ODR &= ~(1U << pinNumber);
	}
}

void GPIOConfigureIRQn(uint8_t IRQn, HAL_LogicLevel enDi)
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

void GPIOEnablePeriphClk(GPIOx_t * GPIOx, HAL_LogicLevel enDi)
{
	// TODO: Refactor this code
	if (enDi == E_EN)
	{
		if (GPIOx == GPIOA)
		{
			RCC->AHB1ENR |= (1U << 0U);
		}
		else if (GPIOx == GPIOB)
		{
			RCC->AHB1ENR |= (1U << 1U);
		}
		else if (GPIOx == GPIOC)
		{
			RCC->AHB1ENR |= (1U << 2U);
		}
		else if (GPIOx == GPIOD)
		{
			RCC->AHB1ENR |= (1U << 3U);
		}
		else if (GPIOx == GPIOE)
		{
			RCC->AHB1ENR |= (1U << 4U);
		}
		else if (GPIOx == GPIOH)
		{
			RCC->AHB1ENR |= (1U << 7U);
		}
		else
		{
			// TODO: Add handling for invalid GPIO enable
		}
	}
	else
	{
		if (GPIOx == GPIOA)
		{
			RCC->AHB1ENR &= ~(1U << 0U);
		}
		else if (GPIOx == GPIOB)
		{
			RCC->AHB1ENR &= ~(1U << 1U);
		}
		else if (GPIOx == GPIOC)
		{
			RCC->AHB1ENR &= ~(1U << 2U);
		}
		else if (GPIOx == GPIOD)
		{
			RCC->AHB1ENR &= ~(1U << 3U);
		}
		else if (GPIOx == GPIOE)
		{
			RCC->AHB1ENR &= ~(1U << 3U);
		}
		else if (GPIOx == GPIOH)
		{
			RCC->AHB1ENR &= ~(1U << 7U);
		}
		else
		{
			// TODO: Add handling for invalid GPIO enable
		}
	}
}

void GPIOSetIRQPriority(uint8_t IRQn, uint8_t priorityNumber)
{
	uint8_t prxRegister = IRQn / 4;
	uint8_t prxSection = IRQn % 4;

	uint8_t shiftAmount = (prxSection * 8U) + (8 - 4); // TODO: Make a macro for 4, as it is the number of priority bits implemented in this MCU

	*(NVIC_IPR0 + (prxRegister * 4)) |= (1 << shiftAmount);
}

void GPIOIRQHandler(uint8_t pinNumber)
{
	if (EXTI->PR & (1 << pinNumber))
	{
		/** Clear the pending register bit */
		EXTI->PR |= (1 << pinNumber);
	}
}
