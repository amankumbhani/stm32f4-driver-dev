#include "stm32f411re.h"
#include "stdint.h"

static uint16_t AHBPrescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint16_t APB1Prescaler[4] = {2, 4, 8, 16};
static uint8_t PLLNPrescaler[4] = {2, 4, 6, 8};

uint32_t RCC_GetPLLOutputClock()
{
	uint8_t selectedClockSource = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22);
	uint32_t pllInputClock = 0u;
	uint8_t mainPllInputClock = 0u;
	uint32_t pllOutputClock = 0u;

	if (selectedClockSource == 0)
	{
		/** HSI oscillator used as the PLL input clock */
		pllInputClock = HSI;
	}
	else
	{
		/** HSE oscillator used as the PLL input clock */
		pllInputClock = HSE;
	}
	mainPllInputClock = pllInputClock / (RCC->PLLCFGR & RCC_PLLCFGR_PLLM);

	uint16_t multiplicationFactor = 0u;
	uint8_t divisionFactor = 0u;

	multiplicationFactor = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
	divisionFactor = PLLNPrescaler[((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16)];

	pllOutputClock = (mainPllInputClock * multiplicationFactor) / divisionFactor;

	return pllOutputClock;
}

uint32_t RCC_GetPLK1Clock()
{
	/** Check the source for the system clock */
	uint8_t selectedClockSource = ((RCC->CFGR & RCC_CFGR_SWS_MASK) >> 2);
	uint32_t systemClock = 0u;
	uint8_t ahb1Prescaler = 0u;
	uint8_t apb1Prescaler = 0u;
	uint32_t ahb1Clock = 0u;
	uint32_t apb1Clock = 0u;

	if (selectedClockSource == 0)
	{
		/** HSI oscillator used as the system clock */
		systemClock = HSI;
	}
	else if (selectedClockSource == 1)
	{
		/** HSE oscillator used as the system clock */
		systemClock = HSE;
	}
	else if (selectedClockSource == 2)
	{
		/** PLL used as the system clock */
		systemClock = RCC_GetPLLOutputClock();
	}
	else
	{
		/** Do nothing, it is invalid! */
	}

	/** Divide the system clock by the AHB prescaler to obtain the AHB clock */
	uint16_t temp = ((RCC->CFGR & RCC_CFGR_HPRE_MASK) >> 4);
	if (temp < 8)
	{
		/** There is no division factor */
		ahb1Prescaler = 1;
	}
	else
	{
		ahb1Prescaler = AHBPrescaler[temp - 8];
	}
	ahb1Clock = systemClock / ahb1Prescaler;

	/** Divide the AHB clock by the APB1 prescaler */
	temp = 0u;
	temp = ((RCC->CFGR & RCC_CFGR_PPRE1_MASK) >> 10);

	if (temp < 4)
	{
		apb1Prescaler = 1;
	}
	else
	{
		apb1Prescaler = APB1Prescaler[temp - 4];
	}
	apb1Clock = ahb1Clock / apb1Prescaler;

	return apb1Clock;
}
