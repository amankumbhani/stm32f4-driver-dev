/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f411re.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static void GPIO_CONFIGURE(void);
static void SPI_Init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
	GPIO_CONFIGURE();
	SPI_Init();
	SPIEnablePeripheral(SPI1, E_EN);
	SPISSIConfig(SPI1, E_EN);

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */
	char data[] = "Hello world";
	SPISendData(SPI1, (uint8_t *)data, strlen(data));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

static void SPI_Init()
{
	// TODO: Add this inside the SPI initialize function
	SPIEnablePeriphClk(SPI1, E_EN);

	SPI_Info_t spiInfo = {0};

	spiInfo.SPIx = SPI1;
	spiInfo.spiConfiguration.e_deviceConfig = E_SPI_MASTER;
	spiInfo.spiConfiguration.e_busConfig = E_SPI_FULL_DUPLEX;
	spiInfo.spiConfiguration.e_clockSpeed = E_SPI_CLK_DIV_2;
	spiInfo.spiConfiguration.e_dataFormat = E_8BIT_DATA;
	spiInfo.spiConfiguration.e_commMode = E_SPI_MODE_0;
	spiInfo.spiConfiguration.e_nssConfig = E_NSS_SSM_EN;

	SPIInitialize(&spiInfo);
}

static void GPIO_CONFIGURE(void)
{
//	/** Enable the GPIO peripheral clock */
//	GPIOEnablePeriphClk(GPIOA, E_EN);
//	GPIOEnablePeriphClk(GPIOC, E_EN);
//
//	GPIO_Info_t gpioInfo = {0};
//	gpioInfo.GPIOx = GPIOA;
//	gpioInfo.pinConfiguration.mode = E_GPIO_OUTPUT;
//	gpioInfo.pinConfiguration.pinNumber = E_GPIO_PIN_5;
//	gpioInfo.pinConfiguration.type = E_GPIO_PUSHPULL;
//	gpioInfo.pinConfiguration.pullUpDownConf = E_GPIO_NO_PULLUP;
//	gpioInfo.pinConfiguration.speed = E_GPIO_LOW_SPEED;
//	GPIOInitialize(&gpioInfo);
//
//	memset(&gpioInfo, 0, sizeof(gpioInfo));
//
//	gpioInfo.GPIOx = GPIOC;
//	gpioInfo.pinConfiguration.mode = E_GPIO_IT_FALLING;
//	gpioInfo.pinConfiguration.pinNumber = E_GPIO_PIN_13;
//	gpioInfo.pinConfiguration.type = E_GPIO_PUSHPULL;
//	gpioInfo.pinConfiguration.pullUpDownConf = E_GPIO_NO_PULLUP;
//	gpioInfo.pinConfiguration.speed = E_GPIO_LOW_SPEED;
//	GPIOInitialize(&gpioInfo);
//
//	/** Disable interrupts */
//	GPIOSetIRQPriority(40, 3);
//	GPIOConfigureIRQn(40, E_DI);

	// PA5 -> SPI1_SCK
	// PA6 -> SPI1_MISO
	// PA7 -> SPI1_MOSI
	// PA4 -> SPI1_NSS

	/** Enable peripheral clocks for GPIOs */
	// TODO: Add this inside the GPIOInitialize function
	GPIOEnablePeriphClk(GPIOA, E_EN);

	GPIO_Info_t gpioInfo = {0};

	gpioInfo.GPIOx = GPIOA;
	gpioInfo.pinConfiguration.pinNumber = E_GPIO_PIN_5;
	gpioInfo.pinConfiguration.mode = E_GPIO_ALTERNATE;
	gpioInfo.pinConfiguration.alternateFunc = E_GPIO_ALT_FUNC_5;
	gpioInfo.pinConfiguration.speed = E_GPIO_FAST_SPEED;
	gpioInfo.pinConfiguration.pullUpDownConf = E_GPIO_NO_PULLUP;
	gpioInfo.pinConfiguration.type = E_GPIO_PUSHPULL;
	GPIOInitialize(&gpioInfo);

	gpioInfo.pinConfiguration.pinNumber = E_GPIO_PIN_6;
	GPIOInitialize(&gpioInfo);

	gpioInfo.pinConfiguration.pinNumber = E_GPIO_PIN_7;
	GPIOInitialize(&gpioInfo);

	gpioInfo.pinConfiguration.pinNumber = E_GPIO_PIN_4;
	GPIOInitialize(&gpioInfo);
}

void EXTI15_10_IRQHandler(void)
{
	/** TODO: Check which EXTI peripheral has interrupted & based on that, call the GPIOIRQHandler with that pin number */
	static _Bool value = 0;
	value = !value;
	GPIOIRQHandler(13);
	GPIOWritePin(GPIOA, E_GPIO_PIN_5, value);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
