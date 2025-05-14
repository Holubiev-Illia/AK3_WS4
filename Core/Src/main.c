/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_GRN   GPIO_PIN_12
#define LED_ORG   GPIO_PIN_13
#define LED_RED   GPIO_PIN_14
#define LED_BLU   GPIO_PIN_15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void ConfigureSysClock(void);
static void InitGPIO(void);
static void InitUSART3(void);
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  ConfigureSysClock();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  InitGPIO();
  InitUSART3();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t byteBuf[1];
    HAL_StatusTypeDef uartStatus;

    uartStatus = HAL_UART_Receive(&huart3, byteBuf, 1, 10);

    if (uartStatus == HAL_OK)
    {
      switch (byteBuf[0])
      {
        case '1':
          HAL_GPIO_TogglePin(GPIOD, LED_GRN);
          if (HAL_GPIO_ReadPin(GPIOD, LED_GRN))
            HAL_UART_Transmit(&huart3, (uint8_t *)" - Green ON\r\n", 13, 100);
          else
            HAL_UART_Transmit(&huart3, (uint8_t *)" - Green OFF\r\n", 14, 100);
          break;

        case '2':
          HAL_GPIO_TogglePin(GPIOD, LED_ORG);
          if (HAL_GPIO_ReadPin(GPIOD, LED_ORG))
            HAL_UART_Transmit(&huart3, (uint8_t *)" - Orange ON\r\n", 14, 100);
          else
            HAL_UART_Transmit(&huart3, (uint8_t *)" - Orange OFF\r\n", 15, 100);
          break;

        case '3':
          HAL_GPIO_TogglePin(GPIOD, LED_RED);
          if (HAL_GPIO_ReadPin(GPIOD, LED_RED))
            HAL_UART_Transmit(&huart3, (uint8_t *)" - Red ON\r\n", 11, 100);
          else
            HAL_UART_Transmit(&huart3, (uint8_t *)" - Red OFF\r\n", 12, 100);
          break;

        case '4':
          HAL_GPIO_TogglePin(GPIOD, LED_BLU);
          if (HAL_GPIO_ReadPin(GPIOD, LED_BLU))
            HAL_UART_Transmit(&huart3, (uint8_t *)" - Blue ON\r\n", 12, 100);
          else
            HAL_UART_Transmit(&huart3, (uint8_t *)" - Blue OFF\r\n", 13, 100);
          break;

        case '5':
          HAL_GPIO_WritePin(GPIOD, LED_GRN | LED_ORG | LED_RED | LED_BLU, GPIO_PIN_SET);
          HAL_UART_Transmit(&huart3, (uint8_t *)" - All LEDs ON\r\n", 16, 100);
          break;

        case '6':
          HAL_GPIO_WritePin(GPIOD, LED_GRN | LED_ORG | LED_RED | LED_BLU, GPIO_PIN_RESET);
          HAL_UART_Transmit(&huart3, (uint8_t *)" - All LEDs OFF\r\n", 17, 100);
          break;

        default:
          HAL_UART_Transmit(&huart3, (uint8_t *)" - UnexpCmd\r\n", 13, 100);
          break;
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void ConfigureSysClock(void)
{
  RCC_OscInitTypeDef oscSettings = {0};
  RCC_ClkInitTypeDef clkSettings = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  oscSettings.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  oscSettings.HSIState = RCC_HSI_ON;
  oscSettings.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscSettings.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&oscSettings) != HAL_OK)
  {
    Error_Handler();
  }

  clkSettings.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                          RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clkSettings.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clkSettings.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkSettings.APB1CLKDivider = RCC_HCLK_DIV1;
  clkSettings.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&clkSettings, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void InitUSART3(void)
{
  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void InitGPIO(void)
{
  GPIO_InitTypeDef pinCfg = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, LED_GRN | LED_ORG | LED_RED | LED_BLU, GPIO_PIN_RESET);

  pinCfg.Pin = LED_GRN | LED_ORG | LED_RED | LED_BLU;
  pinCfg.Mode = GPIO_MODE_OUTPUT_PP;
  pinCfg.Pull = GPIO_NOPULL;
  pinCfg.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &pinCfg);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
