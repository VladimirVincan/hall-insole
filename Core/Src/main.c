/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * @brief 3D Hall 9 Register Map.
 * @details Specified register map of 3D Hall 9 Click driver.
 */
#define C3DHALL9_REG_EEPROM_02                  0x02
#define C3DHALL9_REG_EEPROM_03                  0x03
#define C3DHALL9_REG_EEPROM_0D                  0x0D
#define C3DHALL9_REG_EEPROM_0E                  0x0E
#define C3DHALL9_REG_EEPROM_0F                  0x0F
#define C3DHALL9_REG_VOLATILE_27                0x27
#define C3DHALL9_REG_VOLATILE_28                0x28
#define C3DHALL9_REG_VOLATILE_29                0x29
#define C3DHALL9_REG_CUSTOMER_ACCESS            0x35

/**
 * @brief 3D Hall 9 register setting.
 * @details Specified setting for registers of 3D Hall 9 Click driver.
 */
#define C3DHALL9_EEPROM_02_I2C_THRESHOLD_18V    0x00000200ul
#define C3DHALL9_EEPROM_02_DISABLE_SLAVE_ADC    0x00020000ul
#define C3DHALL9_EEPROM_02_ENABLE_Z             0x00000100ul
#define C3DHALL9_EEPROM_02_ENABLE_Y             0x00000080ul
#define C3DHALL9_EEPROM_02_ENABLE_X             0x00000040ul
#define C3DHALL9_VOLATILE_27_ACTIVE_MODE        0x00000000ul
#define C3DHALL9_VOLATILE_27_SLEEP_MODE         0x00000001ul
#define C3DHALL9_VOLATILE_27_LOW_POWER_MODE     0x00000002ul
#define C3DHALL9_VOLATILE_27_I2C_SINGLE         0x00000000ul
#define C3DHALL9_VOLATILE_27_I2C_FAST_LOOP      0x00000004ul
#define C3DHALL9_VOLATILE_27_I2C_FULL_LOOP      0x00000008ul
#define C3DHALL9_REG_CUSTOMER_ACCESS_CODE       0x2C413534ul

#define C3DHALL9_EEPROM_02_I2C_ADDR_MIN         0x00000400ul
#define C3DHALL9_EEPROM_02_I2C_ADDR_MAX         0x0001FC00ul

/**
 * @brief 3D Hall 9 measurements values.
 * @details Specified values for measurements of 3D Hall 9 Click driver.
 */
#define C3DHALL9_12BIT_RESOLUTION               0x1000
#define C3DHALL9_SIGN_BIT                       0x0800
#define C3DHALL9_DATA_READY_BIT                 0x0080
#define C3DHALL9_GAUSS_RESOLUTION               4.0
#define C3DHALL9_TEMPERATURE_MULTIPLIER         302
#define C3DHALL9_TEMPERATURE_SUBTRACTOR         1708
#define C3DHALL9_HALF_CICRLE_DEGREES            180.0
#define C3DHALL9_MATH_TWO_PI                    6.28318530717958

/**
 * @brief 3D Hall 9 device address setting.
 * @details Specified setting for device slave address selection of
 * 3D Hall 9 Click driver.
 */
#define C3DHALL9_SET_DEV_ADDR                   0x60
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t I2C_DEFAULT_ADDR = 0x60;
static const uint8_t I2C_DEFAULT_RD_ADDR = 0xc1;
static const uint8_t I2C_DEFAULT_WR_ADDR = 0xc0;

static const uint8_t REGISTER_MSB_ADDR = 0x28;
static uint32_t REGISTER_MSB = 0x00;

static const uint8_t REGISTER_LSB_ADDR = 0x29;
static uint32_t REGISTER_LSB = 0x00;

static const uint8_t REGISTER_TEST_ADDR = 0x02;
static uint32_t REGISTER_TEST = 0x00;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void read_register(const uint8_t i2c_addr, const uint8_t reg_addr, uint32_t *reg)
{
	uint8_t buf[4];
	uint8_t transmit[32];
	HAL_StatusTypeDef ret;

	const uint8_t i2c_rd_addr = (i2c_addr << 1) | 0x1;
	const uint8_t i2c_wr_addr = (i2c_addr << 1) | 0x0;

	buf[0] = reg_addr;
	ret = HAL_I2C_Master_Transmit(&hi2c1, i2c_wr_addr, buf, 1, HAL_MAX_DELAY);
	HAL_Delay(100);
	if (ret != HAL_OK) {
		strcpy((char*)transmit, "TxEr\r\n");
		HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
	}
	else {
		ret = HAL_I2C_Master_Receive(&hi2c1, i2c_rd_addr, buf, 4, HAL_MAX_DELAY);
		HAL_Delay(100);
		if (ret != HAL_OK) {
			strcpy((char*)transmit, "RxEr\r\n");
			HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
		}
	}

	/*sprintf(transmit, "%x_[i2c_wr]\r\n", I2C_DEFAULT_WR_ADDR);
	HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
	sprintf(transmit, "%x_[i2c_rd]\r\n", I2C_DEFAULT_RD_ADDR);
	HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
	sprintf(transmit, "%x_[reg_addr]\r\n", reg_addr);
	HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
	sprintf(transmit, "%x_[3]\r\n", buf[3]);
	HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
	sprintf(transmit, "%x_[2]\r\n", buf[2]);
	HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
	sprintf(transmit, "%x_[1]\r\n", buf[1]);
	HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);
	sprintf(transmit, "%x_[0]\r\n", buf[0]);
	HAL_UART_Transmit(&huart2, transmit, strlen((char*)transmit), HAL_MAX_DELAY);*/


	// bug is here!!! 0 1 2 3
	(*reg) = buf[0];
	(*reg) <<= 8;
	(*reg) |= buf[1];
	(*reg) <<= 8;
	(*reg) |= buf[2];
	(*reg) <<= 8;
	(*reg) |= buf[3];
}

void write_register(const uint8_t i2c_addr, const uint8_t reg_addr, const uint32_t reg)
{
	HAL_StatusTypeDef ret;
	uint8_t buf[5];
	uint8_t transmit[32];

	const uint8_t i2c_wr_addr = (i2c_addr << 1) | 0x0;

	buf[0] = reg_addr;
	buf[1] = (uint8_t)((reg >> 24) & 0xFF);
	buf[2] = (uint8_t)((reg >> 16) & 0xFF);
	buf[3] = (uint8_t)((reg >> 8) & 0xFF);
	buf[4] = (uint8_t)(reg & 0xFF);

	ret = HAL_I2C_Master_Transmit(&hi2c1, i2c_wr_addr, buf, 5, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		strcpy((char*)transmit, "WrEr\r\n");
	}
}

void init_sensor(const uint8_t i2c_addr)
{
	write_register(i2c_addr, C3DHALL9_REG_EEPROM_02,
			C3DHALL9_EEPROM_02_ENABLE_X |
			C3DHALL9_EEPROM_02_ENABLE_Y |
			C3DHALL9_EEPROM_02_ENABLE_Z |
			C3DHALL9_EEPROM_02_I2C_THRESHOLD_18V |
			C3DHALL9_EEPROM_02_DISABLE_SLAVE_ADC |
			C3DHALL9_EEPROM_02_I2C_ADDR_MIN);
	write_register(i2c_addr, C3DHALL9_REG_CUSTOMER_ACCESS, C3DHALL9_REG_CUSTOMER_ACCESS_CODE);
}

float get_x()
{
	int16_t val = ((REGISTER_MSB >> 20) & 0x0FF0) | ((REGISTER_LSB >> 16) & 0x0F);
	val = (val ^ C3DHALL9_SIGN_BIT) - C3DHALL9_SIGN_BIT;
	float ret = (float)val / C3DHALL9_GAUSS_RESOLUTION;
	return ret;
}

float get_y()
{
	int16_t val = ((REGISTER_MSB >> 12) & 0x0FF0) | ((REGISTER_LSB >> 12) & 0x0F);
	val = (val ^ C3DHALL9_SIGN_BIT) - C3DHALL9_SIGN_BIT;
	float ret = (float)val / C3DHALL9_GAUSS_RESOLUTION;
	return ret;
}

float get_z()
{
	int16_t val = ((REGISTER_MSB >> 4) & 0x0FF0) | ((REGISTER_LSB >> 8) & 0x0F);
	val = (val ^ C3DHALL9_SIGN_BIT) - C3DHALL9_SIGN_BIT;
	float ret = (float)val / C3DHALL9_GAUSS_RESOLUTION;
	return ret;
}

float get_temp()
{
	int32_t val = ((REGISTER_MSB & 0x0000003F) << 6) | (REGISTER_LSB & 0x0000003F);
	float ret = C3DHALL9_TEMPERATURE_MULTIPLIER*((float)val - C3DHALL9_TEMPERATURE_SUBTRACTOR)/C3DHALL9_12BIT_RESOLUTION;
	return ret;
}

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buf[32];
  init_sensor(I2C_DEFAULT_ADDR);
  while (1)
  {
//	   read_register(I2C_DEFAULT_ADDR, REGISTER_MSB_ADDR, &REGISTER_MSB);
//	   read_register(I2C_DEFAULT_ADDR, REGISTER_LSB_ADDR, &REGISTER_LSB);
//
//	   sprintf(buf, "x : %.1f\ny : %.1f\nz : %.1f\ntemp : %.1f\n", get_x(), get_y(), get_z(), get_temp());
//	   HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//	   HAL_UART_Transmit(&huart2, &REGISTER_MSB, 4, HAL_MAX_DELAY);

       // C3DHALL9_REG_EEPROM_02
	   // C3DHALL9_REG_VOLATILE_28
	   read_register(I2C_DEFAULT_ADDR, C3DHALL9_REG_EEPROM_02, &REGISTER_TEST);
	   sprintf(buf, "%x_\r\n", REGISTER_TEST);
	   HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
	   HAL_Delay(1000);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
