/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author			: Fidel Quezada Guzman
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
#include "stdio.h"						// Standard C Lib
#include "stm32l475e_iot01_tsensor.h"	// Temperature Sensor Lib

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// User Input Buffer
#define UART_RX_BUFFER_SIZE 32

// Coffee Size Selection Input Pins
#define SCUP_6OZ_D0 ARD_D0_Pin		// PA1
#define SMUG_8OZ_D1 ARD_D1_Pin		// PA0
#define LMUG_10OZ_D2 ARD_D2_Pin		// PD14
#define TMUG_12OZ_D4 ARD_D4_Pin		// PA3

// Refill Water to 100% Input Pins
#define REFILL_WATER_D5 ARD_D5_Pin	// PB4

// Strong Brew On/Off
#define STRONG_BREW_D8 ARD_D8_Pin	// PB2

// LED Output Pins
#define MAIN_POWER_D9 ARD_D9_Pin		// PA15
#define STRONG_BREW_D10 ARD_D10_Pin		// PA2
#define WATER_LOW_D11 ARD_D11_Pin		// PA7
#define AUTO_OFF_D12 ARD_D12_Pin		// PA6
#define BREWING_D13 ARD_D13_Pin			// PA5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;		// ADC Handler
I2C_HandleTypeDef hi2c2;		// I2C Handler
UART_HandleTypeDef huart1;		// UART Handler

/* USER CODE BEGIN PV */

// User Input Buffer
uint16_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

// Status Booleans
_Bool power = 0;					// Power On/Off, initialized to false
_Bool strongBrew = 0;				// Strong Brew On/Off, initialized to false

// 8-Bit Integer Variables
uint8_t userCupSizeInput = 0;		// User input for Cup Size Selection
uint8_t waterLevel = 100;			// Water reservoir, default 100oz
uint8_t autoOffTimer = 60;			// Timer counter for Auto-Off Functionality

// 16-Bit Analog Reading Variables
 uint16_t adcPOT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

// UART1 (TX/RX) User Input
void receive_char(char* rx_char);				// User Input Characters
void receive_int(uint8_t* rx_byte);				// User Input Integer

// Toggle Boolean
void togglePower(void);							// Power On/Off
void toggleBrewStrength(void);					// Brew Strength On/Off

// Coffee Progress Bar Animation
void updateProgressBar(int progress, int oz);

// Print Return
void brewCompletion(void);						// Coffee Brewing Strength
void getWaterTemp(void);						// Water Temperature (Simulated)

// Get Water Level from POT Prototype
int adcWaterLevel(void);						// Read ADC POT position
void refillWater(int adcPOT);					// Set Water Level based on ADC Input

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  // Initialize HTS221 Sensor for Temperature Reading
  BSP_TSENSOR_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // Local Variables

  /* variables for Serial text Input option
  char charInput;				// User Input Char for Write/Read Option
  uint8_t intInput = 0;			// User Int Input
  */

  // Coffee Sizes Selections
  const uint8_t smallCupSize = 1;			// 6oz Cup Size
  const uint8_t smallMugSize = 2;			// 8oz Cup Size
  const uint8_t largeMugSize = 3;			// 10oz Cup Size
  const uint8_t travelMugSize = 4;			// 12oz Cup Size

  // Initial User Prompt
  printf("\n\n\n\n\n");
  printf("****************************************\n");
  printf("*           UCSD Spring 2023           *\n");
  printf("*       Embedded C Programming         *\n");
  printf("*        Fidel Quezada Guzman          *\n");
  printf("****************************************\n");
  printf("\n\n");
  printf("\tFidelicious Coffee Maker\n");
  printf("\nPress the ON Button to Start the Revival Process\n");
  printf("Turning OFF in: ");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (autoOffTimer > 0)
  {
	  // Power ON or Auto Off Message
	  if(autoOffTimer > 1)
	  {
		  printf("%d Seconds\n", autoOffTimer);
	  }
	  else
		  printf("%d Second\n", autoOffTimer);
	  autoOffTimer--;
	  HAL_Delay(1000);

	  // Program Start when User Button Pressed under 60s
	  if (power)
	  {
		  // Reset Auto-Off
		  autoOffTimer = 60;

		  // Power On Initial Message
		  printf("\n\n****************************************\n");
		  printf("*       Fidelicious Coffee Maker       *\n");
		  printf("****************************************\n");
		  printf("\nStrong Brew: %d", strongBrew);

		  // Hold while User makes selection & Power is On
		  while (userCupSizeInput == 0 && power == 1)
		  {
			  // User Selects Coffee Size
			  while(power == 1)
			  {
				  // Water Level Low, have User Refill (handled via interrupt function)
				  if (waterLevel < 6  && power == 1)
				  {
					  printf("\nWater Level is too low!!\nPlease Refill & Click on the 'Refill Done' Button\n");

					  while (waterLevel < 6)
					  {
						  // Add Water LEDToggle when Water Low
						  HAL_GPIO_TogglePin(GPIOA, WATER_LOW_D11);
						  HAL_Delay(1000);
					  }
				  }

				  // Water Level Enough to Proceed & Power On
				  if (power == 1 && userCupSizeInput == 0)
				  {
					  // Turn Water Low LED OFF
					  HAL_GPIO_WritePin(GPIOA, WATER_LOW_D11, GPIO_PIN_RESET);

					  // Print Cup Sizes to UART
					  printf("\n\nPlease Select a Coffee Size:\n");
					  printf("1) Small Cup	(6oz)\n");
					  printf("2) Small Mug 	(8oz)\n");
					  printf("3) Large Mug 	(10oz)\n");
					  printf("4) Travel Mug	(12oz)\n\n");

					  // Water Level & Temperature
					  printf("Water Level: %d oz, ", waterLevel);
					  getWaterTemp();
					  printf("\n\n");

					  // Hold till user makes selection
					  while (userCupSizeInput == 0)
					  {
						  HAL_Delay(100);
					  }
				  }

				  // Small Cup (6oz) Selected
				  if (userCupSizeInput == smallCupSize && waterLevel >= 6 && power == 1)
				  {
					  // Brewing in Progress LED ON
					  HAL_GPIO_WritePin(GPIOA, BREWING_D13, GPIO_PIN_SET);

					  // Progress Bar Animation & Brew Strength UART Print
					  printf("\tBrewing: Small Cup (6oz)\n");
					  for (int i = 0; i <= 6; i++)
					  {
						  updateProgressBar(i, 6);
					  }
					  brewCompletion();

					  // Value Updates
					  waterLevel = waterLevel - 6;			// Water Level
					  userCupSizeInput = 0;					// Reset Cup Selection

					  // Brew Progress LED OFF
					  HAL_GPIO_WritePin(GPIOA, BREWING_D13, GPIO_PIN_RESET);
				  }

				  // Small Mug (8oz) Selected
				  else if (userCupSizeInput == smallMugSize && waterLevel >= 8 && power == 1)
				  {
					  // Brewing in Progress LED ON
					  HAL_GPIO_WritePin(GPIOA, BREWING_D13, GPIO_PIN_SET);

					  // Progress Bar Animation & Brew Strength UART Print
					  printf("\tBrewing: Small Mug (8oz)\n");
					  // Progress Bar animation
					  for (int i = 0; i <= 8; i++)
					  {
						  updateProgressBar(i, 8);
					  }
					  brewCompletion();

					  // Value Updates
					  waterLevel = waterLevel - 8;			// Update Water Level
					  userCupSizeInput = 0;					// Reset Cup Selection

					  // Brew Progress LED OFF
					  HAL_GPIO_WritePin(GPIOA, BREWING_D13, GPIO_PIN_RESET);
				  }

				  // Large Mug (10oz) Selected
				  else if (userCupSizeInput == largeMugSize && waterLevel >= 10 && power == 1)
				  {
					  // Brewing in Progress LED ON
					  HAL_GPIO_WritePin(GPIOA, BREWING_D13, GPIO_PIN_SET);

					  // Progress Bar Animation & Brew Strength UART Print
					  printf("\tBrewing: Large Mug (10oz)\n");
					  // Progress Bar animation
					  for (int i = 0; i <= 10; i++)
					  {
						  updateProgressBar(i, 10);
					  }
					  brewCompletion();

					  // Value Updates
					  waterLevel = waterLevel - 10;			// Update Water Level
					  userCupSizeInput = 0;					// Reset Cup Selection

					  // Brew Progress LED OFF
					  HAL_GPIO_WritePin(GPIOA, BREWING_D13, GPIO_PIN_RESET);
				  }

				  // Travel Mug (12oz) Selected
				  else if (userCupSizeInput == travelMugSize && waterLevel >= 12 && power == 1)
				  {
					  // Brewing in Progress LED ON
					  HAL_GPIO_WritePin(GPIOA, BREWING_D13, GPIO_PIN_SET);

					  // Progress Bar Animation & Brew Strength UART Print
					  printf("\tBrewing: Travel Mug (12oz)\n");
					  // Progress Bar animation
					  for (int i = 0; i <= 12; i++)
					  {
						  updateProgressBar(i, 12);
					  }
					  brewCompletion();

					  // Value Updates
					  waterLevel = waterLevel - 12;			// Update Water Level
					  userCupSizeInput = 0;					// Reset Cup Selection

					  // Brew Progress LED OFF
					  HAL_GPIO_WritePin(GPIOA, BREWING_D13, GPIO_PIN_RESET);
				  }
			  }
		  }
	  }

	  // Auto-Off reached, Reboot main Power
	  if (autoOffTimer == 0)
	  {
		  printf("\nPowering OFF\nReboot Main Power\n");
		  // Auto-Off LED ON
		  HAL_GPIO_WritePin(GPIOA, AUTO_OFF_D12, GPIO_PIN_SET);	// ARD_D11 Port
	  }
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
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin
                          |SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin|SPSGRF_915_SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin */
  GPIO_InitStruct.Pin = SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin|ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin
                           SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin
                          |SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ARD_D5_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ARD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DFSDM1_DATIN2_Pin DFSDM1_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM1_DATIN2_Pin|DFSDM1_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : QUADSPI_CLK_Pin QUADSPI_NCS_Pin OQUADSPI_BK1_IO0_Pin QUADSPI_BK1_IO1_Pin
                           QUAD_SPI_BK1_IO2_Pin QUAD_SPI_BK1_IO3_Pin */
  GPIO_InitStruct.Pin = QUADSPI_CLK_Pin|QUADSPI_NCS_Pin|OQUADSPI_BK1_IO0_Pin|QUADSPI_BK1_IO1_Pin
                          |QUAD_SPI_BK1_IO2_Pin|QUAD_SPI_BK1_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin SPSGRF_915_SDN_Pin
                           SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin|SPSGRF_915_SDN_Pin
                          |SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_UART3_TX_Pin INTERNAL_UART3_RX_Pin */
  GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_ID_Pin USB_OTG_FS_DM_Pin USB_OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin|USB_OTG_FS_DM_Pin|USB_OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_SPI3_SCK_Pin INTERNAL_SPI3_MISO_Pin INTERNAL_SPI3_MOSI_Pin */
  GPIO_InitStruct.Pin = INTERNAL_SPI3_SCK_Pin|INTERNAL_SPI3_MISO_Pin|INTERNAL_SPI3_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// UART1 (TX/RX) User Input Characters Functions
void receive_char(char* rx_char)
{
  HAL_UART_Receive(&huart1, (uint8_t*) rx_char, 1, HAL_MAX_DELAY);
}

// UART1 (TX/RX) User Input Hex Functions
void receive_int(uint8_t* rx_byte)
{
  char buffer[3];  // Buffer to hold received data

  HAL_UART_Receive(&huart1, (uint8_t*) buffer, 2, HAL_MAX_DELAY);
  buffer[2] = '\0';  // Add null terminator to buffer
  *rx_byte = strtol(buffer, NULL, 16);  // Convert buffer to integer
}


/*---- Standard ARM CMSIS Library functions ----*/
//  Sending to Serial Port
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, sizeof(uint8_t), HAL_MAX_DELAY);
	return ch;
}

// Receiving from Serial Port
int __io_getchar(void)
{
	uint8_t ch;
	HAL_UART_Receive(&huart1, &ch, sizeof(ch), HAL_MAX_DELAY);
	return ch;
}

// GPIO Interrupt Handler Function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Start Main Program via Blue Board User Button
	if(GPIO_Pin == BUTTON_EXTI13_Pin)
	{
		// Start Main Program
		togglePower();

		// Main Power LED On/Off
		if(power == 1)
		{
			HAL_GPIO_WritePin(GPIOA, MAIN_POWER_D9, GPIO_PIN_SET);
		}
		else
			HAL_GPIO_WritePin(GPIOA, MAIN_POWER_D9, GPIO_PIN_RESET);
	}

	// Water Refill
	else if (GPIO_Pin == REFILL_WATER_D5)
	{
		// Refill Water to Level Selected on POT
		refillWater(adcPOT);

		// Turn Off Water Low LED & Print Water Level
		HAL_GPIO_WritePin(GPIOA, WATER_LOW_D11, GPIO_PIN_RESET);
		printf("Water Level: %d oz, ", waterLevel);
		getWaterTemp();
		printf("\n\n");
	}

	// StrongBrew On/Off
	else if (GPIO_Pin == STRONG_BREW_D8)
	{
		// Toggle strongBrew Bool
		toggleBrewStrength();

		// Toggle StrongBrew LED & Print Strong Brew Status
		if(strongBrew == 1)
		{
			HAL_GPIO_WritePin(ARD_D10_GPIO_Port, STRONG_BREW_D10, GPIO_PIN_SET);
		}
		else
			HAL_GPIO_WritePin(ARD_D10_GPIO_Port, STRONG_BREW_D10, GPIO_PIN_RESET);

		printf("Strong Brew: %d\n", strongBrew);
	}

	// Small Cup (6oz) Selection
	if (GPIO_Pin == SCUP_6OZ_D0)
	{
		userCupSizeInput = 1;
	}

	// Small Mug (8oz) Selection
	else if (GPIO_Pin == SMUG_8OZ_D1)
	{
		userCupSizeInput = 2;
	}

	// Large Mug (10oz) Selection
	else if (GPIO_Pin == LMUG_10OZ_D2)
	{
		userCupSizeInput = 3;
	}

	// Travel Mug (12oz) Selection
	else if (GPIO_Pin == TMUG_12OZ_D4)
	{
		userCupSizeInput = 4;
	}
}

// Toggle Power Boolean
void togglePower(void)
{
    power = !power;
}

// Toggle Strong Brew Boolean
void toggleBrewStrength(void)
{
	strongBrew = !strongBrew;
}

// Coffee Brew Progress Bar Animation
void updateProgressBar(int progress, int oz)
{
	// Width constant/ variable
    const uint8_t barWidth = 40;
    uint8_t completedWidth = barWidth * progress / oz;

    char progressBar[barWidth + 1]; // +1 for null terminator
    memset(progressBar, ' ', barWidth);
    progressBar[barWidth] = '\0';

    // Fill Up Progress
    for (uint8_t i = 0; i < completedWidth; ++i)
    {
    	progressBar[i] = '*';
    }

    // Adjust size as needed
    char output[50];
    snprintf(output, sizeof(output), "[%s] %d oz\r\n", progressBar, progress);

    printf("%s", output);
    HAL_Delay(300);
}

// Coffee Brewing Strength Completion Print
void brewCompletion(void)
{
	if(strongBrew == 1)
	{
		printf("\tStrong Brew Complete! :)\n");
	}
	else if(strongBrew == 0)
	{
		printf("\tNormal Brew Complete! :)\n");
	}
}

// Get Water Temperature
void getWaterTemp(void)
{
	// Read Temperature, Store in variable, Print
	int temperature = BSP_TSENSOR_ReadTemp();
	printf("%d C", temperature);
}

// Get Water Level from POT
int adcWaterLevel(void)
{
	// Get ADC Value from POT
	HAL_ADC_Start(&hadc1);					// Start ACD Conversion
	HAL_ADC_PollForConversion(&hadc1,100);	// Allow for end of conversion in Polling Mode
	adcPOT = HAL_ADC_GetValue(&hadc1);		// Store ADC value gathered into variable(ADC Value: 0-4075)

	// Return ADC Value gathered
	return adcPOT;
}

// Input adcPOT value, Convert, output Water Level
void refillWater(int adcPOT)
{
	adcPOT = adcWaterLevel();

	if (adcPOT > 4000)
	{
		waterLevel = 100;
	}
	else if(adcPOT > 3000 && adcPOT < 4000)
	{
		waterLevel = 90;
	}
	else if(adcPOT > 2000 && adcPOT < 3000 )
	{
		waterLevel = 80;
	}
	else if(adcPOT > 1000 && adcPOT < 2000)
	{
		waterLevel = 70;
	}
	else if(adcPOT > 500 && adcPOT < 1000)
	{
		waterLevel = 60;
	}
	else if(adcPOT > 250 && adcPOT < 500)
	{
		waterLevel = 50;
	}
	else if(adcPOT > 125 && adcPOT < 250)
	{
		waterLevel = 40;
	}
	else if(adcPOT > 60 && adcPOT < 125)
	{
		waterLevel = 30;
	}
	else if(adcPOT > 30 && adcPOT < 60)
	{
		waterLevel = 20;
	}
	else if(adcPOT > 15 && adcPOT < 30)
	{
		waterLevel = 10;
	}
	else
		waterLevel = 0;
}

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
