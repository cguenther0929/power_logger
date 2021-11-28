/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  * TODO Develop button input routines
  * TODO How to store screen buffer?  Multiple buffers or update just one buffer?
  * 
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct timing       time;
struct oled         oled;  
struct buttonStruct btn;
ad4681Data          a2d;
ad4681Data        * a2d_p;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

static const unsigned char logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
FATFS fs;
FATFS * pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t total, free_space;
char sd_buffer[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  setFont(&FreeSans9pt7b);
  setTextSize(1,1);             // 21 characters per line
  display_oled_init(SSD1306_SWITCHCAPVCC, SCREEN_WIDTH, SCREEN_HEIGHT);

  oled.current_screen = MAIN_SCREEN;

  /* Configure time keeping flags */
  time.led_fast_blink = false;
  time.flag_10ms_tick = false;
  time.flag_100ms_tick = false;
  time.flag_500ms_tick = false;
  
  /* Configure button-related flags */
  btn.up_btn_press_ctr = 0;   
  btn.rt_btn_press_ctr = 0;   
  btn.dn_btn_press_ctr = 0;   
  btn.lt_btn_press_ctr = 0;   

  btn.up_btn_pressed = false;
  btn.rt_btn_pressed = false;
  btn.dn_btn_pressed = false;
  btn.lt_btn_pressed = false;

  HAL_TIM_Base_Start(&htim2);			// Start timer #2 for us delay timer
  init_ad4681 ( &a2d );                 // Initialize the A2D


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
 
  /* USER CODE BEGIN 2 */
  print_string("Chip Reset.",LF);

  /**
   * TODO BEGIN
   * 
   * In the following block of code we are testing
   * the ability to write to the SD card
  */

  /* Mount SD Card */ 
  if ( f_mount ( & fs ,  "" ,  0 )  !=  FR_OK ) 
    blocking_us_delay(20);
    // _Error_Handler ( __FILE__ , __LINE__ ) ; 
  
  /* Open file to write */ 
  if ( f_open ( & fil ,  "first.txt" ,  FA_OPEN_ALWAYS  |  FA_READ  |  FA_WRITE )  != FR_OK ) 
    blocking_us_delay(20);
    // _Error_Handler ( __FILE__ , __LINE__ ) ; 
  
  /* Check free space */ 
  if ( f_getfree ( "" ,  & fre_clust ,  & pfs )  !=  FR_OK ) 
    blocking_us_delay(20);
    // _Error_Handler ( __FILE__ , __LINE__ ) ; 
  
  total =  ( uint32_t ) ( ( pfs -> n_fatent -  2 )  * pfs -> csize*  0.5 ) ; 
  free_space =  ( uint32_t ) ( fre_clust * pfs -> csize *  0.5 ) ;    
    
  /* Free space is less than 1kb */ 
  if ( free_space <  1 ) 
    blocking_us_delay(20);
    // _Error_Handler ( __FILE__ , __LINE__ ) ;   
  
  /* Write data to SD card */ 
  f_puts ( "STM32 SD Card I/O Example via SPI\n" ,  & fil ) ;   
  f_puts ( "Save the world!!!" ,  &fil ) ; 

  /* Close file */ 
  if ( f_close ( & fil )  !=  FR_OK ) 
    blocking_us_delay(20);
    // _Error_Handler ( __FILE__ , __LINE__ ) ; 

 /**
  * TODO ENG
  * This is the ending block
  * where we have tested writing to 
  * the SD card
  */




  /**
   * Draw Spalsh screen
   */
  // TODO need to define bitmap image, screen width, and splash screen dimensions
  // display_oled_drawBitmap((oled.screen_width - BITMAP_WIDTH) / 2, (oled.screen_height - BITMAP_HEIGHT) / 2,
  //             oled.splash_screen_data, oled.splash_screen_width, oled.splash_screen_height, 1);


  display_oled_drawBitmap(
    (oled.screen_width  - LOGO_WIDTH ) / 2,
    (oled.screen_height - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    if(time.flag_10ms_tick) {
      time.flag_10ms_tick = false;
      
      /* Grab Sensor Data */
      get_ad4681_samples( &a2d );    
      
      /* Log Sensor Data */
      log_samples();
      
      /* Evaluate Button Inputs */
      evaluate_button_inputs();


    }

    if(time.flag_100ms_tick) {
      time.flag_100ms_tick = false;
      
      /* Update Display */
      update_screen();
    }

    if(time.flag_500ms_tick) {
      time.flag_500ms_tick = false;
      HAL_GPIO_TogglePin(HLTH_LED_GPIO_Port, HLTH_LED_Pin);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 720;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_DUT_PWR_Pin|HLTH_LED_Pin|LED4_Pin|LED1_Pin
                          |LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SWO_Pin|SD_SPI2_CSn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_SPI1_CSn_GPIO_Port, ADC_SPI1_CSn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_DUT_PWR_Pin LED4_Pin LED1_Pin LED2_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = EN_DUT_PWR_Pin|LED4_Pin|LED1_Pin|LED2_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SWO_Pin SD_SPI2_CSn_Pin */
  GPIO_InitStruct.Pin = SWO_Pin|SD_SPI2_CSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1_n_Pin PB2_n_Pin PB3_n_Pin PB4_n_Pin */
  GPIO_InitStruct.Pin = PB1_n_Pin|PB2_n_Pin|PB3_n_Pin|PB4_n_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : HLTH_LED_Pin */
  GPIO_InitStruct.Pin = HLTH_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HLTH_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_SPI1_CSn_Pin */
  GPIO_InitStruct.Pin = ADC_SPI1_CSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_SPI1_CSn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : REV_2_Pin REV_1_Pin REV_0_Pin */
  GPIO_InitStruct.Pin = REV_2_Pin|REV_1_Pin|REV_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_ALRTn_Pin */
  GPIO_InitStruct.Pin = ADC_ALRTn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_ALRTn_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void blocking_delay_10ms_ticks (uint16_t ticks) {
  uint16_t i = 0;
  uint16_t tick = 0; //Used to lock time value
  for (i = ticks; i > 0; i--)
  {
      tick = time.ticks10ms;
      while (tick == time.ticks10ms); 
  }
}

void blocking_delay_500ms_ticks(uint16_t ticks)
{
    uint16_t i = 0;
    uint16_t tick = 0; //Used to lock time value
    for (i = ticks; i > 0; i--)
    {
      tick = time.ticks500ms;
      while (tick == time.ticks500ms); 
    }
}

  // TODO need to implement
void update_screen( void ) {

    char temp_string[32];        //Define the array that will hold the ASCII values
    char temp_number[8];        //Define the array that will hold the ASCII values

    /* USE SPRINT F TO BUILD THE ARRAY OF ASCII CHARACTERS */


  switch (oled.current_screen) {
    case MAIN_SCREEN:
      /* Clear Display */
      oled_clear();

      /* Set Larger Text Size for title */
      setTextSize(2,2);

      /* Write Title Line and Underscore */
      writeOledString("   MAIN  \n", SSD1306_WHITE);

      /* Smaller text size for underline */
      setTextSize(1,1);
      writeOledString("--------------------\n", SSD1306_WHITE);

      /* Print Current Voltage Value */
      memset(temp_string, '\0', 32);                  // Destination, Source, Size
      memset(temp_number, '\0', 8);                   
      memset(temp_string, "Voltage: ", 9);              

      sprintf((char *)temp_number, "%.4f", a2d_p -> voltage_f );   //f tells the function we want to print a float value

      strcat(temp_string, temp_number);         
      strcat(temp_string, '\n');         
      writeOledString(temp_string, SSD1306_WHITE);

       /* Print Current Current Value */
      memset(temp_string, '\0', 32);                 
      memset(temp_number, '\0', 8);                   
      memset(temp_string, "Current: ", 9);              

      sprintf((char *)temp_number, "%.4f", a2d_p -> current_f );   //f tells the function we want to print a float value

      strcat(temp_string, temp_number);         
      strcat(temp_string, '\n');         
      writeOledString(temp_string, SSD1306_WHITE);

      
       /* Print Current Power Value */
      memset(temp_string, '\0', 32);                  
      memset(temp_number, '\0', 8);                   
      memset(temp_string, "Power: ", 7);              

      sprintf((char *)temp_number, "%.4f", a2d_p -> power_f );   //f tells the function we want to print a float value

      strcat(temp_string, temp_number);         
      strcat(temp_string, '\n');         
      writeOledString(temp_string, SSD1306_WHITE);

       /* Indicate run status */
      memset(temp_string, '\0', 32);
      if(a2d_p -> logging_status) {
        writeOledString("Running: True\n", SSD1306_WHITE);
      }
      else {
        writeOledString("Running: False\n", SSD1306_WHITE);
      }

      /**
       * Call function that pushes
       * local data buffer into RAM
       * of display
       */
      updateDisplay();
      
    break;

    case SET_RUN_TIME_HR:
      /* Clear Display */
      oled_clear();

      /* Set Larger Text Size for title */
      setTextSize(2,2);

      /* Write Title Line and Underscore */
      writeOledString("  HOURS  \n", SSD1306_WHITE);

      /* Smaller text size for underline */
      setTextSize(1,1);
      writeOledString("--------------------\n", SSD1306_WHITE);

      /* Indicate run-time in hours*/
      memset(temp_string, '\0', 32);  
      memset(temp_number, '\0', 8);                   
      memset(temp_string, ">Run Time (hr): ", 16);

      sprintf((char *)temp_number, "%.4f", a2d_p -> run_time_hr );   //f tells the function we want to print a float value

      strcat(temp_string, temp_number);         
      strcat(temp_string, '\n');         

      writeOledString(temp_string, SSD1306_WHITE);
      
      /**
       * Call function that pushes
       * local data buffer into RAM
       * of display
       */
      updateDisplay();
      
    break;

    case SET_RUN_TIME_MIN:
      /* Clear Display */
      oled_clear();

      /* Set Larger Text Size for title */
      setTextSize(2,2);

      /* Write Title Line and Underscore */
      writeOledString("   MINS  \n", SSD1306_WHITE);

      /* Smaller text size for underline */
      setTextSize(1,1);
      writeOledString("--------------------\n", SSD1306_WHITE);

      /* Indicate run-time in hours*/
      memset(temp_string, '\0', 32);  
      memset(temp_number, '\0', 8);                   
      memset(temp_string, ">Run Time (min): ", 17);

      sprintf((char *)temp_number, "%.4f", a2d_p -> run_time_min );   //f tells the function we want to print a float value

      strcat(temp_string, temp_number);         
      strcat(temp_string, '\n');         

      writeOledString(temp_string, SSD1306_WHITE);
      
      /**
       * Call function that pushes
       * local data buffer into RAM
       * of display
       */
      updateDisplay();
    break;

    case SET_SENSE_RESISTOR:
      /* Clear Display */
      oled_clear();

      /* Set Larger Text Size for title */
      setTextSize(2,2);

      /* Write Title Line and Underscore */
      writeOledString(" RESISTOR\n", SSD1306_WHITE);

      /* Smaller text size for underline */
      setTextSize(1,1);
      writeOledString("--------------------\n", SSD1306_WHITE);

      /* Indicate run-time in hours*/
      memset(temp_string, '\0', 32);  
      memset(temp_number, '\0', 8);                   
      memset(temp_string, ">Sense Res: ", 12);

      sprintf((char *)temp_number, "%.4f", a2d_p -> cs_res_f );   //f tells the function we want to print a float value

      strcat(temp_string, temp_number);         
      strcat(temp_string, '\n');         

      writeOledString(temp_string, SSD1306_WHITE);
      
      /**
       * Call function that pushes
       * local data buffer into RAM
       * of display
       */
      updateDisplay();
    break;

    default:
      /* Error, so print main screen */
      /* Clear Display */
      oled_clear();

      /* Set Larger Text Size for title */
      setTextSize(2,2);

      /* Write Title Line and Underscore */
      writeOledString("   MAIN  \n", SSD1306_WHITE);

      /* Smaller text size for underline */
      setTextSize(1,1);
      writeOledString("--------------------\n", SSD1306_WHITE);

      /* Print Current Voltage Value */
      memset(temp_string, '\0', 32);                  // Destination, Source, Size
      memset(temp_number, '\0', 8);                   
      memset(temp_string, "Voltage: ", 9);              

      sprintf((char *)temp_number, "%.4f", a2d_p -> voltage_f );   //f tells the function we want to print a float value

      strcat(temp_string, temp_number);         
      strcat(temp_string, '\n');         
      writeOledString(temp_string, SSD1306_WHITE);

        /* Print Current Current Value */
      memset(temp_string, '\0', 32);                 
      memset(temp_number, '\0', 8);                   
      memset(temp_string, "Current: ", 9);              

      sprintf((char *)temp_number, "%.4f", a2d_p -> current_f );   //f tells the function we want to print a float value

      strcat(temp_string, temp_number);         
      strcat(temp_string, '\n');         
      writeOledString(temp_string, SSD1306_WHITE);

      
        /* Print Current Power Value */
      memset(temp_string, '\0', 32);                  
      memset(temp_number, '\0', 8);                   
      memset(temp_string, "Power: ", 7);              

      sprintf((char *)temp_number, "%.4f", a2d_p -> power_f );   //f tells the function we want to print a float value

      strcat(temp_string, temp_number);         
      strcat(temp_string, '\n');         
      writeOledString(temp_string, SSD1306_WHITE);

        /* Indicate run status */
      memset(temp_string, '\0', 32);
      if(a2d_p -> logging_status) {
        writeOledString("Running: True\n", SSD1306_WHITE);
      }
      else {
        writeOledString("Running: False\n", SSD1306_WHITE);
      }

      /**
       * Call function that pushes
       * local data buffer into RAM
       * of display
       */
      updateDisplay();
    
  }

}

  // TODO need to implement 
void log_samples( void ) {
}
  

void evaluate_button_inputs ( void ) {
  
  /* Verify if up button has been pressed */
  if(HAL_GPIO_ReadPin(GPIOE, UP_BUTTON) && !btn.up_btn_pressed) {
      if(btn.up_btn_press_ctr < BTN_DEBOUNCE_THRESHOLD) {
        btn.up_btn_press_ctr++;
      }
      if(btn.up_btn_press_ctr >= BTN_DEBOUNCE_THRESHOLD) {
        btn.up_btn_pressed = true;
      }

  }
  else {
    if(btn.up_btn_press_ctr > 0){
      if(btn.up_btn_press_ctr > 2){
        btn.up_btn_press_ctr -= 2;
      }
      else {
        btn.up_btn_press_ctr = 0;
      }
    }

  }

  /* Verify if rt button has been pressed */
  if(HAL_GPIO_ReadPin(GPIOE, RT_BUTTON) && !btn.rt_btn_pressed) {
      if(btn.rt_btn_press_ctr < BTN_DEBOUNCE_THRESHOLD) {
        btn.rt_btn_press_ctr++;
      }
      if(btn.rt_btn_press_ctr >= BTN_DEBOUNCE_THRESHOLD) {
        btn.rt_btn_pressed = true;
      }

  }
  else {
    if(btn.rt_btn_press_ctr > 0){
      if(btn.rt_btn_press_ctr > 2){
        btn.rt_btn_press_ctr -= 2;
      }
      else {
        btn.rt_btn_press_ctr = 0;
      }
    }

  }
  
  /* Verify if dn button has been pressed */
  if(HAL_GPIO_ReadPin(GPIOE, DN_BUTTON) && !btn.dn_btn_pressed) {
      if(btn.dn_btn_press_ctr < BTN_DEBOUNCE_THRESHOLD) {
        btn.dn_btn_press_ctr++;
      }
      if(btn.dn_btn_press_ctr >= BTN_DEBOUNCE_THRESHOLD) {
        btn.dn_btn_pressed = true;
      }

  }
  else {
    if(btn.dn_btn_press_ctr > 0){
      if(btn.dn_btn_press_ctr > 2){
        btn.dn_btn_press_ctr -= 2;
      }
      else {
        btn.dn_btn_press_ctr = 0;
      }
    }

  }
  
  /* Verify if lt button has been pressed */
  if(HAL_GPIO_ReadPin(GPIOE, LT_BUTTON) && !btn.lt_btn_pressed) {
      if(btn.lt_btn_press_ctr < BTN_DEBOUNCE_THRESHOLD) {
        btn.lt_btn_press_ctr++;
      }
      if(btn.lt_btn_press_ctr >= BTN_DEBOUNCE_THRESHOLD) {
        btn.lt_btn_pressed = true;
      }

  }
  else {
    if(btn.lt_btn_press_ctr > 0){
      if(btn.lt_btn_press_ctr > 2){
        btn.lt_btn_press_ctr -= 2;
      }
      else {
        btn.lt_btn_press_ctr = 0;
      }
    }

  }

}


//TODO need to insert the state machine here...

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
