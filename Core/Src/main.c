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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for LightControl */
osThreadId_t LightControlHandle;
const osThreadAttr_t LightControl_attributes = {
  .name = "LightControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CarData */
osThreadId_t CarDataHandle;
const osThreadAttr_t CarData_attributes = {
  .name = "CarData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for DataCAN */
osThreadId_t DataCANHandle;
const osThreadAttr_t DataCAN_attributes = {
  .name = "DataCAN",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorControl */
osThreadId_t MotorControlHandle;
const osThreadAttr_t MotorControl_attributes = {
  .name = "MotorControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ReadingSensor */
osThreadId_t ReadingSensorHandle;
const osThreadAttr_t ReadingSensor_attributes = {
  .name = "ReadingSensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartlLightControl(void *argument);
void StartCarData(void *argument);
void SendData(void *argument);
void StartMotorInput(void *argument);
void StartSensor(void *argument);

/* USER CODE BEGIN PFP */
//HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
//HAL_I2C_Mem_Read (I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t * pData, uint16_t Size, uint32_t Timeout)
bool IMU_INIT(I2C_HandleTypeDef* handle){
	/*
	HAL_I2C_Mem_Read (I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t * pData, uint16_t Size, uint32_t Timeout)
	if()
	*/
}

int16_t GYRO_READ_DIR(char dir){
	uint8_t low[1];
	uint8_t high[1];
	if(dir == 'x'){
		//two mem reads
		//return((int16_t)high << 8) | low;
	}
}
//int16 READ_READ_DIR(char dir){

//}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///global task values

bool DataReady = 0;




bool Steering_Wheel_Is_Active = true;
//Brake
static uint16_t Brake_Pot_Avg;
//Acc
static uint16_t Throttle_Pot_Avg;
//ADC Channel Selector
  void Select_ADC_Channel(int channel){


	  ADC_ChannelConfTypeDef


	  switch(channel){

	  case 1:
		  sConfig.Channel = ADC_CHANNEL_0;
		    sConfig.Rank = ADC_REGULAR_RANK_1;
		    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
		    sConfig.SingleDiff = ADC_SINGLE_ENDED;
		    sConfig.OffsetNumber = ADC_OFFSET_NONE;
		    sConfig.Offset = 0;
		    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		    {
		      Error_Handler();
		    }
		    break;
	  case 2:
		  sConfig.Channel = ADC_CHANNEL_1;
		    sConfig.Rank = ADC_REGULAR_RANK_1;
		    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
		    sConfig.SingleDiff = ADC_SINGLE_ENDED;
		    sConfig.OffsetNumber = ADC_OFFSET_NONE;
		    sConfig.Offset = 0;
		    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		    {
		      Error_Handler();
		    }
		    break;
	  case 3:
		  sConfig.Channel = ADC_CHANNEL_2;
		    sConfig.Rank = ADC_REGULAR_RANK_1;
		    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
		    sConfig.SingleDiff = ADC_SINGLE_ENDED;
		    sConfig.OffsetNumber = ADC_OFFSET_NONE;
		    sConfig.Offset = 0;
		    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		    {
		      Error_Handler();
		    }
		    break;
	  case 4:
		  sConfig.Channel = ADC_CHANNEL_3;
		    sConfig.Rank = ADC_REGULAR_RANK_1;
		    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
		    sConfig.SingleDiff = ADC_SINGLE_ENDED;
		    sConfig.OffsetNumber = ADC_OFFSET_NONE;
		    sConfig.Offset = 0;
		    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		    {
		      Error_Handler();
		    }
		    break;
	  case 5:
		  sConfig.Channel = ADC_CHANNEL_4;
		    sConfig.Rank = ADC_REGULAR_RANK_1;
		    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
		    sConfig.SingleDiff = ADC_SINGLE_ENDED;
		    sConfig.OffsetNumber = ADC_OFFSET_NONE;
		    sConfig.Offset = 0;
		    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		    {
		      Error_Handler();
		    }
		    break;
	  case 6:
		  sConfig.Channel = ADC_CHANNEL_5;
		    sConfig.Rank = ADC_REGULAR_RANK_1;
		    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
		    sConfig.SingleDiff = ADC_SINGLE_ENDED;
		    sConfig.OffsetNumber = ADC_OFFSET_NONE;
		    sConfig.Offset = 0;
		    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		    {
		      Error_Handler();
		    }
		    break;
	  }

  }


  uint16_t Rough_Potentiometer_Average(uint16_t signals[]){
	  uint16_t functioning_inputs = sizeof(signals)/sizeof(signals[0]);
	  uint16_t sum = 0;
	  uint16_t i = functioning_inputs;

	  while(i>0){
		  if((signals[i]==0 || signals[i] == 4095)&& length>0){
			  functioning_inputs--;
		  }

		  if(functioning_inputs == 0){
			  return 0;
		  }
		  else{
			sum+=signals[i];
		  }

		  i--;

	  }


	  //return average of working signals
	  return sum/functioning_inputs;

  }

  //Will store respective potentiometer values for adc
  uint16_t Throttle_ADC[3];
  uint16_t Brake_ADC[3];
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LightControl */
  LightControlHandle = osThreadNew(StartlLightControl, NULL, &LightControl_attributes);

  /* creation of CarData */
  CarDataHandle = osThreadNew(StartCarData, NULL, &CarData_attributes);

  /* creation of DataCAN */
  DataCANHandle = osThreadNew(SendData, NULL, &DataCAN_attributes);

  /* creation of MotorControl */
  MotorControlHandle = osThreadNew(StartMotorInput, NULL, &MotorControl_attributes);

  /* creation of ReadingSensor */
  ReadingSensorHandle = osThreadNew(StartSensor, NULL, &ReadingSensor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 1;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  /*sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }*/
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hi2c2.Init.Timing = 0x00000E14;
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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Error_LED_Pin|STM_OK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS1_Pin|CS0_Pin|ARRAY_LED_Pin|BPS_ENC8_Pin
                          |MC_ENC9_Pin|E2_Pin|E1_Pin|E0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HeadLights_Pin|RightTurn_Pin|LeftTurn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Error_LED_Pin STM_OK_Pin */
  GPIO_InitStruct.Pin = Error_LED_Pin|STM_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin CS0_Pin ARRAY_LED_Pin BPS_ENC8_Pin
                           MC_ENC9_Pin E2_Pin E1_Pin E0_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS0_Pin|ARRAY_LED_Pin|BPS_ENC8_Pin
                          |MC_ENC9_Pin|E2_Pin|E1_Pin|E0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : HeadLights_Pin RightTurn_Pin LeftTurn_Pin */
  GPIO_InitStruct.Pin = HeadLights_Pin|RightTurn_Pin|LeftTurn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ARRAY_EN_Pin BPS_EN_Pin */
  GPIO_InitStruct.Pin = ARRAY_EN_Pin|BPS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MC_EN_Pin */
  GPIO_InitStruct.Pin = MC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MC_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXI4_CALLBACK (uint16_t GPIO_PIN){
if( GPIO_PIN == GPIO_PIN_0){
	DataReady=1;
}
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartlLightControl */
/**
  * @brief  Function implementing the LightControl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartlLightControl */
void StartlLightControl(void *argument)
{



  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	//if (DataReady == 1)
		//StartLightControl;

  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCarData */
/**
* @brief Function implementing the CarData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCarData */
void StartCarData(void *argument)
{
  /* USER CODE BEGIN StartCarData */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCarData */
}

/* USER CODE BEGIN Header_SendData */
/**
* @brief Function implementing the DataCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendData */
void SendData(void *argument)
{
  /* USER CODE BEGIN SendData */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SendData */
}

/* USER CODE BEGIN Header_StartMotorInput */








/**
* @brief Function implementing the MotorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorInput */
void StartMotorInput(void *argument)
{
  /* USER CODE BEGIN StartMotorInput */
	uint16_t Upper_Bound = 4095;
	uint16_t Lower_Bound = 0;
	if(Steering_Wheel_Is_Active){

		Select_ADC_Channel(1);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		Throttle_ADC[0] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);


		Select_ADC_Channel(2);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		Throttle_ADC[1] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);


		Select_ADC_Channel(3);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		Throttle_ADC[2] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);


		Select_ADC_Channel(4);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		Brake_ADC[0] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);


		Select_ADC_Channel(5);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		Brake_ADC[1] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);


		Select_ADC_Channel(6);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		Brake_ADC[2] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		Brake_Pot_Avg = Rough_Potentiometer_Average(Brake_ADC);
		Throttle_Pot_Avg = Rough_Potentiometer_Average(Throttle_Pot_Avg);



	}
	else{}
}

  //HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);

	//(&hadc);
	  //HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
	  //raw = HAL_ADC_GetValue(&hadc);

	//(&hadc);
	  //HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
	  //raw = HAL_ADC_GetValue(&hadc);
  /* Infinite loop */


  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorInput */
}

/* USER CODE BEGIN Header_StartSensor */
/**
* @brief Function implementing the ReadingSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensor */
void StartSensor(void *argument)
{
  /* USER CODE BEGIN StartSensor */
  /* Infinite loop */

  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSensor */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
