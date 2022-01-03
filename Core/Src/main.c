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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_motion_sensors.h"
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "track_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INSTANCE_GYROSCOPE_ACCELEROMETER 0
#define INSTANCE_MAGNETOMETER 1
#define OS_DELAY_STANDARD 250
#define MUTEX_WAIT_TIMEOUT osWaitForever
#define WAITING_FOR_GREEN_LIGHT 0
#define RACING 1
#define PIT_STOP 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId greenLightTaskHandle;
osThreadId trackDataPrintTaskHandle;
osThreadId userButtonTaskHandle;
osThreadId proximitySensorTaskHandle;
osThreadId raceDataPrintTaskHandle;
osThreadId accelerometerTaskHandle;
osThreadId temperatureSensorTaskHandle;
osThreadId humiditySensorTaskHandle;
osThreadId pressureSensorTaskHandle;
osThreadId buttonInterruptTaskHandle;
osMutexId managerMutexHandle;
osSemaphoreId userButtonInterruptSemaphoreHandle;
/* USER CODE BEGIN PV */

struct manager_t {

	float temperature_value;  // Shared measured temperature value
	uint8_t humidity_value; // Shared measured humidity value
	int pressure_value;  // Shared measured pressure value
	BSP_MOTION_SENSOR_Axes_t  accelerometer_value; //Shared accelerometer value
	uint16_t proximity; //Shared proximity value
	uint8_t status;
	uint8_t b_green_light, b_track_data, b_user_button, b_temperature, b_humidity, b_pressure, b_proximity, b_race_data, b_accelerometer;
	uint8_t pit_stop_executions;
	uint8_t waiting_for_race_director_executions;
	uint8_t race_executions;
	uint8_t opponent_executions;
	int last_opponent_value;

} manager;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
void startGreenLightTask(void const * argument);
void startTrackDataPrintTask(void const * argument);
void startUserButtonTask(void const * argument);
void startProximitySensorTask(void const * argument);
void startRaceDataPrintTask(void const * argument);
void startAccelerometerTask(void const * argument);
void startTemperatureSensorTask(void const * argument);
void startHumiditySensorTask(void const * argument);
void startPressureSensorTask(void const * argument);
void startButtonInterruptTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,10);
	return len;
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
	MX_I2C2_Init();
	MX_SPI3_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	//Proximity

	VL53L0X_PROXIMITY_Init();

	//User button
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

	//Temperature
	BSP_TSENSOR_Init();

	//	//Humidity
	BSP_HSENSOR_Init();

	//	//Pressure
	BSP_PSENSOR_Init();

	//Motion sensors

	//BSP_MOTION_SENSOR_Init(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_GYRO);
	BSP_MOTION_SENSOR_Init(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_ACCELERO);

	//BSP_MOTION_SENSOR_Enable(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_GYRO);
	BSP_MOTION_SENSOR_Enable(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_ACCELERO);

	//Struct elements initialization

	manager.humidity_value = 0;
	manager.pressure_value = 0;
	manager.temperature_value = 0;
	manager.proximity = 0;
	manager.b_accelerometer, manager.b_temperature, manager.b_humidity, manager.b_pressure, manager.b_green_light, manager.b_proximity,
	manager.b_proximity, manager.b_race_data, manager.b_track_data, manager.b_user_button = 0;
	manager.status = WAITING_FOR_GREEN_LIGHT;
	manager.pit_stop_executions = 0;
	manager.waiting_for_race_director_executions = 0;
	manager.race_executions = 0;
	manager.last_opponent_value = -1;
	manager.opponent_executions = 0;

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of managerMutex */
	osMutexDef(managerMutex);
	managerMutexHandle = osMutexCreate(osMutex(managerMutex));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of userButtonInterruptSemaphore */
	osSemaphoreDef(userButtonInterruptSemaphore);
	userButtonInterruptSemaphoreHandle = osSemaphoreCreate(osSemaphore(userButtonInterruptSemaphore), 1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */

	osSemaphoreWait(userButtonInterruptSemaphoreHandle, MUTEX_WAIT_TIMEOUT);
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of greenLightTask */
	osThreadDef(greenLightTask, startGreenLightTask, osPriorityNormal, 0, 160);
	greenLightTaskHandle = osThreadCreate(osThread(greenLightTask), NULL);

	/* definition and creation of trackDataPrintTask */
	osThreadDef(trackDataPrintTask, startTrackDataPrintTask, osPriorityNormal, 0, 192);
	trackDataPrintTaskHandle = osThreadCreate(osThread(trackDataPrintTask), NULL);

	/* definition and creation of userButtonTask */
	osThreadDef(userButtonTask, startUserButtonTask, osPriorityNormal, 0, 128);
	userButtonTaskHandle = osThreadCreate(osThread(userButtonTask), NULL);

	/* definition and creation of proximitySensorTask */
	osThreadDef(proximitySensorTask, startProximitySensorTask, osPriorityNormal, 0, 176);
	proximitySensorTaskHandle = osThreadCreate(osThread(proximitySensorTask), NULL);

	/* definition and creation of raceDataPrintTask */
	osThreadDef(raceDataPrintTask, startRaceDataPrintTask, osPriorityNormal, 0, 160);
	raceDataPrintTaskHandle = osThreadCreate(osThread(raceDataPrintTask), NULL);

	/* definition and creation of accelerometerTask */
	osThreadDef(accelerometerTask, startAccelerometerTask, osPriorityNormal, 0, 160);
	accelerometerTaskHandle = osThreadCreate(osThread(accelerometerTask), NULL);

	/* definition and creation of temperatureSensorTask */
	osThreadDef(temperatureSensorTask, startTemperatureSensorTask, osPriorityBelowNormal, 0, 176);
	temperatureSensorTaskHandle = osThreadCreate(osThread(temperatureSensorTask), NULL);

	/* definition and creation of humiditySensorTask */
	osThreadDef(humiditySensorTask, startHumiditySensorTask, osPriorityBelowNormal, 0, 176);
	humiditySensorTaskHandle = osThreadCreate(osThread(humiditySensorTask), NULL);

	/* definition and creation of pressureSensorTask */
	osThreadDef(pressureSensorTask, startPressureSensorTask, osPriorityBelowNormal, 0, 176);
	pressureSensorTaskHandle = osThreadCreate(osThread(pressureSensorTask), NULL);

	/* definition and creation of buttonInterruptTask */
	osThreadDef(buttonInterruptTask, startButtonInterruptTask, osPriorityNormal, 0, 128);
	buttonInterruptTaskHandle = osThreadCreate(osThread(buttonInterruptTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

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
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

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
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
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
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
			|SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

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

	/*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_EXTI13_Pin */
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
	GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
			|ARD_A1_Pin|ARD_A0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
	GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
	GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D4_Pin */
	GPIO_InitStruct.Pin = ARD_D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D7_Pin */
	GPIO_InitStruct.Pin = ARD_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
	GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D3_Pin */
	GPIO_InitStruct.Pin = ARD_D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D6_Pin */
	GPIO_InitStruct.Pin = ARD_D6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
	GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
			|SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

	/*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
	GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
			|PMOD_IRQ_EXTI12_Pin;
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
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == USER_BUTTON_PIN) {

		osSemaphoreRelease(userButtonInterruptSemaphoreHandle);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startGreenLightTask */
/**
 * @brief  Function implementing the greenLightTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_startGreenLightTask */
void startGreenLightTask(void const * argument)
{
	/* USER CODE BEGIN 5 */

	//	UBaseType_t uxHighWaterMark;
	//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	TickType_t last_tick_time = 0, end_time = 0,diff = 0;
	last_tick_time = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		osMutexWait(managerMutexHandle, MUTEX_WAIT_TIMEOUT);

		if (manager.status != WAITING_FOR_GREEN_LIGHT) {
			BSP_LED_Toggle(LED2);
		}

		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

		//printf("green light task watermark %lu \r\n", uxHighWaterMark);

		osMutexRelease(managerMutexHandle);

		osDelayUntil(&last_tick_time, OS_DELAY_STANDARD / portTICK_PERIOD_MS);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startTrackDataPrintTask */
/**
 * @brief Function implementing the trackDataPrintTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startTrackDataPrintTask */
void startTrackDataPrintTask(void const * argument)
{
	/* USER CODE BEGIN startTrackDataPrintTask */

	//	UBaseType_t uxHighWaterMark;
	//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	TickType_t last_tick_time = 0, end_time = 0,diff = 0;
	last_tick_time = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		osMutexWait(managerMutexHandle, MUTEX_WAIT_TIMEOUT);

		if (manager.status == WAITING_FOR_GREEN_LIGHT
				&& manager.temperature_value > 0
				&& manager.waiting_for_race_director_executions % 40 == 0) {

			//Pressure

			int normalized = manager.pressure_value;

			printf("Track pressure update: %d mBar \r\n", normalized);

			//Temperature

			float temp_value = manager.temperature_value;
			int tmpInt1 = temp_value;
			float tmpFraction = temp_value - tmpInt1;
			int tmpInt2 = trunc(tmpFraction * 100);

			printf("Track temperature update: %d.%02d C\r\n", tmpInt1, tmpInt2);

			//Humidity

			uint8_t hmd = manager.humidity_value;

			printf("Track humidity update: %d %%\r\n\n", hmd);
		}

		//		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		//
		//		printf("track data task watermark %lu \r\n", uxHighWaterMark);

		osMutexRelease(managerMutexHandle);

		osDelayUntil(&last_tick_time, OS_DELAY_STANDARD / portTICK_PERIOD_MS);

	}
	/* USER CODE END startTrackDataPrintTask */
}

/* USER CODE BEGIN Header_startUserButtonTask */
/**
 * @brief Function implementing the userButtonTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startUserButtonTask */
void startUserButtonTask(void const * argument)
{
	/* USER CODE BEGIN startUserButtonTask */

	//	UBaseType_t uxHighWaterMark;
	//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	TickType_t last_tick_time = 0, end_time = 0,diff = 0;
	last_tick_time = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		osMutexWait(managerMutexHandle, MUTEX_WAIT_TIMEOUT);

		if (manager.status == WAITING_FOR_GREEN_LIGHT && manager.waiting_for_race_director_executions % 20 == 0) {

			//Callback is handled in the HAL_GPIO_EXTI_Callback method

			printf("Press the USER button to start the Grand Prix...\r\n\n");

			if (manager.waiting_for_race_director_executions == 200) {

				manager.waiting_for_race_director_executions = 0;
			}
		}

		manager.waiting_for_race_director_executions++;

		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

		//printf("user button task watermark %lu \r\n", uxHighWaterMark);

		osMutexRelease(managerMutexHandle);

		osDelayUntil(&last_tick_time, OS_DELAY_STANDARD / portTICK_PERIOD_MS);
	}
	/* USER CODE END startUserButtonTask */
}

/* USER CODE BEGIN Header_startProximitySensorTask */
/**
 * @brief Function implementing the proximitySensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startProximitySensorTask */
void startProximitySensorTask(void const * argument)
{
	/* USER CODE BEGIN startProximitySensorTask */

	//	UBaseType_t uxHighWaterMark;
	//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	TickType_t last_tick_time = 0, end_time = 0,diff = 0;
	last_tick_time = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		osMutexWait(managerMutexHandle, MUTEX_WAIT_TIMEOUT);

		if (manager.status != WAITING_FOR_GREEN_LIGHT) {

			uint8_t proximity_value = 0;

			proximity_value = VL53L0X_PROXIMITY_GetDistance();

			manager.proximity = proximity_value;
		}

		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

		//printf("proximity sensor task watermark %lu \r\n", uxHighWaterMark);

		osMutexRelease(managerMutexHandle);

		osDelayUntil(&last_tick_time, OS_DELAY_STANDARD / portTICK_PERIOD_MS);
	}
	/* USER CODE END startProximitySensorTask */
}

/* USER CODE BEGIN Header_startRaceDataPrintTask */
/**
 * @brief Function implementing the raceDataPrintTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startRaceDataPrintTask */
void startRaceDataPrintTask(void const * argument)
{
	/* USER CODE BEGIN startRaceDataPrintTask */

	//	UBaseType_t uxHighWaterMark;
	//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	TickType_t last_tick_time = 0, end_time = 0,diff = 0;
	last_tick_time = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		osMutexWait(managerMutexHandle, MUTEX_WAIT_TIMEOUT);

		if (manager.status == RACING) {

			printf("\033[2J"); //Clears the terminal

			if (manager.race_executions % 10 == 0 || manager.last_opponent_value != -1) {

				if (manager.opponent_executions < 7) {

					manager.last_opponent_value = manager.last_opponent_value != -1 ? manager.last_opponent_value : rand() % 16;

					manager.opponent_executions++;

					for (uint8_t i = 0; i < 7; i++) {

						if (i == manager.opponent_executions) {

							printf(computeTrackOpponent(manager.last_opponent_value));

						} else {

							printf(getClearTrackString());
						}
					}

				} else {

					manager.last_opponent_value = -1;
					manager.opponent_executions = 0;

					for (uint8_t i = 0; i < 7; i++) {

						printf(getClearTrackString());
					}
				}

			} else {

				for (uint8_t i = 0; i < 7; i++) {

					printf(getClearTrackString());
				}

				manager.last_opponent_value = -1;
			}

			printf(computeCurrentCarPosition(manager.accelerometer_value.x));

			manager.race_executions++;

			if (manager.race_executions == 150) {

				printf("Chequered flag, good job!\r\n\n");

				manager.race_executions = 0;

				manager.status = WAITING_FOR_GREEN_LIGHT;
			}

		} else if (manager.status == PIT_STOP) {

			if (manager.proximity > 100 && manager.proximity < 200) {

				printf("Hold steady...!\r\n\n");

				printf(performPitStop());

				manager.pit_stop_executions++;

			} else {

				printf("Keep your hands on the wheel!\r\n\n");
			}

			if (manager.pit_stop_executions == 20) {

				manager.pit_stop_executions = 0 ;

				manager.status = RACING;

				printf("GO GO GO!\r\n\n");
			}
		}

		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

		//printf("race data task watermark %lu \r\n", uxHighWaterMark);

		osMutexRelease(managerMutexHandle);

		osDelayUntil(&last_tick_time, OS_DELAY_STANDARD / portTICK_PERIOD_MS);
	}
	/* USER CODE END startRaceDataPrintTask */
}

/* USER CODE BEGIN Header_startAccelerometerTask */
/**
 * @brief Function implementing the accelerometerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startAccelerometerTask */
void startAccelerometerTask(void const * argument)
{
	/* USER CODE BEGIN startAccelerometerTask */

	//	UBaseType_t uxHighWaterMark;
	//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	TickType_t last_tick_time = 0, end_time = 0,diff = 0;
	last_tick_time = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		TickType_t initial_time = 0, end_time = 0,diff = 0;
		initial_time = xTaskGetTickCount();

		osMutexWait(managerMutexHandle, MUTEX_WAIT_TIMEOUT);

		if (manager.status == RACING) {

			BSP_MOTION_SENSOR_Axes_t  acc_value = {0, 0, 0};

			BSP_MOTION_SENSOR_GetAxes(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_ACCELERO, &acc_value);

			manager.accelerometer_value = acc_value;
		}

		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

		//printf("accelerometer task watermark %lu \r\n", uxHighWaterMark);

		osMutexRelease(managerMutexHandle);

		osDelayUntil(&last_tick_time, OS_DELAY_STANDARD / portTICK_PERIOD_MS);
	}
	/* USER CODE END startAccelerometerTask */
}

/* USER CODE BEGIN Header_startTemperatureSensorTask */
/**
 * @brief Function implementing the temperatureSensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startTemperatureSensorTask */
void startTemperatureSensorTask(void const * argument)
{
	/* USER CODE BEGIN startTemperatureSensorTask */

	//	UBaseType_t uxHighWaterMark;
	//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	TickType_t last_tick_time = 0, end_time = 0,diff = 0;
	last_tick_time = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		osMutexWait(managerMutexHandle, MUTEX_WAIT_TIMEOUT);

		manager.temperature_value = BSP_TSENSOR_ReadTemp();

		//		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		//
		//		printf("temperature task watermark %lu \r\n", uxHighWaterMark);

		osMutexRelease(managerMutexHandle);

		osDelayUntil(&last_tick_time, OS_DELAY_STANDARD / portTICK_PERIOD_MS);
	}
	/* USER CODE END startTemperatureSensorTask */
}

/* USER CODE BEGIN Header_startHumiditySensorTask */
/**
 * @brief Function implementing the humiditySensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startHumiditySensorTask */
void startHumiditySensorTask(void const * argument)
{
	/* USER CODE BEGIN startHumiditySensorTask */

	//	UBaseType_t uxHighWaterMark;
	//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	TickType_t last_tick_time = 0, end_time = 0,diff = 0;
	last_tick_time = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		osMutexWait(managerMutexHandle, MUTEX_WAIT_TIMEOUT);

		manager.humidity_value = BSP_HSENSOR_ReadHumidity();

		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

		//printf("humidity task watermark %lu \r\n", uxHighWaterMark);

		osMutexRelease(managerMutexHandle);

		osDelayUntil(&last_tick_time, OS_DELAY_STANDARD / portTICK_PERIOD_MS);
	}
	/* USER CODE END startHumiditySensorTask */
}

/* USER CODE BEGIN Header_startPressureSensorTask */
/**
 * @brief Function implementing the pressureSensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startPressureSensorTask */
void startPressureSensorTask(void const * argument)
{
	/* USER CODE BEGIN startPressureSensorTask */

	//	UBaseType_t uxHighWaterMark;
	//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	TickType_t last_tick_time = 0, end_time = 0,diff = 0;
	last_tick_time = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		osMutexWait(managerMutexHandle, MUTEX_WAIT_TIMEOUT);

		manager.pressure_value = BSP_PSENSOR_ReadPressure();

		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

		//printf("pressure task watermark %lu \r\n", uxHighWaterMark);

		osMutexRelease(managerMutexHandle);

		osDelayUntil(&last_tick_time, OS_DELAY_STANDARD / portTICK_PERIOD_MS);
	}
	/* USER CODE END startPressureSensorTask */
}

/* USER CODE BEGIN Header_startButtonInterruptTask */
/**
 * @brief Function implementing the buttonInterruptTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startButtonInterruptTask */
void startButtonInterruptTask(void const * argument)
{
	/* USER CODE BEGIN startButtonInterruptTask */

	//	UBaseType_t uxHighWaterMark;
	//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

	TickType_t last_tick_time = 0, end_time = 0,diff = 0;
	last_tick_time = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		osSemaphoreWait(userButtonInterruptSemaphoreHandle, MUTEX_WAIT_TIMEOUT);

		osMutexWait(managerMutexHandle, MUTEX_WAIT_TIMEOUT);

		if (manager.status == WAITING_FOR_GREEN_LIGHT) {

			manager.status = RACING;

			printf("Green light!\r\n\n");

		} else if (manager.status == RACING) {

			manager.status = PIT_STOP;

			printf("------- PIT STOP -------\r\n\n");

		} else {

			manager.status = WAITING_FOR_GREEN_LIGHT;

			manager.waiting_for_race_director_executions = 0;

			BSP_LED_Off(LED2);

			printf("We need to retire the car! Sorry.\r\n\n");
		}

		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

		//printf("button interrupt task watermark %lu \r\n", uxHighWaterMark);

		osMutexRelease(managerMutexHandle);

		osDelayUntil(&last_tick_time, OS_DELAY_STANDARD / portTICK_PERIOD_MS);
	}
	/* USER CODE END startButtonInterruptTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
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
