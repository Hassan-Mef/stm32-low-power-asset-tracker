/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
//#include "tim.h"

#include "lsm6dsl.h"
#include "b_l475e_iot01a1_bus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// improved GPS Structure
typedef struct
{
    float latitude;
    float longitude;
    float speed;

    uint8_t validFix;

    uint8_t fixQuality;
    float hdop;

} GPS_Data_t;

// Alert Message
typedef struct
{
    uint32_t alertType;
} AlertMessage_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Earth Radius in Meters
#define EARTH_RADIUS_M 6371000.0f

// Event Group
#define MOTION_DETECTED_BIT   (1 << 0)
#define GPS_READY_BIT         (1 << 1)
#define STATIONARY_BIT        (1 << 2)
#define GEOFENCE_BREACH_BIT   (1 << 3)

// Geofence constants

//#define GEOFENCE_LAT      31.5204f
//#define GEOFENCE_LON      74.3587f
#define GEOFENCE_LAT  31.544230f
#define GEOFENCE_LON  74.285126f
#define GEOFENCE_RADIUS   7.0f     // it is 50.0f for testing reduced

// Accelorometer
#define LSM6DSL_ADDR        (0x6A << 1)
#define WHO_AM_I_REG        0x0F

// motion Threshold
#define MOTION_THRESHOLD 20


// cooldown

#define ALERT_COOLDOWN_MS 60000



// Log Helper function

#define LOG(fmt, ...) \
    printf("[%lu] " fmt, \
           osKernelGetTickCount(), \
           ##__VA_ARGS__)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

LPTIM_HandleTypeDef hlptim1;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotionTask */
osThreadId_t MotionTaskHandle;
const osThreadAttr_t MotionTask_attributes = {
  .name = "MotionTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for GPSTask */
osThreadId_t GPSTaskHandle;
const osThreadAttr_t GPSTask_attributes = {
  .name = "GPSTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BuzzerTask */
osThreadId_t BuzzerTaskHandle;
const osThreadAttr_t BuzzerTask_attributes = {
  .name = "BuzzerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for alertQueue */
osMessageQueueId_t alertQueueHandle;
const osMessageQueueAttr_t alertQueue_attributes = {
  .name = "alertQueue"
};
/* Definitions for gpsTimeoutTimer */
osTimerId_t gpsTimeoutTimerHandle;
const osTimerAttr_t gpsTimeoutTimer_attributes = {
  .name = "gpsTimeoutTimer"
};
/* Definitions for gpsFixTimer */
osTimerId_t gpsFixTimerHandle;
const osTimerAttr_t gpsFixTimer_attributes = {
  .name = "gpsFixTimer"
};
/* Definitions for motionSemaphore */
osSemaphoreId_t motionSemaphoreHandle;
const osSemaphoreAttr_t motionSemaphore_attributes = {
  .name = "motionSemaphore"
};
/* Definitions for systemEvents */
osEventFlagsId_t systemEventsHandle;
const osEventFlagsAttr_t systemEvents_attributes = {
  .name = "systemEvents"
};
/* USER CODE BEGIN PV */

// GPS Data object
GPS_Data_t currentGPS;


// testing functionality variables
//
//float inside_latitude = 31.5204 ;
//float inside_longitude= 74.3587 ;
//
//float outside_latitude = 31.5300 ;
//float outside_longitude= 74.4000 ;
//
//// toggling funcion
//
//static uint8_t toggleLocation = 0;


// Global Counter  upto 3

static uint8_t outsideCount = 0;

// Vraibles for LSM6DSL
LSM6DSL_Object_t MotionSensor;
volatile uint32_t dataRdyIntReceived;


// to store prev accel values
int32_t prevX = 0;
int32_t prevY = 0;
int32_t prevZ = 0;


// motion Debouncing

uint32_t lastMotionTick = 0;


// breach aleart timer
uint8_t alertActive = 0;

// gps output buffer

//char gpsLine[128];
//uint32_t idx = 0;
//uint8_t ch;

//
uint32_t lastAlertTime = 0;


volatile uint32_t wakeInterruptCount = 0;


// global flag for cooldown timer

volatile uint8_t alertCooldownActive = 0;


volatile uint8_t gpsFixTimeout = 0;

volatile uint8_t wakeFlag = 0;


// motion inital values detection fix
static uint8_t firstSample = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_UART4_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void StartMotionTask(void *argument);
void StartGPSTask(void *argument);
void StartBuzzerTask(void *argument);
void gpsTimerCallback01(void *argument);
void gpsFixTimerCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// write helper function for printf

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1,
                       (uint8_t *)ptr,
                       len,
                       HAL_MAX_DELAY);

    return len;
}

// Helper function s

float deg2rad(float deg)
{
    return deg * (3.14159265359f / 180.0f);
}


// for testing GPS Simulation function

void SimulateGPSFix(GPS_Data_t *gps)
{
    gps->latitude = 31.5300f;
    gps->longitude = 74.4000f;
    gps->validFix = 1;
}

// GeoFence Function

float calculateDistance(float def_lat, float def_lon , float new_lat, float new_lon)
{
	float dLat = deg2rad(new_lat - def_lat);
    float dLon = deg2rad(new_lon - def_lon);

    def_lat = deg2rad(def_lat);
    new_lat = deg2rad(new_lat);

    float a =
        sinf(dLat / 2.0f) * sinf(dLat / 2.0f) +
        cosf(def_lat) * cosf(new_lat) *
        sinf(dLon / 2.0f) * sinf(dLon / 2.0f);

    float c =
        2.0f * atan2f(
            sqrtf(a),
            sqrtf(1.0f - a));

    return EARTH_RADIUS_M * c;
}


//

float convertNMEACoordinate(float nmea)
{
    int degrees = (int)(nmea / 100);

    float minutes =
        nmea - (degrees * 100);

    return degrees + (minutes / 60.0f);
}


//
void parseGPRMC(char *sentence)
{
    char status;

    float lat;
    float lon;

    char ns;
    char ew;


//    sscanf(sentence,
//           "$GPRMC,%*[^,],%c,%f,%c,%f,%c",
//           &status,
//           &lat,
//           &ns,
//           &lon,
//           &ew);

    // new scanf

    float speedKnots;

    sscanf(sentence,
           "$GPRMC,%*[^,],%c,%f,%c,%f,%c,%f",
           &status,
           &lat,
           &ns,
           &lon,
           &ew,
           &speedKnots);

    currentGPS.speed =
        speedKnots * 1.852f;

    if(status == 'A')
    {
        currentGPS.validFix = 1;

        currentGPS.latitude =
            convertNMEACoordinate(lat);

        currentGPS.longitude =
            convertNMEACoordinate(lon);

        if(ns == 'S')
            currentGPS.latitude *= -1;

        if(ew == 'W')
            currentGPS.longitude *= -1;
    }
    else
    {
        currentGPS.validFix = 0;
    }
}

void parseGPGGA(char *sentence)
{
    float lat;
    float lon;

    char ns;
    char ew;

    int fixQuality;

//    printf("GGA PARSED\r\n");

    float hdop;

    char *start = strchr(sentence, ',');

    if(start != NULL)
    {
        sscanf(start,
               ",%*[^,],%f,%c,%f,%c,%d,%*d,%f",
               &lat,
               &ns,
               &lon,
               &ew,
               &fixQuality,
               &hdop);
    }

    currentGPS.fixQuality = fixQuality;
    currentGPS.hdop = hdop;
}


// MEMS Init

static void MEMS_Init(void)
{
    LSM6DSL_IO_t io_ctx;
    uint8_t id;
    LSM6DSL_AxesRaw_t axes;

    io_ctx.BusType = LSM6DSL_I2C_BUS;
    io_ctx.Address = LSM6DSL_I2C_ADD_L;
    io_ctx.Init = BSP_I2C2_Init;
    io_ctx.DeInit = BSP_I2C2_DeInit;
    io_ctx.ReadReg = BSP_I2C2_ReadReg;
    io_ctx.WriteReg = BSP_I2C2_WriteReg;
    io_ctx.GetTick = BSP_GetTick;

    LSM6DSL_RegisterBusIO(&MotionSensor, &io_ctx);

    LSM6DSL_ReadID(&MotionSensor, &id);

    LOG("LSM6DSL ID = 0x%02X\r\n", id);

    if(id != LSM6DSL_ID)
    {
        LOG("Sensor not found!\r\n");
        Error_Handler();
    }

    LSM6DSL_Init(&MotionSensor);

    LSM6DSL_ACC_SetOutputDataRate(&MotionSensor, 26.0f);
    LSM6DSL_ACC_SetFullScale(&MotionSensor, 4);

//    LSM6DSL_ACC_Set_INT1_DRDY(&MotionSensor, ENABLE);

    LSM6DSL_ACC_GetAxesRaw(&MotionSensor, &axes);

    LSM6DSL_ACC_Enable(&MotionSensor);

    LSM6DSL_ACC_Set_Wake_Up_Threshold(
        &MotionSensor,
        3);

    LSM6DSL_ACC_Set_Wake_Up_Duration(
        &MotionSensor,
        0);

//    LSM6DSL_ACC_Enable_Wake_Up_Detection(
//        &MotionSensor,
//        LSM6DSL_INT1_PIN);
}


// GPS POwer off mode code

void GPS_PowerOn(void)
{
    LOG("GPS POWER ON\r\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
}

void GPS_PowerOff(void)
{
    LOG("GPS POWER OFF\r\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
}


// GPS Power Function
// Manual Stop Mode 1 entry experiment
//void EnterStopMode(void)
//{
//	printf("Wake Count Before Sleep = %lu\r\n",
//	       wakeInterruptCount);
//    printf("Entering STOP1\r\n");
//
//    uint32_t before = HAL_GetTick();
//
//    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
//
//    HAL_PWREx_EnterSTOP1Mode(
//        PWR_STOPENTRY_WFI);
//
//    uint32_t after = HAL_GetTick();
//
//    SystemClock_Config();
//
//    printf("Woke From STOP1 after %lu ms\r\n",
//           after - before);
//
//    printf("Wake Count After Sleep = %lu\r\n",
//           wakeInterruptCount);
//}
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
  MX_DFSDM1_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_UART4_Init();
  MX_LPTIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of motionSemaphore */
  motionSemaphoreHandle = osSemaphoreNew(1, 0, &motionSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of gpsTimeoutTimer */
  gpsTimeoutTimerHandle = osTimerNew(gpsTimerCallback01, osTimerOnce, NULL, &gpsTimeoutTimer_attributes);

  /* creation of gpsFixTimer */
  gpsFixTimerHandle = osTimerNew(gpsFixTimerCallback, osTimerOnce, NULL, &gpsFixTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of alertQueue */
  alertQueueHandle = osMessageQueueNew (5, sizeof(uint32_t), &alertQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotionTask */
  MotionTaskHandle = osThreadNew(StartMotionTask, NULL, &MotionTask_attributes);

  /* creation of GPSTask */
  GPSTaskHandle = osThreadNew(StartGPSTask, NULL, &GPSTask_attributes);

  /* creation of BuzzerTask */
  BuzzerTaskHandle = osThreadNew(StartBuzzerTask, NULL, &BuzzerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of systemEvents */
  systemEventsHandle = osEventFlagsNew(&systemEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  MEMS_Init();
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
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
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
  HAL_GPIO_WritePin(GPIOC, BUZZER_Pin|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

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
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin ARD_A1_Pin
                           ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin|ARD_A1_Pin
                          |ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t flags;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


    if(GPIO_Pin == GPIO_PIN_11)
    {
//        printf("EXTI IRQ\r\n");
    	wakeInterruptCount++;

    	if(motionSemaphoreHandle != NULL)
    	        {
    		flags =osSemaphoreRelease(
    	                motionSemaphoreHandle);
    	        }


    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	    printf("Entering STOP1 in 5 sec...\r\n");
//
//	    osDelay(5000);
//
//	    printf("Entering STOP1 NOW\r\n");
//
//
//	    __WFI();
//
//	    printf("AFTER WFI\r\n");
//	    printf("WAKEUP FROM STOP1\r\n");
//	  if(wakeFlag)
//	  {
//		LSM6DSL_Event_Status_t status;
//
//		    LSM6DSL_ACC_Get_Event_Status(
//		        &MotionSensor,
//		        &status);
//
//		    printf(" Wake=%d Sleep=%d\n",
//		           status.WakeUpStatus,
//		           status.SleepStatus);
//		    wakeFlag =0;
//	  }

//	  printf("WakeCount=%lu\r\n",
//	            wakeInterruptCount);

	  // removed thi
//	    printf("Tick=%lu\r\n", osKernelGetTickCount());


//
	    osDelay(1000);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotionTask */
/**
* @brief Function implementing the MotionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotionTask */
void StartMotionTask(void *argument)
{
  /* USER CODE BEGIN StartMotionTask */
  /* Infinite loop */

	LOG("Motion Task Started\r\n");
	osDelay(1000);

	LSM6DSL_ACC_Enable_Wake_Up_Detection(
	    &MotionSensor,
	    LSM6DSL_INT1_PIN);
  for(;;)
  {

	  osStatus_t status =
	      osSemaphoreAcquire(
	          motionSemaphoreHandle,
	          osWaitForever);

//	  printf("Acquire=%d\r\n", status);

//	  printf("SEMAPHORE\r\n");

	  	        LSM6DSL_Axes_t acc_axes;

	  	        LSM6DSL_ACC_GetAxes(&MotionSensor, &acc_axes);


	  	      if(firstSample)
	  	      {
	  	          prevX = acc_axes.x;
	  	          prevY = acc_axes.y;
	  	          prevZ = acc_axes.z;

	  	          firstSample = 0;

	  	          continue;
	  	      }

//	  	      printf("X=%ld Y=%ld Z=%ld\r\n",
//	  	             acc_axes.x,
//	  	             acc_axes.y,
//	  	             acc_axes.z);

	  	        int32_t dx = abs(acc_axes.x - prevX);
	  	        int32_t dy = abs(acc_axes.y - prevY);
	  	        int32_t dz = abs(acc_axes.z - prevZ);

	  	        int32_t motion = dx + dy + dz;

//	  	      printf("Motion=%ld dx=%ld dy=%ld dz=%ld\r\n",
//	  	             motion,
//	  	             dx,
//	  	             dy,
//	  	             dz);

	  	        prevX = acc_axes.x;
	  	        prevY = acc_axes.y;
	  	        prevZ = acc_axes.z;

	  	        if(motion > MOTION_THRESHOLD)
	  	        {
	  	        	uint32_t now = osKernelGetTickCount();


//	  	          printf("Tick=%lu Motion=%ld\r\n",
//	  	                 now,
//	  	                 motion);

	  	            if((now - lastMotionTick) > 30)
	  	            {
	  	                lastMotionTick = now;

//	  	                printf("Motion DETECTED\r\n");
// -----DEBUGING ____
//	  	              printf("Cooldown Flag = %d\r\n",
//	  	                     alertCooldownActive);

	  	                osEventFlagsSet(
	  	                    systemEventsHandle,
	  	                    MOTION_DETECTED_BIT);
	  	            }
	  	        }

  }
  /* USER CODE END StartMotionTask */
}

/* USER CODE BEGIN Header_StartGPSTask */
/**
* @brief Function implementing the GPSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGPSTask */
void StartGPSTask(void *argument)
{
  /* USER CODE BEGIN StartGPSTask */
  /* Infinite loop */

	char line[256];
	uint32_t idx = 0;
	uint8_t ch;


	LOG("GPS Task Started\r\n");
	static uint8_t fixAcquired = 0;

	//osDelay(3000);
  for(;;)
  {

	  osEventFlagsWait(
		          systemEventsHandle,
		          MOTION_DETECTED_BIT,
		          osFlagsWaitAny,
		          osWaitForever);

		      LOG("GPS Activated\r\n");
		      GPS_PowerOn();
		      osDelay(5000);

		      gpsFixTimeout = 0;
		      fixAcquired = 0;

		      osTimerStart(
		          gpsFixTimerHandle,
		          120000);   // or 20000 for testing



		while(1)
		{
//	  printf("Loop\r\n");
      HAL_StatusTypeDef status =
              HAL_UART_Receive(&huart4,
                               &ch,
                               1,
                               1000);


      if(status == HAL_OK)
      {
          if(ch != '\r' &&
             ch != '\n' &&
             idx < sizeof(line) - 1)
          {
              line[idx++] = ch;
          }
          else if(idx > 0)
          {
              line[idx] = '\0';
              idx = 0;


              if(strstr(line, "$GPGGA") ||
                 strstr(line, "$GNGGA"))
              {
            	  // for debugging stage
//            	   printf("%s\r\n", line);
                  parseGPGGA(line);
              }

              if(strstr(line, "$GPRMC") ||
                 strstr(line, "$GNRMC"))
              {
                  parseGPRMC(line);


                  if(currentGPS.fixQuality != 1 && currentGPS.fixQuality != 2)
                  {
                      continue;
                  }

                  if(currentGPS.hdop <= 0.0f || currentGPS.hdop > 3.0f)
                  {
                      continue;
                  }


                  if(currentGPS.validFix &&
                     currentGPS.fixQuality > 0 &&
                     currentGPS.hdop < 3.0f)
                  {

                	    if(fixAcquired == 0)
                	    {
                	        osTimerStop(gpsFixTimerHandle);
                	        fixAcquired = 1;
                	    }

                      float distance =
                          calculateDistance(
                              GEOFENCE_LAT,
                              GEOFENCE_LON,
                              currentGPS.latitude,
                              currentGPS.longitude);

                      LOG("\r\nGPS Fix Acquired\r\n");

                      LOG("Lat: %.6f\r\n",
                             currentGPS.latitude);

                      LOG("Lon: %.6f\r\n",
                             currentGPS.longitude);

                      LOG("Distance = %.2f m\r\n",
                             distance);

                      LOG("Speed = %.2f km/h\r\n",
                             currentGPS.speed);

                      LOG("Fix Quality = %d\r\n",
                             currentGPS.fixQuality);

                      LOG("HDOP = %.2f\r\n",
                             currentGPS.hdop);
                      if(distance > GEOFENCE_RADIUS)
                      {
                    	  if(outsideCount < 3)
                    	      {
                    	          outsideCount++;
                    	      }

                          LOG("Outside Count = %d\r\n",
                                 outsideCount);
                      }
                      else
                      {
                          outsideCount = 0;
                      }


                      // -----DEBUGING ____
//                      uint32_t now = HAL_GetTick();
//                      LOG("Tick = %lu\r\n", now);

                      // -----DEBUGING ____
//                      LOG("Cooldown=%d\r\n",
//                             alertCooldownActive);

                      if(outsideCount >= 3 &&
                         alertCooldownActive == 0)
                      {
                    	  outsideCount = 0;
                    	  alertCooldownActive = 1;


                    	  // reduce to 20 sec ofr testing
                    	  osStatus_t timerStatus = osTimerStart(
                    	      gpsTimeoutTimerHandle,
                    	      60000);

                    	  // -----DEBUGING ____
//                    	  LOG("Timer Start Status = %d\r\n",
//                    	         timerStatus);

                    	  LOG("GEOFENCE BREACH!\r\n");

                    	  uint32_t alert = 1;

                    	  osMessageQueuePut(
                    	      alertQueueHandle,
                    	      &alert,
                    	      0,
                    	      0);


                    	  // before software timer (working



                      }
        }

                  }
              }

      }
      else if(status == HAL_TIMEOUT)
      {
//          printf(".");
      }
      else
      {
          LOG("[UART ERROR]\r\n");
      }

      // MOtion Timeout

      if((osKernelGetTickCount() - lastMotionTick) > 10000)
          {
              LOG("No Motion - GPS Sleep\r\n");

              GPS_PowerOff();
              outsideCount = 0;


              osEventFlagsClear(
                  systemEventsHandle,
                  MOTION_DETECTED_BIT);

              osTimerStop(gpsFixTimerHandle);
              fixAcquired = 0;


              break;
          }

      if(gpsFixTimeout)
      {
          LOG("GPS Fix Not Acquired - Sleep\r\n");

          GPS_PowerOff();
          gpsFixTimeout = 0;

          osEventFlagsClear(
              systemEventsHandle,
              MOTION_DETECTED_BIT);

          fixAcquired = 0;
          osTimerStop(gpsFixTimerHandle);

          break;
      }


    }

  }
  /* USER CODE END StartGPSTask */
}

/* USER CODE BEGIN Header_StartBuzzerTask */
/**
* @brief Function implementing the BuzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBuzzerTask */
void StartBuzzerTask(void *argument)
{
  /* USER CODE BEGIN StartBuzzerTask */
  /* Infinite loop */
    uint32_t alert;
    osStatus_t status;

    LOG("Buzzer Task Started\r\n");
  for(;;)
  {
//	  printf("Waiting for alert...\r\n");
//
////	  printf("Count Before = %lu\r\n",
//	         osMessageQueueGetCount(alertQueueHandle));

	        status = osMessageQueueGet(
	                    alertQueueHandle,
	                    &alert,
	                    NULL,
	                    osWaitForever);

//	        printf("GPS Queue = %p\r\n", alertQueueHandle);
//	        printf("Count After Put = %lu\r\n",
//	               osMessageQueueGetCount(alertQueueHandle));
//
//	        printf("Queue status = %d\r\n", status);

	        if(status == osOK)
	        {
	            LOG("BUZZER ALERT RECEIVED!\r\n");

	            HAL_GPIO_TogglePin(
	                    LED2_GPIO_Port,
	                    LED2_Pin);
	        }



	    for(int i = 0; i < 5; i++)
	    {
	        HAL_GPIO_WritePin(LED2_GPIO_Port,
	                          LED2_Pin,
	                          GPIO_PIN_SET);

//	        for(int j = 0; j < 1000; j++)
//	        {
//	            HAL_GPIO_TogglePin(
//	                BUZZER_GPIO_Port,
//	                BUZZER_Pin);
//
//	            HAL_Delay(1);
//	        }

//	        printf("BUZZER ALERT RECEIVED!\r\n");

	        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	        osDelay(500);

	        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

	        HAL_GPIO_WritePin(LED2_GPIO_Port,
	                          LED2_Pin,
	                          GPIO_PIN_RESET);

	        osDelay(300);
	    }

	    osDelay(500);
  }
  /* USER CODE END StartBuzzerTask */
}

/* gpsTimerCallback01 function */
void gpsTimerCallback01(void *argument)
{
  /* USER CODE BEGIN gpsTimerCallback01 */

    alertCooldownActive = 0;

//    HAL_GPIO_TogglePin(
//        LED2_GPIO_Port,
//        LED2_Pin);
    printf("TIMER EXPIRED\r\n");
  /* USER CODE END gpsTimerCallback01 */
}

/* gpsFixTimerCallback function */
void gpsFixTimerCallback(void *argument)
{
  /* USER CODE BEGIN gpsFixTimerCallback */
    gpsFixTimeout = 1;

    printf("GPS FIX TIMEOUT\r\n");
  /* USER CODE END gpsFixTimerCallback */
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
