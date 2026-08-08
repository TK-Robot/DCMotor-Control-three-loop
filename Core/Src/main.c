/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "TypeDefine.h"
#include "MT6701.h"
#include "AD116.h"
#include "VoltageStatus.h"
#include "PID.h"
#include "PWMCapture/PWMCapture.h"
#include "NvmParam.h"
#include "ServoControl.h"
#include "Dynamixel2.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
Param Param_KX;
MT6701 Encoder;
AD116 Drive;
VoltageStatus Voltage;
CaptureData PWMCaptureData;
ServoControl Servo;
Dynamixel2Context DynamixelBus;
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
#define TK_UART_LINK_TEST 0

#if TK_UART_LINK_TEST
static void App_UartLinkTest_1ms(void)
{
  static uint16_t tick_ms = 0;
  static const uint8_t test_frame[] = {'T', 'K', '1', '1', '5', '2', '0', '0', '\r', '\n'};

  if (++tick_ms >= 1000U)
  {
    tick_ms = 0;
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)test_frame, sizeof(test_frame), 10U);
  }
}
#endif

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  Param_KX.DutyRatio = PWMCapture_Calculate(&PWMCaptureData, htim);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  Dynamixel2_RxEventCallback(&DynamixelBus, huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    Dynamixel2_TxCpltCallback(&DynamixelBus, huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    uint32_t err = HAL_UART_GetError(huart);

    if (err != HAL_UART_ERROR_NONE)
    {
      /* Reset both directions so one DMA error cannot leave RX permanently stopped. */
      /* 同时复位收发方向，避免一次 DMA 错误让 RX 永久停止。 */
      (void)HAL_UART_AbortReceive(huart);
      HAL_UART_AbortTransmit(huart);
      Dynamixel2_RecordUartError(&DynamixelBus, err);
      Dynamixel2_TxCpltCallback(&DynamixelBus, huart);
      (void)Dynamixel2_RestartRx(&DynamixelBus);
    }
  }
}

static void App_DefaultParam(Param *param)
{
  param->CycleTimeMs = 1;
  param->TempLimit = 40;
  param->ExpectMA = 0;
  param->CurrentLogical_mA = 0;
  param->EncoderExpect = 0;
  param->EncoderOffset = 3400;
  param->EncoderVeer = false;
  param->EncoderRebaseline = false;
  param->SpeedMax = 45000;
  param->AccelMax = 60000;
  param->DecelMax = 60000;
  param->EncoderSpeedExpect = 0;
  param->PowerSaveVoltage_mV = 4000;
  param->DriveRunMode = 0;
  param->DrivePwmMode = 2U;
  param->DrivePower = 0;
  param->BaudRate = 115200UL;
  param->SerialWatchdogMs = 500U;
  param->PdoMissLimit = 3U;
  param->FailSafePolicy = FAILSAFE_DISABLE_OUTPUT;
  param->ControlSource = CONTROL_SOURCE_PWM_INPUT;
  param->NodeId = 1U;
  param->Topology = BUS_TOPOLOGY_PARALLEL;
  param->NodeCount = 1U;
  param->NodePosition = 1U;
  param->ReplySlotUs = 120U;
  param->FaultCode = 0U;
}

static void App_ApplyUartBaud(uint32_t baud)
{
  if (baud == 115200UL || baud == 500000UL || baud == 1000000UL || baud == 2000000UL)
  {
    huart2.Init.BaudRate = baud;
    (void)HAL_UART_Init(&huart2);
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  PID_Init(&Param_KX);
  App_DefaultParam(&Param_KX);
  (void)NvmParam_Load(&Param_KX);
  Param_KX.CycleTimeMs = 1;
  App_ApplyUartBaud(Param_KX.BaudRate);

  Dynamixel2_Init(&DynamixelBus, &huart2, &Param_KX);
  MT6701_init(&Encoder, &hi2c1, &Param_KX);
  AD116_init(&Drive, &htim3, TIM_CHANNEL_2, TIM_CHANNEL_3, &Param_KX);
  VoltageStatus_init(&Voltage, &hadc1, &Param_KX);
  PWMCapture_Init(&PWMCaptureData, &htim16);
  ServoControl_Init(&Servo, &Param_KX);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    CycleStart(&Drive, &htim14);

    ServoControl_Begin1ms(&Servo);
    Dynamixel2_1msTick(&DynamixelBus);
    ServoControl_SetCommand(&Servo, Dynamixel2_GetActiveCommand(&DynamixelBus));
    VoltageStatus_AnalyzeData(&Voltage);

    if (ServoControl_IsSpeedDue(&Servo))
    {
      MT6701_Update(&Encoder);
      MT6701_SpeedUpdate(&Encoder,
                         (uint32_t)SERVO_SPEED_PERIOD_MS * Param_KX.CycleTimeMs);
    }

    ServoControl_Run1ms(&Servo);
    AD116_Update(&Drive, &Param_KX);
    VoltageStatus_UpdateLogicalCurrent(&Voltage);

    if (ServoControl_ConsumeSaveRequest(&Servo)
        || Dynamixel2_ConsumeSaveRequest(&DynamixelBus))
    {
      (void)NvmParam_Save(&Param_KX);
    }

#if TK_UART_LINK_TEST
    App_UartLinkTest_1ms();
#endif

    CycleBlockingTimer(&Drive, &htim14);
    /* USER CODE END WHILE */
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
