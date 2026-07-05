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
#include "UartProto.h"
#include "AD116.h"
#include "VoltageStatus.h"
#include "PID.h"
#include "PWMCapture/PWMCapture.h"
#include "NvmParam.h"
#include "ServoControl.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
Param Param_KX;
UartProto U2Comm;
MT6701 Encoder;
AD116 Drive;
VoltageStatus Voltage;
CaptureData PWMCaptureData;
ServoControl Servo;
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  Param_KX.DutyRatio = PWMCapture_Calculate(&PWMCaptureData, htim);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  UartProto_RxEventCallback(&U2Comm, huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    U2Comm.tx_busy = false;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint32_t err = HAL_UART_GetError(huart);
    if ((err & HAL_UART_ERROR_DMA) != 0U)
    {
      HAL_UART_AbortTransmit(huart);
      U2Comm.tx_busy = false;
    }
  }
}

static void App_DefaultParam(Param *param)
{
  param->CycleTimeMs = 1;
  param->TempLimit = 40;
  param->ExpectMA = 0;
  param->EncoderExpect = 0;
  param->EncoderOffset = 3400;
  param->SpeedMax = 30000;
  param->AccelMax = 60000;
  param->DecelMax = 60000;
  param->EncoderSpeedExpect = 0;
  param->PowerSaveVoltage_mV = 4000;
  param->DriveRunMode = 0;
  param->DrivePower = 0;
}

static void App_SendTelemetry(void)
{
  int32_t buf[6];

  buf[0] = (int32_t)Servo.command.mode;
  buf[1] = Param_KX.EncoderSpeed;
  buf[2] = Param_KX.EncoderMultiTurnValue;
  buf[3] = Param_KX.INA181_mA;
  buf[4] = Param_KX.DrivePower;
  buf[5] = Param_KX.VCC_mV;
  UartProto_SendLongInt32(&U2Comm, buf, 6);
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

  UartProto_init(&U2Comm, &huart2, &Param_KX);
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
    VoltageStatus_AnalyzeData(&Voltage);

    if (ServoControl_IsSpeedDue(&Servo))
    {
      MT6701_Update(&Encoder);
      MT6701_SpeedUpdate(&Encoder);
    }

    ServoControl_Run1ms(&Servo);
    AD116_Update(&Drive, &Param_KX);

    if (ServoControl_ConsumeSaveRequest(&Servo))
    {
      (void)NvmParam_Save(&Param_KX);
    }

    if (ServoControl_IsTelemetryDue(&Servo))
    {
      App_SendTelemetry();
    }

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
