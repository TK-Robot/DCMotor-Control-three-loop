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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>

#include "TypeDefine.h"
#include "MT6701.h"
#include "UartProto.h"
#include "AD116.h"
#include "VoltageStatus.h"
#include "PID.h"
#include "PWMCapture/PWMCapture.h"
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
Param Param_KX;
UartProto U2Comm;
MT6701 Encoder;
AD116 Drive;
VoltageStatus Voltage;
CaptureData PWMCaptureData;
static volatile bool TxFinishFlag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  Param_KX.DutyRatio =PWMCapture_Calculate(&PWMCaptureData,htim);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  UartProto_RxEventCallback(&U2Comm,huart,Size);
  if( Param_KX.EncoderExpect == 16383)
  {
    Param_KX.EncoderExpect=0;
  }
  else if(Param_KX.EncoderExpect == 0)
  {
    Param_KX.EncoderExpect=16383;
  }
  // if(Param_KX.EncoderSpeedExpect==25000)
  // {
  //  Param_KX.EncoderSpeedExpect=2000;
  // }
  // else if(Param_KX.EncoderSpeedExpect == 2000)
  // {
  //   Param_KX.EncoderSpeedExpect=25000;
  // }
  // if( Param_KX.ExpectMA == 40)
  // {
  //   Param_KX.ExpectMA=80;
  // }
  // else if(Param_KX.ExpectMA == 80)
  // {
  //   Param_KX.ExpectMA=40;
  // }
  //Param_KX.DrivePower= Param_KX.DrivePower+32;
  //Param_KX.EncoderExpect=1000;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    // 发送完成后的回调处理，可根据需要添加逻辑
    TxFinishFlag= false;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    uint32_t err = HAL_UART_GetError(huart);

    if (err & HAL_UART_ERROR_DMA) {
      // 尝试恢复
      HAL_UART_AbortTransmit(huart);  // 终止当前传输
      // 可记录日志、重启DMA、报警等
    }
  }
}

// void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
// {
//   if (hi2c == &hi2c1)
//   {
//     MT6701_CodedManage(&Encoder);
//     Encoder.dma_busy = false;
//   }
// }
//
// void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
// {
//   if (hi2c == &hi2c1)
//   {
//     Encoder.dma_busy = false;
//     HAL_I2C_DeInit(hi2c);
//     HAL_I2C_Init(hi2c);
//   }
// }


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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  Param_KX.CycleTimeMs = 5;
  Param_KX.TempLimit = 40;
  PID_Init(&Param_KX);
  UartProto_init(&U2Comm,&huart2,&Param_KX);
  MT6701_init(&Encoder,&hi2c1,&Param_KX);
  AD116_init(&Drive,&htim3,TIM_CHANNEL_2,TIM_CHANNEL_3,&Param_KX);
  VoltageStatus_init(&Voltage,&hadc1,&Param_KX);
  PWMCapture_Init(&PWMCaptureData,&htim16);
  Param_KX.ExpectMA=40;
  Param_KX.EncoderExpect=16383;
  Param_KX.EncoderOffset=3400;
  Param_KX.SpeedMax=30000;
  Param_KX.AccelMax=60000;
  Param_KX.DecelMax=60000;
  Param_KX.EncoderSpeedExpect=25000;
  int32_t buf[6];
  Param_KX.DrivePower=1000 ;
  //UartProto_SendLongInt32(&U2Comm, buf,sizeof(buf));

  // CycleStart(&Drive,&htim14);//程序开始定时计数
  // MT6701_Update(&Encoder);
  // MT6701_SpeedUpdate(&Encoder);
  // //Param_KX.DriveRunMode=2;
  // //Param_KX.DrivePower=-PID_AbsCalculate(&Param_KX.Pid_Pos, Param_KX.EncoderExpect, Param_KX.EncoderValue);
  // //Param_KX.DrivePower=-PID_Vel_Calc(&Param_KX.Pid_PosVel, Param_KX.EncoderSpeedExpect, Param_KX.EncoderSpeed);
  // Param_KX.DrivePower= -PID_Pos(&Param_KX);
  // AD116_Update(&Drive,&Param_KX);
  // VoltageStatus_AnalyzeData(&Voltage);
  // TempLimit(&Voltage);//温度限制
  // // buf[0]= Param_KX.EncoderSpeed;
  // // buf[1] = Param_KX.EncoderExpect;
  // buf[0] = Param_KX.EncoderValue;
  // // buf[3] = (int32_t)Param_KX.DrivePower;
  // // buf[4] = Param_KX.AccDec;
  // //buf[0]= PWMCaptureData.CaptureOneUpTime;
  // //buf[1]= PWMCaptureData.CaptureOneDownTime;
  // //buf[2]= PWMCaptureData.CaptureTwoUpTime;
  // buf[1]= PWMCaptureData.DutyRatio;
  // //buf[4] = PWMCaptureData.EdgeNumber;
  // UartProto_SendLongInt32(&U2Comm, buf,2);
  // CycleBlockingTimer(&Drive,&htim14);//程序定时计数结束阻塞

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    CycleStart(&Drive,&htim14);//程序开始定时计数
    MT6701_Update(&Encoder);
    MT6701_SpeedUpdate(&Encoder);
    Param_KX.DriveRunMode=2;
    //Param_KX.DrivePower=PID_AbsCalculate(&Param_KX.Pid_Pos, Param_KX.EncoderExpect, Param_KX.EncoderValue);
    //Param_KX.ExpectMA=PID_Vel_Calc(&Param_KX.Pid_PosVel, Param_KX.EncoderSpeedExpect, Param_KX.EncoderSpeed);
    //Param_KX.DrivePower=-PID_AbsCalculate(&Param_KX.Pid_PosEle,Param_KX.ExpectMA,Param_KX.INA181_mA);
    // Param_KX.DrivePower= PID_Pos(&Param_KX);
    AD116_Update(&Drive,&Param_KX);
    VoltageStatus_AnalyzeData(&Voltage);
    //TempLimit(&Voltage);//温度限制
    buf[0]= Param_KX.EncoderSpeed;
    buf[1] =(int32_t) Param_KX.EncoderValue;
    //buf[2]= (int32_t)PWMCaptureData.DutyRatio;
    //Param_KX.EncoderExpect=MAP(PWMCaptureData.DutyRatio,1000,2000,0,16383);
    //buf[1]= (int32_t)Param_KX.EncoderSpeedExpect;
    //buf[2]=Param_KX.EncoderMultiTurnValue;
    //buf[3]=Param_KX.EncoderExpect;
    buf[2]=Param_KX.INA181_mA;
    buf[3]=Param_KX.INA181_mV;
    buf[4]=Param_KX.DrivePower;
    //buf[5]=Param_KX.ExpectMA;
    UartProto_SendLongInt32(&U2Comm, buf,5);
    CycleBlockingTimer(&Drive,&htim14);//程序定时计数结束阻塞
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
