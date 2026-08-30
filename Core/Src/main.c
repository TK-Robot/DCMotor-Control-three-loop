/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "TypeDefine.h"
#include "MT6701.h"
#include "AD116.h"
#include "VoltageStatus.h"
#include "PID.h"
#include "PWMCapture/PWMCapture.h"
#include "NvmParam.h"
#include "ServoControl.h"
#include "Dynamixel2.h"
#include "Crsf.h"
#include "CurrentControl.h"
#include "Sds1601Calibration.h"
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
MT6701 Encoder;
AD116 Drive;
VoltageStatus Voltage;
CaptureData PWMCaptureData;
ServoControl Servo;
Dynamixel2Context DynamixelBus;
CrsfContext CrsfInput;
CurrentControl CurrentLoop;
ServoCommand PwmInputCommand;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TK_UART_LINK_TEST 0
static uint32_t App_CurrentUartBaud;

static void App_ApplySds1601ModelDefaultsIfUnconfigured(Param *param)
{
  const bool electrical_model_unconfigured =
      param->MotorTorqueParams.torque_constant_uNm_per_A == 0U
      && param->MotorTorqueParams.back_emf_uV_per_rpm == 0U
      && param->MotorTorqueParams.terminal_resistance_mOhm == 0U;
  const bool legacy_electrical_calibration =
      param->MotorTorqueParams.torque_constant_uNm_per_A
          == SDS1601_LEGACY_TORQUE_CONSTANT_UNM_PER_A
      && param->MotorTorqueParams.back_emf_uV_per_rpm
          == SDS1601_LEGACY_BACK_EMF_UV_PER_RPM;
  const bool mechanical_model_unconfigured =
      param->MechanicalParams.total_inertia_ug_cm2 == 0U
      && param->MechanicalParams.coulomb_friction_uNm == 0U
      && param->MechanicalParams.viscous_friction_nNm_per_rpm == 0U
      && param->MechanicalParams.friction_deadband_cps == 0U;
  const bool legacy_mechanical_calibration =
      param->MechanicalParams.coulomb_friction_uNm
          == SDS1601_LEGACY_COULOMB_FRICTION_UNM;

  if (electrical_model_unconfigured || legacy_electrical_calibration)
  {
    param->MotorTorqueParams.torque_constant_uNm_per_A =
        SDS1601_TORQUE_CONSTANT_UNM_PER_A;
    param->MotorTorqueParams.torque_temp_coefficient_ppm_per_C = -1000;
    param->MotorTorqueParams.back_emf_uV_per_rpm =
        SDS1601_BACK_EMF_UV_PER_RPM;
    param->MotorTorqueParams.terminal_resistance_mOhm =
        SDS1601_TERMINAL_RESISTANCE_MOHM;
    param->MotorTorqueParams.resistance_temp_coefficient_ppm_per_C = 4000U;
    param->MotorTorqueParams.brush_drop_mV = 0U;
    param->MotorTorqueParams.reference_temperature_C = 25;

    /* New configurations use cycle-by-cycle mixed decay. Existing explicit
     * mode selections remain available for hardware diagnostics. */
  }
  if (param->TorqueCurrentLimit_mA == 0U)
  {
    param->TorqueCurrentLimit_mA = 400U;
  }
  if (param->MotorInductance_uH == 0U) param->MotorInductance_uH = 10U;
  if (param->CurrentPeakLimit_mA == 0U) param->CurrentPeakLimit_mA = 1500U;
  if (param->CurrentAbsoluteLimit_mA == 0U) param->CurrentAbsoluteLimit_mA = 1750U;
  if (param->StallCurrentThreshold_mA == 0U) param->StallCurrentThreshold_mA = 300U;
  if (param->StallSpeedThreshold_cps == 0U) param->StallSpeedThreshold_cps = 300U;
  if (param->StallConfirmTimeMs == 0U) param->StallConfirmTimeMs = 3000U;
  if (mechanical_model_unconfigured || legacy_mechanical_calibration)
  {
    /* Inertia remains disabled until a higher-rate identification is available. */
    param->MechanicalParams.total_inertia_ug_cm2 = 0U;
    param->MechanicalParams.coulomb_friction_uNm =
        SDS1601_COULOMB_FRICTION_UNM;
    param->MechanicalParams.viscous_friction_nNm_per_rpm =
        SDS1601_VISCOUS_FRICTION_NNM_PER_RPM;
    param->MechanicalParams.friction_deadband_cps =
        SDS1601_FRICTION_DEADBAND_CPS;
  }
  param->MotorWindingTemperature_C =
      param->MotorTorqueParams.reference_temperature_C;

  /* Migrate the previously shipped value that converted roughly 300 mA of
   * friction compensation instead of the measured approximately 40 mA. */
  if (param->MechanicalParams.coulomb_friction_uNm == 73500U)
  {
    param->MechanicalParams.coulomb_friction_uNm =
        SDS1601_COULOMB_FRICTION_UNM;
    param->MechanicalParams.viscous_friction_nNm_per_rpm =
        SDS1601_VISCOUS_FRICTION_NNM_PER_RPM;
  }
}

#if TK_UART_LINK_TEST
static void App_UartLinkTest_1ms(void)
{
  static uint16_t tick_ms = 0;
  static const uint8_t test_frame[] = {'T', 'K', '1', '1', '5', '2', '0', '0', '\r', '\n'};

  if (++tick_ms >= 1000U)
  {
    tick_ms = 0;
    for (uint32_t i = 0U; i < sizeof(test_frame); ++i)
    {
      while (!LL_USART_IsActiveFlag_TXE_TXFNF(USART2)) {}
      LL_USART_TransmitData8(USART2, test_frame[i]);
    }
  }
}
#endif

static void App_DefaultParam(Param *param)
{
  param->CycleTimeMs = 1;
  param->DutyRatio = 0U;
  param->PwmInputValid = false;
  param->OutputEnabled = false;
  param->TempLimit = 40;
  param->ExpectMA = 0;
  param->CurrentLogical_mA = 0;
  param->CurrentSampleValid = false;
  param->CurrentEstimated = false;
  param->CurrentHardLimitActive = false;
  param->CurrentPeakLimitActive = false;
  param->CurrentAverage_mA = 0;
  param->CurrentPeakChopEvents = 0U;
  param->TorqueEncoderCountsPerRev = 16384U;
  param->TorqueCurrentLimit_mA = 0U;
  param->MotorInductance_uH = 10U;
  param->CurrentPeakLimit_mA = 1500U;
  param->CurrentAbsoluteLimit_mA = 1750U;
  param->StallCurrentThreshold_mA = 300U;
  param->StallSpeedThreshold_cps = 300U;
  param->StallConfirmTimeMs = 3000U;
  param->StallElapsedMs = 0U;
  param->MotorWindingTemperature_C = 25;
  param->MotorTorqueParams = (MotorTorqueModelParams){
    .torque_constant_uNm_per_A = 0U,
    .torque_temp_coefficient_ppm_per_C = 0,
    .back_emf_uV_per_rpm = 0U,
    .terminal_resistance_mOhm = 0U,
    .resistance_temp_coefficient_ppm_per_C = 4000U,
    .brush_drop_mV = 0U,
    .reference_temperature_C = 25
  };
  param->MechanicalParams = (MechanicalModelParams){0};
  param->MotorTorqueResult = (MotorTorqueModelResult){0};
  param->MechanicalResult = (MechanicalModelResult){0};
  param->TargetTorque_uNm = 0;
  param->TargetElectromagneticTorque_uNm = 0;
  param->TorqueCommandVoltageLimited = false;
  param->LowSpeedCompMaxSpeed_cps = 0U;
  param->EncoderExpect = 0;
  param->EncoderOffset = 3400;
  param->EncoderVeer = false;
  param->EncoderRebaseline = false;
  param->SpeedMax = 45000;
  param->PositionDeadbandCounts = PID_POSITION_DEADBAND_DEFAULT_COUNTS;
  param->AccelMax = 60U;
  param->DecelMax = 60U;
  param->EncoderSpeedExpect = 0;
  param->PowerSaveVoltage_mV = 4000;
  param->DriveRunMode = 0;
  param->DrivePwmMode = 4U;
  param->DrivePower = 0;
  param->CurrentFeedforwardPwm = 0;
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
  param->ProtectionFlags = PROTECTION_NONE;
  param->CrsfPositionChannel = 1U;
  param->CrsfCenterChannel = 0U;
  param->CrsfEnableChannel = 0U;
  param->CrsfAutoEnable = false;
  param->CrsfManualEnable = false;
  param->CrsfChannelMin = 172U;
  param->CrsfChannelCenter = 992U;
  param->CrsfChannelMax = 1811U;
  param->CrsfCenterTrigger = 1200U;
  param->CrsfEnableThreshold = 1200U;
  param->CrsfArmCurrentLimit_mA = 300U;
  param->CrsfArmSpeed_cps = 5000U;
  param->CrsfArmFollowError = 100U;
  param->CrsfArmTimeoutMs = 2000U;
  param->CrsfWatchdogMs = 100U;
  param->CrsfNegativePositionLimit = -16384;
  param->CrsfPositivePositionLimit = 16384;
  param->CrsfStatus = 0U;
  param->CrsfRawPosition = 0U;
  param->CrsfRawEnable = 0U;
  param->CrsfRawCenter = 0U;
  param->CrsfCenterReference = 0;
}

static bool App_ApplyUartBaud(uint32_t baud)
{
  if (baud == 115200UL || baud == 500000UL || baud == 1000000UL || baud == 2000000UL)
  {
    LL_USART_Disable(USART2);
    LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_PRESCALER_DIV1,
                         LL_USART_OVERSAMPLING_16, baud);
    LL_USART_Enable(USART2);
    while (!LL_USART_IsActiveFlag_TEACK(USART2)
           || !LL_USART_IsActiveFlag_REACK(USART2))
    {
    }
    App_CurrentUartBaud = baud;
    return true;
  }
  return false;
}

static bool App_ReconfigureUartBaud(uint32_t baud)
{
  uint32_t previous_baud = App_CurrentUartBaud;

  /* The old-baud ACK already reached UART TC; now RX DMA can be restarted safely. */
  /* 旧波特率 ACK 已到达 UART TC，此时可以安全重启 RX DMA。 */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
  LL_USART_DisableDMAReq_RX(USART2);
  if (!App_ApplyUartBaud(baud) || !Dynamixel2_RestartRx(&DynamixelBus))
  {
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
    LL_USART_DisableDMAReq_RX(USART2);
    (void)App_ApplyUartBaud(previous_baud);
    (void)Dynamixel2_RestartRx(&DynamixelBus);
    return false;
  }
  return true;
}

static void App_CrsfRxObserver(void *user, const uint8_t *data, uint16_t length)
{
  Crsf_ProcessBytes((CrsfContext *)user, data, length);
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

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
  PID_Init(&Param_KX);
  App_DefaultParam(&Param_KX);
  (void)NvmParam_Load(&Param_KX);
  App_ApplySds1601ModelDefaultsIfUnconfigured(&Param_KX);
  PID_MigrateLegacyCurrentTuning(&Param_KX);
  Param_KX.CycleTimeMs = 1;
  (void)App_ApplyUartBaud(Param_KX.BaudRate);

  Crsf_Init(&CrsfInput, &Param_KX);
  Dynamixel2_Init(&DynamixelBus, USART2, &Param_KX);
  Dynamixel2_SetRxObserver(&DynamixelBus, App_CrsfRxObserver, &CrsfInput);
  MT6701_init(&Encoder, I2C1, &Param_KX);
  AD116_init(&Drive, TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3, &Param_KX);
  CurrentControl_Init(&CurrentLoop);
  VoltageStatus_init(&Voltage, ADC1, &Param_KX);
  PWMCapture_Init(&PWMCaptureData, TIM16);
  ServoControl_Init(&Servo, &Param_KX);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    CycleStart(&Drive, TIM14);

    ServoControl_Begin1ms(&Servo);
    PWMCapture_1msTick(&PWMCaptureData);
    Param_KX.PwmInputValid = PWMCaptureData.SignalValid;
    Dynamixel2_1msTick(&DynamixelBus);
    Crsf_1msTick(&CrsfInput);
    {
      uint32_t pending_baud;
      if (Dynamixel2_ConsumeBaudRateChange(&DynamixelBus, &pending_baud))
      {
        bool applied = App_ReconfigureUartBaud(pending_baud);
        Dynamixel2_CompleteBaudRateChange(&DynamixelBus, applied);
      }
    }
    if (Param_KX.ControlSource == CONTROL_SOURCE_PWM_INPUT)
    {
      ServoControl_BuildPwmPositionCommand(Param_KX.DutyRatio,
                                           Param_KX.PwmInputValid,
                                           &PwmInputCommand);
      ServoControl_SetCommand(&Servo, &PwmInputCommand);
    }
    else if (Param_KX.ControlSource == CONTROL_SOURCE_CRSF)
    {
      ServoControl_SetCommand(&Servo, Crsf_GetActiveCommand(&CrsfInput));
    }
    else
    {
      ServoControl_SetCommand(&Servo, Dynamixel2_GetActiveCommand(&DynamixelBus));
    }
    VoltageStatus_AnalyzeData(&Voltage);

    if (ServoControl_IsSpeedDue(&Servo))
    {
      MT6701_Update(&Encoder);
      MT6701_SpeedUpdate(&Encoder, Param_KX.ProcessTimeUs);
    }

    ServoControl_Run1ms(&Servo);
    {
      int16_t physical_target_mA = Param_KX.ExpectMA;
      CurrentControlElectrical electrical;
      uint32_t resistance_mOhm = Param_KX.MotorTorqueResult.effective_resistance_mOhm;
      if (Param_KX.EncoderVeer)
        physical_target_mA = (int16_t)-physical_target_mA;
      if (resistance_mOhm == 0U)
        resistance_mOhm = Param_KX.MotorTorqueParams.terminal_resistance_mOhm;
      if (resistance_mOhm > UINT16_MAX) resistance_mOhm = UINT16_MAX;
      electrical = (CurrentControlElectrical){
        .supply_voltage_mV = Param_KX.VCC_mV,
        .resistance_mOhm = (uint16_t)resistance_mOhm,
        .inductance_uH = Param_KX.MotorInductance_uH,
        .peak_limit_mA = Param_KX.CurrentPeakLimit_mA,
        .absolute_limit_mA = Param_KX.CurrentAbsoluteLimit_mA
      };
      NVIC_DisableIRQ(DMA1_Channel1_IRQn);
      CurrentControl_SetCommand(&CurrentLoop,
                                Param_KX.OutputEnabled,
                                 physical_target_mA,
                                 Param_KX.CurrentFeedforwardPwm,
                                 (Param_KX.CurrentLoopStatus & 0x0080U) != 0U
                                   ? 1U : Param_KX.DrivePwmMode,
                                 &electrical);
      NVIC_EnableIRQ(DMA1_Channel1_IRQn);
      if (!CurrentLoop.enabled)
      {
        AD116_Update(&Drive, &Param_KX);
        Param_KX.CurrentHardLimitActive = false;
      }
    }
    VoltageStatus_UpdateLogicalCurrent(&Voltage);

    {
      bool servo_save_request = ServoControl_ConsumeSaveRequest(&Servo);
      bool dynamixel_save_request = Dynamixel2_ConsumeSaveRequest(&DynamixelBus);
      if (servo_save_request || dynamixel_save_request)
      {
        NvmParamStatus save_status = NvmParam_Save(&Param_KX);
        if (dynamixel_save_request)
        {
          /* Finish the unicast transaction only after durable Flash completion. */
          /* 仅在 Flash 持久化完成后结束单播保存事务。 */
          Dynamixel2_CompleteSaveRequest(&DynamixelBus,
                                         save_status == NVM_PARAM_OK);
        }
      }
    }

#if TK_UART_LINK_TEST
    App_UartLinkTest_1ms();
#endif

    CycleBlockingTimer(&Drive, TIM14);
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
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0U)
  {
  }

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(64000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
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
