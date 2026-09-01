/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* ADC ranks: INA181 OUT, VREFINT, bus voltage, temperature sensor. */
/* ADC 排序：INA181 输出、VREFINT、母线电压、温度传感器。 */
#define ADC_STATUS_CONVERSION_COUNT  4U
#define ADC_STATUS_CURRENT_INDEX     0U
#define ADC_STATUS_VREFINT_INDEX     1U
#define ADC_STATUS_BUS_INDEX         2U
#define ADC_STATUS_TEMPERATURE_INDEX 3U

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

