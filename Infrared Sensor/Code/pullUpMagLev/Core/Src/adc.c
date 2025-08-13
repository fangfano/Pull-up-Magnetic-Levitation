/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//times(读取次数)
uint32_t ADC_Get_Average(ADC_HandleTypeDef *hadc, uint8_t ch, uint8_t times) {
	ADC_ChannelConfTypeDef sConfig;		//通道初始化
	uint32_t value_sum = 0;
	uint8_t i;
	switch (ch)							//选择ADC通道
	{
	case 0:
		sConfig.Channel = ADC_CHANNEL_0;
		break;
	case 1:
		sConfig.Channel = ADC_CHANNEL_1;
		break;
	case 2:
		sConfig.Channel = ADC_CHANNEL_2;
		break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_3;
		break;
	case 4:
		sConfig.Channel = ADC_CHANNEL_4;
		break;
	case 5:
		sConfig.Channel = ADC_CHANNEL_5;
		break;
	case 6:
		sConfig.Channel = ADC_CHANNEL_6;
		break;
	case 7:
		sConfig.Channel = ADC_CHANNEL_7;
		break;
	case 8:
		sConfig.Channel = ADC_CHANNEL_8;
		break;
	case 9:
		sConfig.Channel = ADC_CHANNEL_9;
		break;
	case 10:
		sConfig.Channel = ADC_CHANNEL_10;
		break;
	case 11:
		sConfig.Channel = ADC_CHANNEL_11;
		break;
	case 17:
		sConfig.Channel = ADC_CHANNEL_17;
		break;
	}
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;		//采用周期239.5周期
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(hadc, &sConfig);
	for (i = 0; i < times; i++) {
		HAL_ADC_Start(hadc);								//启动转换
		HAL_ADC_PollForConversion(hadc, 30);				//等待转化结束
		value_sum += HAL_ADC_GetValue(hadc);				//求和
		HAL_ADC_Stop(hadc);								//停止转换
	}
	return (double) value_sum / times;									//返回平均值
}

//以内部参考电压为基准得到ADC的平均电压
//ifCali=0 不进行校准    ifCali！=0 进行校准
double ADC_GetVotage(ADC_HandleTypeDef *hadc, uint8_t ch, uint8_t times,
		uint8_t ifCali) {
	uint32_t inf, value;
	if (ifCali) {

		HAL_ADCEx_Calibration_Start(hadc);
	}
	value = ADC_Get_Average(hadc, ch, times);
	if (ifCali) {

		HAL_ADCEx_Calibration_Start(&hadc1);
	}
	inf = ADC_Get_Average(&hadc1, 17, times);
	return 1.20 * (double)value / (double)inf;

}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
