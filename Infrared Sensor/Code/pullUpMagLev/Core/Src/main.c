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
 * opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h> // For atof()
#include <string.h> // For strcmp()
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 20 // Define size for UART reception buffer
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_TYPE pid;
uint8_t isTriggered = 0;
uint8_t num; // For single character commands
float target = 3.18; // Initial target value, now a global variable
uint8_t pid_running = 0; // Flag to control PID operation, 0=stop, 1=start

// Buffer for receiving new target value from UART
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float filtered_value=0;
	float measure;
	// float target = 2.4; // Moved to global scope
	uint16_t adc, inf;
	uint16_t count_p = 0;
	ADC_ChannelConfTypeDef sConfig;
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	pid.P = 1025;        //P.I.D常量初始化   不同的线圈,不同的电源,参数不同
	pid.I = 0;
	pid.D = 568;
	pid.Ilimit = 0.8;
	pid.PreError = 0;
	pid.Ilimit_flag = 0;
	pid.Irang = 10;
	pid.Integral = 0;

	HAL_ADCEx_Calibration_Start(&hadc1);    //校准ADC
	measure = ADC_GetVotage(&hadc1, 0, 3, 1);
	pid.PreError = target - measure;

	HAL_TIM_Base_Start_IT(&htim2);          //开启定时器,定时采样
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // Start with PWM off

	HAL_UART_Receive_IT(&huart1, &num, 1);    //开启串口接收中断

	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;		//采用周期239.5周期
	sConfig.Rank = 1;

	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);           //开机LED闪烁
	HAL_Delay(200);												 //调试时使用,以判断单片机是否运行
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	HAL_Delay(200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (isTriggered) {
			isTriggered = 0;

            // ADC sampling part
			sConfig.Channel = ADC_CHANNEL_0;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 30);
			adc = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			sConfig.Channel = ADC_CHANNEL_17;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 30);
			inf = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			measure = 1.20 * (float) adc / (float) inf;
			filtered_value = 0.5 * filtered_value + 0.5 * measure;

            if(pid_running)
            {

                PID_Position_Cal(&pid, target, measure);

                if (pid.Output < 0.0) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                } else if (pid.Output > 100.0) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Instance->ARR);
                } else {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t )(pid.Output / 100.0 * htim3.Instance->ARR));
                }

                count_p++;
                if(count_p >= 50){ // Use >= for safety
                    count_p = 0;
                    printf("t%d,m%d,pid%d\n",(int)(target*100),(int)(measure*100), (int)(pid.Output));
                }

            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // Keep PWM off if stopped
                pid.Integral = 0; // Reset integral term to prevent windup
            }
		}

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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/* USER CODE BEGIN 4 */

//定时采样
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { // 1.25Khz
	if (htim == (&htim2)) {
		isTriggered = 1;
	}
}


/******************************************************************************
 * @brief  UART接收完成回调函数
 * @note   此函数在每次通过UART1成功接收一个字节后被调用
 * - 实现了通过"start"和"stop"指令控制PID启停
 * - 实现了通过特定字符('+','-','(',')','<','>')在线调整PID参数
 * - 实现了通过接收一串数字（以换行符'\n'结尾）来修改目标值`target`
 * - 新增：通过 'p', 'i', 'd' + 数字 的形式直接设定PID参数
 ******************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        // Check for single character PID tuning commands
        if (num == '+') {
            pid.D += 1;
			printf("P:%d, I*10:%d, D:%d\n", (int)pid.P, (int)(pid.I * 10), (int)pid.D);
        } else if (num == '-') {
            pid.D -= 1;
			printf("P:%d, I*10:%d, D:%d\n", (int)pid.P, (int)(pid.I * 10), (int)pid.D);
        } else if (num == ')') {
            pid.P += 2;
			printf("P:%d, I*10:%d, D:%d\n", (int)pid.P, (int)(pid.I * 10), (int)pid.D);
        } else if (num == '(') {
            pid.P -= 2;
			printf("P:%d, I*10:%d, D:%d\n", (int)pid.P, (int)(pid.I * 10), (int)pid.D);
        } else if (num == '<') {
            pid.I -= 0.1;
			printf("P:%d, I*10:%d, D:%d\n", (int)pid.P, (int)(pid.I * 10), (int)pid.D);
        } else if (num == '>') {
            pid.I += 0.1;
			printf("P:%d, I*10:%d, D:%d\n", (int)pid.P, (int)(pid.I * 10), (int)pid.D);
        }
        // Process string commands (start/stop/target value/PID set)
        else {
            if (num != '\n' && num != '\r' && rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = num;
            } else { // Received a newline or buffer is full
                rx_buffer[rx_index] = '\0';
                if (rx_index > 0) {
                    if (strcmp((const char*)rx_buffer, "start") == 0) {
                        pid_running = 1;
                        printf("PID started.\n");
                    } else if (strcmp((const char*)rx_buffer, "stop") == 0) {
                        pid_running = 0;
                        printf("PID stopped.\n");
                    }
                    // ===== NEW FUNCTIONALITY START =====
                    else if (rx_buffer[0] == 'p' || rx_buffer[0] == 'P') {
                        pid.P = atof((const char*)rx_buffer + 1);
                        printf("P:%d, I*10:%d, D:%d\n", (int)pid.P, (int)(pid.I * 10), (int)pid.D);
                    } else if (rx_buffer[0] == 'i' || rx_buffer[0] == 'I') {
                        pid.I = atof((const char*)rx_buffer + 1);
                        printf("P:%d, I*10:%d, D:%d\n", (int)pid.P, (int)(pid.I * 10), (int)pid.D);
                    } else if (rx_buffer[0] == 'd' || rx_buffer[0] == 'D') {
                        pid.D = atof((const char*)rx_buffer + 1);
                        printf("P:%d, I*10:%d, D:%d\n", (int)pid.P, (int)(pid.I * 10), (int)pid.D);
                    }
                    // ===== NEW FUNCTIONALITY END =====
                    else {
                        float new_target = atof((const char*)rx_buffer);
                        if (new_target > 0) { // Basic validation
                            target = new_target;
                            printf("New Target set to: %d\n", (int)(target * 100));
                        }
                    }
                }
                rx_index = 0; // Reset buffer index for the next command
            }
        }

        // Re-enable UART receive interrupt
        HAL_UART_Receive_IT(&huart1, &num, 1);
    }
}

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
	while (1) {
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
