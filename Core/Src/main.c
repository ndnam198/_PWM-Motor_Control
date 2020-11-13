/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myDebug.h"
#include "myF103.h"
//#include "myFlash.h"
//#include "myBootLoader.h"
#include "myMisc.h"
//#include "myRingBuffer.h"
//#include "myRTOSaddons.h"
#include "retarget.h"
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MOTOR_1 (1U)
#define MOTOR_2 (2U)
#define CLOCKWISE (1)
#define ANTI_CLOCKWISE (-1)
#define TIM_PWM_OVERFLOW_VALUE (100U)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int32_t rotate_direction = 1;
volatile uint32_t motor_select = 1;

static RTC_TimeTypeDef time_stamp = {0};

uint16_t adc_value[3];
uint8_t speed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void _Error_Handler(char *file, int line);
void motorControl(uint32_t speed, int32_t rotate_direction,
                  uint32_t motor_select);
void motorHalt(uint32_t motor_select);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  // MX_IWDG_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  __RETARGET_INIT(DEBUG_USART);
  __PRINT_RESET_CAUSE();
  __MY_OFF_ALL_LED();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value,
                    NUMBER_OF_ELEMENT(adc_value));

  time_stamp.Hours = 0x01;
  time_stamp.Minutes = 0x2F;
  time_stamp.Seconds = 0x00;
  if (HAL_RTC_SetTime(&hrtc, &time_stamp, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    newline;
    vTimeStamp(HAL_GetTick());
    PRINT_VAR((uint32_t)adc_value[0]);
    speed = (int)(((float)adc_value[0] / 4096) * TIM_PWM_OVERFLOW_VALUE);
    motorControl(speed, rotate_direction, motor_select);
    __MY_TOGGLE_LED(LED_2);
    HAL_RTC_GetTime(&hrtc, &time_stamp, RTC_FORMAT_BIN);
    HAL_Delay(500);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint8_t motor_state, rotate_state;
  /* Change rotate direction */
  if (GPIO_Pin == BT_DOWN_Pin)
  {
    volatile uint32_t wait = 100000;
    while ((wait--) != 0)
    {
      __NOP();
    }
    if (HAL_GPIO_ReadPin(BT_DOWN_GPIO_Port, BT_DOWN_Pin) == 0)
    {
      __MY_TOGGLE_LED(LED_3);
      if (rotate_state == 0)
      {
        rotate_state = 1;
        rotate_direction = ANTI_CLOCKWISE;
      }
      else if (rotate_state == 1)
      {
        rotate_state = 0;
        rotate_direction = CLOCKWISE;
      }
      PRINTF("--------------------\r\n");
      PRINTF("BT_DOWN pressed\r\n");
      PRINT_VAR(rotate_direction);
      PRINTF("--------------------\r\n");
    }
    else
    {
      return;
    }
  }

  /* Switch control between motor */
  if (GPIO_Pin == BT_CENTER_Pin)
  {
    volatile uint32_t wait = 100000;
    while ((wait--) != 0)
    {
      __NOP();
    }
    if (HAL_GPIO_ReadPin(BT_CENTER_GPIO_Port, BT_CENTER_Pin) == 0)
    {
      if (motor_state == 0)
      {
        motor_state = 1;
        motor_select = MOTOR_2;
      }
      else if (motor_state == 1)
      {
        motor_state = 0;
        motor_select = MOTOR_1;
      }
      PRINTF("--------------------\r\n");
      PRINTF("BT_CENTER pressed\r\n");
      PRINT_VAR(motor_select);
      PRINTF("--------------------\r\n");
    }
    else
    {
      return;
    }
  }

  if (GPIO_Pin == BT_UP_Pin)
  {
    volatile uint32_t wait = 100000;
    while ((wait--) != 0)
    {
      __NOP();
    }
    if (HAL_GPIO_ReadPin(BT_UP_GPIO_Port, BT_UP_Pin) == 0)
    {

      HAL_GPIO_TogglePin(L298N_RELAY_CONTROL_GPIO_Port,
                         L298N_RELAY_CONTROL_Pin);
      PRINTF("--------------------\r\n");
      PRINTF("BT_UP pressed\r\n");
      PRINTF("L298N power state changed\r\n");
      PRINTF("--------------------\r\n");
    }
    else
    {
      return;
    }
  }
}
/* Switch control to another motor */

/**
 * @brief Control motor speed 
 * 
 * @param speed equivalent to duty_cycle impose to motor, range from 1 -> TIM_PWM_OVERFLOW_VALUE
 * @param rotate_direction 
 */
void motorControl(uint32_t speed, int32_t rotate_direction,
                  uint32_t motor_select)
{
  if ((rotate_direction != 1) && (rotate_direction != -1))
  {
    PRINTF("Wrong parameter: rotate_direction\r\n");
    return;
  }

  if (speed > TIM_PWM_OVERFLOW_VALUE)
  {
    PRINTF("Wrong parameter: speed\r\n");
    return;
  }
  uint32_t duty_cycle = speed;
  PRINT_VAR(speed);
  switch (motor_select)
  {
  case MOTOR_1: /* Anti-clockwise rotate */
  {
    PRINTF("MOTOR_1 selected\r\n");
    motorHalt(MOTOR_2);
    if (rotate_direction == CLOCKWISE)
    {
      PRINTF("MOTOR_1: Clockwise rotating\r\n");
      HAL_GPIO_WritePin(INT2_NO_PWM_MOTOR_1_GPIO_Port,
                        INT2_NO_PWM_MOTOR_1_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,
                            TIM_PWM_OVERFLOW_VALUE - duty_cycle);
    }
    else if (rotate_direction == ANTI_CLOCKWISE)
    {
      PRINTF("MOTOR_1: Anti-Clockwise rotating\r\n");
      HAL_GPIO_WritePin(INT2_NO_PWM_MOTOR_1_GPIO_Port,
                        INT2_NO_PWM_MOTOR_1_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty_cycle);
    }
    break;
  }
  case MOTOR_2: /* Clockwise rotate */
  {
    PRINTF("MOTOR_2 selected\r\n");
    motorHalt(MOTOR_1);
    if (rotate_direction == CLOCKWISE)
    {
      PRINTF("MOTOR_2: Clockwise rotating\r\n");
      HAL_GPIO_WritePin(INT4_NO_PWM_MOTOR_2_GPIO_Port,
                        INT4_NO_PWM_MOTOR_2_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_cycle);
    }
    else if (rotate_direction == ANTI_CLOCKWISE)
    {
      PRINTF("MOTOR_2: Anti-Clockwise rotating\r\n");
      HAL_GPIO_WritePin(INT4_NO_PWM_MOTOR_2_GPIO_Port,
                        INT4_NO_PWM_MOTOR_2_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
                            TIM_PWM_OVERFLOW_VALUE - duty_cycle);
    }
    break;
  }
  default:
    break;
  }
}

void motorHalt(uint32_t motor_select)
{
  if (motor_select == MOTOR_1)
  {
    HAL_GPIO_WritePin(INT2_NO_PWM_MOTOR_1_GPIO_Port,
                      INT2_NO_PWM_MOTOR_1_Pin, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  }
  else if (motor_select == MOTOR_2)
  {
    HAL_GPIO_WritePin(INT4_NO_PWM_MOTOR_2_GPIO_Port,
                      INT4_NO_PWM_MOTOR_2_Pin, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  static volatile uint32_t count;

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
