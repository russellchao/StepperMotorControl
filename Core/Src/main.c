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
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// The max and min clock ticks that the motor moves between. Cruising occurs at MIN_STEP_INTERVAL.
#define MAX_STEP_INTERVAL 200U
#define MIN_STEP_INTERVAL 20U
#define STEPS_PER_DIRECTION_CRUISE 2000U

#define OCT_BASE_TICKS_PER_STEP 20U
#define OCT_BASE_STEPS_PER_D 300U
#define SOLENOID_DELAY 10000U

#define OCT_LEN 9U   /* 8 vertices + closing point (same as the first) */

/* Pins */
#define X_PUL_PIN GPIO_PIN_0
#define Y_PUL_PIN GPIO_PIN_1
#define X_DIR_PIN GPIO_PIN_5
#define Y_DIR_PIN GPIO_PIN_6
#define SOLENOID_PIN GPIO_PIN_7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* Ramping (not used right now) */
typedef enum
{
  MOTION_RAMP_UP,
  MOTION_CRUISE,
  MOTION_RAMP_DOWN
} motion_state_t;

volatile motion_state_t motion_state = MOTION_RAMP_UP;
volatile uint32_t step_interval = MAX_STEP_INTERVAL;
volatile uint32_t ticks_between_intervals = 0;
volatile uint32_t cruise_step_count = 0;               /* shared with ISR, so volatile */

volatile uint32_t secondary_ticks = 0;
volatile uint32_t secondary_step_count = 0;

/* Drawing an Octagon */
static const int32_t oct_x[OCT_LEN] = {1, 3, 4, 4, 3, 1, 0, 0, 1}; // Note: The last index is the same as the first
static const int32_t oct_y[OCT_LEN] = {0, 0, 3, 5, 8, 8, 5, 3, 0};

volatile int32_t oct_current_stage = 0;
volatile int32_t oct_x_ticks_per_step = 0;
volatile int32_t oct_y_ticks_per_step = 0;

/* Per-segment progress. Step counts are exact; tick intervals may be rounded. */
volatile uint32_t oct_x_steps_left = 0;
volatile uint32_t oct_y_steps_left = 0;
volatile int32_t  oct_x_tick_count = 0;
volatile int32_t  oct_y_tick_count = 0;

volatile int32_t solenoid_tick_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void set_ticks_per_xy_step(int32_t dx, int32_t dy);
static void oct_start_segment(int32_t stage);

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
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

  /* Load the first segment (sets DIR pins) before the timer starts ticking.
     Assumes the pen starts at (oct_x[0], oct_y[0]). */
  oct_current_stage = 0;
  oct_start_segment(oct_current_stage);

  solenoid_tick_count = 0;

  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  /* TIM2 not started: TIM1 now drives both axes so they share one time base. */
  // HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA5 PA6
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void set_ticks_per_xy_step(int32_t dx, int32_t dy)
{
  const int32_t base = (int32_t)OCT_BASE_TICKS_PER_STEP;
  int32_t adx = abs(dx);
  int32_t ady = abs(dy);

  if (adx == 0 && ady == 0)            /* no movement */
  {
    oct_x_ticks_per_step = 0;
    oct_y_ticks_per_step = 0;
    return;
  }

  if (adx >= ady)                      /* X is the major axis */
  {
    oct_x_ticks_per_step = base;
    oct_y_ticks_per_step = (ady == 0) ? 0 : (base * adx + ady / 2) / ady;  /* rounded */
  }
  else                                 /* Y is the major axis */
  {
    oct_y_ticks_per_step = base;
    oct_x_ticks_per_step = (adx == 0) ? 0 : (base * ady + adx / 2) / adx;  /* rounded */
  }
}

/**
  * Load segment `stage` -> `stage + 1`: set DIR pins, exact step counts,
  * and (possibly rounded) tick intervals. Call only while PUL pins are low.
  * Flip the SET/RESET choice for an axis if it moves the wrong way.
  */
static void oct_start_segment(int32_t stage)
{
  int32_t dx = oct_x[stage + 1] - oct_x[stage];
  int32_t dy = oct_y[stage + 1] - oct_y[stage];

  HAL_GPIO_WritePin(GPIOA, X_DIR_PIN, (dx >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, Y_DIR_PIN, (dy >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  oct_x_steps_left = (uint32_t)abs(dx) * OCT_BASE_STEPS_PER_D;
  oct_y_steps_left = (uint32_t)abs(dy) * OCT_BASE_STEPS_PER_D;

  set_ticks_per_xy_step(dx, dy);

  oct_x_tick_count = 0;
  oct_y_tick_count = 0;
}

/**
  * Called on TIM1 CH1 and TIM2 CH1 compare match, i.e. the falling edge of each STEP pulse.
  * Toggling DIR here (not at the rising edge) gives the driver plenty of
  * DIR setup time before the next step.
  *
  * One full cycle per direction:
  *   RAMP_UP   :
  *   CRUISE    :
  *   RAMP_DOWN :
  *   -> toggle DIR (while slowest) -> back to RAMP_UP
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
	  /* Main interrupt: fires every tick (100 microseconds) and drives BOTH axes. */

	  /* 1. End any PUL pulse started on the previous tick (pulse width = 1 tick). */
	  HAL_GPIO_WritePin(GPIOA, X_PUL_PIN | Y_PUL_PIN, GPIO_PIN_RESET);

	  /* 2. Segment finished? Both axes must have taken their exact step count.
	        Done here, with PUL low, so DIR changes a full tick before the next
	        rising edge. */
	  if (oct_x_steps_left == 0 && oct_y_steps_left == 0)
	  {
		  if (oct_current_stage + 1 >= (int32_t)(OCT_LEN - 1))
		  {
			  /* Back at the start. Introduce a 1s delay (10000 ticks) for the solenoid pen to rise/drop.
			   * The pen will draw every other cycle while it's dropped. */
			  if (solenoid_tick_count < SOLENOID_DELAY)
			  {
				  // During delay
				  if (solenoid_tick_count == 0)
					  HAL_GPIO_TogglePin(GPIOA, SOLENOID_PIN);
				  solenoid_tick_count++;
				  return;
			  }
			  else
			  {
				  // End of delay
				  oct_current_stage = 0;
				  solenoid_tick_count = 0;
			  }

		  }
		  else
		  {
			  oct_current_stage++;
		  }
		  oct_start_segment(oct_current_stage);

		  return;
	  }

	  /* 3. X axis: step when its interval elapses, until its steps run out. */
	  if (oct_x_steps_left > 0 && ++oct_x_tick_count >= oct_x_ticks_per_step)
	  {
		  oct_x_tick_count = 0;
		  HAL_GPIO_WritePin(GPIOA, X_PUL_PIN, GPIO_PIN_SET);   /* rising edge */
		  oct_x_steps_left--;
	  }

	  /* 4. Y axis: same, with its own interval and count. */
	  if (oct_y_steps_left > 0 && ++oct_y_tick_count >= oct_y_ticks_per_step)
	  {
		  oct_y_tick_count = 0;
		  HAL_GPIO_WritePin(GPIOA, Y_PUL_PIN, GPIO_PIN_SET);
		  oct_y_steps_left--;
	  }

	  return;
  }

  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
	  /* Secondary motor interrupt */
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
