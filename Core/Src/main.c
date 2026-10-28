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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ST 1500

/*
 * Packet from the Pi (20 Hz):   $1500,1500,1500,1,0*05\n
 *   fields: left, right, servo, winch1, winch2
 *   checksum: XOR of every char between '$' and '*', as 2 hex digits
 * Pi is wired to USART1: PA9 = TX (Nucleo D8), PA10 = RX (Nucleo D2).
 */
#define PI_UART            huart1   /* UART connected to the Raspberry Pi */
#define RX_MAX_LEN         40       /* longer lines are thrown away       */
#define NUM_FIELDS         5
#define FAILSAFE_MS        250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Values from the last good packet (startup = neutral / stopped) */
volatile uint16_t leftPulse      = ST;
volatile uint16_t rightPulse     = ST;
volatile uint16_t servoPulse     = 1500;
volatile uint8_t  winch1Cmd      = 0;
volatile uint8_t  winch2Cmd      = 0;
volatile uint32_t lastPacketTick = 0;
volatile uint32_t rejectedCount  = 0;

/* Receive state (only touched inside the UART interrupt) */
static uint8_t  rxByte;                  /* HAL writes each new byte here    */
static char     rxBuf[RX_MAX_LEN + 1];   /* line being built, +1 for '\0'    */
static uint8_t  rxLen = 0;
static uint8_t  rxOverflow = 0;          /* 1 = skipping a too-long line     */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void processLine(char *line, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setThrusters(int left, int right){
	if ((left >= 1000) && (left <= 2000) && (right >= 1000) && (right <= 2000)){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right);
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ST);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ST);
	}
}

void setServos(int angleVal){
	if ((angleVal >= 500) && (angleVal <= 2500)){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, angleVal);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, angleVal);
	}
	else {
		return;
	}
}

/* Turn one hex character into 0-15, or -1 if it isn't hex. */
static int hexVal(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

/*
 * Check one complete line (newline already removed) and, if it is good,
 * save the five numbers. Returns 1 if accepted, 0 if rejected.
 * Ranges are NOT checked here; setThrusters / setServos do that.
 */
static uint8_t parseLine(const char *line, uint8_t len, uint16_t out[NUM_FIELDS])
{
  /* Check 1: starts with '$' */
  if (len == 0 || line[0] != '$') return 0;

  /* Check 2: has a '*' */
  const char *star = memchr(line, '*', len);
  if (star == NULL) return 0;

  /* After the star there must be exactly two hex digits */
  uint8_t starPos = (uint8_t)(star - line);
  if (len != starPos + 3) return 0;
  int hi = hexVal(line[starPos + 1]);
  int lo = hexVal(line[starPos + 2]);
  if (hi < 0 || lo < 0) return 0;
  uint8_t sentCs = (uint8_t)((hi << 4) | lo);

  /* Check 3: checksum = XOR of everything between '$' and '*' */
  uint8_t cs = 0;
  for (uint8_t i = 1; i < starPos; i++) cs ^= (uint8_t)line[i];
  if (cs != sentCs) return 0;

  /* Check 4: exactly five comma-separated fields, digits only */
  uint8_t  field  = 0;
  uint8_t  digits = 0;
  uint32_t value  = 0;
  for (uint8_t i = 1; i <= starPos; i++)
  {
    char c = (i == starPos) ? ',' : line[i];   /* treat '*' as the last comma */

    if (c >= '0' && c <= '9')
    {
      if (++digits > 5) return 0;              /* keeps the number from overflowing */
      value = value * 10 + (uint32_t)(c - '0');
    }
    else if (c == ',')
    {
      if (digits == 0 || field >= NUM_FIELDS) return 0;  /* empty field or too many */
      out[field++] = (uint16_t)value;
      value = 0;
      digits = 0;
    }
    else
    {
      return 0;                                /* anything else (space, '-', ...) */
    }
  }
  return (field == NUM_FIELDS);
}

/* Called once per finished line. Saves values only if every check passes. */
static void processLine(char *line, uint8_t len)
{
  uint16_t v[NUM_FIELDS];

  if (parseLine(line, len, v))
  {
    leftPulse      = v[0];
    rightPulse     = v[1];
    servoPulse     = v[2];
    winch1Cmd      = (uint8_t)v[3];
    winch2Cmd      = (uint8_t)v[4];
    lastPacketTick = HAL_GetTick();
  }
  else
  {
    rejectedCount++;                           /* bad line: change nothing */
  }
}

/* HAL calls this every time one byte has arrived. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &PI_UART) return;

  char c = (char)rxByte;

  if (c == '\n')
  {
    if (rxOverflow)
    {
      rxOverflow = 0;                          /* end of the too-long line */
    }
    else
    {
      if (rxLen > 0 && rxBuf[rxLen - 1] == '\r') rxLen--;  /* allow \r\n too */
      rxBuf[rxLen] = '\0';
      processLine(rxBuf, rxLen);
    }
    rxLen = 0;
  }
  else if (!rxOverflow)
  {
    if (rxLen < RX_MAX_LEN)
    {
      rxBuf[rxLen++] = c;
    }
    else
    {
      /* Longer than 40 chars: throw it away and ignore the rest of it */
      rxLen = 0;
      rxOverflow = 1;
      rejectedCount++;
    }
  }

  HAL_UART_Receive_IT(&PI_UART, &rxByte, 1);   /* listen for the next byte */
}

/* If noise causes a UART error (overrun, framing...), HAL stops receiving.
   Throw away the half-built line and start listening again. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != &PI_UART) return;
  rxLen = 0;
  rxOverflow = 0;
  HAL_UART_Receive_IT(&PI_UART, &rxByte, 1);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_UART_Receive_IT(&PI_UART, &rxByte, 1);   /* start listening to the Pi */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* to run one winch
	  *int dir = HAL_GPIO_ReadPin(Winch_Direction_GPIO_Port, Winch_Direction_Pin) == GPIO_PIN_SET;
	  *int run = HAL_GPIO_ReadPin(Winch_State_GPIO_Port, Winch_State_Pin) == GPIO_PIN_SET;
	  *Winch_Update(run,dir);
	  */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* Safety: no good line for 250 ms -> stop thrusters and winches.
	     The servo stays where it is. (Starts in failsafe until the first
	     good packet, since lastPacketTick starts at 0.) */
	  if (HAL_GetTick() - lastPacketTick > FAILSAFE_MS)
	  {
		  leftPulse  = ST;
		  rightPulse = ST;
		  winch1Cmd  = 0;
		  winch2Cmd  = 0;
	  }

	  setThrusters(leftPulse, rightPulse);
	  setServos(servoPulse);
	  /* winch1Cmd / winch2Cmd are saved but not used yet: the winch code
	     above is still commented out by the team. */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Right_High_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Left_High_Pin|Right_Low_Pin|Left_Low_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Right_High_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Right_High_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_High_Pin Right_Low_Pin Left_Low_Pin */
  GPIO_InitStruct.Pin = Left_High_Pin|Right_Low_Pin|Left_Low_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
