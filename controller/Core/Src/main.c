/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c - CONTROLLER
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart3;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
volatile uint8_t motion_pending = 0; //Generic motion indicator field for the waker and loop
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void enter_stop_mode(void); //Routine to sleep CPU
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Low-power waker callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MPU_INT_Pin) { //PB0
    motion_pending = 1; //Set motion indicator
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t lora_params[8], tx_buf[LoRa_MAX_DATA_SIZE], rx_buf[64];
  uint32_t cur_tick;
  HAL_StatusTypeDef status;
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
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  MX_I2C2_Init();
  memset(tx_buf, 0, LoRa_MAX_DATA_SIZE);
  memset(rx_buf, 0, 64);

  //Wait initially until the controller io transactions is available
  cur_tick = HAL_GetTick();
  while(((HAL_GetTick() - cur_tick) < 1000) && HAL_GPIO_ReadPin(GPIOB, LoRa_AUX_Pin) != GPIO_PIN_SET); //Wait until the AUX pin is set

  //If controller is busy and exceeds the timeout, blink the led then abort
  if(HAL_GPIO_ReadPin(GPIOB, LoRa_AUX_Pin) != GPIO_PIN_SET){
	  HAL_GPIO_WritePin(GPIOA, USER_LED_Pin, GPIO_PIN_SET);
	  Error_Handler();
  }
  else{
	  //Set the initial lora module mode [11 -> Sleep]
	  HAL_GPIO_WritePin(GPIOB, LoRa_M0_Pin, GPIO_PIN_SET); //M0
	  HAL_GPIO_WritePin(GPIOB, LoRa_M1_Pin, GPIO_PIN_SET); //M1
	  HAL_Delay(20);

	  //Wait until the controller is available
	  cur_tick = HAL_GetTick();
	  while(((HAL_GetTick() - cur_tick) < 100) && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) != GPIO_PIN_SET); //Wait until the AUX pin is set

	  //Clear buffer & send the initial option parameters
	  memset(lora_params, 0, 8);

	  //Save after shutdown (through command flash interface)
	  lora_params[0] = 0xC0;

	  //Broadcast transmission (NC address)
	  lora_params[1] = 0x00;
	  lora_params[2] = 0x00;

	  //UART PARITY: 8N1, BAUD: 9600bps, DATA RATE (SF): SF7 / 2.4k,
	  lora_params[3] = 0x1A; //0b00011010

	  //Channel settings: 868 MHz (862 + OFFSET->6)
	  lora_params[4] = 0x06;

	  //Options: Transparent Transmission, IO Push-Pull, Wireless wakeup: 250ms, FEC OFF, Power: 30 dBm
	  lora_params[5] = 0x40; //0b0100 0000;

	  //Send the option bytes (wait for the AUX pin after the transaction)
	  if((status = HAL_UART_Transmit(&huart3, lora_params, 6, 100)) == HAL_OK){
		  if((status = HAL_UART_Receive(&huart3, rx_buf, 6, 1000)) == HAL_OK){
			  HAL_Delay(50);
			  //Check if the params sent equal to the echo packet sent back
			  if(strncmp((const char*)rx_buf, (const char*)lora_params, 6) == 0){
				  //Set the normal mode and get ready for the operations
				  HAL_GPIO_WritePin(GPIOB, LoRa_M0_Pin, GPIO_PIN_RESET); //M0
				  HAL_GPIO_WritePin(GPIOB, LoRa_M1_Pin, GPIO_PIN_RESET); //M1
				  memset(rx_buf, 0, 64);
				  HAL_Delay(1);
				  int ret = mpu6050_init(&hi2c2);
				  if (ret != 0) Error_Handler();
				  HAL_Delay(1);
			  }
			  else{
				  HAL_GPIO_WritePin(GPIOA, USER_LED_Pin, GPIO_PIN_SET);
				  Error_Handler();
			  }
		  }
		  else{
			  HAL_GPIO_WritePin(GPIOA, USER_LED_Pin, GPIO_PIN_SET);
			  Error_Handler();
		  }
	  }
	  else{
		  HAL_GPIO_WritePin(GPIOA, USER_LED_Pin, GPIO_PIN_SET);
		  Error_Handler();
	  }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    // If there is no pending motion and no incoming request, go to STOP mode
    if (!motion_pending)
    {
      // Try to catch a request quickly; if none, sleep.
      if (HAL_UART_Receive(&huart3, rx_buf, 1, 50) != HAL_OK)
      {
        enter_stop_mode();
        continue; // after wake-up, loop again
      }
    }

    //Wait for the window request from the gateway and any motion
    if (HAL_UART_Receive(&huart3, rx_buf, 1, 500) == HAL_OK
    		&& rx_buf[0] == 0x01 && motion_pending)
    {
    	//Clear the motion flag
        motion_pending = 0;

        // Prepare response window, then send
        tx_buf[0] = 0x00;
        tx_buf[1] = 0x00;
        tx_buf[2] = 0x06;

        if (imu_send_window(&hi2c2, &huart3, tx_buf, LoRa_MAX_DATA_SIZE, 3, 96, 10) == 0)
        {
          HAL_GPIO_TogglePin(GPIOA, USER_LED_Pin);
        }
      }
      // After sending, go back to STOP quickly (next loop iteration will sleep)
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LoRa_M0_Pin|LoRa_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LoRa_M0_Pin LoRa_M1_Pin */
  GPIO_InitStruct.Pin = LoRa_M0_Pin|LoRa_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LoRa_AUX_Pin */
  GPIO_InitStruct.Pin = LoRa_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LoRa_AUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin  = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;	//Rising-edge interrupt
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;        	//IDLE low, PULSE high
  HAL_GPIO_Init(MPU_INT_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;

  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  // Analog filter Configuration
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  // Digital filter Configuration
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void enter_stop_mode(void)
{
  // Clear pending EXTI to avoid immediate wake-up
  __HAL_GPIO_EXTI_CLEAR_IT(MPU_INT_Pin);

  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  HAL_ResumeTick();

  //After STOP, clocks must be restored
  SystemClock_Config();

  //Re enable the interfaces being used
  MX_USART3_UART_Init();
  MX_I2C2_Init();
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
