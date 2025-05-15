/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define LD2_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_10

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_15


#define DUST_PWM_PERIOD  100 // from manufacture  (100ms)
#define K_FACTOR 1.0			//from manufacture
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/*~timer*/
static uint16_t vSchTask_Cnt = 0;
// TIM2 주기적 인터럽트 콜백
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		vSchTask_Cnt++;
		if(vSchTask_Cnt == 1000){
			vSchTask_Cnt = 0;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}

	}
} //timer

/* uart*/
char msg[60];// 온습도 메시지 저장 버퍼
char msg2[60];
char msg3[60];
//char msg4[50];
//char msg5[50];
//char msg6[50];
//char msg7[50];
char msg8[50];
// UART 메시지 전송 함수
void send_uart_msg(char *msg)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
/* dht11*/
/* DHT11 센서 관련 변수 */
uint8_t Rh_Byte1, Rh_Byte2, Temp_Byte1, Temp_Byte2;
uint16_t SUM, RH, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;
// GPIO 핀을 출력으로 설정
void set_pin_output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
// GPIO 핀을 입력으로 설정
void set_pin_input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
// 마이크로초 단위 딜레이 함수
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while(__HAL_TIM_GET_COUNTER(&htim2) <us);
}

/*	DHT11 start signal*/
/* DHT11 센서 시작 신호 전송 */
void dht11_start(void)
{
	set_pin_output(DHT11_PORT, DHT11_PIN);


	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN,0);


//	delay_us(18000); //wait for 18ms
	  HAL_Delay(18);


	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN,1);


	delay_us(20);
	set_pin_input(DHT11_PORT, DHT11_PIN);
}


/*	DHT11 response */
/* DHT11 센서 응답 확인 */
uint8_t dht11_check_response(void)
{
	uint8_t resp =0;
	delay_us(40);
	if(!(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)))
	{
		delay_us(80);
		if((HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)))
			resp =1;
		else
			resp = -1; //255
	}
	while(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN));

	return resp;
}
/* DHT11 데이터 읽기 */
uint8_t dht11_read(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        uint32_t timeout = 10000;
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && timeout--)
            ; //
        if (timeout == 0) return 0xFF; //

        delay_us(40);
        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
        {
            i &= ~(1 << (7 - j)); // 0 기록
        }
        else
        {
            i |= (1 << (7 - j));
            timeout = 10000;
            while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && timeout--)
                ;
            if (timeout == 0) return 0xFF;
        }
    }
    return i;
} // DHT11 sensor

/*dust sensor */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 =0;

float frequency =0;
float dutyCycle =0;

uint32_t pwm_start = 0;
uint32_t pwm_end = 0;
uint32_t pulse_width = 0;
/* 먼지 센서용 PWM 값 처리 */
HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) 
{
	/*if interrupt source is channel1*/
#if 0
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		TIM2->CNT =0;
	}
#elif 0  /* pin 1ea : check only two rising edge for period width(T)*/
		//but dust sensor formular need low state  time
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {

        if (pwm_start == 0)
        {
            pwm_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        }
        else
        {

            pwm_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (pwm_end > pwm_start)
            {
                pulse_width = pwm_end - pwm_start;
            }
            else
            {

                pulse_width = (htim->Init.Period - pwm_start) + pwm_end;
            }

            pwm_start = 0;
        }
    }
	/*if interrupt source is channel2*/
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) //
	{
		IC_Val2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
		dutyCycle= (float)IC_Val2/(float)IC_Val1 *100;
		frequency = 1000000/ (float)IC_Val1;
	}
#elif 0 /*pin 2ea : check rising edge and falling edge for low state time*/
    static uint32_t falling_edge =0;
    static uint32_t rising_edge =0;

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
    	rising_edge = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);

    }
    else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
    	falling_edge = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);

    	if(falling_edge> rising_edge)
    	{
    		pulse_width = falling_edge -rising_edge;
    		//tiem of low->high , this is  low state duration
    	}
    	else
    	{
    		pulse_width = (htim->Init.Period - rising_edge) + falling_edge;
    	}



    }
#elif 1 /*pin 1ea : check rising, falling edge*/
    static uint32_t last_capture =0;
    static uint8_t is_high_edge =0;

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
    	uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
    	if(is_high_edge ==0 )
    	{
    		/*check falling edge*/
    		is_high_edge =1;
    		if(current_capture >= last_capture)
    		{
    			pulse_width = current_capture - last_capture; //low ?��?��
    		}
    		else
    		{
    			//timer overflow process
    			pulse_width = (htim->Init.Period - last_capture) + current_capture;
    		}
//    		snprintf(msg8, sizeof(msg8), "Low Duration: %d us\r\n", (int)pulse_width);
//    		send_uart_msg(msg8);
    	}
    	else
    	{
    		/*check  uprising edge*/
    		is_high_edge =0;
    		//compare current rising edge with next falling edge
    	}
    	last_capture = current_capture;
    }
#endif


}
/* 먼지 농도 계산 함수 */
float calculate_dust_density(uint32_t low_duration, uint32_t period, float k_factor)
{
	float ratio = (float)low_duration /  (float)period;
	// 200/100ms
	float dust_density = k_factor * ratio;

	return dust_density;
}
//dust sensor
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);




   //send_uart_msg("init1\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  dht11_start();// DHT11 센서 시작


	  Presence = dht11_check_response();// DHT11 응답 확인


		Rh_Byte1 = dht11_read();// 습도 데이터 읽기
		Rh_Byte2 = dht11_read();
		Temp_Byte1 = dht11_read();// 온도 데이터 읽기
		Temp_Byte2 = dht11_read();
		SUM = dht11_read();// 데이터 합계 읽기

		RH = (int)Rh_Byte1;
		TEMP = (int)Temp_Byte1;

		// UART로 데이터 전송
		// snprintf(msg, sizeof(msg), "RH = %d %%\r\n", RH);
		snprintf(msg, sizeof(msg), "%d,", RH);
		send_uart_msg(msg);
		//snprintf(msg2, sizeof(msg2), "temp = %d `C%%\r\n", TEMP);
		snprintf(msg2, sizeof(msg2), "%d,", TEMP);
		send_uart_msg(msg2);



		// 먼지 농도 계산 및 UART 전송
		/*pwm ic*/
        float dust_concentration_g_m3 = calculate_dust_density(pulse_width, DUST_PWM_PERIOD, K_FACTOR);
        float dust_concentration_ug_m3 = dust_concentration_g_m3 ;// convert g/m^3 to ug/m^3
        //snprintf(msg3, sizeof(msg3), "Dust Density : %.2f ug/m^3\r\n", dust_concentration_ug_m3);
        snprintf(msg3, sizeof(msg3), "%.2f\r\n", dust_concentration_ug_m3);
        send_uart_msg(msg3);

        HAL_Delay(2000);





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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
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
