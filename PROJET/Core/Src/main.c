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
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
void initialisation();
void affichemod();
void fonctionmod();
void chronometre();
void arreter_chronometre();
void switchmode();

void adcfunction();
void affiche_num();
void minuteur();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int mode = 1;
int stopchrono = 1;

int analogValue=0;
int analogValueMin=0;

volatile int valeur = 0;
volatile int valeurMin = 0;

int dizaines =0;
int unites =0;
int dizainesminutes = 0;
int minutes =0;
int valide = 0;
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
  MX_SPI1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

	affichemod();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		fonctionmod();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 2;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
  AnalogWDGConfig.ITMode = DISABLE;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  if (HAL_ADC_AnalogWDGConfig(&hadc, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_384CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim3.Init.Prescaler = 32000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTNCarte_Pin */
  GPIO_InitStruct.Pin = BTNCarte_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTNCarte_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN4_Pin BTN3_Pin */
  GPIO_InitStruct.Pin = BTN4_Pin|BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		//__io_putchar(*ptr++);
		ITM_SendChar(*ptr++);
	}
	return len;
}





void initialisation(uint32_t MyDelay) {
	MAX7219_Clear();
	MAX7219_Init();
	MAX7219_DisplayChar(1,'1', 0); // Pas de point décimal
	MAX7219_DisplayChar(2,'2', 1); // Avec point décimal
	MAX7219_DisplayChar(3,'3', 0); // Pas de point décimal
	MAX7219_DisplayChar(4,'4', 0); // Pas de point décimal
	HAL_Delay(MyDelay);
	MAX7219_Clear();
}






void affichemod(){
	MAX7219_Clear();
	MAX7219_Init();
	MAX7219_DisplayChar(1,'S', 0); // Pas de point décimal
	MAX7219_DisplayChar(2,'E', 0); // Avec point décimal
	MAX7219_DisplayChar(3,'T', 1); // Pas de point décimal
	switchmode();

}






void fonctionmod(){
	switch(mode){
	case 1:
		if (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_RESET) {
			printf("Appui sur le bouton 1 \n");
			chronometre();
		}
		break;

	case 2:
		if (valide == 0){
			HAL_Delay(1000);
			adcfunction();
			affiche_num();
		}
		if (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == GPIO_PIN_RESET){
			HAL_ADC_Stop(&hadc);
			valide = 1;
			printf("Temps validé !\n");

			minuteur();
		}

		break;
	}

}







void chronometre() {
	MAX7219_Clear();
	MAX7219_Init();
	stopchrono = 1 ;

	uint32_t start_time = HAL_GetTick(); // Temps de départ en millisecondes

	uint32_t minutes = 0; // Initialisation des minutes à 0
	uint32_t seconds = 0; // Initialisation des secondes à 0
	while ((minutes < 99)&&(stopchrono == 1)) { // Tant que moins de 99 minutes se sont écoulées
		uint32_t elapsed_time = HAL_GetTick() - start_time; // Temps écoulé depuis le début du chronomètre

		// Calculez les minutes et les secondes
		minutes = (elapsed_time / (1000 * 60)) % 100; // Limiter les minutes à 99
		seconds = (elapsed_time / 1000) % 60;

		// Affichez les valeurs calculées sur les afficheurs 7 segments
		MAX7219_DisplayChar(1, minutes / 10 + '0', 0); // Affiche les dizaines de minutes
		MAX7219_DisplayChar(2, minutes % 10 + '0', 1); // Affiche les minutes
		MAX7219_DisplayChar(3, seconds / 10 + '0', 0); // Affiche les dizaines de secondes
		MAX7219_DisplayChar(4, seconds % 10 + '0', 0); // Affiche les secondes

		HAL_Delay(1000); // Attendez une seconde avant de mettre à jour l'affichage

		MAX7219_Clear(); // Effacez l'affichage une fois que 99 minutes se sont écoulées
		if (HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == GPIO_PIN_RESET){
			arreter_chronometre();
		}
	}
}







void arreter_chronometre() {
	// Arrêter la mise à jour de l'affichage
	stopchrono = 0;
	printf("stopchrono = %d\n",stopchrono);
	MAX7219_DisplayChar(1, '0', 0);
	MAX7219_DisplayChar(2, '0', 1);
	MAX7219_DisplayChar(3, '0', 0);
	MAX7219_DisplayChar(4, '0', 0);

}






void switchmode(){
	switch(mode){
	case 1:
		MAX7219_DisplayChar(4,'1', 0); // Pas de point décimal
		break;
	case 2:
		MAX7219_DisplayChar(4,'2', 0);
		break;
	case 3:
		MAX7219_DisplayChar(4,'3', 0); // Pas de point décimal
		break;
	case 4:
		MAX7219_DisplayChar(4,'4', 0);
		break;
	}
}

void adcfunction(){
	/*
	//Secondes
	HAL_ADC_Start(&hadc);
	ADC_ChannelConfTypeDef sConfig = {0}; // Réinitialisation de la structure sConfig
	sConfig.Channel = ADC_CHANNEL_0;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);
	HAL_ADC_PollForEvent(&hadc, ADC_AWD_EVENT, 1000);
	HAL_ADC_PollForConversion(&hadc, 1000);
	analogValue = HAL_ADC_GetValue(&hadc);
	valeur = analogValue / 68;

	HAL_ADC_Stop(&hadc);
	printf("ADC Value= %d\n", analogValue);
	printf("Valeur 60 : %d\n",valeur);
	HAL_Delay(100);
	*/

	// Minutes
	HAL_ADC_Start(&hadc);
	ADC_ChannelConfTypeDef sConfig = {0}; // Réinitialisation de la structure sConfig
	sConfig.Channel = ADC_CHANNEL_0; // Réaffectation du canal ADC
	HAL_ADC_ConfigChannel(&hadc, &sConfig);
	HAL_ADC_PollForEvent(&hadc, ADC_AWD_EVENT, 1000);
	HAL_ADC_PollForConversion(&hadc, 1000);
	analogValueMin = HAL_ADC_GetValue(&hadc);
	valeurMin = analogValueMin / 100;

	HAL_ADC_Stop(&hadc);
	printf("ADC ValueMin= %d\n", analogValueMin);
	printf("Valeur 100 : %d\n",valeurMin);
	HAL_Delay(100);

}

void affiche_num(){
	MAX7219_Clear();
	MAX7219_Init();

	dizainesminutes = valeurMin / 10;
	minutes = valeurMin % 10;
	dizaines = valeur / 10;
	unites = valeur % 10;

	MAX7219_DisplayChar(1, dizainesminutes + '0', 0);
	MAX7219_DisplayChar(2, minutes + '0', 0);
	MAX7219_DisplayChar(3, dizaines + '0', 0);
	MAX7219_DisplayChar(4, unites + '0', 0);
}

void decrementunites(){
	while (unites > 0){
		unites --;
		MAX7219_DisplayChar(4, unites + '0', 0);
		HAL_Delay(1000);
	}
}

void decrementdizaines(){
	while (dizaines > 0){
		decrementunites();
		dizaines --;
		unites = 9;
		MAX7219_DisplayChar(3, dizaines + '0', 0);
		MAX7219_DisplayChar(4, unites + '0', 0);
		HAL_Delay(1000);
		decrementunites();

	}
}

void minuteur(){
	while ((dizaines != 0) && (unites !=0)){
		decrementdizaines();
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	switch (GPIO_Pin){
	case BTNCarte_Pin :
		if (mode < 4){
			mode ++;
			printf("Mode : %d\n", mode);
		}
		else{
			mode = 1;
			printf("Mode : %d\n", mode);
		}
		affichemod();


		break;
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
