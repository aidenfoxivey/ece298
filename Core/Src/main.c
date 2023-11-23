/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 Beanbean and Latte.
 * All rights reserved.
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

/* USER CODE BEGIN PV */
uint8_t cmd_dist = 0x55;

volatile uint8_t us100_Rx_flag = 0;
volatile uint8_t blue_pb_flag = 0x00;
uint8_t us100_buffer[2] = { 0 };
uint8_t pwm_buffer[2] = { 0 };
uint8_t msg_buffer[128] = { 0 };

volatile uint16_t distance = 0;
volatile uint8_t hour = 0;

volatile uint8_t distance_percent = 0;

/* PWM */
volatile uint8_t inlet_pwm_mode = 0;
volatile uint8_t zone_1_pwm_mode = 0;
volatile uint8_t zone_2_pwm_mode = 0;
volatile uint8_t zone_3_pwm_mode = 0;

/* TIME */
volatile uint8_t wall_clock_time = 0;
volatile uint8_t inlet_start = 0;
volatile uint8_t inlet_end = 0;
volatile uint8_t zone_1_start = 0;
volatile uint8_t zone_1_end = 0;
volatile uint8_t zone_2_start = 0;
volatile uint8_t zone_2_end = 0;
volatile uint8_t zone_3_start = 0;
volatile uint8_t zone_3_end = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* ****** SETUP MODE BEGIN ****** */
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET); /* green LED must be off during setup */

		sprintf((char*) msg_buffer, "SETUP MODE\r\n");
		HAL_UART_Transmit(&huart6, msg_buffer, strlen((char*) msg_buffer),
				1000);

		/* PWM SELECT */
		query_pwm(
				"INLET\t Choose 0 for manual, 1 for 60% PWM, 2 for 80% PWM, or 3 for 99% PWM\r\n",
				&inlet_pwm_mode);
		query_pwm(
				"ZONE 1\t Choose 0 for manual, 1 for 60% PWM, 2 for 80% PWM, or 3 for 99% PWM\r\n",
				&zone_1_pwm_mode);
		query_pwm(
				"ZONE 2\t Choose 0 for manual, 1 for 60% PWM, 2 for 80% PWM, or 3 for 99% PWM\r\n",
				&zone_2_pwm_mode);
		query_pwm(
				"ZONE 3\t Choose 0 for manual, 1 for 60% PWM, 2 for 80% PWM, or 3 for 99% PWM\r\n",
				&zone_3_pwm_mode);

		/* TIME SELECT */
		query_time("CURRENT\t WALL CLOCK TIME (0-23):\r\n", &wall_clock_time);

		query_time("INLET\t WALL CLOCK START TIME (0-23):\r\n", &inlet_start);
		query_time("INLET\t WALL CLOCK STOP TIME (0-23):\r\n", &inlet_stop);

		query_time("ZONE 1\t WALL CLOCK START TIME (0-23):\r\n", &zone_1_start);
		query_time("ZONE 1\t WALL CLOCK STOP TIME (0-23):\r\n", &zone_1_end);

		query_time("ZONE 2\t WALL CLOCK START TIME (0-23):\r\n", &zone_2_start);
		query_time("ZONE 2\t WALL CLOCK STOP TIME (0-23):\r\n", &zone_2_end);

		query_time("ZONE 3\t WALL CLOCK START TIME (0-23):\r\n", &zone_3_start);
		query_time("ZONE 3\t WALL CLOCK STOP TIME (0-23):\r\n", &zone_3_end);
		/* ****** SETUP MODE END ****** */

		while (blue_pb_flag == 0x00) {
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
			HAL_Delay(250);
		}

		/* RUN MODE BEGIN */
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET); /* start the green LED */
		sprintf((char*) msg_buffer,
				"Wall-Clock Time,  Zone/Inlet, Motor %RPM, Water Reservoir Depth(%)\r\n");
		HAL_UART_Transmit(&huart6, msg_buffer, strlen((char*) msg_buffer),
				1000);

		while (1) {
			HAL_UART_Receive_IT(&huart1, us100_buffer, 2);
			HAL_UART_Transmit(&huart1, &cmd_dist, 1, 500);
			while (us100_Rx_flag == 0x00) {
			};
			distance = us100_buffer[1] << 8 | us100_buffer[0];
		}
		/* RUN MODE END */

		sprintf((char*) msg_buffer, "RESERVOIR IS EMPTY\r\n");
		HAL_UART_Transmit(&huart6, msg_buffer, strlen((char*) msg_buffer),
				1000);

		HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET); /* turn off green LED */
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, BLU_Pin | GRN_Pin | RED_Pin,
				GPIO_PIN_RESET); /* reset pins */

		/* write LEDs in RGB pattern */
		while (1) {
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, RED_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, RED_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOA, LD2_Pin, GRN_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, GRN_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOA, LD2_Pin, BLU_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, BLU_Pin, GPIO_PIN_RESET);
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/* recount the correct colours for each zone */
void run_zone(uint8_t time) {
	if (time > inlet_start && time < inlet_end) {
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, GRN_Pin, GPIO_PIN_SET);
	}

	else if (time > zone_1_start && time < zone_1_end) {
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, BLU_Pin, GPIO_PIN_SET);
	}

	else if (time > zone_2_start && time < zone_2_end) {
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, GRN_Pin, GPIO_PIN_SET);
	}

	else if (time > zone_3_start && time < zone_3_end) {

	}

	else {
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, BLU_Pin | GRN_Pin | RED_Pin,
				GPIO_PIN_RESET); /* reset pins */
	}
}

void query_distance(uint8_t* us100_buffer) {
	HAL_UART_Receive_IT(&huart6, us100_buffer, 2);
	return us100_buffer[1] << 8 + us100_buffer[0];
}

void query_time(char *msg, uint8_t *clock_time) {
	uint8_t msg_buffer[128] = { 0 };
	uint8_t pwm_buffer[2] = { 0 };

	sprintf((char*) msg_buffer, msg);
	HAL_UART_Transmit(&huart6, msg_buffer, strlen((char*) msg_buffer), 1000);
	HAL_UART_Receive_IT(&huart6, pwm_buffer, 2);

	*clock_time = array_to_time(pwm_buffer);
}

void query_pwm(char *msg, uint8_t *pwm_mode) {
	uint8_t msg_buffer[128] = { 0 };
	uint8_t pwm_buffer[2] = { 0 };

	sprintf((char*) msg_buffer, msg);
	HAL_UART_Transmit(&huart6, msg_buffer, strlen((char*) msg_buffer), 1000);
	HAL_UART_Receive_IT(&huart6, pwm_buffer, 2);

	*pwm_mode = pwm_buffer[0];
}

void HAL_UART_RxCpltCallback(UART_HandeTypeDef *huart) {
	if (huart->Instance == USART1) {
		us100_Rx_flag = 0x01;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == the_blue_one) {
		blue_pb_flag = 0x01;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5) {
		/* assumed to be a variable for seconds */
		clock_secs++;
	}
}

uint8_t array_to_time(uint8_t values[2]) {
	return values[1] * 10 + values[0];
}

void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B) {
	uint8_t DIGITA_VAL = 0x0F & DIGIT_A; // mask off higher 4 bits
	int Abit0 = (DIGITA_VAL) & 1; // extract Abit0 of the 4-bit value
	int Abit1 = (DIGITA_VAL >> 1) & 1; // extract Abit1 of the 4-bit value
	int Abit2 = (DIGITA_VAL >> 2) & 1; // extract Abit2 of the 4-bit value
	int Abit3 = (DIGITA_VAL >> 3) & 1; // extract Abit3 of the 4-bit value

	uint8_t DIGITB_VAL = 0x0F & DIGIT_B; // mask off higher 4 bits
	int Bbit0 = (DIGITB_VAL) & 1; // extract Bbit0 of the 4-bit value
	int Bbit1 = (DIGITB_VAL >> 1) & 1; // extract Bbit1 of the 4-bit value
	int Bbit2 = (DIGITB_VAL >> 2) & 1; // extract Bbit2 of the 4-bit value
	int Bbit3 = (DIGITB_VAL >> 3) & 1; // extract Bbit3 of the 4-bit value

	if (Abit0 == (0)) {
		HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);
	}
	if (Abit1 == (0)) {
		HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET);
	}
	if (Abit2 == (0)) {
		HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);
	}
	if (Abit3 == (0)) {
		HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_SET);
	}

	if (Bbit0 == (0)) {
		HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_SET);
	}
	if (Bbit1 == (0)) {
		HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_SET);
	}
	if (Bbit2 == (0)) {
		HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_SET);
	}
	if (Bbit3 == (0)) {
		HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_SET);
	}
}

void ADC_Select_CH(int CH);
void ADC_Select_CH(int CH) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	switch (CH) {
	case 0:
		sConfig.Channel = ADC_CHANNEL_0;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 1:
		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 2:
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_3;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 4:
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 5:
		sConfig.Channel = ADC_CHANNEL_5;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 6:
		sConfig.Channel = ADC_CHANNEL_6;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 7:
		sConfig.Channel = ADC_CHANNEL_7;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 8:
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 9:
		sConfig.Channel = ADC_CHANNEL_9;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 10:
		sConfig.Channel = ADC_CHANNEL_10;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 11:
		sConfig.Channel = ADC_CHANNEL_11;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 12:
		sConfig.Channel = ADC_CHANNEL_12;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 13:
		sConfig.Channel = ADC_CHANNEL_13;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 14:
		sConfig.Channel = ADC_CHANNEL_14;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	case 15:
		sConfig.Channel = ADC_CHANNEL_15;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		break;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
