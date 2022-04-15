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
 *                        opensource.org/licenses/BSD-3-Clause
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

HCD_HandleTypeDef hhcd_USB_OTG_FS;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
static void MX_IWDG_Init(void);
static void MX_WWDG_Init(void);
static void MX_RTC_Init(void);
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
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	MX_I2C1_Init();
	MX_USB_OTG_FS_HCD_Init();
	//MX_IWDG_Init();
	//MX_WWDG_Init();
	MX_RTC_Init();

	// MIRAR PERQUE FA EL RESET
	reset_cause_t reset_cause = reset_cause_get();
	/* USER CODE BEGIN 2 */
	/* USER CODE BEGIN Init */

	// INITIALIZE SENSORS
	initsensors(&hi2c1);

	// RTC TEST
		RTC_TimeTypeDef sTime = { 0 };
		RTC_DateTypeDef sDate = { 0 };
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		// Save in RTC_TIME_ADDR
		RTC_Time(&sTime, &sDate);

	uint8_t currentState, exit_low, nominal, low, critical;
	Write_Flash(CURRENT_STATE_ADDR, INIT, 1);

	Read_Flash(CURRENT_STATE_ADDR, &currentState, sizeof(currentState));

	// Update the battery thresholds
	Write_Flash(NOMINAL_ADDR, 90, sizeof(90));
	Read_Flash(NOMINAL_ADDR, &nominal, sizeof(nominal));
	Write_Flash(LOW_ADDR, 85, sizeof(85));
	Read_Flash(LOW_ADDR, &low, sizeof(low));
	Write_Flash(CRITICAL_ADDR, 80, sizeof(80));
	Read_Flash(CRITICAL_ADDR, &critical, sizeof(critical));



	//Només les notis que farem cas amb un |

	// Signals related to NOTIFICATIONS
	uint32_t signal_received,signal_to_wait;
	uint32_t settime;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//system_state(&hi2c1);

		//Look for a new notification
		if (xTaskNotifyWait(0, signal_to_wait, &signal_received,
				portMAX_DELAY) == pdTRUE) {
			if (signal_received & NOMINAL_NOTI) {
				//If GS changed the NOMINAL thereshold
				Read_Flash(NOMINAL_ADDR, &nominal, sizeof(nominal));
			}
			if (signal_received & LOW_NOTI) {
				//If GS changed the LOW thereshold
				Read_Flash(LOW_ADDR, &low, sizeof(low));
			}
			if (signal_received & CRITICAL_NOTI) {
				//If GS changed the CRITICAL thereshold
				Read_Flash(CRITICAL_ADDR, &critical, sizeof(critical));
			}
			if (signal_received & SETTIME_NOTI) {
				//If GS send TIME
				Read_Flash(SET_TIME_ADDR, &settime, sizeof(critical));
			}
			if (signal_received & GS_NOTI) {
				//If contact region with GS -> NOTIFY COMMS
				xTaskNotify("Task COMMS", GS_NOTI, eSetBits);
			}
		}

		switch (currentState) {

		case CHECK:
			/* State that periodically checks the satellite general state (batteries,
			 * temperatures, voltages...
			 * From this state the satellite can go to contingency states
			 * if systemstate() returns a value different than 0 */

			// It is only done the first time we enter to IDLE
			// We send notifications to COMMS, ADCS tasks to start tunning (IDLE_State_noti)
			if (system_state(&hi2c1,nominal,low,critical) > 0) {
				currentState = CONTINGENCY;
				// Actualitzem l'estat només si canviem d'estat
				Write_Flash(PREVIOUS_STATE_ADDR, CHECK, 1);
				Write_Flash(CURRENT_STATE_ADDR, currentState,
						sizeof(currentState));
			} else {
				sensorReadings(&hi2c1); /*Updates the values of temperatures, voltages and currents*/
			}
			break;

//		case COMMS:	// This might refer ONLY refer to TX!!!
//			// Creem un fil on ens comunicarem simultaneament amb el GS
//			// configuration();
//			// pthread_create(&thread_comms, NULL, stateMachine(), NULL);
//			xTaskCreate("nomdelafuncio", 'nomdelatasca=commstask', 'StackDepth = 100', '', 'Priority = 0', pxCreatedTask)
//			// check if the picture or spectrogram has to be sent and send it if needed
//			if(system_state(&hi2c1)>0) currentState = CONTINGENCY;
//			else if(comms_state);
//				if(telecommand()) // function that receives orders from "COMMS" si true --> tornem a IDLE
//			// Si no rebem cap ordre, enviem telemetria -->
//					process_telecommand(SENDTELEMETRY,0);  //Aviso a COMMS que envii telemetry
//
//			// Una vegada ens hem comunicat amb COMMS, tornem a IDLE
//			// comms_state = false;
//			// No hauriem de fer un write?
//			Write_Flash(COMMS_STATE_ADDR, FALSE, 1);
//			currentState = IDLE;
//			Write_Flash(PREVIOUS_STATE_ADDR, COMMS, 1);
//			break;
//		case PAYLOAD:
//			if(system_state(&hi2c1)>0) currentState = CONTINGENCY;
//			/* The idea of this state is to start a new thread which starts the following function:
//			 * make the camera point to Earth (ADCS), when it reaches the final position
//			 * wait until it's time to take the photo
//			 * Meanwhile the main thread returns to IDLE, so it can't continue checking the
//			 * batteries, temperature, etc */
//
//			// Create a secondary thread (antenna_pointing + waits to takePhoto) -> vTaskPayload
//			// Main thread --> Back to IDLE
//			Write_Flash(PAYLOAD_STATE_ADDR, FALSE, 1);
//			currentState = IDLE;
//			Write_Flash(PREVIOUS_STATE_ADDR, PAYLOAD, 1);
//			break;

		case CONTINGENCY:
			/*Turn STM32 to Stop Mode or Standby Mode
			 *Loop to check at what batterylevel are we
			 *Out of CONTINGENCY State when batterylevel is NOMINAL
			 */
			// TODO: LOW POWER RUN MODE (De momento podriem fer un SleepMode)
			// Avisar a COMMS que entrem en CONTINGENCY (enviar paquet de telemetria)
			// S'envia cada vegada que rebem un EXITLOWPOWER_NOTI i la bateria no millora
			xTaskNotify("Task COMMS", CONTINGENCY_NOTI, eSetBits);

			// Esperem que COMMS rebi telecommand de la GS (EXITLOWPOWER_NOTI)
			if (xTaskNotifyWait(0, signal_to_wait, &signal_received,
					portMAX_DELAY) == pdTRUE) {
				if (signal_received & EXITLOWPOWER_NOTI) {
					// Mirem el nivell de bateria
					// Si ha empitjorat
					if (system_state(&hi2c1,nominal,low,critical) > 1) {
						currentState = SUNSAFE;
						// Actualitzem l'estat només si canviem d'estat
						Write_Flash(PREVIOUS_STATE_ADDR, CONTINGENCY, 1);
						Write_Flash(CURRENT_STATE_ADDR, currentState,
								sizeof(currentState));

						// Si ha millorat
					} else if (system_state(&hi2c1,nominal,low,critical) == 0) {
						/*Return to Run Mode*/
						currentState = CHECK;
						// Actualitzem l'estat només si canviem d'estat
						Write_Flash(PREVIOUS_STATE_ADDR, CONTINGENCY, 1);
						Write_Flash(CURRENT_STATE_ADDR, currentState,
								sizeof(currentState));
					}
				}
			}

			// Una opció és fer reset total del satelit quan surti de contingency

			break;

		case SUNSAFE:
			Write_Flash(PREVIOUS_STATE_ADDR, SUNSAFE, 1);
			// Entrem en el SleepMode
			// HAL_PWR_EnterSLEEPMode();

			// IWDG initialize
			// HAL_Delay(30000);
			// Refresh IWDG to avoid reseting the system
			HAL_IWDG_Refresh(&hiwdg);
			// IWDG ens permet sortir del SleepMode automaticament
			// Mirem si ha millorat o empitjorat la bateria
			// system_state = 3 --> battery level < CRITICAL
			if (system_state(&hi2c1,nominal,low,critical) == 3) {
				currentState = SURVIVAL;
				// Actualitzem l'estat només si canviem d'estat
				Write_Flash(PREVIOUS_STATE_ADDR, SUNSAFE, 1);
				Write_Flash(CURRENT_STATE_ADDR, currentState,
						sizeof(currentState));

				// system_state = 1 --> LOW < battery level < NOMINAL
			} else if (system_state(&hi2c1,nominal,low,critical) == 1) {
				/*Return to Run Mode*/
				currentState = CONTINGENCY;
				// Actualitzem l'estat només si canviem d'estat
				Write_Flash(PREVIOUS_STATE_ADDR, SUNSAFE, 1);
				Write_Flash(CURRENT_STATE_ADDR, currentState,
						sizeof(currentState));
			}
			break;

		case SURVIVAL:
			// Anar a mode Low Power Sleep Mode
			// Necessitem el IDWD
			// Refresh IWDG to avoid reseting the system
			HAL_IWDG_Refresh(&hiwdg);
			// Només podem anar a CONTINGENCY
			if (system_state(&hi2c1,nominal,low,critical) == 1) {
				currentState = CONTINGENCY;
				// Actualitzem l'estat només si canviem d'estat
				Write_Flash(PREVIOUS_STATE_ADDR, SURVIVAL, 1);
				Write_Flash(CURRENT_STATE_ADDR, currentState,
						sizeof(currentState));
			}

			break;

		case INIT:
			init(&hi2c1);
			Write_Flash(PREVIOUS_STATE_ADDR, INIT, 1);
			Read_Flash(CURRENT_STATE_ADDR, &currentState, sizeof(currentState));
			// Wake up COMMS and ADCS tasks
			xTaskNotify("Task ADCS", DONEPHOTO_NOTI, eSetBits);
			xTaskNotify("Task COMMS", DONEPHOTO_NOTI, eSetBits);
			// Send notification to ADCS --> DETUMBLING_NOTI
			//xTaskNotify("Task ADCS", DETUMBLING_NOTI, eSetBits);

			break;
			/*If we reach this state something has gone wrong*/
		default:
			/*REBOOT THE SYSTEM*/
			break;
		}

		/*Start a TIMER*/

//	    return 0;
		//todo variable que conti ticks rellotge per fer reset
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 17;
	sTime.Minutes = 40;
	sTime.Seconds = 0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
	sDate.Month = RTC_MONTH_MARCH;
	sDate.Date = 17;
	sDate.Year = 0;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_HCD_Init(void) {

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hhcd_USB_OTG_FS.Init.Host_channels = 8;
	hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
	hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
	hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/**
 * @brief WWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_WWDG_Init(void) {

	/* USER CODE BEGIN WWDG_Init 0 */

	/* USER CODE END WWDG_Init 0 */

	/* USER CODE BEGIN WWDG_Init 1 */

	/* USER CODE END WWDG_Init 1 */
	hwwdg.Instance = WWDG;
	hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
	hwwdg.Init.Window = 64;
	hwwdg.Init.Counter = 64;
	hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
	if (HAL_WWDG_Init(&hwwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN WWDG_Init 2 */

	/* USER CODE END WWDG_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
