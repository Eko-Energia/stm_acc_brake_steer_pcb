/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_ADC2_MAX_VALUE 1023 // Maksymalna wartość z ADC2_VAL[2] odpowiadająca 100%
#define SENSOR_ADC1_MAX_VALUE 1023 //Max value read from all ADC1 channels
#define NUM_SAMPLES_FOR_STEER_AVG 1 // Liczba próbek do uśrednienia dla czujnika skrętu
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC1_VAL[3];
uint16_t ADC2_VAL[3];

uint8_t TxData[8];
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailBox;

volatile int isADC1finished = 0;
volatile int isADC2finished = 0;

// Zmienne do uśredniania dla czujnika skrętu (np. z ADC1_VAL[2])
uint32_t steer_adc_sum = 0;
uint16_t steer_sample_count = 0;
uint8_t steer_percentage_to_send = 0; // Ostatnia obliczona wartość procentowa skrętu

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    isADC1finished = 1;
  }
  if (hadc->Instance == ADC2)
  {
    isADC2finished = 1;
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
  // Ta sekcja powinna być pusta. Inicjalizacja zmiennych globalnych odbywa się
  // automatycznie (na 0, jeśli nie podano inaczej) lub w miejscu ich deklaracji.
  // Dynamiczne przypisania muszą być w pętli lub po inicjalizacji peryferiów.
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
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc2, ADC2_VAL, 3);

  HAL_ADC_Start_DMA(&hadc1, ADC1_VAL, 3);



  if ( HAL_CAN_Start(&hcan) != HAL_OK)
  {
  Error_Handler();
  }

  TxHeader.StdId = 0x123; //id ramki
  TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_DATA oznacza że nasza ramka będzie przenosić dane, mogłoby być jeszcze CAN_RTR_REMOTE wtedy ramka nie przenosi danych
  // tylko służy do żądania danych od innego węzła
  TxHeader.IDE = CAN_ID_STD; //określa czy id jest normalne czy extended

  TxHeader.DLC = 6; // określa ilość kontenerów w wiadomości
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // Ogólne miganie diodą "życia"
    HAL_Delay(100); // Ogólne opóźnienie pętli

    // Sprawdź, czy dane z obu ADC są gotowe
    if (isADC1finished && isADC2finished)
    {
      isADC1finished = 0; // Wyzeruj flagi
      isADC2finished = 0;

      // --- Przetwarzanie danych z ADC1 dla kierownicy ---

      uint16_t current_steer_val = ADC1_VAL[2];
      uint8_t steer_percentage_to_send;


      if (current_steer_val >= SENSOR_ADC1_MAX_VALUE) {
			steer_percentage_to_send = 100;
      }
      else {
    	uint32_t temp_calc_steer = ((uint32_t)current_steer_val * 100) + (SENSOR_ADC1_MAX_VALUE / 2);
		steer_percentage_to_send = (uint8_t)(temp_calc_steer / SENSOR_ADC1_MAX_VALUE);
		}
      if (steer_percentage_to_send > 100) {
		 steer_percentage_to_send = 100;
		}

      // --- Przetwarzanie danych z ADC1 dla pedału gazu ---

		uint16_t current_accel_val1 = ADC1_VAL[0];

		uint8_t accel_1_percentage_to_send;


		if (current_accel_val1 >= SENSOR_ADC1_MAX_VALUE) {
			accel_1_percentage_to_send = 100;
		}
		else {
		uint32_t temp_calc_accel_1 = ((uint32_t)current_accel_val1 * 100) + (SENSOR_ADC1_MAX_VALUE / 2);
		accel_1_percentage_to_send = (uint8_t)(temp_calc_accel_1 / SENSOR_ADC1_MAX_VALUE);
		}
		if (accel_1_percentage_to_send > 100) {
			accel_1_percentage_to_send = 100;
		}

		// --- Przetwarzanie danych z ADC2 dla pedału gazu ---


		uint16_t current_accel_val2 = ADC2_VAL[0];
		uint8_t accel_2_percentage_to_send;


		if (current_accel_val2 >= SENSOR_ADC2_MAX_VALUE) {
			accel_2_percentage_to_send = 100;
		}
		else {
		uint32_t temp_calc_accel_2 = ((uint32_t)current_accel_val2 * 100) + (SENSOR_ADC2_MAX_VALUE / 2);
		accel_2_percentage_to_send = (uint8_t)(temp_calc_accel_2 / SENSOR_ADC2_MAX_VALUE);
		}
		if (accel_2_percentage_to_send > 100) {
			accel_2_percentage_to_send = 100;
		}


		//--- Przetwarzanie danych z ADC2 dla pedału hamulca - hall ---

		  uint16_t current_brake_adc_val = ADC2_VAL[2];
		  uint8_t brake_hall_percentage_to_send;

		  if (current_brake_adc_val >= SENSOR_ADC2_MAX_VALUE) {
				  brake_hall_percentage_to_send = 100;
		  } else {
			  uint32_t temp_calc_brake = ((uint32_t)current_brake_adc_val * 100) + (SENSOR_ADC2_MAX_VALUE / 2);
			  brake_hall_percentage_to_send = (uint8_t)(temp_calc_brake / SENSOR_ADC2_MAX_VALUE);
		  }
		  if (brake_hall_percentage_to_send > 100) {
			  brake_hall_percentage_to_send = 100;
		  }

      //--- Przetwarzanie danych z ADC2 dla pedału hamulca - tłoczki ---

	  uint16_t current_brake_piston_adc_val_1 = ADC1_VAL[1]; // Odczyt z ADC2 (wartość 6-bitowa, 0-63)
	  uint8_t brake_piston_1_percentage_to_send;

	  if (current_brake_piston_adc_val_1 >= SENSOR_ADC1_MAX_VALUE) {
		  brake_piston_1_percentage_to_send = 100;
		} else {
			uint32_t temp_calc_brake_piston_1 = ((uint32_t)current_brake_piston_adc_val_1 * 100) + (SENSOR_ADC1_MAX_VALUE / 2);
			brake_piston_1_percentage_to_send = (uint8_t)(temp_calc_brake_piston_1 / SENSOR_ADC1_MAX_VALUE);
		}
		if (brake_piston_1_percentage_to_send > 100) {
			brake_piston_1_percentage_to_send = 100;
		}
      //--- Przetwarzanie danych z ADC2 dla pedału hamulca - tłoczki ---

      uint16_t current_brake_piston_adc_val_2 = ADC2_VAL[1]; // Odczyt z ADC2 (wartość 6-bitowa, 0-63)
	  uint8_t brake_piston_2_percentage_to_send;

	  if (current_brake_piston_adc_val_2 >= SENSOR_ADC2_MAX_VALUE) {
		  brake_piston_2_percentage_to_send = 100;
		} else {
			uint32_t temp_calc_brake_piston_2 = ((uint32_t)current_brake_piston_adc_val_2 * 100) + (SENSOR_ADC2_MAX_VALUE / 2);
			brake_piston_2_percentage_to_send = (uint8_t)(temp_calc_brake_piston_2 / SENSOR_ADC2_MAX_VALUE);
		}
		if (brake_piston_2_percentage_to_send > 100) {
			brake_piston_2_percentage_to_send = 100;
		}



      // --- Przygotowanie i wysłanie ramki CAN ---
      TxData[0] = steer_percentage_to_send;
      TxData[1] = brake_piston_1_percentage_to_send;
      TxData[2] = brake_piston_2_percentage_to_send;
      TxData[3] = brake_hall_percentage_to_send;
      TxData[4] = accel_1_percentage_to_send;
      TxData[5] = accel_2_percentage_to_send;


      // TxHeader.DLC jest już ustawione na 3

      if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox) != HAL_OK)
      {
        Error_Handler();
      }
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(200);
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
