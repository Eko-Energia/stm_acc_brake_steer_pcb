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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vehicle_types.h"
#include "can_bus.h"
#include "engine_control.h"
#include "vehicle_fsm.h"
#include "pedals_const_val.h"
#include "sensors.h"


/* @brief Used EKO drivers. */
#include "error_handler.h"
#include "can_driver.h"
#include "led_driver.h"
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

/** @brief Structure used to initialize CAN driver */
struct CAN_scheduledMsgList canScheduler = {0};

/** @brief Global Error Handler Object */
EH_HandleTypeDef heh;


struct LED statusLed = {
		.GPIO_Port = LED_RED_GPIO_Port,
		.GPIO_Pin = LED_RED_Pin
};

extern TIM_HandleTypeDef htim2;

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
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* @brief Timer 2 initialization. */

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	{
	Error_Handler();
	}

  /* @brief DMA initialization for ADC2. */

  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2_DMA_Buff, 3 * ADC_SAMPLES) != HAL_OK)
	{
	Error_Handler();
	}

  /* @brief DMA initialization for ADC1. */

  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_DMA_Buff, 3 * ADC_SAMPLES) != HAL_OK)
	{
	Error_Handler();
	}

	/* @brief Custom CAN filters initialization. (NOT FROM EKO CAN Driver)  */
	CAN_Custom_Init(&hcan);

	/* @brief Error handler initialization. */
	EH_init(&heh, &hcan, 64, &canScheduler);

	LED_ChangeState(&statusLed, LED_BLINK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

/*  @brief Main infinite loop.*/
  while (1)
  {
	  /* @brief Turning on CAN scheduler from EKO CAN Driver. */
	  CAN_HandleScheduled(&hcan, &canScheduler);

	  /* @brief Processing of CAN frames buffered in the RX interrupt. */
	  CAN_ProcessIncoming();

	  /* @brief ADC data processing.*/
	  Process_ADC_Buffers();

	  /* @brief Execute vehicle logic. */
	  stateActions();
  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


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


/**
  * @brief  Period Elapsed Callback (System Watchdog).
  * @details Executed by TIM2 interrupt.
  * Performs safety checks:
  * 1. **LED Status**: Indicates Charging status.
  * 2. **PRND Watchdog**: If no frame received for 3000ms -> Shift to neutral gear & Error LED.
  * 3. **Wheel Speed Watchdog**: If no frame received for 1000ms -> Shift to neutral gear & Error LED.
  * 3. **Jetson Watchdog**: If no frame received for 1000ms -> Reset Jetson Data & Flag.
  * * @param  htim Pointer to TIM handle.
  */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        uint32_t now = HAL_GetTick();

        /* @brief After 3 seconds from the last received PRND */
        uint32_t timeout = 3000;

        /* @brief After 1 second from the last received Wheel Speed */
        uint32_t timeoutWheelSpeed = 1000;
       
        // Checking rear left wheel speed (ID 0x6A1)
        bool isWheelSpeedRLTimeout = (now - Vehicle.WheelSpeed.LastMsgTickRL > timeoutWheelSpeed);
      
        // Checking rear right wheel speed (ID 0x681)
        bool isWheelSpeedRRTimeout = (now - Vehicle.WheelSpeed.LastMsgTickRR > timeoutWheelSpeed);
        
        /*
        bool isChargerTimeout = (now - Vehicle.Charger.LastMsgTick > timeout);

        // --- 1. WATCHDOG (Connection error) ---
        if (isChargerTimeout)
        {
            // Safety logic
            Vehicle.Charger.IsConnected = false;
            Vehicle.Charger.RawStatus = STOP_CHARGING;

            if (getEngineFlag() == 1 || getEngineFlag() == 0)
			{
				stopEngine();
				setEngineFlag(ENGINE_STOP_NEUTRAL);
			}

            // LED TOGGLE signaling error
            const uint32_t interval = 100;
            static uint32_t lastTick = 0;

            if (now - lastTick >= interval)
            {
                HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
                lastTick = now;
            }
        }
        */

        // na razie ramka prnd jest wysyłana onEvent
//        bool isPRNDTimeout = (now - Vehicle.PRND.LastMsgTick > timeout);
        bool isPRNDTimeout = 0;

		// --- 1. WATCHDOG (Connection error) ---
		if (isPRNDTimeout || isWheelSpeedRLTimeout || isWheelSpeedRRTimeout)
		{
      /*
      if (isPRNDTimeout) {
			  EH_report(&heh, 0x100, ERROR_SEVERITY_ERROR);
      }

      if (isWheelSpeedFLTimeout) {
        EH_report(&heh, 0x101, ERROR_SEVERITY_ERROR);
      }
      if (isWheelSpeedFRTimeout) {
        EH_report(&heh, 0x102, ERROR_SEVERITY_ERROR);
      }
      if (isWheelSpeedRLTimeout) {
        EH_report(&heh, 0x103, ERROR_SEVERITY_ERROR);
      }
      if (isWheelSpeedRRTimeout) {
        EH_report(&heh, 0x104, ERROR_SEVERITY_ERROR);
      }

			// Safety logic
			Vehicle.PRND.IsConnected = false;
			Vehicle.PRND.RawStatus = NEUTRAL_GEAR;

      Vehicle.WheelSpeed.IsConnectedFL = false;
      Vehicle.WheelSpeed.IsConnectedFR = false;
      Vehicle.WheelSpeed.IsConnectedRL = false;
      Vehicle.WheelSpeed.IsConnectedRR = false;

			if (getEngineFlag() != ENGINE_STOP_NEUTRAL)
			{
				neutralEngine();
				setEngineFlag(ENGINE_STOP_NEUTRAL);
			}
*/
			// -------- Beginning of LED area ---------

			// Visually signaling no connection with PRND

			if (statusLed.state != LED_FAST_BLINK)
			{
				statusLed.state = LED_FAST_BLINK;
			}

			LED_Handle(&statusLed);

			// -------- End of LED area ---------

		}
        // --- 2. No error detected - normal state ---
        else
        {
            // Executes only when no timeout detected

        	/*
        	// SIGNALING CHARGING
            if (Vehicle.Charger.RawStatus == START_CHARGING)
            {
                 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 1);
            }
            else // SIGNALING STOP CHARGING
            {
                 HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 0);
            }
            */

        	/* @brief Clearing the no connection with PRND error. */
        	// EH_clear(&heh, 0x100);

          // Clearing the no connection with Wheel Speed error.
          // EH_clear(&heh, 0x101);
          // EH_clear(&heh, 0x102);
          // EH_clear(&heh, 0x103);
          // EH_clear(&heh, 0x104);


			// -------- Beginning of LED area ---------

			// Signaling connection with PRND

			if (statusLed.state != LED_BLINK)
			{
				statusLed.state = LED_BLINK;
			}

			LED_Handle(&statusLed);

			// -------- End of LED area ---------


        }
		// JETSON currently not used
		/*
        // --- JETSON WATCHDOG ---
        if (now - Vehicle.Jetson.LastMsgTick > timeout) {
            Vehicle.Jetson.IsConnected = false;
            // Safety Fail-safe: Clear stale data to prevent unintended behavior.
            // IN OTHER WORDS: Prevent "ghost" inputs. If connection is lost, we must not execute the last received command forever.
           // memset((void*)Vehicle.Jetson.RawData, 0, 8);		//clears the data received from Jetson
        }
        */
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
	  ;
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
