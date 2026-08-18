
/**
  ******************************************************************************
  * @file      adc_driver.c
  * @author    Bartosz Rychlicki
  * @Title     Universal driver for ADC peripheral
  * @brief     This file contains common functions' bodies with ADC Driver's functions implementation
  ******************************************************************************
  * @attention Error codes are called when exact incorrect use of function is made
  *
  * Copyright (c) 2026 AGH Eko-Energy.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#include "adc_driver.h"


/* Functions' bodies ------------------------------------------------------------------------------------ */

/*
 * @brief Private function that Initialize ADC channel, when there is no DMA support, ADC is in independent mode and ADC only converts single channel | only checks if correct flag has been set
 */
static HAL_StatusTypeDef ADC_Init_NoDMA_Independent(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc){


    // Checking of correct number of discontinuous conversion is set in case user works witm multiple channels
	if(cadc->numberOfSelectedChannels > 1){
		if(hadc->Init.NbrOfDiscConversion != 1){
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}

/*
 * @brief Private function that Initialize ADC with DMA support
 */
static HAL_StatusTypeDef ADC_Init_DMA(ADC_HandleTypeDef* hadc, ADC_BufferTypeDef* badc){

	// security check
	if(NULL == hadc || NULL == badc || NULL == hadc->DMA_Handle){
		return HAL_ERROR;
	}

	// checking if user turned on correct mode of DMA and ADC's Continuous flag
	if(ADC_DMA_GetMode(hadc) != DMA_MODE_CIRCULAR || ADC_Continuous(hadc) == DISABLE){
		return HAL_ERROR;
	}

	/*
	 * @brief Checking if dma continuous request is enabled
	 * @note This flag is only on F3 cores
	 */
	#if defined(STM32F3_FAMILY)
		if(DISABLE == hadc->Init.DMAContinuousRequests){
			return HAL_ERROR;
		}
	#endif

	// launching DMA for ADC
	if(hadc->Instance == ADC1){


		if(HAL_ADC_Start_DMA(hadc, (uint32_t*)badc->adc1Values, ADC1_BUFFER_SIZE) == HAL_OK){
			return HAL_OK;
		}


	}else if(hadc->Instance == ADC2){


		if(HAL_ADC_Start_DMA(hadc, (uint32_t*)badc->adc2Values, ADC2_BUFFER_SIZE) == HAL_OK){
			return HAL_OK;
		}


	}else{

		// returning error status
		return HAL_ERROR;
	}



	return HAL_ERROR;
}

/*
 * @brief Private function that reads ADC channels, when there is no DMA support, ADC is in independent mode and ADC converts severals channels
 */
static HAL_StatusTypeDef ADC_ReadChannel_NoDMA_Independent(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, uint8_t* rank, uint16_t* retval){

	// Declaration of variable which will temporarily store sampled value
	uint16_t tempValue = 0, endValue = 0;

    // iterating through all ranks
    for(uint8_t i = 0; i < cadc->numberOfSelectedChannels; ++i){

    	// Starting sequencer for ADC
    	if(HAL_ADC_Start(hadc) != HAL_OK){
    	    return HAL_ERROR;
    	}

    	// Starting polling
    	if(HAL_ADC_PollForConversion(hadc, ADC_POLLING_TIMEOUT) == HAL_OK){

    		// reading value to temporary variable
            tempValue = HAL_ADC_GetValue(hadc);

    		// checking if current iteration is co-related to rank for given channel
            if(*rank == i){

            	// assigning to end value of sampled ADC value
            	endValue = tempValue;
            }
    	}else{

    		// if polling crashes then return error status
    		return HAL_ERROR;
    	}

    }

    // Stopping ADC to reset sequencer
    if(HAL_ADC_Stop(hadc) != HAL_OK){
    	return HAL_ERROR;
    }

    // checking if end value is valid
    if(endValue >= 0 && endValue <= ADC_Resolution(hadc)){

    	*retval = endValue;

    	// returning OK status
        return HAL_OK;
    }

    // returning Error status when sampled value is not in safe bounds
    return HAL_ERROR;
}

/*
 * @brief Private function that reads ADC channel, when there is no DMA support, ADC is in independent mode and ADC only converts single channel
 */
static HAL_StatusTypeDef ADC_ReadChannel_NoDMA_Independent_SingleConversion(ADC_HandleTypeDef* hadc, uint16_t* retval){

	// initializing temporary variable to store read value before checking if it's correct
	uint16_t tempValue = 0;

	// starting ADC
	if(HAL_ADC_Start(hadc) != HAL_OK){
		return HAL_ERROR;
	}

	// Starting polling - ADC now reads channel
	if(HAL_ADC_PollForConversion(hadc, ADC_POLLING_TIMEOUT) == HAL_OK){

		tempValue = HAL_ADC_GetValue(hadc);

	}else{

		// stop ADC before returning error to leave peripheral in known state
		HAL_ADC_Stop(hadc);
		// when error occurred while polling then error status is returned
		return HAL_ERROR;
	}


	// Stopping ADC to reset sequencer
	if(HAL_ADC_Stop(hadc) != HAL_OK){
		return HAL_ERROR;
	}


	// checking if read value is correct
	if(tempValue <= ADC_Resolution(hadc)){

		// assigning read value to retval
		*retval = tempValue;

		// returning OK status
		return HAL_OK;

	}

	// when something went wrong, return error status
	return HAL_ERROR;
}


/*
 * @brief Function provides averaging of severals channel's sampled values to provide high efficient measures
 */
static HAL_StatusTypeDef ADC_Averaging(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, ADC_BufferTypeDef* badc, uint8_t rank, uint16_t* retval){

	// security check if user correctly configured buffer size
	if(ADC1_BUFFER_SIZE < 1 || ADC2_BUFFER_SIZE < 1){
		return HAL_ERROR;
	}


	// init of variable to store sum of measures from one channel
	uint32_t value = 0;

	// variable stores number of samples
	uint8_t samples = 0;

	if(hadc->Instance == ADC1){

		// reading ADC1 buffer amount of samples for one channel
		samples = ADC1_SAMPLING;


	}else if(hadc->Instance == ADC2){

		// reading ADC2 buffer amount of samples for one channel
		samples = ADC2_SAMPLING;

	}else{

		// returning error state when incorrect handle was given
		return HAL_ERROR;
	}



	// summing measures for one channel
	/*
	 * @note i increments up to samples, because it defines amount of iterations, for loops need to do to access all samples despite of channel's rank
	 */
	for(uint8_t i = 0; i < samples; ++i){
		value += (hadc->Instance == ADC1) ? badc->adc1Values[(uint8_t)((i * cadc->numberOfSelectedChannels) + rank)] : badc->adc2Values[i * cadc->numberOfSelectedChannels + rank];
	}

	// calculating averaged value
	value /= samples;

	// security check, if calculated values in well averaged
	if(value <= ADC_Resolution(hadc)){

		// assigning calculated value to function's output
		*retval = value;

		// returning OK status
		return HAL_OK;
	}


	return HAL_ERROR;
}


/*
 * @brief Function read ADC register associated with L data - number of channels in use
 */
static HAL_StatusTypeDef ADC_ChannelsNumberConfig(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc){

	// Implementation for F1 family
    #if defined(STM32F1_FAMILY) || defined(STM32F3_FAMILY)

        // reading number of channels in used from SQR1
	    cadc->numberOfSelectedChannels = (((hadc->Instance->SQR1 >> ADC_SQR1_L_Pos) & 0xF) + 1);

	    // if read number of selected channels in not 0, then return OK status
	    if(cadc->numberOfSelectedChannels != 0){
	    	return HAL_OK;
	    }


    #endif

	return HAL_ERROR;
}

/*
 * @brief Function read ADC register associated with ranks configuration - SQR1, SQR2, SQR3 and in some families SQR4
 */
static HAL_StatusTypeDef ADC_ChannelsConfig(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc){

	// checking if correct number of channels has been set
    if(0 == cadc->numberOfSelectedChannels){
    	return HAL_ERROR;
    }

    // Executing operation for F1 family
    #if defined(STM32F1_FAMILY)

	    for(uint8_t i = 0; i < cadc->numberOfSelectedChannels; ++i){

	    	// Execution of read operation of SQR3 register
	    	if(i < ADC_SQR3_2_OFFSET){

                 cadc->ranks[i] = (uint8_t)(hadc->Instance->SQR3 >> (i * ADC_SQR_DATA_RES)) & ADC_SQR_DATA_Msk;

	    	}
	    	// Execution of read operation of SQR2 register
	    	else if(i < ADC_SQR3_1_OFFSET){

	    		 /* Subtracting ADC_SQR3_2_OFFSET because at this point i is no lower than 7, in bit shifting we need to start from 0*/
	    		 cadc->ranks[i] = (uint8_t)(hadc->Instance->SQR2 >> ((i - ADC_SQR3_2_OFFSET)  * ADC_SQR_DATA_RES)) & ADC_SQR_DATA_Msk;

	    	}
	    	// Execution of read operation of SQR1 register
	    	else{

	    		 /* Subtracting ADC_SQR3_1_OFFSET because at this point i is no lower than 13, in bit shifting we need to start from 0*/
	    		 cadc->ranks[i] = (uint8_t)(hadc->Instance->SQR1 >> ((i - ADC_SQR3_1_OFFSET)  * ADC_SQR_DATA_RES)) & ADC_SQR_DATA_Msk;

	    	}

	    }

	    // returning OK status
	    return HAL_OK;

    #elif defined(STM32F3_FAMILY)

	    for(uint8_t i = 0; i < cadc->numberOfSelectedChannels; ++i){

	    	// reading channels from SQR1
            if(i < ADC_SQR1_2_OFFSET){

            	cadc->ranks[i] = (uint8_t)((hadc->Instance->SQR1 >> ((i + ADC_SQR1_L_OFFSET) * ADC_SQR_DATA_RES)) & ADC_SQR_DATA_Msk);

            }
            // Reading SQR2
            else if(i < ADC_SQR1_3_OFFSET){

            	cadc->ranks[i] = (uint8_t)((hadc->Instance->SQR2 >> ((i - ADC_SQR1_2_OFFSET) * ADC_SQR_DATA_RES)) & ADC_SQR_DATA_Msk);

            }
            // Reading SQR3
            else if(i < ADC_SQR1_4_OFFSET){

            	cadc->ranks[i] = ((hadc->Instance->SQR3 >> ((i - ADC_SQR1_3_OFFSET) * ADC_SQR_DATA_RES)) & ADC_SQR_DATA_Msk);

            }
            // Reading SQR4
            else{

                cadc->ranks[i] = ((hadc->Instance->SQR4 >> ((i - ADC_SQR1_4_OFFSET) * ADC_SQR_DATA_RES)) & ADC_SQR_DATA_Msk);

            }

		}

        // returning OK status
	    return HAL_OK;
    #endif

    // returning OK status
	return HAL_ERROR;
}

HAL_StatusTypeDef ADC_Init(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, ADC_BufferTypeDef* badc){

	// checking if user passed null pointer to ADC handle or cadc structure
	if(NULL == hadc || NULL == cadc){
		return HAL_ERROR;
	}

	// Initializing channels configuration operation
	if(ADC_Get_ChannelsConfiguration(hadc, cadc) != HAL_OK){
		return HAL_ERROR;
	}

	if(ADC_DMA_ENABLED(hadc) == DISABLE){

		// Init ADC for correct work mode
		if(ADC_Continuous(hadc) == DISABLE){

			// if user uses more than 1 channel, then discontinuous flag should be checked if is enabled
			if(cadc->numberOfSelectedChannels > 1){
				if(ADC_Discontinuous(hadc) == DISABLE){
					return HAL_ERROR;
				}
			}

			// Checking if User made correct config in .ioc file
			if(ADC_Init_NoDMA_Independent(hadc, cadc) != HAL_OK){
				return HAL_ERROR;
			}
		}else{

			// when flag for manual mode is not set, then return error
			return HAL_ERROR;
		}
	}else{
		if(ADC_Init_DMA(hadc, badc) != HAL_OK){
			return HAL_ERROR;
		}
	}

	if(hadc->Instance == ADC1){

		// Enabling clock for ADC1
		__HAL_RCC_ADC1_CLK_ENABLE();

	}else if(hadc->Instance == ADC2){

		// Enabling clock for ADC2
		__HAL_RCC_ADC2_CLK_ENABLE();
	}

	return HAL_OK;
}


HAL_StatusTypeDef ADC_ReadChannel(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, ADC_BufferTypeDef* badc, uint8_t channel, uint16_t* retval){

	if(NULL == hadc || NULL == retval  || NULL == cadc || NULL == badc){
		return HAL_ERROR;
	}

    // Declaration of rank for given channel
	uint8_t rank = 0;

	// Reading rank
	if(ADC_Get_ChannelRank(hadc, cadc, channel, &rank) != HAL_OK){
		return HAL_ERROR;
	}

	// Checking of firmware correctly sensed rank for passed channel | value can be set to one of values from [0, 15]
	if(rank >= 16){
		return HAL_ERROR;
	}

	// Executing reading, with calling correct to .ioc config, private function
    if(DISABLE == ADC_DMA_ENABLED(hadc)){

    	// Executing reading channel for mode without DMA
    	if(ADC_Discontinuous(hadc) == ENABLE){

    		if(ADC_ReadChannel_NoDMA_Independent(hadc, cadc, &rank, retval) == HAL_OK){
    			return HAL_OK;
    		}
        // Driver does not support different reading mode without DMA usage, so it returns error
    	}else{

    		//  if flag is not set while working with several channels, then error is returned
    		if(cadc->numberOfSelectedChannels > 1){

    			return HAL_ERROR;

    		}else{

    			// When single conversion is only set, correct read mode is called
    			if(ADC_ReadChannel_NoDMA_Independent_SingleConversion(hadc, retval) == HAL_OK){
    				return HAL_OK;
    			}

    		}

    	}
    }else{

    	// launching averaging function - to calculate ADC's converted value
    	if(ADC_Averaging(hadc, cadc, badc, rank, retval) == HAL_OK){
    		return HAL_OK;
    	}
    }

	return HAL_ERROR;
}

HAL_StatusTypeDef ADC_Get_PinVoltage(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, ADC_BufferTypeDef* badc, uint8_t channel, float* retval){

	// checking if user gave null pointers
	if(NULL == hadc || NULL == retval){
		return HAL_ERROR;
	}

	// Declaration of variables to store temporary data
	uint16_t binaryValue = 0;
	float tempValue = 0.0f;

	// Reading channel's sampled value
	if(ADC_ReadChannel(hadc, cadc, badc, channel, &binaryValue) != HAL_OK){
		return HAL_ERROR;
	}

	// Calculating voltage on pin
	tempValue = (float)((float)binaryValue/(float)ADC_Resolution(hadc) * STM32_VCC);

	// Security check
	if(tempValue >= STM32_GND || tempValue <= STM32_VCC){

		// assigning calculated value to return value
		*retval = tempValue;

		// returning OK status
		return HAL_OK;

	}

	return HAL_ERROR;
}


HAL_StatusTypeDef ADC_Get_ChannelRank(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, uint8_t channel, uint8_t* rank){

	// iterating through table of channels in cadc
	for(uint8_t i = 0; i < cadc->numberOfSelectedChannels; ++i){
		if(channel == cadc->ranks[i]){

			// overwriting parameter's value - rank - with i value, rank to which channel is assigned
			*rank = i;

			return HAL_OK;
		}
	}

	return HAL_ERROR;
}


HAL_StatusTypeDef ADC_Get_ChannelsConfiguration(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc){

	// reading number of channels to be used
	if(ADC_ChannelsNumberConfig(hadc, cadc) != HAL_OK){
		return HAL_ERROR;
	}

	// reading channels numbers assigned to ADC's ranks
	if(ADC_ChannelsConfig(hadc, cadc) != HAL_OK){
		return HAL_ERROR;
	}

	return HAL_OK;
}
