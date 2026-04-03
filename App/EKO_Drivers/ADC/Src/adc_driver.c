
/**
  ******************************************************************************
  * @file      adc_driver.c
  * @author    Bartosz Rychlicki
  * @Title     Universal driver for ADC peripheral
  * @brief     This file contains common functions' bodies with ADC Driver's functions implementation
  ******************************************************************************
  * @attention Error codes are called when exact incorrect use of function is made
  *
  * Copyright (c) 2025 AGH Eko-Energy.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/


#include "adc_driver.h"

// Private variables
// Container of ADC ranks to return rank's register while number of rank is given
static 			uint32_t ADC_RANKS_REGS[16] = {
    ADC_RANK1_REG, ADC_RANK2_REG, ADC_RANK3_REG, ADC_RANK4_REG,
    ADC_RANK5_REG, ADC_RANK6_REG, ADC_RANK7_REG, ADC_RANK8_REG,
    ADC_RANK9_REG, ADC_RANK10_REG, ADC_RANK11_REG, ADC_RANK12_REG,
    ADC_RANK13_REG, ADC_RANK14_REG, ADC_RANK15_REG, ADC_RANK16_REG
};

// Container of ADC ranks bit position to return provide correct flow of bit shifting
static 			uint32_t ADC_RANKS_BITPOS[16] = {
    ADC_RANK1_BITPOS, ADC_RANK2_BITPOS, ADC_RANK3_BITPOS, ADC_RANK4_BITPOS,
    ADC_RANK5_BITPOS, ADC_RANK6_BITPOS, ADC_RANK7_BITPOS, ADC_RANK8_BITPOS,
    ADC_RANK9_BITPOS, ADC_RANK10_BITPOS, ADC_RANK11_BITPOS, ADC_RANK12_BITPOS,
    ADC_RANK13_BITPOS, ADC_RANK14_BITPOS, ADC_RANK15_BITPOS, ADC_RANK16_BITPOS
};

/**
  * @brief  ADC1 Initialization Function, performs calibration and starts conversions.
  * @param  hadc  Pointer to ADC handle.
  * @param  badc  Pointer to ADC buffer configuration/structure.
  * @param  cadc  Pointer to ADC channels configuration/structure.
  * @retval ADC_StatusTypeDef  ADC driver status.
  */
HAL_StatusTypeDef ADC_Init(ADC_HandleTypeDef* hadc, ADC_BufferTypeDef* badc, ADC_ChannelsTypeDef* cadc){

	// check if ADC is started to stop it to calibrate ADC
	if(__ADC_IS_CONV_STARTED(hadc) != 0){
		if(HAL_ADC_Stop(hadc) != HAL_OK){
			return HAL_ERROR;
		}
	}


	#if !(defined(STM32F2_FAMILY) || defined(STM32F4_FAMILY))

			#if defined(STM32F1_FAMILY)

				// launching calibration for F1 core
				if(HAL_ADCEx_Calibration_Start(hadc) != HAL_OK){
					return HAL_ERROR;
				}

			#else
				// launching calibration for F3 core
				if(HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK){
					return HAL_ERROR;
				}

			#endif
	#endif


	// launching ADC
	if(HAL_ADC_Start(hadc) != HAL_OK){
		return HAL_ERROR;
	}

	// getting ranks config
	if(ADC_ConfigGetRanksOfChannels(hadc, cadc, badc)!= HAL_OK){
		return HAL_ERROR;
	}


	// check if dual mode is enabled
	if(__ADC_IS_DMA_MULTIMODE(hadc) == 0){

			// checking if DMA is enabled
			if(__ADC_IS_DMA_ENABLED(hadc) != 0){

				// starting DMA with ADC in Independent mode
				if(HAL_ADC_Start_DMA(hadc, (uint32_t*)badc->idma.BufferADC, ADC_BUFF_SIZE) != HAL_OK){
					return HAL_ERROR;
				}
			}
	}else{

			// checking if DMA is enabled
			if(__ADC_IS_DMA_ENABLED(hadc) != 0){

				// stopping ADC to reconfigure it for dual mode DMA
				if(HAL_ADC_Stop(hadc) != HAL_OK){
					return HAL_ERROR;
				}
			}

	}


	return HAL_OK; // returning positive status
}


/**
  * @brief ADC Init function, initialize ADC in dual mode | Should be called in init function of slave ADC
  * @param  hadc    - pointer to ADC Master handle
  * @param  badc    - pointer to ADC buffer structure used in dual mode
  * @retval status  - HAL status if init went successfully
  */
HAL_StatusTypeDef ADC_InitMultimode(ADC_HandleTypeDef* hadcMaster, ADC_BufferTypeDef* badc){

	// checking if correct parameter was provide
	if(hadcMaster->Instance != ADC1){
		return HAL_ERROR;
	}
	// checking if buff struct is initialized
	if(badc == NULL){
		return HAL_ERROR;
	}

	// For F2 and F4 family Calibration function does not exist
	#if !(defined(STM32F2_FAMILY) || defined(STM32F4_FAMILY))

		#if defined(STM32F1_FAMILY)

			// starting calibration for F1 Core
			if(HAL_ADCEx_Calibration_Start(hadcMaster) != HAL_OK){
				return HAL_ERROR;
			}

		#else

			// starting calibration for F3 Core
			if(HAL_ADCEx_Calibration_Start(hadcMaster, ADC_SINGLE_ENDED) != HAL_OK){
				return HAL_ERROR;
			}

			#endif
	#endif

	// launching dual mode conversion
	if(HAL_ADCEx_MultiModeStart_DMA(hadcMaster, badc->ddma.BufferMultiMode, ADC_BUFF_SIZE) != HAL_OK){
		return HAL_ERROR;
	}


	return HAL_OK;

}


/**
  * @brief ADC Reading channel function
  * @param  hadc    - pointer to ADC handle
  * @param  channel - number of channel to be read
  * @param  retval  - pointer to variable, whose contains return value
  * @retval status  - HAL status if Reading channel went successfully
  */
HAL_StatusTypeDef ADC_ReadChannel(ADC_HandleTypeDef* hadc, ADC_ChannelsTypeDef* cadc, ADC_BufferTypeDef* badc, uint8_t channel, uint16_t*  retval){


	// checking ADC status | is launched?
	if(__ADC_IS_CONV_STARTED(hadc) == 0){ // ADC not started
		return HAL_ERROR;
	}

	// security check | is given number of channel correct
	if(channel > 16){
		return HAL_ERROR;
	}

	uint8_t rank  = 0; // initialize variable that stores the rank of the given channel

	// reading rank of given channel and writing it to correct variable
	if(ADC_GetRank(cadc, channel, &rank) != HAL_OK){
		return HAL_ERROR;
	}

	// checking status of DMA
	if(__ADC_IS_DMA_ENABLED(hadc) == 0){  // DMA Disabled

		// iterating through all ranks to read value from correct channel's rank in ADC without DMA
		for(int i  = 0 ; i <= rank ; ++i){

			 if(__ADC_IS_DMA_MULTIMODE(hadc) == 0){  // single conversion | independent mode

				 uint16_t value = 0;

				 // getting converted value
				 value = HAL_ADC_GetValue(hadc);

				 // checking if converted value is valid
				 if(value > __ADC_RESOLUTION(hadc) || value < 0){
					 return HAL_ERROR;
				 }

				 // overwriting value in buffer only if process of reading from given channel is executed
				 if(i == rank){
					 badc->ADC_Buff[rank] = value;
				 }

			}else{
				// single conversion | dual mode

				uint32_t value = 0;

				// getting converted value
				value = HAL_ADCEx_MultiModeGetValue(hadc);

				// checking if converted value is valid
				if(value > __ADC_RESOLUTION(hadc) || value < 0){
					 return HAL_ERROR;
				}

				// overwriting value in buffer only if process of reading from given channel is executed
				if(i == rank){
					value = HAL_ADCEx_MultiModeGetValue(hadc);
				}
			}
		}

		// security check | if converted value is higher than ADC's resolution or less than 0
		if(badc->ADC_Buff[channel] > __ADC_RESOLUTION(hadc) || badc->ADC_Buff[channel] < 0){
			return  HAL_ERROR;
		}

		//overwriting converted value of ADC | otherwise no assign of value will be conducted
		*(retval) = badc->ADC_Buff[rank];

		// re-launching ADC if its mode is non-continuous
		if(__ADC_MODE(hadc) == 0){
			if(HAL_ADC_Start(hadc) != HAL_OK){
				return HAL_ERROR;
			}
		}

	}else{								  // DMA Enabled

		// averaging converted values from given channel
		if(ADC_Averaging(hadc, badc, cadc, channel, retval) != HAL_OK){ // averaging transfer
			return HAL_ERROR;
		}

		// re-launching conversion for DMA in normal mode | with checking if ADC/ADCs are in independent or dual mode
		if(__ADC_IS_DMA_MULTIMODE(hadc) != 0){     // ADC in dual mode | DMA [ON]

			// re-launching ADC in dual mode conversion with DMA
			if(__ADC_DMA_MODE(hadc) == 0){
				if(HAL_ADCEx_MultiModeStart_DMA(hadc, badc->ddma.BufferMultiMode, ADC_BUFF_SIZE) != HAL_OK){
					return HAL_ERROR;
				}
			}

		}else{									   // ADC in independent mode | DMA [ON]

			// re-launching ADC in independent conversion with DMA
			if(__ADC_DMA_MODE(hadc) != 0){
				if(HAL_ADC_Start_DMA(hadc, (uint32_t*)badc->idma.BufferADC, ADC_BUFF_SIZE) != HAL_OK){
					return HAL_ERROR;
				}
			}
		}


	}


	return HAL_OK;
}

/**
  * @brief ADC Basic function of returning value
  * 	   if calculating value's logic is different, then developer should implement his function
  * @param  hadc    - pointer to ADC handle
  * @param  channel - number of channel to be read
  * @param  retval  - pointer to variable, whose contains return value
  * @retval status  - HAL status if Reading channel went successfully
  */
__weak HAL_StatusTypeDef  ADC_GetValue(ADC_HandleTypeDef* hadc, ADC_ChannelsTypeDef* cadc, ADC_BufferTypeDef* badc, float max, uint8_t channel, float * retval){

	uint16_t binaryType = 0; 							// init of variable which stores converted value from channel
	uint32_t adcResolutiion = __ADC_RESOLUTION(hadc);  // reading ADC resolution

	// reading channel's cconverted value
	if(ADC_ReadChannel(hadc, cadc, badc, channel, &binaryType) != HAL_OK){
		return HAL_ERROR;
	}

	// Basic math here | calculating float value with formula, example: voltage = binary/value/adc_resoltuion * maxVoltage
	*retval = max * ((float)binaryType / (float)adcResolutiion);


	return HAL_OK;

}


/*
 * @brief Empty implementation of callback function | If body of this function should does some functionalities, then implementation is required
 */
/*
void               HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	UNUSED(hadc); // unused variable to avoid warnings

}
*/
/**
  * @brief ADC channels configuration function | auto detects ranks and overwrite ADC_ChannelsTypeDef object's content
  * @param  hadc    - pointer to ADC handle
  * @retval status  - HAL status if reading channels configuration went successfully
  */
HAL_StatusTypeDef  ADC_ConfigGetRanksOfChannels(ADC_HandleTypeDef* hadc, ADC_ChannelsTypeDef* cadc, ADC_BufferTypeDef* badc){

	//Initialize variable that stores number of conversions
	uint32_t numberOfConversions;


	//Reading number of channels to be converted
	numberOfConversions = ((hadc->Instance->SQR1 & ADC_SQR1_L_Msk) >> ADC_SQR1_L_Pos) + 1;


	// Security check if number of ADC's number of turned on channels was correctly calculated
	if(numberOfConversions > ADC_MAX_CHANNELS || numberOfConversions < 1){
		return HAL_ERROR;
	}

	// Overwriting macro which stores info of number of channels to be converted
	ADC_CONVERTED_CHANNELS = numberOfConversions;


	// reading ranks' assigned channels
	for(int i = 0; i < numberOfConversions; ++i){

			#if defined(ADC_SQR4_SQ15_Pos) // if core has 4 registers of SQRx

				switch(ADC_RANKS_REGS[i]){ // reading register to which rank is assigned
						case SQR_1:
							cadc->ranks[i] = ((hadc->Instance->SQR1 >> ADC_RANKS_BITPOS[i]) & 0x1f); // extracting binary value on correct position in SQR1
							break;
						case SQR_2:
							cadc->ranks[i] = ((hadc->Instance->SQR2 >> ADC_RANKS_BITPOS[i]) & 0x1f); // extracting binary value on correct position in SQR2
							break;
						case SQR_3:
							cadc->ranks[i] = ((hadc->Instance->SQR3 >> ADC_RANKS_BITPOS[i]) & 0x1f); // extracting binary value on correct position in SQR3
							break;
						case SQR_4:
							cadc->ranks[i] = ((hadc->Instance->SQR4 >> ADC_RANKS_BITPOS[i]) & 0x1f); // extracting binary value on correct position in SQR4
							break;
						default:
							break;
					}


			#else							// the rest of cores, for which driver is built, has only 3 registers of SQRx

				switch(ADC_RANKS_REGS[i]){  // reading register to which rank is assigned
					case SQR_1:
						cadc->ranks[i] = ((hadc->Instance->SQR1 >> ADC_RANKS_BITPOS[i]) & 0x1f); // extracting binary value on correct position in SQR1
						break;
					case SQR_2:
						cadc->ranks[i] = ((hadc->Instance->SQR2 >> ADC_RANKS_BITPOS[i]) & 0x1f); // extracting binary value on correct position in SQR2
						break;
					case SQR_3:
						cadc->ranks[i] = ((hadc->Instance->SQR3 >> ADC_RANKS_BITPOS[i]) & 0x1f); // extracting binary value on correct position in SQR3
						break;
					default:
						break;
				}

			#endif


	}

	return HAL_OK;
}

/**
  * @brief ADC channels' ranks return function. In case of wanting channel's rank, function returns it
  * @param  hadc    - pointer to ADC handle
  * @retval status  - ADC status
  */
HAL_StatusTypeDef  ADC_GetRank(ADC_ChannelsTypeDef *cadc, uint8_t channel, uint8_t* rank){

	// iterating though all buffer's elements to return given channel's rank
	for(int i = 0 ; i < ADC_MAX_CHANNELS; ++i ){
		if(cadc->ranks[i] == channel){
			*rank = (uint8_t)i;

			break;
		}
	}

	// Checking if read rank is correct | ADC has maximum 16 ranks
	if(*rank >= ADC_MAX_CHANNELS){
		return HAL_ERROR;
	}

	return HAL_OK;
}

/**
  * @brief ADC averaging function. ADC's channels' values oscillate in 40 Hz, function averages measures from exact number of conversions.
  * 	   Macro: ADC_AVERAGED_MEASURES stores information about number of latest conversions to be measured
  * @param  hadc    - pointer to ADC handle
  * @param  badc    - ADC buffer, which stores converted values
  * @param  retval  - pointer to returning value | overwrite value from 0 to 15
  * @retval status  - ADC status
  */
HAL_StatusTypeDef ADC_Averaging(ADC_HandleTypeDef* hadc, ADC_BufferTypeDef* badc, ADC_ChannelsTypeDef* cadc, uint8_t channel , uint16_t* retval){

	uint64_t sum = 0; // sum of values from averaged channel
	uint8_t rank;     // channel's rank

	// Getting channel rank
	if(ADC_GetRank(cadc, channel, &rank) != HAL_OK){
		return HAL_ERROR;
	}

	if(__ADC_IS_DMA_MULTIMODE(hadc) == 0){ // ADC in independent mode
		if(sizeof(badc->idma.BufferADC)/sizeof(badc->idma.BufferADC[0]) < ADC_AVERAGED_MEASURES){
			return HAL_ERROR;
		}


	}else{								   // ADC in dual mode

		if(hadc->Instance == ADC1){

			// Extracting ADC1 values from dual mode buffer
			for(int  i = 0 ; i < ADC_BUFF_SIZE ; ++i){
				badc->ddma.BufferADC_Master[i] = (uint16_t)((badc->ddma.BufferMultiMode[i] >> 16));
			}

		}else{

			// Extracting ADC2 values from dual mode buffer
			for(int  i = 0 ; i < ADC_BUFF_SIZE ; ++i){
				badc->ddma.BufferADC_Slave[i] = (uint16_t)((badc->ddma.BufferMultiMode[i]));
			}

		}


	}


	int id = 0; // current position of averaged value

	for( int i = 0; i < ADC_AVERAGED_MEASURES; ++i){
		id = (i * ADC_CONVERTED_CHANNELS + rank); // id calculation base on multiplying current iteration by number of conversions to be measures, cause DMA stores continuously conversion though channels until last index of DMA buffer occurs

		// security check | if calculated id is beyond array limits
		if(id >= ADC_BUFF_SIZE){
			return HAL_ERROR;
		}

		// adding to sum variable next value correlated to current channel
		sum += ((__ADC_IS_DMA_MULTIMODE(hadc) == 0)
					 ? badc->idma.BufferADC[id] 							 // adding value of ADC in independent mode
				      :((hadc->Instance == ADC1)
					 ? badc->ddma.BufferADC_Master[id]
	                  : badc->ddma.BufferADC_Slave[id]));                   // adding value of ADC in dual mode | checking instance
	}

	*retval = (uint16_t)(sum / ADC_AVERAGED_MEASURES); // averaging by dividing sum with number of averaged conversions

	return HAL_OK;
}





