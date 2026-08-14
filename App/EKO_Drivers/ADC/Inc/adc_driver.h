/**
  ******************************************************************************
  * @file    adc_driver.h
  * @author  Bartosz Rychlicki
  * @author  AGH Eko-Energia

  * @Title   Universal driver for ADC peripheral (Built for F1 and F3 families, but can be implemented for all families)

  * @brief   This file contains common defines, flags and macros that are used to prevent high quality of driver's functionalities.
  * 		 All flags, macros and typedefs are built to sense ADC settings and its linked DMA settings
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 AGH Eko-Energy.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_ADC_DRIVER_H_
#define INC_ADC_DRIVER_H_

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes  ---------------------------------------------------------------------------------------------------------- */
#include <stdint.h>
#include "adc_utils.h"
#include "adc_conf.h"


/* Typedefs and structures -------------------------------------------------------------------------------------------- */

/*
 *	@brief Structure stores channels numbers assigned to exact rank
 */
typedef struct{

	uint8_t  ranks[ADC_MAX_RANKS_NUMBER];                                   //< Table of ADC's channels' configuration for each rank
	uint8_t  numberOfSelectedChannels;                                      //< Number of detected channels, which user selected to use
}ADC_ChannelsConfigTypeDefs;

/*
 *	@brief Structure stores converted ADC values with DMA usage
 */
typedef struct{


    uint16_t adc1Values[ADC1_BUFFER_SIZE];									//< Table of ADC1's channels' read values
	uint16_t adc2Values[ADC2_BUFFER_SIZE];									//< Table of ADC2's channels' read values

}ADC_BufferTypeDef;


/* Private constant macros -------------------------------------------------------------------------------------------- */


/* Functions' prototypes ------------------------------------------------------------------------------------ */

/*
 * @brief  ADC initialization function, initialize type of conversion, set correct ADC's regs' values
 * @param  hadc - handle to ADC instance
 * @retval status of HAL's operation
 */
HAL_StatusTypeDef ADC_Init(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, ADC_BufferTypeDef* badc);


/*
 * @brief  ADC reading channel's value function.
 * @param  hadc    - handle to ADC instance
 * @param  cadc     - handle to ADC's Channels structure
 * @param  channel - pointer to number of channel, to which value is going to be read
 * @param  *retval - pointer to value, which stores converted value
 * @retval status of HAL's operation
 */
HAL_StatusTypeDef ADC_ReadChannel(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, ADC_BufferTypeDef* badc, uint8_t channel, uint16_t* retval);


/*
 * @brief  Read and scale ADC conversion result for the specified rank into the voltage on the assigned STM32 pin.
 * @param  hadc     - handle to ADC instance
 * @param  cadc     - handle to ADC's Channels structure
 * @param  rank     - pointer to conversion channel, which value is going to be read
 * @param  *retval  - pointer to variable that will store pin voltage for future operations
 */
HAL_StatusTypeDef ADC_Get_PinVoltage(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, ADC_BufferTypeDef* badc, uint8_t channel, float* retval);

/*
 * @brief  Function extracts information about channels configuration from ADC registers - it simply read channel number for each rank and amout of channels in use
 * @param  hadc     - handle to ADC instance
 * @param  cadc     - pointer to cadc variable which stores channels' configuration
 * @retval          - HAL_StatusTypeDef status of operation
 */
HAL_StatusTypeDef ADC_Get_ChannelsConfiguration(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc);

/*
 * @brief  Function extracts information about channels configuration from ADC registers - it simply read channel number for each rank and amout of channels in use
 * @param  hadc     - handle to ADC instance
 * @param  channel  - number of channel, which rank number will be returned
 * @param  rank 	- pointer to rank, which value will be overwritten with rank number for given channel (Pay attention - rank should be passed from [0:N-1] to ensure, because indexes in C starts from 0 (not 1))
 * @retval          - HAL_StatusTypeDef status of operation
 */
HAL_StatusTypeDef ADC_Get_ChannelRank(ADC_HandleTypeDef* hadc, ADC_ChannelsConfigTypeDefs* cadc, uint8_t channel, uint8_t* rank);


#ifdef __cplusplus
}
#endif

#endif /* INC_ADC_DRIVER_H_ */
