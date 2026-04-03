/**
  ******************************************************************************
  * @file    adc_driver.h
  * @author  Bartosz Rychlicki

  * @Title   Universal driver for ADC peripheral (Built for F1, F2, F3 and F4 families, but can be implemented for all families)

  * @brief   This file contains common defines, flags and macros that are used to prevent high quality of driver's functionalities.
  * 		 All flags, macros and typedefs are built to sense ADC settings and its linked DMA settings
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 AGH Eko-Energy.
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

/* Includes ----------------------------------------------------------------------------*/
#include "main.h"
#include "stm32_family.h"


/* Universal Macros (Object Type)------------------------------------------------------ */
#define 			SQR_1				    1
#define 			SQR_2				    2
#define 			SQR_3				    3
#define				SQR_4					4

#define 			ADC_MAX_CHANNELS       16											// maximum number of ADC's channels
#define 			ADC_AVERAGED_MEASURES  5											// common number of measures from one channel to be averaged
#define 			ADC_BUFF_SIZE 		   (ADC_AVERAGED_MEASURES * ADC_MAX_CHANNELS)	// ADC Buffers' Size, includes size of all channels and their number of averaged measures

/* Variables--------------------------------------------------------------------------- */
static volatile int ADC_CONVERTED_CHANNELS =  4;	// default value of converted which will be overwrite by program in runtime after auto-detect process



/* Typedefs --------------------------------------------------------------------------- */
/**
  * @brief  DMA buffer typedef for ADCs in dual mode
  */
typedef union{
		uint32_t BufferMultiMode [ADC_BUFF_SIZE];
		uint16_t BufferADC_Master[ADC_BUFF_SIZE];
		uint16_t BufferADC_Slave [ADC_BUFF_SIZE];

}DMA_DualmodeBufferTypeDef;

/**
  * @brief  DMA buffer typedef for ADCs in independent mode
  */
typedef union{
	uint16_t BufferADC[ADC_BUFF_SIZE];

}DMA_IndependentModeBufferTypeDef;

typedef struct{

	DMA_DualmodeBufferTypeDef 		 ddma;				// dma buffer for dual mode

	DMA_IndependentModeBufferTypeDef idma;				// dma buffer for independent mode

	uint32_t ADC_Buff[ADC_MAX_CHANNELS];

}ADC_BufferTypeDef;


/**
  * @brief  ADC's channels' ranks definition
  */
typedef struct{

	uint8_t ranks[ADC_MAX_CHANNELS];					// Channels for all ranks | auto detect

}ADC_ChannelsTypeDef;



/* Private Macros ------------------------------------------------------------------- */
#if defined(STM32F1_FAMILY)

	/* Function-type macros for core of F1 family------------------------------------ */
	#define __ADC_IS_DMA_MULTIMODE(__HANDLE__)                                              												\
											((((((__HANDLE__)->Instance->CR1      >> ADC_CR1_DUALMOD_Pos) & 0xF) == 0U) ? 0U : 1U))

	#define __ADC_IS_CONV_STARTED(__HANDLE__)                                               												\
											(((((__HANDLE__)->Instance->SR)       >> ADC_SR_STRT_Pos ) & 0x1U))

	#define __ADC_IS_DMA_ENABLED(__HANDLE__)                                                												\
											((((__HANDLE__)->DMA_Handle == NULL)   ? 0 : 1))

	#define __ADC_RESOLUTION(__HANDLE__)                                                    												\
											(4095U)

	#define __ADC_DMA_MODE(__HANDLE__)                                                      												\
											(((((__HANDLE__)->DMA_Handle->Instance->CCR >> DMA_CCR_CIRC_Pos) & 0x1U))? 0:1)

	#define __ADC_EOC(__HANDLE__)                                                           												\
											((((__HANDLE__)->Instance->SR          >> ADC_SR_EOC_Pos) & 0x1U))

	#define __ADC_MODE(__HANDLE__)                                                          												\
											((((__HANDLE__)->Instance->CR2         >> ADC_CR2_CONT_Pos) & 0x1U))


	// ====================================================================
	// RANK DEFINITIONS (RANK 1 do RANK 16)
	// ====================================================================

	// RANKS 1 - 6 (Register SQR3)
	// --------------------------------------------------------------------
	#define ADC_RANK1_REG                   SQR_3
	#define ADC_RANK1_BITPOS                ADC_SQR3_SQ1_Pos

	#define ADC_RANK2_REG                   SQR_3
	#define ADC_RANK2_BITPOS                ADC_SQR3_SQ2_Pos

	#define ADC_RANK3_REG                   SQR_3
	#define ADC_RANK3_BITPOS                ADC_SQR3_SQ3_Pos

	#define ADC_RANK4_REG                   SQR_3
	#define ADC_RANK4_BITPOS                ADC_SQR3_SQ4_Pos

	#define ADC_RANK5_REG                   SQR_3
	#define ADC_RANK5_BITPOS                ADC_SQR3_SQ5_Pos

	#define ADC_RANK6_REG                   SQR_3
	#define ADC_RANK6_BITPOS                ADC_SQR3_SQ6_Pos


	// RANKS 7 - 12 (Register SQR2)
	// --------------------------------------------------------------------
	#define ADC_RANK7_REG                   SQR_2
	#define ADC_RANK7_BITPOS                ADC_SQR2_SQ7_Pos

	#define ADC_RANK8_REG                   SQR_2
	#define ADC_RANK8_BITPOS                ADC_SQR2_SQ8_Pos

	#define ADC_RANK9_REG                   SQR_2
	#define ADC_RANK9_BITPOS                ADC_SQR2_SQ9_Pos

	#define ADC_RANK10_REG                  SQR_2
	#define ADC_RANK10_BITPOS               ADC_SQR2_SQ10_Pos

	#define ADC_RANK11_REG                  SQR_2
	#define ADC_RANK11_BITPOS               ADC_SQR2_SQ11_Pos

	#define ADC_RANK12_REG                  SQR_2
	#define ADC_RANK12_BITPOS               ADC_SQR2_SQ12_Pos


	// RANKS 13 - 16 (Register SQR1)
	// --------------------------------------------------------------------
	#define ADC_RANK13_REG                  SQR_1
	#define ADC_RANK13_BITPOS               ADC_SQR1_SQ13_Pos

	#define ADC_RANK14_REG                  SQR_1
	#define ADC_RANK14_BITPOS               ADC_SQR1_SQ14_Pos

	#define ADC_RANK15_REG                  SQR_1
	#define ADC_RANK15_BITPOS               ADC_SQR1_SQ15_Pos

	#define ADC_RANK16_REG                  SQR_1
	#define ADC_RANK16_BITPOS               ADC_SQR1_SQ16_Pos

	// Number of Converted Channels
	#define ADC_NBR_OF_CONVERSIONS_REG		SQR_1
	#define ADC_NBR_OF_CONVERSIONS_BITPOS	ADC_SQR1_L_Pos


#elif defined(STM32F2_FAMILY)

	/* Macros Function type for core of F2 family-------------------------------------- */

	#include "stm32f2xx.h"					// Including lib, which contains READ_REG and READ_BIT functions

	#define __ADC_IS_DMA_MULTIMODE(__HANDLE__) 																											\
		    							    ((READ_BIT(ADC->CCR, ADC_CCR_MULTI) == 0U) ? 0U : 1U)

	#define __ADC_IS_CONV_STARTED(__HANDLE__)                                               												\
											(((((__HANDLE__)->Instance->SR) >> ADC_SR_STRT_Pos) & 0x1U))

	#define __ADC_IS_DMA_ENABLED(__HANDLE__)                                                												\
											(((((__HANDLE__)->Instance->CR2) >> ADC_CR2_DMA_Pos) & 0x1U))

	#define __ADC_RESOLUTION(__HANDLE__)                                                    												\
											(((((__HANDLE__)->Instance->CR1 >> ADC_CR1_RES_Pos) & 0b11) == 0b00) ? 4095U : 				    \
											 ((((__HANDLE__)->Instance->CR1 >> ADC_CR1_RES_Pos) & 0b11) == 0b01) ? 1023U : 				    \
											 ((((__HANDLE__)->Instance->CR1 >> ADC_CR1_RES_Pos) & 0b11) == 0b10) ? 255U  : 63U )

	#define __ADC_DMA_MODE(__HANDLE__)                                                      												\
											(((__HANDLE__)->DMA_Handle->Init.Mode == DMA_NORMAL) ? 0U : 1U)

	#define __ADC_EOC(__HANDLE__)                                                           												\
											((((__HANDLE__)->Instance->SR >> ADC_SR_EOC_Pos) & 0x1U))

	#define __ADC_MODE(__HANDLE__)                                                          												\
											((((__HANDLE__)->Instance->CR2 >> ADC_CR2_CONT_Pos) & 0x1U))

	// ====================================================================
	// RANK DEFINITIONS (RANK 1 do RANK 16)
	// ====================================================================

	// RANKS 1 - 6 (Register SQR3)
	// --------------------------------------------------------------------
	#define ADC_RANK1_REG                   SQR_3
	#define ADC_RANK1_BITPOS                ADC_SQR3_SQ1_Pos

	#define ADC_RANK2_REG                   SQR_3
	#define ADC_RANK2_BITPOS                ADC_SQR3_SQ2_Pos

	#define ADC_RANK3_REG                   SQR_3
	#define ADC_RANK3_BITPOS                ADC_SQR3_SQ3_Pos

	#define ADC_RANK4_REG                   SQR_3
	#define ADC_RANK4_BITPOS                ADC_SQR3_SQ4_Pos

	#define ADC_RANK5_REG                   SQR_3
	#define ADC_RANK5_BITPOS                ADC_SQR3_SQ5_Pos

	#define ADC_RANK6_REG                   SQR_3
	#define ADC_RANK6_BITPOS                ADC_SQR3_SQ6_Pos


	// RANKS 7 - 12 (Register SQR2)
	// --------------------------------------------------------------------
	#define ADC_RANK7_REG                   SQR_2
	#define ADC_RANK7_BITPOS                ADC_SQR2_SQ7_Pos

	#define ADC_RANK8_REG                   SQR_2
	#define ADC_RANK8_BITPOS                ADC_SQR2_SQ8_Pos

	#define ADC_RANK9_REG                   SQR_2
	#define ADC_RANK9_BITPOS                ADC_SQR2_SQ9_Pos

	#define ADC_RANK10_REG                  SQR_2
	#define ADC_RANK10_BITPOS               ADC_SQR2_SQ10_Pos

	#define ADC_RANK11_REG                  SQR_2
	#define ADC_RANK11_BITPOS               ADC_SQR2_SQ11_Pos

	#define ADC_RANK12_REG                  SQR_2
	#define ADC_RANK12_BITPOS               ADC_SQR2_SQ12_Pos


	// RANKS 13 - 16 (Register SQR1)
	// --------------------------------------------------------------------
	#define ADC_RANK13_REG                  SQR_1
	#define ADC_RANK13_BITPOS               ADC_SQR1_SQ13_Pos

	#define ADC_RANK14_REG                  SQR_1
	#define ADC_RANK14_BITPOS               ADC_SQR1_SQ14_Pos

	#define ADC_RANK15_REG                  SQR_1
	#define ADC_RANK15_BITPOS               ADC_SQR1_SQ15_Pos

	#define ADC_RANK16_REG                  SQR_1
	#define ADC_RANK16_BITPOS               ADC_SQR1_SQ16_Pos

	// Number of Converted Channels
	#define ADC_NBR_OF_CONVERSIONS_REG		SQR_1
	#define ADC_NBR_OF_CONVERSIONS_BITPOS	ADC_SQR1_L_Pos

#elif defined(STM32F3_FAMILY)

	#include "stm32f3xx.h"			// Including lib, which contains READ_REG and READ_BIT functions

	#define  ADC_CCR_OFFSET 0x300	// CCR reg address offset from base ADC1 address


	/* Macros Function type for core of F3 family-------------------------------------- */
	#define __ADC_IS_DMA_MULTIMODE(__HANDLE__) \
											((READ_BIT(*(volatile uint32_t *)(ADC1_BASE + ADC_CCR_OFFSET), ADC_CCR_DUAL_Msk) != 0U) ? 1U : 0U)

	#define __ADC_IS_CONV_STARTED(__HANDLE__)                                               												\
											(((((__HANDLE__)->Instance->CR >> ADC_CR_ADSTART_Pos) & 0x1U)))

	#define __ADC_IS_DMA_ENABLED(__HANDLE__)                                                												\
											((READ_BIT((__HANDLE__)->Instance->CFGR, ADC_CFGR_DMAEN)))

	#define __ADC_RESOLUTION(__HANDLE__)                                                    												\
											(((((__HANDLE__)->Instance->CFGR >> ADC_CFGR_RES_Pos) & 0b11) == 0b00) ? 4095U : 			    \
											 ((((__HANDLE__)->Instance->CFGR >> ADC_CFGR_RES_Pos) & 0b11) == 0b01) ? 1023U : 			    \
											 ((((__HANDLE__)->Instance->CFGR >> ADC_CFGR_RES_Pos) & 0b11) == 0b10) ? 255U  : 63U )

	#define __ADC_DMA_MODE(__HANDLE__)                                                      												\
											(((((__HANDLE__)->Instance->CFGR >> ADC_CFGR_DMACFG_Pos) & 0x1U)))

	#define __ADC_EOC(__HANDLE__)                                                           												\
											((((__HANDLE__)->Instance->ISR >> ADC_ISR_EOC_Pos) & 0x1U))

	#define __ADC_MODE(__HANDLE__)                                                          												\
											((((__HANDLE__)->Instance->CFGR >> ADC_CFGR_CONT_Pos) & 0x1U))

	// ====================================================================
	// RANK DEFINITIONS (RANK 1 do RANK 16)
	// ====================================================================

	// RANKS 1 - 4 (Register SQR1)
	// --------------------------------------------------------------------
	#define ADC_RANK1_REG                   SQR_1
	#define ADC_RANK1_BITPOS                ADC_SQR1_SQ1_Pos

	#define ADC_RANK2_REG                   SQR_1
	#define ADC_RANK2_BITPOS                ADC_SQR1_SQ2_Pos

	#define ADC_RANK3_REG                   SQR_1
	#define ADC_RANK3_BITPOS                ADC_SQR1_SQ3_Pos

	#define ADC_RANK4_REG                   SQR_1
	#define ADC_RANK4_BITPOS                ADC_SQR1_SQ4_Pos


	// RANKS 5 - 9 (Register SQR2)
	// --------------------------------------------------------------------
	#define ADC_RANK5_REG                   SQR_2
	#define ADC_RANK5_BITPOS                ADC_SQR2_SQ5_Pos

	#define ADC_RANK6_REG                   SQR_2
	#define ADC_RANK6_BITPOS                ADC_SQR2_SQ6_Pos

	#define ADC_RANK7_REG                   SQR_2
	#define ADC_RANK7_BITPOS                ADC_SQR2_SQ7_Pos

	#define ADC_RANK8_REG                   SQR_2
	#define ADC_RANK8_BITPOS                ADC_SQR2_SQ8_Pos

	#define ADC_RANK9_REG                   SQR_2
	#define ADC_RANK9_BITPOS                ADC_SQR2_SQ9_Pos


	// RANKS 10 - 14 (Register SQR3)
	// --------------------------------------------------------------------
	#define ADC_RANK10_REG                  SQR_3
	#define ADC_RANK10_BITPOS               ADC_SQR3_SQ10_Pos

	#define ADC_RANK11_REG                  SQR_3
	#define ADC_RANK11_BITPOS               ADC_SQR3_SQ11_Pos

	#define ADC_RANK12_REG                  SQR_3
	#define ADC_RANK12_BITPOS               ADC_SQR3_SQ12_Pos

	#define ADC_RANK13_REG                  SQR_3
	#define ADC_RANK13_BITPOS               ADC_SQR3_SQ13_Pos

	#define ADC_RANK14_REG                  SQR_3
	#define ADC_RANK14_BITPOS               ADC_SQR3_SQ14_Pos


	// RANKS 15 - 16 (Register SQR4)
	// --------------------------------------------------------------------
	#define ADC_RANK15_REG                  SQR_4
	#define ADC_RANK15_BITPOS               ADC_SQR4_SQ15_Pos

	#define ADC_RANK16_REG                  SQR_4
	#define ADC_RANK16_BITPOS               ADC_SQR4_SQ16_Pos

	// Number of Converted Channels
	#define ADC_NBR_OF_CONVERSIONS_REG		SQR_1
	#define ADC_NBR_OF_CONVERSIONS_BITPOS	ADC_SQR1_L_Pos


#elif defined(STM32F4_FAMILY)

	#include "stm32f4xx.h"					// Including lib, which contains READ_REG and READ_BIT functions

	/* Macros Function type for core of F4 family-------------------------------------- */
	#define __ADC_IS_DMA_MULTIMODE(__HANDLE__) 																									\
		    							    ((READ_BIT(ADC->CCR, ADC_CCR_MULTI) == 0U) ? 0U : 1U)

	#define __ADC_IS_CONV_STARTED(__HANDLE__)                                               												\
											(((((__HANDLE__)->Instance->SR) >> ADC_SR_STRT_Pos) & 0x1U))

	#define __ADC_IS_DMA_ENABLED(__HANDLE__)                                                												\
											(((((__HANDLE__)->Instance->CR2) >> ADC_CR2_DMA_Pos) & 0x1U))

	#define __ADC_RESOLUTION(__HANDLE__)                                                    												\
											(((((__HANDLE__)->Instance->CR1 >> ADC_CR1_RES_Pos) & 0b11) == 0b00) ? 4095U : 				    \
											 ((((__HANDLE__)->Instance->CR1 >> ADC_CR1_RES_Pos) & 0b11) == 0b01) ? 1023U : 				    \
											 ((((__HANDLE__)->Instance->CR1 >> ADC_CR1_RES_Pos) & 0b11) == 0b10) ? 255U  : 63U )

	#define __ADC_DMA_MODE(__HANDLE__)                                                      												\
											(((__HANDLE__)->DMA_Handle->Init.Mode == DMA_NORMAL) ? 0U : 1U)

	#define __ADC_EOC(__HANDLE__)                                                           												\
											((((__HANDLE__)->Instance->SR >> ADC_SR_EOC_Pos) & 0x1U))

	#define __ADC_MODE(__HANDLE__)                                                          												\
											((((__HANDLE__)->Instance->CR2 >> ADC_CR2_CONT_Pos) & 0x1U))

	// ====================================================================
	// RANK DEFINITIONS (RANK 1 do RANK 16)
	// ====================================================================

	// RANKS 1 - 6 (Register SQR3)
	// --------------------------------------------------------------------
	#define ADC_RANK1_REG                   SQR_3
	#define ADC_RANK1_BITPOS                ADC_SQR3_SQ1_Pos

	#define ADC_RANK2_REG                   SQR_3
	#define ADC_RANK2_BITPOS                ADC_SQR3_SQ2_Pos

	#define ADC_RANK3_REG                   SQR_3
	#define ADC_RANK3_BITPOS                ADC_SQR3_SQ3_Pos

	#define ADC_RANK4_REG                   SQR_3
	#define ADC_RANK4_BITPOS                ADC_SQR3_SQ4_Pos

	#define ADC_RANK5_REG                   SQR_3
	#define ADC_RANK5_BITPOS                ADC_SQR3_SQ5_Pos

	#define ADC_RANK6_REG                   SQR_3
	#define ADC_RANK6_BITPOS                ADC_SQR3_SQ6_Pos


	// RANKS 7 - 12 (Register SQR2)
	// --------------------------------------------------------------------
	#define ADC_RANK7_REG                   SQR_2
	#define ADC_RANK7_BITPOS                ADC_SQR2_SQ7_Pos

	#define ADC_RANK8_REG                   SQR_2
	#define ADC_RANK8_BITPOS                ADC_SQR2_SQ8_Pos

	#define ADC_RANK9_REG                   SQR_2
	#define ADC_RANK9_BITPOS                ADC_SQR2_SQ9_Pos

	#define ADC_RANK10_REG                  SQR_2
	#define ADC_RANK10_BITPOS               ADC_SQR2_SQ10_Pos

	#define ADC_RANK11_REG                  SQR_2
	#define ADC_RANK11_BITPOS               ADC_SQR2_SQ11_Pos

	#define ADC_RANK12_REG                  SQR_2
	#define ADC_RANK12_BITPOS               ADC_SQR2_SQ12_Pos


	// RANKS 13 - 16 (Register SQR1)
	// --------------------------------------------------------------------
	#define ADC_RANK13_REG                  SQR_1
	#define ADC_RANK13_BITPOS               ADC_SQR1_SQ13_Pos

	#define ADC_RANK14_REG                  SQR_1
	#define ADC_RANK14_BITPOS               ADC_SQR1_SQ14_Pos

	#define ADC_RANK15_REG                  SQR_1
	#define ADC_RANK15_BITPOS               ADC_SQR1_SQ15_Pos

	#define ADC_RANK16_REG                  SQR_1
	#define ADC_RANK16_BITPOS               ADC_SQR1_SQ16_Pos

	// Number of Converted Channels
	#define ADC_NBR_OF_CONVERSIONS_REG		SQR_1
	#define ADC_NBR_OF_CONVERSIONS_BITPOS	ADC_SQR1_L_Pos


#endif


/* Functions Prototypes --------------------------------------------------------------------  */
HAL_StatusTypeDef          ADC_Init(ADC_HandleTypeDef* hadc, ADC_BufferTypeDef* badc, ADC_ChannelsTypeDef* cadc);

HAL_StatusTypeDef          ADC_InitMultimode(ADC_HandleTypeDef* hadcMaster, ADC_BufferTypeDef* badc);

HAL_StatusTypeDef          ADC_ReadChannel(ADC_HandleTypeDef* hadc, ADC_ChannelsTypeDef* cadc, ADC_BufferTypeDef* badc, uint8_t channel, uint16_t*  retval);

__weak HAL_StatusTypeDef   ADC_GetValue(ADC_HandleTypeDef* hadc, ADC_ChannelsTypeDef* cadc, ADC_BufferTypeDef* badc, float max, uint8_t channel, float * retval);

HAL_StatusTypeDef          ADC_ConfigGetRanksOfChannels(ADC_HandleTypeDef* hadc, ADC_ChannelsTypeDef* cadc, ADC_BufferTypeDef* badc);

HAL_StatusTypeDef          ADC_GetRank(ADC_ChannelsTypeDef *cadc, uint8_t channel, uint8_t* rank);

HAL_StatusTypeDef          ADC_Averaging(ADC_HandleTypeDef* hadc, ADC_BufferTypeDef* badc, ADC_ChannelsTypeDef* cadc, uint8_t channel , uint16_t* retval);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADC_DRIVER_H_ */
