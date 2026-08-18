/**
  ******************************************************************************
  * @file    adc_utils.h
  * @author  Bartosz Rychlicki
  * @author  AGH Eko-Energia

  * @Title   Universal driver for ADC peripheral (Built for F1, F2, F3 and F4 families, but can be implemented for all families)

  * @brief   This file contains common defines, flags and macros that are used to extract ADC configuration from ADC registers
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 AGH Eko-Energy.
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef ADC_UTILS_H_
#define ADC_UTILS_H_

#pragma once

/* Includes ------------------------------------------------------------------------------------------------ */
#include "stm32_family.h"
#include "main.h"


/* Macros -------------------------------------------------------------------------------------------------- */

/*
 * @defgroup ADC Macros, which definitions depends on type of MCU used in application layer
 * @brief    ADC macros which define offsets between SQRx regs while reading channels configuration for regular group
 */
#if defined(STM32F1_FAMILY)

    #define ADC_SQR3_2_OFFSET     (6)        //< Offset in reading SQR3 reg of ADC - used in shifting bits while reading ranks 1 - 6, cause for loop would trigger end of bound error
    #define ADC_SQR3_1_OFFSET     (12)       //< Offset in reading SQR2 reg of ADC - used in shifting bits while reading ranks 7 - 12, cause for loop would trigger end of bound error
    #define ADC_SQR_DATA_RES      (5)        //< Resolution of channel number in regs - generally 5 bits of data

#elif defined(STM32F3_FAMILY)

    #define ADC_SQR1_2_OFFSET     (4)        //< Offset in reading SQR1 reg of ADC - used in shifting bits while reading ranks 1 - 4, cause for loop would trigger end of bound error
    #define ADC_SQR1_3_OFFSET     (9)        //< Offset in reading SQR3 reg of ADC - used in shifting bits while reading ranks 5 - 9, cause for loop would trigger end of bound error
    #define ADC_SQR1_4_OFFSET     (14)       //< Offset in reading SQR3 reg of ADC - used in shifting bits while reading ranks 10 - 14, cause for loop would trigger end of bound error
    #define ADC_SQR_DATA_RES      (6)        //< Resolution of channel number in regs - generally 5 bits of data
    #define ADC_SQR1_L_OFFSET     (1)        //< Offset in reading SQR1 reg of ADC, because on first bits of SQR1 L data is stored
    #define ADC_CFGR_RES_Mask     (0x3)      //< Data resolution of stored ADC's resolution param

#endif


/*
 * @defgroup ADC Macros, which definitions does not depend on type of MCU used in application layer
 * @brief    ADC macros which define provides correct casting of values read from ADC registers and polling timeout, etc.
 */
#define ADC_SQR_DATA_Msk          (0x1F)     //< Mask for channel number in regs - generally 4:0 bits of data so 11111
#define ADC_SQR_L_DATA_res        (0xF)      //< Resolution of L Data in SQRx regs of ADC
#define ADC_CONT_Mask			  (1)        //< Mask for reading continuous flag state
#define ADC_MAX_RANKS_NUMBER      (16)       //< Maximum number of ADC ranks
#define ADC_POLLING_TIMEOUT       (10)       //< Macro stores max ADC polling timeout, to prevent endless blocking by polling


/*
 * @defgroup DMA Macros, which definitions are strongly related to DMA regs reading mode status
 * @brief    DMA macros which define provides correct checking DMA mode
 */
#define DMA_MODE_NORMAL			  (0)        //< DMA Mode definition - normal: every time DMA request polling for ADC
#define DMA_MODE_CIRCULAR		  (1)        //< DMA Mode definition - circular:  DMA automatically request polling for ADC
#define DMA_MODE_Mask             (1)        //< DMA mode mask to provide correct casting while reading DMA's regs

/*
 * @defgroup MCUs Macros, which definitions are strongly related to MCUs physical parameters
 * @brief    MCUs macros which define provides correct reading voltage on exact pin
 */
#define STM32_VCC                 (3.3f)	 //< STM32 supply voltage for reading ADC's pins' voltages
#define STM32_GND                 (0.0f)     //< STM32 ground voltage for providing reference while reading ADC's pins' voltages




/* Exported variables --------------------------------------------------------------------------------------- */
extern ADC_HandleTypeDef hadc1;



/* Functions' prototypes ------------------------------------------------------------------------------------ */

/*
 * @ brief Function that returns binary type value of ADC Mode according to table presented below
 *
 * Binary Value | Description
   0000         | Independent mode.
   0001         | Combined regular simultaneous + injected simultaneous mode
   0010         | Combined regular simultaneous + alternate trigger mode
   0011         | Combined injected simultaneous + fast interleaved mode
   0100         | Combined injected simultaneous + slow Interleaved mode
   0101         | Injected simultaneous mode only
   0110         | Regular simultaneous mode only
   0111         | Fast interleaved mode only
   1000         | Slow interleaved mode only
   1001         | Alternate trigger mode only


 * @param hadc - pointer to ADC handle
 * @retval binary type value similar to one of thosee, presented in table above
 */
uint8_t ADC_GetMode(ADC_HandleTypeDef* hadc);

/*
 * @ brief Function that returns binary type value of ADC Continuous conversion mode
 * @ note  Continuous conversion mode on ADC indicates if ADC re-launch after sampling sequence of all channels
     or if HAL_ADC_Start should be called every time another  sampling sequence should be launched

 * @param hadc - pointer to ADC handle
 * @ return value:
   - DISABLED - 0
   - ENABLED  - 1
 */
uint8_t ADC_Continuous(ADC_HandleTypeDef* hadc);

/*
 * @ brief Function that returns binary type value of ADC Discontinuous conversion mode
 * @ note  Discontinuous conversion mode on ADC indicates how many channels does ADC scan during single launch by HAL_ADC_Start

 * @param hadc - pointer to ADC handle
 * @ return value:
   - DISABLED - 0
   - ENABLED  - 1
 */
uint8_t ADC_Discontinuous(ADC_HandleTypeDef* hadc);

/*
 * @ brief Function that returns binary type value of DMA status for given ADC
 * @ note  Function returns uint8_t, but its logic returns ENABLE or DISABLE as far as mentiooned states are 0 or 1, then uint8_t provides correct casting

 * @param hadc - pointer to ADC handle
 * @ return value:
   - DISABLED - 0
   - ENABLED  - 1
 */
uint8_t ADC_DMA_ENABLED(ADC_HandleTypeDef* hadc);

/*
 * @ brief Function that returns binary type value of ADC's Resolution
 * @ note  Function returns uint16_t, because 12-bit resolution cannot be written on data types, like int8_t, etc.

 * @param hadc - pointer to ADC handle
 * @ return value:
   - 4095 - 12 Bits Resolution
   - 1023 - 10 Bits Resolution
   - 255  - 8  Bits Resolution
   - 63   - 6  Bits Reseolution
   - 15   - 4  Bits Reseolution

 */
uint16_t ADC_Resolution(ADC_HandleTypeDef* hadc);

/*
 * @ brief Function that returns binary type of DMA mode

 * @param hadc - pointer to ADC handle
 * @ return value:
   - 0 - Normal Mode
   - 1 - Circular Mode

 */
uint8_t ADC_DMA_GetMode(ADC_HandleTypeDef* hadc);

#endif /* ADC_UTILS_H_ */
