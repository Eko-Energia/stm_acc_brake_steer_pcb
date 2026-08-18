/**
  ******************************************************************************
  * @file    adc_utils.c
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


#include "adc_utils.h"

/* Functions' bodies ------------------------------------------------------------------------------------ */

uint8_t ADC_GetMode(ADC_HandleTypeDef* hadc){

	// checking if hadc is a NULL to prevent launching incorrect operations
	if(hadc == NULL){
		return 0xFF;
	}

	// extracting data of registers, correct for F1 family
	#if defined(STM32F1_FAMILY)

		// returning CR1 dual mode data value to local variable
        return (uint8_t)(hadc->Instance->CR1 >> ADC_CR1_DUALMOD_Pos);

    #elif defined(STM32F3_FAMILY)

        return (((ADC12_COMMON)->CCR >> ADC_CCR_DUAL_Pos)& 0x1F);

	#endif

    // returning incorrect value if target was not detected
	return 0xFF;
}


uint8_t ADC_Continuous(ADC_HandleTypeDef* hadc){

	// checking if hadc is a NULL to prevent launching incorrect operations
	if(hadc == NULL){
		return 0xFF;
	}

	// extracting data of registers, correct for F1 family

	#if defined(STM32F1_FAMILY)

	    // returning CR2 continuous conversion data value to local variable
	    return ((uint8_t)((hadc->Instance->CR2 >> ADC_CR2_CONT_Pos) & ADC_CONT_Mask) == DISABLE) ? DISABLE : ENABLE;

	#elif defined(STM32F3_FAMILY)

	    return ((uint8_t)((hadc->Instance->CFGR >> ADC_CFGR_CONT_Pos ) & ADC_CONT_Mask) == DISABLE) ? DISABLE : ENABLE;

	#endif

	 // returning incorrect value if target was not detected
	return 0xFF;
}

uint8_t ADC_Discontinuous(ADC_HandleTypeDef* hadc){

	// checking if hadc is a NULL to prevent launching incorrect operations
	if(hadc == NULL){
		return 0xFF;
	}

	// extracting data of registers, correct for F1 family
	#if defined(STM32F1_FAMILY)

	    // returning CR2 continuous conversion data value to local variable
	    return (uint8_t)((hadc->Instance->CR1 >> ADC_CR1_DISCEN_Pos) & 0x1);

    #elif defined(STM32F3_FAMILY)

	    return (((hadc->Instance->CFGR >> ADC_CFGR_DISCEN_Pos) & 0x1) ? ENABLE : DISABLE);

	#endif

	// returning incorrect value if target was not detected
	return 0xFF;

}


uint8_t ADC_DMA_ENABLED(ADC_HandleTypeDef* hadc){

	// Checking if correct pointer was passed
	if(hadc == NULL){
		return 0xFF;
	}

    #if defined(STM32F1_FAMILY) || defined(STM32F3_FAMILY)
        return (hadc->DMA_Handle == NULL) ? DISABLE : ENABLE;
    #endif

    // error state
    return 0xFF;
}

uint16_t ADC_Resolution(ADC_HandleTypeDef* hadc){

   // checking if correct pointer was given
   if(hadc == NULL){
	   return 0xFFFF;
   }

   // executing command for F1 family
   #if defined(STM32F1_FAMILY)

       // because F1 MCUs does not have settings to change their resolution, they have constant value 4095 of resolution
       return (4095);

   #elif defined(STM32F3_FAMILY)

       uint8_t res = (uint16_t)((hadc->Instance->CFGR >> ADC_CFGR_RES_Pos) & ADC_CFGR_RES_Mask);

       return ((res == 0x0) ? 4095 : ((res == 0x1) ? 1023 : ((res == 0x2) ? 255 : 63)));

   #endif


  // error state
  return 0xFFFF;
}

uint8_t ADC_DMA_GetMode(ADC_HandleTypeDef* hadc){

	#if defined(STM32F1_FAMILY)

		return (((hadc->DMA_Handle->Instance->CCR >> DMA_CCR_CIRC_Pos) & DMA_MODE_Mask) == 0) ? DMA_MODE_NORMAL : DMA_MODE_CIRCULAR;

	#elif defined(STM32F3_FAMILY)

		return (((hadc->DMA_Handle->Instance->CCR >> DMA_CCR_CIRC_Pos) & DMA_MODE_Mask) == 0) ? DMA_MODE_NORMAL : DMA_MODE_CIRCULAR;

	#endif

	// error state
	return 0xFF;
}
