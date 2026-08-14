/**
  ******************************************************************************
  * @file    adc_conf.h
  * @author  Bartosz Rychlicki
  * @author  AGH Eko-Energia

  * @Title   ADC Driver configuration file

  * @brief   This file contains defines, flags and macros that should be sey (or configured) by user.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 AGH Eko-Energy.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_ADC_CONF_H_
#define INC_ADC_CONF_H_

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/* Public constant macros -------------------------------------------------------------------------------------------- */

/*
 * ADC1 configuration macros section
 */
#define ADC1_USED_CHANNELS (3)                                          //< Macro defines number of channels for ADC1,
#define ADC1_SAMPLING      (4)                                          //< Macro defines number of measures of one channel to be averaged
#define ADC1_BUFFER_SIZE   (ADC1_USED_CHANNELS * ADC1_SAMPLING) 		//< Macro stores data buffer length

/*
 * ADC2 configuration macros section
 */
#define ADC2_USED_CHANNELS (3)  										//< Macro defines number of channels for ADC2,
#define ADC2_SAMPLING      (4)  										//< Macro defines number of measures of one channel to be averaged
#define ADC2_BUFFER_SIZE   (ADC2_USED_CHANNELS * ADC2_SAMPLING) 		//< Macro stores data buffer length


#ifdef __cplusplus
}
#endif

#endif /* INC_ADC_CONF_H_ */
