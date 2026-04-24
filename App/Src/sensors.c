/**
  ******************************************************************************
  * @file    sensors.c
  * @brief   Implementation of raw sensor data processing.
  * @author  [Michał Jurek]
  * @date    2025-03-21
 * @details This module handles the physical memory allocation for ADC buffers
 * and provides the logic to average the oversampled analog signals
 * originating from the vehicle's sensors.
 *
  ******************************************************************************
  */


#include "sensors.h"
#include "pedals_const_val.h"
/** * @brief DMA buffer for ADC1 readings.
 * - Index 0: Accelerator Pedal 1
 * - Index 1: Brake Pressure 1
 * - Index 2: Steering Angle
 */
uint16_t ADC1_VAL[AMOUNT_OF_ADC_CHANNELS];

/** * @brief DMA buffer for ADC2 readings (Redundant sensors).
 * - Index 0: Accelerator Pedal 2
 * - Index 1: Brake Pressure 2
 * - Index 2: Brake Hall Sensor
 */
uint16_t ADC2_VAL[AMOUNT_OF_ADC_CHANNELS];

/** * @brief ADC buffers to store 5 samples from each of 3 channels.
 * 	- (3 channels * 5 samples = 15 samples)
 *  */
uint16_t ADC1_DMA_Buff[AMOUNT_OF_ADC_CHANNELS * ADC_SAMPLES];
uint16_t ADC2_DMA_Buff[AMOUNT_OF_ADC_CHANNELS * ADC_SAMPLES];

/* ==================================================================== */
/* Functions                                                            */
/* ==================================================================== */

void Process_ADC_Buffers(void)
{
    uint32_t sum1_ch1 = 0, sum1_ch2 = 0, sum1_ch3 = 0;
    uint32_t sum2_ch1 = 0, sum2_ch2 = 0, sum2_ch3 = 0;

    /* @brief Summing up all samples from the DMA buffers. */
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum1_ch1 += ADC1_DMA_Buff[i * 3 + 0];
        sum1_ch2 += ADC1_DMA_Buff[i * 3 + 1];
        sum1_ch3 += ADC1_DMA_Buff[i * 3 + 2];

        sum2_ch1 += ADC2_DMA_Buff[i * 3 + 0];
        sum2_ch2 += ADC2_DMA_Buff[i * 3 + 1];
        sum2_ch3 += ADC2_DMA_Buff[i * 3 + 2];
    }

    /* @brief Calculating the arithmetic mean and updating global containers. */
    ADC1_VAL[0] = sum1_ch1 / ADC_SAMPLES;
    ADC1_VAL[1] = sum1_ch2 / ADC_SAMPLES;
    ADC1_VAL[2] = sum1_ch3 / ADC_SAMPLES;

    ADC2_VAL[0] = sum2_ch1 / ADC_SAMPLES;
    ADC2_VAL[1] = sum2_ch2 / ADC_SAMPLES;
    ADC2_VAL[2] = sum2_ch3 / ADC_SAMPLES;
}
