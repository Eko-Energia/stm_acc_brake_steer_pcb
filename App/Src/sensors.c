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

/**
 * @brief  Maps the LPF ADC reading to signed rack travel in whole millimetres.
 * @details Uses the same mm-per-count scale as the full ADC/mm calibration, then
 * offsets by @ref LFP_CENTER so that ADC reading returns 0 mm and travel is
 * +/- around it. Out-of-range ADC values are clamped to the calibrated ends.
 *
 * @return Displacement from the centred rack [mm].
 */
int16_t Get_SteeringValue(void)
{
    uint32_t adc_reading = ADC1_VAL[2];

    if (adc_reading < LPF_SENSOR_ADC_MIN_VALUE) {
        adc_reading = LPF_SENSOR_ADC_MIN_VALUE;
    }
    else if (adc_reading > LPF_SENSOR_ADC_MAX_VALUE) {
        adc_reading = LPF_SENSOR_ADC_MAX_VALUE;
    }

    const int32_t adc_span = (int32_t)LPF_SENSOR_ADC_MAX_VALUE
            - (int32_t)LPF_SENSOR_ADC_MIN_VALUE;
    const int32_t mm_span = (int32_t)LPF_SENSOR_MM_MAX_VALUE
            - (int32_t)LPF_SENSOR_MM_MIN_VALUE;
    const int32_t adc_delta = (int32_t)adc_reading - (int32_t)LFP_CENTER;
    const int32_t round = (adc_delta >= 0) ? (adc_span / 2) : -(adc_span / 2);

    return (int16_t)((adc_delta * mm_span + round) / adc_span);
}
