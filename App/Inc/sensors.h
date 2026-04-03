/**
  ******************************************************************************
  * @file    sensors.h
  * @brief   Header file for raw sensor data acquisition and processing.
  * @author  [Michał Jurek]
  * @date    2025-03-31
  * @details Contains external declarations for ADC DMA buffers and processed sensor
  * values (Accelerator, Brake Pressure, Steering Angle, etc.).
  ******************************************************************************
  */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "main.h" // Required for ADC_SAMPLES and other system macros

/** * @brief   Array containing averaged values from ADC1.
 * @details Index 0: Accelerator Pedal 1
 * Index 1: Brake Pressure 1
 * Index 2: Steering Angle
 */
extern uint16_t ADC1_VAL[3];

/** * @brief   Array containing averaged values from ADC2 (Redundant sensors).
 * @details Index 0: Accelerator Pedal 2
 * Index 1: Brake Pressure 2
 * Index 2: Brake Hall Sensor
 */
extern uint16_t ADC2_VAL[3];

/** * @brief   DMA buffer for ADC1 raw samples.
 * @note    Actual size is defined in sensors.c as [3 * ADC_SAMPLES].
 */
extern uint16_t ADC1_DMA_Buff[];

/** * @brief   DMA buffer for ADC2 raw samples.
 * @note    Actual size is defined in sensors.c as [3 * ADC_SAMPLES].
 */
extern uint16_t ADC2_DMA_Buff[];

/**
 * @brief   Processes raw ADC DMA buffers to calculate average sensor values.
 * @details Iterates through the raw samples collected by the DMA controller
 * and computes the arithmetic mean for each channel to filter out
 * signal noise and stabilize the readings.
 * @retval  None
 */
void Process_ADC_Buffers(void);

#endif /* SENSORS_H */
