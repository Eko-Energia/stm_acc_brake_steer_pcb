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

/**
 * @defgroup LPF_Sensor_Calibration LPF Sensor Calibration
 * @brief    Raw ADC values defining the physical range of the LPF sensor.
 * @note     These values must be updated if the potentiometer or mechanical mounting changes.
 * @{
 */
#define LPF_SENSOR_ADC_MIN_VALUE (0)
#define LPF_SENSOR_ADC_MAX_VALUE (1007)
#define LPF_SENSOR_MM_MIN_VALUE (0)
#define LPF_SENSOR_MM_MAX_VALUE (174)
/** Raw ADC value at the centred rack, from calibration. Replace with the tested reading. */
#define LFP_CENTER (503)
/** Resolution of Get_SteeringValue(): units per millimetre.
 *  The measured quantity is rack travel in millimetres; 10 is only how finely
 *  it is carried, matching the 0.1 factor the DBC gives the Steering signal of
 *  frame 0x41, so the value goes into that frame as it is. One ADC count is
 *  about 0.17 mm, so the tenths are real resolution rather than padding.
 *  Divide by this constant wherever whole millimetres are wanted. */
#define STEERING_UNITS_PER_MM (10)

/** * @brief   Array containing averaged values from ADC1.
 *
 * @details Index 0: Accelerator Pedal 1
 * Index 1: Brake Pressure 1
 * Index 2: Steering Angle
 */
extern uint16_t ADC1_VAL[3];

/** * @brief   Array containing averaged values from ADC2 (Redundant sensors).
 *
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
 *
 * @details Iterates through the raw samples collected by the DMA controller
 * and computes the arithmetic mean for each channel to filter out
 * signal noise and stabilize the readings.
 * @retval  None
 */
void Process_ADC_Buffers(void);

/**
 * @brief  Maps the LPF steering sensor to signed rack travel.
 * @return Displacement from @ref LFP_CENTER in 0.1 mm units (see @ref STEERING_UNITS_PER_MM).
 */
int16_t Get_SteeringValue(void);

#endif /* SENSORS_H */
