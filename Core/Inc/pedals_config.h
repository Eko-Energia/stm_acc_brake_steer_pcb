/**
 * @file    pedals_config.h
 * @brief   Private configuration and calibration constants for the Pedal subsystem.
 * @details This file contains raw ADC calibration values, error thresholds,
 * and hardware limits. It should ONLY be included by pedal_map.c.
 *
 * @author  [Michał Jurek]
 * @date    [29.01.2026]
 */

#ifndef pedals_const_val
#define pedals_const_val

/**
 * @defgroup ADC_Settings ADC Global Settings
 * @brief    General configuration for Analog-to-Digital Converter.
 * @{
 */

/**
 * @brief Maximum possible value read from the ADC.
 * @note  Based on 10-bit resolution (2^10 - 1 = 1023).
 */

#define SENSOR_ADC_MAX_VALUE 1023

/** @} */ // End of ADC_Settings

/**
 * @defgroup Safety_Limits Safety Thresholds
 * @brief    Maximum allowed deviation between redundant sensors (Implausibility Check).
 * @{
 */

/**
 * @brief Maximum allowed error between two brake pressure sensors.
 * @note  Unit: 0.1%. Value 14 represents 1.4% deviation.
 */

#define ACCEPT_BRAKE_ERROR 14

/**
 * @brief Maximum allowed error between two accelerator potentiometers.
 * @note  Unit: 0.1%. Value 14 represents 1.4% deviation.
 */

#define ACCEPT_ACCEL_ERROR 14

/** @} */ // End of Safety_Limits

/**
 * @defgroup Accel_Calibration Accelerator Calibration
 * @brief    Raw ADC values defining the physical range of the accelerator pedal.
 * @note     These values must be updated if the potentiometer or mechanical mounting changes.
 * @{
 */

/**
 * @brief Raw ADC value when the accelerator pedal is fully released (0%).
 */

#define ACCEL_MIN_VAL 126

/**
 * @brief Raw ADC value when the accelerator pedal is fully pressed (100%).
 */

#define ACCEL_MAX_VAL 896

/**
 * @brief Calculated working range of the accelerator pedal.
 * @note  Used for mapping the raw value to a percentage.
 */

#define ACCEL_RANGE (ACCEL_MAX_VAL - ACCEL_MIN_VAL) // result: 770

/** @} */ // End of Accel_Calibration

#endif /* PEDALS_CONFIG_H */
