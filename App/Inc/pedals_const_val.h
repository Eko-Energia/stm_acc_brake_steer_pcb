/**
 * @file    pedals_const_val.h
 * @brief   Private configuration and calibration constants for the subsystem.
 * @details This file contains raw ADC calibration values, error thresholds,
 * and hardware limits.
 *
 * @author  [Michał Jurek]
 * @date    [31.03.2026]
 */

#ifndef PEDALS_CONST_VAL_H
#define PEDALS_CONST_VAL_H

/**
 * @defgroup ADC_Settings ADC Global Settings
 * @brief    General configuration for Analog-to-Digital Converter.
 * @{
 */

/** @brief Amount of samples taken by ADC before averaging. */
#define ADC_SAMPLES (300)

/**
 * @brief Maximum possible value read from the ADC.
 * @note  Based on 10-bit resolution (2^10 - 1 = 1023).
 */

#define SENSOR_ADC_MAX_VALUE (1023)

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

#define ACCEPT_BRAKE_ERROR (14)

/**
 * @brief Maximum allowed error between two accelerator potentiometers.
 * @note  Unit: 0.1%. Value 14 represents 1.4% deviation.
 */

#define ACCEPT_ACCEL_ERROR (140)

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

#define ACCEL_MIN_VAL (130)

/**
 * @brief Raw ADC value when the accelerator pedal is fully pressed (100%).
 */

#define ACCEL_MAX_VAL (890)

/**
 * @brief Calculated working range of the accelerator pedal.
 * @note  Used for mapping the raw value to a percentage.
 */

#define ACCEL_RANGE (ACCEL_MAX_VAL - ACCEL_MIN_VAL) // result: 770

/** @} */ // End of Accel_Calibration

/**
 * @defgroup Brake_Hall_Calibration Brake Hall Calibration
 * @brief    Raw ADC values defining the physical range of the brake Hall sensor.
 * @note     These values must be updated if the sensor or mechanical mounting changes.
 * @{
 */

/**
 * @brief Raw ADC value when the brake pedal is fully released (0%).
 */
#define BRAKE_HALL_MIN_VAL (166)

/**
 * @brief Raw ADC value when the brake pedal is fully pressed (100%).
 */
#define BRAKE_HALL_MAX_VAL (724)

/**
 * @brief Calculated working range of the brake Hall sensor.
 * @note  Used for mapping the raw value to a percentage.
 */
#define BRAKE_HALL_RANGE (BRAKE_HALL_MAX_VAL - BRAKE_HALL_MIN_VAL) // result: 558

/** @} */ // End of Brake_Hall_Calibration


/**
 * @brief Amount of used ADC channels.
 */
#define AMOUNT_OF_ADC_CHANNELS (3)

/**
 * @defgroup LPF_Sensor_Calibration LPF Sensor Calibration
 * @brief    Raw ADC values defining the physical range of the LPF sensor.
 * @note     These values must be updated if the potentiometer or mechanical mounting changes.
 * @{
 */
#define LPF_SENSOR_ADC_MIN_VALUE (0)
#define LPF_SENSOR_ADC_MAX_VALUE (1023)
#define LPF_SENSOR_MM_MIN_VALUE (0)
#define LPF_SENSOR_MM_MAX_VALUE (175)

#endif /* PEDALS_CONST_H */
