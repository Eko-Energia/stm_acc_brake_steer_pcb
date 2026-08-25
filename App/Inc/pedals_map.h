/**
  ******************************************************************************
  * @file    pedals_map.h
  * @brief   Header file for pedal and steering signal processing.
  * @author  [Michał Jurek]
  * @date    2025-01-29
  * @details This file declares the interface for:
  * - Redundancy checks for dual-sensor systems (Accelerator, Brake Pressure).
  * - Normalization of raw ADC values to percentage (0-100%).
  * - Conversion of normalized values to engines control signals.
  * * These functions serve as the signal conditioning layer between raw ADC
  * hardware drivers and the high-level Vehicle FSM.
  ******************************************************************************
  */

#ifndef PEDALS_MAP_H
#define PEDALS_MAP_H
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "adc.h"
#include "vehicle_types.h"
#include "error_handler.h"

#define ACCEL_Msk (0xFF)
#define ACCEL_Bitpos (8)

#define BRAKE_Msk (0xFF)
#define BRAKE_Bitpos (8)



// Import the global Error Handler object defined in main.c
extern EH_HandleTypeDef heh;

// ============================================================================
// PLAUSIBILITY CHECKS
// ============================================================================

/**
 * @brief  Checks consistency between two accelerator pedal sensors.
 * @details Performs a plausibility check to ensure both sensors are reading
 * correlated values within a defined error margin. Essential for safety.
 * * @param[in] accel1val Raw ADC value from Sensor 1.
 * @param[in] accel2val Raw ADC value from Sensor 2.
 * @param[in] acceptAccelError Max allowable difference threshold.
 * * @return CountedVal_e
 * @retval GOOD (0) Sensors are consistent (OK).
 * @retval BAD (1) Discrepancy detected (Error).
 */

CountedVal_e accelPedalCheck(uint16_t *accel1val, uint16_t *accel2val, uint8_t acceptAccelError);

/**
 * @brief  Checks consistency between two brake piston pressure sensors.
 * * @param[in] brakePiston1 Raw ADC value from Sensor 1.
 * @param[in] brakePiston2 Raw ADC value from Sensor 2.
 * @param[in] acceptBrakeError Max allowable difference threshold.
 * * @return CountedVal_e
 * @retval GOOD (0) Sensors are consistent (OK).
 * @retval BAD (1) Discrepancy detected (Error).
 */
CountedVal_e brakePistonsCheck(uint16_t *brakePiston1, uint16_t *brakePiston2, uint8_t acceptBrakeError);

// ============================================================================
// SIGNAL MAPPING & SCALING
// ============================================================================

/**
 * @brief  Calculates the brake pedal position based on the Hall sensor.
 * @details Linearly maps the raw ADC value to a 0-100% range.
 * * @param[in] brakeHall Raw ADC value from the Hall sensor.
 * @return uint8_t Brake position in percentage (0-100).
 */
uint8_t brakeHallValue(uint16_t *brakeHall);

/**
 * @brief  Calculates the accelerator pedal position.
 * @details Integrates the redundancy check (@ref accelPedalCheck) and dead-zone
 * mapping. Returns 0 if an error is detected.
 * * @param[in] accel1val Raw ADC value from Sensor 1.
 * @param[in] accel2val Raw ADC value from Sensor 2.
 * @return uint8_t Throttle position in percentage (0-100). Returns 0 on error.
 */
uint8_t accelPedalValue(uint16_t *accel1val, uint16_t *accel2val);

/**
 * @brief  Calculates the brake pressure percentage.
 * @details Maps Sensor 1 to 0-100%. Does not run @ref brakePistonsCheck.
 * * @param[in] brakePiston1 Raw ADC value from Sensor 1.
 * @param[in] brakePiston2 Raw ADC value from Sensor 2 (unused).
 * @return uint8_t Brake pressure in percentage (0-100).
 */
uint8_t brakePistonsValue(uint16_t *brakePiston1, uint16_t *brakePiston2);

/**
 * @brief  Calculates the steering wheel position.
 * @details Maps raw ADC value to 0-100% range.
 * * @param[in] steerVal Raw ADC value from steering angle sensor.
 * @return uint8_t Steering position in percentage (0-100).
 */
uint8_t steerValue(uint16_t *steerVal);
/**
 * @brief  Calculates the final engine control command.
 * @details Converts the accelerator pedal position (0-100%) into the
 * specific integer range required by the inverter protocol.
 * * @param[in] accel1val Raw ADC value from Sensor 1.
 * @param[in] accel2val Raw ADC value from Sensor 2.
 * @return uint16_t Torque/Speed command value (Range: 0 - 32767).
 *
 * @warning Currently unused, and still linear - it does not apply the throttle
 * curve. Use @ref ThrottleCurve_Apply instead.
 */
uint16_t engineSteer(uint16_t *accel1val, uint16_t *accel2val);

// ============================================================================
// THROTTLE CURVE
// ============================================================================

/** @brief Lowest accepted curve exponent. z = 1 gives a linear response. */
#define THROTTLE_Z_MIN (1.0f)

/** @brief Highest accepted curve exponent. Widen if the CAN specification changes. */
#define THROTTLE_Z_MAX (4.0f)

/** @brief Inverter full-scale torque command - the 'max_value' of the curve. */
#define THROTTLE_MAX_VAL (32767)

/** @brief Highest accepted throttle limit, in percent of pedal travel. */
#define THROTTLE_LIMIT_MAX (50u)

/**
 * @brief  Sets the throttle curve exponent 'z'.
 * @details Stores the value and raises a rebuild flag; the table itself is
 * rebuilt lazily by @ref ThrottleCurve_Apply. Safe to call from any
 * context, including the CAN RX interrupt. Repeated calls with an
 * unchanged value are free.
 *
 * @param[in] z Exponent, must be within [@ref THROTTLE_Z_MIN, @ref THROTTLE_Z_MAX].
 *
 * @return bool
 * @retval true  Value accepted.
 * @retval false Rejected (out of range, NaN or Inf). Active curve left unchanged.
 */
bool ThrottleCurve_SetZ(float z);

/**
 * @brief  Sets the maximum accelerator pedal travel that reaches the motors.
 * @details Pedal positions above the limit are treated as if the pedal were held
 * exactly at the limit, so the torque command saturates there. Safe to
 * call from any context, including the CAN RX interrupt.
 *
 * @param[in] limitPercent Limit in percent of pedal travel, 0 - @ref THROTTLE_LIMIT_MAX.
 *
 * @return bool
 * @retval true  Value accepted.
 * @retval false Rejected (above the maximum). Active limit left unchanged.
 *
 * @note The resulting torque ceiling is THROTTLE_MAX_VAL * (limit/100)^z, so it
 * scales with the active curve exponent. A low limit combined with a steep
 * exponent yields a very small ceiling.
 */
bool ThrottleCurve_SetLimit(uint8_t limitPercent);

/**
 * @brief  Maps accelerator pedal position to the inverter torque command.
 * @details Implements y = THROTTLE_MAX_VAL * (x/100)^z via a precomputed
 * lookup table, saturated at the limit set by @ref ThrottleCurve_SetLimit.
 *
 * @param[in] accelPercent Pedal position in percent (0-100).
 * @return int16_t Torque command for the inverter (0 - @ref THROTTLE_MAX_VAL).
 */
int16_t ThrottleCurve_Apply(uint8_t accelPercent);

#endif
