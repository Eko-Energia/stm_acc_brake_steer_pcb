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
 * @details Integrates the redundancy check (@ref brakePistonsCheck).
 * * @param[in] brakePiston1 Raw ADC value from Sensor 1.
 * @param[in] brakePiston2 Raw ADC value from Sensor 2.
 * @return uint8_t Brake pressure in percentage (0-100). Returns 0 on error.
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
 */
uint16_t engineSteer(uint16_t *accel1val, uint16_t *accel2val);

#endif
