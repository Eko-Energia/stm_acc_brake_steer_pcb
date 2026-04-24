/**
  ******************************************************************************
  * @file    pedals_map.c
  * @brief   Implementation of pedal and steering mapping logic.
  * @author  [Michał Jurek]
  * @date    2025-03-21
  * @details This file contains functions to:
  * - Verify plausibility of redundant sensors (Accelerator, Brakes).
  * - Map raw ADC values to normalized percentages (0-100%).
  * - Convert percentage values to motor controller setpoints (0-32767).
  * - Handle calibration limits and dead zones.
  ******************************************************************************
  */

#include "pedals_map.h"
#include <pedals_const_val.h>
#include "adc.h"
#include "can.h"
#include "vehicle_types.h"
#include <stdlib.h>
#include "engine_control.h"


// ============================================================================
// BEGIN ACCEL. PEDAL SECTION
// ============================================================================

/**
 * @brief  Verifies the consistency between two accelerator pedal sensors.
 * @details Performs a plausibility check by comparing the normalized values of
 * two redundant sensors. It assumes a specific electrical characteristic
 * where Sensor 2 value (halved) should roughly match Sensor 1.
 *
 * @param[in] accel2val Raw ADC value from Accelerator Sensor 2.
 * @param[in] accel1val Raw ADC value from Accelerator Sensor 1.
 * @param[in] acceptAccelError Maximum allowed difference (threshold) to consider the reading valid.
 *
 * @return CountedVal_e
 * @retval BAD (0) Check passed (sensors are consistent).
 * @retval GOOD (1) Check failed (divergence exceeds @p acceptAccelError).
 */
CountedVal_e accelPedalCheck(uint16_t *accel2val, uint16_t *accel1val, uint8_t acceptAccelError)
{
	// Calculate difference. accel2val is divided by 2 (2:1 sensor ratio)
	int32_t pedalCheck = (*accel2val * 1000 / SENSOR_ADC_MAX_VALUE / 2 ) - (*accel1val * 1000 / SENSOR_ADC_MAX_VALUE);
	//int16_t pedalCheck = (accel2val * 1000 / SENSOR_ADC_MAX_VALUE ) - (accel1val * 1000 / SENSOR_ADC_MAX_VALUE); //breadboard test
	if (abs(pedalCheck) >= acceptAccelError) //value based on accel. pedal documentation
	{
		// Report Accelerator Implausibility Error to the CAN network
		uint8_t diagData[4];

		// Sensor 1 (uint16_t) 2 bytes (Little Endian)
		diagData[0] = (uint8_t)(*accel1val & ACCEL_Msk);
		diagData[1] = (uint8_t)((*accel1val >> ACCEL_Bitpos) & ACCEL_Msk);

		// Sensor 2 (uint16_t) 2 bytes (Little Endian)
		diagData[2] = (uint8_t)(*accel2val & ACCEL_Msk);
		diagData[3] = (uint8_t)((*accel2val >> ACCEL_Bitpos) & ACCEL_Msk);

		// Error report with 4 bytes of data
		EH_reportEx(&heh, 0x0AC, ERROR_SEVERITY_ERROR, diagData, 4);
		return BAD; //Difference between ADC1 and ADC2 exceeds accepted error -> error flag
	}
	else
	{
		// Values are matching. Clear the error state if it was previously set.
		EH_clear(&heh, 0x0AC);
		return GOOD; //all's good
	}
}
/**
 * @brief  Calculates the accelerator pedal position in percentage (0-100%).
 * @details First checks for sensor errors using @ref accelPedalCheck.
 * If valid, it maps the raw ADC value to a percentage range,
 * handling dead zones defined by @ref ACCEL_MIN_VAL and @ref ACCEL_MAX_VAL.
 *
 * @param[in] accel1val Raw ADC value from Accelerator Sensor 1 (Main).
 * @param[in] accel2val Raw ADC value from Accelerator Sensor 2 (Redundant).
 *
 * @return uint8_t
 * @retval 0-100 Valid pedal position percentage.
 * @retval 0     If a sensor error is detected (Safety fallback).
 */
uint8_t accelPedalValue(uint16_t *accel1val, uint16_t *accel2val)
{
	if (!accelPedalCheck(accel1val, accel2val, ACCEPT_ACCEL_ERROR))
	{

		uint16_t calibrated_val = *accel1val;

			// Handling dead zones (Min/Max limits)
		    if (calibrated_val < ACCEL_MIN_VAL) {
		        calibrated_val = ACCEL_MIN_VAL; // Below min deadzone -> 0%
		    }
		    else if (calibrated_val > ACCEL_MAX_VAL) {
		        calibrated_val = ACCEL_MAX_VAL; // Above max deadzone -> 100%
		    }

		    // Scaling to 0-100% range
			// Formula: (Value - Min) * 100 / (Max - Min)

		    // Calculate offset
		    // + Safety check to prevent underflow of uint32_t
		    uint32_t val_normalized = (calibrated_val > ACCEL_MIN_VAL) ? (calibrated_val - ACCEL_MIN_VAL) : 0;

		    // Calculate percentage
		    // Add half of the divisor (ACCEL_RANGE) for proper integer rounding
		    uint8_t accel_percentage_scaled = (uint8_t)((val_normalized * 100 + (ACCEL_RANGE / 2)) / ACCEL_RANGE);

		    // ONLY FOR TESTS
		    if (accel_percentage_scaled > 10)
		    {
		    	accel_percentage_scaled = 10;
		    }

		    return accel_percentage_scaled;
	}
	else
	{
		return 0x0; // Error state: Return 0 throttle
	}

}
// ============================================================================
// END OF ACCEL. PEDAL SECTION
// ============================================================================


// ============================================================================
// BEGIN OF ENGINE SECTION
// ============================================================================

/**
 * @brief  Converts pedal position to motor controller torque/speed command.
 * @details maps the 0-100% pedal value to the 16-bit integer range expected
 * by the inverter (0 to 32767).
 *
 * @param[in] accel1val Raw ADC value from Accelerator Sensor 1.
 * @param[in] accel2val Raw ADC value from Accelerator Sensor 2.
 *
 * @return uint16_t Control value for the engine (0 - 32767).
 */
uint16_t engineSteer(uint16_t *accel1val, uint16_t *accel2val)
{
	uint16_t accelEngine = (float)accelPedalValue(accel1val, accel2val) / 100.0f * 32767;
	return accelEngine;

}
// ============================================================================
// END OF ENGINE SECTION
// ============================================================================


// ============================================================================
// BEGIN OF BRAKE PEDAL (PISTONS) SECTION
// ============================================================================

/**
 * @brief  Verifies the consistency between two brake pressure sensors.
 * @details Compares the normalized values of two brake sensors against a
 * defined error threshold.
 *
 * @param[in] brakePiston1 Raw ADC value from Brake Sensor 1.
 * @param[in] brakePiston2 Raw ADC value from Brake Sensor 2.
 * @param[in] acceptBrakeError Maximum allowed difference (threshold).
 *
 * @return CountedVal_e
 * @retval 0 Check passed.
 * @retval 1 Check failed.
 */
CountedVal_e brakePistonsCheck(uint16_t *brakePiston1, uint16_t *brakePiston2, uint8_t acceptBrakeError)
{
	int8_t pistonsCheck = (*brakePiston1 * 1000) / SENSOR_ADC_MAX_VALUE  - (*brakePiston2 * 1000) / SENSOR_ADC_MAX_VALUE;
	if (abs(pistonsCheck) > acceptBrakeError)
	{
		// Report Accelerator Implausibility Error to the CAN network
		uint8_t diagData[4];
		// Sensor 1 (uint16_t) 2 bytes (Little Endian)
		diagData[0] = (uint8_t)(*brakePiston1 & BRAKE_Msk);
		diagData[1] = (uint8_t)((*brakePiston1 >> BRAKE_Bitpos) & BRAKE_Msk);

		// Sensor 2 (uint16_t) 2 bytes (Little Endian)
		diagData[2] = (uint8_t)(*brakePiston2 & BRAKE_Msk);
		diagData[3] = (uint8_t)((*brakePiston2 >> BRAKE_Bitpos) & BRAKE_Msk);
		// Report Brake Sensor Implausibility Error to the CAN network
		EH_reportEx(&heh, 0x0B, ERROR_SEVERITY_ERROR, diagData, 4);
		return BAD;
	}
	else
	{
		EH_clear(&heh, 0x0B);
		return GOOD;
	}
}

/**
 * @brief  Calculates brake pressure percentage based on piston sensors.
 * @details Checks for sensor errors first. If valid, maps the raw ADC value
 * to a 0-100% range relative to @ref SENSOR_ADC_MAX_VALUE.
 *
 * @param[in] brakePiston1 Raw ADC value from Brake Sensor 1.
 * @param[in] brakePiston2 Raw ADC value from Brake Sensor 2.
 *
 * @return uint8_t
 * @retval 0-100 Valid brake percentage.
 * @retval 0     If error detected.
 */
uint8_t brakePistonsValue(uint16_t *brakePiston1, uint16_t *brakePiston2)
{
	uint8_t brake_piston_percentage_scaled;
	if (!brakePistonsCheck(brakePiston1, brakePiston2, ACCEPT_BRAKE_ERROR))
	{
		uint16_t current_brake_piston_val = *brakePiston1;
		if (current_brake_piston_val >= SENSOR_ADC_MAX_VALUE) {
			brake_piston_percentage_scaled = 100;
			return brake_piston_percentage_scaled;
		}
		else {
			// Add half of the divisor (SENSOR_ADC_MAX_VALUE) for proper integer rounding
			uint32_t temp_calc_brake_piston = ((uint32_t)current_brake_piston_val * 100) + (SENSOR_ADC_MAX_VALUE / 2);
			brake_piston_percentage_scaled = (uint8_t)(temp_calc_brake_piston / SENSOR_ADC_MAX_VALUE);
			return brake_piston_percentage_scaled;
		}
	}
	else
	{
		return 0x0;
	}
}

// ============================================================================
// END OF BRAKE PEDAL (PISTONS) SECTION
// ============================================================================


// ============================================================================
// BEGIN OF BRAKE PEDAL (HALL) SECTION
// ============================================================================

/**
 * @brief  Calculates brake pedal position based on Hall effect sensor.
 * @details Maps the raw ADC value directly to a 0-100% range based on
 * @ref SENSOR_ADC_MAX_VALUE.
 *
 * @param[in] brakeHal Raw ADC value from Brake Hall Sensor.
 *
 * @return uint8_t Brake position (0-100%).
 */
uint8_t brakeHallValue(uint16_t *brakeHal)
{
	uint8_t brake_hall_scaled;
	if (*brakeHal >= SENSOR_ADC_MAX_VALUE)
	{
		brake_hall_scaled = 100;
		return brake_hall_scaled;
	}
	else
	{
		// Add half of the divisor (SENSOR_ADC_MAX_VALUE) for proper integer rounding
		uint32_t temp_hall_brake = ((uint32_t)*brakeHal * 100) + (SENSOR_ADC_MAX_VALUE / 2);
		brake_hall_scaled = (uint8_t)(temp_hall_brake / SENSOR_ADC_MAX_VALUE);
		return brake_hall_scaled;
	}
}
// ============================================================================
// END OF BRAKE PEDAL (HALL) SECTION
// ============================================================================


// ============================================================================
// BEGIN OF STEERING WHEEL SECTION
// ============================================================================

/**
 * @brief  Calculates steering wheel position percentage.
 * @details Maps the raw ADC value directly to a 0-100% range based on
 * @ref SENSOR_ADC_MAX_VALUE.
 *
 * @param[in] steerWheel Raw ADC value from Steering Angle Sensor.
 *
 * @return uint8_t Steering position (0-100%).
 */
uint8_t steerValue(uint16_t *steerWheel)
{
	uint8_t steer_percentage_scaled;
	uint16_t current_steer_val = *steerWheel;
	if (current_steer_val >= SENSOR_ADC_MAX_VALUE) {
	  steer_percentage_scaled = 100;
	  return steer_percentage_scaled;
	  }
	else {
	  // Add half of the divisor (SENSOR_ADC_MAX_VALUE) for proper integer rounding
	  uint32_t temp_calc_steer = ((uint32_t)current_steer_val * 100) + (SENSOR_ADC_MAX_VALUE / 2);
	  steer_percentage_scaled = (uint8_t)(temp_calc_steer / SENSOR_ADC_MAX_VALUE);
	  return steer_percentage_scaled;
	}
}

// ============================================================================
// END OF STEERING WHEEL SECTION
// ============================================================================
