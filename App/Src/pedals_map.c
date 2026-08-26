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
#include <stdbool.h>
#include <math.h>

bool brakesPressed = false;
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

	// TEST IT WHETHER IT WORKS
	//int32_t rawDiff = (int32_t)(*accel2val) - ((int32_t)(*accel1val) * 2);


	//int16_t pedalCheck = (accel2val * 1000 / SENSOR_ADC_MAX_VALUE ) - (accel1val * 1000 / SENSOR_ADC_MAX_VALUE); //breadboard test
	 if (abs(pedalCheck) >= acceptAccelError) //value based on accel. pedal documentation
//	if (abs(rawDiff) >= 1030) // 1% OF 1023
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
	if (!accelPedalCheck(accel2val, accel1val, ACCEPT_ACCEL_ERROR) && !brakesPressed)
	{

		uint16_t calibrated_val = *accel2val;

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
 *
 * @warning Currently unused - nothing calls this and the linker discards it.
 * It still implements the original purely linear mapping, so it does
 * NOT match the live path, which applies the progressive throttle
 * curve. If this is ever wired up, replace the body with a call to
 * @ref ThrottleCurve_Apply instead of duplicating the formula.
 */
uint16_t engineSteer(uint16_t *accel1val, uint16_t *accel2val)
{
	uint16_t accelEngine = (float)accelPedalValue(accel1val, accel2val) / 100.0f * 32767;
	return accelEngine;

}

// ----------------------------------------------------------------------------
// Throttle curve
//
// y = THROTTLE_MAX_VAL * (x/100)^z is evaluated through a lookup table. The
// pedal position is a whole percent, so a 101-entry table covers the entire
// input domain: no interpolation, no approximation error, and the runtime cost
// collapses to a single array read.
// ----------------------------------------------------------------------------

/** @brief Table length: one entry per whole percent of pedal travel (0-100). */
#define THROTTLE_LUT_SIZE (101)

// Build-time guard for the throttle limit: it is used directly as a table index
// after clamping, so the configured maximum must be a valid index. Compile-time
// only.
_Static_assert(THROTTLE_LIMIT_MAX <= (THROTTLE_LUT_SIZE - 1),
               "throttle limit must not exceed the table's highest index");

static int16_t throttleLut[THROTTLE_LUT_SIZE];
static volatile float throttleZRequested = THROTTLE_Z_MIN;
static volatile bool throttleLutDirty = true;

// Applied at lookup time, so it needs no rebuild and no dirty flag.
static volatile uint8_t throttleLimitPct = THROTTLE_LIMIT_MAX;

/**
 * @brief  Sets the throttle curve exponent 'z'.
 * @details Stores the value and raises a rebuild flag; the table itself is
 * rebuilt lazily by @ref ThrottleCurve_Apply. Safe to call from any
 * context, including the CAN RX interrupt.
 *
 * @param[in] z Exponent, must be within [@ref THROTTLE_Z_MIN, @ref THROTTLE_Z_MAX].
 *
 * @return bool
 * @retval true  Value accepted.
 * @retval false Rejected (out of range, NaN or Inf). Active curve left unchanged.
 */
bool ThrottleCurve_SetZ(float z)
{
	// Test for the valid window rather than for the invalid one: a NaN compares
	// false against everything, so this rejects NaN and Inf along with
	// out-of-range values. A malformed frame must never reach the curve.
	if (!(z >= THROTTLE_Z_MIN && z <= THROTTLE_Z_MAX))
	{
		return false;
	}

	// Compared against the last request, not against the table: either the table
	// already reflects this value, or a rebuild for it is already scheduled. The
	// frame is expected to be periodic, so repeats must not pay the rebuild cost.
	if (z == throttleZRequested)
	{
		return true;
	}

	throttleZRequested = z;
	throttleLutDirty = true; // set last, so the flag never precedes the value
	return true;
}

/**
 * @brief  Sets the maximum accelerator pedal travel that reaches the motors.
 * @details Rejecting rather than clamping an out-of-range value lets the caller
 * report the bad frame, and leaves the previous limit in force.
 *
 * @param[in] limitPercent Limit in percent of pedal travel, 0 - @ref THROTTLE_LIMIT_MAX.
 *
 * @return bool
 * @retval true  Value accepted.
 * @retval false Rejected. Active limit left unchanged.
 */
bool ThrottleCurve_SetLimit(uint8_t limitPercent)
{
	if (limitPercent > THROTTLE_LIMIT_MAX)
	{
		return false;
	}

	throttleLimitPct = limitPercent;
	return true;
}

/**
 * @brief  Returns the active throttle limit.
 * @return uint8_t Limit in percent of pedal travel, 0 - @ref THROTTLE_LIMIT_MAX.
 */
uint8_t ThrottleCurve_GetLimit(void)
{
	return throttleLimitPct;
}

/**
 * @brief  Rebuilds the lookup table for the pending exponent.
 * @details Runs 99 powf() calls, which takes roughly 1-2 ms. Main-loop context
 * only - never call this from an interrupt.
 */
static void ThrottleCurve_Rebuild(void)
{
	// Clear the flag before snapshotting z. If a new value lands mid-build the
	// flag goes up again and the table is rebuilt on the next pass, instead of
	// losing the update or mixing two exponents into a single table.
	throttleLutDirty = false;
	const float z = throttleZRequested;

	// Endpoints are pinned: full throttle must be exactly full scale regardless
	// of how powf() rounds, and a released pedal must be exactly zero.
	throttleLut[0] = 0;
	throttleLut[THROTTLE_LUT_SIZE - 1] = THROTTLE_MAX_VAL;

	for (uint8_t i = 1; i < THROTTLE_LUT_SIZE - 1; i++)
	{
		float norm = powf((float)i / 100.0f, z);

		// Add half a count before truncating so the cast rounds to nearest instead
		// of biasing every entry downwards. Same trick as the integer mapping
		// helpers in this file, which add half the divisor before dividing.
		throttleLut[i] = (int16_t)((norm * (float)THROTTLE_MAX_VAL) + 0.5f);
	}
}

/**
 * @brief  Maps accelerator pedal position to the inverter torque command.
 * @details Rebuilds the table first if the exponent changed since the last call.
 *
 * @param[in] accelPercent Pedal position in percent (0-100).
 *
 * @return int16_t Torque command for the inverter (0 - @ref THROTTLE_MAX_VAL).
 *
 * @note Main-loop context only, because it may trigger a rebuild.
 */
int16_t ThrottleCurve_Apply(uint8_t accelPercent)
{
	if (throttleLutDirty)
	{
		ThrottleCurve_Rebuild();
	}

	// Saturate at the configured limit. Snapshot it first: the setter may run from
	// the CAN RX interrupt between the comparison and the lookup.
	const uint8_t limit = throttleLimitPct;
	if (accelPercent > limit)
	{
		accelPercent = limit;
	}

	return throttleLut[accelPercent];
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
 * @details Maps the raw ADC value to a 0-100% range relative to
 * @ref SENSOR_ADC_MAX_VALUE. Plausibility is handled separately by
 * @ref brakePistonsCheck — do not call it from here, it reports errors
 * that force the FSM into Neutral and drop periodic CAN frames.
 *
 * @param[in] brakePiston1 Raw ADC value from Brake Sensor 1.
 * @param[in] brakePiston2 Raw ADC value from Brake Sensor 2 (unused, API kept).
 *
 * @return uint8_t Brake percentage 0-100.
 */
uint8_t brakePistonsValue(uint16_t *brakePiston1, uint16_t *brakePiston2)
{
	(void)brakePiston2;
	uint16_t current_brake_piston_val = *brakePiston1;
	if (current_brake_piston_val >= SENSOR_ADC_MAX_VALUE) {
		return 100;
	}

	uint32_t temp_calc_brake_piston = ((uint32_t)current_brake_piston_val * 100) + (SENSOR_ADC_MAX_VALUE / 2);
	return (uint8_t)(temp_calc_brake_piston / SENSOR_ADC_MAX_VALUE);
}

// ============================================================================
// END OF BRAKE PEDAL (PISTONS) SECTION
// ============================================================================


// ============================================================================
// BEGIN OF BRAKE PEDAL (HALL) SECTION
// ============================================================================

/**
 * @brief  Calculates brake pedal position based on Hall effect sensor.
 * @details Maps the raw ADC value to a 0-100% range, handling dead zones
 * defined by @ref BRAKE_HALL_MIN_VAL and @ref BRAKE_HALL_MAX_VAL.
 *
 * @param[in] brakeHal Raw ADC value from Brake Hall Sensor.
 *
 * @return uint8_t Brake position (0-100%).
 */
uint8_t brakeHallValue(uint16_t *brakeHal)
{
	uint16_t calibrated_val = *brakeHal;

	// Handling dead zones (Min/Max limits)
	if (calibrated_val < BRAKE_HALL_MIN_VAL) {
		calibrated_val = BRAKE_HALL_MIN_VAL; // Below min deadzone -> 0%
	}
	else if (calibrated_val > BRAKE_HALL_MAX_VAL) {
		calibrated_val = BRAKE_HALL_MAX_VAL; // Above max deadzone -> 100%
	}

	// Scaling to 0-100% range
	// Formula: (Value - Min) * 100 / (Max - Min)

	// Calculate offset
	// + Safety check to prevent underflow of uint32_t
	uint32_t val_normalized = (calibrated_val > BRAKE_HALL_MIN_VAL) ? (calibrated_val - BRAKE_HALL_MIN_VAL) : 0;

	// Calculate percentage
	// Add half of the divisor (BRAKE_HALL_RANGE) for proper integer rounding
	uint8_t brake_hall_scaled = (uint8_t)((val_normalized * 100 + (BRAKE_HALL_RANGE / 2)) / BRAKE_HALL_RANGE);

	if (brake_hall_scaled > 0) {
		brakesPressed = true;
	}
	else {
		brakesPressed = false;
	}
	return brake_hall_scaled;
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
