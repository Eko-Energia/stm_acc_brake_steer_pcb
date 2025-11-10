#include "pedals.h"
#include "pedals_const_val.h"
#include "adc.h"
#include <stdlib.h>
volatile int isADC1finished = 0;
volatile int isADC2finished = 0;

// BEGIN ACCEL. pedal section
int accelPedalCheck(uint16_t accel1val, uint16_t accel2val, uint8_t acceptAccelError)
{
	int16_t pedalCheck = (accel1val * 1000 / SENSOR_ADC_MAX_VALUE / 2 ) - (accel2val * 1000 / SENSOR_ADC_MAX_VALUE);
	if (abs(pedalCheck) >= acceptAccelError) //value based on accel. pedal documentation
	{
		return 1; //Difference between ADC1 and ADC2 is bigger than accepted error -> error flag
	}
	else
	{
		return 0; //all's good
	}
}

uint8_t accelPedalValue(uint16_t accel1val, uint16_t accel2val)
{
	if (!accelPedalCheck(accel1val, accel2val, ACCEPT_ACCEL_ERROR))
	{
		uint16_t current_accel_val = accel1val;
		uint8_t accel_percentage_scaled;
		if (current_accel_val >= SENSOR_ADC_MAX_VALUE)
		{
			accel_percentage_scaled = 100;
			return accel_percentage_scaled;
		}
		else
		{
			uint32_t temp_calc_accel = ((uint32_t)current_accel_val * 100) + (SENSOR_ADC_MAX_VALUE / 2);
			accel_percentage_scaled = (uint8_t)(temp_calc_accel / SENSOR_ADC_MAX_VALUE);
			return accel_percentage_scaled;
		}
	}
	else
	{
		return 0x0;
	}

}
//END of ACCEL. pedal section


//BEGIN of brake pedal (pistons) section

//	acceptError -> accepted error in percentage value
//	How to input wanted percentage value, ex. below:
//	0.5% -> 5
//	50.6% -> 506
int brakePistonsCheck(uint16_t brakePiston1, uint16_t brakePiston2, uint8_t acceptBrakeError)
{
	int8_t pistonsCheck = (brakePiston1 * 1000) / SENSOR_ADC_MAX_VALUE  - (brakePiston2 * 1000) / SENSOR_ADC_MAX_VALUE;
	if (abs(pistonsCheck) > acceptBrakeError)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//	to do: include piston ratio
uint8_t brakePistonsValue(uint16_t brakePiston1, uint16_t brakePiston2)
{
	uint8_t brake_piston_percentage_scaled;
	if (brakePistonsCheck(brakePiston1, brakePiston2, ACCEPT_BRAKE_ERROR))
	{
		uint16_t current_brake_piston_val = brakePiston1;
		if (current_brake_piston_val >= SENSOR_ADC_MAX_VALUE) {
			brake_piston_percentage_scaled = 100;
			return brake_piston_percentage_scaled;
		}
		else {
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

//END of BRAKE pedal (pistons) section

//BEGIN of BRAKE pedal (Hall) section
uint8_t brakeHallValue(uint16_t brakeHal)
{
	uint8_t brake_hall_scaled;
	if (brakeHal >= SENSOR_ADC_MAX_VALUE)
	{
		brake_hall_scaled = 100;
		return brake_hall_scaled;
	}
	else
	{
		uint32_t temp_hall_brake = ((uint32_t)brakeHal * 100) + (SENSOR_ADC_MAX_VALUE / 2);
		brake_hall_scaled = (uint8_t)(temp_hall_brake / SENSOR_ADC_MAX_VALUE);
		return brake_hall_scaled;
	}
}
//END of BRAKE pedal (Hall) section

//BEGIN of STEERING wheel section

uint8_t steerValue(uint16_t steerWheel)
{
	uint8_t steer_percentage_scaled;
	uint16_t current_steer_val = steerWheel;
	if (current_steer_val >= SENSOR_ADC_MAX_VALUE) {
	  steer_percentage_scaled = 100;
	  return steer_percentage_scaled;
	  }
	else {
	  uint32_t temp_calc_steer = ((uint32_t)current_steer_val * 100) + (SENSOR_ADC_MAX_VALUE / 2);
	  steer_percentage_scaled = (uint8_t)(temp_calc_steer / SENSOR_ADC_MAX_VALUE);
	  return steer_percentage_scaled;
	}
}

//END of STEERING wheel section


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    isADC1finished = 1;
  }
  if (hadc->Instance == ADC2)
  {
    isADC2finished = 1;
  }
}
