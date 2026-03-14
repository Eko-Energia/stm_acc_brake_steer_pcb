/**
  ******************************************************************************
  * @file    vehicle_fsm.c
  * @brief   Finite State Machine (FSM) implementation for vehicle control.
  * @author  [Michał Jurek]
  * @date    2025-01-29
  * @details This file implements the high-level logic of the vehicle. It determines
  * the current operating mode based on system connectivity (Jetson, Charger)
  * and executes the appropriate actions (CAN transmission, Engine Start/Stop).
  *
  * The logic is divided into two main branches:
  * 1. **Jetson Working:** The MCU acts as a pass-through, sending sensor data to the PC.
  * 2. **Jetson Down:** The MCU takes direct control of the inverter (Safety/Manual mode).
  ******************************************************************************
  */

#include "vehicle_fsm.h"


/**
 * @brief  Determines the current state of the vehicle machine.
 * @details Checks system flags in a specific priority order:
 * 1. Jetson Connection (Is the autonomous computer alive?)
 * 2. Charger Status (Is the car plugged in?)
 * 3. Gear Status (Future implementation)
 *
 * @return MachineState_e The current state of the system.
 */

MachineState_e machineState()
{

	// ========================================================================
	// BRANCH 1: JETSON IS CONNECTED (Autonomous/Assisted Mode)
	// ========================================================================

	if (Vehicle.Jetson.IsConnected == true)
	{
		/*
	//Charging state
		if (Vehicle.Charger.RawStatus == START_CHARGING)
		{
			return JTSN_WORKS_CHARGE_STATE;
		}
		*/

	//N gear in

		if (Vehicle.PRND.RawStatus == NEUTRAL_GEAR)
		{
			return JTSN_WORKS_NEUTRAL_GEAR_STATE;
		}

	//Active state
		else
		{
			return JTSN_WORKS_DRIVE_STATE;
		}

	}

	// ========================================================================
	// BRANCH 2: JETSON IS DOWN (This PCB is in charge)
	// ========================================================================

	else
	{
		/*
		//Charging state
		if (Vehicle.Charger.RawStatus == START_CHARGING)
		{

			return JTSN_DOWN_CHARGE_STATE;
		}
		*/

		// P gear in
		if (Vehicle.PRND.RawStatus == PARKING_GEAR)
		{
			return	JTSN_DOWN_PARKING_STATE;
		}

		// R gear in
		else if (Vehicle.PRND.RawStatus == REVERSE_GEAR)
		{
			return	JTSN_DOWN_REVERSE_STATE;
		}

		// N gear in
		else if (Vehicle.PRND.RawStatus == NEUTRAL_GEAR)
		{
			return	JTSN_DOWN_NEUTRAL_GEAR_STATE;
		}

		// D gear in => Active state
		else
		{
			return JTSN_DOWN_DRIVE_STATE;
		}
	}

}

/**
 * @brief  Executes the logic associated with the current machine state.
 * @details This function calls @ref machineState() to update the mode and then
 * runs a switch-case to perform actions:
 * - **JTSN_WORKS_DRIVE_STATE**: Reads pedals/steering ADC, packs them into a CAN frame,
 * and sends them to the Jetson (using `TxHeader`).
 * - **JTSN_DOWN_DRIVE_STATE**: Reads Accelerator ADC, maps it to torque,
 * and sends it DIRECTLY to the Inverter (using `TxHeaderTH`).
 * - **Other States**: Handles engine start/stop logic to ensure safety (e.g., stopping
 * engine during charging).
 *
 * @note Relies on global variables `ADC1_VAL`, `ADC2_VAL`, `TxData`, `hcan`, etc.
 */

void stateActions()
{
	switch(machineState()){

	// ========================================================================
	// SCENARIO: JETSON WORKING
	// ========================================================================

	/*
	//charger connected
	case JTSN_WORKS_CHARGE_STATE:

		break;
	*/

	//N gear in
	case JTSN_WORKS_NEUTRAL_GEAR_STATE:

		break;
	//active state
	case JTSN_WORKS_DRIVE_STATE:
		TxData[0] = steerValue(ADC1_VAL[2]);;
		TxData[1] = brakePistonsValue(ADC1_VAL[1], ADC2_VAL[1]);
		TxData[2] = brakeHallValue(ADC2_VAL[2]);
		TxData[3] = accelPedalValue(ADC1_VAL[0], ADC2_VAL[0]);

		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox) != HAL_OK)
		{
		  Error_Handler();
		}

		break;
		// ========================================================================
		// SCENARIO: JETSON DOWN
		// ========================================================================
		//Sending engine frames directly when in active state


		/*
		// Charger connected - Safety Stop
	case JTSN_DOWN_CHARGE_STATE:
		if (getEngineFlag() == ENGINE_RUN)
		{
			stopEngine();
			setEngineFlag(ENGINE_STOP);
		}

		break;
		*/

	// N gear in
	case JTSN_DOWN_NEUTRAL_GEAR_STATE:
		if (getEngineFlag() == ENGINE_RUN)
		{
			neutralEngine();
			setEngineFlag(ENGINE_STOP_NEUTRAL);
		}
		break;
	// Active state - Direct Pedal-to-Engine
	case JTSN_DOWN_DRIVE_STATE:
		// 1. One-shot Start Engine sequence
		// Only executed if we just entered this state or engine was stopped
		if (getEngineFlag() == ENGINE_STOP_NEUTRAL)
		{
			startEngine();
			setEngineFlag(ENGINE_RUN);
		}
		// 2. Calculate Torque command based on Accelerator position
		tempTH = engineSteer(ADC1_VAL[0], ADC2_VAL[0]);

		// 3. Split 16-bit value into two 8-bit bytes (Little Endian)
		TxDataTH[0] = (uint8_t)tempTH & 0xFF; 			// lsb
		TxDataTH[1] = (uint8_t)(tempTH >> 8) & 0xFF;    // msb

		if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderTH, TxDataTH, &TxMailBox) != HAL_OK)
		{
		  Error_Handler();
		}
		break;

	// Parking gear in
	case JTSN_DOWN_PARKING_STATE:
		stopEngine();
		break;


	case SAFE_STOP_STATE:
	{
		// if case prevents continuous messages to the inverter
		if (getEngineFlag() != ENGINE_STOP_NEUTRAL)
		{
			neutralEngine();
			setEngineFlag(ENGINE_STOP_NEUTRAL);
		}
		break;
	}
	default:
		//error handler?
		break;
	}
}
