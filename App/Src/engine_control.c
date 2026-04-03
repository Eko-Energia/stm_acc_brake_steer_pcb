/**
******************************************************************************
* @file    engine_control.c
* @brief   Engine state management and CAN NMT command transmission.
* @author  [Michał Jurek]
* @date    2025-03-21
* @details This file handles the logic for starting and stopping the engine.
* It manages a local state flag to prevent redundant CAN frames
* and sends specific Network Management (NMT) frames to change the
* inverter's operating mode (Operational vs Pre-Operational/Stop).
******************************************************************************
*/
#include "engine_control.h"


/** @brief Specific Transmit Buffer for NMT commands (Start/Stop Inverter). */
uint8_t TxDataNMT[2]={0};


/** * @brief Internal variable to track the current engine state.
 * @details Initialized to @ref ENGINE_STOP_NEUTRAL7 to ensure safety at startup.
 */

static uint8_t engineState = ENGINE_STOP_NEUTRAL;

/**
 * @brief  Gets the current internal state of the engine.
 * @return uint8_t Current state (e.g., @ref ENGINE_RUN or @ref ENGINE_STOP).
 */
uint8_t getEngineFlag()
{
	return engineState;
}
/**
 * @brief  Sets the internal state of the engine.
 * @details This function updates the local flag used by the FSM to track
 * whether the start/stop command has already been sent.
 * @param[in] engineFlag The new state to set (e.g., @ref ENGINE_RUN).
 */
void setEngineFlag(uint8_t engineFlag)
{
	engineState = engineFlag;
}
/**
 * @brief  Sends the NMT Start Command to the inverter via CAN.
 * @details Sets the NMT payload to `0x01` (Enter Operational Mode).
 * This command is required to enable the inverter to accept torque/throttle commands.
 * * @note   Uses the global `TxHeaderNMT` (ID 0x0).
 * @note   Consider sending this frame multiple times in a loop to ensure
 * reception if the bus is busy or the inverter is slow to wake up.
 */

//maybe send below frames a few times to make sure it'll get to the desired target
void startEngine()
{
	TxDataNMT[0] = 0x01; //puts engines in the operating mode
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderNMT, TxDataNMT, &TxMailBox) != HAL_OK)
	{
	  Error_Handler();
	}
}

// We have to test the engines in order to confirm the statement below
/**
 * @brief  Sends the NMT Stop Command to the inverter via CAN.
 * @details Sets the NMT payload to `0x02` to block the inverter.
 * This safely disables the motor output ensuring safe stop.
 * * @note   Uses the global `TxHeaderNMT`.
 */
void stopEngine()
{
	TxDataNMT[0]= 0x02; //stops inverters immediately and blocks them
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderNMT, TxDataNMT, &TxMailBox) != HAL_OK)
	{
	  Error_Handler();
	}
}

void neutralEngine()
{
	TxDataNMT[0]= 0x80; //puts engines in the NEUTRAL mode to make a safe stop
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderNMT, TxDataNMT, &TxMailBox) != HAL_OK)
	{
	  Error_Handler();
	}
}

