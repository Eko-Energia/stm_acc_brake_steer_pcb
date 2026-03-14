/**
  ******************************************************************************
  * @file    vehicle_fsm.h
  * @brief   Header file for the Vehicle Finite State Machine (FSM).
  * @author  [Michał Jurek]
  * @date    2025-01-29
  * @details This file exposes the high-level control functions and the external
  * global variables required by the FSM. It links the low-level hardware
  * buffers (ADC, CAN) with the high-level logic.
  ******************************************************************************
  */
#ifndef VEHICLE_FSM_H
#define VEHICLE_FSM_H

#include "vehicle_types.h"
#include "adc.h"
#include "can.h"
#include "pedals_map.h"
#include "engine_control.h"

/**
 * @brief Temporary storage for the calculated Torque/Throttle value.
 * Used to store the result of @ref engineSteer before packing into a CAN frame.
 */

extern uint16_t tempTH;

// ============================================================================
// CAN HANDLES & HEADERS
// ============================================================================

/**
 * @brief General purpose CAN Transmission Header.
 * Used primarily for sending sensor data to the Jetson (StdID typically 0x100).
 */

extern CAN_TxHeaderTypeDef TxHeader;
/**
 * @brief Variable to store the mailbox index where the last Tx message was stored.
 */

extern uint32_t TxMailBox;
/**
 * @brief CAN Header for Inverter/Throttle control frames.
 * Used in manual mode (Jetson Down) to drive the motor (StdID typically 0x512).
 */

extern uint16_t ADC1_VAL[3];
// ============================================================================
// DATA BUFFERS
// ============================================================================

/**
 * @brief DMA Buffer for ADC1 readings.
 * - Index 0: Accelerator Pedal 1
 * - Index 1: Brake Pressure 1
 * - Index 2: Steering Angle
 */
extern uint16_t ADC2_VAL[3];
/**
 * @brief DMA Buffer for ADC2 readings.
 * - Index 0: Accelerator Pedal 2 (Redundant)
 * - Index 1: Brake Pressure 2 (Redundant)
 * - Index 2: Brake Hall Sensor
 */

extern uint8_t TxData[8];
/**
 * @brief General purpose CAN Transmit Buffer (8 bytes).
 * Used for sending mapped sensor data to Jetson.
 */

extern CAN_TxHeaderTypeDef TxHeaderTH, TxHeaderNMT;
/**
 * @brief Specific Transmit Buffer for Throttle/Torque commands (8 bytes).
 */
extern uint8_t TxDataTH[8];
/**
 * @brief Specific Transmit Buffer for NMT commands (2 bytes).
 * Payload: 0x01 (Start) or 0x02 (Stop).
 */
extern uint8_t TxDataNMT[2];

/**
 * @brief Global Vehicle State Object.
 * Volatile because it is updated in ISRs (CAN Rx Callback) and read in the main loop.
 */

extern volatile VehicleState_t Vehicle;

// ============================================================================
// FSM FUNCTION PROTOTYPES
// ============================================================================

/**
 * @brief  Calculates the next state of the vehicle's Finite State Machine.
 * @details Evaluates connectivity flags (Jetson, Charger) and priorities to
 * determine the operating mode.
 * @return MachineState_e The resolved state.
 */
MachineState_e machineState();

/**
 * @brief  Executes the control logic for the current state.
 * @details This function should be called cyclically (e.g., every 20ms).
 * It handles data acquisition, processing, and CAN transmission.
 */
void stateActions();

#endif
