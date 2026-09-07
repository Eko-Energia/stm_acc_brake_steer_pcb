/**
  ******************************************************************************
  * @file    engine_control.h
  * @brief   Header file for engine state management and NMT commands.
  * @author  [Michał Jurek]
  * @date    2025-01-29
  * @details This file defines the interface for direct inverter control.
  * It provides functions to start/stop the motor controller via CAN NMT
  * (Network Management) frames and tracks the internal state (Running/Stopped)
  * to avoid redundant CAN bus traffic.
  *
  * These functions are primarily used in the **JTSN_DOWN** states (Safety Mode).
  ******************************************************************************
  */

#ifndef ENGINE_CONTROL_H
#define ENGINE_CONTROL_H

#include <stdint.h>
#include "vehicle_types.h"
#include <can.h>

/**
 * @brief CAN Header for Throttle/Torque control frames.
 * Typically configured with ID 0x227 to drive the inverter directly.
 */
extern CAN_TxHeaderTypeDef TxHeaderTHR;

/**
 * @brief CAN Header for Throttle/Torque control frames.
 * Typically configured with ID 0x226 to drive the inverter directly.
 */
extern CAN_TxHeaderTypeDef TxHeaderTHL;

/**
 * @brief CAN Header for Network Management (NMT) frames.
 * Configured with ID 0x000 to broadcast state changes to all nodes or specific node.
 */
extern CAN_TxHeaderTypeDef TxHeaderNMT;

/**
 * @brief Data buffer for NMT payload (2 bytes).
 * Byte 0 contains the command (0x01 Start, 0x02 Stop).
 */
extern uint8_t TxDataNMT[2];

/**
 * @brief General purpose CAN Header (used for Jetson communication).
 */
extern CAN_TxHeaderTypeDef TxHeader;

/**
 * @brief Mailbox index variable required by HAL_CAN_AddTxMessage.
 */
extern uint32_t TxMailBox;

/**
 * @brief  Checks if the engine is currently flagged as operating.
 * @details This accessor is used by the FSM to determine if an NMT Start command
 * needs to be sent before sending torque commands.
 * @return uint8_t Current state (@ref ENGINE_RUN or @ref ENGINE_STOP).
 */
uint8_t getEngineFlag(void);

/**
 * @brief  Updates the internal engine state flag.
 * @details Should be called immediately after successfully sending a Start/Stop NMT frame.
 * @param[in] engineFlag The new state to set (@ref ENGINE_RUN or @ref ENGINE_STOP).
 */
void setEngineFlag(uint8_t engineFlag);

/**
 * @brief  Sends the "Enter Operational" NMT command to the inverter.
 * @details Transmits `0x01` on ID 0x000. This enables the inverter's power stage
 * and allows it to process torque commands.
 */
HAL_StatusTypeDef startEngine(void);

/**
 * @brief  Sends the "Pre-Operational" NMT command to the inverter.
 * @details Transmits `0x02` on ID 0x0.
 * This disables and blocks the inverter output immediately.
 */
HAL_StatusTypeDef stopEngine(void);

/**
 * @brief  Sends the " "Pre-Operational" NMT command to the inverter.
 * @details Transmits 0x80 depending on protocol on ID 0x0.
 * This disables the inverter output immediately without blocking it.
 */
HAL_StatusTypeDef neutralEngine(void);


#endif

