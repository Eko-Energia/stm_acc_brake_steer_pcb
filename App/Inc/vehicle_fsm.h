/**
  ******************************************************************************
  * @file    vehicle_fsm.h
  * @brief   Header file for the Vehicle Finite State Machine (FSM).
  * @author  [Michał Jurek]
  * @date    2025-03-21
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
#include "can_driver.h"
#include "torque_vectoring.h"
#include "tv_config.h"

#define TH_mask (0xFF)
#define TH_bitpos (8)
#define NMTcommands (2)

/** @brief Macros for data[] indexing in Jetson_GetData function.
 * */
#define steerValIndex (0)
#define brakePistonsValIndex (1)
#define brakeHallValIndex (2)
#define accelPedalValIndex (3)
/**
 * @brief  Global Vehicle State Object.
 *
 * @details Centralized structure holding the current status and connection health
 * of critical subsystems (Charger, Jetson, PRND). Declared as 'volatile' to
 * guarantee safe, unoptimized memory access across the main loop and asynchronous
 * interrupts (e.g., TIM2 Watchdog).
 *
 * @note   Actual memory allocation (definition) is located in vehicle_fsm.c.
 */
extern volatile VehicleState_t Vehicle;

/**
 * @brief  External reference to the global CAN message scheduler.
 *
 * @details This structure manages the queue of periodic CAN transmissions.
 * It is physically defined in main.c and processed continuously
 * in the main background loop by the CAN driver.
 * It is exposed here so the Finite State Machine (FSM) can dynamically
 * add or remove periodic messages (e.g., Jetson status, Engine throttle)
 * based on the current vehicle state.
 *
 * * @note    Do not modify this structure directly. Always use the driver API
 * functions (CAN_addScheduledMessage, CAN_removeScheduledMessage)
 * to interact with the scheduler.
 */

extern struct CAN_scheduledMsgList canScheduler;

/**
 * @brief Temporary storage for the calculated Torque/Throttle value.
 * Used to store the result of @ref engineSteer before packing into a CAN frame.
 */

extern int16_t tempTH;

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

// ============================================================================
// DATA BUFFERS
// ============================================================================

extern uint16_t ADC1_VAL[3];

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

extern CAN_TxHeaderTypeDef TxHeaderTHR, TxHeaderTHL, TxHeaderNMT;

/**
 * @brief Specific Transmit Buffer for NMT commands (2 bytes).
 * Payload: 0x01 (Start) or 0x02 (Stop).
 */
extern uint8_t TxDataNMT[NMTcommands];

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
 *
 * @details Evaluates connectivity flags (Jetson, Charger) and priorities to
 * determine the operating mode.
 *
 * @return MachineState_e The resolved state.
 */
MachineState_e machineState(void);

/**
 * @brief  Executes the control logic for the current state.
 *
 * @details This function should be called cyclically (e.g., every 20ms).
 * It handles data acquisition, processing, and CAN transmission.
 */
void stateActions(void);

/**
 * @brief  Callback function to populate the Jetson periodic CAN frame.
 *
 * @details This function is invoked automatically by the CAN message scheduler
 * right before transmitting the status message to the autonomous computer (Jetson).
 * It only packs already mapped percentages from @ref Vehicle.Pedals.
 * Mapping itself is done once per cycle in @ref stateActions.
 * * Byte mapping:
 * - Byte 0: Steering angle
 * - Byte 1: Brake pistons pressure
 * - Byte 2: Brake Hall sensor status
 * - Byte 3: Accelerator pedal position
 * - Bytes 4-7: Reserved (Zeroed by the driver automatically)
 *
 * @param[out] data    Pointer to the 8-byte payload buffer provided by the CAN driver.
 * @param[in]  context Optional user context pointer (currently unused, expected NULL).
 */
void Jetson_GetData(uint8_t *data, void *context);

/**
 * @brief  Callback function to populate the Engine/Inverter periodic CAN frame.
 *
 * @details This function is invoked automatically by the CAN message scheduler
 * right before transmitting the torque/throttle command to the motor controller.
 * It calculates the required torque based on redundant accelerator pedal
 * readings and formats the 16-bit result into Little-Endian byte order.
 *
 * Byte mapping (Little-Endian):
 * - Byte 0: Torque command LSB (Least Significant Byte)
 * - Byte 1: Torque command MSB (Most Significant Byte)
 * - Bytes 2-7: Reserved (Zeroed by the driver automatically)
 *
 * @param[out] data    Pointer to the 8-byte payload buffer provided by the CAN driver.
 * @param[in]  context Optional user context pointer (currently unused, expected NULL).
 */
void EngineThrottle_GetData(uint8_t *data, void *context);

// ============================================================================
// TORQUE VECTORING
// ============================================================================

/**
 * @brief  Sets how much of the calculated torque split reaches the wheels.
 *
 * @details Blends between an open differential and the full split calculated
 * from the steering rack position and the vehicle speed. Expected to change
 * while driving - per lap or per surface - so it is a remotely set parameter
 * just like @ref ThrottleCurve_SetLimit, not a vehicle constant. Safe to call
 * from any context, including the CAN RX interrupt.
 *
 * @param[in] gainPercent Gain in percent, 0 - @ref TV_CONFIG_GAIN_MAX. 0 gives
 * both wheels the same command, 100 gives the whole calculated split.
 *
 * @return bool
 * @retval true  Value accepted.
 * @retval false Rejected (above the maximum). Active gain left unchanged.
 */
bool TorqueVectoring_SetGain(uint8_t gainPercent);

/**
 * @brief  Returns the active torque vectoring gain.
 *
 * @details Defaults to @ref TV_CONFIG_GAIN_DEFAULT until a new value arrives.
 *
 * @return uint8_t Gain in percent, 0 - @ref TV_CONFIG_GAIN_MAX.
 */
uint8_t TorqueVectoring_GetGain(void);

#endif
