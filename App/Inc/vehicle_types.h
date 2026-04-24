/**
  ******************************************************************************
  * @file    vehicle_types.h
  * @brief   Global data structures and enumeration definitions.
  * @author  [Michał Jurek]
  * @date    2025-03-21
  * @details This file defines the core data types used throughout the application:
  * - @ref VehicleState_t: The master structure holding the current system state.
  * - @ref MachineState_e: The states for the main Finite State Machine.
  * - Enums for Engine, Charger, PRND, ADC states.
  ******************************************************************************
  */
#ifndef VEHICLE_TYPES_H
#define VEHICLE_TYPES_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/**
 * @struct VehicleState_t
 * @brief  Master structure collecting all information about the vehicle's subsystems.
 *
 * @details This structure is typically instantiated as a global volatile variable
 * to be accessible by ISRs and the main loop.
 */
typedef struct {
	/**
	 * @brief CURRENTLY NOT USED PART Charger subsystem status.
	 * Handles communication watchdog and status flags for the external charger.
	 */
    struct {
    	bool 		IsConnected;  	///< Connection flag (Watchdog). True if valid frames received recently.
		uint8_t  	RawStatus;    	///< Current status/command. See @ref ChargerState_e.
		uint32_t 	LastMsgTick;  	///< Timestamp (HAL_GetTick) of the last received CAN frame.
    } Charger;

    /**
	 * @brief Jetson (Autonomous Computer) subsystem status.
	 * Handles incoming control data and connection monitoring.
	 */
    struct {
        uint8_t  	RawData[8];   	///< Buffer for raw data received from Jetson
        bool     	IsConnected;	///< Connection flag. True if Jetson is alive.
        uint32_t 	LastMsgTick;	///< Timestamp of the last received heartbeat/control frame.
    } Jetson;

    /**
	 * @brief PRND subsystem status.
	 * Handles communication watchdog and status flags for PRND.
	 */
    struct {
    	bool IsConnected; 			///< Connection flag (Watchdog). True if valid frames received recently.
    	uint8_t RawStatus;			///< Current status/command.
    	uint32_t LastMsgTick;		///< Timestamp (HAL_GetTick) of the last received CAN frame.
    } PRND;


} VehicleState_t;

/**
 * @enum EngineState_e
 * @brief Internal flags for the Engine/Inverter control logic.
 */

typedef enum{
	ENGINE_RUN,  					///< Engine is in Operational mode (Torque enabled).
	ENGINE_STOP_NEUTRAL,  			///< Engine is in Stopped mode (Torque disabled).
	ENGINE_STOP_BLOCKED 			///< Engine is in Pre-Operational (NEUTRAL).
}EngineState_e;


/**
 * @enum WhichEngine_e
 * @brief Used for sending data to inverters.
 */

typedef enum{
	LEFT_ENGINE,
	RIGHT_ENGINE
}WhichEngine_en;

/**
 * @enum ChargerState_e
 * @brief Status codes for the charging process.
 */

typedef enum{
	START_CHARGING, 				///< System is currently charging or requesting charge.
	STOP_CHARGING   				///< System is not charging.
}ChargerState_e;

/**
 * @enum MachineState_e
 * @brief States for the central Finite State Machine (FSM).
 * @details The states are divided into two main branches based on the availability
 * of the autonomous computer (Jetson).
 */

typedef enum{
	// --- JETSON WORKS BRANCH ---
	JTSN_WORKS_CHARGE_STATE,        ///< Jetson connected, Vehicle is charging.
	JTSN_WORKS_NEUTRAL_GEAR_STATE,  ///< Jetson connected, Gear is Neutral.
	JTSN_WORKS_DRIVE_STATE,         ///< Jetson connected, Active Drive mode (MCU passes data to Jetson).

	// --- JETSON DOWN BRANCH ---
	JTSN_DOWN_CHARGE_STATE,         ///< Jetson lost/disconnected, Vehicle is charging.
	JTSN_DOWN_NEUTRAL_GEAR_STATE,   ///< Jetson lost/disconnected, Gear is Neutral.
	JTSN_DOWN_DRIVE_STATE,          ///< Jetson lost/disconnected, Direct Manual Drive.
	JTSN_DOWN_PARKING_STATE,		///< Jetson lost/disconnected, Gear is Parking.
	JTSN_DOWN_REVERSE_STATE,		///< Jetson lost/disconnected, Gear is Reverse.
	SAFE_STOP_STATE					///< No signal from the PRND/charger, cutting off accel. signal from the engines.

}MachineState_e;

/**
 * @enum PRND_State_e
 * @brief Currently used gear.
 */

typedef enum{
	PARKING_GEAR,					///< Parking gear in.
	REVERSE_GEAR,					///< Reverse gear in.
	NEUTRAL_GEAR,					///< Neutral gear in.
	DRIVE_GEAR						///< Drive gear in.
}PRND_State_e;


/**
 * @enum ADC_state_e
 * @brief States for the ADC.
 * @details The states are divided into two, whether given ADC is ready or not.
 */
typedef enum{
	NOT_READY,
	READY
}ADC_state_e;


/**
 * @enum CountedVal_e
 * @brief Describes plausibility of counted values
 * @details Used to represent whether counted values are within accepted error range.
 */
typedef enum{
	GOOD,
	BAD
}CountedVal_e;

#endif
