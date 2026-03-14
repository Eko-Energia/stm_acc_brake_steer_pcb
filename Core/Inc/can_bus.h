/**
  ******************************************************************************
  * @file    can_bus.h
  * @brief   Header file for CAN frame parsing and signal extraction.
  * @author  [Michał Jurek]
  * @date    2025-01-29
  * @details This file declares the structures and functions required to:
  * - Define signal parameters (Start Bit, Length, Factor, Offset).
  * - Extract physical values from raw CAN byte arrays.
  * - Handle the CAN RX interrupt and route data to the global Vehicle state.
  ******************************************************************************
  */
#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // do memcpy
#include "vehicle_types.h"
#include "can.h"

/**
 * @brief External reference to the global Vehicle State.
 * Defined in main.c, modified here upon receiving new CAN frames.
 */
extern volatile VehicleState_t Vehicle;


/**
 * @struct CAN_Signal_Config_t
 * @brief  Configuration structure for extracting a single signal from a CAN frame.
 * @details Describes how to interpret a specific sequence of bits within the
 * 8-byte data field. Corresponds to signal definitions found in .DBC files.
 */
typedef struct {
		uint8_t 	startBit;      	/**< Start bit position (0-63). Usually LSB for Intel byte order. */
	    uint8_t 	length;        	/**< Length of the signal in bits (1-64). */
	    float 		factor;         /**< Scaling factor. Physical = (Raw * Factor) + Offset. */
	    float 		offset;			/**< Offset value. Physical = (Raw * Factor) + Offset. */
	    bool 		isSigned;       /**< True if the raw value is a signed integer (Two's complement). */
} CAN_Signal_Config_t;

/**
 * @brief Configuration for the "CONTROL" signal from the Charger.
 * Frame ID: 0x1806E5F4 (Extended).
 * Contains status bits defining if charging is active.
 */
extern const CAN_Signal_Config_t SIG_CHARGER_CONTROL;


extern const CAN_Signal_Config_t SIG_PRND_CONTROL;
/**
 * @brief  Extracts a physical value from a raw CAN frame payload.
 * @details Unpacks bits based on the provided configuration, handles sign extension
 * for signed values, and applies the linear scaling formula (y = ax + b).
 * * @param[in]  frameData Pointer to the 8-byte raw data buffer.
 * @param[in]  config    Signal configuration (start bit, length, etc.).
 * @param[out] outValue  Pointer to float where the result will be stored.
 * * @return bool
 * @retval true  Extraction successful.
 * @retval false Input pointers were NULL.
 */
bool CAN_ExtractSignal(const uint8_t* frameData, CAN_Signal_Config_t config, float* outValue);

/**
 * @brief  Processes a received CAN frame and updates the global Vehicle structure.
 * @details Identifies the message by its ID (Std or Ext) and calls the appropriate
 * parsing logic or extraction function.
 * * @param[in] pHeader Pointer to the CAN Rx Header (ID, IDE, DLC).
 * @param[in] data    Pointer to the 8-byte payload.
 */
void CAN_ProcessFrame(CAN_RxHeaderTypeDef *pHeader, uint8_t* data);

/**
 * @brief  HAL CAN Rx FIFO 0 Msg Pending Callback.
 * @details This function is called by the HAL ISR when a message arrives in FIFO0.
 * It reads the message using @ref HAL_CAN_GetRxMessage and passes it to
 * @ref CAN_ProcessFrame.
 * * @param[in] hcan Pointer to the CAN handle.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


#endif
