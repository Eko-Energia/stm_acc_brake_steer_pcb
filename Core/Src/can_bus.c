/**
  ******************************************************************************
  * @file    can_bus.c
  * @brief   Implementation of CAN Bus signal extraction and frame processing.
  * @author  [Michał Jurek]
  * @date    2025-01-29
  * * @details This file contains functions to:
  * - Extract physical values (float) from raw CAN frames.
  * - Dispatch received frames based on their CAN ID (Charger, Jetson).
  * - Handle the HAL CAN RX callback.
  ******************************************************************************
  */

#include "can_bus.h"
#include "vehicle_types.h"

/**
 * @brief  Extracts a physical signal value from a raw CAN frame based on configuration.
 * * @details This function interprets a specific sequence of bits within the 8-byte
 * CAN payload as a number (signed or unsigned), applies a scaling factor
 * and an offset, and returns the physical value (e.g., voltage, speed).
 * * @param[in]  frameData Pointer to the 8-byte raw data buffer from the CAN frame.
 * @param[in]  config    Structure containing signal parameters (start bit, length,
 * factor, offset, signedness).
 * @param[out] outValue  Pointer to a float variable where the result will be stored.
 * * @return bool
 * @retval true  Signal extracted successfully.
 * @retval false Input pointers were NULL.
 */
bool CAN_ExtractSignal(const uint8_t* frameData, CAN_Signal_Config_t config, float* outValue) {
    if (frameData == NULL || outValue == NULL) {	// performing check to make sure used variables are not empty
        return false;
    }

    uint64_t raw64 = 0;		//storage for our data to perform operations on

    // Copy 8 bytes to a 64-bit integer to handle bitwise operations across byte boundaries
    memcpy(&raw64, frameData, 8);

    // Create a mask for the specific bit length
    uint64_t mask = (1ULL << config.length) - 1;

    // Shift and mask to get the raw integer value
    uint64_t rawValue = (raw64 >> config.startBit) & mask;

    // Handle signed numbers
    if (config.isSigned) {
    	// Check if the sign bit is set
        if (rawValue & (1ULL << (config.length - 1))) {
            rawValue |= ~mask; // Sign extension
        }

        int64_t signedRaw = (int64_t)rawValue;
        *outValue = (float)signedRaw * config.factor + config.offset;

    } else {
    	// Unsigned calculation
        *outValue = (float)rawValue * config.factor + config.offset;
    }

    return true;
}
/**
 * @brief  Processes a received CAN frame and updates the Vehicle state.
 * * @details This function checks the CAN ID of the incoming message and routes
 * the data to the appropriate part of the global @ref Vehicle structure.
 * It handles:
 * - Charger status (ExtID: 0x1806E5F4)
 * - Jetson data (StdID: 0x512)
 * * @param[in] pHeader Pointer to the CAN Rx Header structure containing ID, IDE, DLC, etc.
 * @param[in] data    Pointer to the payload data (8 bytes).
 * * @note   Updates the `LastMsgTick` for connection watchdog monitoring.
 */
void CAN_ProcessFrame(CAN_RxHeaderTypeDef *pHeader, uint8_t* data) {

    // Checking the charger (Extended ID: 0x1806E5F4)
    if (pHeader->IDE == CAN_ID_EXT && pHeader->ExtId == 0x1806E5F4) {
        float val;
        // processing signal CONTROL
        if (CAN_ExtractSignal(data, SIG_CHARGER_CONTROL, &val)) {
            Vehicle.Charger.RawStatus = (uint8_t)val;
            Vehicle.Charger.LastMsgTick = HAL_GetTick();
            Vehicle.Charger.IsConnected = true;
        }
    }

    // Checking JETSON (Standard ID: 0x512)
    if (pHeader->IDE == CAN_ID_STD && pHeader->StdId == 0x512) {
        // coping data and marking the time

    	// if not used comment also memset((void*)Vehicle.Jetson.RawData, 0, 8); in HAL_TIM_PeriodElapsedCallback
        //memcpy((void*)Vehicle.Jetson.RawData, data, 8); 		// in case when some operations will be needed to perform on the data from Jetson
        Vehicle.Jetson.LastMsgTick = HAL_GetTick();
        Vehicle.Jetson.IsConnected = true;
    }

    // Checking PRND UNKNOWN ID FIX IT LATER
    //	assuming random id for tests
    if (pHeader-> IDE == CAN_ID_STD && pHeader->StdId == 0x420){
    	float val;
    	if (CAN_ExtractSignal(data, SIG_PRND_CONTROL, &val))
		{
    		Vehicle.PRND.RawStatus = (uint8_t)val;
			Vehicle.PRND.LastMsgTick = HAL_GetTick();
			Vehicle.PRND.IsConnected = true;
		}
    }
}
/**
 * @brief  Rx FIFO 0 message pending callback.
 * * @details This function is called by the HAL library when a new CAN message
 * arrives in FIFO 0. It retrieves the message and passes it to
 * @ref CAN_ProcessFrame for logic handling.
 * * @param[in] hcan Pointer to the CAN handle structure.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        CAN_ProcessFrame(&rxHeader, rxData);
    }
}

