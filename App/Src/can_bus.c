/**
  ******************************************************************************
  * @file    can_bus.c
  * @brief   Implementation of CAN Bus signal extraction and frame processing.
  * @author  [Michał Jurek]
  * @date    2025-03-31
  * * @details This file contains functions to:
  * - Extract physical values (float) from raw CAN frames.
  * - Dispatch received frames based on their CAN ID (Charger, Jetson).
  * - Handle the HAL CAN RX callback.
  ******************************************************************************
  */

#include "can_bus.h"
#include "vehicle_types.h"

/** @brief General CAN Header (used for status reports to Jetson). */
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailBox;

/** @brief CAN Header for left Inverter Control (ID 0x226). */
CAN_TxHeaderTypeDef TxHeaderTHL;

/** @brief CAN Header for right Inverter Control (ID 0x227). */
CAN_TxHeaderTypeDef TxHeaderTHR;

/** @brief CAN Header for Network Management (NMT) (ID 0x0). */
CAN_TxHeaderTypeDef TxHeaderNMT;


/** @brief Configuration for extracting the "CONTROL" signal from the Charger CAN frame. */
const CAN_SignalConfig_t SIG_CHARGER_CONTROL = {
    .startBit = 32,
    .length = 8,
    .factor = 1.0f,
    .offset = 0.0f,
    .isSigned = false
};

/** @brief Configuration for extracting the "CONTROL" signal from the PRND CAN frame. */
const CAN_SignalConfig_t SIG_PRND_CONTROL = {
    .startBit = 14,
    .length = 2,
    .factor = 1.0f,
    .offset = 0.0f,
    .isSigned = false
};


/**
 * @brief  Extracts a physical signal value from a raw CAN frame based on configuration.
 *
 * * @details This function interprets a specific sequence of bits within the 8-byte
 * CAN payload as a number (signed or unsigned), applies a scaling factor
 * and an offset, and returns the physical value (e.g., voltage, speed).
 *
 * * @param[in]  frameData Pointer to the 8-byte raw data buffer from the CAN frame.
 * @param[in]  config    Structure containing signal parameters (start bit, length,
 * factor, offset, signedness).
 * @param[out] outValue  Pointer to a float variable where the result will be stored.
 * * @return bool
 *
 * @retval true  Signal extracted successfully.
 * @retval false Input pointers were NULL.
 */
bool CAN_ExtractSignal(const uint8_t* frameData, const CAN_SignalConfig_t *config, float* outValue) {
    if ( NULL == frameData || NULL == outValue) {	// performing check to make sure used variables are not empty
        return false;
    }

    uint64_t raw64 = 0;		//storage for our data to perform operations on

    // Copy 8 bytes to a 64-bit integer to handle bitwise operations across byte boundaries
    memcpy(&raw64, frameData, 8);

    // Create a mask for the specific bit length
    uint64_t mask = (1ULL << config->length) - 1;

    // Shift and mask to get the raw integer value
    uint64_t rawValue = (raw64 >> config->startBit) & mask;

    // Handle signed numbers
    if (config->isSigned) {
    	// Check if the sign bit is set
        if (rawValue & (1ULL << (config->length - 1))) {
            rawValue |= ~mask; // Sign extension
        }

        int64_t signedRaw = (int64_t)rawValue;
        *outValue = (float)signedRaw * config->factor + config->offset;

    } else {
    	// Unsigned calculation
        *outValue = (float)rawValue * config->factor + config->offset;
    }

    return true;
}
/**
 * @brief  Processes a received CAN frame and updates the Vehicle state.
 *
 * * @details This function checks the CAN ID of the incoming message and routes
 * the data to the appropriate part of the global @ref Vehicle structure.
 * It handles:
 * - Charger status (ExtID: 0x1806E5F4)
 * - Jetson data (StdID: 0x200)
 *
 * * @param[in] pHeader Pointer to the CAN Rx Header structure containing ID, IDE, DLC, etc.
 * @param[in] data    Pointer to the payload data (8 bytes).
 *
 * * @note   Updates the `LastMsgTick` for connection watchdog monitoring.
 */
void CAN_ProcessFrame(CAN_RxHeaderTypeDef *pHeader, uint8_t* data) {

    // Checking the charger (Extended ID: 0x1806E5F4)
    if (pHeader->IDE == CAN_ID_EXT && pHeader->ExtId == 0x1806E5F4) {
        float val;
        // processing signal CONTROL
        if (CAN_ExtractSignal(data, &SIG_CHARGER_CONTROL, &val)) {
            Vehicle.Charger.RawStatus = (uint8_t)val;
            Vehicle.Charger.LastMsgTick = HAL_GetTick();
            Vehicle.Charger.IsConnected = true;
        }
    }

    // Checking JETSON (Standard ID: 0x200)
    if (pHeader->IDE == CAN_ID_STD && pHeader->StdId == 0x200) {
        // coping data and marking the time

    	// if not used comment also memset((void*)Vehicle.Jetson.RawData, 0, 8); in HAL_TIM_PeriodElapsedCallback
        //memcpy((void*)Vehicle.Jetson.RawData, data, 8); 		// in case when some operations will be needed to perform on the data from Jetson
        Vehicle.Jetson.LastMsgTick = HAL_GetTick();
        Vehicle.Jetson.IsConnected = true;
    }

    // Checking PRND UNKNOWN ID FIX IT LATER
    //	assuming random id for tests
    if (pHeader-> IDE == CAN_ID_STD && pHeader->StdId == 0x3e1){
    	float val;
    	if (CAN_ExtractSignal(data, &SIG_PRND_CONTROL, &val))
		{
    		Vehicle.PRND.RawStatus = (uint8_t)val;
			Vehicle.PRND.LastMsgTick = HAL_GetTick();
			Vehicle.PRND.IsConnected = true;
		}
    }
}
/**
 * @brief  Rx FIFO 0 message pending callback.
 *
 * * @details This function is called by the HAL library when a new CAN message
 * arrives in FIFO 0. It retrieves the message and passes it to
 *
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

/**
 * @brief  Configures custom CAN filters and starts the CAN peripheral.
 *
 * @details This function sets up the hardware filter banks to accept only
 * specific messages required by the VCU, significantly reducing
 * CPU load. It configures the following filter banks:
 * - Bank 0: Charger (Extended ID: 0x1806E5F4)
 * - Bank 1: Jetson (Standard ID: 0x200)
 * - Bank 2: PRND (Standard ID: 0x420)
 * * After configuring the filters to route accepted messages into RX FIFO0,
 * it activates the FIFO0 message pending interrupt and starts the CAN module.
 *
 * * @note    If any of the HAL CAN configuration functions fail, this function
 * will block execution by calling Error_Handler().
 *
 * @param[in,out] hcan Pointer to a CAN_HandleTypeDef structure that contains
 * the configuration information for the specified CAN peripheral.
 * * @retval None
 */
void CAN_Custom_Init(CAN_HandleTypeDef *hcan) {
	CAN_FilterTypeDef filterConfig;

	filterConfig.SlaveStartFilterBank = 14; // dont care (only matters when > 1 CAN)


	// CAN filter config - NEEDS CORRECTION AFTER ARRANGEMENTS ABOUT PRND !!!!!
	filterConfig.FilterMode = CAN_FILTERMODE_IDLIST; // list mode
	filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // messages will go to FIFO0
	filterConfig.FilterActivation = ENABLE; // turning on the filter


	//FILTER 1 - CHARGER (Extended ID: 0x1806E5F4) -> BANK 0
	filterConfig.FilterBank = 0; // choosing bank of filters (0-13)

	//formatting filter frame to get bits on proper positions
	uint32_t extendedID_Formatted = (0x1806E5F4 << 3) | 4;

	// divide into two 16-bit parts for STM32 registers
	filterConfig.FilterIdHigh = (extendedID_Formatted >> 16) & 0xFFFF;
	filterConfig.FilterIdLow  = (extendedID_Formatted & 0xFFFF);

	filterConfig.FilterMaskIdHigh = 0;
	filterConfig.FilterMaskIdLow  = 0;

	if (HAL_CAN_ConfigFilter(hcan, &filterConfig) != HAL_OK) {
	  Error_Handler();
	}

	// FILTER 2 - JETSON (Standard ID: 0x200) -> BANK 1
	filterConfig.FilterBank = 1;

	filterConfig.FilterIdHigh = (0x200 << 5);
	filterConfig.FilterIdLow  = 0;

	filterConfig.FilterMaskIdHigh = 0;
	filterConfig.FilterMaskIdLow  = 0;

	if (HAL_CAN_ConfigFilter(hcan, &filterConfig) != HAL_OK) {
		  Error_Handler();
		}

	// FILTER 3 - PRND (Standard ID: 0x420) -> BANK 2
	filterConfig.FilterBank = 2;
	filterConfig.FilterIdHigh = (0x3e1 << 5);
	filterConfig.FilterIdLow  = 0;
	filterConfig.FilterMaskIdHigh = 0;
	filterConfig.FilterMaskIdLow  = 0;
    if (HAL_CAN_ConfigFilter(hcan, &filterConfig) != HAL_OK) { Error_Handler(); }

    // Turning on interrupts and CAN
    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_Start(hcan) != HAL_OK) {
        Error_Handler();
    }
    HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);


	// Throttle/Torque Header (Direct Left Engine Control)
	TxHeaderTHL.StdId = 0x226;    // LEFT inverter throttle address
	TxHeaderTHL.IDE = CAN_ID_STD;
	TxHeaderTHL.RTR = CAN_RTR_DATA;
	TxHeaderTHL.DLC = 8;

	// Throttle/Torque Header (Direct Right Engine Control)
	TxHeaderTHR.StdId = 0x227;    // RIGHT inverter throttle address
	TxHeaderTHR.IDE = CAN_ID_STD;
	TxHeaderTHR.RTR = CAN_RTR_DATA;
	TxHeaderTHR.DLC = 8;

	// NMT Header (Engine Control)
	TxHeaderNMT.StdId = 0x0;	// Address to change the working state of both engines
	TxHeaderNMT.IDE = CAN_ID_STD;
	TxHeaderNMT.RTR = CAN_RTR_DATA;
	TxHeaderNMT.DLC = 2;

	// General status header of this node
	TxHeader.StdId = 0x41;     // 65 (decimal)
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 4;
}

