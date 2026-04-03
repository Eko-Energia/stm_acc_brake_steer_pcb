/**
 * @file error_handler.h
 * @brief Error handling and reporting for PERLA CAN network
 * @author AGH EKO-ENERGIA
 *
 * @details This module provides standardized error reporting for the PERLA CAN network.
 *          Errors are transmitted as CAN frames with a specific Message ID offset.
 *
 *          Supports STM32 bxCAN architecture.
 */

#ifndef EH_H
#define EH_H

#include "main.h"
#include "can_driver.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ============================================================================
 * Configuration
 * ============================================================================ */

/** @brief Error frame DLC (8 bytes payload) */
#define ERROR_FRAME_DLC (8)

/** @brief Safe State frame ID (highest priority) */
#define SAFE_STATE_FRAME_ID (0x000)

/** @brief Maximum diagnostic data bytes */
#define ERROR_SPECIFIC_DATA_SIZE (5)

/** @brief Heartbeat OK error code */
#define HEARTBEAT_ERROR_CODE (0xFFFF)

/** @brief Interval of heartbeat message */
#define HEARTBEAT_INTERVAL (1000)

/** @brief Interval of error message */
#define ERROR_INTERVAL (300)

/* ============================================================================
 * Severity Levels
 * ============================================================================ */

/**
 * @brief Error severity levels
 */
typedef enum
{
	ERROR_SEVERITY_SAFE_STATE = 0, /**< Critical: requires emergency power disconnect */
	ERROR_SEVERITY_ERROR,          /**< Pit stop: requires immediate service intervention */
	ERROR_SEVERITY_WARNING,        /**< Something should be fixed, but safe to continue driving */
	ERROR_SEVERITY_INFO            /**< Maintenance info, good to know, no immediate action needed */
} errorSeverity_e;

/* ============================================================================
 * Error Frame Structures
 * ============================================================================ */

/**
 * @brief Error flags bitfield structure
 * Bit layout (Byte 2):
 *   Bit 0 (MSB): HALTED
 *   Bits 1-3:    SEVERITY
 *   Bits 4-7:    RESERVED
 */
struct errorFlags
{
	uint8_t halted : 1;
	uint8_t severity : 3;
	uint8_t reserved : 4;
};

/**
 * @brief Error frame payload structure
 */
struct errorFramePayload
{
	uint16_t errorCode;
	struct errorFlags flags;
	uint8_t specificData[ERROR_SPECIFIC_DATA_SIZE];
};

/* ============================================================================
 * Handle Structure
 * ============================================================================ */

/**
 * @brief Error Handler Handle Structure
 */
typedef struct
{
	CAN_HandleTypeDef *phcan;                                /**< CAN Handle */
	struct CAN_scheduledMsgList *scheduler;                  /**< Pointer to scheduler */
	uint16_t nodeId;                                         /**< Local Node ID */
	uint32_t errorFrameId;                                   /**< Calculated Error Frame ID */
	uint16_t activeErrorCode;                                /**< Active Error Code */
	errorSeverity_e activeSeverity;                          /**< Active Severity */
	uint8_t activeSpecificData[ERROR_SPECIFIC_DATA_SIZE];    /**< Specific Data Buffer */
	uint8_t activeSpecificDataLen;                           /**< Specific Data Length */
	uint8_t isInitialized;                                   /**< Module initialized flag */
	uint8_t isHalted;                                        /**< Node Halted Flag */
} EH_HandleTypeDef;

/* ============================================================================
 * Initialization
 * ============================================================================ */

/**
 * @brief Initialize error handler with bxCAN
 *
 * @param hehandler Pointer to Error Handler handle
 * @param hcan      Pointer to CAN handle
 * @param nodeId    Node ID (used as error frame ID)
 * @param scheduler Pointer to the CAN scheduled message list
 */
void EH_init(EH_HandleTypeDef *hehandler, CAN_HandleTypeDef *hcan, uint16_t nodeId, struct CAN_scheduledMsgList *scheduler);

/* ============================================================================
 * Error Reporting Functions
 * ============================================================================ */

/**
 * @brief Report an error without stopping the node
 * @param hehandler    Pointer to Error Handler handle
 * @param errorCode    Unique error code
 * @param severity     Error severity level
 */
void EH_report(EH_HandleTypeDef *hehandler, uint16_t errorCode, errorSeverity_e severity);

/**
 * @brief Report an error and halt the node
 * @param hehandler    Pointer to Error Handler handle
 * @param errorCode    Unique error code
 * @param severity     Error severity level
 */
void EH_stop(EH_HandleTypeDef *hehandler, uint16_t errorCode, errorSeverity_e severity);

/**
 * @brief Trigger system-wide Safe State
 * @param hehandler Pointer to Error Handler handle
 * @param reason    Reason code
 */
void EH_triggerSafeState(EH_HandleTypeDef *hehandler, uint16_t reason);

/* ============================================================================
 * Extended Error Reporting (with diagnostic data)
 * ============================================================================ */

/**
 * @brief Report an error with additional diagnostic data
 */
void EH_reportEx(EH_HandleTypeDef *hehandler, uint16_t errorCode, errorSeverity_e severity, const uint8_t *data, uint8_t dataLen);

/**
 * @brief Report an error with diagnostic data and halt the node
 */
void EH_stopEx(EH_HandleTypeDef *hehandler, uint16_t errorCode, errorSeverity_e severity, const uint8_t *data, uint8_t dataLen);

/**
 * @brief Clear a specific error from the active error state
 *
 * Checks if there are no more active errors. If so, returns to Heartbeat OK.
 * @param hehandler Pointer to Error Handler handle
 * @param errorCode The error code to clear
 */
void EH_clear(EH_HandleTypeDef *hehandler, uint16_t errorCode);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Get the configured Node ID
 * @param hehandler Pointer to Error Handler handle
 * @return Current node ID
 */
uint16_t EH_getNodeId(EH_HandleTypeDef *hehandler);

/**
 * @brief Check if error handler is initialized
 * @param hehandler Pointer to Error Handler handle
 * @return 1 if initialized, 0 otherwise
 */
uint8_t EH_isInitialized(EH_HandleTypeDef *hehandler);

#ifdef __cplusplus
}
#endif

#endif // EH_H
