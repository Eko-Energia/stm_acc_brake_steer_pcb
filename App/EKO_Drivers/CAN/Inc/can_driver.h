
/**
 * @file can_driver.h
 * @brief CAN bus driver for PERLA
 * @author AGH EKO-ENERGIA
 * @author Kacper Lasota
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "can_id_list.h"
#include "main.h"
#include <stdio.h>

/**
 * Defines
 */

#define CAN_MAX_DLC (8)
#define CAN_MAX_MSG (32)

/**
 * @brief Generic macro to swap endianness based on variable type.~
 * 
 * Endiannes should be handlend in GetData function of every 
 * * usage: 
 * uint32_t val = 0x12345678;
 * val = SWAP_ENDIANNESS(val); // Becomes 0x78563412
 */
#define SWAP_ENDIANNESS(x) _Generic((x),       \
    uint8_t:  (x),                             \
    int8_t:   (x),                             \
    uint16_t: __builtin_bswap16(x),                  \
    int16_t:  __builtin_bswap16(x),                  \
    uint32_t: __builtin_bswap32(x),                  \
    int32_t:  __builtin_bswap32(x),                  \
    uint64_t: __builtin_bswap64(x),                  \
    int64_t:  __builtin_bswap64(x)                   \
)

/**
 * @brief Extracts the n-th byte from variable x.
 * @warning Do not pass expressions with side effects (e.g., x++) as arguments,
 * as they may be evaluated multiple times.
 * @param x The source variable (uint8_t, uint16_t, or uint32_t).
 * @param n The byte index (0 for LSB).
 */
#define GET_BYTE(x, n) ((uint8_t)(((x) >> ((n) * 8u)) & 0xFFu))

/**
 * Periodic CAN message
 */
struct CAN_scheduledMsg
{
	CAN_TxHeaderTypeDef header;     // frame header
	uint32_t periodMs;              // period of this message
	uint32_t lastTick;              // time stamp of the last message
	void (*getData)(uint8_t *data, void *context); // fetches data
	void *context;                  // user callback context
};

/**
 * Periodic CAN message list used for automation
 */
struct CAN_scheduledMsgList
{
	struct CAN_scheduledMsg list[CAN_MAX_MSG];
	uint8_t size;
	uint32_t txMailbox;
};

/**
 * Setup functions
 */

/**
 * @brief Initialize CAN
 *
 * This function initializes the CAN peripheral.
 *
 * @param hcan      Pointer to CAN handle
 */
void CAN_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief Process all scheduled CAN messages (call in main loop)
 *
 * @param hcanPtr      Pointer to CAN handle
 * @param scheduler    Pointer to the message scheduler
 */
void CAN_HandleScheduled(CAN_HandleTypeDef *hcanPtr, struct CAN_scheduledMsgList *scheduler);

/**
 * Functions for scheduled messages
 */

/**
 * @brief Add new message to the periodic buffer
 *
 * @param msg      Pointer to the message to add
 * @param buffer   Pointer to the buffer that holds messages
 * @retval HAL_StatusTypeDef   State of the operation
 */
HAL_StatusTypeDef CAN_AddScheduledMsg(const struct CAN_scheduledMsg *msg, struct CAN_scheduledMsgList *buffer);

/**
 * @brief Remove message from the periodic buffer
 *
 * @param id       ID of the message to remove
 * @param buffer   Pointer to the buffer that holds messages
 * @retval HAL_StatusTypeDef   State of the operation
 */
HAL_StatusTypeDef CAN_RemoveScheduledMsg(uint32_t id, struct CAN_scheduledMsgList *buffer);

#endif /* CAN_DRIVER_H */
