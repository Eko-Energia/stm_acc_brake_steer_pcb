/**
  * @file can_driver.h
  * @brief CAN bus driver for PERLA
  * @author AGH EKO-ENERGIA
  * @author Kacper Lasota
  */

#ifndef INC_CAN_DRIVER_H_
#define INC_CAN_DRIVER_H_

#include "main.h"
#include "can_id_list.h"
#include <stdio.h>

/**
 * Defines
 */

#define CAN_MAX_DLC 8
#define CAN_MAX_MSG 32

/**
 * Periodic CAN message
 */
typedef struct {
	CAN_TxHeaderTypeDef header;							// frame header
	uint32_t 			period_ms;						// period of this message
	uint32_t 			last_tick;						// time stamp of the last message
	void 				(*GetData)(uint8_t *data);		// fetches data
}CAN_ScheduledMsg;

/**
 * Periodic CAN message list used for automation
 */
typedef struct {
	CAN_ScheduledMsg list[CAN_MAX_MSG];
	uint8_t size;
	uint32_t txMailbox;
}CAN_ScheduledMsgList;

/**
 * Setup functions
 */
void CAN_Init(CAN_HandleTypeDef*);

/**
 * Functions for scheduled messages
 */
HAL_StatusTypeDef CAN_AddScheduledMessage(CAN_ScheduledMsg, CAN_ScheduledMsgList*);
HAL_StatusTypeDef CAN_RemoveScheduledMessage(uint32_t, CAN_ScheduledMsgList*);

void CAN_HandleScheduled(CAN_HandleTypeDef *hcan, CAN_ScheduledMsgList*);

/**
 * Functions for received messages
 */
void CAN_HandleReceived(CAN_HandleTypeDef *hcan, uint8_t fifo);


#endif /* INC_CAN_DRIVER_H_ */
