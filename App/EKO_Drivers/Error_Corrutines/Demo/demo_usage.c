/**
 * @file demo_usage.c
 * @brief Demonstration of the PERLA CAN Error Handler with concurrent fault multiplexing
 * @author AGH EKO-ENERGIA
 */

#include "main.h"
#include "error_handler.h"
#include "can_driver.h"

/* Node definition: Pedals node using Kvasser ID logic */
#define MY_NODE_ID (64 >> 5)

/* Global references (simulating main.c configuration) */
extern CAN_HandleTypeDef hcan;
struct CAN_scheduledMsgList schedulerList = {0};
static EH_HandleTypeDef hErrorHandler = {0};

void app_main(void)
{
	/* Initialize CAN */
	/* Configure filter, start, etc. done in CAN_Init */
	/* We now pass the scheduler list to be handled internally by the driver */
	CAN_Init(&hcan);

	/* 
	 * Initialize the Error Handler and attach it to the CAN scheduler 
	 * This automatically queues the Heartbeat OK message (1s period).
	 */
	EH_init(&hErrorHandler, &hcan, MY_NODE_ID, &schedulerList);

	uint32_t start_ms = HAL_GetTick();
	uint8_t errorsAdded = 0;

	/* Main application loop */
	while (1)
	{
		/* 
		 * Process queued CAN messages (Heartbeats or Multiplexed Error Frames)
		 * Must be called continuously in the loop to handle period intervals.
		 */
		CAN_HandleScheduled(&hcan, &schedulerList);

		uint32_t elapsed_ms = HAL_GetTick() - start_ms;

		/* 
		 * PHASE 1 (0s - 5s): Testing Heartbeat
		 * Heartbeat OK message (0xFFFF) is sent dynamically every 1000ms.
		 */

		/* 
		 * PHASE 2 (5s - 15s): Fault Injection and Multiplexing
		 * Sequentially adding multiple errors to test priority eviction and time-scaling.
		 */
		if (elapsed_ms >= 5000 && elapsed_ms < 15000)
		{
			if (elapsed_ms >= 5000 && errorsAdded == 0) {
				/* Inject Warning #1 (300ms cycle) */
				EH_report(&hErrorHandler, 0x0100, ERROR_SEVERITY_WARNING);
				errorsAdded++;
			}
			else if (elapsed_ms >= 7000 && errorsAdded == 1) {
				/* Inject Warning #2 (270ms cycle) */
				EH_report(&hErrorHandler, 0x0200, ERROR_SEVERITY_WARNING);
				errorsAdded++;
			}
			else if (elapsed_ms >= 9000 && errorsAdded == 2) {
				/* Inject Error #1 (240ms cycle) */
				EH_report(&hErrorHandler, 0x0300, ERROR_SEVERITY_ERROR);
				errorsAdded++;
			}
			else if (elapsed_ms >= 11000 && errorsAdded == 3) {
				/* Inject Warning #3 & Clear Older Faults (270ms cycle recovery) */
				EH_report(&hErrorHandler, 0x0400, ERROR_SEVERITY_WARNING);

				/* Dynamically clear the first 3 registered errors */
				EH_clear(&hErrorHandler, 0x0100);
				EH_clear(&hErrorHandler, 0x0200);
				EH_clear(&hErrorHandler, 0x0300);
				
				errorsAdded++;
			}
			else if (elapsed_ms >= 13000 && errorsAdded == 4) {
				/* Inject Error #2 (240ms cycle limit reached) */
				EH_report(&hErrorHandler, 0x0500, ERROR_SEVERITY_ERROR);
				errorsAdded++;
			}
		}

		/* 
		 * PHASE 3 (20s+): Critical Safe State Halt
		 */
		if (elapsed_ms > 20000)
		{
			/* 
			 * Halt the node due to a critical SAFE_STATE fault.
			 * The system blocks here but retains CAN transmission multiplexing 
			 * for all active errors prior to the emergency vehicle state.
			 */
			EH_stop(&hErrorHandler, 0xDEAD, ERROR_SEVERITY_SAFE_STATE);
		}

		/* Process background application tasks */
		HAL_Delay(10);
	}
}
