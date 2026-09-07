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
 #include "string.h"
 
 /**
  * Defines
  */
 
 #define CAN_MAX_DLC (8)
 #define CAN_MAX_MSG (32)

 /**
  * @brief Maximum time a frame may occupy a Tx mailbox before it is aborted.
  *
  * @details With AutoRetransmission enabled bxCAN keeps retrying a frame that
  * is not acknowledged, holding its mailbox indefinitely. Three such frames
  * pin all three mailboxes and every further AddTxMessage() fails.
  *
  * Must stay SHORTER than the shortest periodic frame (95 ms). Otherwise two
  * generations of the same ID end up in two mailboxes, and because
  * TransmitFifoPriority is DISABLE bxCAN transmits by identifier - not by
  * request order - so the newer torque value can leave before the older one.
  *
  * 20 ms is ~74 frame slots at 500 kbit/s: a frame that cannot leave within
  * that window is facing a broken bus, not a busy one.
  */
 #define CAN_TX_TIMEOUT_MS (20u)

 /** @brief Number of bxCAN transmit mailboxes. Fixed by the peripheral. */
 #define CAN_TX_MAILBOX_COUNT (3u)

 /** @brief @ref CAN_Diag_t flags, mirroring the matching CAN_ESR bits. */
 #define CAN_DIAG_FLAG_EWGF (1u << 0) /**< Error warning (TEC or REC >= 96). */
 #define CAN_DIAG_FLAG_EPVF (1u << 1) /**< Error passive (TEC or REC > 127). */
 #define CAN_DIAG_FLAG_BOFF (1u << 2) /**< Bus-off (TEC > 255). */

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
  * Incoming CAN message
  */
 struct CAN_IncomingMsg
 {
	 CAN_RxHeaderTypeDef header;
	 uint8_t data[CAN_MAX_DLC];
 };
 
 /**
  * Incoming CAN message buffer
  */
 struct CAN_IncomingMsgList
 {
	 struct CAN_IncomingMsg list[CAN_MAX_MSG];
	 uint8_t count;
	 uint8_t receiveFlag;
	 uint8_t head;
	 uint8_t tail;
 };
 
 /**
  * @brief Bus health snapshot, refreshed by @ref CAN_HandleTxWatchdog.
  *
  * @details Read-only diagnostics. Nothing in the control path depends on it -
  * it exists so a bus fault leaves a trace: with ABOM enabled the hardware
  * recovers from bus-off on its own and would otherwise do so silently.
  */
 struct CAN_Diag
 {
	 uint8_t  tec;            /**< Transmit error counter (CAN_ESR[23:16]). */
	 uint8_t  rec;            /**< Receive error counter (CAN_ESR[31:24]). */
	 uint8_t  lastErrorCode;  /**< LEC (CAN_ESR[6:4]). 3 = acknowledgement error. */
	 uint8_t  flags;          /**< CAN_DIAG_FLAG_* bitmask. */
	 uint16_t txAbortCount;   /**< Mailboxes killed by the watchdog. */
	 uint16_t busOffCount;    /**< Bus-off entries, counted on the rising edge. */
	 uint16_t txFailCount;    /**< AddTxMessage() rejections (all mailboxes busy). */
 };

 /**
  * Setup functions
  */

 /**
  * @brief Initialize CAN peripheral
  *
  * @param hcanPtr   Pointer to CAN handle
  */
 void CAN_Init(CAN_HandleTypeDef *hcan);

 /**
  * @brief Free Tx mailboxes stuck past @ref CAN_TX_TIMEOUT_MS and sample bus health.
  *
  * @details Call from the main loop, before @ref CAN_HandleScheduled.
  *
  * Mailbox state is read from TSR rather than tracked at the AddTxMessage()
  * call sites, so frames sent outside the scheduler - the NMT command from
  * engine_control.c - are covered by the same watchdog.
  *
  * @param hcanPtr   Pointer to CAN handle
  */
 void CAN_HandleTxWatchdog(CAN_HandleTypeDef *hcanPtr);

 /**
  * @brief Access the diagnostics snapshot.
  *
  * @return const struct CAN_Diag* Never NULL.
  */
 const struct CAN_Diag *CAN_GetDiag(void);

 /**
  * Functions for scheduled messages
  */
 
  /**
  * @brief Process all scheduled CAN messages (call in main loop)
  *
  * @param hcanPtr      Pointer to CAN handle
  * @param scheduler    Pointer to the message scheduler
  */
 void CAN_HandleScheduled(CAN_HandleTypeDef *hcanPtr, struct CAN_scheduledMsgList *scheduler);
 
 /**
  * @brief Add new message to the periodic buffer
  *
  * @param msg      Pointer to the message to add
  * @param buffer   Pointer to the buffer that holds messages
  * @retval HAL_StatusTypeDef   State of the operation
  */
 HAL_StatusTypeDef CAN_AddScheduledMsg(struct CAN_scheduledMsg *msg, struct CAN_scheduledMsgList *buffer);
 
 /**
  * @brief Remove message from the periodic buffer
  *
  * @param id       ID of the message to remove
  * @param buffer   Pointer to the buffer that holds messages
  * @retval HAL_StatusTypeDef   State of the operation
  */
 HAL_StatusTypeDef CAN_RemoveScheduledMsg(uint32_t id, struct CAN_scheduledMsgList *buffer);
 
 /* Incoming CAN message buffer */
 
 /**
  * @brief Add incoming CAN message to the buffer
  *
  * @param header  Pointer to received CAN header
  * @param data    Pointer to received CAN payload
  * @retval HAL_StatusTypeDef   State of the operation
  */
 HAL_StatusTypeDef CAN_AddIncomingMsg(struct CAN_IncomingMsgList *buffer, CAN_RxHeaderTypeDef *header, uint8_t *data);
 
/**
 * @brief Read and remove the oldest pending message (FIFO order)
 *
 * @param buffer Pointer to the incoming message buffer
 * @param msg    Pointer to storage for the received message
 * @retval HAL_StatusTypeDef   State of the operation
 */
HAL_StatusTypeDef CAN_GetLatestMessage(struct CAN_IncomingMsgList *buffer, struct CAN_IncomingMsg *msg);
 
 #endif /* CAN_DRIVER_H */