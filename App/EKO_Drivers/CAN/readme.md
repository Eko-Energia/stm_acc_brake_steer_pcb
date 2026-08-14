# CAN_DRIVER
The driver is designed to simplify the usage of CAN bus communication on STM32F303 (might work on other MCUs). Features  of the module are designed specificaly to be used in PERLA - a solar car built by AGH Eko-Energia

## Limitations
The module currently sets filter banks to accept every incoming frame by default. (TBD)

## Usage

### Initialization
Requires HAL CAN instance to be imported from main.c as well as a buffer `CAN_scheduledMsgList`
```C
// app.c

extern CAN_HandleTypeDef;

struct CAN_scheduledMsgList buffer;

void app_main(void)
{
    CAN_Init(&hcan);
    ...
}

```
Unlimited number of buffers can be created, to change maximum number of messages for one buffer edit `CAN_MAX_MSG`

### GetData
Creating a frame requires a function that will fill the outgoing buffer with data on every call. 

```C
static void chargerGetData(uint8_t *data, void *context)
{
	// preserve one decimal place
	float maxCurrent = maxChargerCurrent * 10;
	float maxVoltage = MAX_CHARGER_VOLTAGE * 10;

	uint16_t maxChargerCurrentInt = (uint16_t) maxCurrent;
	uint16_t maxVoltageInt = (uint16_t) maxVoltage;

	data[0] = GET_BYTE(maxVoltageInt, 0);
	data[1] = GET_BYTE(maxVoltageInt, 1);
	data[2] = GET_BYTE(maxChargerCurrentInt, 0);
	data[3] = GET_BYTE(maxChargerCurrentInt, 1);
}
```


### Adding a frame to the buffer
```C
struct CAN_scheduledMsg chargerComms;

chargerComms.header.extId = CANID_RCD_STATIC_CHARGER1COMMS;
chargerComms.header.DLC = 3;
chargerComms.header.IDE = CAN_ID_EXT;
chargerComms.header.RTR = CAN_RTR_DATA;
chargerComms.lastTick = 0;
chargerComms.periodMs = 1000;
chargerComms.getData = chargerGetData;

CAN_AddScheduledMsg(chargerComms, &CAN_buffer);
```

### Handling messages added to the buffer

The function should propably be on the end of the main loop
```C
CAN_HandleScheduled(&hcan, &buffer);
```

### Removing a frame
```C
CAN_RemoveScheduledMsg(CANID_RCD_STATIC_CHARGER1COMMS);
```




# TO-DO
- [ ] Add non-periodic frame handling (period = 0), send once on handle and delete
- [ ] Handling on bus errors and generic error messages
- [ ] Filter configuration
- [ ] Handling of received messages