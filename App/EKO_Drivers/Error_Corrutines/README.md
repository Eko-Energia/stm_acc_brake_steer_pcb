# Error Handler Module

Enhanced error reporting and heartbeat module for the STM32 CAN network. integrated with the CAN scheduler.

## Features

- **Standardized Addressing**: Uses `(NodeID << 5) | 0x01` logic for error frame IDs.
- **Scheduled Reporting**: 
  - **Heartbeat (Normal)**: Sends `0xFFFF` every 1000ms.
  - **Error (Active)**: Sends the active error code every 300ms.
- **Node Halted State**: `EH_stop` halts application logic but keeps the CAN bus alive.

## CAN Frame Format

| Bytes | Field         | Type       | Description                       |
| ----- | ------------- | ---------- | --------------------------------- |
| 0-1   | ERROR CODE    | uint16_t   | Unique error code (Little Endian) |
| 2     | FLAGS/SEVERITY| byte       | Bit 0: Halted, Bits 1-3: Severity |
| 3-7   | SPECIFIC DATA | uint8_t[5] | Snapshot of diagnostic data       |

## API Reference

### Initialization
```c
/* Initialize with CAN handle and Scheduler */
CAN_Init(&hcan);
EH_init(&hcan, MY_NODE_ID, &schedulerList);
```

### Reporting
```c
// Report and continue (overwrites any previous error)
EH_report(0x0123, ERROR_SEVERITY_WARNING);

// Report with data snapshot
EH_reportEx(0x0123, ERROR_SEVERITY_ERROR, dataPtr, 5);

// Report and halt
EH_stop(0xDEAD, ERROR_SEVERITY_ERROR);
```

### Clearing
```c
// Clear specific error (returns to Heartbeat if code matches)
EH_clear(0x0123);
```
