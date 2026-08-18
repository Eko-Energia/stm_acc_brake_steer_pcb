# PWM Driver – STM32 (Input Capture + PWM Output)

## Overview

This module provides initialization and update routines for:

* Measuring **PWM duty cycle** using **STM32 Timer Input Capture in PWM Input mode**
* Generating a **PWM output signal**
* Monitoring PWM input for **signal timeout / constant level detection**

The driver supports PWM input configured on:

* **TIM Channel 1** → Period on CH1, Duty on CH2
* **TIM Channel 2** → Period on CH2, Duty on CH1

---

## Important Configuration Notes

To achieve correct and stable measurements:

* The **timer counter frequency MUST be lower than the measured PWM signal frequency**.
* Whenever possible, use the **maximum timer ARR (auto‑reload) value** to improve measurement resolution.
* The calculated duty cycle will be **slightly lower than the real duty cycle** due to the finite rise/fall time of the signal. Slower signal transitions reduce measurement accuracy.

---

## How PWM Input Measurement Works

The timer must be configured in **PWM Input Mode**. In this mode:

| Configuration | Period Measurement | High Time Measurement |
| ------------- | ------------------ | --------------------- |
| Input on CH1  | CH1                | CH2                   |
| Input on CH2  | CH2                | CH1                   |

Duty cycle is calculated as:

```
duty = (high_time / period) * 100
```

The duty value is stored as a **float percentage (0–100)**.

---

## PWM Input Signal Monitoring (Signal Loss Detection)

The driver includes a monitoring function that detects when the PWM signal stops changing (no edges detected).

If a timeout occurs:

* If pin state is HIGH → duty = **100%**
* If pin state is LOW → duty = **0%**

Timeout is calculated as:

```
TIMEOUT_MS = (1000 / frequency) * PWM_monitorPeriodCount
```

Minimum timeout is **1 ms**.

This mechanism allows detection of:

* Disconnected signal
* Stuck HIGH signal
* Stuck LOW signal

---

## PWM Output Generation

PWM output duty cycle is set using the timer compare register:

```
compare = ARR * (duty / 100)
```

Where ARR is the timer auto‑reload register value.

Changing duty while PWM is running or calling PMW_Out_Init on an already running signal is safe, but **may cause a short glitch** in the output signal.

---

## Data Structures

### PWM_IC_signal

Structure used for PWM input measurement.

| Field     | Description                           |
| --------- | ------------------------------------- |
| duty      | Measured duty cycle (%)               |
| frequency | Expected PWM frequency                |
| icVal     | Captured timer period value           |
| ch1       | true → PWM input on Channel 1         |
| clock     | Last capture timestamp (HAL_GetTick)  |
| sConfigIC | Input capture configuration structure |

### PWM_Out_signal

Structure used for PWM output generation.

| Field     | Description           |
| --------- | --------------------- |
| duty      | Output duty cycle (%) |
| frequency | PWM frequency         |
| Channel   | Timer channel         |
| htim      | Timer handle          |

---

## Functions

### PWM Input

| Function       | Description                                                   |
| -------------- | ------------------------------------------------------------- |
| PWM_IC_Init    | Initialize PWM input structure and configure IC channel       |
| PWM_IC_update  | Calculate duty cycle from captured values (call in interrupt) |
| PWM_IC_Monitor | Detect signal timeout and force 0% or 100%                    |

### PWM Output

| Function        | Description                        |
| --------------- | ---------------------------------- |
| PWM_Out_Init    | Start PWM and initialize structure |
| PWM_Out_setDuty | Set PWM duty cycle                 |

---

## Usage Examples

### PWM Output Example

```c
PWM_Out_signal pwmOut;

PWM_Out_Init(&pwmOut, &htim3, TIM_CHANNEL_1, 50.0f, 1000);

// Change duty later
PWM_Out_setDuty(&pwmOut, 75.0f);
```

---

### PWM Input Example

```c
PWM_IC_signal pwmIn;

PWM_IC_Init(&pwmIn, &htim2, 1000, true);

```

Interrupt callback:

```c
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        PWM_IC_update(&pwmIn, htim);
    }
}
```

Main loop monitoring:

```c
PWM_IC_Monitor(&pwmIn, GPIOA, GPIO_PIN_0);
```

