# LED Driver

This driver provides simple LED control for STM32 HAL-based boards, supporting off, on, and blinking modes. This module is to be implemented on every board in PERLA solar car to ensure standard behavior.

## Basic Usage

To indicate that everything is fine on the STM32 board, you can set the LED to blink at a regular interval (1 second on/off cycle).

### 1. Include the Header

```c
#include "led_driver.h"
```

### 2. Define and Initialize the LED

```c
struct LED statusLed;

// Assuming GPIO is already configured in your main.c or initialization code
statusLed.GPIO_Port = GPIOA;  // Replace with your GPIO port
statusLed.GPIO_Pin = GPIO_PIN_5;  // Replace with your GPIO pin
LED_ChangeState(&statusLed, LED_BLINK);  // Set LED to blink when everything is fine
```

### 3. Handle LED in Main Loop

Call `LED_Handle()` periodically in your main loop or timer interrupt:

```c
int main(void) {
    // ... HAL initialization code ...

    // Initialize LED as above

    while (1) {
        LED_Handle(&statusLed);
        // ... your application code ...
    }
}
```

### LED States

- `LED_OFF`: LED is turned off
- `LED_FAST_BLINK`: LED toggles every 100ms (fast blink)
- `LED_BLINK`: LED toggles every 1000ms (regular blink) - use for "everything is fine"
- `LED_ON`: LED is permanently on

### Notes

- Ensure GPIO pins are configured as output in your STM32CubeMX or initialization code
- `LED_Handle()` should be called frequently enough to maintain blink timing
- The driver uses `HAL_GetTick()` for timing, so ensure HAL is initialized
