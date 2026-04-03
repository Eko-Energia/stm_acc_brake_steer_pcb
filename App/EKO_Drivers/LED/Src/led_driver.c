#include "led_driver.h"

void LED_Handle(struct LED *led)
{
    const uint32_t currentTick = HAL_GetTick();
    switch(led->state)
    {
        case LED_OFF:
        case LED_ON:
        break;
        case LED_FAST_BLINK:
            if(led->lastTick + LED_FAST_BLINK < currentTick)
            {
                HAL_GPIO_TogglePin(led->GPIO_Port, led->GPIO_Pin);
                led->lastTick = currentTick;
            }
            break;
        case LED_BLINK:
            if(led->lastTick + LED_BLINK < currentTick)
            {
                HAL_GPIO_TogglePin(led->GPIO_Port, led->GPIO_Pin);
                led->lastTick = currentTick;
            }
    }
}

void LED_ChangeState(struct LED *led, LED_STATE_e state)
{
    led->state = state;
    switch(led->state)
    {
        case LED_OFF:
            HAL_GPIO_WritePin(led->GPIO_Port, led->GPIO_Pin, GPIO_PIN_RESET);
            break;
        case LED_FAST_BLINK:
        case LED_BLINK:
        case LED_ON:
            HAL_GPIO_WritePin(led->GPIO_Port, led->GPIO_Pin, GPIO_PIN_SET);
            led->lastTick = HAL_GetTick();
        break;
    }
}
