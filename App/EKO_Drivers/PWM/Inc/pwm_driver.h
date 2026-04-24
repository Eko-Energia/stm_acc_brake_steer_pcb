/**
  * @file pwm_driver.h
  * @brief PWM signal driver for PERLA
  * @author AGH EKO-ENERGIA
  * @author Andrzej Gondek
  */

#ifndef PWM_SIGNAL_H
#define PWM_SIGNAL_H

#include "main.h"
#include <math.h>
#include <stdbool.h>

/**
 * PWM signal
 */
struct PWM_signal {
    uint32_t frequency;
    volatile float PWM_width;
    volatile bool readFlag;
    volatile uint32_t icVal;
    bool ch1;
    TIM_IC_InitTypeDef sConfigIC;
};
/**
 * PWM signal setup
 */
void PWM_initialize(struct PWM_signal* signal, int frequency, bool isChannel1,TIM_HandleTypeDef *htim);
/**
 * Computes PWM parameters, to be used within HAL_TIM_IC_CaptureCallback
 */
void PWM_update(TIM_HandleTypeDef *htim, struct PWM_signal *PWM);

#endif /* PWM_SIGNAL_H */
