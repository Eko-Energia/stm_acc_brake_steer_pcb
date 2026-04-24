/**
 * @file pwm_driver.c
 * @brief PWM input signal driver for PERLA project
 *
 * This module provides initialization and update routines
 * for measuring PWM duty cycle using STM32 timer input capture
 * in PWM Input mode.
 *
 * The driver supports PWM input configured either on:
 *  - TIM Channel 1 (period on CH1, duty on CH2)
 *  - TIM Channel 2 (period on CH2, duty on CH1)
 *
 * @author AGH EKO-ENERGIA
 * @author Andrzej Gondek
 */

#include "pwm_driver.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/**
 * @brief Initialize PWM input signal structure and timer channel
 *
 * This function initializes the PWM_signal structure fields and
 * configures the selected timer input capture channel for PWM
 * input measurement.
 *
 * The timer must already be configured in PWM Input mode
 *
 * @param signal Pointer to PWM_signal structure to initialize
 * @param frequency Expected PWM signal frequency
 * @param isChannel1
 *        - true  : PWM input configured on TIM Channel 1
 *        - false : PWM input configured on TIM Channel 2
 * @param htim Pointer to the timer handle used for input capture
 */
void PWM_initialize(struct PWM_signal* signal,
                    int frequency,
                    bool isChannel1,
                    TIM_HandleTypeDef *htim)
{
    signal->PWM_width = 0.0f;
    signal->readFlag = false;
    signal->frequency = frequency;
    signal->icVal = 0;
    signal->ch1 = isChannel1;
    /* Configure input capture parameters */
    signal->sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    signal->sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    signal->sConfigIC.ICPrescaler = TIM_ICPSC_DIV8;
    signal->sConfigIC.ICFilter    = 0;
    if (isChannel1)
    {
        HAL_TIM_IC_ConfigChannel(htim, &signal->sConfigIC, TIM_CHANNEL_1);
        HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
        HAL_TIM_IC_Start(htim, TIM_CHANNEL_2);
    }
    else
    {
        HAL_TIM_IC_ConfigChannel(htim, &signal->sConfigIC, TIM_CHANNEL_2);
        HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
        HAL_TIM_IC_Start(htim, TIM_CHANNEL_1);
    }
}

/**
 * @brief Update PWM duty cycle measurement
 *
 * This function reads captured timer values and computes
 * the PWM duty cycle in percent.
 *
 * Behavior depends on the PWM input configuration:
 * - If PWM input is on Channel 1:
 *     - CH1 captures period
 *     - CH2 captures high time
 * - If PWM input is on Channel 2:
 *     - CH2 captures period
 *     - CH1 captures high time
 *
 * The result is stored in PWM_signal::PWM_width as a percentage
 * value in the range 0–100.
 *
 * @param htim Pointer to the timer handle used for input capture
 * @param PWM Pointer to initialized PWM_signal structure
 */
void PWM_update(TIM_HandleTypeDef *htim, struct PWM_signal *PWM)
{
    if (PWM->ch1)
    {
        PWM->icVal = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (PWM->icVal != 0)
        {
            PWM->PWM_width =
                ((float)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) * 100.0f)
                / (float)PWM->icVal;
        }
    }
    else
    {
        PWM->icVal = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

        if (PWM->icVal != 0)
        {
            PWM->PWM_width =
                ((float)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) * 100.0f)
                / (float)PWM->icVal;
        }
    }
}
