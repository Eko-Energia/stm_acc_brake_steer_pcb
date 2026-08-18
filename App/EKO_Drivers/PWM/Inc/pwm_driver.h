/**
  * @file pwm_driver.h
  * @brief PWM signal driver for PERLA
  * @author AGH EKO-ENERGIA
  * @author Andrzej Gondek
  */


#ifndef PWM_SIGNAL_H
#define PWM_SIGNAL_H

#define PWM_monitorPeriodCount 4

#include "main.h"
#include <math.h>
#include <stdbool.h>

/**
 * PWM IC signal
 */
struct PWM_IC_signal {
    uint32_t frequency;
    volatile float duty;
    volatile uint32_t icVal;
    volatile uint32_t icPulseVal; // Add this to store the raw pulse width
    volatile bool dataReady;      // Add this to signal new data is available
    bool ch1;
    TIM_IC_InitTypeDef sConfigIC;
    uint32_t clock;
};
/*
 * PWM out signal
 */
struct PWM_Out_signal{
	TIM_HandleTypeDef* htim;
	int frequency;
	uint32_t Channel;
	float duty;

};
/*
 *
 */
void PWM_IC_Monitor(struct PWM_IC_signal* signal,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
/**
 * Set PWM signal up for Input Capture
 */
void PWM_IC_Init(struct PWM_IC_signal* signal, TIM_HandleTypeDef *htim, int frequency, bool isChannel1);
/*
 * Set PWM signal up for being sent
 */
void PWM_Out_Init(struct PWM_Out_signal *PWM, TIM_HandleTypeDef *htim, uint32_t Channel, float duty,int frequency);
/**
 * Computes PWM parameters, to be used within HAL_TIM_IC_CaptureCallback
 */
void PWM_IC_update(struct PWM_IC_signal *PWM, TIM_HandleTypeDef *htim);
/**
 * Set PWM signal duty
 */
void PWM_Out_setDuty(struct PWM_Out_signal *PWM, float duty);

#endif /* PWM_SIGNAL_H */
