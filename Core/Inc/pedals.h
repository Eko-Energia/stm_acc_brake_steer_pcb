#ifndef pedals_h
#define pedals_h

#include <stdint.h>
#include "adc.h"

extern volatile int isADC1finished;
extern volatile int isADC2finished;

//checks if values from both accel. pedal's sensors are properly read
int accelPedalCheck(uint16_t accel1val, uint16_t accel2val, uint8_t acceptAccelError);

//checks if values from both brake pistons' sensors are properly read
int brakePistonsCheck(uint16_t brakePiston1, uint16_t brakePiston2, uint8_t acceptBrakeError);

//returns scaled value of brake pedal (Hall sensor)
uint8_t brakeHallValue(uint16_t brakeHall);

//returns scaled value of accel. pedal
uint8_t accelPedalValue(uint16_t accel1val, uint16_t accel2val);

//returns scaled value of brake pistons
uint8_t brakePistonsValue(uint16_t brakePiston1, uint16_t brakePiston2);

//returns scaled value of steering wheel
uint8_t steerValue(uint16_t steerVal);

//sets flag when ADC read the value
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

#endif
