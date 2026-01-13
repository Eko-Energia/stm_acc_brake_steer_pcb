#ifndef pedals_const_val
#define pedals_const_val

#define SENSOR_ADC_MAX_VALUE 1023 //Max value read from all ADC channels
#define ACCEPT_BRAKE_ERROR 14 //max value of error between both brake pistons (14 ->1.4%)
#define ACCEPT_ACCEL_ERROR 14 //max value of error between both accel. sensors (14 ->1.4%)

#define ACCEL_MIN_VAL 126   // Value when accel. pedal released (0%)
#define ACCEL_MAX_VAL 896   // Value when accel. pedal pushed to the end (100%)
#define ACCEL_RANGE (ACCEL_MAX_VAL - ACCEL_MIN_VAL) // Work range (770)



#endif
