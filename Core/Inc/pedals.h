#ifndef pedals_h
#define pedals_h
#include <string.h>
#include <stdbool.h>
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

//test engine
uint16_t engineSteer(uint16_t accel1val, uint16_t accel2val);

//struct for extracting data from a CAN frame
typedef struct {
    uint8_t startBit;      // Start bit (0-63)
    uint8_t length;        // Signal length in bits (1-64)
    float factor;
    float offset;
    bool isSigned;         // signed or unsigned
} CAN_Signal_Config_t;

//"CONTROL" signal from frame 0x1806E5F4 (dec:403105268)
extern const CAN_Signal_Config_t SIG_CHARGER_CONTROL;

//function for CAN data extraction
bool CAN_ExtractSignal(const uint8_t* frameData, CAN_Signal_Config_t config, float* outValue);

//function assigning data from the charger and jetson to the structure
void CAN_ProcessFrame(CAN_RxHeaderTypeDef *pHeader, uint8_t* data);

//structure gathering info about the car
typedef struct {
    // --- CHARGER ---
    struct {
        bool     IsConnected;  // do we get any frames from the charger
        uint8_t  RawStatus;    // 0 = Start Charging, 1 = Stop Charging
        uint32_t LastMsgTick;  // when the last frame has been received
    } Charger;

    // --- JETSON ---
    struct {
        uint8_t  RawData[8];   // all will be updated when ready
        bool     IsConnected;
        uint32_t LastMsgTick;
    } Jetson;

} VehicleState_t;

extern volatile VehicleState_t Vehicle;

//CAN interrupt
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

//Timer interrupt for period connection checks of charger + jetson + PRND
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2);

//function deciding about current state of the machine
int machineState();

// actions based on the current state of the machine
void stateActions();

//informs whether engine are put in operating mode (WHEN JETSON DOWN)
uint8_t getEngineFlag();

//changes flag of engine operating mode (WHEN JETSON DOWN)
void setEngineFlag(uint8_t engineFlag);

//puts engines in the operating mode
void startEngine();

//puts engines in the stop mode
void stopEngine();


extern uint16_t ADC1_VAL[3];
extern uint16_t ADC2_VAL[3];

extern uint8_t TxData[8];
extern uint8_t TxDataTH[8];
extern uint8_t TxDataNMT[2];

extern uint16_t tempTH;

extern CAN_TxHeaderTypeDef TxHeader;
extern uint32_t TxMailBox;

extern CAN_TxHeaderTypeDef TxHeaderTH, TxHeaderNMT;


#endif
