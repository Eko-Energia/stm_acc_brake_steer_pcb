//#include "pedals.h"
//#include "pedals_const_val.h"
//#include "adc.h"
//#include "can.h"
//#include <stdlib.h>
//volatile int isADC1finished = 0;
//volatile int isADC2finished = 0;
//
//// BEGIN ACCEL. pedal section
//int accelPedalCheck(uint16_t accel2val, uint16_t accel1val, uint8_t acceptAccelError)
//{
//	int16_t pedalCheck = (accel2val * 1000 / SENSOR_ADC_MAX_VALUE / 2 ) - (accel1val * 1000 / SENSOR_ADC_MAX_VALUE);
//	//int16_t pedalCheck = (accel2val * 1000 / SENSOR_ADC_MAX_VALUE ) - (accel1val * 1000 / SENSOR_ADC_MAX_VALUE); //breadboard test
//	if (abs(pedalCheck) >= acceptAccelError) //value based on accel. pedal documentation
//	{
//		return 1; //Difference between ADC1 and ADC2 is bigger than accepted error -> error flag
//	}
//	else
//	{
//		return 0; //all's good
//	}
//}
//
//uint8_t accelPedalValue(uint16_t accel1val, uint16_t accel2val)
//{
//	if (!accelPedalCheck(accel1val, accel2val, ACCEPT_ACCEL_ERROR))
//	{
//		/*
//		uint16_t current_accel_val = accel1val;
//		uint8_t accel_percentage_scaled;
//		if (current_accel_val >= SENSOR_ADC_MAX_VALUE)
//		{
//			accel_percentage_scaled = 100;
//			return accel_percentage_scaled;
//		}
//		else
//		{
//			uint32_t temp_calc_accel = ((uint32_t)current_accel_val * 100) + (SENSOR_ADC_MAX_VALUE / 2);
//			accel_percentage_scaled = (uint8_t)(temp_calc_accel / SENSOR_ADC_MAX_VALUE);
//			return accel_percentage_scaled;
//		}
//		*/
//		uint16_t calibrated_val = accel1val;
//
//		    // obsługa martwych stref
//		    if (calibrated_val < ACCEL_MIN_VAL) {
//		        calibrated_val = ACCEL_MIN_VAL; // Poniżej minimum to 0%
//		    }
//		    else if (calibrated_val > ACCEL_MAX_VAL) {
//		        calibrated_val = ACCEL_MAX_VAL; // Powyżej maksimum to 100%
//		    }
//
//		    // skalowanie do zakresu 0-100%
//		    // Wzór: (Wartość - Min) * 100 / (Max - Min)
//
//		    // Obliczeniey przesunięcia
//		    uint32_t val_normalized = calibrated_val - ACCEL_MIN_VAL;
//
//		    // Obliczenie procentu z zaokrąglaniem (+ ACCEL_RANGE/2)
//
//		    uint8_t accel_percentage_scaled = (uint8_t)(val_normalized * 100/ ACCEL_RANGE);
//		    return accel_percentage_scaled;
//	}
//	else
//	{
//		return 0x0;
//	}
//
//}
////END of ACCEL. pedal section
//
////BEGIN of engine section
//
//uint16_t engineSteer(uint16_t accel1val, uint16_t accel2val)
//{
//	uint16_t accelEngine = (float)accelPedalValue(accel1val, accel2val) / 100.0f * 32767;
//	return accelEngine;
//
//}
////END of engine section
//
////BEGIN of brake pedal (pistons) section
//
////	acceptBrakeError -> accepted error in percentage value
////	How to input wanted percentage value, ex. below:
////	0.5% -> 5
////	50.6% -> 506
////	UPDATE: variable moved to pedal_const.h
//int brakePistonsCheck(uint16_t brakePiston1, uint16_t brakePiston2, uint8_t acceptBrakeError)
//{
//	int8_t pistonsCheck = (brakePiston1 * 1000) / SENSOR_ADC_MAX_VALUE  - (brakePiston2 * 1000) / SENSOR_ADC_MAX_VALUE;
//	if (abs(pistonsCheck) > acceptBrakeError)
//	{
//		return 1;
//	}
//	else
//	{
//		return 0;
//	}
//}
//
////	to do: include piston ratio
//uint8_t brakePistonsValue(uint16_t brakePiston1, uint16_t brakePiston2)
//{
//	uint8_t brake_piston_percentage_scaled;
//	if (!brakePistonsCheck(brakePiston1, brakePiston2, ACCEPT_BRAKE_ERROR))
//	{
//		uint16_t current_brake_piston_val = brakePiston1;
//		if (current_brake_piston_val >= SENSOR_ADC_MAX_VALUE) {
//			brake_piston_percentage_scaled = 100;
//			return brake_piston_percentage_scaled;
//		}
//		else {
//			uint32_t temp_calc_brake_piston = ((uint32_t)current_brake_piston_val * 100) + (SENSOR_ADC_MAX_VALUE / 2);
//			brake_piston_percentage_scaled = (uint8_t)(temp_calc_brake_piston / SENSOR_ADC_MAX_VALUE);
//			return brake_piston_percentage_scaled;
//		}
//	}
//	else
//	{
//		return 0x0;
//	}
//}
//
////END of BRAKE pedal (pistons) section
//
////BEGIN of BRAKE pedal (Hall) section
//uint8_t brakeHallValue(uint16_t brakeHal)
//{
//	uint8_t brake_hall_scaled;
//	if (brakeHal >= SENSOR_ADC_MAX_VALUE)
//	{
//		brake_hall_scaled = 100;
//		return brake_hall_scaled;
//	}
//	else
//	{
//		uint32_t temp_hall_brake = ((uint32_t)brakeHal * 100) + (SENSOR_ADC_MAX_VALUE / 2);
//		brake_hall_scaled = (uint8_t)(temp_hall_brake / SENSOR_ADC_MAX_VALUE);
//		return brake_hall_scaled;
//	}
//}
////END of BRAKE pedal (Hall) section
//
////BEGIN of STEERING wheel section
//
//uint8_t steerValue(uint16_t steerWheel)
//{
//	uint8_t steer_percentage_scaled;
//	uint16_t current_steer_val = steerWheel;
//	if (current_steer_val >= SENSOR_ADC_MAX_VALUE) {
//	  steer_percentage_scaled = 100;
//	  return steer_percentage_scaled;
//	  }
//	else {
//	  uint32_t temp_calc_steer = ((uint32_t)current_steer_val * 100) + (SENSOR_ADC_MAX_VALUE / 2);
//	  steer_percentage_scaled = (uint8_t)(temp_calc_steer / SENSOR_ADC_MAX_VALUE);
//	  return steer_percentage_scaled;
//	}
//}
//
////END of STEERING wheel section
//
//
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//  if (hadc->Instance == ADC1)
//  {
//    isADC1finished = 1;
//  }
//  if (hadc->Instance == ADC2)
//  {
//    isADC2finished = 1;
//  }
//}
//
////Extracting data from a CAN frame
//bool CAN_ExtractSignal(const uint8_t* frameData, CAN_Signal_Config_t config, float* outValue) {
//    if (frameData == NULL || outValue == NULL) {
//        return false;
//    }
//
//    uint64_t raw64 = 0;		//storage for our data to perform operations on
//    memcpy(&raw64, frameData, 8);
//
//    uint64_t mask = (1ULL << config.length) - 1;
//
//    uint64_t rawValue = (raw64 >> config.startBit) & mask;
//
//    //case for signed numbers
//    if (config.isSigned) {
//        if (rawValue & (1ULL << (config.length - 1))) {
//            rawValue |= ~mask;
//        }
//
//        int64_t signedRaw = (int64_t)rawValue;
//        *outValue = (float)signedRaw * config.factor + config.offset;
//
//    } else {
//        *outValue = (float)rawValue * config.factor + config.offset;
//    }
//
//    return true;
//}
//
//void CAN_ProcessFrame(CAN_RxHeaderTypeDef *pHeader, uint8_t* data) {
//
//    // Checking the charger (Extended ID: 0x1806E5F4)
//    if (pHeader->IDE == CAN_ID_EXT && pHeader->ExtId == 0x1806E5F4) {
//        float val;
//        // processing signal CONTROL
//        if (CAN_ExtractSignal(data, SIG_CHARGER_CONTROL, &val)) {
//            Vehicle.Charger.RawStatus = (uint8_t)val;
//            Vehicle.Charger.LastMsgTick = HAL_GetTick();
//            Vehicle.Charger.IsConnected = true;
//        }
//    }
//
//    // Checking JETSON (Standard ID: 0x512)
//    if (pHeader->IDE == CAN_ID_STD && pHeader->StdId == 0x512) {
//        // coping data and marking the time
//        memcpy((void*)Vehicle.Jetson.RawData, data, 8);
//        Vehicle.Jetson.LastMsgTick = HAL_GetTick();
//        Vehicle.Jetson.IsConnected = true;
//    }
//}
//
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//    CAN_RxHeaderTypeDef rxHeader;
//    uint8_t rxData[8];
//
//    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
//        CAN_ProcessFrame(&rxHeader, rxData);
//    }
//}
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM2)
//    {
//        uint32_t now = HAL_GetTick();
//        uint32_t timeout = 1000;
//
//        if (Vehicle.Charger.RawStatus == 0)
//		{
//        	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
//		}
//        else if (Vehicle.Charger.RawStatus == 1)
//        {
//			 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
//        }
//
//        // --- CHECKING CONNECTIONS (WATCHDOG) ---
//
//        // CHARGER
//        if (now - Vehicle.Charger.LastMsgTick > timeout) {
//            Vehicle.Charger.IsConnected = false;
//            Vehicle.Charger.RawStatus = 0xFF;
//            if (getEngineFlag() == 1)
//			{
//				stopEngine();
//				setEngineFlag(0x2); //only power RESET can get you out of here
//			}
//
//
//            const int interval = 100;
//            int lastTick = 0;
//            int currentTick = HAL_GetTick();
//            if (currentTick - lastTick >= interval)
//            {
//            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//            lastTick = HAL_GetTick();
//            }
//        }
//
//        // JETSON
//        if (now - Vehicle.Jetson.LastMsgTick > timeout) {
//            Vehicle.Jetson.IsConnected = false;
//            //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
//            memset((void*)Vehicle.Jetson.RawData, 0, 8);
//        }
//
//    }
//}
//
//static uint8_t engineState = 0;
//uint8_t getEngineFlag()
//{
//	return engineState;
//}
//
//void setEngineFlag(uint8_t engineFlag)
//{
//	engineState = engineFlag;
//}
////maybe send below frames a few times to make sure it'll get to the desired target
//void startEngine()
//{
//	TxDataNMT[0] = 0x01; //sets engines in the operating mode
//	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderNMT, TxDataNMT, &TxMailBox) != HAL_OK)
//	{
//	  Error_Handler();
//	}
//}
//
//void stopEngine()
//{
//	TxDataNMT[0]= 0x02; //puts engines in the stop mode
//	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderNMT, TxDataNMT, &TxMailBox) != HAL_OK)
//	{
//	  Error_Handler();
//	}
//}
//
//
//
//int machineState()
//{
//	// --------- JETSON WORKING ----------
//	if (Vehicle.Jetson.IsConnected == true)
//	{
//	//Charging state
//		if (Vehicle.Charger.RawStatus == 0)
//		{
//			 return 100;
//		}
//
//	//N gear in
//
//		//return 101
//
//	//Active state
//		else
//		{
//			return 110;
//		}
//
//	}
//
//	// --------- JETSON DOWN ------------
//
//
//	else
//	{
//		//Charging state
//		if (Vehicle.Charger.RawStatus == 0)
//		{
//
//			return 000;
//		}
//
//		//N gear in
//
//			//return 001
//
//		//Active state
//		else
//		{
//			return 010;
//		}
//	}
//
//}
//
//void stateActions()
//{
//	int currentState = machineState();
//
//	switch(currentState){
//
//	// ------- JETSON WORKING -------
//
//	//charger connected
//	case 100:
//
//		break;
//	//N gear in
//	case 101:
//
//		break;
//	//active state
//	case 110:
//		TxData[0] = steerValue(ADC1_VAL[2]);;
//		TxData[1] = brakePistonsValue(ADC1_VAL[1], ADC2_VAL[1]);
//		TxData[2] = brakeHallValue(ADC2_VAL[2]);
//		TxData[3] = accelPedalValue(ADC1_VAL[0], ADC2_VAL[0]);
//
//		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox) != HAL_OK)
//		{
//		  Error_Handler();
//		}
//
//		break;
//	// -------- JETSON DOWN ---------
//		//sending engine frames directly when in active state
//
//	//Charger connected
//	case 000:
//		if (getEngineFlag() == 1)
//		{
//			stopEngine();
//			setEngineFlag(0x0);
//		}
//
//		break;
//	//N gear in
//	case 001:
//		if (getEngineFlag() == 1)
//		{
//			stopEngine();
//			setEngineFlag(0x0);
//		}
//		break;
//	//Active state
//	case 010:
//		//if statement that only gets executed once
//		if (getEngineFlag() == 0)
//		{
//			startEngine();
//			setEngineFlag(0x1);
//		}
//		tempTH = engineSteer(ADC1_VAL[0], ADC2_VAL[0]);
//
//		TxDataTH[0] = (uint8_t)tempTH & 0xFF; // lsb
//		TxDataTH[1] = (uint8_t)(tempTH >> 8) & 0xFF;   // msb
//
//		if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderTH, TxDataTH, &TxMailBox) != HAL_OK)
//		{
//		  Error_Handler();
//		}
//
//		break;
//	default:
//		//error handler?
//		break;
//	}
//}
