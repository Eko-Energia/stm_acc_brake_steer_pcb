/**
  ******************************************************************************
  * @file    vehicle_fsm.c
  * @brief   Finite State Machine (FSM) implementation for vehicle control.
  * @author  [Michał Jurek]
  * @date    2025-03-21
  * @details This file implements the high-level logic of the vehicle. It determines
  * the current operating mode based on system connectivity (Jetson, PRND)
  * and executes the appropriate actions (CAN transmission, Engine Start/Stop).
  *
  * The logic is divided into two main branches:
  * 1. **Jetson Working:** The MCU acts as a pass-through, sending sensor data to the PC.
  * 2. **Jetson Down:** The MCU takes direct control of the inverters.
  ******************************************************************************
  */

#include "vehicle_fsm.h"

/** @brief Temporary variable for calculated torque before transmission. */
int16_t tempTH = 0;

/** @brief Torque commands of both rear wheels after the torque vectoring split. */
static int16_t tempTHL = 0;
static int16_t tempTHR = 0;

// The split works directly in inverter command units, so nothing is rescaled
// between the throttle curve and the CAN frame.
_Static_assert(THROTTLE_MAX_VAL == TV_CONFIG_COMMAND_MAX,
               "throttle curve and torque vectoring must share one command range");

// Applied at every call, so it needs no rebuild and no dirty flag.
static volatile uint8_t tvGainPercent = TV_CONFIG_GAIN_DEFAULT;


/**
 * @brief   Global instance of the Vehicle State.
 * @details Centralized data structure tracking the real-time operational status,
 * command data, and connection health (watchdogs) of critical vehicle
 * subsystems: Charger, Jetson (Autonomous logic), and PRND (Gear selector).
 * Declared as 'volatile' to guarantee memory-safe, unoptimized access
 * between the synchronous FSM main loop and asynchronous hardware ISRs
 * (e.g., CAN Rx interrupts, TIM2 timeout watchdogs).
 */
volatile VehicleState_t Vehicle = {
    .Charger = {
        .RawStatus = STOP_CHARGING,
        .IsConnected = false,
        .LastMsgTick = 0
    },
    .Jetson = {
        .RawData = {0},
        .IsConnected = false,
        .LastMsgTick = 0
    },
	.PRND = {
		.RawStatus = PARKING_GEAR,
		.IsConnected = false,
		.LastMsgTick = 0
    },
	.Pedals = {
		.Steer = 0,
		.BrakePistons = 0,
		.BrakeHall = 0,
		.Accel = 0
	}
};
/**
 * @brief  Determines the current state of the vehicle machine.
 * @details Checks system flags in a specific priority order:
 * 1. Is there an active error in the system?
 * 2. Jetson Connection (Is the JETSON alive?)
 * 3. Gear Status
 *
 * @return MachineState_e The current state of the system.
 */

MachineState_e machineState()
{
	// If there is any active error => neutral gear in
	if (heh.activeErrorCount != 0)
	{
		return JTSN_DOWN_NEUTRAL_GEAR_STATE;
	}

	// ========================================================================
	// BRANCH 1: JETSON IS CONNECTED
	// ========================================================================

	if (Vehicle.Jetson.IsConnected == true)
	{
	//N gear in
		if (Vehicle.PRND.RawStatus == NEUTRAL_GEAR)
		{
			return JTSN_WORKS_NEUTRAL_GEAR_STATE;
		}

	// D gear in
		else
		{
			return JTSN_WORKS_DRIVE_STATE;
		}
	}

	// ========================================================================
	// BRANCH 2: JETSON IS DOWN (This PCB is in charge)
	// ========================================================================

	else
	{
		switch(Vehicle.PRND.RawStatus)
		{
		case PARKING_GEAR: // P gear in
			return JTSN_DOWN_PARKING_STATE;

		case REVERSE_GEAR: // R gear in
			return JTSN_DOWN_REVERSE_STATE;

		case NEUTRAL_GEAR: // N gear in
			return JTSN_DOWN_NEUTRAL_GEAR_STATE;

		default:           // D gear in
			return JTSN_DOWN_DRIVE_STATE;
		}
	}

}


// ============================================================================
// TORQUE VECTORING
// ============================================================================

/**
 * @brief  Sets how much of the calculated torque split actually reaches the wheels.
 * @details Rejecting rather than clamping an out-of-range value lets the caller
 * report the bad frame, and leaves the previous gain in force. Safe to
 * call from any context, including the CAN RX interrupt.
 *
 * @param[in] gainPercent Blend between an open differential and full torque
 * vectoring, 0 - @ref TV_CONFIG_GAIN_MAX. 0 gives both wheels the same
 * command, 100 gives the whole calculated split.
 *
 * @return bool
 * @retval true  Value accepted.
 * @retval false Rejected (above the maximum). Active gain left unchanged.
 */
bool TorqueVectoring_SetGain(uint8_t gainPercent)
{
	if (gainPercent > TV_CONFIG_GAIN_MAX)
	{
		return false;
	}

	tvGainPercent = gainPercent;
	return true;
}

/**
 * @brief  Returns the active torque vectoring gain.
 * @return uint8_t Gain in percent, 0 - @ref TV_CONFIG_GAIN_MAX.
 */
uint8_t TorqueVectoring_GetGain(void)
{
	return tvGainPercent;
}

/**
 * @brief  Converts one wheel speed reading to the units used by the algorithm.
 * @details Direction is carried by the gear, not by the speed, so a reversing
 * wheel is reported by its magnitude.
 *
 * @param[in] speedMps Wheel speed [m/s].
 * @return uint32_t Wheel speed [mm/s].
 */
static uint32_t wheelSpeedMmps(float speedMps)
{
	if (speedMps < 0.0f)
	{
		speedMps = -speedMps;
	}

	return (uint32_t)(speedMps * 1000.0f);
}

/**
 * @brief  Estimates the speed of the centre of mass from the wheel speeds.
 * @details The front pair only refines the estimate while driving straight;
 * without it the rear axle alone still gives a bicycle-model speed, so a
 * missing front sensor degrades the result instead of stopping the split.
 *
 * @param[in] vehicle Vehicle geometry passed to the algorithm.
 * @return uint32_t Speed of the centre of mass [mm/s].
 */
static uint32_t vehicleSpeedMmps(const VehicleParameters *vehicle)
{
	const uint32_t rearLeftMmps = wheelSpeedMmps(Vehicle.WheelSpeed.SpeedRL);
	const uint32_t rearRightMmps = wheelSpeedMmps(Vehicle.WheelSpeed.SpeedRR);

	if (Vehicle.WheelSpeed.IsConnectedFL && Vehicle.WheelSpeed.IsConnectedFR)
	{
		return tv_com_velocity_from_wheel_speeds_mmps(vehicle,
				wheelSpeedMmps(Vehicle.WheelSpeed.SpeedFL),
				wheelSpeedMmps(Vehicle.WheelSpeed.SpeedFR),
				rearLeftMmps,
				rearRightMmps);
	}

	return tv_com_velocity_from_rear_wheels_mmps(vehicle, rearLeftMmps, rearRightMmps);
}

/**
 * @brief  Splits one throttle command between the two rear wheels.
 * @details Input and output are inverter command units, the same range the
 * throttle curve produces, so this only changes how the demand is shared
 * between the wheels - never the range of the frames themselves.
 *
 * @param[in]  pedalCommand Throttle command from @ref ThrottleCurve_Apply.
 * @param[out] leftCommand  Command for the left rear wheel.
 * @param[out] rightCommand Command for the right rear wheel.
 *
 * @note Main loop context only, same as @ref ThrottleCurve_Apply.
 */
static void torqueVectoringSplit(int16_t pedalCommand, int16_t *leftCommand, int16_t *rightCommand)
{
	// Geometry never changes at runtime, so it is built once and reused.
	static VehicleParameters tvVehicle;
	static bool tvVehicleReady = false;

	if (!tvVehicleReady)
	{
		tvVehicle = tv_default_vehicle();
		tvVehicleReady = true;
	}

	// Equal split is what the car does today and what every unusable reading
	// falls back to, so it is written first and only overwritten on success.
	*leftCommand = pedalCommand;
	*rightCommand = pedalCommand;

	const WheelCommands split = tv_calculate_rear_commands_from_rack(
			&tvVehicle,
			Vehicle.Steering.IsConnected,
			Vehicle.Steering.RackMm,
			vehicleSpeedMmps(&tvVehicle),
			pedalCommand,
			tvGainPercent);

	// Only a complete calculation may change the commands. Every other status,
	// TV_LATERAL_GRIP_EXCEEDED included, leaves the equal split above, so this
	// stays a redistribution of the driver's demand and never takes torque
	// away - exactly what the car does today. Add TV_LATERAL_GRIP_EXCEEDED here
	// to let the algorithm cut the torque at the lateral grip limit instead.
	if (TV_OK == split.status)
	{
		*leftCommand = (int16_t)split.rear_left;
		*rightCommand = (int16_t)split.rear_right;
	}
}

// ============================================================================
// CAN DRIVER CALLBACKS (Data acquisition for periodic CAN frames)
// ============================================================================

void Jetson_GetData(uint8_t *data, void *context) {
    (void)context;
    // Pack values already computed in stateActions(); do not remap ADC here.
    data[steerValIndex] 		= Vehicle.Pedals.Steer;
    data[brakePistonsValIndex] 	= Vehicle.Pedals.BrakePistons;
    data[brakeHallValIndex] 	= Vehicle.Pedals.BrakeHall;
    data[accelPedalValIndex] 	= Vehicle.Pedals.Accel;
}

void EngineThrottle_GetData(uint8_t *data, void *context ) {

	// Unwrapping 'context' which is actually an enum type
	// describing which engine is currently used by this
	// function.
	// To see how it was done go to StateMachine()
	// where CAN driver structure is being set up

	uint8_t which_engine = (uint32_t)context;

	// localTH used for not tampering with the global one
	int16_t localTH = (LEFT_ENGINE == which_engine) ? tempTHL : tempTHR;

	if(Vehicle.WheelSpeed.SpeedRL != 0 || Vehicle.WheelSpeed.SpeedRR != 0)
	{
		// Checked on the mapped command, not on the pedal percent: with a
		// progressive curve (z > 1) a small non-zero pedal still maps to 0.
		// Only an incidental zero is raised. A limit of zero is a deliberate
		// request for no torque and must reach the inverters unchanged.
		if(localTH == 0 && ThrottleCurve_GetLimit() != 0)
		{
			localTH = 1;
		}
	}

	// DO NOT REDUCE THESE IF INSTRUCTIONS!
	// Value for left engine needs to be POSITIVE when PRND is set to REVERSE gear
	if (LEFT_ENGINE == which_engine)
	{
		localTH = -localTH;
	}
	// REVERSE GEAR IN
    if(REVERSE_GEAR == Vehicle.PRND.RawStatus)
    {
    	localTH = -localTH;
    }

    data[0] = (uint8_t)localTH & TH_mask;           // LSB
    data[1] = (uint8_t)(localTH >> TH_bitpos) & TH_mask;    // MSB

}

// ============================================================================
// FINITE STATE MACHINE (FSM)
// ============================================================================


void stateActions()
{
    // Store the last state. -1 forces a reaction on the first boot of the MCU.
    static MachineState_e lastState = (MachineState_e)-1;
    MachineState_e currentState = machineState();

    // Map ADC once per cycle. Jetson_GetData() only packs this snapshot.
    // Do not run plausibility checks here — EH_report() would force Neutral
    // and CAN_RemoveScheduledMsg() would delete 0x226/0x227/0x41 before they fire.
    Vehicle.Pedals.Steer        = steerValue(&ADC1_VAL[2]);
    Vehicle.Pedals.BrakePistons = brakePistonsValue(&ADC1_VAL[1], &ADC2_VAL[1]);
    Vehicle.Pedals.BrakeHall    = brakeHallValue(&ADC2_VAL[2]);

    // Updating global variable tempTH with the data from accel. pedal.
	// It is used by EngineThrottle_GetData() but it cannon be
	// inside of it as there could be a difference in readings
	// as this function is called twice to send two separate
	// CAN frames to the inverters.
	// Only updated when in driving mode.

	if (currentState == JTSN_DOWN_DRIVE_STATE ||
		currentState == JTSN_DOWN_REVERSE_STATE ||
		currentState == JTSN_WORKS_DRIVE_STATE)
	{
		Vehicle.Pedals.Accel = accelPedalValue(&ADC1_VAL[0], &ADC2_VAL[0]);
		tempTH = ThrottleCurve_Apply(Vehicle.Pedals.Accel);

		// Split once per cycle, for the same reason tempTH is calculated here:
		// both inverter frames must be built from one reading.
		torqueVectoringSplit(tempTH, &tempTHL, &tempTHR);
	}

    // Execute actions ONLY when the gear or connection status changes
    if (currentState != lastState)
    {
        // 1. CLEANUP: Remove old periodic driving frames from the scheduler
        CAN_RemoveScheduledMsg(TxHeader.StdId, &canScheduler);   // Remove ECU frame
        CAN_RemoveScheduledMsg(TxHeaderTHR.StdId, &canScheduler); // Remove right Engine Torque frame
        CAN_RemoveScheduledMsg(TxHeaderTHL.StdId, &canScheduler); // Remove left Engine Torque frame

        struct CAN_scheduledMsg msgJetson = {
            .header = TxHeader,        // Configured in can_bus.c (0x41)
            .periodMs = 100,
            .getData = Jetson_GetData,
            .context = NULL
        };
        CAN_AddScheduledMsg(&msgJetson, &canScheduler);
        
        bool isBad = 0;

        // 2. NEW TASKS FOR THE CURRENT STATE
        switch(currentState) {

            // // --- JETSON WORKING ---
            // case JTSN_WORKS_DRIVE_STATE: {
            //     struct CAN_scheduledMsg msgJetson = {
            //         .header = TxHeader,        // Configured in can_bus.c (0x41)
			// 		.periodMs = 100,
			// 		.getData = Jetson_GetData, // Data packing function
            //         .context = NULL
            //     };
            //     CAN_AddScheduledMsg(&msgJetson, &canScheduler);
            //     break;
            // }

            // --- JETSON DOWN (MANUAL CONTROL) ---
            case JTSN_DOWN_REVERSE_STATE:
            	// Fall through case to not repeat code
            case JTSN_DOWN_DRIVE_STATE: {
                // NMT (Start) frame is a one-shot signal - bypassing the periodic driver
                if (getEngineFlag() != ENGINE_RUN) {
                    startEngine();
                    setEngineFlag(ENGINE_RUN);
                }

                // Add periodic torque transmission to the driver
                struct CAN_scheduledMsg msgLeftThrottle = {
                    .header = TxHeaderTHL,            // Configured in can_bus.c (0x226)
                    .periodMs = 95,                   // Send every 100 ms
                    .getData = EngineThrottle_GetData,
                    .context = (void*)LEFT_ENGINE
                };

                // Add periodic torque transmission to the driver
			    struct CAN_scheduledMsg msgRightThrottle = {
				   .header = TxHeaderTHR,            // Configured in can_bus.c (0x227)
				   .periodMs = 95,
				   .getData = EngineThrottle_GetData,
				   .context = (void*)RIGHT_ENGINE
			   };
			    CAN_AddScheduledMsg(&msgLeftThrottle, &canScheduler);
                CAN_AddScheduledMsg(&msgRightThrottle, &canScheduler);
                
                break;
            }

            case JTSN_DOWN_NEUTRAL_GEAR_STATE:
            	// Fall through case to not repeat the code
            case SAFE_STOP_STATE: {
            	if(Vehicle.WheelSpeed.SpeedRL != 0 || Vehicle.WheelSpeed.SpeedRR != 0)
            	{
            		isBad = 1;
            		break;
            	}
				if (getEngineFlag() != ENGINE_STOP_NEUTRAL) {
					neutralEngine(); // NMT (Neutral) frame - one-shot
					setEngineFlag(ENGINE_STOP_NEUTRAL);
				}
                break;
            }

            case JTSN_DOWN_PARKING_STATE: {
            	if(Vehicle.WheelSpeed.SpeedRL != 0 || Vehicle.WheelSpeed.SpeedRR != 0)
            	{
            		isBad = 1;
            		break;
            	}
				if (getEngineFlag() != ENGINE_STOP_BLOCKED) {
					stopEngine(); // NMT (Stop) frame - one-shot
					setEngineFlag(ENGINE_STOP_BLOCKED);
				}
			break;
            }

            default:
                break;
        }

        // 3. Save the current state, so nothing is executed no change occurs
        if(!isBad)
        {
        	lastState = currentState;
        }
    }
}
