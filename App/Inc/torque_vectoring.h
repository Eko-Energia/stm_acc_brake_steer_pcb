/**
 * @file torque_vectoring.h
 * @brief Integer-only torque-vectoring API intended for STM32 targets.
 *
 * The real-time path uses fixed physical units and contains no floating-point
 * operations, dynamic allocation, or dependency on math.h/libm/arm_math.
 *
 * Every value in this API is 32-bit or narrower, and the implementation keeps
 * its intermediates 32-bit wherever the validated input ranges prove that they
 * fit, because a 64-bit division on Cortex-M is a libgcc call rather than a
 * single instruction. If floating point is ever introduced here, use the float
 * overloads of the math functions (sqrtf, powf, fabsf, ...): the double
 * versions run in software on a single-precision FPU and each implicit
 * promotion costs cycles for accuracy this controller does not need.
 */

#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Physical parameters and command range of the vehicle Perła. */
typedef struct {
    /** Vehicle mass [kg]. */
    uint16_t mass_kg;

    /** Centre-of-mass height above ground, h [mm]. */
    uint16_t cg_height_mm;

    /**
     * Longitudinal centre-of-mass offset x_c [mm]. Positive points toward the
     * rear; -40 means 40 mm toward the front from the wheelbase midpoint.
     */
    int16_t cg_offset_from_midpoint_mm;

    /**
     * Distance between front and rear axle centres, L [mm]. This and every
     * other dimension must lie within [TV_CONFIG_MIN_GEOMETRY_MM,
     * TV_CONFIG_MAX_GEOMETRY_MM].
     */
    uint16_t wheelbase_mm;

    /** Distance between left and right wheel centres, t [mm]. */
    uint16_t track_width_mm;

    /**
     * Dynamic wheel radius r [mm]. Used to convert wheel RPM or milliradians
     * per second to linear speed. The torque split does not use this field.
     */
    uint16_t wheel_radius_mm;

    /** Tyre friction coefficient [1/1000]; 800 represents 0.8. */
    uint16_t friction_permille;

    /** Gravitational acceleration [mm/s^2]. */
    uint16_t gravity_mmps2;

    /** Pedal/output value representing zero requested torque. */
    int32_t command_min;

    /** Maximum accepted pedal/output value. */
    int32_t command_max;
} VehicleParameters;

/** @brief Result code returned by the control calculation. */
typedef enum {
    /** Calculation succeeded; outputs may be applied. */
    TV_OK = 0,

    /** Requested speed and radius exceed mu*g; outputs equal command_min. */
    TV_LATERAL_GRIP_EXCEEDED,

    /** Non-zero rack value is outside the calibrated range. */
    TV_RACK_OUT_OF_RANGE,

    /** Speed exceeds TV_CONFIG_MAX_SPEED_MMPS. */
    TV_SPEED_OUT_OF_RANGE,

    /**
     * Invalid pointer, pedal command, or vehicle parameters. Geometry must be
     * within [TV_CONFIG_MIN_GEOMETRY_MM, TV_CONFIG_MAX_GEOMETRY_MM] with the
     * centre of mass between the axles, and command_max - command_min must not
     * exceed UINT16_MAX.
     */
    TV_INVALID_ARGUMENT
} TvStatus;

/** @brief Rear-wheel commands and diagnostic values for one control iteration. */
typedef struct {
    /** Rear-left motor command [configured command units]. */
    int32_t rear_left;

    /** Rear-right motor command [configured command units]. */
    int32_t rear_right;

    /** Unsigned fitted turn radius [mm]; 0 in fallback or error. */
    uint32_t turn_radius_mm;

    /** Calculated lateral acceleration [mm/s^2]. */
    uint32_t lateral_acceleration_mmps2;

    /** True when an unequal torque split was calculated. */
    bool torque_vectoring_active;

    /** Validity and status of all fields above. */
    TvStatus status;
} WheelCommands;

/**
 * @brief Caller-owned EWMA state for one wheel-speed signal [mm/s].
 *
 * Zero-initialize before the first update. The kinematics functions do not
 * store this state; the caller retains the last filtered sample if a CAN
 * frame is missing in the current cycle.
 */
typedef struct {
    /** Last filtered linear speed [mm/s]. */
    uint32_t speed_mmps;

    /** False until the first sample has been accepted. */
    bool initialized;
} TvWheelSpeedFilter;

/** @brief Rear-vs-front kinematic comparison used for slip detection. */
typedef struct {
    /** Bicycle-model CoM speed from the rear axle [mm/s]. */
    uint32_t rear_com_velocity_mmps;

    /** Front-axle centre speed projected from rear kinematics [mm/s]. */
    uint32_t front_projected_mmps;

    /** Average of the two front wheel speeds [mm/s]. */
    uint32_t front_measured_mmps;

    /** True when the front measurement disagrees with the rear projection. */
    bool slip_detected;
} TvSlipCheck;

/**
 * @brief Returns the default parameters of Perła.
 * @return Vehicle parameters in integer STM32 units.
 *
 * The returned structure may be copied and adjusted before it is passed to the
 * controller. The function has no side effects and uses no global mutable state.
 */
VehicleParameters tv_default_vehicle(void);

/**
 * @brief Converts signed steering-rack displacement to an unsigned turn radius.
 * @param rack_displacement_mm Rack displacement from its centre position [mm].
 *        Its sign selects the turn direction; its magnitude selects the radius.
 * @return Turn radius [mm], or 0 if the value is zero/outside calibration.
 *
 * Uses the integer form R_mm = 507000 / abs(rack_mm), followed by configured
 * permille/offset correction for non-zero displacements from 1 to 70 mm.
 * The caller must use the input sign to determine left/right direction. A zero
 * result is not a valid finite radius.
 */
uint32_t tv_rack_displacement_to_radius_mm(int32_t rack_displacement_mm);

/**
 * @brief Estimates CoM velocity magnitude from rear-left and rear-right speeds.
 * @param vehicle Pointer to vehicle geometry; must remain valid for the call.
 * @param rear_left_speed_mmps Rear-left wheel linear speed [mm/s].
 * @param rear_right_speed_mmps Rear-right wheel linear speed [mm/s].
 * @return CoM speed [mm/s], or 0 if vehicle is NULL, the geometry is outside
 *         the accepted range, or any speed exceeds TV_CONFIG_MAX_SPEED_MMPS.
 *
 * Wheelbase and track width must lie within [TV_CONFIG_MIN_GEOMETRY_MM,
 * TV_CONFIG_MAX_GEOMETRY_MM] and the centre of mass between the axles, which
 * is what keeps the calculation inside 32-bit arithmetic.
 *
 * Uses bicycle-model kinematics with the non-steered rear axle:
 * v_x = (v_RL + v_RR) / 2,
 * v_y = (v_RR - v_RL) * l_r / t,
 * v_CoM = sqrt(v_x^2 + v_y^2),
 * where t is track_width_mm and l_r is the rear-axle-to-CoM distance. Integer
 * division is rounded to the nearest value. The result is a magnitude, so
 * swapping the two wheel speeds does not change it.
 */
uint32_t tv_com_velocity_from_rear_wheels_mmps(
    const VehicleParameters *vehicle,
    uint32_t rear_left_speed_mmps,
    uint32_t rear_right_speed_mmps
);

/**
 * @brief Converts wheel rotation rate in RPM to linear speed.
 * @param vehicle Pointer to parameters providing wheel_radius_mm.
 * @param rpm Wheel rotational speed in revolutions per minute.
 * @return Linear speed [mm/s], or 0 if vehicle is NULL or radius is zero.
 *
 * Uses v = RPM * r * π / 30 with π ≈ 355/113 and nearest-integer division.
 */
uint32_t tv_wheel_rpm_to_speed_mmps(
    const VehicleParameters *vehicle,
    uint32_t rpm
);

/**
 * @brief Converts wheel angular speed to linear speed.
 * @param vehicle Pointer to parameters providing wheel_radius_mm.
 * @param angular_speed_mradps Angular speed [mrad/s]; 1000 = 1 rad/s.
 * @return Linear speed [mm/s], or 0 if vehicle is NULL or radius is zero.
 *
 * Uses v = ω * r with milliradian scaling: v_mmps = ω_mradps * r_mm / 1000.
 */
uint32_t tv_wheel_angular_speed_to_linear_mmps(
    const VehicleParameters *vehicle,
    uint32_t angular_speed_mradps
);

/**
 * @brief Applies a fixed-point EWMA to one wheel-speed sample.
 * @param filter Caller-owned filter state; must remain valid for the call.
 * @param sample_mmps New linear speed reading [mm/s].
 * @return Filtered speed [mm/s], or 0 if filter is NULL.
 *
 * Samples above TV_CONFIG_MAX_SPEED_MMPS are clamped to that limit, as is a
 * state value written directly by the caller, so one implausible reading
 * cannot scale the filter out of its 32-bit range.
 *
 * The first sample initialises the state. Later updates use
 * y = α * x + (1-α) * y_prev with α from TV_CONFIG_EWMA_ALPHA_PERMILLE
 * (200 = 0.2). If a CAN frame is missing, do not call this function; reuse
 * filter->speed_mmps.
 */
uint32_t tv_filter_wheel_speed_mmps(
    TvWheelSpeedFilter *filter,
    uint32_t sample_mmps
);

/**
 * @brief Estimates CoM velocity from all four wheel speeds.
 * @param vehicle Pointer to vehicle geometry; must remain valid for the call.
 * @param front_left_speed_mmps Front-left wheel linear speed [mm/s].
 * @param front_right_speed_mmps Front-right wheel linear speed [mm/s].
 * @param rear_left_speed_mmps Rear-left wheel linear speed [mm/s].
 * @param rear_right_speed_mmps Rear-right wheel linear speed [mm/s].
 * @return CoM speed [mm/s], or 0 if vehicle is NULL, the geometry is outside
 *         the range accepted by tv_com_velocity_from_rear_wheels_mmps, or any
 *         speed exceeds TV_CONFIG_MAX_SPEED_MMPS.
 *
 * When the rear-axle yaw rate is at or below TV_CONFIG_STRAIGHT_YAW_MRADPS,
 * returns the average of all four speeds. Otherwise returns the rear-axle
 * bicycle-model estimate and ignores the front pair.
 */
uint32_t tv_com_velocity_from_wheel_speeds_mmps(
    const VehicleParameters *vehicle,
    uint32_t front_left_speed_mmps,
    uint32_t front_right_speed_mmps,
    uint32_t rear_left_speed_mmps,
    uint32_t rear_right_speed_mmps
);

/**
 * @brief Compares rear-axle kinematics with measured front-axle speed.
 * @param vehicle Pointer to vehicle geometry; must remain valid for the call.
 * @param front_left_speed_mmps Front-left wheel linear speed [mm/s].
 * @param front_right_speed_mmps Front-right wheel linear speed [mm/s].
 * @param rear_left_speed_mmps Rear-left wheel linear speed [mm/s].
 * @param rear_right_speed_mmps Rear-right wheel linear speed [mm/s].
 * @return Diagnostic speeds and slip_detected. All speeds are 0 if vehicle is
 *         NULL, the geometry is outside the range accepted by
 *         tv_com_velocity_from_rear_wheels_mmps, or any speed exceeds
 *         TV_CONFIG_MAX_SPEED_MMPS.
 *
 * Projects the front-axle centre speed as hypot(v_x, ω * L) from the rear
 * wheels and compares it with (v_FL + v_FR) / 2. Disagreement of at least
 * TV_CONFIG_SLIP_SPEED_PERMILLE of the projected speed flags slip, but never
 * less than TV_CONFIG_SLIP_MIN_MMPS, so sensor noise near standstill does not
 * raise false positives.
 */
TvSlipCheck tv_check_wheel_slip(
    const VehicleParameters *vehicle,
    uint32_t front_left_speed_mmps,
    uint32_t front_right_speed_mmps,
    uint32_t rear_left_speed_mmps,
    uint32_t rear_right_speed_mmps
);

/**
 * @brief Calculates rear-left and rear-right commands using integer arithmetic.
 * @param vehicle Pointer to validated vehicle parameters; must remain valid for
 *        the duration of the call.
 * @param rack_position_available true if rack_displacement_mm is a fresh and
 *        valid sensor measurement; false activates equal-split fallback.
 * @param rack_displacement_mm Signed displacement from centred rack [mm].
 *        Positive means a left turn, negative a right turn, and zero activates
 *        equal-split fallback.
 * @param vehicle_speed_mmps Non-negative vehicle speed [mm/s]. For example,
 *        5000 means 5 m/s.
 * @param pedal_command Driver demand in [vehicle.command_min, vehicle.command_max].
 * @param tv_gain_percent Driver/surface-selectable blend between an open
 *        differential and full torque vectoring, as a percentage from 0 to
 *        100; 0 = equal split on both rear wheels regardless of the
 *        calculated load transfer (identical to the rack-unavailable
 *        fallback), 100 = full calculated split (previous, unblended
 *        behaviour). Values above 100 are clamped to 100. Intermediate
 *        values linearly blend each wheel's command between the two, so the
 *        wheel commands still sum to 2 * pedal_command whenever the full
 *        split itself is unsaturated.
 * @return Commands and diagnostics. Apply motor commands only when status is
 *         TV_OK. In fallback, both outputs equal pedal_command and
 *         torque_vectoring_active is false. torque_vectoring_active is also
 *         false whenever blending happens to make both commands equal, such
 *         as tv_gain_percent == 0.
 *
 * @note Call this function once per control-loop iteration. It is re-entrant,
 * deterministic, allocation-free, and has no dependency on an FPU or DSP library.
 */
WheelCommands tv_calculate_rear_commands_from_rack(
    const VehicleParameters *vehicle,
    bool rack_position_available,
    int32_t rack_displacement_mm,
    uint32_t vehicle_speed_mmps,
    int32_t pedal_command,
    uint32_t tv_gain_percent
);

/**
 * @brief Returns a static English description of a TvStatus value.
 * @param status Status code to describe.
 * @return Pointer to a null-terminated static string. Never free this pointer.
 * @note Intended for logs/debug builds; the control algorithm does not require it.
 */
const char *tv_status_string(TvStatus status);

#endif
