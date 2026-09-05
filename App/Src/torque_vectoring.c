#include "torque_vectoring.h"
#include "tv_config.h"

#include <limits.h>
#include <stddef.h>

/*
 * Integer width policy for this file.
 *
 * Every value that crosses the API is 32-bit or narrower, and each intermediate
 * is computed in 32-bit arithmetic wherever the validated input ranges prove
 * that it fits. The reason is the target core: a Cortex-M4/M7 divides 32-bit
 * integers with a single UDIV instruction, while a 64-bit division becomes a
 * call to __aeabi_uldivmod that costs roughly two orders of magnitude more
 * cycles. 64-bit multiplication is cheap by comparison (UMULL/UMLAL), so the
 * remaining 64-bit code is limited to expressions that genuinely need the extra
 * bits and multiply rather than divide: squared wheel speeds and the
 * load-transfer proxies.
 *
 * The static assertions below pin every bound the 32-bit paths rely on, so a
 * later edit of config.h cannot silently reintroduce an overflow.
 */

/**
 * Largest speed the kinematic projections can return [mm/s]:
 * hypot(v, v * L / t) < v * (1 + L / t) with v, L and t at their limits.
 */
#define TV_MAX_KINEMATIC_MMPS                                                 \
    ((uint32_t)TV_CONFIG_MAX_SPEED_MMPS *                                     \
     (1U + TV_CONFIG_MAX_GEOMETRY_MM / TV_CONFIG_MIN_GEOMETRY_MM))

/** Largest pair of arguments hypot_rounded_u32 can square inside 32 bits. */
#define TV_HYPOT_NARROW_MAX_MMPS 46340U

_Static_assert(TV_CONFIG_MIN_GEOMETRY_MM > 0U &&
                   TV_CONFIG_MIN_GEOMETRY_MM <= TV_CONFIG_MAX_GEOMETRY_MM,
               "geometry limits must form a non-empty range");
_Static_assert(TV_CONFIG_MAX_GEOMETRY_MM <= UINT16_MAX,
               "geometry limit must be representable in the uint16_t fields");
_Static_assert((uint64_t)TV_CONFIG_MAX_SPEED_MMPS * 4U <= UINT32_MAX,
               "sum of four wheel speeds must fit in 32 bits");
_Static_assert((uint64_t)TV_CONFIG_MAX_SPEED_MMPS * TV_CONFIG_MAX_GEOMETRY_MM +
                       TV_CONFIG_MAX_GEOMETRY_MM / 2U <=
                   UINT32_MAX,
               "speed difference times a vehicle dimension must fit in 32 bits");
_Static_assert((uint64_t)TV_CONFIG_MAX_SPEED_MMPS * 1000U +
                       TV_CONFIG_MAX_GEOMETRY_MM / 2U <=
                   UINT32_MAX,
               "yaw-rate and EWMA numerators must fit in 32 bits");
_Static_assert((uint64_t)TV_MAX_KINEMATIC_MMPS * TV_CONFIG_SLIP_SPEED_PERMILLE +
                       500U <=
                   UINT32_MAX,
               "slip-threshold numerator must fit in 32 bits");
_Static_assert((uint64_t)TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM * 1000U *
                       TV_CONFIG_RADIUS_CORRECTION_PERMILLE +
                       500U <=
                   UINT32_MAX,
               "corrected turn-radius numerator must fit in 32 bits");
_Static_assert((uint64_t)UINT16_MAX * 100U + 50U <= UINT32_MAX,
               "gain blend numerator must fit in 32 bits");
_Static_assert(2U * ((uint64_t)TV_HYPOT_NARROW_MAX_MMPS *
                     TV_HYPOT_NARROW_MAX_MMPS) <=
                   UINT32_MAX,
               "narrow hypot radicand must fit in 32 bits");

/** @brief Absolute value for int32_t; unsigned negation is exact at INT32_MIN. */
static uint32_t absolute_i32(int32_t value)
{
    return value < 0 ? 0U - (uint32_t)value : (uint32_t)value;
}

/**
 * @brief Checks the geometry the kinematic equations multiply and divide by.
 *
 * Beyond rejecting a division by zero, the accepted ranges are what makes the
 * 32-bit products in this file provably safe: speed * dimension stays inside
 * 32 bits, and requiring the centre of mass to lie between the axles keeps
 * l_r below the wheelbase.
 */
static bool geometry_is_valid(const VehicleParameters *vehicle)
{
    if (vehicle == NULL ||
        vehicle->wheelbase_mm < TV_CONFIG_MIN_GEOMETRY_MM ||
        vehicle->wheelbase_mm > TV_CONFIG_MAX_GEOMETRY_MM ||
        vehicle->track_width_mm < TV_CONFIG_MIN_GEOMETRY_MM ||
        vehicle->track_width_mm > TV_CONFIG_MAX_GEOMETRY_MM) {
        return false;
    }
    return 2U * absolute_i32(vehicle->cg_offset_from_midpoint_mm) <
           vehicle->wheelbase_mm;
}

/** @brief Checks physical ranges required by the integer equations. */
static bool vehicle_is_valid(const VehicleParameters *vehicle)
{
    if (!geometry_is_valid(vehicle) || vehicle->mass_kg == 0U ||
        vehicle->cg_height_mm == 0U ||
        vehicle->cg_height_mm > TV_CONFIG_MAX_GEOMETRY_MM ||
        vehicle->gravity_mmps2 == 0U || vehicle->friction_permille == 0U ||
        vehicle->command_min >= vehicle->command_max) {
        return false;
    }

    /* command_max > command_min, so the unsigned difference is exact even when
       the two straddle zero. Every later use of the range relies on this
       bound, which is what keeps the blend arithmetic 32-bit. */
    const uint32_t command_range =
        (uint32_t)vehicle->command_max - (uint32_t)vehicle->command_min;
    return command_range <= UINT16_MAX;
}

/** @brief Command range of a validated vehicle [command units]. */
static uint32_t command_range_of(const VehicleParameters *vehicle)
{
    /* At most UINT16_MAX after validation, so the signed subtraction cannot
       overflow. */
    return (uint32_t)(vehicle->command_max - vehicle->command_min);
}

/**
 * @brief Unsigned 32-bit division rounded to the nearest integer.
 *
 * Callers must keep numerator + denominator / 2 inside 32 bits; the static
 * assertions above cover every call site in this file.
 */
static uint32_t divide_rounded_u32(uint32_t numerator, uint32_t denominator)
{
    return (numerator + denominator / 2U) / denominator;
}

/**
 * @brief Unsigned division rounded to the nearest integer, 64-bit capable.
 *
 * Reserved for the expressions whose operands cannot be bounded to 32 bits:
 * squared wheel speeds and the load-transfer proxies. Operands that do fit
 * take the 32-bit path, which is one UDIV instead of an __aeabi_uldivmod call,
 * so range-checked inputs never pay for the wide case.
 */
static uint64_t divide_rounded_u64(uint64_t numerator, uint64_t denominator)
{
    const uint64_t biased = numerator + denominator / 2U;
    if (biased <= UINT32_MAX && denominator <= UINT32_MAX) {
        return (uint32_t)biased / (uint32_t)denominator;
    }
    return biased / denominator;
}

/** @brief Floor of sqrt(value) for a 32-bit radicand; digit-by-digit. */
static uint32_t isqrt_u32(uint32_t value)
{
    uint32_t remainder = value;
    uint32_t root = 0U;
    uint32_t bit = (uint32_t)1U << 30;

    while (bit > remainder) {
        bit >>= 2;
    }
    while (bit != 0U) {
        if (remainder >= root + bit) {
            remainder -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return root;
}

/**
 * @brief Floor of sqrt(value) for a 64-bit radicand.
 *
 * Same algorithm as isqrt_u32 for the radicands that exceed 32 bits; the
 * result always fits in 32 bits.
 */
static uint32_t isqrt_u64(uint64_t value)
{
    uint64_t remainder = value;
    uint64_t root = 0U;
    uint64_t bit = (uint64_t)1U << 62;

    while (bit > remainder) {
        bit >>= 2;
    }
    while (bit != 0U) {
        if (remainder >= root + bit) {
            remainder -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)root;
}

/** @brief sqrt(value) rounded to nearest, 32-bit radicand. */
static uint32_t isqrt_rounded_u32(uint32_t value)
{
    const uint32_t root = isqrt_u32(value);
    /* value - root^2 > root means value lies closer to (root + 1)^2. */
    if (value - root * root > root) {
        return root + 1U;
    }
    return root;
}

/** @brief sqrt(value) rounded to the nearest unsigned 32-bit integer. */
static uint32_t isqrt_rounded_u64(uint64_t value)
{
    const uint32_t root = isqrt_u64(value);
    if (root == UINT32_MAX) {
        return root;
    }
    const uint64_t remainder = value - (uint64_t)root * root;
    if (remainder > root) {
        return root + 1U;
    }
    return root;
}

/** @brief Distance from the rear axle to the CoM [mm]. */
static uint32_t rear_axle_to_cg_mm(const VehicleParameters *vehicle)
{
    /* geometry_is_valid() guarantees 2 * |x_c| < L, so l_r = (L - 2 * x_c) / 2
       is positive and below the wheelbase. */
    return (uint32_t)(((int32_t)vehicle->wheelbase_mm -
                       2 * (int32_t)vehicle->cg_offset_from_midpoint_mm) /
                      2);
}

/** @brief Absolute difference of two unsigned speeds. */
static uint32_t unsigned_delta_u32(uint32_t a, uint32_t b)
{
    return a >= b ? a - b : b - a;
}

/**
 * @brief True when both speeds are at most TV_CONFIG_MAX_SPEED_MMPS.
 *
 * Rejecting out-of-range sensor values here is what keeps every kinematic
 * product in 32 bits and every result in 32 bits.
 */
static bool speeds_in_range_mmps(uint32_t a_mmps, uint32_t b_mmps)
{
    return a_mmps <= TV_CONFIG_MAX_SPEED_MMPS &&
           b_mmps <= TV_CONFIG_MAX_SPEED_MMPS;
}

/** @brief hypot(a, b) rounded to the nearest unsigned 32-bit integer. */
static uint32_t hypot_rounded_u32(uint32_t a, uint32_t b)
{
    if (a <= TV_HYPOT_NARROW_MAX_MMPS && b <= TV_HYPOT_NARROW_MAX_MMPS) {
        return isqrt_rounded_u32(a * a + b * b);
    }

    /* Only reachable for speeds far above the operating envelope, where the
       sum of squares needs more than 32 bits. */
    const uint64_t a_squared = (uint64_t)a * a;
    const uint64_t b_squared = (uint64_t)b * b;
    const uint64_t sum =
        a_squared > UINT64_MAX - b_squared
            ? UINT64_MAX
            : a_squared + b_squared;
    return isqrt_rounded_u64(sum);
}

/**
 * @brief Blends a command offset toward the equal-split (pedal) offset.
 * @param offset Calculated per-wheel offset from command_min.
 * @param pedal_offset Equal-split offset, i.e. pedal_command - command_min.
 * @param gain_percent Blend factor already clamped to [0, 100]; 0 returns
 *        pedal_offset exactly, 100 returns offset unchanged.
 * @return offset moved gain_percent of the way from pedal_offset, rounded to
 *         the nearest integer.
 *
 * Both offsets are at most the command range, i.e. UINT16_MAX, so the whole
 * blend fits in 32 bits. It is implemented with an unsigned magnitude and an
 * explicit direction so that no intermediate value can underflow, matching the
 * rest of this file's unsigned-arithmetic style.
 */
static uint32_t blend_offset_towards_pedal(
    uint32_t offset,
    uint32_t pedal_offset,
    uint32_t gain_percent)
{
    if (offset >= pedal_offset) {
        const uint32_t delta = offset - pedal_offset;
        return pedal_offset + divide_rounded_u32(delta * gain_percent, 100U);
    }
    const uint32_t delta = pedal_offset - offset;
    /* gain_percent <= 100, so the rounded reduction never exceeds delta. */
    const uint32_t reduction = divide_rounded_u32(delta * gain_percent, 100U);
    return pedal_offset - reduction;
}

/** @brief Adds an unsigned offset and clamps it to the configured command range. */
static int32_t command_from_offset(
    const VehicleParameters *vehicle,
    uint32_t offset)
{
    const uint32_t command_range = command_range_of(vehicle);
    if (offset > command_range) {
        offset = command_range;
    }
    /* command_min + offset is at most command_max, so the signed addition
       cannot overflow. */
    return vehicle->command_min + (int32_t)offset;
}

VehicleParameters tv_default_vehicle(void)
{
    return (VehicleParameters){
        /* Measured mass of the complete vehicle [kg]. */
        .mass_kg = 850U,

        /* CG is 511 mm above ground. */
        .cg_height_mm = 511U,

        /* CG is 40 mm ahead of the wheelbase midpoint. */
        .cg_offset_from_midpoint_mm = -40,

        /* Axle spacing, rear track width, and dynamic wheel radius [mm]. */
        .wheelbase_mm = 2750U,
        .track_width_mm = 1700U,
        .wheel_radius_mm = 350U,

        /* Tyre-road friction coefficient: 800 / 1000 = 0.8. */
        .friction_permille = 800U,

        /* Standard gravity expressed in integer STM32 units [mm/s^2]. */
        .gravity_mmps2 = 9810U,

        /* Valid pedal input and rear-wheel output range. */
        .command_min = TV_CONFIG_COMMAND_MIN,
        .command_max = TV_CONFIG_COMMAND_MAX,
    };
}

uint32_t tv_rack_displacement_to_radius_mm(int32_t rack_displacement_mm)
{
    const uint32_t magnitude_mm = absolute_i32(rack_displacement_mm);
    if (magnitude_mm < TV_CONFIG_RACK_MIN_MM ||
        magnitude_mm > TV_CONFIG_RACK_MAX_MM) {
        return 0U;
    }

    const uint32_t fitted_radius_mm = divide_rounded_u32(
        (uint32_t)TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM * 1000U, magnitude_mm);
    const uint32_t scaled_radius_mm = divide_rounded_u32(
        fitted_radius_mm * TV_CONFIG_RADIUS_CORRECTION_PERMILLE, 1000U);

    /* Apply the signed calibration offset without widening to 64 bits. */
    if (TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM < 0) {
        const uint32_t reduction_mm =
            0U - (uint32_t)TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM;
        return scaled_radius_mm > reduction_mm
                   ? scaled_radius_mm - reduction_mm
                   : 0U;
    }
    const uint32_t increase_mm = (uint32_t)TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM;
    if (scaled_radius_mm > UINT32_MAX - increase_mm) {
        return 0U;
    }
    return scaled_radius_mm + increase_mm;
}

uint32_t tv_com_velocity_from_rear_wheels_mmps(
    const VehicleParameters *vehicle,
    uint32_t rear_left_speed_mmps,
    uint32_t rear_right_speed_mmps)
{
    if (!geometry_is_valid(vehicle) ||
        !speeds_in_range_mmps(rear_left_speed_mmps, rear_right_speed_mmps)) {
        return 0U;
    }

    const uint32_t vx_mmps =
        divide_rounded_u32(rear_left_speed_mmps + rear_right_speed_mmps, 2U);
    const uint32_t speed_diff_mmps =
        unsigned_delta_u32(rear_left_speed_mmps, rear_right_speed_mmps);
    const uint32_t vy_mmps = divide_rounded_u32(
        speed_diff_mmps * rear_axle_to_cg_mm(vehicle),
        vehicle->track_width_mm);
    return hypot_rounded_u32(vx_mmps, vy_mmps);
}

uint32_t tv_wheel_rpm_to_speed_mmps(
    const VehicleParameters *vehicle,
    uint32_t rpm)
{
    if (vehicle == NULL || vehicle->wheel_radius_mm == 0U) {
        return 0U;
    }

    /* v = RPM * r * pi / 30 with pi ~ 355/113 -> RPM * r * 355 / 3390. The RPM
       argument is deliberately not range-checked, so the wide helper guards the
       product; every rotation rate a wheel can physically reach still takes its
       32-bit path. */
    const uint64_t speed_mmps = divide_rounded_u64(
        (uint64_t)rpm * vehicle->wheel_radius_mm * 355U, 3390U);
    return speed_mmps > UINT32_MAX ? UINT32_MAX : (uint32_t)speed_mmps;
}

uint32_t tv_wheel_angular_speed_to_linear_mmps(
    const VehicleParameters *vehicle,
    uint32_t angular_speed_mradps)
{
    if (vehicle == NULL || vehicle->wheel_radius_mm == 0U) {
        return 0U;
    }

    const uint64_t speed_mmps = divide_rounded_u64(
        (uint64_t)angular_speed_mradps * vehicle->wheel_radius_mm, 1000U);
    return speed_mmps > UINT32_MAX ? UINT32_MAX : (uint32_t)speed_mmps;
}

uint32_t tv_filter_wheel_speed_mmps(
    TvWheelSpeedFilter *filter,
    uint32_t sample_mmps)
{
    if (filter == NULL) {
        return 0U;
    }
    /* Clamping both the sample and the stored state to the configured speed
       limit matches the range contract of the other kinematics functions and
       keeps the weighted sum below 1000 * TV_CONFIG_MAX_SPEED_MMPS, i.e. inside
       32 bits. */
    if (sample_mmps > TV_CONFIG_MAX_SPEED_MMPS) {
        sample_mmps = TV_CONFIG_MAX_SPEED_MMPS;
    }
    if (!filter->initialized) {
        filter->speed_mmps = sample_mmps;
        filter->initialized = true;
        return sample_mmps;
    }

    uint32_t previous_mmps = filter->speed_mmps;
    if (previous_mmps > TV_CONFIG_MAX_SPEED_MMPS) {
        previous_mmps = TV_CONFIG_MAX_SPEED_MMPS;
    }
    uint32_t alpha_permille = TV_CONFIG_EWMA_ALPHA_PERMILLE;
    if (alpha_permille > 1000U) {
        alpha_permille = 1000U;
    }
    const uint32_t filtered_mmps = divide_rounded_u32(
        alpha_permille * sample_mmps +
            (1000U - alpha_permille) * previous_mmps,
        1000U);
    filter->speed_mmps = filtered_mmps;
    return filtered_mmps;
}

uint32_t tv_com_velocity_from_wheel_speeds_mmps(
    const VehicleParameters *vehicle,
    uint32_t front_left_speed_mmps,
    uint32_t front_right_speed_mmps,
    uint32_t rear_left_speed_mmps,
    uint32_t rear_right_speed_mmps)
{
    if (!geometry_is_valid(vehicle) ||
        !speeds_in_range_mmps(front_left_speed_mmps, front_right_speed_mmps) ||
        !speeds_in_range_mmps(rear_left_speed_mmps, rear_right_speed_mmps)) {
        return 0U;
    }

    const uint32_t yaw_mradps = divide_rounded_u32(
        unsigned_delta_u32(rear_left_speed_mmps, rear_right_speed_mmps) * 1000U,
        vehicle->track_width_mm);
    if (yaw_mradps <= TV_CONFIG_STRAIGHT_YAW_MRADPS) {
        return divide_rounded_u32(
            front_left_speed_mmps + front_right_speed_mmps +
                rear_left_speed_mmps + rear_right_speed_mmps,
            4U);
    }
    return tv_com_velocity_from_rear_wheels_mmps(
        vehicle, rear_left_speed_mmps, rear_right_speed_mmps);
}

TvSlipCheck tv_check_wheel_slip(
    const VehicleParameters *vehicle,
    uint32_t front_left_speed_mmps,
    uint32_t front_right_speed_mmps,
    uint32_t rear_left_speed_mmps,
    uint32_t rear_right_speed_mmps)
{
    TvSlipCheck result = {0};
    if (!geometry_is_valid(vehicle) ||
        !speeds_in_range_mmps(front_left_speed_mmps, front_right_speed_mmps) ||
        !speeds_in_range_mmps(rear_left_speed_mmps, rear_right_speed_mmps)) {
        return result;
    }

    result.rear_com_velocity_mmps = tv_com_velocity_from_rear_wheels_mmps(
        vehicle, rear_left_speed_mmps, rear_right_speed_mmps);

    const uint32_t vx_mmps =
        divide_rounded_u32(rear_left_speed_mmps + rear_right_speed_mmps, 2U);
    const uint32_t omega_times_wheelbase_mmps = divide_rounded_u32(
        unsigned_delta_u32(rear_left_speed_mmps, rear_right_speed_mmps) *
            vehicle->wheelbase_mm,
        vehicle->track_width_mm);
    result.front_projected_mmps =
        hypot_rounded_u32(vx_mmps, omega_times_wheelbase_mmps);
    result.front_measured_mmps =
        divide_rounded_u32(front_left_speed_mmps + front_right_speed_mmps, 2U);

    const uint32_t disagreement_mmps = unsigned_delta_u32(
        result.front_measured_mmps, result.front_projected_mmps);
    const uint32_t limit_mmps = divide_rounded_u32(
        result.front_projected_mmps * TV_CONFIG_SLIP_SPEED_PERMILLE, 1000U);
    result.slip_detected = disagreement_mmps >= TV_CONFIG_SLIP_MIN_MMPS &&
                           disagreement_mmps >= limit_mmps;
    return result;
}

WheelCommands tv_calculate_rear_commands_from_rack(
    const VehicleParameters *vehicle,
    bool rack_position_available,
    int32_t rack_displacement_mm,
    uint32_t vehicle_speed_mmps,
    int32_t pedal_command,
    uint32_t tv_gain_percent)
{
    if (tv_gain_percent > 100U) {
        tv_gain_percent = 100U;
    }

    WheelCommands commands = {.status = TV_INVALID_ARGUMENT};
    if (!vehicle_is_valid(vehicle)) {
        return commands;
    }

    commands.rear_left = vehicle->command_min;
    commands.rear_right = vehicle->command_min;
    if (pedal_command < vehicle->command_min ||
        pedal_command > vehicle->command_max) {
        return commands;
    }
    if (vehicle_speed_mmps > TV_CONFIG_MAX_SPEED_MMPS) {
        commands.status = TV_SPEED_OUT_OF_RANGE;
        return commands;
    }

    if (!rack_position_available || rack_displacement_mm == 0) {
        commands.rear_left = pedal_command;
        commands.rear_right = pedal_command;
        commands.status = TV_OK;
        return commands;
    }

    const uint32_t radius_mm =
        tv_rack_displacement_to_radius_mm(rack_displacement_mm);
    if (radius_mm == 0U) {
        commands.status = TV_RACK_OUT_OF_RANGE;
        return commands;
    }
    commands.turn_radius_mm = radius_mm;

    /* A squared speed needs 34 bits at the configured speed limit, so this is
       one of the few 64-bit values here. The grip comparison below is a 64-bit
       multiplication only, and the division takes its 32-bit path up to
       65.5 m/s. */
    const uint64_t speed_squared =
        (uint64_t)vehicle_speed_mmps * vehicle_speed_mmps;
    const uint32_t grip_acceleration_mmps2 =
        (uint32_t)vehicle->friction_permille * vehicle->gravity_mmps2 / 1000U;
    if (speed_squared > (uint64_t)radius_mm * grip_acceleration_mmps2) {
        commands.status = TV_LATERAL_GRIP_EXCEEDED;
        return commands;
    }

    const uint32_t lateral_acceleration_mmps2 =
        (uint32_t)divide_rounded_u64(speed_squared, radius_mm);
    commands.lateral_acceleration_mmps2 = lateral_acceleration_mmps2;

    /* Common scale factors cancel in the inner/outer torque ratio. Proxies
       below are proportional to rear-wheel normal loads, and their products of
       three physical quantities exceed 32 bits by construction. They need
       multiplication only (a single UMULL/UMLAL pair each); the split division
       further down is the one place where the exact ratio needs a 64-bit
       divide. */
    const uint32_t rear_numerator_mm =
        (uint32_t)((int32_t)vehicle->wheelbase_mm +
                   2 * (int32_t)vehicle->cg_offset_from_midpoint_mm);
    const uint64_t static_proxy = (uint64_t)vehicle->gravity_mmps2 *
        rear_numerator_mm * vehicle->track_width_mm;
    const uint64_t transfer_proxy = 4U * (uint64_t)vehicle->wheelbase_mm *
        lateral_acceleration_mmps2 * vehicle->cg_height_mm;

    const uint64_t inner_weight =
        static_proxy > transfer_proxy ? static_proxy - transfer_proxy : 0U;
    const uint64_t outer_weight = static_proxy + transfer_proxy;
    if (outer_weight == 0U) {
        return commands;
    }

    const uint64_t total_proxy = inner_weight + outer_weight;
    const uint32_t pedal_offset =
        (uint32_t)(pedal_command - vehicle->command_min);
    const uint32_t command_range = command_range_of(vehicle);

    /* Both offsets end up at most command_range, i.e. inside 16 bits, so the
       blend and clamp below stay in 32-bit arithmetic. */
    uint32_t inner_offset;
    uint32_t outer_offset;
    if ((uint64_t)pedal_offset * 2U * outer_weight >
        (uint64_t)command_range * total_proxy) {
        /* Saturate both sides proportionally, preserving the torque split. */
        outer_offset = command_range;
        inner_offset = (uint32_t)divide_rounded_u64(
            (uint64_t)command_range * inner_weight, outer_weight);
    } else {
        inner_offset = (uint32_t)divide_rounded_u64(
            (uint64_t)pedal_offset * 2U * inner_weight, total_proxy);
        outer_offset = (uint32_t)divide_rounded_u64(
            (uint64_t)pedal_offset * 2U * outer_weight, total_proxy);
    }

    const uint32_t blended_inner_offset =
        blend_offset_towards_pedal(inner_offset, pedal_offset, tv_gain_percent);
    const uint32_t blended_outer_offset =
        blend_offset_towards_pedal(outer_offset, pedal_offset, tv_gain_percent);
    const int32_t inner_command = command_from_offset(vehicle, blended_inner_offset);
    const int32_t outer_command = command_from_offset(vehicle, blended_outer_offset);
    if (rack_displacement_mm > 0) {
        commands.rear_left = inner_command;
        commands.rear_right = outer_command;
    } else {
        commands.rear_left = outer_command;
        commands.rear_right = inner_command;
    }
    commands.torque_vectoring_active = inner_command != outer_command;
    commands.status = TV_OK;
    return commands;
}

const char *tv_status_string(TvStatus status)
{
    switch (status) {
    case TV_OK:
        return "ok";
    case TV_LATERAL_GRIP_EXCEEDED:
        return "lateral grip exceeded";
    case TV_RACK_OUT_OF_RANGE:
        return "rack displacement outside calibration range";
    case TV_SPEED_OUT_OF_RANGE:
        return "vehicle speed outside configured range";
    case TV_INVALID_ARGUMENT:
        return "invalid argument";
    default:
        return "unknown status";
    }
}
