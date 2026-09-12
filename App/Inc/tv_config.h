/**
 * @file tv_config.h
 * @brief Compile-time calibration and safety limits for torque vectoring.
 *
 * Copied from the torque-vectoring repository (c_implementation/config.h).
 * Renamed because App/Inc is shared by the whole application and "config.h"
 * is too generic a name to put there. Keep the rest in sync with upstream;
 * the command range and the gain defaults below are this vehicle's settings.
 */

#ifndef TV_CONFIG_H
#define TV_CONFIG_H

/** Minimum pedal and motor command; represents zero requested torque. */
#ifndef TV_CONFIG_COMMAND_MIN
#define TV_CONFIG_COMMAND_MIN 0
#endif

/**
 * Maximum pedal and motor command. Set to the inverter full-scale command, so
 * the split works in the very units the 0x226/0x227 frames carry and nothing
 * is rescaled on the way out. Must stay equal to THROTTLE_MAX_VAL from
 * pedals_map.h; vehicle_fsm.c asserts it at compile time.
 */
#ifndef TV_CONFIG_COMMAND_MAX
#define TV_CONFIG_COMMAND_MAX 32767
#endif

/**
 * Highest accepted torque-vectoring gain [%]; 100 is the full calculated
 * split. Values above it are rejected by TorqueVectoring_SetGain().
 */
#define TV_CONFIG_GAIN_MAX 100U

/**
 * Gain used until a new one arrives over CAN [%]; 50 splits the difference
 * between an open differential (0) and the full calculated split (100).
 */
#define TV_CONFIG_GAIN_DEFAULT 50U

/** Smallest supported non-zero rack displacement magnitude [mm]. */
#define TV_CONFIG_RACK_MIN_MM 1U

/** Largest calibrated rack displacement magnitude [mm]. */
#define TV_CONFIG_RACK_MAX_MM 70U

/** Constant C [m*mm] in the fitted relation R_m = C / abs(rack_mm). */
#define TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM 507U

/** Radius scale [permille]; 1000 = 1.000, 980 = 0.980. */
#define TV_CONFIG_RADIUS_CORRECTION_PERMILLE 1000U

/** Signed radius correction applied after scaling [mm]. */
#define TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM 0

/** Maximum accepted speed sensor value [mm/s]; 100000 = 100 m/s. */
#define TV_CONFIG_MAX_SPEED_MMPS 100000U

/**
 * Smallest accepted vehicle dimension [mm]. Together with the maximum below it
 * bounds every speed-times-length product, which is what lets the calculations
 * stay in 32-bit arithmetic.
 */
#define TV_CONFIG_MIN_GEOMETRY_MM 100U

/** Largest accepted vehicle dimension [mm]; 10000 = 10 m. */
#define TV_CONFIG_MAX_GEOMETRY_MM 10000U

/** EWMA weight for a new wheel-speed sample [permille]; 200 = 0.2. */
#ifndef TV_CONFIG_EWMA_ALPHA_PERMILLE
#define TV_CONFIG_EWMA_ALPHA_PERMILLE 200U
#endif

/**
 * Yaw-rate magnitude at or below which all four wheel speeds are averaged
 * [mrad/s]; 50 = 0.050 rad/s.
 */
#ifndef TV_CONFIG_STRAIGHT_YAW_MRADPS
#define TV_CONFIG_STRAIGHT_YAW_MRADPS 50U
#endif

/**
 * Relative front/rear disagreement that flags wheel slip [permille of the
 * rear-projected front-axle speed]; 100 = 10%.
 */
#ifndef TV_CONFIG_SLIP_SPEED_PERMILLE
#define TV_CONFIG_SLIP_SPEED_PERMILLE 100U
#endif

/**
 * Minimum absolute front/rear disagreement that can flag wheel slip [mm/s];
 * keeps sensor noise near standstill from raising false positives.
 */
#ifndef TV_CONFIG_SLIP_MIN_MMPS
#define TV_CONFIG_SLIP_MIN_MMPS 300U
#endif

#endif
