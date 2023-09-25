// Parameters for the quad controller
// Kaleb, Hardik
// August 2023

/**
 * Distance from front axle to rear axle
 *
 * A value of 0.31 is typical for 1/10 RC cars.
 *
 * @unit m
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group RoverControl
 */
PARAM_DEFINE_FLOAT(GND_WHEEL_BASE, 0.31f);

/**
 * Rover Position Error Gain
 *
 * @min 0.0
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group RoverControlPosition
 */
PARAM_DEFINE_FLOAT(ROVER_KX, 1.0f);

/**
 * Rover Velocity Error Gain
 *
 * @min 0.0
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group RoverControl
 */
PARAM_DEFINE_FLOAT(ROVER_KV, 0.3f);

/**
 * Rover Angle Error Gain
 *
 * @min 0.0
 * @max 50.0
 * @decimal 3
 * @increment 0.01
 * @group RoverControl
 */
PARAM_DEFINE_FLOAT(ROVER_KOMEGA, 1.0f);

/**
 * Max Rover Wheel Rotation speed
 *
 * @unit m/s
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group RoverControl
 */
PARAM_DEFINE_FLOAT(ROVER_SPEED_MAX, 1.3f);
