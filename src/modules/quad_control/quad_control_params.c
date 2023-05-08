// Parameters for the quad controller
// Devansh Agrawal
// April 2023


/**
 * Position controller gain kx
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_KX, 1.0f);

/**
 * Position controller gain kv
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_KV, 2.0f);

/**
 * Position controller gain ki
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.001
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_KI, 0.001f);

/**
 * Position controller gain kR
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_KR, 0.35f);

/**
 * Position controller gain kOmega
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_KOMEGA, 0.15f);

/**
 * Mass of Quadrotor
 *
 * @unit kg
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_M, 0.8f);

/**
 * Moment of inertia about xx axis
 *
 * @unit kg m^2
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_JXX, 0.005);

/**
 * Moment of inertia about yy axis
 *
 * @unit kg m^2
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_JYY, 0.005);

/**
 * Moment of inertia about zz axis
 *
 * @unit kg m^2
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_JZZ, 0.009);

/**
 * Motor Thrust Coefficient (N / (kilo-rad/s)^2 ) 
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_KTHRUST, 5.48);

/**
 * Motor Torque Coefficient (N m / (kilo-rad/s)^2) 
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_KTORQUE, 0.33);


/**
 * Max Motor Speed (krad/s)
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_OMEGA_MAX, 1.1);

/**
 * Land Speed (m/s) 
 *
 * @unit m/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_LAND_SPEED, 0.2);


/**
 * ESC Nonlinearity Coefficient [0, 1]. linear: 0.0, quadratic: 1.0;
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_ESC_NONLIN, 0.0);

/**
 * Rotor Position (m)
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_ROT1_POSX, 0.15); 
/**
 * Rotor Position (m)
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_ROT2_POSX, -0.15); 
/**
 * Rotor Position (m)
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_ROT3_POSX, 0.15); 
/**
 * Rotor Position (m)
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_ROT4_POSX, -0.15); 

/**
 * Rotor Position (m)
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_ROT1_POSY, 0.15); 
/**
 * Rotor Position (m)
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_ROT2_POSY, -0.15); 
/**
 * Rotor Position (m)
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_ROT3_POSY, -0.15); 
/**
 * Rotor Position (m)
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group QuadControl
 */
PARAM_DEFINE_FLOAT(QUAD_ROT4_POSY, 0.15); 
