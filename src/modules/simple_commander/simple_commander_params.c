
/**
 * @file simple_commander_params.c
 *
 * Parameters definition for Commander.
 *
 * @author Devansh Agrawal
 */


/**
 * Run Preflight checks on arming
 *
 * @group SimpleCommander
 * @reboot_required false
 * @boolean
 *
 */
PARAM_DEFINE_INT32(COMM_PREFL_ARM, 0);

/**
 * Run preflight checks on offboard mode
 *
 * @group SimpleCommander
 * @reboot_required false
 * @boolean
 *
 */
PARAM_DEFINE_INT32(COMM_PREFL_OFFB, 1);


