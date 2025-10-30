/**
 * Flight state parameter, 0 Nothing 1 Kamikaze
 *
 * @boolean
 * @group Nova
 */
PARAM_DEFINE_INT32(FLIGHT_STATE, 0);

/**
 * Kamikaze dive altitude
 *
 * @min 10
 * @max 60
 *
 * @group Nova
 */
PARAM_DEFINE_INT32(KAMIK_DIVE_ALT, 30);

/**
 * Kamikaze dive angle
 *
 * @min 30
 * @max 60
 *
 * @group Nova
 */
PARAM_DEFINE_INT32(KAMIK_DIVE_ANG, 45);

/**
 * Kamikaze dive throttle (0-1)
 *
 * @min 0.0
 * @max 1.0
 *
 * @group Nova
 */
PARAM_DEFINE_FLOAT(KAMIK_DIVE_THR, 0.2);
