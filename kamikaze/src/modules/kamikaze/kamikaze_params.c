/**
 * Flight state parameter, 0 Nothing 1 Kamikaze
 *
 *
 * @group Nova
 */
PARAM_DEFINE_INT32(FLIGHT_STATE, 0);

/**
 * Kamikaze dive altitude
 *
 *
 * @group Nova
 */
PARAM_DEFINE_INT32(KAMIK_DIVE_ALT, 30);

/**
 * Kamikaze rise altitude
 *
 *
 * @group Nova
 */
PARAM_DEFINE_INT32(KAMIK_RISE_ALT, 50);

/**
 * Kamikaze dive angle
 *
 *
 * @group Nova
 */
PARAM_DEFINE_INT32(KAMIK_DIVE_ANG, 45);

/**
 * Kamikaze rise angle
 *
 *
 * @group Nova
 */
PARAM_DEFINE_INT32(KAMIK_RISE_ANG, 25);

/**
 * Kamikaze dive throttle (0-1)
 *
 *
 * @group Nova
 */
PARAM_DEFINE_FLOAT(KAMIK_DIVE_THR, 0.2);

/**
 * Kamikaze level throttle (0-1)
 *
 *
 * @group Nova
 */
PARAM_DEFINE_FLOAT(KAMIK_LEVEL_THR, 0.3);

/**
 * Kamikaze rise throttle (0-1)
 *
 *
 * @group Nova
 */
PARAM_DEFINE_FLOAT(KAMIK_RISE_THR, 0.85);
