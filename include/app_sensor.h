/* =============================================================================
 * app_sensor.h  --  L4 flood-sensor node (ROLE_SENDER) entry points.
 * -----------------------------------------------------------------------------
 * Reads float water-level sensors + GPS, drives the buzzer/relay, stores CAP
 * alerts received (over the mesh) from the base station, and sends telemetry up
 * to the sink. No WiFi. Definitions are compiled only for non-SINK builds; the
 * declarations stay visible so the node facade can #if-dispatch cleanly.
 * ===========================================================================*/
#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include <stdint.h>
#include "mesh_types.h"

void app_sensor_init(void);
void app_sensor_tick(uint32_t now_ms);
/* Called by the L3 l4_deliver hook when a DATA frame arrives for us (the CAP
 * downlink from the base station). */
void app_sensor_on_downlink(node_id_t from, const uint8_t *payload, uint8_t len);

#endif /* APP_SENSOR_H */
