/* =============================================================================
 * app_basestation.h  --  L4 base station / gateway (ROLE_SINK) entry points.
 * -----------------------------------------------------------------------------
 * The only node with WiFi/MQTT. Bridges the LoRa mesh and the MQTT broker:
 *   uplink  : mesh telemetry  -> publish JSON to rainfall/status
 *   downlink: cap-alert/<id>, flood-confirmed/<id> (MQTT) -> compact CAP frame
 *             routed over the mesh to the matching sensor node, with periodic
 *             re-sync. Definitions compiled only for SINK builds.
 * ===========================================================================*/
#ifndef APP_BASESTATION_H
#define APP_BASESTATION_H

#include <stdint.h>
#include "mesh_types.h"

void app_base_init(void);
void app_base_tick(uint32_t now_ms);
/* Called by the L3 l4_deliver hook when telemetry from a sensor arrives. */
void app_base_on_uplink(node_id_t from, const uint8_t *payload, uint8_t len);

#endif /* APP_BASESTATION_H */
