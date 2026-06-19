/* =============================================================================
 * mesh_app.h  --  L4 Application layer: sensor endpoints + telemetry.
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY.
 *
 * The application layer is the ONLY consumer of mesh_l3_send() and the ONLY
 * implementer of the l4_deliver hook registered in mesh_l3_hooks_t. It:
 *   - Samples sensors on a millis()-driven schedule (app_tick).
 *   - Serializes sensor readings into a compact payload.
 *   - Calls mesh_l3_send() toward the configured sink address.
 *   - Receives and logs/processes DATA payloads from other nodes.
 *
 * This layer has no knowledge of routing, CRC, or radio. It only sees the
 * logical mesh: send-to-address and receive-from-address.
 *
 * APP PORTS (1-byte L4 demux, carried in DATA payload byte[0]):
 *   APP_PORT_TELEMETRY (1) — sensor data (temp, humidity, vbat, uptime)
 *   APP_PORT_CMD       (2) — downlink command (future use, sink -> sensor)
 * ===========================================================================*/
#ifndef MESH_APP_H
#define MESH_APP_H

#include <stdint.h>
#include "mesh_types.h"
#include "mesh_config.h"

/* ── App port identifiers ────────────────────────────────────────────────── */
typedef MESH_ENUM8(app_port_e) {
    APP_PORT_TELEMETRY = 1,
    APP_PORT_CMD       = 2
} app_port_t;

/* ── Telemetry payload (carried in DATA.app_data after the app_port byte) ── */
typedef struct {
    int16_t  temp_c_x100;    /* temperature in °C × 100 (e.g. 2350 = 23.50°C) */
    uint16_t humidity_x100;  /* relative humidity × 100 (e.g. 5500 = 55.00%)  */
    uint16_t vbat_mv;        /* battery voltage in mV                          */
    uint32_t uptime_s;       /* node uptime in seconds                         */
} app_telemetry_t;           /* 10 bytes; + 1 app_port byte = 11 B payload     */

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/* Initialize the application layer. `sink` is the destination node for all
 * locally-originated DATA frames (normally NODE_SINK_ADDR from mesh_config.h).
 * Must be called after mesh_l3_init(). */
void app_init(node_id_t sink);

/* ── Main-loop pump ──────────────────────────────────────────────────────── */

/* Called once per loop() pass (non-blocking). Checks if the sample interval
 * has elapsed; if so, reads sensors, builds a telemetry payload, and calls
 * mesh_l3_send() toward the configured sink. Does nothing for RELAY/SINK
 * roles (they receive but do not originate telemetry). */
void app_tick(uint32_t now_ms);

/* ── Upward callback (registered as mesh_l3_hooks.l4_deliver) ────────────── */

/* Called by L3 when a DATA frame addressed to this node arrives. Parses the
 * app_port and dispatches to the appropriate handler (log, forward via USB,
 * etc.). `payload` points into L3's rx buffer and is valid only for the
 * duration of this call — copy if retention is needed. */
void app_on_deliver(node_id_t from_origin, seq_t seq,
                    const uint8_t *payload, uint8_t payload_len);

/* ── Payload (de)serialization ───────────────────────────────────────────── */

/* Serialize `t` into `out` (app_port byte + telemetry fields, LE).
 * Returns the number of bytes written (11), or 0 if cap too small. */
uint8_t app_serialize_telemetry(const app_telemetry_t *t,
                                uint8_t *out, uint8_t cap);

/* Deserialize bytes into `out`. Returns true on success, false if the
 * payload is too short or the port byte does not match APP_PORT_TELEMETRY. */
bool app_deserialize_telemetry(const uint8_t *in, uint8_t in_len,
                               app_telemetry_t *out);

/* ── Core1 sensor offload (USE_CORE1_SENSOR only) ───────────────────────── */
/* When USE_CORE1_SENSOR is enabled, app_tick() does not read sensors
 * directly but instead calls sample_ring_pop() to drain the SPSC ring
 * filled by core1_entry() in mesh_node.h. See mesh_node.h for the ring type.*/

#endif /* MESH_APP_H */
