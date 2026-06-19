/* =============================================================================
 * app_config.h  --  L4 flood-sensor application configuration.
 * -----------------------------------------------------------------------------
 * Compile-time configuration for the P8 application layer: the flood sensor
 * (ROLE_SENDER) and the base station / gateway (ROLE_SINK). Mirrors the old
 * WiFi-direct firmware, adapted to the LoRa mesh: only the base station has
 * WiFi/MQTT; sensors reach the broker through the mesh via the base station.
 *
 * !! PIN REASSIGNMENT (IMPORTANT) !!
 *   The old firmware put the 5 float sensors on GP16..GP20. Those pins are now
 *   the SX1278 SPI bus (MISO16, CS17, SCK18, MOSI19, RST20) -- they CANNOT be
 *   reused for floats. The floats are moved to GP10..GP14 here; rewire the
 *   hardware accordingly. Relay/buzzer stays on GP15, GPS on GP8/GP9 (all free).
 *   LoRa pins live in mesh_config.h (PIN_*); do not duplicate them here.
 * ===========================================================================*/
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include "mesh_config.h"     /* node_id_t, ROLE_*, NODE_* */
#include "mesh_types.h"

/* ============================ SENSOR HARDWARE ============================= */
/* Relay / buzzer (Active LOW: LOW = ON, HIGH = OFF) -- unchanged from old fw. */
#define APP_RELAY_PIN     15

/* Float sensors (GND-to-GPIO, Active LOW via INPUT_PULLUP). MOVED off the LoRa
 * SPI pins (old GP16..20) to GP10..14. Lowest level first. */
#define APP_FLOAT1_PIN    10
#define APP_FLOAT2_PIN    11
#define APP_FLOAT3_PIN    12
#define APP_FLOAT4_PIN    13
#define APP_FLOAT5_PIN    14

/* GPS NEO-6M on hardware UART1 (Serial2 in the earlephilhower core).
 *   Pico GP8 = TX  -> GPS RX
 *   Pico GP9 = RX  <- GPS TX                                                  */
#define APP_GPS_TX_PIN    8
#define APP_GPS_RX_PIN    9
#define APP_GPS_BAUD      9600

/* Water level that, together with an active CAP alert, triggers the buzzer.   */
#define APP_WATER_THRESHOLD   3

/* Default location used until/unless GPS provides a fix (matches old fw).      */
#define APP_DEFAULT_LAT   27.31f
#define APP_DEFAULT_LON   94.88f

/* ============================ APP TIMING ================================== */
/* Sensor: periodic telemetry heartbeat even if nothing changed, so the base /
 * MQTT view stays fresh (it also sends immediately on any change).            */
#define APP_TELEMETRY_PERIOD_MS   60000UL

/* Base station CAP delivery: send once, then resend ONLY until the sensor
 * returns an end-to-end CAP_ACK for the current version. Exponential backoff
 * from _WAIT_, doubling each attempt, capped at _CAP_, giving up after _MAX_
 * attempts. Once acked: zero traffic. (Reboots are handled by the sensor's
 * boot-time CAP_SYNC request, not by blind periodic resends.)               */
#define APP_CAP_ACK_WAIT_MS       8000UL     /* base: wait for app-ack before resend */
#define APP_CAP_RETRY_CAP_MS      300000UL   /* backoff ceiling (5 min)              */
#define APP_CAP_MAX_ATTEMPTS      6u         /* then stop (await next change/sync)   */

/* Sensor: ms after boot to ask the base to (re)send our CAP set, so a power-
 * cycled node re-syncs its (RAM-only) alert state without a periodic push.    */
#define APP_BOOT_SYNC_MS          5000UL

/* ============================ BASE: WiFi / MQTT =========================== */
/* Credentials (SSID/PASS, MQTT host/user/pass) live in the gitignored
 * secrets.h. Copy secrets.h.example -> secrets.h and fill in your deployment
 * values. Only the ROLE_SINK build uses them.                                 */
#include "secrets.h"          /* APP_WIFI_SSID/PASS, APP_MQTT_HOST/USER/PASS */

#define APP_MQTT_PORT     8883

#define APP_MQTT_STATUS_TOPIC   "rainfall/status"
#define APP_MQTT_CAP_SUB        "cap-alert/#"        /* wildcard: all sensors  */
#define APP_MQTT_FLOOD_SUB      "flood-confirmed/#"
#define APP_MQTT_CAP_PREFIX     "cap-alert/"
#define APP_MQTT_FLOOD_PREFIX   "flood-confirmed/"
#define APP_MQTT_BUFFER_SIZE    2048

#define APP_WIFI_RETRY_MS   10000UL
#define APP_MQTT_RETRY_MS   5000UL

/* ============================ NODE REGISTRY ============================== *
 * Maps mesh node_id <-> MQTT sensor-id string. The base station uses this to
 * (a) label uplink telemetry with the right sensor-id for `rainfall/status`,
 * (b) route a downlink (cap-alert/<id>) to the correct node. ADD A ROW PER
 * DEPLOYED SENSOR. Keep node_ids in sync with the per-node -D NODE_ID build.
 * ========================================================================*/
typedef struct {
    node_id_t   node;
    const char *sensor_id;
} app_node_entry_t;

static const app_node_entry_t APP_NODE_REGISTRY[] = {
    { 0x0001, "pico_lora_sensor_01" },
    /* { 0x0002, "pico_w_flood_sensor_02" }, */
    /* { 0x0003, "pico_w_flood_sensor_03" }, */
};
#define APP_REGISTRY_COUNT  (sizeof(APP_NODE_REGISTRY)/sizeof(APP_NODE_REGISTRY[0]))

/* node_id -> sensor-id string (NULL if unknown). */
static inline const char *app_sensor_id_for_node(node_id_t n){
    for (unsigned i=0;i<APP_REGISTRY_COUNT;i++)
        if (APP_NODE_REGISTRY[i].node==n) return APP_NODE_REGISTRY[i].sensor_id;
    return 0;
}
/* sensor-id string -> node_id (ADDR_INVALID if unknown). */
static inline node_id_t app_node_for_sensor_id(const char *id){
    if (!id) return ADDR_INVALID;
    for (unsigned i=0;i<APP_REGISTRY_COUNT;i++){
        const char *a=APP_NODE_REGISTRY[i].sensor_id; const char *b=id;
        while (*a && *b && *a==*b){ a++; b++; }
        if (*a==0 && *b==0) return APP_NODE_REGISTRY[i].node;
    }
    return ADDR_INVALID;
}

#endif /* APP_CONFIG_H */
