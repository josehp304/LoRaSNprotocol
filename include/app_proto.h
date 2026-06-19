/* =============================================================================
 * app_proto.h  --  L4 application wire protocol (compact, fits MAX_PAYLOAD).
 * -----------------------------------------------------------------------------
 * Pure logic, shared by the sensor (ROLE_SENDER) and the base station
 * (ROLE_SINK). Two message types ride inside a mesh DATA payload, demuxed by a
 * 1-byte app_port at offset 0:
 *
 *   UPLINK  TELEMETRY (sensor -> base):  flood readings + position.
 *   DOWNLINK CAP       (base -> sensor): replace-set of active CAP alerts.
 *
 * The verbose MQTT CAP JSON (string ids + ISO8601) does NOT fit a 64-byte
 * frame, so the base station translates it to this compact binary form:
 * alert-id String -> uint32 FNV-1a hash, ISO8601 -> uint32 epoch seconds.
 * All multi-byte fields little-endian, serialized field-by-field (no struct
 * memcpy), consistent with the rest of the stack.
 * ===========================================================================*/
#ifndef APP_PROTO_H
#define APP_PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_config.h"   /* MAX_PAYLOAD */

/* app_port (DATA payload byte 0) */
#define APP_PORT_TELEMETRY  1u   /* sensor -> base : flood readings           */
#define APP_PORT_CAP        2u   /* base -> sensor : CAP replace-set          */
#define APP_PORT_CAP_ACK    3u   /* sensor -> base : "applied set version V"  */
#define APP_PORT_CAP_SYNC   4u   /* sensor -> base : "(re)send my CAP set"    */

/* CAP subtype (downlink byte 2) */
#define APP_CAP_SUB_ALERT   0u   /* cap-alert/<id>        */
#define APP_CAP_SUB_FLOOD   1u   /* flood-confirmed/<id>  */

/* Max CAP alerts carried in (and stored from) one replace-set frame.
 * Budget: 1(port)+1(sub)+1(ver)+4(time)+1(count) = 8, + N*12 <= MAX_PAYLOAD(64).
 *   N <= (64-8)/12 = 4. */
#define APP_MAX_CAP_ALERTS  4u

/* Fixed wire lengths. */
#define APP_TELEMETRY_LEN   11u   /* port+flags+water+lat4+lon4   */
#define APP_CAP_HDR_LEN     8u    /* port+sub+ver+time4+count     */
#define APP_CAP_ENTRY_LEN   12u   /* idhash4+start4+end4          */
#define APP_CAP_ACK_LEN     2u    /* port+version                 */
#define APP_CAP_SYNC_LEN    1u    /* port                         */

/* Telemetry flag bits. */
#define APP_TLM_FLAG_GPSFIX     0x01u
#define APP_TLM_FLAG_CAPACTIVE  0x02u

/* ---- decoded structures (host order) ----------------------------------- */
typedef struct {
    uint8_t  water_level;   /* 0..5                                          */
    bool     gps_fix;
    bool     cap_alert;     /* sender's current cap-active view              */
    int32_t  lat_e6;        /* degrees * 1e6                                 */
    int32_t  lon_e6;
} flood_telemetry_t;

typedef struct {
    uint32_t id_hash;       /* FNV-1a of the cap_alert_id string             */
    uint32_t start_epoch;   /* effective_start (UTC seconds)                 */
    uint32_t end_epoch;     /* effective_end   (UTC seconds)                 */
} cap_entry_t;

typedef struct {
    uint8_t    subtype;     /* APP_CAP_SUB_*                                 */
    uint8_t    version;     /* per-node set version; echoed in CAP_ACK       */
    uint32_t   current_time;/* base's epoch at send (UTC seconds)           */
    uint8_t    count;       /* valid entries in alerts[]                     */
    cap_entry_t alerts[APP_MAX_CAP_ALERTS];
} cap_downlink_t;

/* ---- codec (return bytes written, or 0 on error; bool on parse) -------- */
uint8_t app_serialize_telemetry(const flood_telemetry_t *t, uint8_t *out, uint8_t cap);
bool    app_deserialize_telemetry(const uint8_t *in, uint8_t len, flood_telemetry_t *out);

uint8_t app_serialize_cap(const cap_downlink_t *d, uint8_t *out, uint8_t cap);
bool    app_deserialize_cap(const uint8_t *in, uint8_t len, cap_downlink_t *out);

/* FNV-1a 32-bit hash of a NUL-terminated string (alert-id compaction). */
uint32_t app_fnv1a(const char *s);

#endif /* APP_PROTO_H */
