/* =============================================================================
 * app_proto.cpp  --  L4 application wire codec (telemetry + CAP). Pure logic.
 * ===========================================================================*/
#include "app_proto.h"
#include <string.h>

/* ---- little-endian field helpers ---------------------------------------- */
static inline void p8 (uint8_t *b, uint8_t *o, uint8_t  v){ b[(*o)++]=v; }
static inline void p16(uint8_t *b, uint8_t *o, uint16_t v){ b[(*o)++]=(uint8_t)v; b[(*o)++]=(uint8_t)(v>>8); }
static inline void p32(uint8_t *b, uint8_t *o, uint32_t v){ for(int i=0;i<4;i++) b[(*o)++]=(uint8_t)(v>>(8*i)); }
static inline uint8_t  g8 (const uint8_t *b, uint8_t *o){ return b[(*o)++]; }
static inline uint32_t g32(const uint8_t *b, uint8_t *o){ uint32_t v=0; for(int i=0;i<4;i++) v|=((uint32_t)b[(*o)+i])<<(8*i); *o+=4; return v; }

uint32_t app_fnv1a(const char *s){
    uint32_t h=2166136261u;
    if (s) while (*s){ h ^= (uint8_t)(*s++); h *= 16777619u; }
    return h;
}

/* ---- TELEMETRY ---------------------------------------------------------- */
uint8_t app_serialize_telemetry(const flood_telemetry_t *t, uint8_t *out, uint8_t cap){
    if (!t || !out || cap < APP_TELEMETRY_LEN) return 0;
    uint8_t o=0;
    p8(out,&o,(uint8_t)APP_PORT_TELEMETRY);
    uint8_t flags=0;
    if (t->gps_fix)   flags |= APP_TLM_FLAG_GPSFIX;
    if (t->cap_alert) flags |= APP_TLM_FLAG_CAPACTIVE;
    p8(out,&o,flags);
    p8(out,&o,t->water_level);
    p32(out,&o,(uint32_t)t->lat_e6);
    p32(out,&o,(uint32_t)t->lon_e6);
    return o;   /* == APP_TELEMETRY_LEN */
}
bool app_deserialize_telemetry(const uint8_t *in, uint8_t len, flood_telemetry_t *out){
    if (!in || !out || len < APP_TELEMETRY_LEN) return false;
    uint8_t o=0;
    if (g8(in,&o) != APP_PORT_TELEMETRY) return false;
    uint8_t flags=g8(in,&o);
    out->gps_fix   = (flags & APP_TLM_FLAG_GPSFIX)   != 0;
    out->cap_alert = (flags & APP_TLM_FLAG_CAPACTIVE)!= 0;
    out->water_level = g8(in,&o);
    out->lat_e6 = (int32_t)g32(in,&o);
    out->lon_e6 = (int32_t)g32(in,&o);
    return true;
}

/* ---- CAP DOWNLINK ------------------------------------------------------- */
uint8_t app_serialize_cap(const cap_downlink_t *d, uint8_t *out, uint8_t cap){
    if (!d || !out) return 0;
    uint8_t n = d->count; if (n > APP_MAX_CAP_ALERTS) n = APP_MAX_CAP_ALERTS;
    uint8_t need = (uint8_t)(APP_CAP_HDR_LEN + n*APP_CAP_ENTRY_LEN);
    if (cap < need) return 0;
    uint8_t o=0;
    p8(out,&o,(uint8_t)APP_PORT_CAP);
    p8(out,&o,d->subtype);
    p8(out,&o,d->version);
    p32(out,&o,d->current_time);
    p8(out,&o,n);
    for (uint8_t i=0;i<n;i++){
        p32(out,&o,d->alerts[i].id_hash);
        p32(out,&o,d->alerts[i].start_epoch);
        p32(out,&o,d->alerts[i].end_epoch);
    }
    return o;
}
bool app_deserialize_cap(const uint8_t *in, uint8_t len, cap_downlink_t *out){
    if (!in || !out || len < APP_CAP_HDR_LEN) return false;
    uint8_t o=0;
    if (g8(in,&o) != APP_PORT_CAP) return false;
    out->subtype      = g8(in,&o);
    out->version      = g8(in,&o);
    out->current_time = g32(in,&o);
    uint8_t n = g8(in,&o);
    if (n > APP_MAX_CAP_ALERTS) n = APP_MAX_CAP_ALERTS;
    if (len < (uint8_t)(APP_CAP_HDR_LEN + n*APP_CAP_ENTRY_LEN)) return false;
    out->count = n;
    for (uint8_t i=0;i<n;i++){
        out->alerts[i].id_hash     = g32(in,&o);
        out->alerts[i].start_epoch = g32(in,&o);
        out->alerts[i].end_epoch   = g32(in,&o);
    }
    return true;
}
