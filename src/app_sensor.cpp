/* =============================================================================
 * app_sensor.cpp  --  L4 flood-sensor node (ROLE_SENDER).
 * -----------------------------------------------------------------------------
 * Port of the old WiFi-direct flood firmware to the LoRa mesh. Sensor reads
 * floats + GPS, computes CAP-active + buzzer exactly as before, but instead of
 * MQTT it (a) sends compact telemetry to the sink over the mesh, and (b) stores
 * CAP alerts pushed down from the base station. WiFi/MQTT live only on the base.
 *
 * Compiled only for non-SINK builds (the whole body is #if-guarded); a SINK
 * build gets an empty translation unit.
 * ===========================================================================*/
#include "mesh_config.h"

#if NODE_ROLE != ROLE_SINK

#include <Arduino.h>
#include <TinyGPS++.h>
#include "app_sensor.h"
#include "app_proto.h"
#include "app_config.h"
#include "mesh_l3.h"
#include "node_identity.h"

/* ---- peripherals -------------------------------------------------------- */
#define gpsSerial Serial2
static TinyGPSPlus gps;

/* ---- location / fix ----------------------------------------------------- */
static float    s_lat = APP_DEFAULT_LAT;
static float    s_lon = APP_DEFAULT_LON;
static bool     s_gps_fix = false;
static bool     s_last_gps_fix = false;

/* ---- CAP alert storage (replace-set from the base) ---------------------- */
static cap_entry_t s_caps[APP_MAX_CAP_ALERTS];
static uint8_t     s_cap_count = 0;
static uint8_t     s_cap_version = 0;       /* last applied set version        */
static bool        s_cap_active = false;
static bool        s_last_cap_active = false;

/* ---- boot CAP re-sync (recover RAM-only alert state after a reboot) ----- */
static uint32_t    s_boot_sync_at = 0;
static bool        s_boot_sync_done = false;

/* ---- time sync (downlink current_time > GPS > millis) ------------------- */
static uint32_t s_backend_epoch = 0, s_backend_ms = 0;
static uint32_t s_gps_epoch = 0,     s_gps_ms = 0;
static bool     s_gps_time_ok = false;

/* ---- buzzer + telemetry cadence ----------------------------------------- */
static bool     s_buzzer = false;
static int      s_prev_level = -1;
static bool     s_prev_cap_active = false;
static uint32_t s_next_tlm_ms = 0;

/* ---- time helpers (mirror old firmware) -------------------------------- */
static uint32_t unix_time(int Y,int M,int D,int h,int m,int s){
    static const int dim[]={0,31,28,31,30,31,30,31,31,30,31,30,31};
    uint32_t days=0;
    for (int y=1970;y<Y;y++){ days+=365; if((y%4==0&&y%100!=0)||(y%400==0)) days++; }
    for (int mo=1;mo<M;mo++){ days+=dim[mo]; if(mo==2&&((Y%4==0&&Y%100!=0)||(Y%400==0))) days++; }
    days+=(D-1);
    return days*86400UL + (uint32_t)h*3600UL + (uint32_t)m*60UL + (uint32_t)s;
}
static uint32_t app_now_epoch(void){
    if (s_backend_epoch) return s_backend_epoch + (millis()-s_backend_ms)/1000UL;
    if (s_gps_time_ok)   return s_gps_epoch + (millis()-s_gps_ms)/1000UL;
    return millis()/1000UL;
}

/* ---- sensors ------------------------------------------------------------ */
static void set_buzzer(bool on){
    if (on==s_buzzer) return;
    digitalWrite(APP_RELAY_PIN, on ? LOW : HIGH);   /* active LOW */
    s_buzzer=on;
    Serial.print("[BUZZER] "); Serial.println(on?"ON":"OFF");
}
static int read_water_level(void){
    if (digitalRead(APP_FLOAT5_PIN)==LOW) return 5;
    if (digitalRead(APP_FLOAT4_PIN)==LOW) return 4;
    if (digitalRead(APP_FLOAT3_PIN)==LOW) return 3;
    if (digitalRead(APP_FLOAT2_PIN)==LOW) return 2;
    if (digitalRead(APP_FLOAT1_PIN)==LOW) return 1;
    return 0;
}
static void read_gps(void){
    while (gpsSerial.available()>0) gps.encode(gpsSerial.read());
    if (gps.location.isValid()){
        s_lat=gps.location.lat(); s_lon=gps.location.lng();
        s_gps_fix=(gps.location.age()<10000);
    } else s_gps_fix=false;
    if (s_gps_fix && !s_last_gps_fix) Serial.println("[GPS] fix acquired");
    s_last_gps_fix=s_gps_fix;
    if (gps.date.isValid() && gps.time.isValid() && gps.date.year()>=2026
        && gps.date.age()<2000 && gps.time.age()<2000){
        s_gps_epoch=unix_time(gps.date.year(),gps.date.month(),gps.date.day(),
                              gps.time.hour(),gps.time.minute(),gps.time.second());
        s_gps_ms=millis(); s_gps_time_ok=true;
    }
}
static bool cap_active_now(void){
    uint32_t t=app_now_epoch();
    for (uint8_t i=0;i<s_cap_count;i++)
        if (t>=s_caps[i].start_epoch && t<=s_caps[i].end_epoch) return true;
    return false;
}

/* ---- telemetry send ----------------------------------------------------- */
static void send_telemetry(int water_level){
    flood_telemetry_t t;
    t.water_level=(uint8_t)water_level;
    t.gps_fix=s_gps_fix;
    t.cap_alert=s_cap_active;
    t.lat_e6=(int32_t)(s_lat*1e6f);
    t.lon_e6=(int32_t)(s_lon*1e6f);
    uint8_t buf[APP_TELEMETRY_LEN];
    uint8_t n=app_serialize_telemetry(&t,buf,sizeof(buf));
    if (!n) return;
    seq_t seq=0;
    l3_result_t rc=mesh_l3_send(node_sink_addr(), buf, n, &seq);
    Serial.print("[TLM] wl="); Serial.print(water_level);
    Serial.print(" cap="); Serial.print(s_cap_active?1:0);
    Serial.print(" -> sink rc="); Serial.println((int)rc);
}

/* End-to-end ack back to the base confirming we applied set `version`. */
static void send_cap_ack(node_id_t to, uint8_t version){
    uint8_t buf[APP_CAP_ACK_LEN];
    buf[0]=(uint8_t)APP_PORT_CAP_ACK; buf[1]=version;
    seq_t seq=0;
    mesh_l3_send(to, buf, sizeof(buf), &seq);
    Serial.print("[CAP] ack v="); Serial.print(version);
    Serial.print(" -> 0x"); Serial.println(to,HEX);
}

/* Ask the base to (re)send our current CAP set (used once after boot). */
static void send_cap_sync(void){
    uint8_t buf[APP_CAP_SYNC_LEN];
    buf[0]=(uint8_t)APP_PORT_CAP_SYNC;
    seq_t seq=0;
    mesh_l3_send(node_sink_addr(), buf, sizeof(buf), &seq);
    Serial.println("[CAP] boot sync request -> sink");
}

/* ========================================================================= */
void app_sensor_init(void){
    pinMode(APP_RELAY_PIN, OUTPUT);
    digitalWrite(APP_RELAY_PIN, HIGH);              /* off (active LOW) */
    pinMode(APP_FLOAT1_PIN, INPUT_PULLUP);
    pinMode(APP_FLOAT2_PIN, INPUT_PULLUP);
    pinMode(APP_FLOAT3_PIN, INPUT_PULLUP);
    pinMode(APP_FLOAT4_PIN, INPUT_PULLUP);
    pinMode(APP_FLOAT5_PIN, INPUT_PULLUP);
    gpsSerial.setTX(APP_GPS_TX_PIN);
    gpsSerial.setRX(APP_GPS_RX_PIN);
    gpsSerial.begin(APP_GPS_BAUD);
    s_next_tlm_ms = millis() + 3000UL;             /* first telemetry shortly */
    s_boot_sync_at = millis() + APP_BOOT_SYNC_MS;  /* re-pull CAP set after boot */
    s_boot_sync_done = false;
    Serial.println("[APP] flood sensor initialized");
}

void app_sensor_tick(uint32_t now_ms){
    /* one-shot CAP re-sync request shortly after boot */
    if (!s_boot_sync_done && (int32_t)(now_ms - s_boot_sync_at) >= 0){
        send_cap_sync();
        s_boot_sync_done = true;
    }

    read_gps();
    int level = read_water_level();
    s_cap_active = cap_active_now();

    if (s_cap_active != s_last_cap_active){
        Serial.print("[CAP] active="); Serial.println(s_cap_active?"YES":"no");
        s_last_cap_active=s_cap_active;
    }
    set_buzzer((level>=APP_WATER_THRESHOLD) && s_cap_active);

    bool changed = (level != s_prev_level) || (s_cap_active != s_prev_cap_active);
    bool due     = (int32_t)(now_ms - s_next_tlm_ms) >= 0;
    if (changed || due){
        if (changed && level!=s_prev_level){
            Serial.print("[WL] "); Serial.print(s_prev_level);
            Serial.print(" -> "); Serial.println(level);
        }
        send_telemetry(level);
        s_prev_level=level; s_prev_cap_active=s_cap_active;
        s_next_tlm_ms = now_ms + APP_TELEMETRY_PERIOD_MS;
    }
}

void app_sensor_on_downlink(node_id_t from, const uint8_t *payload, uint8_t len){
    (void)from;
    if (len==0 || payload[0]!=APP_PORT_CAP) return;   /* not a CAP downlink   */
    cap_downlink_t dl;
    if (!app_deserialize_cap(payload,len,&dl)) return;

    if (dl.current_time){ s_backend_epoch=dl.current_time; s_backend_ms=millis(); }
    /* replace-set semantics, matching the old MQTT handler */
    s_cap_count = dl.count;
    for (uint8_t i=0;i<dl.count;i++) s_caps[i]=dl.alerts[i];
    s_cap_version = dl.version;

    Serial.print("[CAP] downlink sub="); Serial.print(dl.subtype);
    Serial.print(" v="); Serial.print(dl.version);
    Serial.print(" count="); Serial.print(dl.count);
    Serial.print(" time="); Serial.println((unsigned long)dl.current_time);
    /* recompute immediately so the buzzer reacts without waiting for tick */
    s_cap_active = cap_active_now();
    /* confirm receipt end-to-end so the base stops resending this version */
    send_cap_ack(from, dl.version);
}

#endif /* NODE_ROLE != ROLE_SINK */
