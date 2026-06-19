/* =============================================================================
 * app_basestation.cpp  --  L4 base station / gateway (ROLE_SINK).
 * -----------------------------------------------------------------------------
 * The single WiFi/MQTT node. It is the mesh's bridge to the broker:
 *   - UPLINK:   decode mesh telemetry -> publish JSON on rainfall/status
 *               { sensor_id, latitude, longitude, level }.
 *   - DOWNLINK: subscribe cap-alert/# and flood-confirmed/#, translate the
 *               verbose JSON to a compact binary CAP frame, and route it over
 *               the mesh to the matching sensor node (per the node registry),
 *               re-sending each node's current set every APP_CAP_RESEND_MS.
 *
 * Compiled only for SINK builds (whole body #if-guarded).
 *
 * CAVEAT: WiFi + mesh share core0. A TLS connect briefly blocks the mesh pump;
 * acceptable for an always-on mains/solar gateway. Move WiFi to core1 later if
 * RX gaps during (re)connect become a problem.
 * ===========================================================================*/
#include "mesh_config.h"

#if NODE_ROLE == ROLE_SINK

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "app_basestation.h"
#include "app_proto.h"
#include "app_config.h"
#include "mesh_l3.h"
#include "node_identity.h"

/* ---- network clients ---------------------------------------------------- */
static WiFiClientSecure s_wifi;
static PubSubClient     s_mqtt(s_wifi);

/* ---- base clock (from MQTT current_time) ------------------------------- */
static uint32_t s_btime=0, s_btime_ms=0;
static uint32_t base_now(void){
    return s_btime ? s_btime + (millis()-s_btime_ms)/1000UL : millis()/1000UL;
}

/* ---- per-sensor downlink state (parallel to APP_NODE_REGISTRY) ----------
 * Reliable, ack-driven delivery: send once, resend (backoff) ONLY until the
 * sensor returns a CAP_ACK for the current `version`; silent once acked. */
typedef struct {
    bool        valid;       /* we have a set to deliver to this node          */
    bool        acked;       /* sensor confirmed the current version           */
    uint8_t     version;     /* monotonically bumped per update                */
    uint8_t     attempts;    /* sends issued since last version bump            */
    uint32_t    nextTryMs;   /* next resend deadline (backoff)                 */
    cap_downlink_t dl;
} dlstate_t;
static dlstate_t s_dl[APP_REGISTRY_COUNT ? APP_REGISTRY_COUNT : 1];

static int registry_index(node_id_t node){
    for (unsigned i=0;i<APP_REGISTRY_COUNT;i++) if (APP_NODE_REGISTRY[i].node==node) return (int)i;
    return -1;
}

/* ---- reconnect timers --------------------------------------------------- */
static uint32_t s_wifi_at=0, s_mqtt_at=0;
static bool s_wifi_logged=false, s_mqtt_logged=false;

/* ---- ISO8601 -> epoch (ported from old firmware) ----------------------- */
static uint32_t unix_time(int Y,int M,int D,int h,int m,int s){
    static const int dim[]={0,31,28,31,30,31,30,31,31,30,31,30,31};
    uint32_t days=0;
    for (int y=1970;y<Y;y++){ days+=365; if((y%4==0&&y%100!=0)||(y%400==0)) days++; }
    for (int mo=1;mo<M;mo++){ days+=dim[mo]; if(mo==2&&((Y%4==0&&Y%100!=0)||(Y%400==0))) days++; }
    days+=(D-1);
    return days*86400UL + (uint32_t)h*3600UL + (uint32_t)m*60UL + (uint32_t)s;
}
static uint32_t iso8601_epoch(const String &s){
    if (s.length()<19) return 0;
    int Y=s.substring(0,4).toInt(), M=s.substring(5,7).toInt(), D=s.substring(8,10).toInt();
    int h=s.substring(11,13).toInt(), mn=s.substring(14,16).toInt(), se=s.substring(17,19).toInt();
    uint32_t e=unix_time(Y,M,D,h,mn,se);
    int i=19;
    if (i<(int)s.length() && s[i]=='.'){ i++; while(i<(int)s.length() && isDigit(s[i])) i++; }
    if (i<(int)s.length()){
        char sign=s[i];
        if (sign=='+'||sign=='-'){
            String off=s.substring(i+1); int c=off.indexOf(':'); int oh=0,om=0;
            if (c!=-1){ oh=off.substring(0,c).toInt(); om=off.substring(c+1).toInt(); }
            else oh=off.toInt();
            uint32_t os=(uint32_t)(oh*3600+om*60);
            if (sign=='+') e-=os; else e+=os;
        }
    }
    return e;
}

/* ---- downlink: send a node's current CAP set over the mesh -------------- */
static void send_node_downlink(unsigned idx){
    if (idx>=APP_REGISTRY_COUNT || !s_dl[idx].valid) return;
    s_dl[idx].dl.current_time = base_now();
    s_dl[idx].dl.version      = s_dl[idx].version;
    uint8_t buf[MAX_PAYLOAD];
    uint8_t n=app_serialize_cap(&s_dl[idx].dl, buf, sizeof(buf));
    if (!n) return;
    seq_t seq=0;
    node_id_t node=APP_NODE_REGISTRY[idx].node;
    l3_result_t rc=mesh_l3_send(node, buf, n, &seq);

    s_dl[idx].attempts++;
    uint32_t backoff = APP_CAP_ACK_WAIT_MS << (s_dl[idx].attempts-1);  /* 8s,16s,32s.. */
    if (backoff > APP_CAP_RETRY_CAP_MS) backoff = APP_CAP_RETRY_CAP_MS;
    s_dl[idx].nextTryMs = millis() + backoff;

    Serial.print("[DL] -> 0x"); Serial.print(node,HEX);
    Serial.print(" v="); Serial.print(s_dl[idx].version);
    Serial.print(" try="); Serial.print(s_dl[idx].attempts);
    Serial.print(" count="); Serial.print(s_dl[idx].dl.count);
    Serial.print(" rc="); Serial.println((int)rc);
}

/* ---- MQTT inbound: cap-alert/<id> or flood-confirmed/<id> --------------- */
static void mqtt_cb(char *topic, byte *payload, unsigned int length){
    String t(topic);
    uint8_t sub;
    const char *id=0;
    if (t.startsWith(APP_MQTT_CAP_PREFIX))        { sub=APP_CAP_SUB_ALERT; id=topic+strlen(APP_MQTT_CAP_PREFIX); }
    else if (t.startsWith(APP_MQTT_FLOOD_PREFIX)) { sub=APP_CAP_SUB_FLOOD; id=topic+strlen(APP_MQTT_FLOOD_PREFIX); }
    else return;

    node_id_t node=app_node_for_sensor_id(id);
    if (node==ADDR_INVALID){ Serial.print("[DL] unknown sensor-id: "); Serial.println(id); return; }

    String msg; msg.reserve(length+1);
    for (unsigned i=0;i<length;i++) msg+=(char)payload[i];

    JsonDocument doc;
    if (deserializeJson(doc, msg)){ Serial.println("[DL] JSON parse failed"); return; }

    if (doc["current_time"].is<const char*>()){
        uint32_t e=iso8601_epoch(doc["current_time"].as<String>());
        if (e){ s_btime=e; s_btime_ms=millis(); }
    }

    cap_downlink_t dl; dl.subtype=sub; dl.current_time=base_now(); dl.count=0;
    if (doc["alerts"].is<JsonArray>()){
        for (JsonObject a : doc["alerts"].as<JsonArray>()){
            if (dl.count>=APP_MAX_CAP_ALERTS) break;
            uint32_t h=app_fnv1a(a["cap_alert_id"].as<const char*>());
            bool dup=false;
            for (uint8_t i=0;i<dl.count;i++) if (dl.alerts[i].id_hash==h) dup=true;
            if (dup) continue;
            dl.alerts[dl.count].id_hash=h;
            dl.alerts[dl.count].start_epoch=iso8601_epoch(a["effective_start"].as<String>());
            dl.alerts[dl.count].end_epoch  =iso8601_epoch(a["effective_end"].as<String>());
            dl.count++;
        }
    }

    /* store the replace-set, bump version, arm ack-driven delivery, send now. */
    int i=registry_index(node);
    if (i<0) return;
    s_dl[i].valid=true; s_dl[i].acked=false; s_dl[i].attempts=0;
    s_dl[i].version++; s_dl[i].dl=dl;
    Serial.print("[DL] MQTT "); Serial.print(topic);
    Serial.print(" -> node 0x"); Serial.print(node,HEX);
    Serial.print(" alerts="); Serial.println(dl.count);
    send_node_downlink((unsigned)i);
}

/* ---- WiFi / MQTT keepalive (non-blocking) ------------------------------ */
static void handle_wifi(uint32_t now){
    if (WiFi.status()==WL_CONNECTED){
        if (!s_wifi_logged){ Serial.println("[WiFi] connected"); s_wifi_logged=true; }
        return;
    }
    s_wifi_logged=false;
    if (now - s_wifi_at >= APP_WIFI_RETRY_MS){
        s_wifi_at=now;
        Serial.println("[WiFi] connecting...");
        WiFi.begin(APP_WIFI_SSID, APP_WIFI_PASS);
    }
}
static void handle_mqtt(uint32_t now){
    if (WiFi.status()!=WL_CONNECTED){ if (s_mqtt.connected()) s_mqtt.disconnect(); return; }
    if (s_mqtt.connected()){
        if (!s_mqtt_logged){
            Serial.println("[MQTT] connected");
            s_mqtt_logged=true;
            s_mqtt.subscribe(APP_MQTT_CAP_SUB);
            s_mqtt.subscribe(APP_MQTT_FLOOD_SUB);
        }
        s_mqtt.loop();
        return;
    }
    s_mqtt_logged=false;
    if (now - s_mqtt_at >= APP_MQTT_RETRY_MS){
        s_mqtt_at=now;
        Serial.println("[MQTT] connecting...");
        String cid=String("base_")+String(node_id(),HEX)+"_"+String(random(1000,9999));
        if (!s_mqtt.connect(cid.c_str(), APP_MQTT_USER, APP_MQTT_PASS)){
            Serial.print("[MQTT] connect failed, state="); Serial.println(s_mqtt.state());
        }
    }
}

/* ========================================================================= */
void app_base_init(void){
    for (unsigned i=0;i<APP_REGISTRY_COUNT;i++) s_dl[i].valid=false;
    WiFi.mode(WIFI_STA);
    WiFi.begin(APP_WIFI_SSID, APP_WIFI_PASS);
    s_wifi_at=millis();
    s_wifi.setInsecure();
    s_mqtt.setServer(APP_MQTT_HOST, APP_MQTT_PORT);
    s_mqtt.setCallback(mqtt_cb);
    s_mqtt.setBufferSize(APP_MQTT_BUFFER_SIZE);
    Serial.println("[APP] base station initialized");
}

void app_base_tick(uint32_t now_ms){
    handle_wifi(now_ms);
    handle_mqtt(now_ms);
    /* resend an unacked CAP set on backoff, until acked or attempts exhausted */
    for (unsigned i=0;i<APP_REGISTRY_COUNT;i++){
        dlstate_t *d=&s_dl[i];
        if (!d->valid || d->acked) continue;
        if (d->attempts >= APP_CAP_MAX_ATTEMPTS) continue;   /* gave up; await sync/change */
        if ((int32_t)(now_ms - d->nextTryMs) >= 0) send_node_downlink(i);
    }
}

void app_base_on_uplink(node_id_t from, const uint8_t *payload, uint8_t len){
    if (len==0) return;

    /* End-to-end CAP ack: sensor confirms it applied version V -> stop resending. */
    if (payload[0]==APP_PORT_CAP_ACK){
        if (len < APP_CAP_ACK_LEN) return;
        int i=registry_index(from);
        if (i>=0 && s_dl[i].valid && payload[1]==s_dl[i].version){
            s_dl[i].acked=true;
            Serial.print("[DL] ack from 0x"); Serial.print(from,HEX);
            Serial.print(" v="); Serial.println((int)payload[1]);
        }
        return;
    }

    /* Boot re-sync request: re-arm delivery of this node's current set. */
    if (payload[0]==APP_PORT_CAP_SYNC){
        int i=registry_index(from);
        if (i>=0 && s_dl[i].valid){
            s_dl[i].acked=false; s_dl[i].attempts=0;
            Serial.print("[DL] sync request from 0x"); Serial.println(from,HEX);
            send_node_downlink((unsigned)i);
        }
        return;
    }

    if (payload[0]!=APP_PORT_TELEMETRY) return;
    flood_telemetry_t t;
    if (!app_deserialize_telemetry(payload,len,&t)) return;

    const char *sid=app_sensor_id_for_node(from);
    String idStr = sid ? String(sid) : (String("node_0x")+String(from,HEX));

    JsonDocument doc;
    doc["sensor_id"]=idStr;
    doc["latitude"] =serialized(String((double)t.lat_e6/1e6, 6));
    doc["longitude"]=serialized(String((double)t.lon_e6/1e6, 6));
    doc["level"]=t.water_level;

    String out; serializeJson(doc, out);
    if (s_mqtt.connected() && s_mqtt.publish(APP_MQTT_STATUS_TOPIC, out.c_str()))
        Serial.print("[UP] published ");
    else
        Serial.print("[UP] (mqtt down) ");
    Serial.println(out);
}

#endif /* NODE_ROLE == ROLE_SINK */
