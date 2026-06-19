/* =============================================================================
 * mesh_node.cpp  --  Top-level node facade + L2<->L3 integration glue.
 * -----------------------------------------------------------------------------
 * Implements the entry points declared in mesh_node.h. This is the ONLY place
 * the L2 (datalink) and L3 (mesh_l3) type worlds meet; the hook adapters here
 * translate between l3_tx_frame_t/link_tx_req_t and l3_rx_frame_t/mesh_rx_frame_t
 * with no mesh logic of their own.
 *
 * SCOPE (current phase: P3-P6 bring-up):
 *   node_setup() initializes L1 (PHY) + L2 (datalink) + L3 (mesh_l3).
 *   node_loop()  pumps L2 (link_poll) and L3 (mesh_l3_tick) each iteration.
 * The L4 app layer, power manager, and cooperative scheduler (mesh_app.h,
 * power_mgr.h, scheduler.h) are later phases (P7/P8); their headers are pulled
 * in via mesh_node.h but their functions are intentionally not called yet. The
 * l4_deliver / l4_send_result hooks below print to Serial for bring-up; replace
 * them with app_on_deliver() once L4 lands.
 * ===========================================================================*/
#include <Arduino.h>
#include <string.h>
#include "mesh_node.h"
#include "app_proto.h"
#include "app_sensor.h"
#include "app_basestation.h"

/* ---- handle -> (nextHop,seq) map so L2 tx-completions map to L3 events ---- */
#define TXMAP_SIZE 8u   /* must be >= LINK_TX_QUEUE_SIZE in datalink.cpp */
static struct { bool used; node_id_t nextHop; seq_t seq; } s_txmap[TXMAP_SIZE];

/* ---- injectable randomness for L3 (xorshift) ---------------------------- */
static uint32_t s_rng = 0xC0FFEE11UL;
static uint32_t l3_rand(void){ uint32_t x=s_rng; x^=x<<13; x^=x>>17; x^=x<<5; s_rng=x?x:1; return s_rng; }
static uint32_t l3_now(void){ return millis(); }

/* ---- DOWN: L3 asks us to transmit -------------------------------------- */
static bool hook_l2_send(const l3_tx_frame_t *f){
    link_tx_req_t req;
    req.src=f->src; req.dst=f->dst; req.next_hop=f->nextHop;
    req.pkt_type=(pkt_type_t)f->type; req.ttl=f->ttl; req.hop_count=f->hopCount;
    req.seq=f->seqNum; req.require_ack=f->wantsAck;
    req.payload_len=f->payloadLen;
    if (f->payloadLen) memcpy(req.payload, f->payload, f->payloadLen);

    link_tx_handle_t h=link_send(&req);
    if (h==LINK_TX_HANDLE_INVALID) return false;
    if (h<TXMAP_SIZE){ s_txmap[h].used=true; s_txmap[h].nextHop=f->nextHop; s_txmap[h].seq=f->seqNum; }
    return true;
}

/* ---- UP: L4 delivery / send-result (bring-up: print; replace with app) -- */
static void hook_l4_deliver(node_id_t from, seq_t seq, const uint8_t *pl, uint8_t len){
    (void)seq;
    /* Dispatch a delivered DATA payload to the role's app layer:
     *   SINK  receives telemetry from sensors; others receive the CAP downlink. */
#if NODE_ROLE == ROLE_SINK
    app_base_on_uplink(from, pl, len);
#else
    app_sensor_on_downlink(from, pl, len);
#endif
}
static void hook_l4_send_result(node_id_t dest, seq_t seq, bool ok){
    Serial.print("[L4 TX] dest=0x"); Serial.print(dest,HEX);
    Serial.print(" seq="); Serial.print((unsigned)seq);
    Serial.println(ok ? " -> DELIVERED(next hop)" : " -> FAILED");
}

/* ---- UP: L2 hands us a validated inbound frame -------------------------- */
static void on_link_rx(const mesh_rx_frame_t *frame){
    l3_rx_frame_t f;
    f.src=frame->hdr.src; f.dst=frame->hdr.dst; f.nextHop=frame->hdr.next_hop;
    f.linkFrom=frame->hdr.link_src;          /* the §0 one-hop-sender field    */
    f.hopCount=frame->hdr.hop_count; f.ttl=frame->hdr.ttl; f.seqNum=frame->hdr.seq;
    f.type=(mesh_pkttype_t)frame->hdr.pkt_type;
    f.rssi=frame->rssi_dbm; f.snr=frame->snr_db;
    f.payload=frame->payload; f.payloadLen=frame->hdr.payload_len;
    mesh_l3_on_frame(&f);
}

/* ---- UP: L2 reports a transmit outcome --------------------------------- */
static void on_link_tx_done(link_tx_handle_t h, link_tx_state_t st){
    if (h>=TXMAP_SIZE || !s_txmap[h].used) return;
    node_id_t nh=s_txmap[h].nextHop; seq_t seq=s_txmap[h].seq;
    s_txmap[h].used=false;
    switch (st){
        case LINK_TX_SENT:    mesh_l3_on_link_event(L3_LINK_TX_DONE,   nh, seq); break;
        case LINK_TX_NO_ACK:  mesh_l3_on_link_event(L3_LINK_TX_FAILED, nh, seq); break;
        default: /* BUSY/DUTY/DROPPED: treat as a transient link failure */
                              mesh_l3_on_link_event(L3_LINK_TX_FAILED, nh, seq); break;
    }
}

/* ---- halt helper (unrecoverable startup failure) ----------------------- */
static void node_halt(const char *why, int code){
    uint32_t last=0;
    for (;;){
        uint32_t now=millis();
        if (now-last >= 1000){
            last=now;
            Serial.print("[HALT] "); Serial.print(why);
            Serial.print(" code="); Serial.print(code);
            Serial.print(" radio_err="); Serial.println((int)phy_last_radio_err());
        }
    }
}

/* ===========================================================================
 * PUBLIC ENTRY POINTS
 * ==========================================================================*/
void node_setup(void){
    s_rng ^= (uint32_t)micros() ^ ((uint32_t)node_id()<<16);
    memset(s_txmap, 0, sizeof(s_txmap));

    /* L1 + L2: link_begin() brings up the PHY and starts receiving. */
    link_callbacks_t cb; cb.on_rx=on_link_rx; cb.on_tx_done=on_link_tx_done;
    mesh_err_t rc = link_begin(node_id(), NULL, &cb);
    if (rc != MESH_OK) node_halt("link_begin failed", (int)rc);

    /* L3: AODV routing + neighbor discovery. */
    mesh_l3_hooks_t hk;
    hk.l2_send=hook_l2_send;
    hk.l4_deliver=hook_l4_deliver;
    hk.l4_send_result=hook_l4_send_result;
    hk.now_ms=l3_now;
    hk.rand_u32=l3_rand;
    if (mesh_l3_init(node_id(), &hk) != L3_OK) node_halt("mesh_l3_init failed", -1);

    /* L4: bring up the role's application layer. */
#if NODE_ROLE == ROLE_SINK
    app_base_init();
#else
    app_sensor_init();
#endif

    Serial.println("[OK] L1+L2+L3+L4 initialized");
}

void node_loop(void){
    link_poll();      /* pump L1+L2 (RX dispatch, CSMA/ACK engine)            */
    mesh_l3_tick();   /* run L3 timers (beacon, aging, RREQ retry, queue)     */
#if NODE_ROLE == ROLE_SINK
    app_base_tick(millis());     /* WiFi/MQTT bridge + CAP re-sync            */
#else
    app_sensor_tick(millis());   /* sensors + buzzer + telemetry             */
#endif
}

void node_print_stats(void){
    link_stats_t ls = link_get_stats();
    const mesh_l3_state_t *st = mesh_l3_state();
    Serial.print("[STATS] L2 rx_ok="); Serial.print((unsigned)ls.rx_ok);
    Serial.print(" crc="); Serial.print((unsigned)ls.rx_crc_drop);
    Serial.print(" dup="); Serial.print((unsigned)ls.rx_dup_drop);
    Serial.print(" tx_ok="); Serial.print((unsigned)ls.tx_ok);
    Serial.print(" retx="); Serial.print((unsigned)ls.tx_retx);
    Serial.print(" noack="); Serial.print((unsigned)ls.tx_no_ack);
    Serial.print(" acks="); Serial.println((unsigned)ls.acks_sent);
    Serial.print("        L3 nbrs="); Serial.print((int)mesh_l3_neighbor_count());
    Serial.print(" fwd="); Serial.print((unsigned)st->statForwarded);
    Serial.print(" dropTTL="); Serial.print((unsigned)st->statDroppedTtl);
    Serial.print(" dropNoRoute="); Serial.print((unsigned)st->statDroppedNoRoute);
    Serial.print(" dupSup="); Serial.println((unsigned)st->statDupSuppressed);
}
