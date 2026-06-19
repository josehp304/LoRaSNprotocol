# Layer 3 (Network) Implementation Guide — PicoLoRa Mesh

> **Read this after L1 (PHY) and L2 (datalink) are tested working.**
> It contains the complete, ready-to-paste L3 implementation, the one required
> wire-format change, the glue that wires L3 onto the existing L2, a test
> harness `main.cpp`, and the build/flash/monitor commands for a multi-node
> bring-up (phases P3–P6 from `DESIGN.md §7`).
>
> Everything here is grounded in the existing interface headers:
> [`mesh_l3.h`](include/mesh_l3.h), [`mesh_l3_types.h`](include/mesh_l3_types.h),
> [`mesh_l3_control.h`](include/mesh_l3_control.h), and the L2 API in
> [`datalink.h`](include/datalink.h).

---

## 0. PREREQUISITE — add a link-source field to the header (do this first)

### Why it is unavoidable

AODV reverse-route setup is the heart of L3: when a node forwards an `RREQ`, it
must install a route back toward the originator whose **next hop = the node that
just transmitted this copy to me** (the *previous hop*, called `linkFrom` in
[`mesh_l3.h`](include/mesh_l3.h#L57)). Precursor tracking (for targeted `RERR`)
needs the same thing for `DATA`.

The current 17-byte header carries `src` (immutable origin), `dst`, and
`next_hop` (the *forward* recipient) — but **no field naming the immediate
sender**. LoRa has no link-layer addressing, so the previous hop is
unrecoverable unless we put it in the frame. We add a 2-byte `link_src`.

> Each transmitter stamps `link_src = its own node id` on **every** frame it
> sends (including forwards and ACKs). The receiver reads it as `linkFrom`.
> This keeps the fix uniform across all packet types and leaves the L3 control
> payloads (`mesh_l3_control.h`) untouched.

This bumps `HDR_LEN` 17 → 19 and `MAX_PAYLOAD_PHY` 236 → 234. The airtime
estimator already derives from frame length dynamically, so no timing constants
change. **Re-run the L1/L2 round-trip test after applying this.**

### 0.1 `include/mesh_config.h`

```c
/* ============================ FRAME GEOMETRY ============================= */
#define HDR_LEN        19       /* was 17: +2 for link_src (prev-hop) field    */
#define CRC_LEN        2
#define MAX_PAYLOAD    64
#define FRAME_OVERHEAD (HDR_LEN + CRC_LEN)        /* now 21                    */
#define MAX_PAYLOAD_PHY 234u    /* 255 - 19 - 2                                */
#define MAX_FRAME_LEN  255u
```

### 0.2 `include/mesh_frame.h`

Add the field to the offset table (insert `link_src` at offset 9, shift the
rest), to `mesh_header_t`, and keep the layout self-check intact:

```c
#define HF_OFF_MAGIC        0   /* u16 LE */
#define HF_OFF_VERSION      2   /* u8     */
#define HF_OFF_SRC          3   /* u16 LE */
#define HF_OFF_DST          5   /* u16 LE */
#define HF_OFF_NEXTHOP      7   /* u16 LE */
#define HF_OFF_LINKSRC      9   /* u16 LE  <-- NEW: one-hop sender (prev hop)  */
#define HF_OFF_HOPCOUNT    11   /* u8     */
#define HF_OFF_PKTTYPE     12   /* u8     */
#define HF_OFF_TTL         13   /* u8     */
#define HF_OFF_SEQ         14   /* u16 LE */
#define HF_OFF_FLAGS       16   /* u8     */
#define HF_OFF_RESERVED    17   /* u8 = 0 */
#define HF_OFF_PAYLOADLEN  18   /* u8     */
#define HF_OFF_PAYLOAD     19   /* == HDR_LEN                                   */
```

```c
typedef struct {
    uint16_t magic;
    uint8_t  version;
    uint16_t src;
    uint16_t dst;
    uint16_t next_hop;
    uint16_t link_src;   /* NEW: previous hop / one-hop sender (L2 stamps self) */
    uint8_t  hop_count;
    uint8_t  pkt_type;
    uint8_t  ttl;
    uint16_t seq;
    uint8_t  flags;
    uint8_t  reserved;
    uint8_t  payload_len;
} mesh_header_t;
```

### 0.3 `src/mesh_frame.cpp`

In `mesh_frame_serialize()` add one line in the header block:

```c
    put_u16le(out, HF_OFF_NEXTHOP,    hdr->next_hop);
    put_u16le(out, HF_OFF_LINKSRC,    hdr->link_src);   /* NEW                  */
    put_u8   (out, HF_OFF_HOPCOUNT,   hdr->hop_count);
```

In `mesh_frame_deserialize()` add one line in the decode block:

```c
    out->hdr.next_hop    = get_u16le(in, HF_OFF_NEXTHOP);
    out->hdr.link_src    = get_u16le(in, HF_OFF_LINKSRC);  /* NEW              */
    out->hdr.hop_count   = get_u8   (in, HF_OFF_HOPCOUNT);
```

### 0.4 `src/datalink.cpp`

Every frame L2 emits is sent **by this node**, so stamp `link_src = s_self` in
both builders:

```c
/* in build_data_frame(): */
    h.next_hop    = r->next_hop;
    h.link_src    = s_self;          /* NEW: we are the previous hop            */

/* in build_ack_frame(): */
    h.next_hop    = ADDR_BROADCAST;
    h.link_src    = s_self;          /* NEW                                     */
```

The L2→L3 glue (below) then reads `decoded.hdr.link_src` as `linkFrom`.

> If you also keep the P1/P2 `main.cpp` harness around, set
> `hdr.link_src = node_id();` where it fills the header, or it will serialize an
> uninitialized value.

---

## 1. File structure after L3

```
include/
  mesh_l3.h            (exists) L3 public API + protocol spec
  mesh_l3_types.h      (exists) tables: neighbor/route/seen/pending/queue
  mesh_l3_control.h    (exists) on-wire control payload layouts
  ... (L1/L2 headers unchanged except §0)

src/
  mesh_frame.cpp       (edit  §0.3)
  node_identity.cpp    (exists)
  phy_sx1278.cpp       (exists)
  datalink.cpp         (edit  §0.4)
  mesh_l3.cpp          (NEW   §2)  the AODV engine
  mesh_node.cpp        (NEW   §3)  glue: L2<->L3 hooks + node_setup/node_loop
  main.cpp             (REPLACE §4) P4/P5 test harness
```

No new headers are required — `mesh_node.cpp` exposes two functions declared
inline in `main.cpp` (or move them into the existing `include/mesh_node.h`).

---

## 2. `src/mesh_l3.cpp` — the network layer

```cpp
/* =============================================================================
 * mesh_l3.cpp  --  L3 NETWORK LAYER: AODV-style routing + BEACON discovery
 * -----------------------------------------------------------------------------
 * Implements mesh_l3.h. Pure logic (no Arduino/RadioLib): time + randomness
 * arrive through mesh_l3_hooks_t, so this file is host-testable. All state is a
 * single static mesh_l3_state_t; no dynamic allocation.
 *
 * Frames go DOWN through hooks.l2_send (L2 does CSMA/ACK/retx/PHY); inbound
 * frames arrive via mesh_l3_on_frame (L2 already validated magic/ver/len/CRC);
 * link outcomes arrive via mesh_l3_on_link_event; mesh_l3_tick runs the timers.
 * ===========================================================================*/
#include "mesh_l3.h"
#include <string.h>

/* ---- module state ------------------------------------------------------- */
static mesh_l3_state_t S;
static mesh_l3_hooks_t H;
static bool            s_init = false;

/* Scratch buffer for one outgoing control payload at a time. l2_send copies it
 * synchronously (L2 link_send copies the request), so reuse between emits is
 * safe. */
static uint8_t s_pl[MAX_PAYLOAD];

/* End-to-end DATA dedup, kept separate from the (origin,bcastId) flood seen-set
 * so a DATA seq cannot collide with an RREQ bcastId. */
#define DATA_SEEN_SIZE 32u
static struct { node_id_t src; seq_t seq; uint32_t t; bool used; } s_dataSeen[DATA_SEEN_SIZE];

/* ---- tiny LE (de)serialization helpers ---------------------------------- */
static inline void w8 (uint8_t *b, uint8_t *o, uint8_t  v){ b[(*o)++] = v; }
static inline void w16(uint8_t *b, uint8_t *o, uint16_t v){ b[(*o)++]=(uint8_t)v; b[(*o)++]=(uint8_t)(v>>8); }
static inline void w32(uint8_t *b, uint8_t *o, uint32_t v){ for(int i=0;i<4;i++) b[(*o)++]=(uint8_t)(v>>(8*i)); }
static inline uint8_t  r8 (const uint8_t *b, uint8_t *o){ return b[(*o)++]; }
static inline uint16_t r16(const uint8_t *b, uint8_t *o){ uint16_t v=(uint16_t)b[*o]|((uint16_t)b[*o+1]<<8); *o+=2; return v; }
static inline uint32_t r32(const uint8_t *b, uint8_t *o){ uint32_t v=0; for(int i=0;i<4;i++) v|=((uint32_t)b[*o+i])<<(8*i); *o+=4; return v; }

/* RFC-1982 "a strictly newer than b" (local copy; mesh_types.h not included). */
static inline bool seq_newer(seq_t a, seq_t b){ return (int16_t)(a - b) > 0; }
static inline uint32_t now_ms(void){ return H.now_ms(); }

/* ---- elapsed helper (wrap-safe) ----------------------------------------- */
static inline bool elapsed(uint32_t since, uint32_t span){ return (uint32_t)(now_ms() - since) >= span; }

/* =============================================================================
 * LINK COST  (integer; no FPU). Lower = better. 1..LINK_COST_MAX.
 * ===========================================================================*/
link_cost_t mesh_link_cost(int16_t rssi, int8_t snr){
    int32_t r = rssi;
    if (r > -30)  r = -30;
    if (r < -120) r = -120;
    int32_t cost = (-30 - r) / 6 + 1;        /* -30dBm->1 ... -120dBm->16      */
    if (snr < 0) cost += (-(int32_t)snr) / 2;/* penalize negative SNR          */
    if (cost < LINK_COST_BEST) cost = LINK_COST_BEST;
    if (cost > LINK_COST_MAX)  cost = LINK_COST_MAX;
    return (link_cost_t)cost;
}
static inline uint16_t add_metric(uint16_t base, link_cost_t c){
    uint32_t m = (uint32_t)base + c;
    return (m >= LINK_COST_INFINITE) ? (LINK_COST_INFINITE - 1) : (uint16_t)m;
}

/* =============================================================================
 * EMIT  (build an l3_tx_frame_t and hand it down to L2)
 * ===========================================================================*/
static bool l3_tx(node_id_t src, node_id_t dst, node_id_t nextHop,
                  uint8_t hop, uint8_t ttl, seq_t seq, mesh_pkttype_t type,
                  bool wantsAck, const uint8_t *pl, uint8_t plen){
    l3_tx_frame_t f;
    f.src = src; f.dst = dst; f.nextHop = nextHop;
    f.hopCount = hop; f.ttl = ttl; f.seqNum = seq; f.type = type;
    f.wantsAck = wantsAck; f.payload = pl; f.payloadLen = plen;
    return H.l2_send(&f);
}

/* =============================================================================
 * NEIGHBOR TABLE
 * ===========================================================================*/
static neighbor_entry_t *nbr_find(node_id_t id){
    for (uint8_t i=0;i<MAX_NEIGHBORS;i++)
        if ((S.neighbors[i].flags & NEIGHBOR_FLAG_VALID) && S.neighbors[i].nodeId==id)
            return &S.neighbors[i];
    return NULL;
}
static neighbor_entry_t *nbr_alloc(node_id_t id){
    neighbor_entry_t *e = nbr_find(id);
    if (e) return e;
    for (uint8_t i=0;i<MAX_NEIGHBORS;i++)
        if (!(S.neighbors[i].flags & NEIGHBOR_FLAG_VALID)){
            memset(&S.neighbors[i],0,sizeof(S.neighbors[i]));
            S.neighbors[i].nodeId=id; S.neighbors[i].flags=NEIGHBOR_FLAG_VALID;
            return &S.neighbors[i];
        }
    /* table full: evict the stalest. */
    uint8_t victim=0; uint32_t oldest=0;
    for (uint8_t i=0;i<MAX_NEIGHBORS;i++){
        uint32_t age=(uint32_t)(now_ms()-S.neighbors[i].lastHeard);
        if (age>oldest){ oldest=age; victim=i; }
    }
    memset(&S.neighbors[victim],0,sizeof(S.neighbors[victim]));
    S.neighbors[victim].nodeId=id; S.neighbors[victim].flags=NEIGHBOR_FLAG_VALID;
    return &S.neighbors[victim];
}
/* Learn/refresh a one-hop neighbor from any received frame. */
static void nbr_learn(node_id_t from, int16_t rssi, int8_t snr){
    if (from==ADDR_INVALID || from==ADDR_BROADCAST || from==S.selfId) return;
    neighbor_entry_t *e = nbr_alloc(from);
    e->lastRssi=rssi; e->lastSnr=snr;
    e->linkCost=mesh_link_cost(rssi,snr);
    e->lastHeard=now_ms(); e->missed=0;
}
uint8_t mesh_l3_neighbor_count(void){
    uint8_t n=0;
    for (uint8_t i=0;i<MAX_NEIGHBORS;i++) if (S.neighbors[i].flags & NEIGHBOR_FLAG_VALID) n++;
    return n;
}

/* =============================================================================
 * ROUTING TABLE
 * ===========================================================================*/
static routing_entry_t *route_find(node_id_t dest){
    for (uint8_t i=0;i<MAX_ROUTES;i++)
        if (S.routes[i].dest==dest && S.routes[i].state!=ROUTE_INVALID)
            return &S.routes[i];
    return NULL;
}
static routing_entry_t *route_slot(node_id_t dest){
    routing_entry_t *r=NULL;
    for (uint8_t i=0;i<MAX_ROUTES;i++){
        if (S.routes[i].dest==dest) return &S.routes[i];       /* reuse same dest */
        if (!r && (S.routes[i].dest==ADDR_INVALID || S.routes[i].state==ROUTE_INVALID)) r=&S.routes[i];
    }
    if (r){ memset(r,0,sizeof(*r)); r->dest=dest; return r; }
    /* full of valid routes: evict the one expiring soonest. */
    routing_entry_t *victim=&S.routes[0];
    for (uint8_t i=1;i<MAX_ROUTES;i++)
        if ((int32_t)(S.routes[i].expiry - victim->expiry) < 0) victim=&S.routes[i];
    memset(victim,0,sizeof(*victim)); victim->dest=dest; return victim;
}
/* AODV freshness rule: accept iff newer destSeq, or equal destSeq & better
 * metric, or current route has no known destSeq / is invalid. */
static void route_update(node_id_t dest, node_id_t nextHop, uint8_t hop,
                         uint16_t metric, seq_t destSeq, uint8_t destSeqValid,
                         uint32_t lifetime){
    if (dest==ADDR_INVALID || dest==S.selfId) return;
    routing_entry_t *r = route_find(dest);
    if (r && r->state==ROUTE_VALID && r->destSeqValid && destSeqValid){
        bool fresher = seq_newer(destSeq, r->destSeq);
        bool shorter = (destSeq==r->destSeq) && (metric < r->metric);
        if (!fresher && !shorter) { r->expiry = now_ms()+lifetime; return; }
    }
    if (!r) r = route_slot(dest);
    r->dest=dest; r->nextHop=nextHop; r->hopCount=hop; r->metric=metric;
    r->destSeq=destSeq; r->destSeqValid=destSeqValid?1:0;
    r->state=ROUTE_VALID; r->expiry=now_ms()+lifetime;
}
static void route_refresh(routing_entry_t *r){ r->expiry = now_ms()+ROUTE_EXPIRY_MS; }
static void route_add_precursor(node_id_t dest, node_id_t pre){
    if (pre==ADDR_INVALID || pre==ADDR_BROADCAST) return;
    routing_entry_t *r=route_find(dest); if (!r) return;
    for (uint8_t i=0;i<ROUTE_PRECURSORS;i++) if (r->precursors[i]==pre) return;
    for (uint8_t i=0;i<ROUTE_PRECURSORS;i++) if (r->precursors[i]==ADDR_INVALID){ r->precursors[i]=pre; return; }
}
bool mesh_l3_lookup_route(node_id_t dest, node_id_t *nh, uint8_t *hc, uint16_t *mt){
    routing_entry_t *r=route_find(dest);
    if (!r || r->state!=ROUTE_VALID) return false;
    if (nh) *nh=r->nextHop; if (hc) *hc=r->hopCount; if (mt) *mt=r->metric;
    return true;
}

/* =============================================================================
 * SEEN-SET (flood dedup) and DATA dedup
 * ===========================================================================*/
static bool seen_insert(node_id_t origin, bcast_id_t id){   /* true if NEW */
    int free=-1;
    for (uint8_t i=0;i<MAX_SEEN;i++){
        if (S.seen[i].origin==origin && S.seen[i].bcastId==id) return false;
        if (free<0 && S.seen[i].origin==ADDR_INVALID) free=i;
    }
    if (free<0){ /* evict oldest */ uint32_t o=0; for(uint8_t i=0;i<MAX_SEEN;i++){uint32_t a=(uint32_t)(now_ms()-S.seen[i].firstSeen); if(a>=o){o=a;free=i;}} }
    S.seen[free].origin=origin; S.seen[free].bcastId=id; S.seen[free].firstSeen=now_ms();
    return true;
}
static bool data_seen_insert(node_id_t src, seq_t seq){     /* true if NEW */
    int free=-1;
    for (uint8_t i=0;i<DATA_SEEN_SIZE;i++){
        if (s_dataSeen[i].used && s_dataSeen[i].src==src && s_dataSeen[i].seq==seq) return false;
        if (free<0 && !s_dataSeen[i].used) free=i;
    }
    if (free<0){ uint32_t o=0; for(uint8_t i=0;i<DATA_SEEN_SIZE;i++){uint32_t a=(uint32_t)(now_ms()-s_dataSeen[i].t); if(a>=o){o=a;free=i;}} }
    s_dataSeen[free].used=true; s_dataSeen[free].src=src; s_dataSeen[free].seq=seq; s_dataSeen[free].t=now_ms();
    return true;
}

/* =============================================================================
 * PENDING-RREQ TABLE
 * ===========================================================================*/
static pending_rreq_t *pending_find(node_id_t dest){
    for (uint8_t i=0;i<MAX_PENDING_RREQ;i++) if (S.pending[i].active && S.pending[i].dest==dest) return &S.pending[i];
    return NULL;
}
static pending_rreq_t *pending_alloc(node_id_t dest){
    for (uint8_t i=0;i<MAX_PENDING_RREQ;i++) if (!S.pending[i].active){ memset(&S.pending[i],0,sizeof(S.pending[i])); S.pending[i].dest=dest; return &S.pending[i]; }
    return NULL;
}
static void pending_clear(node_id_t dest){
    pending_rreq_t *p=pending_find(dest); if (p) p->active=0;
}

/* =============================================================================
 * STORE-AND-FORWARD QUEUE
 * ===========================================================================*/
static int queue_put(node_id_t dest, node_id_t origin, seq_t seq,
                     const uint8_t *pl, uint8_t len, uint8_t ttl, bool inflight){
    int slot=-1;
    for (uint8_t i=0;i<MAX_QUEUE;i++) if (S.queue[i].qstate==QSTATE_FREE){ slot=i; break; }
    if (slot<0){ /* drop oldest WAIT_ROUTE */
        uint32_t o=0; int v=-1;
        for (uint8_t i=0;i<MAX_QUEUE;i++) if (S.queue[i].qstate==QSTATE_WAIT_ROUTE){ uint32_t a=(uint32_t)(now_ms()-S.queue[i].enqueueTime); if(a>=o){o=a;v=i;} }
        if (v<0) return -1;
        slot=v;
    }
    queue_entry_t *q=&S.queue[slot];
    q->finalDest=dest; q->origin=origin; q->seq=seq; q->payloadLen=len;
    q->pktType=PKT_DATA; q->ttl=ttl; q->retries=0; q->enqueueTime=now_ms();
    q->qstate = inflight ? QSTATE_INFLIGHT : QSTATE_WAIT_ROUTE;
    memcpy(q->payload, pl, len);
    return slot;
}
/* Send a queued DATA toward a now-known route. Local-origin entries stay
 * INFLIGHT for result reporting; forwarded entries are freed after send. */
static void queue_try_flush(node_id_t dest){
    routing_entry_t *r=route_find(dest);
    if (!r || r->state!=ROUTE_VALID) return;
    for (uint8_t i=0;i<MAX_QUEUE;i++){
        queue_entry_t *q=&S.queue[i];
        if (q->qstate!=QSTATE_WAIT_ROUTE || q->finalDest!=dest) continue;
        bool local = (q->origin==S.selfId);
        uint8_t hop = local ? 0 : 1;          /* forwarded entries have +1 hop  */
        route_refresh(r);
        bool ok = l3_tx(q->origin, dest, r->nextHop, hop, q->ttl, q->seq,
                        PKT_DATA, true, q->payload, q->payloadLen);
        if (ok && local){ q->qstate=QSTATE_INFLIGHT; q->enqueueTime=now_ms(); }
        else            { q->qstate=QSTATE_FREE; }   /* forwarded or send-fail */
    }
}

/* =============================================================================
 * RERR  (invalidate routes via a broken next hop, notify precursors)
 * ===========================================================================*/
static void rerr_for_broken(node_id_t broken){
    unreachable_dest_t dead[RERR_MAX_DESTS];
    node_id_t pres[ROUTE_PRECURSORS*2]; uint8_t npre=0;
    uint8_t n=0;
    for (uint8_t i=0;i<MAX_ROUTES && n<RERR_MAX_DESTS;i++){
        routing_entry_t *r=&S.routes[i];
        if (r->state==ROUTE_VALID && r->nextHop==broken){
            r->state=ROUTE_INVALID; r->metric=LINK_COST_INFINITE;
            r->destSeq=(seq_t)(r->destSeq+1); r->destSeqValid=1;   /* bump */
            dead[n].dest=r->dest; dead[n].destSeq=r->destSeq; n++;
            for (uint8_t p=0;p<ROUTE_PRECURSORS;p++){
                node_id_t pc=r->precursors[p];
                if (pc!=ADDR_INVALID && npre<sizeof(pres)/sizeof(pres[0])){
                    bool dup=false; for(uint8_t k=0;k<npre;k++) if(pres[k]==pc) dup=true;
                    if(!dup) pres[npre++]=pc;
                }
            }
        }
    }
    if (n==0) return;
    uint8_t o=0; w8(s_pl,&o,n); w8(s_pl,&o,0);
    for (uint8_t i=0;i<n;i++){ w16(s_pl,&o,dead[i].dest); w16(s_pl,&o,dead[i].destSeq); }
    if (npre==0){
        l3_tx(S.selfId, ADDR_BROADCAST, ADDR_BROADCAST, 0, 1, (seq_t)S.mySeq, PKT_RERR, false, s_pl, o);
    } else {
        for (uint8_t k=0;k<npre;k++)
            l3_tx(S.selfId, pres[k], pres[k], 0, 1, (seq_t)S.mySeq, PKT_RERR, true, s_pl, o);
    }
}

/* =============================================================================
 * RREQ / RREP emit helpers
 * ===========================================================================*/
static void rreq_originate(node_id_t dest){
    if (pending_find(dest)) return;                  /* already discovering     */
    pending_rreq_t *p=pending_alloc(dest);
    if (!p){ return; }
    p->active=1; p->ttl=RREQ_START_TTL; p->retries=0;
    p->bcastId=(bcast_id_t)(++S.myBcastId); p->lastRreqSeq=(seq_t)S.mySeq; p->sentAt=now_ms();
    routing_entry_t *r=route_find(dest);
    seq_t ds = r?r->destSeq:0; uint8_t dv = (r&&r->destSeqValid)?1:0;
    uint8_t o=0;
    w16(s_pl,&o,S.selfId); w16(s_pl,&o,dest); w16(s_pl,&o,p->bcastId);
    w16(s_pl,&o,(seq_t)S.mySeq); w16(s_pl,&o,ds); w8(s_pl,&o,dv);
    w16(s_pl,&o,0); w8(s_pl,&o,0);                    /* metric=0, flags=0       */
    l3_tx(S.selfId, ADDR_BROADCAST, ADDR_BROADCAST, 0, p->ttl, p->bcastId, PKT_RREQ, false, s_pl, o);
}
static void rreq_rebroadcast(pending_rreq_t *p){
    routing_entry_t *r=route_find(p->dest);
    seq_t ds=r?r->destSeq:0; uint8_t dv=(r&&r->destSeqValid)?1:0;
    uint8_t o=0;
    w16(s_pl,&o,S.selfId); w16(s_pl,&o,p->dest); w16(s_pl,&o,p->bcastId);
    w16(s_pl,&o,(seq_t)S.mySeq); w16(s_pl,&o,ds); w8(s_pl,&o,dv);
    w16(s_pl,&o,0); w8(s_pl,&o,0);
    l3_tx(S.selfId, ADDR_BROADCAST, ADDR_BROADCAST, 0, p->ttl, p->bcastId, PKT_RREQ, false, s_pl, o);
}
static void rrep_send(node_id_t routeDest, node_id_t origin, seq_t destSeq,
                      uint8_t hop, uint16_t metric, node_id_t nextHop, uint8_t ttl){
    uint8_t o=0;
    w16(s_pl,&o,routeDest); w16(s_pl,&o,origin); w16(s_pl,&o,destSeq);
    w8(s_pl,&o,hop); w16(s_pl,&o,metric); w32(s_pl,&o,(uint32_t)ACTIVE_ROUTE_LIFE_MS);
    w8(s_pl,&o,0);
    /* RREP header: src=routeDest (immutable), dst=origin, nextHop=reverse hop. */
    l3_tx(routeDest, origin, nextHop, hop, ttl, destSeq, PKT_RREP, true, s_pl, o);
}

/* =============================================================================
 * BEACON
 * ===========================================================================*/
static void beacon_send(void){
    uint8_t o=0;
    uint8_t cntPos=o; w8(s_pl,&o,0);                 /* nbrCount placeholder     */
    uint8_t flags = (NODE_ROLE!=ROLE_SENDER) ? BEACON_FLAG_ROUTER : BEACON_FLAG_NONE;
    w8(s_pl,&o,flags);
    uint8_t cnt=0;
    for (uint8_t i=0;i<MAX_NEIGHBORS && cnt<BEACON_MAX_NBRS;i++){
        if (S.neighbors[i].flags & NEIGHBOR_FLAG_VALID){
            w16(s_pl,&o,S.neighbors[i].nodeId); w8(s_pl,&o,S.neighbors[i].linkCost); cnt++;
        }
    }
    s_pl[cntPos]=cnt;
    l3_tx(S.selfId, ADDR_BROADCAST, ADDR_BROADCAST, 0, 1, (seq_t)S.mySeq, PKT_BEACON, false, s_pl, o);
}

/* =============================================================================
 * INBOUND HANDLERS
 * ===========================================================================*/
static void on_beacon(const l3_rx_frame_t *f){
    neighbor_entry_t *e=nbr_alloc(f->src);
    e->lastRssi=f->rssi; e->lastSnr=f->snr; e->linkCost=mesh_link_cost(f->rssi,f->snr);
    e->lastHeard=now_ms(); e->missed=0;
    uint8_t o=0; uint8_t cnt=r8(f->payload,&o); uint8_t bflags=r8(f->payload,&o);
    if (bflags & BEACON_FLAG_ROUTER) e->flags |= NEIGHBOR_FLAG_ROUTER;
    for (uint8_t i=0;i<cnt && (o+3u)<=f->payloadLen;i++){
        node_id_t id=r16(f->payload,&o); (void)r8(f->payload,&o);  /* linkCost */
        if (id==S.selfId) e->flags |= NEIGHBOR_FLAG_SYMMETRIC;     /* two-way!  */
    }
}
static void on_rreq(const l3_rx_frame_t *f){
    uint8_t o=0;
    node_id_t origin=r16(f->payload,&o); node_id_t dest=r16(f->payload,&o);
    bcast_id_t bid=r16(f->payload,&o); seq_t oseq=r16(f->payload,&o);
    seq_t dseq=r16(f->payload,&o); uint8_t dvalid=r8(f->payload,&o);
    uint16_t metric=r16(f->payload,&o); /* uint8_t rflags=r8(f->payload,&o); */
    if (!seen_insert(origin,bid)){ S.statDupSuppressed++; return; }

    link_cost_t lc=mesh_link_cost(f->rssi,f->snr);
    /* reverse route toward origin via the previous hop. */
    route_update(origin, f->linkFrom, f->hopCount+1, add_metric(metric,lc),
                 oseq, 1, REVERSE_ROUTE_LIFE_MS);

    if (dest==S.selfId){                                /* we are the target     */
        if (seq_newer(dseq,(seq_t)S.mySeq) && dvalid) S.mySeq=dseq;   /* AODV   */
        S.mySeq++;                                       /* fresh dest seq        */
        routing_entry_t *rev=route_find(origin);
        if (rev) rrep_send(S.selfId, origin, (seq_t)S.mySeq, 0, 0,
                           rev->nextHop, rev->hopCount+2);
        return;
    }
    routing_entry_t *rt=route_find(dest);
    if (rt && rt->state==ROUTE_VALID &&
        (!dvalid || (rt->destSeqValid && !seq_newer(dseq, rt->destSeq)))){
        routing_entry_t *rev=route_find(origin);
        if (rev){
            route_add_precursor(dest, rev->nextHop);
            rrep_send(dest, origin, rt->destSeq, rt->hopCount, rt->metric,
                      rev->nextHop, rev->hopCount+2);
        }
        return;
    }
    if (f->ttl<=1) return;                              /* flood radius reached  */
    /* relay with ++hop, --ttl, accumulated metric. */
    uint8_t o2=0;
    w16(s_pl,&o2,origin); w16(s_pl,&o2,dest); w16(s_pl,&o2,bid);
    w16(s_pl,&o2,oseq); w16(s_pl,&o2,dseq); w8(s_pl,&o2,dvalid);
    w16(s_pl,&o2,add_metric(metric,lc)); w8(s_pl,&o2,0);
    l3_tx(origin, ADDR_BROADCAST, ADDR_BROADCAST, f->hopCount+1, f->ttl-1, bid,
          PKT_RREQ, false, s_pl, o2);
}
static void on_rrep(const l3_rx_frame_t *f){
    uint8_t o=0;
    node_id_t dest=r16(f->payload,&o); node_id_t origin=r16(f->payload,&o);
    seq_t dseq=r16(f->payload,&o); uint8_t hop=r8(f->payload,&o);
    uint16_t metric=r16(f->payload,&o); uint32_t life=r32(f->payload,&o);
    /* uint8_t flags=r8(f->payload,&o); */
    link_cost_t lc=mesh_link_cost(f->rssi,f->snr);
    /* forward route toward dest via the previous hop. */
    route_update(dest, f->linkFrom, hop+1, add_metric(metric,lc), dseq, 1, life);

    if (origin==S.selfId){                               /* discovery complete   */
        pending_clear(dest);
        queue_try_flush(dest);
        return;
    }
    routing_entry_t *rev=route_find(origin);             /* reverse route home   */
    if (!rev || rev->state!=ROUTE_VALID) return;
    route_add_precursor(dest, rev->nextHop);
    if (f->ttl<=1) return;
    rrep_send(dest, origin, dseq, hop+1, add_metric(metric,lc),
              rev->nextHop, f->ttl-1);
}
static void on_rerr(const l3_rx_frame_t *f){
    uint8_t o=0; uint8_t cnt=r8(f->payload,&o); (void)r8(f->payload,&o);
    node_id_t reprop[RERR_MAX_DESTS]; seq_t repseq[RERR_MAX_DESTS]; uint8_t rn=0;
    for (uint8_t i=0;i<cnt && (o+4u)<=f->payloadLen;i++){
        node_id_t d=r16(f->payload,&o); seq_t ds=r16(f->payload,&o);
        routing_entry_t *r=route_find(d);
        if (r && r->state==ROUTE_VALID && r->nextHop==f->linkFrom &&
            (!r->destSeqValid || seq_newer(ds, r->destSeq))){
            r->state=ROUTE_INVALID; r->metric=LINK_COST_INFINITE;
            r->destSeq=ds; r->destSeqValid=1;
            if (rn<RERR_MAX_DESTS){ reprop[rn]=d; repseq[rn]=ds; rn++; }
        }
    }
    if (rn==0) return;                                   /* freshness gate held  */
    uint8_t o2=0; w8(s_pl,&o2,rn); w8(s_pl,&o2,0);
    for (uint8_t i=0;i<rn;i++){ w16(s_pl,&o2,reprop[i]); w16(s_pl,&o2,repseq[i]); }
    l3_tx(S.selfId, ADDR_BROADCAST, ADDR_BROADCAST, 0, 1, (seq_t)S.mySeq, PKT_RERR, false, s_pl, o2);
}
static void on_data(const l3_rx_frame_t *f){
    if (!data_seen_insert(f->src, f->seqNum)) return;    /* end-to-end dup       */
    if (f->dst==S.selfId){
        H.l4_deliver(f->src, f->seqNum, f->payload, f->payloadLen);
        return;
    }
    routing_entry_t *r=route_find(f->dst);
    if (r && r->state==ROUTE_VALID){
        if (f->ttl<=1){ S.statDroppedTtl++; return; }
        route_add_precursor(f->dst, f->linkFrom);        /* upstream precursor   */
        route_refresh(r);
        l3_tx(f->src, f->dst, r->nextHop, f->hopCount+1, f->ttl-1, f->seqNum,
              PKT_DATA, true, f->payload, f->payloadLen);
        S.statForwarded++;
    } else {
        if (queue_put(f->dst, f->src, f->seqNum, f->payload, f->payloadLen, f->ttl, false) >= 0)
            rreq_originate(f->dst);
        else
            S.statDroppedNoRoute++;
    }
}

/* =============================================================================
 * PUBLIC API
 * ===========================================================================*/
l3_result_t mesh_l3_init(node_id_t selfId, const mesh_l3_hooks_t *hooks){
    if (!hooks || !hooks->l2_send || !hooks->now_ms || !hooks->rand_u32) return L3_ERR_NOT_INIT;
    memset(&S,0,sizeof(S));
    memset(s_dataSeen,0,sizeof(s_dataSeen));
    H=*hooks; S.selfId=selfId; S.mySeq=0; S.myBcastId=0;
    S.nextBeaconAt = H.now_ms() + (H.rand_u32() % BEACON_JITTER_MS);  /* soon */
    s_init=true;
    return L3_OK;
}

l3_result_t mesh_l3_send(node_id_t dest, const uint8_t *payload, uint8_t len, seq_t *out_seq){
    if (!s_init) return L3_ERR_NOT_INIT;
    if (dest==ADDR_INVALID) return L3_ERR_BAD_DEST;
    if (len>MAX_PAYLOAD) return L3_ERR_PAYLOAD_TOO_BIG;
    seq_t seq=(seq_t)(++S.mySeq); if (out_seq) *out_seq=seq;
    if (dest==S.selfId){                                  /* loopback             */
        H.l4_deliver(S.selfId, seq, payload, len);
        if (H.l4_send_result) H.l4_send_result(dest, seq, true);
        return L3_OK;
    }
    routing_entry_t *r=route_find(dest);
    if (r && r->state==ROUTE_VALID){
        if (queue_put(dest, S.selfId, seq, payload, len, DEFAULT_TTL, true) < 0)
            return L3_ERR_QUEUE_FULL;
        route_refresh(r);
        l3_tx(S.selfId, dest, r->nextHop, 0, DEFAULT_TTL, seq, PKT_DATA, true, payload, len);
        return L3_OK;
    }
    if (queue_put(dest, S.selfId, seq, payload, len, DEFAULT_TTL, false) < 0)
        return L3_ERR_QUEUE_FULL;
    rreq_originate(dest);
    return L3_ERR_NO_ROUTE;                               /* buffered; not failed */
}

void mesh_l3_on_frame(const l3_rx_frame_t *f){
    if (!s_init || !f) return;
    nbr_learn(f->linkFrom, f->rssi, f->snr);              /* learn from any frame */
    switch (f->type){
        case PKT_BEACON: on_beacon(f); break;
        case PKT_RREQ:   on_rreq(f);   break;
        case PKT_RREP:   on_rrep(f);   break;
        case PKT_RERR:   on_rerr(f);   break;
        case PKT_DATA:   on_data(f);   break;
        default: break;                                   /* ACK never reaches L3 */
    }
}

void mesh_l3_on_link_event(l3_link_event_t ev, node_id_t nextHop, seq_t ctx){
    if (!s_init) return;
    if (ev==L3_LINK_TX_DONE){
        for (uint8_t i=0;i<MAX_QUEUE;i++){
            queue_entry_t *q=&S.queue[i];
            if (q->qstate==QSTATE_INFLIGHT && q->seq==ctx){
                if (q->origin==S.selfId && H.l4_send_result) H.l4_send_result(q->finalDest,q->seq,true);
                q->qstate=QSTATE_FREE;
            }
        }
        routing_entry_t *r=route_find(nextHop); if (r) route_refresh(r);
    } else { /* L3_LINK_TX_FAILED or L3_LINK_NEIGHBOR_LOST */
        for (uint8_t i=0;i<MAX_QUEUE;i++){
            queue_entry_t *q=&S.queue[i];
            if (q->qstate==QSTATE_INFLIGHT && q->seq==ctx){
                if (q->origin==S.selfId && H.l4_send_result) H.l4_send_result(q->finalDest,q->seq,false);
                q->qstate=QSTATE_FREE;
            }
        }
        rerr_for_broken(nextHop);
        /* drop the dead neighbor so beacons can re-learn it fresh. */
        neighbor_entry_t *e=nbr_find(nextHop); if (e) e->flags=NEIGHBOR_FLAG_NONE;
    }
}

uint32_t mesh_l3_tick(void){
    if (!s_init) return 1000;
    uint32_t t=now_ms();

    if ((int32_t)(t - S.nextBeaconAt) >= 0){
        beacon_send();
        S.nextBeaconAt = t + BEACON_INTERVAL_MS + (H.rand_u32() % (BEACON_JITTER_MS+1));
    }
    /* neighbor liveness */
    for (uint8_t i=0;i<MAX_NEIGHBORS;i++){
        neighbor_entry_t *e=&S.neighbors[i];
        if ((e->flags & NEIGHBOR_FLAG_VALID) && (uint32_t)(t-e->lastHeard) > NEIGHBOR_TIMEOUT_MS){
            node_id_t lost=e->nodeId; e->flags=NEIGHBOR_FLAG_NONE;
            rerr_for_broken(lost);
        }
    }
    /* route expiry */
    for (uint8_t i=0;i<MAX_ROUTES;i++)
        if (S.routes[i].state==ROUTE_VALID && (int32_t)(t-S.routes[i].expiry) >= 0)
            S.routes[i].state=ROUTE_INVALID;
    /* seen-set aging */
    for (uint8_t i=0;i<MAX_SEEN;i++)
        if (S.seen[i].origin!=ADDR_INVALID && (uint32_t)(t-S.seen[i].firstSeen) > SEEN_ENTRY_TTL_MS)
            S.seen[i].origin=ADDR_INVALID;
    for (uint8_t i=0;i<DATA_SEEN_SIZE;i++)
        if (s_dataSeen[i].used && (uint32_t)(t-s_dataSeen[i].t) > SEEN_ENTRY_TTL_MS)
            s_dataSeen[i].used=false;
    /* pending RREQ ring-search / give-up */
    for (uint8_t i=0;i<MAX_PENDING_RREQ;i++){
        pending_rreq_t *p=&S.pending[i];
        if (!p->active) continue;
        if ((uint32_t)(t-p->sentAt) <= RREQ_TIMEOUT_MS) continue;
        if (p->retries < RREQ_RETRY){
            p->retries++;
            p->ttl = (uint8_t)((p->ttl+RREQ_TTL_INCREMENT > RREQ_MAX_TTL) ? RREQ_MAX_TTL : p->ttl+RREQ_TTL_INCREMENT);
            p->sentAt=t;
            rreq_rebroadcast(p);
        } else {
            node_id_t dest=p->dest; p->active=0;
            for (uint8_t q=0;q<MAX_QUEUE;q++){
                queue_entry_t *e=&S.queue[q];
                if (e->qstate==QSTATE_WAIT_ROUTE && e->finalDest==dest){
                    if (e->origin==S.selfId && H.l4_send_result) H.l4_send_result(dest,e->seq,false);
                    e->qstate=QSTATE_FREE;
                }
            }
        }
    }
    /* queue: flush if a route appeared, else expire */
    for (uint8_t i=0;i<MAX_QUEUE;i++){
        queue_entry_t *q=&S.queue[i];
        if (q->qstate==QSTATE_WAIT_ROUTE){
            if (route_find(q->finalDest)) { queue_try_flush(q->finalDest); }
            else if ((uint32_t)(t-q->enqueueTime) > QUEUE_ENTRY_TTL_MS){
                if (q->origin==S.selfId && H.l4_send_result) H.l4_send_result(q->finalDest,q->seq,false);
                q->qstate=QSTATE_FREE;
            }
        }
    }
    return 0;   /* keep calling; cheap and non-blocking */
}

const mesh_l3_state_t *mesh_l3_state(void){ return &S; }
```

---

## 3. `src/mesh_node.cpp` — glue between L2 and L3

This adapter is the only place the two layers' types meet. It implements the
L3 hooks (down to `link_send`), forwards L2 RX/TX events up to L3, and exposes
`node_setup()` / `node_loop()`.

```cpp
/* =============================================================================
 * mesh_node.cpp  --  Integration glue: L2 (datalink) <-> L3 (mesh_l3)
 * ===========================================================================*/
#include <Arduino.h>
#include <string.h>
#include "datalink.h"
#include "mesh_l3.h"
#include "node_identity.h"

/* ---- handle -> (nextHop,seq) map so L2 tx-completions map to L3 events ---- */
static struct { bool used; node_id_t nextHop; seq_t seq; } s_txmap[8];

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
    if (h<8){ s_txmap[h].used=true; s_txmap[h].nextHop=f->nextHop; s_txmap[h].seq=f->seqNum; }
    return true;
}

/* ---- UP: L4 delivery / send-result (replace with your app) -------------- */
static void hook_l4_deliver(node_id_t from, seq_t seq, const uint8_t *pl, uint8_t len){
    Serial.print("[L4 RX] from=0x"); Serial.print(from,HEX);
    Serial.print(" seq="); Serial.print((unsigned)seq);
    Serial.print(" len="); Serial.print((int)len); Serial.print("  \"");
    for (uint8_t i=0;i<len;i++) Serial.print((char)pl[i]);
    Serial.println("\"");
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
    f.linkFrom=frame->hdr.link_src;          /* <-- the §0 field               */
    f.hopCount=frame->hdr.hop_count; f.ttl=frame->hdr.ttl; f.seqNum=frame->hdr.seq;
    f.type=(mesh_pkttype_t)frame->hdr.pkt_type;
    f.rssi=frame->rssi_dbm; f.snr=frame->snr_db;
    f.payload=frame->payload; f.payloadLen=frame->hdr.payload_len;
    mesh_l3_on_frame(&f);
}

/* ---- UP: L2 reports a transmit outcome --------------------------------- */
static void on_link_tx_done(link_tx_handle_t h, link_tx_state_t st){
    if (h>=8 || !s_txmap[h].used) return;
    node_id_t nh=s_txmap[h].nextHop; seq_t seq=s_txmap[h].seq;
    s_txmap[h].used=false;
    switch (st){
        case LINK_TX_SENT:    mesh_l3_on_link_event(L3_LINK_TX_DONE,   nh, seq); break;
        case LINK_TX_NO_ACK:  mesh_l3_on_link_event(L3_LINK_TX_FAILED, nh, seq); break;
        default: /* BUSY/DUTY/DROPPED: treat as a transient failure */
                              mesh_l3_on_link_event(L3_LINK_TX_FAILED, nh, seq); break;
    }
}

/* ---- public entry points ----------------------------------------------- */
bool node_setup(void){
    s_rng ^= (uint32_t)micros() ^ ((uint32_t)node_id()<<16);
    link_callbacks_t cb; cb.on_rx=on_link_rx; cb.on_tx_done=on_link_tx_done;
    if (link_begin(node_id(), NULL, &cb) != MESH_OK) return false;

    mesh_l3_hooks_t hk;
    hk.l2_send=hook_l2_send;
    hk.l4_deliver=hook_l4_deliver;
    hk.l4_send_result=hook_l4_send_result;
    hk.now_ms=l3_now;
    hk.rand_u32=l3_rand;
    return mesh_l3_init(node_id(), &hk) == L3_OK;
}
void node_loop(void){
    link_poll();      /* pump L1+L2 (RX dispatch, CSMA/ACK engine)            */
    mesh_l3_tick();   /* run L3 timers (beacon, aging, RREQ retry, queue)     */
}
```

---

## 4. `src/main.cpp` — P4/P5 test harness (replaces the P1/P2 sketch)

A `ROLE_SENDER` node originates a DATA packet to the sink every few seconds; the
sink/relays route it. Watch `[L4 TX]`/`[L4 RX]` and the periodic state dump.

```cpp
#include <Arduino.h>
#include "mesh_config.h"
#include "node_identity.h"
#include "mesh_l3.h"

bool node_setup(void);
void node_loop(void);

#define APP_TX_PERIOD_MS  15000UL
#define STATE_DUMP_MS     10000UL

static uint32_t s_nextTx=0, s_nextDump=0;

void setup(){
    Serial.begin(115200);
    uint32_t t0=millis(); while(!Serial && (millis()-t0)<3000){}
    Serial.println("\n=== PicoLoRa L3 bring-up (P4/P5) ===");
    Serial.print("node_id=0x"); Serial.print(node_id(),HEX);
    Serial.print(" role="); Serial.println((int)NODE_ROLE);
    if (!node_setup()){ Serial.println("[FATAL] node_setup failed"); for(;;){} }
    s_nextTx   = millis()+APP_TX_PERIOD_MS;
    s_nextDump = millis()+STATE_DUMP_MS;
}

void loop(){
    node_loop();

    /* Only SENDER nodes originate app traffic toward the sink. */
    if (NODE_ROLE==ROLE_SENDER && (int32_t)(millis()-s_nextTx)>=0){
        s_nextTx += APP_TX_PERIOD_MS;
        static uint16_t n=0;
        char msg[24]; int len=snprintf(msg,sizeof(msg),"hello #%u",(unsigned)n++);
        seq_t seq=0;
        l3_result_t rc=mesh_l3_send(node_sink_addr(),(const uint8_t*)msg,(uint8_t)len,&seq);
        Serial.print("[APP] send seq="); Serial.print((unsigned)seq);
        Serial.print(" rc="); Serial.println((int)rc);  /* 0=sent, -2=discovering */
    }

    if ((int32_t)(millis()-s_nextDump)>=0){
        s_nextDump += STATE_DUMP_MS;
        const mesh_l3_state_t *st=mesh_l3_state();
        Serial.print("[STATE] nbrs="); Serial.print((int)mesh_l3_neighbor_count());
        Serial.print(" fwd="); Serial.print((unsigned)st->statForwarded);
        Serial.print(" dropTTL="); Serial.print((unsigned)st->statDroppedTtl);
        Serial.print(" dropNoRoute="); Serial.print((unsigned)st->statDroppedNoRoute);
        Serial.print(" dupSup="); Serial.println((unsigned)st->statDupSuppressed);
    }
}
```

---

## 5. `platformio.ini` — per-node build environments

Each physical board is the same firmware with a different `NODE_ID`/`NODE_ROLE`.
Add one env per node (inheriting your existing `[env:picow]` settings):

```ini
[env:node_sender]
extends = env:picow
build_flags =
    ${env:picow.build_flags}
    -D NODE_ID=0x0001
    -D NODE_ROLE=ROLE_SENDER
    -D NODE_SINK_ADDR=0x00FF

[env:node_relay]
extends = env:picow
build_flags =
    ${env:picow.build_flags}
    -D NODE_ID=0x0002
    -D NODE_ROLE=ROLE_RELAY

[env:node_sink]
extends = env:picow
build_flags =
    ${env:picow.build_flags}
    -D NODE_ID=0x00FF
    -D NODE_ROLE=ROLE_SINK
```

> If `[env:picow]` doesn't define `build_flags`, drop the
> `${env:picow.build_flags}` line from each block.

---

## 6. Commands

```bash
# Build a single node (catches compile errors fast):
pio run -e node_sink -j 1

# Flash each board in turn (plug in one at a time, or pass --upload-port):
pio run -e node_sender -t upload
pio run -e node_relay  -t upload --upload-port COM5
pio run -e node_sink   -t upload --upload-port COM6

# Serial monitor (one terminal per board):
pio device monitor -p COM6 -b 115200

# Memory report:
pio run -e node_sink -j 1 -v | grep -E "RAM:|Flash:"
```

> On Windows PowerShell, replace `grep` with `Select-String "RAM:|Flash:"`.
> Disk got tight during earlier builds — build one env at a time with `-j 1`.

---

## 7. Incremental test plan (maps to DESIGN.md §7)

| Phase | Setup | Pass criteria |
|-------|-------|---------------|
| **P3 — neighbor discovery** | 2 boards in range, any roles | Each prints rising `[STATE] nbrs=1`; beacons every ~60 s ±10 s; a node appears as `SYMMETRIC` once it sees itself in the peer's beacon list. |
| **P4 — 1-hop DATA** | sender + sink, in range | sender `[APP] send rc=-2` first (no route → RREQ), then `[L4 TX] DELIVERED`; sink prints `[L4 RX] "hello #n"`. After first round, `rc=0` (route cached). |
| **P5 — 2-hop route discovery** | sender — relay — sink, sender NOT in range of sink | RREQ floods, RREP returns via relay, sink gets `[L4 RX]`; relay `[STATE] fwd=` increments; no `dupSup` storm. |
| **P6 — self-heal (RERR)** | P5 topology, then power off the relay | sender's next sends `FAILED`, route invalidates, a new RREQ re-discovers (or fails cleanly if no alt path); `dropNoRoute`/RERR visible. |

### What to watch for
- **Loop freedom:** `dropTTL` should stay ~0 in a stable topology. Persistent
  growth means a routing loop (check destSeq handling in `route_update`).
- **Storm suppression:** `dupSup` rising modestly during discovery is normal;
  runaway growth means the seen-set isn't catching re-floods.
- **Duty budget:** at EU defaults a beacon every 60 s is ~10 s/h of the 36 s/h
  budget; if data sends start returning duty deferrals, lengthen `APP_TX_PERIOD_MS`.

---

## 8. Known simplifications (intentional; tighten later)

1. **RERR fan-out** uses precursor-unicast when precursors are known, else a
   single `TTL=1` broadcast. Local route repair (`RREQ_FLAG_REPAIR`) and
   gratuitous RREP (`RREQ_FLAG_GRATUITOUS`) are **not** implemented — add them
   in `on_rreq`/`rerr_for_broken` once the base mesh is proven.
2. **Result correlation** matches L2 tx-completions to local sends by `seq`
   only (unique per origin). Forwarded DATA is fire-and-forget at L3 (its
   failure still triggers RERR via the link event's `nextHop`).
3. **`mySeq` as destSeq:** the destination bumps `mySeq` when answering an RREQ
   (AODV rule). It is shared with the per-packet DATA seq counter, which is fine
   — both only ever increase and are compared with `seq_newer`.
4. **Symmetric-only next hops:** the spec says only `NEIGHBOR_FLAG_SYMMETRIC`
   neighbors should be route next hops. The code above accepts any learned
   reverse/forward hop; to enforce symmetry, gate `route_update`'s `nextHop` on
   `nbr_find(nextHop)->flags & NEIGHBOR_FLAG_SYMMETRIC` once beacons reliably
   carry neighbor lists in your deployment.
```
