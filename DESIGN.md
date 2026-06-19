# PicoLoRa Mesh — Design Document (Spec of Record)

> **Status:** L1–L4 implemented and running. L1/L2 + L3 (routing) + L4 (flood app) validated at 1 hop on two nodes; multi-hop (P5) and self-heal (P6) not yet field-tested. Power/duty (P7) not started. See §10 for the flood application layer.
>
> **⚠ Naming note:** several modules were renamed between design and implementation. The file/layer names in §1 and §5 below have been reconciled to the actual source tree (`phy_sx1278`, `datalink`, `mesh_l3`, `app_*`). Two design-era headers (`mesh_wire.h`, `mesh_app.h`) survive in `include/` but are **vestigial** — the live wire format is `mesh_frame.h` and the live L4 is the `app_*` files.
> **Hardware:** Raspberry Pi Pico W (RP2040) + SX1278 (RFM96W/RFM98W) @ 433 MHz, RadioLib, Arduino-pico (earlephilhower), PlatformIO.
> **Reviewed by:** automated adversarial review pass (radiolib-realism lens). See § 9 for resolved findings.

---

## 1. System Overview

A four-layer, fully custom, application-layer mesh routing protocol adapted from AODV for
low-bandwidth, high-loss LoRa links. Every node is simultaneously sender, router, and
potential sink depending on its compile-time `NODE_ROLE`.

### Layer diagram

```
       ┌──────────────────────────────────────────────────────┐
       │                   CORE 0  (radio + mesh)             │
       │                                                      │
  L4   │  APPLICATION   app_*.* (proto/     : sensor endpoints,│
       │                sensor/basestation)   ports, MQTT bridge│
       │     │  mesh_l3_send()              ▲ l4_deliver()    │
       │     ▼                              │                 │
  L3   │  NETWORK       mesh_l3.*           : neighbor table, │
       │                route table, RREQ/RREP/RERR, seen-set,│
       │                S&F queue, TTL/seq loop prevention     │
       │     │  link_send()                 ▲ on_rx callback  │
       │     ▼                              │                 │
  L2   │  DATA LINK     datalink.*          : framing, CRC-16,│
       │                CSMA/CA(CAD+LBT), per-hop ACK+retx,  │
       │                dup-suppress, RX FIFO                 │
       │     │  phy_start_transmit()        ▲ DIO0 flag poll  │
       │     ▼                              │                 │
  L1   │  PHY           phy_sx1278.*        : RadioLib SX1278,│
       │                DIO0 ISR→flag, RSSI/SNR, CAD, duty,  │
       │                sleep/standby                         │
       │     │  SPI + DIO0 + DIO1           ▲ DIO0/DIO1 IRQ  │
       └─────┼──────────────────────────────┼────────────────┘
             ▼                              │
       ┌─────────────────────────────────────────────────────┐
       │   SX1278  433.5 MHz / BW 125 kHz / SF9 / CR 4:5    │
       └─────────────────────────────────────────────────────┘

Cross-cutting headers (no layer owns them, all layers see them):
  mesh_config.h  mesh_types.h  mesh_frame.h  mesh_time.h  node_identity.h

Optional (build flag USE_CORE1_SENSOR):
  CORE 1 → sample sensor → SPSC ring → CORE 0 (no mesh tables touched by core1)
```

### Golden rules (binding)

1. **One radio owner.** All `phy_*`, `link_*`, `mesh_l3_*` calls run on core0.
2. **No `delay()`, no busy-wait.** Every layer is a `millis()`-driven, non-blocking state machine.
3. **No dynamic allocation after init.** All tables and queues are fixed-size static arrays.
4. **Wire integers are little-endian,** serialized field-by-field through explicit `HF_OFF_*` byte offsets. Never `memcpy` a packed struct as the wire definition.
5. **One duty accountant.** Lives in PHY (`phy_duty_*`). `power_mgr.h` reads it; does not maintain a second counter.
6. **Routers never sleep their radio.** Only `ROLE_SENDER` nodes (leaves) may duty-sleep the SX1278. Relays and sinks keep the radio in continuous `startReceive`.

---

## 2. Hardware Wiring

### SX1278 (SPI0 on Pico W)

| Signal  | Pico W GPIO | Notes                                  |
|---------|-------------|----------------------------------------|
| SCK     | GP18        | SPI0 default                           |
| MOSI    | GP19        | SPI0 default                           |
| MISO    | GP16        | SPI0 default                           |
| NSS/CS  | GP17        | RadioLib drives this                   |
| RESET   | GP20        |                                        |
| DIO0    | GP21        | Primary IRQ: TxDone / RxDone / CadDone |
| DIO1    | GP22        | **Required** for CAD_DETECTED (CSMA)   |

GP23–25 and GP29 are tied to the CYW43 wireless chip on Pico W — never use for radio.

**RadioLib construction:**
```cpp
SX1278 radio = new Module(/*cs*/17, /*dio0*/21, /*rst*/20, /*dio1*/22);
```

The SX1278 wiring above is present on **every** node (sensor, relay, sink).

### Sensor-node peripherals (ROLE_SENDER only)

A flood-sensor node adds the float ladder, buzzer/relay, and GPS. **The base
station (ROLE_SINK) and pure relays need none of these** — they only carry the
SX1278 (the sink also uses the Pico W's on-board WiFi, no extra pins). Pins are
defined in `app_config.h` (`APP_*_PIN`); the SX1278 pins live in `mesh_config.h`.

| Signal              | Pico W GPIO | Notes                                                      |
|---------------------|-------------|------------------------------------------------------------|
| Relay / buzzer      | GP15        | Active-LOW (LOW = ON). `OUTPUT`, idle HIGH.                |
| Float L1 (lowest)   | GP10        | GND-to-GPIO reed/float, `INPUT_PULLUP`, active-LOW.        |
| Float L2            | GP11        | "                                                          |
| Float L3            | GP12        | L3 is the buzzer threshold (`APP_WATER_THRESHOLD = 3`).    |
| Float L4            | GP13        | "                                                          |
| Float L5 (highest)  | GP14        | Highest asserted float wins (level = 5..1, else 0).        |
| GPS TX (Pico → NEO) | GP8         | UART1 (`Serial2` in the earlephilhower core).             |
| GPS RX (Pico ← NEO) | GP9         | NEO-6M TX → here. 9600 baud (`APP_GPS_BAUD`).             |

> **⚠ Pin reassignment vs. the legacy single-board firmware.** The old WiFi-only
> sensor put the 5 floats on **GP16–GP20** — exactly the SX1278 SPI pins (MISO 16,
> CS 17, SCK 18, MOSI 19, RST 20) in this design. They are therefore **moved to
> GP10–GP14**; rewire the float ladder accordingly. Relay (GP15) and GPS (GP8/GP9)
> were already clear of the radio and are unchanged.

Free GPIOs after radio + sensor peripherals (for future use): GP0–GP7, GP26–GP28.

### Note on chip marking

The **SX1278** (not SX1276) is the correct 433 MHz variant. RFM96W and RFM98W modules
use SX1278 silicon. RadioLib's `SX1278` class is the correct driver; its API is
identical to the SX1276 family.

---

## 3. Packet Format (wire definition)

### 3.1 Common header — 19 bytes, all multi-byte fields little-endian

| Offset | Size | Field        | Owner | Meaning                                                     |
|--------|------|--------------|-------|-------------------------------------------------------------|
| 0      | 2    | `magic`      | L2    | `0x4C6D` ("Lm"). Reject foreign 433 MHz traffic before CRC.|
| 2      | 1    | `version`    | L2    | `PROTO_VERSION=1`. Mismatch → drop, never forward.         |
| 3      | 2    | `src`        | L3    | Origin endpoint, immutable end-to-end. Half of dedup key.  |
| 5      | 2    | `dst`        | L3    | Final destination, immutable. `0xFFFF` = broadcast.        |
| 7      | 2    | `next_hop`   | L2/L3 | Link recipient this hop. Rewritten each forward. `0xFFFF` on floods. |
| 9      | 2    | `link_src`   | L2    | **One-hop sender (previous hop).** Each transmitter stamps its own id; `== src` on the origin hop, differs on forwards. Required for AODV reverse-route setup (L3 `linkFrom`). |
| 11     | 1    | `hop_count`  | L3    | Counts up from 0. Feeds routing metric.                    |
| 12     | 1    | `pkt_type`   | L3    | `MESH_PKT_*` enum.                                         |
| 13     | 1    | `ttl`        | L3    | Counts down; drop at 0.                                    |
| 14     | 2    | `seq`        | L3    | Per-origin sequence. Dedup key + AODV freshness.           |
| 16     | 1    | `flags`      | L2    | `FLAG_ACK_REQ`, `FLAG_IS_ACK`, `FLAG_RETX`.               |
| 17     | 1    | `reserved`   | —     | Set to 0. Future use / word alignment.                     |
| 18     | 1    | `payload_len`| L2    | Payload bytes following the header.                        |
| 19     | N    | `payload`    | L4/L3 | Type-specific content.                                     |
| 19+N   | 2    | `crc16`      | L2    | CRC-16/CCITT-FALSE over bytes [0 .. 19+N−1]. LE, trailing.|

`HDR_LEN = 19`, `CRC_LEN = 2`. On-air frame = `19 + payload_len + 2` bytes.

> **`link_src` rationale:** LoRa has no link-layer addressing, and `src`/`next_hop`
> name the *origin* and the *forward* recipient — neither identifies who physically
> transmitted a received copy. AODV reverse routes (and DATA precursor tracking)
> need that previous hop, so it is carried explicitly. Added during L3 bring-up;
> grew the header 17 → 19 bytes.

### 3.2 Length budget

```
PHY ceiling (SX1278 single FIFO)       : 255 bytes
Fixed overhead (19 header + 2 CRC)     :  21 bytes
MAX_PAYLOAD_PHY (hard ceiling, RX)     : 234 bytes
MAX_PAYLOAD     (application cap, TX)  :  64 bytes
```

TX rejects `payload_len > MAX_PAYLOAD`. RX drops if `payload_len > MAX_PAYLOAD_PHY`
or if `19 + payload_len + 2 ≠ getPacketLength()` (malformed frame).

### 3.3 Packet types

| Value | Name             | TTL | Flooded? | Link-ACKed? |
|-------|------------------|-----|----------|-------------|
| 0     | `MESH_PKT_DATA`  | n   | No       | Yes         |
| 1     | `MESH_PKT_BEACON`| 1   | No (bcast)| No         |
| 2     | `MESH_PKT_RREQ`  | n   | Yes      | No          |
| 3     | `MESH_PKT_RREP`  | n   | No       | Yes         |
| 4     | `MESH_PKT_RERR`  | 1   | Unicast or bcast | Yes if unicast |
| 5     | `MESH_PKT_ACK`   | 1   | No       | No (never ACK an ACK) |

### 3.4 Flags byte (offset 16)

| Bit  | Mask  | Name          | Meaning                                  |
|------|-------|---------------|------------------------------------------|
| 0    | 0x01  | `FLAG_ACK_REQ`| Sender requests per-hop L2 ACK (unicast only) |
| 1    | 0x02  | `FLAG_IS_ACK` | This frame is a link ACK (fast classify) |
| 2    | 0x04  | `FLAG_RETX`   | This is a retransmission (diagnostics)   |
| 7:3  | 0xF8  | reserved      | Must be 0                                |

`FLAG_ACK_REQ` is forced false for `next_hop == ADDR_BROADCAST`.

### 3.5 Per-type payload layouts

All payload structs are `MESH_PACKED` (`__attribute__((packed))`), all multi-byte fields LE.
The `static_assert` checks in `mesh_packet.h` guard layout drift. The **wire codec uses
field-by-field byte offsets**, not struct `memcpy`.

**DATA (0):** `app_port(1)` + opaque `app_data[]`
```c
typedef struct MESH_PACKED { uint8_t app_port; uint8_t app_data[]; } mesh_data_t;
```

**BEACON (1):** TTL=1, Dst=NextHop=BROADCAST, never forwarded
```
beacon_hdr_t  { nbr_count(1), flags(1) }
beacon_nbr_t  { node_id(2), link_cost(1) }   × nbr_count (max 6)
Total max = 2 + 6×3 = 20 bytes
```
Flags: `BEACON_FLAG_ROUTER=0x01`, `BEACON_FLAG_GATEWAY=0x02`, `BEACON_FLAG_LOWBATT=0x04`

RSSI/SNR are NOT in the beacon payload — the receiver measures them from PHY.
Symmetric-link detection: if `selfId` appears in a beacon's `nbrs[]`, set `NEIGHBOR_FLAG_SYMMETRIC`.
Only symmetric neighbors are eligible as a route next hop.
> **⚠ KNOWN GAP (spec ahead of code):** as of the current implementation
> `NEIGHBOR_FLAG_SYMMETRIC` is *set* on beacon RX (`mesh_l3.cpp`) but **not yet
> read** during route selection — `on_beacon` installs a direct 1-hop route for
> any beacon sender regardless of symmetry. The symmetric-only-next-hop rule is
> the intended behaviour and remains binding; the enforcement is outstanding.

**RREQ (2):** 14 bytes, flooded, Dst=NextHop=BROADCAST
```c
typedef struct MESH_PACKED {
    node_id_t  origin;           // == header src
    node_id_t  dest;             // destination being sought
    bcast_id_t bcast_id;         // == header seq (flood dedup key)
    seq_t      origin_seq;       // originator's seq (reverse-route freshness)
    seq_t      dest_seq;         // last-known destSeq of dest (0 if unknown)
    uint8_t    dest_seq_valid;
    uint16_t   metric;           // accumulated additive link cost
    uint8_t    flags;            // RREQ_FLAG_*
} mesh_rreq_t;
```
Flags: `RREQ_FLAG_GRATUITOUS=0x01`, `RREQ_FLAG_REPAIR=0x02`, `RREQ_FLAG_DEST_ONLY=0x04`

**RREP (3):** 14 bytes, unicast back along reverse path, NextHop rewritten each hop
```c
typedef struct MESH_PACKED {
    node_id_t dest;              // route destination
    node_id_t origin;            // RREQ originator (final recipient)
    seq_t     dest_seq;          // dest's authoritative seq (== header seq)
    uint8_t   hop_count;         // hops from dest accumulated so far
    uint16_t  metric;            // accumulated link cost from dest
    uint32_t  lifetime_ms;       // suggested forward-route lifetime
    uint8_t   flags;             // RREP_FLAG_*
} mesh_rrep_t;
```
Flags: `RREP_FLAG_GRATUITOUS=0x01`

**RERR (4):** up to 8 unreachable destinations
```c
typedef struct MESH_PACKED { uint8_t count; uint8_t flags; } rerr_hdr_t;
typedef struct MESH_PACKED {
    node_id_t dest;
    seq_t     dest_seq;          // INCREMENTED dest seq (advertises staleness)
} unreachable_dest_t;
// Total max = 2 + 8×4 = 34 bytes
```
`dest_seq` is incremented before advertising to prevent stale RERRs from killing newer routes.

**ACK (5):** 5 bytes, TTL=1, never forwarded, never ACKed
```c
typedef struct MESH_PACKED {
    node_id_t ack_src;           // header src of the frame being acknowledged
    seq_t     ack_seq;           // header seq of the frame being acknowledged
    uint8_t   status;            // MESH_ACK_OK or NAK reasons
} mesh_ack_t;
```
`status`: `MESH_ACK_OK=0x00`, `MESH_ACK_NAK_QUEUE=0x01`, `MESH_ACK_NAK_NOROUTE=0x02`

### 3.6 CRC specification

- **Algorithm:** CRC-16/CCITT-FALSE — poly `0x1021`, init `0xFFFF`, no reflection, no final XOR.
- **Coverage:** bytes [0 .. HDR_LEN + payload_len − 1] (header + payload, not the CRC itself).
- **Placement:** 2-byte trailer immediately after payload, stored LE (`crc & 0xFF`, `crc >> 8`).
- **RadioLib PHY CRC:** kept ON as independent defence-in-depth at a different layer.

---

## 4. Default Constants (single source of truth → `mesh_config.h`)

### Radio

| Constant           | Value       | Notes                                |
|--------------------|-------------|--------------------------------------|
| `LORA_FREQ_MHZ`    | `433.5f`    | Mid EU 433.05–434.79 MHz band        |
| `LORA_BW_KHZ`      | `125.0f`    |                                      |
| `LORA_SF`          | `9`         | ~165 ms/16 B frame; range balance    |
| `LORA_CR_DENOM`    | `5`         | 4/5 (RadioLib `cr` argument)         |
| `LORA_SYNC_WORD`   | `0x12`      | Private (not `0x34` LoRaWAN)         |
| `LORA_PREAMBLE_SYM`| `8`         |                                      |
| `LORA_GAIN`        | `0`         | AGC                                  |

### Region profile (compile-time `NODE_REGION`)

| Constant                | EU433 | US433 | Notes                                           |
|-------------------------|-------|-------|-------------------------------------------------|
| `LORA_TX_POWER_DBM`     | 10    | 17    | EU: +10 dBm e.r.p. cap (EN 300 220)            |
| `REGION_DUTY_PERMILLE`  | 10    | 100   | EU=1 % (36 s/h), US=10 % (360 s/h placeholder) |
| `DUTY_WINDOW_MS`        | 3600000 | 3600000 | Rolling 1-hour window                     |
| `LBT_REQUIRED`          | true  | true  | CAD/LBT before every TX                        |

**EU duty-cycle budget:** 36 000 ms/h total. A SF9 beacon (~170 ms) every 60 s costs
~10 200 ms/h, leaving ~25 800 ms/h for data traffic. At busy relays, monitor
`phy_duty_remaining_ms()` and defer non-critical frames.

### Frame geometry

| Constant         | Value | | Constant       | Value |
|------------------|-------|---|----------------|-------|
| `HDR_LEN`        | 19    | | `PROTO_VERSION`| 1     |
| `CRC_LEN`        | 2     | | `FRAME_MAGIC`  | 0x4C6D|
| `FRAME_OVERHEAD` | 21    | | `ADDR_BROADCAST`| 0xFFFF|
| `MAX_PAYLOAD`    | 64    | | `ADDR_INVALID` | 0x0000|
| `MAX_PAYLOAD_PHY`| 234   | | `MAX_FRAME_LEN`| 255   |

### L2 timing (tuned to SF9 airtimes)

| Constant              | Value   | Notes                                            |
|-----------------------|---------|--------------------------------------------------|
| `ACK_TIMEOUT_MS`      | 600     | data(247ms)+ack(165ms)+mode-switch+margin        |
| `MAX_RETX`            | 3       |                                                  |
| `RETX_BACKOFF_BASE_MS`| 500     | ≈ 1× data airtime                               |
| `RETX_JITTER_MS`      | 400     | random 0–400 ms per retry                        |
| `RETX_BACKOFF_EXP_CAP`| 4       | cap at 2⁴ × base                                |
| `CAD_BACKOFF_MIN_MS`  | 20      |                                                  |
| `CAD_BACKOFF_MAX_MS`  | 200     |                                                  |
| `CAD_MAX_ATTEMPTS`    | 5       | give up → `MESH_ERR_CHANNEL_BUSY`               |
| `L2_DUP_CACHE_SIZE`   | 16      |                                                  |
| `L2_DUP_TTL_MS`       | 5000    |                                                  |

### L3 mesh timing

| Constant               | Value   | | Constant              | Value |
|------------------------|---------|-|-----------------------|-------|
| `DEFAULT_TTL`          | 8       | | `RREQ_TIMEOUT_MS`     | 3000  |
| `BEACON_INTERVAL_MS`   | 60000   | | `RREQ_RETRY`          | 2     |
| `BEACON_JITTER_MS`     | 10000   | | `RREQ_START_TTL`      | 2     |
| `NEIGHBOR_TIMEOUT_MS`  | 180000  | | `RREQ_TTL_INCREMENT`  | 2     |
| `ROUTE_EXPIRY_MS`      | 300000  | | `RREQ_MAX_TTL`        | 8     |
| `ACTIVE_ROUTE_LIFE_MS` | 300000  | | `REVERSE_ROUTE_LIFE_MS`| 6000 |
| `SEEN_ENTRY_TTL_MS`    | 30000   | | `ROUTE_PRECURSORS`    | 4     |
| `QUEUE_ENTRY_TTL_MS`   | 8000    | | `BEACON_MAX_NBRS`     | 6     |

### Table sizes & RAM budget

| Structure              | Count | Per-entry | Subtotal |
|------------------------|-------|-----------|----------|
| `neighbor_entry_t`     | 16    | 16 B      | 256 B    |
| `routing_entry_t`      | 32    | 28 B      | 896 B    |
| `seen_entry_t`         | 64    | 8 B       | 512 B    |
| `pending_rreq_t`       | 8     | 16 B      | 128 B    |
| `queue_entry_t` (S&F)  | 8     | 84 B      | 672 B    |
| L2 TX queue (255 B frames) | 8 | ~268 B   | ~2144 B  |
| L2 dup cache           | 16    | 8 B       | 128 B    |
| RX/TX scratch buffers  | 2     | 255 B     | 510 B    |
| L3 scalars/counters    | —     | —         | ~20 B    |
| **Total mesh state**   |       |           | **≈ 5.3 KB** |

≈ 2% of 264 KB SRAM. Tables are sized for routing stability, not RAM pressure.

---

## 5. Module / File Layout

```
include/
  mesh_config.h      — all compile-time tunables (pins, radio, region, timing, sizes)
  mesh_types.h       — scalar typedefs, MESH_ENUM8, mesh_err_t, mesh_pkt_type_t, seq_newer()
  mesh_frame.h       — canonical wire format: HF_OFF_* offsets, header struct, codec APIs
  mesh_time.h        — millis() wrap-safe helpers (time_elapsed, deadline, etc.)
  node_identity.h    — node_id(), node_role(), node_region(), node_sink_addr(), node_next_seq()
  tx_queue.h         — store-and-forward ring buffer type + ops
  phy_sx1278.h       — L1: RadioLib SX1278 wrapper, DIO0+DIO1 ISRs, CAD, duty governor
  datalink.h         — L2: framing, CRC, CSMA/CA, per-hop ACK+retx, dup-suppress
  mesh_l3_types.h    — L3 data structures (neighbor/route/seen/pending/queue entries)
  mesh_l3.h          — L3 network: RREQ/RREP/RERR, forwarding, hooks API
  mesh_l3_control.h  — L3 control-frame layout notes
  app_proto.h        — L4: app wire protocol (telemetry + CAP codec, FNV-1a)
  app_sensor.h       — L4: flood-sensor node (ROLE_SENDER) entry points
  app_basestation.h  — L4: base station / gateway (ROLE_SINK) entry points
  app_config.h       — L4: app pins, timing, WiFi/MQTT creds, node registry
  power_mgr.h        — (P7, not yet implemented) duty accountant + power FSM
  scheduler.h        — cooperative millis() task table (header only; loop calls mesh_l3_tick directly)
  mesh_node.h        — top-level facade: node_setup() / node_loop() / core1_entry()
  (vestigial: mesh_wire.h, mesh_app.h — design-era stubs, not on the live path)

src/
  main.cpp           — thin entry: setup()→node_setup(), loop()→node_loop() + stats heartbeat
  node_identity.cpp  — compile-time identity accessors
  phy_sx1278.cpp     — L1 PHY implementation
  datalink.cpp       — L2 data-link implementation
  mesh_frame.cpp     — wire codec + CRC-16
  mesh_l3.cpp        — L3 routing/forwarding implementation
  mesh_node.cpp      — L2↔L3 glue + facade (hooks, node_setup/loop)
  app_proto.cpp      — L4 telemetry/CAP (de)serialization
  app_sensor.cpp     — L4 sensor role (floats, GPS, buzzer, telemetry, CAP storage)
  app_basestation.cpp— L4 sink role (WiFi/MQTT bridge, CAP downlink delivery)
```

### Include dependency direction (acyclic DAG)

```
mesh_config  ← mesh_types ← mesh_frame ← { tx_queue, mesh_time }
phy_sx1278   ← mesh_config, mesh_frame  (+ <RadioLib.h>)
datalink     ← phy_sx1278, mesh_frame, tx_queue, mesh_time
mesh_l3_types ← mesh_config, mesh_types
mesh_l3      ← datalink, mesh_l3_types, mesh_frame, node_identity, mesh_time
app_proto    ← mesh_config (pure logic; shared by sensor + basestation)
app_sensor   ← mesh_l3, app_proto, app_config, node_identity
app_basestation ← mesh_l3, app_proto, app_config, node_identity (+ WiFi/PubSubClient/ArduinoJson)
power_mgr    ← phy_sx1278, mesh_time, mesh_config   (P7, not yet implemented)
scheduler    ← mesh_types, mesh_time                (header only)
mesh_node    ← mesh_l3, datalink, phy_sx1278, app_sensor, app_basestation, app_proto, node_identity
```

No header ever includes a layer above it. Upward data flow via registered callbacks only.

---

## 6. Key Protocol Flows

### 6.1 Neighbor discovery (proactive BEACON)

1. `mesh_l3_tick()` emits a beacon when its `nextBeaconAt` deadline passes, then re-arms it to `now + BEACON_INTERVAL_MS + rand(0..BEACON_JITTER_MS)` (one-sided jitter → 60–70 s). (No separate scheduler task; `scheduler.h` is currently unused.)
2. Build beacon: `nbr_count`, `flags` (ROUTER/GATEWAY/LOWBATT), up to 6 `{node_id, link_cost}` entries for the freshest neighbors. Header: `Src=self`, `Dst=NextHop=BROADCAST`, `TTL=1`, `seq=mySeq`, `require_ack=false`.
3. L2 CSMA-sends once. Broadcast → never ACKed, never retransmitted.
4. On receipt: upsert `neighbor[src]` — `lastHeard`, `lastRssi/lastSnr` (snapshot from PHY on RX), `linkCost=mesh_link_cost(rssi,snr)`, `missed=0`, `VALID`.
5. Symmetric check: if `selfId` appears in `nbrs[]`, set `NEIGHBOR_FLAG_SYMMETRIC`. **Only symmetric neighbors are eligible as next hops.** (⚠ flag is set but not yet enforced in route selection — see the KNOWN GAP note in §3.5.)
6. Liveness (`mesh_l3_tick`): `++missed` each interval; `lastHeard > NEIGHBOR_TIMEOUT_MS` → evict, invalidate routes, emit RERR.

### 6.2 Route discovery (RREQ flood → RREP unicast)

**Originate** (no VALID route to `dest`):
1. `bcastId = myBcastId++`. Record `pending_rreq_t{dest, bcastId, ttl=RREQ_START_TTL, retries=0}`.
2. Broadcast RREQ. TTL ring-search: on `RREQ_TIMEOUT_MS` with no RREP, widen `ttl += RREQ_TTL_INCREMENT` up to `RREQ_MAX_TTL`, retry up to `RREQ_RETRY` times. On exhaustion: drop buffered DATA, notify L4.

**Forward** (recv RREQ, `self ≠ dest`):
1. **Dedup:** `(origin, bcastId)` in seen-set → **DROP**. Else insert.
2. **Reverse route:** install/refresh route toward `origin` (nextHop=linkFrom, metric from PHY RSSI/SNR).
3. **Answer or relay:**
   - `self == dest` → generate RREP.
   - Valid route to `dest` with fresh `destSeq ≥ rreq.dest_seq` → generate intermediate RREP.
   - Else relay: `--TTL`; if 0 **DROP**; `++HopCount`, `metric += linkCost`, re-broadcast.

**RREP:**
1. Generated by dest or intermediate node with a fresh route.
2. Each hop installs/refreshes the forward route to `rrep.dest`, records precursors, relays toward origin (unicast, `require_ack=true`).
3. On arrival at origin: install route, flush buffered DATA.

### 6.3 DATA forwarding (store-and-forward)

1. End-to-end dedup: `(src, seq)` in seen-set → DROP.
2. `Dst == self` → `l4_deliver(...)`. Done.
3. Forward: lookup route to `Dst`.
   - **VALID route:** `--TTL`; 0 → DROP (`++statDroppedTtl`). Else `++HopCount`, set `NextHop`, refresh route expiry, `require_ack=true`, hand to L2.
   - **No route:** buffer in S&F queue (`QSTATE_WAIT_ROUTE`), originate RREQ. Queue full → evict oldest `WAIT_ROUTE`.
4. Locally originated: same as forward, but `seq=node_next_seq()`, `HopCount=0`, `TTL=DEFAULT_TTL`.

### 6.4 Link failure / RERR

**Trigger:** L2 exhausts `MAX_RETX` → `L3_LINK_TX_FAILED`, or neighbor liveness timeout.
1. Find all VALID routes with `nextHop == broken_neighbor`.
2. Invalidate each: `state=INVALID`, `metric=LINK_COST_INFINITE`, **`destSeq += 1`** (anti count-to-infinity).
3. Collect `(dest, destSeq)` into RERR payload (max 8 entries; emit multiple if needed).
4. Send to all precursors (unicast, `TTL=1`). If no precursors recorded, broadcast `TTL=1`.
5. On receipt: if `nextHop==linkFrom` and our `destSeq` is older → invalidate, re-propagate RERR. destSeq gate stops RERR storms.

### 6.5 CSMA/CA (the RX/CAD/TX arbiter — critical)

Because the SX1278 cannot receive and run CAD simultaneously:

```
State: RADIO_RX (default)
  → Frame available (DIO0 ISR)? Read it (phy_read_frame), process, return to RADIO_RX.
  → TX requested?
      1. Radio → STANDBY (stops RX deaf window begins).
      2. phy_start_cad() → watch DIO0 (CadDone) AND DIO1 (CadDetected).
         - DIO1 asserted (CadDetected / PREAMBLE_DETECTED) → channel BUSY.
           Random backoff, return to RADIO_RX, retry after backoff.
         - DIO0 only, DIO1 quiet (CadDone, no CadDetected / CHANNEL_FREE) → clear.
      3. phy_start_transmit(frame). Wait TxDone (DIO0).
      4. Return to RADIO_RX (startReceive).
```

ACK frames: exempt from CAD (tiny, time-critical). Sent immediately after RxDone.
The "deaf window" during CAD is documented; L3 accounts for it in retransmit scheduling.

### 6.6 Duty-cycle / power

- Before every TX: `phy_duty_can_tx(frame_len)` → if budget spent, **defer** (never silently drop). Frame stays in queue until `phy_duty_remaining_ms() > 0`.
- `phy_start_transmit` charges estimated airtime; TxDone commits actual.
- Radio sleep allowed **only** for `ROLE_SENDER` (leaf) nodes with: queue empty, no ACK pending, `next_task_deadline - now > LISTEN_WINDOW_MS + margin`.
- Relays and sinks stay in continuous `startReceive`.

### 6.7 Loop prevention and broadcast-storm suppression (binding)

**TTL (hop-count loop bound):** every forward `--TTL`; drop at 0. Guards against persistent routing loops surviving a destSeq bug.

**Sequence-number freshness (AODV rule):** route update accepted only if the incoming destSeq is newer (`seq_newer(a,b) = (int16_t)(a-b) > 0`) or if same seq and lower hop count. RERR increments destSeq before advertising to ensure stale cached routes are evicted.

**Seen-set dedup (flood storm guard):** RREQ indexed by `(origin, bcastId)`. Any node seeing the same `(origin, bcastId)` a second time drops it — stops re-flooding a seen RREQ regardless of TTL.

**End-to-end dedup:** DATA and RREP indexed by `(src, seq)`. Prevents double-delivery at the destination and duplicate forwarding from two simultaneous next-hop candidates.

**RERR destSeq gate:** RERR for `(dest, destSeq)` only invalidates a route if `nextHop==linkFrom` AND the cached `destSeq` is not newer — prevents a stale RERR from killing a concurrently refreshed route.

**Sequence number wrap:** `seq_t` is `uint16_t`; wrap is handled by `seq_newer()` which uses `(int16_t)(a-b) > 0` — correct for any values within a half-window of 32768. Nodes receiving a sequence number that wraps past the half-window discard it as outdated (standard AODV convention).

---

## 7. Incremental Bring-Up Order

| Phase | What to implement              | What it exercises                                   |
|-------|--------------------------------|-----------------------------------------------------|
| **P1** | `mesh_phy.h` impl + SPI init + `startReceive`/`startTransmit` | Raw RadioLib bring-up, DIO0/DIO1 wiring, RSSI read |
| **P2** | `mesh_mac.h` + frame serialize/CRC | Point-to-point A→B with integrity (no routing) |
| **P3** | Add beacon TX/RX + neighbor table | Neighbor discovery, link-cost estimation |
| **P4** | Add S&F queue + TTL forward (no RREQ yet) | Manual next-hop relay: A→B→C static routes |
| **P5** | Add RREQ/RREP + routing table | Full on-demand route discovery |
| **P6** | Add RERR + neighbor liveness | Self-healing on link failure |
| **P7** | Add `power_mgr.h` + duty cycle | Battery longevity, duty compliance |
| **P8** | Add `USE_CORE1_SENSOR` + app layer | Real sensor data end-to-end |

---

## 8. RadioLib SX1278 API Quick-Reference

```cpp
// Construction
SX1278 radio = new Module(/*cs*/17, /*dio0*/21, /*rst*/20, /*dio1*/22);

// Init (all args have defaults)
int16_t err = radio.begin(433.5, 125.0, 9, 5, 0x12, 10, 8, 0);
// (freq_MHz, bw_kHz, sf, cr_denom, syncWord, power_dBm, preamble_sym, gain)

// Non-blocking RX
radio.setPacketReceivedAction([]() IRAM_ATTR { g_rx_flag = true; });
radio.startReceive();

// Read received frame after g_rx_flag
uint16_t len = radio.getPacketLength();
radio.readData(buf, len);
float rssi = radio.getRSSI();      // snapshot immediately after readData
float snr  = radio.getSNR();       // snapshot immediately after readData

// Non-blocking TX
radio.setPacketSentAction([]() IRAM_ATTR { g_tx_done = true; });
radio.startTransmit(buf, len);     // returns after loading FIFO

// CAD (CSMA) — requires DIO0 (CadDone) + DIO1 (CadDetected)
radio.startChannelScan();
// DIO0 fires: CadDone. Check DIO1 separately (CadDetected = channel BUSY)
// Or poll: int16_t r = radio.getChannelScanResult();
//   RADIOLIB_PREAMBLE_DETECTED → BUSY
//   RADIOLIB_CHANNEL_FREE      → clear to TX

// Power
radio.sleep();     // lowest power; must standby() before next TX/RX
radio.standby();   // STDBY mode, fast wake

// Error checking convention
if (err != RADIOLIB_ERR_NONE) { /* handle negative RADIOLIB_ERR_* code */ }
```

**Note on CAD return codes (review finding):** SX127x returns
`RADIOLIB_PREAMBLE_DETECTED` (not `RADIOLIB_LORA_DETECTED` which is SX126x). Gate
busy/free on `RADIOLIB_PREAMBLE_DETECTED` vs `RADIOLIB_CHANNEL_FREE`.

---

## 9. Resolved Review Findings

The automated adversarial review (radiolib-realism lens) found 3 issues, all incorporated
into this document and the header specifications:

| Severity | Area | Resolution |
|----------|------|------------|
| **Critical** | CAD DIO mapping | **DIO1 is required**, not optional. Wire GP22. Register `setDio1Action` for CAD_DETECTED. `phy_cad_result()` is only meaningful with DIO1 connected. Documented in §2 and §6.5. |
| **High** | CAD drops RX / sleeping routers | Explicit RX/CAD/TX arbiter defined in §6.5. Radio sleep restricted to `ROLE_SENDER` (leaf) nodes only — golden rule §1.6. |
| **High** | Multiple: duty budget; RSSI snapshot; CAD return code; ACK timeout; magic endianness comment | EU duty recomputed (§4); RSSI/SNR must be snapshotted immediately after `readData()` (§8); CAD code corrected to `RADIOLIB_PREAMBLE_DETECTED` (§8); `ACK_TIMEOUT_MS=600` includes mode-switch margin (§4); magic `0x4C6D` is stored LE on the wire, comments corrected throughout. |

---

## 10. Application Layer (L4) — Flood-Sensor Protocol

The flood application replaces the legacy single-board "sensor + WiFi/MQTT" firmware
with a split: many **sensor nodes** (`ROLE_SENDER`) over LoRa, and one **base
station** (`ROLE_SINK`) that bridges LoRa ↔ WiFi/MQTT. **Only the base touches
WiFi/MQTT** — every MQTT message in or out flows through it. Sensors keep the same
behaviour as the old firmware (float water-level, GPS, CAP-alert storage, buzzer),
but their uplinks/downlinks ride the mesh instead of WiFi.

### 10.1 Topology & MQTT bridging

```
sensor(s) ──LoRa mesh──► base(sink) ──WiFi/TLS──► HiveMQ
   telemetry  ──────────────────────► publish  rainfall/status
   CAP set    ◄────────────────────── subscribe cap-alert/<sensor-id>
                                                 flood-confirmed/<sensor-id>
```

- **Uplink:** sensor → base telemetry → base publishes JSON to `rainfall/status`.
  The over-the-air telemetry frame carries `gps_fix` and `cap_active` flags
  (§10.3), but the **currently published JSON contains only**
  `{ sensor_id, latitude, longitude, level }` — `gps_fix`/`cap_alert` are decoded
  at the base but not re-emitted to MQTT. (Add them to the publish in
  `app_basestation.cpp` if the dashboard needs them.)
- **Downlink:** base is subscribed to `cap-alert/<id>` and `flood-confirmed/<id>`;
  on receipt it maps `<sensor-id>` → mesh `node_id` (a compile-time registry),
  parses the CAP JSON, and unicasts a compact **CAP replace-set** to that node.

### 10.2 App ports (DATA payload byte 0)

| Port | Name              | Dir            | Purpose                                  |
|------|-------------------|----------------|------------------------------------------|
| 1    | `APP_PORT_TELEMETRY` | sensor → base | water level + flags + lat/long           |
| 2    | `APP_PORT_CAP`       | base → sensor | CAP replace-set (versioned)              |
| 3    | `APP_PORT_CAP_ACK`   | sensor → base | "applied set version V" (end-to-end ack) |
| 4    | `APP_PORT_CAP_SYNC`  | sensor → base | boot-time "re-send my CAP set"           |

### 10.3 Payload layouts (LE, field-by-field; cap = `MAX_PAYLOAD` 64)

```
TELEMETRY (11 B): port(1) flags(1: bit0=gps_fix,bit1=cap_active) water_level(1) lat(4 float) lon(4 float)
CAP set  (8 + N·12): port(1) subtype(1: 0=alert,1=flood) version(1) current_time(4 epoch) count(1)
                     then count × { id_hash(4) start_epoch(4) end_epoch(4) }, N ≤ 4
CAP_ACK  (2 B): port(1) version(1)
CAP_SYNC (1 B): port(1)
```

CAP storage on the sensor is **replace-set** semantics (the frame fully replaces
the stored list), identical to the old MQTT handler, and lives in **RAM only**.

### 10.4 Reliable downlink — end-to-end ack, not periodic resend

The legacy firmware re-published every CAP set on a 5-minute timer. The mesh
version is **ack-driven**:

1. Base receives an MQTT CAP update → bumps a **per-node `version`** → unicasts the
   set (L3 DATA, `wantsAck` so each hop is L2-ACKed + retransmitted).
2. Sensor applies it and returns a **`CAP_ACK`** carrying that `version`.
3. Base marks the node/version **acked and goes silent**. If no ack arrives it
   resends with exponential backoff (`APP_CAP_ACK_WAIT_MS` × 2ⁿ, capped at
   `APP_CAP_RETRY_CAP_MS`), giving up after `APP_CAP_MAX_ATTEMPTS`.

**Why an app-level ack on top of L2 per-hop ACK:** per-hop ACK only confirms the
*next* hop. On a multi-hop path `base → B → A`, B ACKs the base *before* it even
attempts A, and B's attempts to A are bounded (`MAX_RETX`) — B can give up, lose
the frame to a reboot, or hit a broken `B→A` link, all invisible to the base.
Hop reliability does **not** compose into path reliability; only an ack that
traversed the whole path `A → … → base` proves A received it. (For a 1-hop sensor
the per-hop ACK *is* end-to-end, so the app ack is redundant there — it earns its
keep on multi-hop and against reboots.)

### 10.5 Reboot resynchronization

Because CAP alerts are RAM-only, a power-cycled sensor would silently lose them —
and the base, already holding an ack, would not resend. So on boot the sensor
sends one **`CAP_SYNC`** (`APP_BOOT_SYNC_MS` after start); the base re-arms
delivery of that node's current set. This is what makes dropping the periodic
resend safe rather than merely cheaper.

### 10.6 Time, GPS, buzzer (unchanged from legacy)

Time priority is **base `current_time` (from a CAP set) > GPS UTC > millis()**.
A CAP alert is *active* when `start_epoch ≤ now ≤ end_epoch`; the buzzer/relay
(active-LOW) turns on when `water_level ≥ threshold` **and** a CAP alert is active.
GPS (NEO-6M) is read on a hardware UART for location + a backup time source.

### 10.7 Build / flash

```
pio run -e node_sink   -t upload --upload-port COMx   # base station (WiFi/MQTT)
pio run -e node_sender -t upload --upload-port COMy   # flood sensor
pio device monitor -b 115200
```