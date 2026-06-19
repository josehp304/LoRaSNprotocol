/* =============================================================================
 * mesh_config.h  --  Shared compile-time configuration for the LoRa mesh stack
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY. No implementation. This header carries the verified
 * constants from the Ground-Truth Constraints Report and is included by every
 * layer (L1 PHY, L2 Data-Link, L3 Network, L4 App) so that all timing, sizing
 * and addressing decisions derive from a single source of truth.
 *
 * Target: Raspberry Pi Pico W (RP2040, no FPU) + Semtech SX1278 @ 433 MHz,
 *         driven by RadioLib (jgromes/RadioLib) over SPI, Arduino-pico core.
 *
 * Region note: defaults are EU 433 (ERC 70-03 / EN 300 220) compliant. The
 * REGION block is intended to be overridden at compile time (-D) per locale.
 * ===========================================================================*/
#ifndef MESH_CONFIG_H
#define MESH_CONFIG_H

#include <stdint.h>

/* ----------------------------------------------------------------------------
 * Region selection. Override with e.g. -D MESH_REGION_US in platformio.ini.
 * Only one region's TX power / duty rules apply at a time.
 * --------------------------------------------------------------------------*/
#if !defined(MESH_REGION_EU) && !defined(MESH_REGION_US)
#define MESH_REGION_EU 1   /* default to the conservative EU 433 ruleset */
#endif

/* ============================ PHY / RADIO ================================= */
/* These map 1:1 onto RadioLib SX1278::begin() arguments (see phy_sx1278.h).  */
#define LORA_FREQ_MHZ            433.5f   /* mid EU 433.05-434.79 band; 125kHz fits */
#define LORA_BW_KHZ              125.0f   /* design default bandwidth               */
#define LORA_SF                  9        /* 6-12 valid; 9 = 165ms/16B balance      */
#define LORA_CR_DENOM            5        /* 4/5 coding rate (RadioLib arg = 5)     */
#define LORA_SYNC_WORD           0x12     /* private (NOT 0x34 LoRaWAN); mesh-unique*/
#define LORA_PREAMBLE_SYM        8        /* preamble length in symbols             */
#define LORA_GAIN                0        /* 0 = AGC (auto LNA gain)                */

#if defined(MESH_REGION_EU)
  #define LORA_TX_POWER_DBM      10       /* +10 dBm = 10 mW e.r.p. -> EU compliant */
  #define REGION_DUTY_PERMILLE   10       /* 1% of 1000 -> conservative EU design   */
                                          /* (regulatory ceiling is 10%; we use 1%) */
#elif defined(MESH_REGION_US)
  #define LORA_TX_POWER_DBM      17       /* FCC Part 15 allows more; tune per app  */
  #define REGION_DUTY_PERMILLE   100      /* 10% placeholder; LBT required (see EU)  */
#endif

/* Duty-cycle accounting window. The L1/L2 duty governor tracks on-air time in a
 * rolling window of this length and refuses TX once the budget is exhausted.   */
#define DUTY_WINDOW_MS           3600000UL  /* 1 hour rolling window               */
/* Budget (ms of TX air time) per window = WINDOW * permille / 1000.            */
#define DUTY_BUDGET_MS           ((DUTY_WINDOW_MS / 1000UL) * REGION_DUTY_PERMILLE)

/* ============================ SPI / PINS (Pico W, SPI0) =================== */
/* Avoids GP23-25 / GP29 (tied to CYW43 wireless / onboard functions).        */
#define PIN_SCK    18
#define PIN_MOSI   19
#define PIN_MISO   16
#define PIN_CS     17
#define PIN_RST    20
#define PIN_DIO0   21    /* primary IRQ: TxDone / RxDone / CadDone             */
#define PIN_DIO1   22    /* REQUIRED for CAD_DETECTED (CSMA/LBT). Must be wired.*/

/* ============================ ADDRESSING / IDs =========================== */
/* node_id_t / seq_t are guarded with the same macros mesh_types.h uses so the
 * two headers never double-define (mesh_types.h is the canonical owner; these
 * remain here for older headers that include mesh_config.h alone). The richer
 * type set — bcast_id_t, link_cost_t, mesh_pkt_type_t, mesh_err_t, and the
 * seq_newer() helper — lives ONLY in mesh_types.h. Include it for those.      */
#ifndef MESH_NODE_ID_T_DEFINED
#define MESH_NODE_ID_T_DEFINED
typedef uint16_t node_id_t;   /* 16-bit node address                          */
#endif
#ifndef MESH_SEQ_T_DEFINED
#define MESH_SEQ_T_DEFINED
typedef uint16_t seq_t;       /* 16-bit per-origin sequence number, wraps mod 2^16 */
#endif

#define ADDR_INVALID   0x0000  /* reserved: unassigned / null                  */
#define ADDR_BROADCAST 0xFFFF  /* reserved: link broadcast (NextHop & Dst)     */

/* ---- Node role (governs sleep eligibility + forwarding behavior) ---------- */
#define ROLE_SENDER  0u   /* leaf: originates DATA; MAY sleep its radio        */
#define ROLE_RELAY   1u   /* router: forwards only; ALWAYS in startReceive     */
#define ROLE_SINK    2u   /* gateway: terminates DATA; ALWAYS in startReceive  */

/* ---- Regulatory region identifiers (see PHY / duty block above) ----------- */
#define REGION_EU433 0u
#define REGION_US433 1u

/* ---- Per-node identity. Override at flash time with -D in platformio.ini. -- */
/* Each physical node MUST be flashed with a unique NODE_ID. NODE_ROLE and     */
/* NODE_SINK_ADDR select behavior; sensible single-node defaults below.        */
#ifndef NODE_ID
#define NODE_ID        0x0001          /* unique per node; MUST override        */
#endif
#ifndef NODE_ROLE
#define NODE_ROLE      ROLE_SENDER     /* ROLE_SENDER | ROLE_RELAY | ROLE_SINK  */
#endif
#ifndef NODE_SINK_ADDR
#define NODE_SINK_ADDR 0x00FF          /* default sink/gateway address          */
#endif
#ifndef NODE_REGION
  #if defined(MESH_REGION_US)
  #define NODE_REGION  REGION_US433
  #else
  #define NODE_REGION  REGION_EU433
  #endif
#endif

/* ---- Dual-core sensor offload (core1 fills SPSC ring; see mesh_node.h) ----- */
#ifndef USE_CORE1_SENSOR
#define USE_CORE1_SENSOR 0             /* 1 = sample on core1; 0 = inline core0 */
#endif

/* seq_newer() — wrap-safe "a strictly newer than b" — is defined in
 * mesh_types.h (the canonical type header) to avoid a duplicate definition. */

/* ============================ FRAME GEOMETRY ============================= */
/* See mesh_frame.h for the full struct. Sizes are duplicated here as compile-
 * time constants so buffer arrays can be declared without pulling structs.    */
#define HDR_LEN        19       /* on-wire header bytes (incl. SeqNum + LinkSrc) */
#define CRC_LEN        2        /* trailing CRC-16 over header+payload          */
#define MAX_PAYLOAD    64       /* application payload cap (frame max is 255)   */
#define FRAME_OVERHEAD (HDR_LEN + CRC_LEN)        /* 21 B fixed per frame       */
#define MAX_PAYLOAD_PHY 234u    /* 255 - 19 - 2; hard RX-path ceiling           */
#define MAX_FRAME_LEN  255u     /* SX1278 FIFO ceiling (header+payload+CRC)     */

#define FRAME_MAGIC    0x4C6Du  /* 'Lm'; stored LITTLE-ENDIAN on wire (byte[0]=0x6D, byte[1]=0x4C) */
#define PROTO_VERSION  1u

/* ============================ L2 TIMING (tuned to SF9 airtime) =========== */
/* Airtime references (BW125, CR4/5, explicit hdr, CRC on), from AN1200.13:
 *   SF7 : 16B 51.5ms / 32B 71.9ms / 64B 118.0ms
 *   SF9 : 16B 164.9ms / 32B 246.8ms / 64B 390.1ms
 *   SF12: 16B 1318.9ms / 32B 1810.4ms / 64B 2793.5ms
 * Timeouts below are sized for the SF9 default. If SF is changed at compile
 * time, scale these or use the airtime estimator declared in phy_sx1278.h.    */
#define ACK_TIMEOUT_MS         600   /* > data(247)+ack(165)+turnaround+margin */
#define MAX_RETX               3     /* bounded retransmits per hop            */
#define RETX_BACKOFF_BASE_MS   500   /* ~1 data-airtime base                   */
#define RETX_JITTER_MS         400   /* random 0..400 ms added per retry       */
#define RETX_BACKOFF_EXP_CAP   4     /* cap exponential multiplier (2^4 = 16x) */

#define CAD_BACKOFF_MIN_MS     20    /* CSMA random backoff window (min)       */
#define CAD_BACKOFF_MAX_MS     200   /* CSMA random backoff window (max)       */
#define CAD_MAX_ATTEMPTS       5     /* give up CSMA after this many busy CADs  */

/* Duplicate-frame suppression cache (L2 link-level, distinct from L3 seen-set)*/
#define L2_DUP_CACHE_SIZE      16    /* recent (src,seq) pairs to suppress      */
#define L2_DUP_TTL_MS          5000  /* age out L2 dup entries                  */

/* ============================ L3 MESH (referenced for sizing) ============ */
#define DEFAULT_TTL            8u
#define BEACON_INTERVAL_MS     60000UL
#define BEACON_JITTER_MS       10000UL
#define BEACON_MAX_NBRS        6u    /* max neighbors advertised per beacon    */
#define NEIGHBOR_TIMEOUT_MS    180000UL   /* ~3 missed beacons => stale        */
#define ROUTE_EXPIRY_MS        300000UL
#define ACTIVE_ROUTE_LIFE_MS   300000UL   /* lifetime stamped into RREP        */
#define REVERSE_ROUTE_LIFE_MS  6000UL     /* reverse route held awaiting RREP  */
#define RREQ_RETRY             2u
#define RREQ_TIMEOUT_MS        3000UL
#define RREQ_START_TTL         2u    /* TTL ring-search: initial value        */
#define RREQ_TTL_INCREMENT     2u    /* widened by this on each retry         */
#define RREQ_MAX_TTL           DEFAULT_TTL
#define SEEN_ENTRY_TTL_MS      30000UL
#define QUEUE_ENTRY_TTL_MS     8000UL    /* buffered DATA dropped if no route  */
#define ROUTE_PRECURSORS       4u    /* max precursor slots per route entry   */

/* ============================ TABLE SIZES (static, no malloc) ============ */
#define MAX_NEIGHBORS          16u
#define MAX_ROUTES             32u
#define MAX_SEEN               64u
#define MAX_QUEUE              8u    /* store-and-forward depth               */
#define MAX_PENDING_RREQ       8u    /* concurrent outstanding route discoveries */

#endif /* MESH_CONFIG_H */
