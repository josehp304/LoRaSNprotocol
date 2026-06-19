/* =============================================================================
 * mesh_node.h  --  Top-level node facade.
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY.
 *
 * mesh_node.h is the ONLY header that src/main.cpp needs to include. It wires
 * together all layers:
 *
 *   node_setup()  called from Arduino setup():
 *     1. Initialize SPI and SX1278 via phy_begin() with compile-time config.
 *     2. Initialize L2 (link_begin) with node_id() and L3 callbacks.
 *     3. Initialize L3 (mesh_l3_init) with node_id() and L2/L4 hooks.
 *     4. Initialize L4 (app_init) with sink address.
 *     5. Initialize power_mgr and scheduler.
 *     6. Register ISR callbacks for DIO0 and DIO1.
 *     7. If USE_CORE1_SENSOR: start core1 running core1_entry().
 *
 *   node_loop()  called from Arduino loop():
 *     1. Consume g_phy_rx_flag: if set, call link_poll() which calls phy_poll()
 *        → reads frame → deserialized → L3 callback → L3 routes/delivers.
 *     2. Call link_poll() unconditionally (drains TX queue, drives CSMA/ACK).
 *     3. Call sched_run() (fires BEACON, L3 tick, app sample as due).
 *     4. Call power_mgr_step() (may sleep radio for SENDER role).
 *
 * GLUE (the hook adapters wiring L3 ↔ L2 and L3 ↔ L4):
 *   These are thin static functions defined in mesh_node.cpp (not exposed as
 *   public API). They translate between l3_tx_frame_t / link_tx_req_t and
 *   l3_rx_frame_t / mesh_rx_frame_t without any mesh logic of their own.
 *
 * LAYER OWNERSHIP of error handling:
 *   phy_begin() failure → Serial.println + halt (unrecoverable at startup).
 *   link_begin() failure → same.
 *   Runtime RadioLib errors → phy_last_radio_err(); logged, not halted.
 * ===========================================================================*/
#ifndef MESH_NODE_H
#define MESH_NODE_H

#include "mesh_config.h"
#include "mesh_types.h"
#include "mesh_time.h"
#include "node_identity.h"
#include "phy_sx1278.h"      /* PHY (a.k.a. "mesh_phy" in the blueprint)        */
#include "datalink.h"
#include "mesh_l3.h"
#include "mesh_app.h"
#include "power_mgr.h"
#include "scheduler.h"

/* ── Main entry points ───────────────────────────────────────────────────── */

/* Call from Arduino setup(). Initializes all layers in dependency order.
 * Panics (infinite loop with Serial error) on unrecoverable radio init failure.*/
void node_setup(void);

/* Call from Arduino loop(). Non-blocking; drives all layer state machines. */
void node_loop(void);

/* ── Core1 sensor offload (USE_CORE1_SENSOR=1 only) ─────────────────────── */
#if USE_CORE1_SENSOR

#define SAMPLE_RING_CAP 4

typedef struct {
    float    temp_c;       /* temperature in °C (float is fine on core1 side) */
    float    humidity;     /* relative humidity % */
    uint32_t sample_ms;    /* millis() of sampling (set by core1; approximate) */
    uint16_t seq;          /* core1-local monotone counter (for loss detection) */
} sensor_sample_t;

/* Lock-free single-producer single-consumer ring between core1 (writer)
 * and core0 (reader). Indices are volatile; items are word-size so writes
 * are atomic on M0+. Producer: write buf[w & mask], then ++w. Consumer:
 * if r != w, read buf[r & mask], then ++r. */
typedef struct {
    sensor_sample_t buf[SAMPLE_RING_CAP];
    volatile uint8_t w;   /* write index (core1 owns) */
    volatile uint8_t r;   /* read index  (core0 owns) */
} sample_ring_t;

/* Core1 entry point (passed to multicore_launch_core1 in node_setup). */
void core1_entry(void);

/* Pop one sample from the ring into *out. Returns true if a sample was
 * available. Call from core0 (app_tick). */
bool sample_ring_pop(sample_ring_t *ring, sensor_sample_t *out);

/* Access the shared ring (single global instance in mesh_node.cpp).       */
sample_ring_t *node_sample_ring(void);

#endif /* USE_CORE1_SENSOR */

/* ── Task index constants (for sched_set_enabled / sched_trigger_now) ───── */
#define TASK_IDX_L3_TICK     0u
#define TASK_IDX_APP_SAMPLE  1u

/* ── Introspection (diagnostics / serial monitor) ───────────────────────── */
void node_print_stats(void);   /* print link_stats + mesh_l3_state to Serial */

#endif /* MESH_NODE_H */
