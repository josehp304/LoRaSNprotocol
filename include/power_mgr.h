/* =============================================================================
 * power_mgr.h  --  Duty-cycle accountant + radio/MCU power state machine.
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY.
 *
 * Manages two orthogonal concerns:
 *   1. DUTY-CYCLE ACCOUNTING: wraps phy_duty_remaining_ms() and enforces the
 *      regulatory TX budget (EU=1%, US=10%). The duty accountant lives in the
 *      PHY (phy_duty_*); this module reads it and enforces deferral policy.
 *
 *   2. POWER FSM: governs when the radio transitions between RX / STANDBY /
 *      SLEEP, and when the MCU itself may enter a low-power wait state. This
 *      is separate from the duty accountant (duty ≠ power mode).
 *
 * KEY RULES (from DESIGN.md §1 golden rules):
 *   - Radio sleep is ONLY allowed for ROLE_SENDER (leaf) nodes.
 *   - ROLE_RELAY and ROLE_SINK keep the radio in continuous startReceive.
 *   - No sleep while a TX is in flight or an ACK timer is armed.
 *   - `power_mgr_step` is called once per loop() from node_loop().
 * ===========================================================================*/
#ifndef POWER_MGR_H
#define POWER_MGR_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_config.h"
#include "mesh_types.h"

/* ── Power FSM states ────────────────────────────────────────────────────── */
typedef MESH_ENUM8(power_state_e) {
    PWR_BOOT = 0,       /* before phy_begin() succeeds                        */
    PWR_LISTEN,         /* radio in startReceive (default, always for router) */
    PWR_TX_ACTIVE,      /* TX or ACK-wait in progress (no sleep allowed)      */
    PWR_IDLE_STANDBY,   /* radio in standby (brief gap between RX/TX)         */
    PWR_SLEEP_RADIO,    /* radio in sleep(), SENDER only, between listen windows */
    PWR_DUTY_BLOCKED    /* TX would exceed budget; radio stays RX, queued     */
} power_state_t;

/* ── Wake source (why power_mgr_step transitioned out of sleep) ──────────── */
typedef MESH_ENUM8(wake_src_e) {
    WAKE_NONE       = 0,
    WAKE_DIO0_RX,       /* RxDone ISR woke the node                           */
    WAKE_TIMER,         /* millis() deadline expired                           */
    WAKE_TX_REQUEST     /* L3/L2 has a frame to send                          */
} wake_src_t;

/* ── Region profile (compile-time or runtime override) ──────────────────── */
typedef struct {
    int8_t   tx_power_dbm;      /* capped by region ceiling                   */
    uint16_t duty_permille;     /* EU=10 (1%), US=100 (10%)                   */
    uint32_t duty_window_ms;    /* rolling window (3600000 = 1 hour)          */
    bool     lbt_required;      /* listen-before-talk (CAD) required          */
} region_profile_t;

/* ── power_mgr_t: opaque state (single static instance in power_mgr.cpp) ── */
typedef struct {
    power_state_t state;
    uint32_t      listen_start_ms;    /* entered PWR_LISTEN at this time      */
    uint32_t      sleep_until_ms;     /* wakeup deadline for PWR_SLEEP_RADIO  */
    uint8_t       role;               /* cached NODE_ROLE                     */
    bool          ack_pending;        /* true while an ACK timer is armed     */
    bool          tx_inflight;        /* true while a TX is in progress       */
} power_mgr_t;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/* Initialize the power manager. Must be called after phy_begin().
 * `rp` is the region profile (nullable -> build-time defaults from config).
 * `role` is NODE_ROLE (ROLE_SENDER/RELAY/SINK). */
void power_mgr_init(power_mgr_t *p, const region_profile_t *rp,
                    uint8_t role, uint32_t now_ms);

/* ── Main-loop pump ──────────────────────────────────────────────────────── */

/* Called once per loop() from node_loop(). Evaluates the current power
 * state and may:
 *   - transition to PWR_SLEEP_RADIO if conditions are met (SENDER role only),
 *   - wake the radio on g_phy_rx_flag or scheduler deadline,
 *   - defer blocked TX to next duty window.
 * `next_task_deadline_ms` is the scheduler's sched_next_deadline() return
 * (used to determine how long it is safe to sleep). */
void power_mgr_step(power_mgr_t *p, uint32_t now_ms,
                    uint32_t next_task_deadline_ms);

/* Query current state (read-only for L2/L3 to decide sleep eligibility).  */
power_state_t power_mgr_state(const power_mgr_t *p);

/* ── Duty-cycle helpers (thin wrappers over phy_duty_*) ─────────────────── */

/* True if a frame of `frame_len` bytes can be transmitted within budget.  */
bool duty_can_tx(uint16_t frame_len, uint32_t now_ms);

/* Notify the power manager that a TX just started (charges the governor).
 * Called by L2 immediately before phy_start_transmit(). */
void duty_commit_tx(uint32_t airtime_est_ms, uint32_t now_ms);

/* Remaining transmittable air-time (ms) in the current rolling window.    */
uint32_t duty_remaining_ms(uint32_t now_ms);

/* ── ACK / in-flight hooks (called by L2) ───────────────────────────────── */

/* Signal that a unicast TX is in flight (prevent sleep).                  */
void power_mgr_set_tx_inflight(power_mgr_t *p, bool inflight);

/* Signal that an ACK timer is armed (prevent sleep).                      */
void power_mgr_set_ack_pending(power_mgr_t *p, bool pending);

#endif /* POWER_MGR_H */
