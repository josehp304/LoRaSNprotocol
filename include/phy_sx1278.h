/* =============================================================================
 * phy_sx1278.h  --  L1 PHY interface: thin non-blocking driver over RadioLib
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY -- interface signatures, no implementation.
 *
 * Role: wrap RadioLib SX1278 so that L2 never touches the radio directly. This
 * layer owns the radio object, the DIO0 ISR, the radio state machine, the
 * RSSI/SNR snapshot, CAD (channel-busy) checks, sleep/standby, and the duty-
 * cycle air-time governor. It deals in RAW BYTE BUFFERS only -- it has no
 * knowledge of the mesh header, CRC, addressing, or ACKs (those are L2/L3).
 *
 * NON-BLOCKING CONTRACT
 * ---------------------
 *   * Only the RadioLib non-blocking calls are used (startTransmit /
 *     finishTransmit / startReceive / readData / startChannelScan) --
 *     never the blocking transmit() / receive() / scanChannel().
 *   * DIO0 and DIO1 ISRs each set a volatile flag. DIO0 = CadDone/TxDone/RxDone.
 *     DIO1 = CadDetected (asserted when a LoRa preamble is heard during CAD).
 *     Both ISRs do NOTHING else (no SPI). phy_poll() consumes both flags,
 *     finalizes RadioLib transactions, snapshots RSSI/SNR, and advances the
 *     PHY state machine. DIO1 (GP22) is REQUIRED — without it CSMA cannot
 *     distinguish channel-free from channel-busy after a CadDone event.
 *   * SX127x DIO0 signals TxDone/RxDone/CadDone depending on mode; RadioLib
 *     manages the DIO mapping inside startReceive/startTransmit/startChannelScan.
 *
 * THREADING / RP2040 DUAL CORE
 * ----------------------------
 *   The design runs the entire radio stack on ONE core (core0). The ISR fires
 *   on whichever core armed the interrupt -> keep all phy_*() calls on core0.
 *   The volatile flag is the only ISR<->loop shared state; reads/writes of a
 *   single 32-bit word are atomic on M0+. If L4 sensor work runs on core1, it
 *   hands frames to L2 via a lock-free SPSC ring (NOT defined here).
 * ===========================================================================*/
#ifndef PHY_SX1278_H
#define PHY_SX1278_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_config.h"
#include "mesh_frame.h"   /* for mesh_err_t + MESH_* codes */

/* Hard PHY transmit ceiling: the SX1278 FIFO holds one full L2 frame. Equal to
 * MAX_FRAME_LEN (header + payload + CRC). phy_start_transmit() rejects anything
 * larger with MESH_ERR_FRAME_TOO_LONG. */
#ifndef PHY_FIFO_MAX
#define PHY_FIFO_MAX  MAX_FRAME_LEN
#endif

/* ----------------------------------------------------------------------------
 * PHY operating state (radio state machine, read-only to upper layers).
 * --------------------------------------------------------------------------*/
typedef enum {
    PHY_STATE_UNINIT = 0, /* before phy_begin()                               */
    PHY_STATE_SLEEP,      /* radio asleep (lowest power)                      */
    PHY_STATE_STANDBY,    /* oscillator on, idle, ready to arm RX/TX/CAD      */
    PHY_STATE_RX,         /* continuous receive armed, waiting RxDone         */
    PHY_STATE_CAD,        /* channel-activity-detection in progress          */
    PHY_STATE_TX          /* transmit in flight, waiting TxDone               */
} phy_state_t;

/* ----------------------------------------------------------------------------
 * Result of a completed CAD scan (channel-busy check).
 * Maps RadioLib SX127x status after startChannelScan():
 *   RADIOLIB_PREAMBLE_DETECTED (-14) -> BUSY  (preamble heard on DIO1)
 *   RADIOLIB_CHANNEL_FREE      (-15) -> FREE  (no activity detected)
 *   anything else                    -> ERROR
 * Note: RADIOLIB_LORA_DETECTED (-702) is an SX126x code, NOT SX127x.
 * Gate CSMA on PREAMBLE_DETECTED vs CHANNEL_FREE for SX1278 correctness.
 * --------------------------------------------------------------------------*/
typedef enum {
    PHY_CAD_PENDING = 0,  /* scan still running                               */
    PHY_CAD_FREE,         /* channel clear -> caller may TX                   */
    PHY_CAD_BUSY,         /* LoRa preamble detected -> back off               */
    PHY_CAD_ERROR         /* radio error during scan                          */
} phy_cad_result_t;

/* ----------------------------------------------------------------------------
 * Link metrics of the most recently received packet. Snapshotted by phy_poll()
 * immediately after readData() so RSSI/SNR are coherent with the frame.
 * --------------------------------------------------------------------------*/
typedef struct {
    int16_t rssi_dbm;   /* getRSSI(packet=true) rounded to whole dBm          */
    int8_t  snr_db;     /* getSNR() rounded to whole dB                       */
    uint8_t valid;      /* 1 if a frame has been received since init          */
} phy_link_metric_t;

/* ----------------------------------------------------------------------------
 * Optional radio config override passed to phy_begin(). Pass NULL to use the
 * compile-time defaults from mesh_config.h.
 * --------------------------------------------------------------------------*/
typedef struct {
    float   freq_mhz;
    float   bw_khz;
    uint8_t sf;
    uint8_t cr_denom;     /* 5..8 -> 4/5..4/8                                  */
    uint8_t sync_word;
    int8_t  tx_power_dbm;
    uint16_t preamble_sym;
    uint8_t gain;         /* 0 = AGC                                          */
} phy_config_t;

/* ============================================================================
 * LIFECYCLE
 * ==========================================================================*/

/* Initialize SPI bus + SX1278 + DIO0 ISR. Applies `cfg` (or defaults if NULL).
 * Leaves the radio in PHY_STATE_STANDBY (not yet receiving).
 * Returns MESH_OK, or MESH_ERR_RADIO (raw RadioLib code via phy_last_radio_err)
 * if begin/SPI/chip-detect failed. */
mesh_err_t phy_begin(const phy_config_t *cfg /* nullable */);

/* Re-apply frequency at runtime (e.g. region change). MESH_OK / MESH_ERR_RADIO. */
mesh_err_t phy_set_frequency(float freq_mhz);

/* Re-apply TX power (clamped to region ceiling by caller). */
mesh_err_t phy_set_tx_power(int8_t dbm);

/* ============================================================================
 * MAIN-LOOP PUMP  (call exactly once per loop() iteration, non-blocking)
 * ----------------------------------------------------------------------------
 * Consumes the DIO0 flag, finalizes any completed TX/RX/CAD, snapshots link
 * metrics, advances the PHY state machine, and re-arms RX when idle. Returns:
 *   MESH_OK        -- nothing notable happened, or housekeeping only
 *   MESH_ERR_NO_RX -- (informational) idle, no frame available
 * It NEVER blocks and NEVER calls delay(). Errors from a finished transaction
 * are reported through the TX/RX accessor calls below, not thrown from poll.
 * ==========================================================================*/
mesh_err_t phy_poll(void);

/* ============================================================================
 * RECEIVE PATH
 * ==========================================================================*/

/* Arm continuous non-blocking receive (RadioLib startReceive). Idempotent if
 * already in RX. Transitions STANDBY/SLEEP -> RX. MESH_OK / MESH_ERR_RADIO. */
mesh_err_t phy_start_receive(void);

/* True if phy_poll() has finalized a freshly received raw frame that has not
 * yet been consumed via phy_read_frame(). Edge-triggered: cleared by read. */
bool phy_frame_available(void);

/* Copy the last received RAW frame (PHY payload = full L2 frame incl. CRC) into
 * `buf` (capacity `cap`). Writes the byte count to *out_len and the coherent
 * link metrics to *metric (nullable). Clears the available flag.
 * Returns: MESH_OK, MESH_ERR_NO_RX (nothing pending),
 *          MESH_ERR_FRAME_TOO_LONG (cap too small),
 *          MESH_ERR_RADIO (readData failed / PHY CRC if RadioLib CRC left on). */
mesh_err_t phy_read_frame(uint8_t *buf, uint16_t cap,
                          uint16_t *out_len, phy_link_metric_t *metric /*nullable*/);

/* Direct accessor for the last RX metrics (same snapshot as phy_read_frame). */
phy_link_metric_t phy_last_metric(void);

/* ============================================================================
 * TRANSMIT PATH
 * ----------------------------------------------------------------------------
 * Fire-and-forget at the PHY level: starts a non-blocking transmit. L2 is
 * responsible for CSMA (via the CAD calls below) and for ACK/retransmit logic.
 * The PHY only refuses TX for hard reasons (already transmitting, duty budget
 * exhausted, oversize, radio error).
 * ==========================================================================*/

/* Begin transmitting `len` raw bytes (must be <= PHY_FIFO_MAX). Transitions to
 * PHY_STATE_TX; completion is observed when phy_poll() sees TxDone and calls
 * phy_tx_done() true. Charges `len` air-time against the duty governor on start.
 * Returns: MESH_OK,
 *          MESH_ERR_TX_BUSY        (a TX already in flight),
 *          MESH_ERR_FRAME_TOO_LONG (len > PHY_FIFO_MAX),
 *          MESH_ERR_DUTY_EXCEEDED  (no air-time budget left this window),
 *          MESH_ERR_RADIO. */
mesh_err_t phy_start_transmit(const uint8_t *buf, uint16_t len);

/* True once phy_poll() has observed TxDone for the in-flight TX and called
 * finishTransmit(). After a true result the PHY auto-returns to RX (or the
 * state requested by phy_set_idle_mode). Edge-triggered: cleared on read. */
bool phy_tx_done(void);

/* Hard-abort an in-flight TX (rare; e.g. shutdown). Returns to STANDBY. */
mesh_err_t phy_abort_tx(void);

/* ============================================================================
 * CSMA / CAD  (channel-busy check; used by L2 before every unicast/broadcast TX)
 * ----------------------------------------------------------------------------
 * Non-blocking CAD: phy_start_cad() arms a single channel-activity scan; the
 * result lands via phy_poll() and is read with phy_cad_result(). L2's CSMA
 * state machine loops: start_cad -> poll until result -> if BUSY random-backoff
 * and retry (up to CAD_MAX_ATTEMPTS) -> if FREE start_transmit.
 * CAVEAT (documented): SX127x CAD detects only a LoRa *preamble*, not mid-packet
 * payload, so a node already transmitting past its preamble can be missed.
 * CSMA here is best-effort; pair with random backoff + per-hop ACK/retransmit.
 * ==========================================================================*/

/* Arm a single non-blocking CAD scan (RadioLib startChannelScan).
 * Transitions STANDBY/RX -> CAD. MESH_OK / MESH_ERR_RADIO. */
mesh_err_t phy_start_cad(void);

/* Latest CAD outcome. PHY_CAD_PENDING until phy_poll() finalizes the scan. */
phy_cad_result_t phy_cad_result(void);

/* ============================================================================
 * AIR-TIME ESTIMATOR  (Semtech AN1200.13) -- used by L2 for timeouts/backoff
 *                     and by the duty governor to charge air-time.
 * Pure integer math (NO FPU on RP2040): returns whole milliseconds, rounded up.
 * Uses the currently configured SF/BW/CR/preamble.
 * ==========================================================================*/

/* Estimated on-air time in ms for a frame of `frame_len` bytes (full L2 frame
 * including header+payload+CRC) at the current radio settings. */
uint32_t phy_airtime_ms(uint16_t frame_len);

/* Convenience: airtime of a minimum ACK-sized frame at current settings.
 * Used by L2 to compute ACK_TIMEOUT dynamically when SF != default. */
uint32_t phy_ack_airtime_ms(void);

/* ============================================================================
 * DUTY-CYCLE GOVERNOR  (regulatory air-time budget, rolling DUTY_WINDOW_MS)
 * ----------------------------------------------------------------------------
 * Tracks cumulative TX air-time in a rolling window. phy_start_transmit()
 * consults it and returns MESH_ERR_DUTY_EXCEEDED when the budget is spent.
 * L2/L3 can query remaining budget to schedule beacons/forwarding politely.
 * ==========================================================================*/

/* Remaining transmittable air-time (ms) in the current rolling window. */
uint32_t phy_duty_remaining_ms(void);

/* True if a frame of `frame_len` bytes would fit within the remaining budget. */
bool phy_duty_can_tx(uint16_t frame_len);

/* ============================================================================
 * POWER MANAGEMENT  (duty-cycling hooks for battery longevity)
 * ==========================================================================*/

/* Put radio to lowest-power sleep (config retained). Upper layers must call
 * phy_standby() or phy_start_receive() before any further TX/RX. */
mesh_err_t phy_sleep(void);

/* Bring radio to STANDBY (fast wake, oscillator on, ready to arm). */
mesh_err_t phy_standby(void);

/* Choose what state phy_poll() returns to after a completed TX/RX:
 * true  -> auto re-arm continuous RX (default, "always listening" mesh node);
 * false -> drop to STANDBY (duty-cycled node that sleeps between windows). */
void phy_set_auto_rx(bool enable);

/* ============================================================================
 * SHARED ISR FLAGS  (set in ISR, consumed in phy_poll)
 * ----------------------------------------------------------------------------
 * Declared here so the integration layer (mesh_node.cpp) can register the
 * ISR callbacks. Set volatile; never written outside the ISR.
 * ==========================================================================*/
extern volatile bool g_phy_rx_flag;    /* DIO0 fired in RX mode (RxDone)     */
extern volatile bool g_phy_tx_flag;    /* DIO0 fired in TX mode (TxDone)     */
extern volatile bool g_phy_cad_done;   /* DIO0 fired in CAD mode (CadDone)   */
extern volatile bool g_phy_cad_det;    /* DIO1 fired (CadDetected = BUSY)    */

/* ============================================================================
 * INTROSPECTION
 * ==========================================================================*/
phy_state_t phy_state(void);          /* current radio state machine state    */
int16_t     phy_last_radio_err(void); /* raw RadioLib code of last failure     */

#endif /* PHY_SX1278_H */
