/* =============================================================================
 * phy_sx1278.cpp  --  L1 PHY implementation: non-blocking RadioLib SX1278 driver
 * -----------------------------------------------------------------------------
 * Implements the contract declared in phy_sx1278.h. This file OWNS the radio
 * object, the DIO0/DIO1 ISR flags, the PHY state machine, the RSSI/SNR
 * snapshot, CAD-based CSMA, sleep/standby, the integer air-time estimator, and
 * the rolling duty-cycle governor. It deals in RAW BYTE BUFFERS only and knows
 * nothing of the mesh header/CRC/addressing (those are L2/L3).
 *
 * NON-BLOCKING: only RadioLib's startXxx / finishTransmit / readData calls are
 * used here. The two ISRs (below) merely set volatile flags; phy_poll() does
 * all SPI work.
 *
 * RadioLib v7.x SX1278 API is used throughout. Where a v7.x symbol could not be
 * verified against the exact installed RadioLib headers, the most probable name
 * is used and tagged with "// TODO-VERIFY:" so the build step can flag it.
 * ===========================================================================*/

#include <RadioLib.h>
#include <Arduino.h>      /* SPI, millis(), pin/IRQ helpers (arduino-pico core) */
#include <SPI.h>
#include <string.h>       /* memcpy                                             */
#include <math.h>         /* lroundf for whole-unit RSSI/SNR rounding           */

#include "phy_sx1278.h"

/* ----------------------------------------------------------------------------
 * PHY_FIFO_MAX is referenced by the header's TX contract but is not given its
 * own #define in mesh_config.h. The SX1278 FIFO ceiling is MAX_FRAME_LEN (255).
 * Define it here from that single source of truth (do NOT hardcode 255).
 * --------------------------------------------------------------------------*/
#ifndef PHY_FIFO_MAX
#define PHY_FIFO_MAX  MAX_FRAME_LEN
#endif

/* ============================================================================
 * RADIO OBJECT
 * ----------------------------------------------------------------------------
 * The Module is the ONLY allocation made via `new`, and it happens at static-
 * init time (before setup()), as permitted by the project rules. Module wiring:
 *   new Module(NSS/CS, IRQ/DIO0, RST, GPIO/DIO1).
 * ==========================================================================*/
static SX1278 radio = new Module(PIN_CS, PIN_DIO0, PIN_RST, PIN_DIO1);

/* ============================================================================
 * SHARED ISR FLAGS  (declared extern in the header; defined here exactly once)
 * ----------------------------------------------------------------------------
 * Each is a single bool word -> atomic read/write on Cortex-M0+. Written ONLY
 * inside the ISRs; consumed (and cleared) in phy_poll().
 * ==========================================================================*/
volatile bool g_phy_rx_flag   = false;  /* DIO0 fired while in RX  (RxDone)    */
volatile bool g_phy_tx_flag   = false;  /* DIO0 fired while in TX  (TxDone)    */
volatile bool g_phy_cad_done  = false;  /* DIO0 fired while in CAD (CadDone)   */
volatile bool g_phy_cad_det   = false;  /* DIO1 fired (CadDetected = BUSY)     */

/* ============================================================================
 * MODULE-PRIVATE CACHED STATE  (read back through the introspection accessors)
 * ==========================================================================*/

/* Current radio state machine state. */
static volatile phy_state_t s_state = PHY_STATE_UNINIT;

/* Raw RadioLib status code of the most recent failing call (for diagnostics). */
static int16_t s_last_radio_err = RADIOLIB_ERR_NONE;

/* Coherent link metrics of the most recently received frame. */
static phy_link_metric_t s_metric = { 0, 0, 0 };

/* Outcome of the most recent CAD scan. */
static phy_cad_result_t s_cad_result = PHY_CAD_PENDING;

/* Edge-triggered availability/completion flags (cleared on read). */
static bool s_frame_available = false;   /* a finalized RX frame is buffered    */
static bool s_tx_done         = false;   /* an in-flight TX has completed        */

/* Where phy_poll() returns after a completed TX/RX: true=re-arm RX, false=STANDBY. */
static bool s_auto_rx = true;

/* ----------------------------------------------------------------------------
 * Active radio parameters. Seeded from mesh_config.h defaults in phy_begin();
 * the air-time estimator reads these so it tracks runtime SF/BW/CR/preamble.
 * Stored as integers where the formula needs them (kHz, symbol denom, etc.).
 * --------------------------------------------------------------------------*/
static uint8_t  s_sf           = LORA_SF;
static uint16_t s_bw_khz       = (uint16_t)LORA_BW_KHZ;   /* 125 etc.           */
static uint8_t  s_cr_denom     = LORA_CR_DENOM;           /* 5..8               */
static uint16_t s_preamble_sym = LORA_PREAMBLE_SYM;

/* ----------------------------------------------------------------------------
 * RX staging buffer. phy_poll() drains the radio FIFO into here on RxDone so the
 * caller can copy it out later with phy_read_frame(). Static (no malloc).
 * --------------------------------------------------------------------------*/
static uint8_t  s_rx_buf[MAX_FRAME_LEN];
static uint16_t s_rx_len = 0;

/* ----------------------------------------------------------------------------
 * Length of the TX frame currently in flight (informational only).
 * --------------------------------------------------------------------------*/
static uint16_t s_tx_len = 0;

/* ============================================================================
 * DUTY-CYCLE GOVERNOR STATE  (rolling DUTY_WINDOW_MS budget of air-time)
 * ----------------------------------------------------------------------------
 * A small ring of (timestamp, airtime_ms) records charged on each successful
 * TX start. phy_duty_remaining_ms() sums the air-time of records inside the
 * last DUTY_WINDOW_MS (wrap-safe unsigned subtraction) and subtracts from the
 * budget. The ring depth caps the number of distinct TX events tracked within
 * one window; oldest record is overwritten on overflow (its air-time is small
 * relative to the budget, so the approximation is conservative-enough).
 * ==========================================================================*/
#define DUTY_RING_SIZE 32u

typedef struct {
    uint32_t t_ms;        /* millis() at the moment TX was charged              */
    uint32_t air_ms;      /* charged air-time in ms                             */
} duty_rec_t;

static duty_rec_t s_duty_ring[DUTY_RING_SIZE];
static uint8_t    s_duty_head = 0;   /* index of next slot to write             */
static uint8_t    s_duty_count = 0;  /* number of valid records (<= ring size)  */

/* ============================================================================
 * ISR HANDLERS  (set a flag, nothing else -- NO SPI, NO RadioLib calls)
 * ----------------------------------------------------------------------------
 * DIO0 carries TxDone / RxDone / CadDone depending on the mode RadioLib armed.
 * We disambiguate by the current PHY state at interrupt time. DIO1 carries
 * CadDetected (LoRa preamble heard during a scan) -> channel busy.
 * On RP2040 a plain void(*)(void) is an acceptable ISR; we keep them minimal.
 *
 * IRAM_ATTR is an ESP-ism (place code in IRAM); arduino-pico does not define it.
 * Provide a portable no-op BEFORE the ISRs so the attribute expands cleanly on
 * RP2040 while remaining correct if ever built for an ESP target.
 * ==========================================================================*/
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

static void IRAM_ATTR phy_isr_dio0(void) {
    /* Disambiguate the shared DIO0 event by the state we are currently in. */
    switch (s_state) {
        case PHY_STATE_RX:  g_phy_rx_flag  = true; break;
        case PHY_STATE_TX:  g_phy_tx_flag  = true; break;
        case PHY_STATE_CAD: g_phy_cad_done = true; break;
        default:            /* spurious edge in STANDBY/SLEEP: ignore */ break;
    }
}

static void IRAM_ATTR phy_isr_dio1(void) {
    /* DIO1 == CadDetected: a LoRa preamble was heard -> channel is busy. */
    g_phy_cad_det = true;
}

/* ============================================================================
 * INTERNAL HELPERS
 * ==========================================================================*/

/* Record a RadioLib status as the last-error iff it indicates failure, and
 * return the same code so callers can `return record_err(st)`-style chain. */
static inline int16_t note_err(int16_t st) {
    if (st != RADIOLIB_ERR_NONE) {
        s_last_radio_err = st;
    }
    return st;
}

/* Arm continuous RX and update the cached state. Returns RadioLib status. */
static int16_t arm_receive(void) {
    int16_t st = radio.startReceive();
    if (st == RADIOLIB_ERR_NONE) {
        s_state = PHY_STATE_RX;
    } else {
        note_err(st);
    }
    return st;
}

/* Decide post-transaction idle behavior: re-arm RX, or drop to STANDBY. */
static void go_idle_after_txrx(void) {
    if (s_auto_rx) {
        if (arm_receive() != RADIOLIB_ERR_NONE) {
            /* RX re-arm failed: fall back to a known-safe standby state. */
            radio.standby();
            s_state = PHY_STATE_STANDBY;
        }
    } else {
        radio.standby();
        s_state = PHY_STATE_STANDBY;
    }
}

/* ============================================================================
 * LIFECYCLE
 * ==========================================================================*/
mesh_err_t phy_begin(const phy_config_t *cfg) {
    /* Resolve effective parameters: explicit cfg overrides compile-time defaults. */
    float    freq    = cfg ? cfg->freq_mhz     : LORA_FREQ_MHZ;
    float    bw      = cfg ? cfg->bw_khz       : LORA_BW_KHZ;
    uint8_t  sf      = cfg ? cfg->sf           : (uint8_t)LORA_SF;
    uint8_t  crd     = cfg ? cfg->cr_denom     : (uint8_t)LORA_CR_DENOM;
    uint8_t  sync    = cfg ? cfg->sync_word    : (uint8_t)LORA_SYNC_WORD;
    int8_t   pwr     = cfg ? cfg->tx_power_dbm : (int8_t)LORA_TX_POWER_DBM;
    uint16_t pre     = cfg ? cfg->preamble_sym : (uint16_t)LORA_PREAMBLE_SYM;
    uint8_t  gain    = cfg ? cfg->gain         : (uint8_t)LORA_GAIN;

    /* Bind SPI0 to the project pins BEFORE radio.begin() (arduino-pico SPI). */
    SPI.setSCK(PIN_SCK);
    SPI.setTX(PIN_MOSI);
    SPI.setRX(PIN_MISO);
    SPI.begin();

    /* Bring up the SX1278 with the full LoRa parameter set. */
    int16_t st = radio.begin(freq, bw, sf, crd, sync, pwr, pre, gain);
    if (st != RADIOLIB_ERR_NONE) {
        note_err(st);
        s_state = PHY_STATE_UNINIT;
        return MESH_ERR_RADIO;
    }

    /* Cache the parameters the air-time estimator depends on. */
    s_sf           = sf;
    s_bw_khz       = (uint16_t)bw;
    s_cr_denom     = crd;
    s_preamble_sym = pre;

    /* Wire the two interrupt actions. DIO0 = Tx/Rx/CadDone, DIO1 = CadDetected. */
    radio.setDio0Action(phy_isr_dio0, RISING);
    radio.setDio1Action(phy_isr_dio1, RISING);

    /* Clear any pre-existing edges and runtime state. */
    g_phy_rx_flag = g_phy_tx_flag = g_phy_cad_done = g_phy_cad_det = false;
    s_frame_available = false;
    s_tx_done         = false;
    s_cad_result      = PHY_CAD_PENDING;
    s_rx_len          = 0;
    s_tx_len          = 0;
    s_duty_head       = 0;
    s_duty_count      = 0;
    s_metric.valid    = 0;

    /* Leave the radio idle in STANDBY (NOT yet receiving), per the contract. */
    radio.standby();
    s_state = PHY_STATE_STANDBY;
    return MESH_OK;
}

mesh_err_t phy_set_frequency(float freq_mhz) {
    int16_t st = radio.setFrequency(freq_mhz);
    if (st != RADIOLIB_ERR_NONE) {
        note_err(st);
        return MESH_ERR_RADIO;
    }
    return MESH_OK;
}

mesh_err_t phy_set_tx_power(int8_t dbm) {
    int16_t st = radio.setOutputPower(dbm);
    if (st != RADIOLIB_ERR_NONE) {
        note_err(st);
        return MESH_ERR_RADIO;
    }
    return MESH_OK;
}

/* ============================================================================
 * MAIN-LOOP PUMP
 * ----------------------------------------------------------------------------
 * Consume whichever ISR flag is set, finalize the RadioLib transaction, snapshot
 * metrics, advance the state machine, and set the edge-triggered availability /
 * tx-done flags. Never blocks.
 * ==========================================================================*/
mesh_err_t phy_poll(void) {

    /* ---- TxDone -------------------------------------------------------------*/
    if (g_phy_tx_flag) {
        g_phy_tx_flag = false;
        /* Finalize the transmit (release the FIFO / restore registers). */
        int16_t st = radio.finishTransmit();
        if (st != RADIOLIB_ERR_NONE) {
            note_err(st);
        }
        s_tx_len = 0;
        s_tx_done = true;            /* edge for phy_tx_done() (cleared on read) */
        go_idle_after_txrx();        /* re-arm RX or drop to STANDBY            */
        return MESH_OK;
    }

    /* ---- RxDone -------------------------------------------------------------*/
    if (g_phy_rx_flag) {
        g_phy_rx_flag = false;

        /* Length of the just-received packet (PHY view of the L2 frame). */
        size_t len = radio.getPacketLength();
        if (len > sizeof(s_rx_buf)) {
            len = sizeof(s_rx_buf);  /* clamp to staging buffer capacity        */
        }

        int16_t st = radio.readData(s_rx_buf, len);
        if (st == RADIOLIB_ERR_NONE) {
            s_rx_len = (uint16_t)len;

            /* Snapshot link metrics coherently with this frame. RSSI/SNR are
             * floats in RadioLib; round to whole units for the integer fields. */
            float rssi = radio.getRSSI();        /* per-packet RSSI (dBm)        */
            float snr  = radio.getSNR();         /* per-packet SNR  (dB)         */
            s_metric.rssi_dbm = (int16_t)lroundf(rssi);
            s_metric.snr_db   = (int8_t)lroundf(snr);
            s_metric.valid    = 1;

            s_frame_available = true; /* edge for phy_frame_available()         */
        } else {
            /* CRC/length error from the PHY: drop the frame, keep listening. */
            note_err(st);
            s_rx_len = 0;
        }

        /* Honor the configured idle behavior (re-arm RX, or drop to STANDBY for
         * duty-cycled senders) exactly like the TX/CAD completion paths. After
         * draining the FIFO this re-arms continuous RX when s_auto_rx is set. */
        go_idle_after_txrx();
        return MESH_OK;
    }

    /* ---- CadDone -----------------------------------------------------------*/
    if (g_phy_cad_done) {
        g_phy_cad_done = false;

        /* Primary signal: DIO1 (CadDetected) latched during the scan -> BUSY.  */
        bool detected = g_phy_cad_det;
        g_phy_cad_det = false;

        /* Cross-check against the RadioLib scan result where available.        */
        int16_t res = radio.getChannelScanResult();    /* SX127x: PREAMBLE_DETECTED / CHANNEL_FREE */
        if (res == RADIOLIB_PREAMBLE_DETECTED) {
            s_cad_result = PHY_CAD_BUSY;
        } else if (res == RADIOLIB_CHANNEL_FREE) {
            s_cad_result = PHY_CAD_FREE;
        } else {
            /* Result code not one of the two expected values: trust DIO1. */
            s_cad_result = detected ? PHY_CAD_BUSY : PHY_CAD_FREE;
        }
        /* DIO1 always wins as "busy" if it fired, even if the result said free. */
        if (detected) {
            s_cad_result = PHY_CAD_BUSY;
        }

        /* CAD is one-shot; return to the configured idle behavior so the caller
         * can immediately TX (if FREE) or back off (if BUSY). */
        go_idle_after_txrx();
        return MESH_OK;
    }

    /* ---- Nothing happened ---------------------------------------------------*/
    if (s_state == PHY_STATE_STANDBY || s_state == PHY_STATE_SLEEP) {
        return MESH_ERR_NO_RX;       /* informational: idle, no frame pending   */
    }
    return MESH_OK;
}

/* ============================================================================
 * RECEIVE PATH
 * ==========================================================================*/
mesh_err_t phy_start_receive(void) {
    if (s_state == PHY_STATE_UNINIT) {
        return MESH_ERR_NOT_INIT;
    }
    if (s_state == PHY_STATE_RX) {
        return MESH_OK;              /* idempotent: already receiving           */
    }
    if (arm_receive() != RADIOLIB_ERR_NONE) {
        return MESH_ERR_RADIO;
    }
    return MESH_OK;
}

bool phy_frame_available(void) {
    return s_frame_available;
}

mesh_err_t phy_read_frame(uint8_t *buf, uint16_t cap,
                          uint16_t *out_len, phy_link_metric_t *metric) {
    if (buf == NULL || out_len == NULL) {
        return MESH_ERR_INVALID_ARG;
    }
    if (!s_frame_available) {
        return MESH_ERR_NO_RX;       /* nothing pending                         */
    }
    if (cap < s_rx_len) {
        /* Caller's buffer cannot hold the frame; leave it pending so a larger
         * read can retrieve it (do NOT clear the available flag). */
        return MESH_ERR_FRAME_TOO_LONG;
    }

    memcpy(buf, s_rx_buf, s_rx_len);
    *out_len = s_rx_len;
    if (metric != NULL) {
        *metric = s_metric;
    }

    s_frame_available = false;       /* edge consumed                           */
    return MESH_OK;
}

phy_link_metric_t phy_last_metric(void) {
    return s_metric;
}

/* ============================================================================
 * TRANSMIT PATH
 * ==========================================================================*/
mesh_err_t phy_start_transmit(const uint8_t *buf, uint16_t len) {
    if (buf == NULL || len == 0) {
        return MESH_ERR_INVALID_ARG;
    }
    if (s_state == PHY_STATE_UNINIT) {
        return MESH_ERR_NOT_INIT;
    }
    if (s_state == PHY_STATE_TX) {
        return MESH_ERR_TX_BUSY;     /* a transmit is already in flight         */
    }
    if (len > PHY_FIFO_MAX) {
        return MESH_ERR_FRAME_TOO_LONG;
    }

    /* Duty governor: refuse if this frame would overrun the rolling budget. */
    uint32_t air = phy_airtime_ms(len);
    if (air > phy_duty_remaining_ms()) {
        return MESH_ERR_DUTY_EXCEEDED;
    }

    int16_t st = radio.startTransmit(const_cast<uint8_t *>(buf), (size_t)len);
    if (st != RADIOLIB_ERR_NONE) {
        note_err(st);
        return MESH_ERR_RADIO;
    }

    /* Charge the estimate against the duty window on a SUCCESSFUL start. */
    s_duty_ring[s_duty_head].t_ms   = millis();
    s_duty_ring[s_duty_head].air_ms = air;
    s_duty_head = (uint8_t)((s_duty_head + 1) % DUTY_RING_SIZE);
    if (s_duty_count < DUTY_RING_SIZE) {
        s_duty_count++;
    }

    s_tx_len  = len;
    s_tx_done = false;
    s_state   = PHY_STATE_TX;
    return MESH_OK;
}

bool phy_tx_done(void) {
    bool v = s_tx_done;
    s_tx_done = false;               /* edge-triggered: cleared on read         */
    return v;
}

mesh_err_t phy_abort_tx(void) {
    /* Force the radio out of TX back to a known-safe standby. */
    int16_t st = radio.standby();
    if (st != RADIOLIB_ERR_NONE) {
        note_err(st);
    }
    s_tx_len  = 0;
    s_tx_done = false;
    g_phy_tx_flag = false;           /* discard any pending TxDone edge         */
    s_state = PHY_STATE_STANDBY;
    return (st == RADIOLIB_ERR_NONE) ? MESH_OK : MESH_ERR_RADIO;
}

/* ============================================================================
 * CSMA / CAD
 * ==========================================================================*/
mesh_err_t phy_start_cad(void) {
    if (s_state == PHY_STATE_UNINIT) {
        return MESH_ERR_NOT_INIT;
    }

    /* Clear the detection latch and pending result before arming the scan. */
    g_phy_cad_det  = false;
    g_phy_cad_done = false;
    s_cad_result   = PHY_CAD_PENDING;

    int16_t st = radio.startChannelScan();
    if (st != RADIOLIB_ERR_NONE) {
        note_err(st);
        s_cad_result = PHY_CAD_ERROR;
        return MESH_ERR_RADIO;
    }
    s_state = PHY_STATE_CAD;
    return MESH_OK;
}

phy_cad_result_t phy_cad_result(void) {
    return s_cad_result;
}

/* ============================================================================
 * AIR-TIME ESTIMATOR  (Semtech AN1200.13, INTEGER math, round up to whole ms)
 * ----------------------------------------------------------------------------
 * Tsym = 2^SF / BW  (seconds, with BW in Hz).  We keep everything in integer
 * microseconds to retain precision on the FPU-less M0+.
 *
 *   Tsym_us           = (2^SF * 1e6) / BW_Hz
 *   Tpreamble_us      = (preamble_sym * 4 + 17) * Tsym_us / 4   [(pre+4.25)*Tsym]
 *   payloadSymbNb     = 8 + max( ceil( (8*PL - 4*SF + 28 + 16*CRC - 20*IH)
 *                                      / (4*(SF - 2*DE)) ) * (CR+4), 0 )
 *                       where CR = cr_denom-4 (1..4), so (CR+4) == cr_denom.
 *   Tpayload_us       = payloadSymbNb * Tsym_us
 *   Tframe_us         = Tpreamble_us + Tpayload_us
 *   ms                = ceil(Tframe_us / 1000)
 *
 * Constants for this stack: CRC=1 (RadioLib CRC on), IH=0 (explicit header),
 * DE=1 iff (SF>=11 && BW==125kHz) else 0.
 * ==========================================================================*/
uint32_t phy_airtime_ms(uint16_t frame_len) {
    const uint32_t SF  = s_sf;
    const uint32_t PL  = frame_len;            /* full L2 frame length in bytes  */
    const uint32_t CRC = 1;                    /* RadioLib hardware CRC enabled  */
    const uint32_t IH  = 0;                    /* explicit header                */
    const uint32_t CRdenom = s_cr_denom;       /* 5..8 -> multiplies symbol count*/

    /* Low-data-rate optimization is mandatory for SF11/12 at BW125. */
    const uint32_t DE = ((SF >= 11) && (s_bw_khz == 125)) ? 1u : 0u;

    /* Tsym in microseconds: (2^SF) / BW_kHz * 1000  ==  (2^SF * 1000) / BW_kHz. */
    const uint32_t two_pow_sf = (uint32_t)1u << SF;          /* 2^SF             */
    /* Round Tsym up so we never under-estimate air-time. */
    const uint32_t tsym_us =
        (two_pow_sf * 1000u + (s_bw_khz - 1u)) / s_bw_khz;

    /* Preamble time = (preamble_sym + 4.25) * Tsym.
     * Use the 4*pre + 17 trick to express (pre + 4.25) in quarters, then /4. */
    const uint32_t pre_quarters = (uint32_t)s_preamble_sym * 4u + 17u; /* *4 of (pre+4.25) */
    /* Round up the division by 4. */
    const uint32_t tpreamble_us =
        (pre_quarters * tsym_us + 3u) / 4u;

    /* Numerator of the payload-symbol ceiling term (signed: can go negative). */
    const int32_t num =
        (int32_t)(8u * PL) - (int32_t)(4u * SF) + 28
        + (int32_t)(16u * CRC) - (int32_t)(20u * IH);

    /* Denominator: 4*(SF - 2*DE), always positive for valid SF (>=6). */
    const int32_t den = (int32_t)(4u * (SF - 2u * DE));

    /* ceil(num/den) but clamped at >= 0 (the max(...,0) in the spec). */
    int32_t ceil_term;
    if (num <= 0) {
        ceil_term = 0;
    } else {
        ceil_term = (num + den - 1) / den;     /* ceil for positive operands     */
    }

    /* payloadSymbNb = 8 + ceil_term * CRdenom   (CRdenom == CR+4). */
    const uint32_t payload_symb = 8u + (uint32_t)ceil_term * CRdenom;

    const uint32_t tpayload_us = payload_symb * tsym_us;
    const uint32_t tframe_us   = tpreamble_us + tpayload_us;

    /* Convert microseconds to milliseconds, rounding UP. */
    return (tframe_us + 999u) / 1000u;
}

uint32_t phy_ack_airtime_ms(void) {
    /* A minimum frame is the header + CRC with zero payload. */
    return phy_airtime_ms((uint16_t)(HDR_LEN + CRC_LEN));
}

/* ============================================================================
 * DUTY-CYCLE GOVERNOR
 * ==========================================================================*/
uint32_t phy_duty_remaining_ms(void) {
    const uint32_t now = millis();
    uint32_t used = 0;

    /* Sum air-time of every record whose age is within the rolling window.
     * Wrap-safe: (now - t) is correct under unsigned arithmetic across overflow. */
    for (uint8_t i = 0; i < s_duty_count; ++i) {
        const duty_rec_t *r = &s_duty_ring[i];
        if ((uint32_t)(now - r->t_ms) < (uint32_t)DUTY_WINDOW_MS) {
            used += r->air_ms;
        }
    }

    if (used >= (uint32_t)DUTY_BUDGET_MS) {
        return 0;                    /* budget spent (clamped >= 0)             */
    }
    return (uint32_t)DUTY_BUDGET_MS - used;
}

bool phy_duty_can_tx(uint16_t frame_len) {
    return phy_airtime_ms(frame_len) <= phy_duty_remaining_ms();
}

/* ============================================================================
 * POWER MANAGEMENT
 * ==========================================================================*/
mesh_err_t phy_sleep(void) {
    int16_t st = radio.sleep();
    if (st != RADIOLIB_ERR_NONE) {
        note_err(st);
        return MESH_ERR_RADIO;
    }
    s_state = PHY_STATE_SLEEP;
    return MESH_OK;
}

mesh_err_t phy_standby(void) {
    int16_t st = radio.standby();
    if (st != RADIOLIB_ERR_NONE) {
        note_err(st);
        return MESH_ERR_RADIO;
    }
    s_state = PHY_STATE_STANDBY;
    return MESH_OK;
}

void phy_set_auto_rx(bool enable) {
    s_auto_rx = enable;
}

/* ============================================================================
 * INTROSPECTION
 * ==========================================================================*/
phy_state_t phy_state(void) {
    return s_state;
}

int16_t phy_last_radio_err(void) {
    return s_last_radio_err;
}
