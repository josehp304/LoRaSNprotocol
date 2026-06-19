/* =============================================================================
 * main.cpp  --  Thin entry point for the flood-mesh node (L1+L2+L3+L4).
 * -----------------------------------------------------------------------------
 * setup() -> node_setup(); loop() -> node_loop(), per the mesh_node.h facade.
 * The application layer (L4) now drives traffic: a sensor (ROLE_SENDER) reads
 * floats/GPS and sends telemetry to the sink; the base station (ROLE_SINK)
 * bridges to WiFi/MQTT. This file only prints a banner and a periodic stats
 * heartbeat; all behaviour lives below the facade.
 * ===========================================================================*/
#include <Arduino.h>
#include "mesh_config.h"
#include "node_identity.h"
#include "mesh_node.h"

#define STATE_DUMP_MS    60000UL   /* stats heartbeat                          */
#define SERIAL_WAIT_MS   3000UL    /* bounded wait for USB CDC                  */

static uint32_t s_nextDump = 0;

static const char *role_name(uint8_t r){
    switch (r){ case ROLE_SENDER: return "SENDER"; case ROLE_RELAY: return "RELAY";
                case ROLE_SINK: return "SINK"; default: return "?"; }
}

void setup(){
    Serial.begin(115200);
    uint32_t t0=millis();
    while (!Serial && (millis()-t0)<SERIAL_WAIT_MS){ /* spin; no delay() */ }

    Serial.println();
    Serial.println("=====================================================");
    Serial.println(" PicoLoRa flood mesh -- L1+L2+L3+L4");
    Serial.print  ("  node_id : 0x"); Serial.println(node_id(), HEX);
    Serial.print  ("  role    : "); Serial.print(role_name(NODE_ROLE));
    Serial.print  (" ("); Serial.print((int)NODE_ROLE); Serial.println(")");
    Serial.print  ("  sink    : 0x"); Serial.println(node_sink_addr(), HEX);
    Serial.println("=====================================================");

    node_setup();   /* halts internally on unrecoverable radio init failure     */

    s_nextDump = millis() + STATE_DUMP_MS;
    Serial.println("[RUN] entering loop()");
}

void loop(){
    node_loop();   /* pump L1+L2+L3 and the role's L4 app every iteration       */

    if ((int32_t)(millis()-s_nextDump) >= 0){
        s_nextDump += STATE_DUMP_MS;
        node_print_stats();
    }
}
