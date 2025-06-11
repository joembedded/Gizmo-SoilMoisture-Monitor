/**
 * @file ltx_rak3172_ext_at.ino
 * @brief LoRaWAN Sensor Firmware for RAK3172 modules - Extended AT for simple integration
 * @author JoEmbedded.de
 *
 **/

//--- User defines first, then headers ---
#define USAGE_EXT_AT     // USE as AT modem with extended commands
//#define LED_PIN PA4      // optionally Standard LED vs. VCC - This is PIN for State-LED (if defined)

//#define STD_ENERGY_RAK3172MODULE_3V3 // Standard and E/T/SIP
#define STD_ENERGY_RAK3172LSIP_3V3 // Low Power-SIP only


// --- Headers now charged for setup ---
#include "ltx_rak3172_user.h"  // Specific User Setup for this Device
#include "ltx_globaldefs.h"    // LTX common

//---------- MAIN ------------
void setup() {
	ltx_lib_setup();
}
void loop() {
	ltx_lib_loop();
}

// **
