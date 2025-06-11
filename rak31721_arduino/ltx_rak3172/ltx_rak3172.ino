#include <ltx_globaldefs.h>
#include <ltx_rak3172_user.h>

/**
 * @file ltx_rak3172_simple_node.ino
 * @brief LoRaWAN Sensor Firmware for RAK3172 modules - A simple node
 * @author JoEmbedded.de
 *
 * .INO copied from /templates/
 *
 **/

//--- User defines first, then headers ---
#define USAGE_STANDALONE // Single-Chip Stand alone solution
#define USAGE_EXT_AT     // optionally USE as AT modem with extended commands
#define LED_PIN PA4      // optionally Standard LED vs. VCC - This is PIN for State-LED (if defined)

//#define STD_ENERGY_RAK3172MODULE_3V3 // Standard and E/T/SIP
#define STD_ENERGY_RAK3172LSIP_3V3 // Low Power-SIP only

#define DEVICE_TYPE 8001         // Typically 8000: Basic, choose your own value for your application.
#define DEV_FAMILY "LTX-RAK3172-BasicSimpleNode" // Name of the application
#define DEVICE_FW_VERSION 1      // Steps of 0.1 (e.g. 32 is Firmware V3.2)

#define HK_FLAGS 11        // HK values 1:BatVolt 2:Temp (4:Humidity) 8:Energy (16:Barometer)
#define ANZ_KOEFF 8        // Number of coefficients for this application
#define MAX_CHANNELS 4     // Maximum number of channels. For F16: 20 channels occupy 2+40 bytes payload plus optionally 1+3 bytes per HK channel
// 3V3: LTX calculates the consumed energy in mAH. Values for RAK3172L-SIP:
#define MESSEN_ENERGY 1000 // uC - Assume 10mA for 100 msec (= 1 mC)


// --- Headers now charged for setup ---
#include "ltx_rak3172_user.h"  // Specific User Setup for this Device
#include "ltx_globaldefs.h"    // LTX common

// ---- User stuff for USAGE_STANDALONE ----
/* Watchdog is recommended. If disabled: interval max. 120 sec, otherwise max. 30 sec */
PARAM param = { _PMAGIC , "_measure_command_", 3600, 5, 1, /*WD*/ true, { 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0 } };
const char *koeff_desc[ANZ_KOEFF] = {
  "Multiplier Channel #0", "Offset Channel #0",
  "Multiplier Channel #1", "Offset Channel #1",
  "unused", "unused",
  "unused", "unused"
};

// User routines - Setup/Measure
#define ANAPIN1 PB4  // - on SIP-Eval at the back! Init as Output!

void user_setup(void) {
  analog_setup();               // Enable analog routines
  analog_read_single(ANAPIN1);  // Force Port2 to Analog mode
}

/* Simulated measurement
* Feeding the watchdog required if takes >1 sec 
* ADC measures VCC as reference and has bandgap VREF 
* as channel with stored value for 3V3.
* First, 'user_measure_values()' is executed. 
* If HK values are needed, they can be cached here.
* The string 'param.cmd' may be used as measurment command.
*/
float modvcc;             // Module's VCC
uint32_t user_count = 1;  // Simply increment
// Here: 2 Channels: 1 analog and other simply counts or displays error
int16_t user_measure_values(int16_t ival) {
  uint16_t rcali3v3 = *(uint16_t *)0x1FFF75AA;  // Factory calibrated at 3V3, approx. 1500
  modvcc = 3.30 / (float)analog_read_single(UDRV_ADC_CHANNEL_VREFINT) * (float)rcali3v3;

  // Optionally linearize, e.g. .floatval = (fval * param.koeff[x]) - param.koeff[x+1]
  channel_value[0].fe.errno = 0;                                                       // Error code if needed
  channel_value[0].fe.floatval = analog_read_average(ANAPIN1, 100) * modvcc / 4096.0;  // Scale to V
  channel_value[0].unit = "V_PB4";                                                     // For display
  channel_value[1].fe.errno = 0; // 0: NoError

  channel_value[1].fe.floatval = (float)user_count;
  channel_value[1].unit = "(Cnt/Err)";
  channel_value[1].fe.errno = (user_count & 1) ? 0 : 7;  // if odd: Error 7 "NoData" on this channel, 
  user_count++;

  anz_values = 2;  // global variable

  return 0;  // OK - no transmission if <0
}
// HK is measured in the 2nd step if needed
float user_measure_hk_battery(void) {
  return modvcc;  // use cached value
}
float user_measure_hk_temperature(void) {
  return analog_read_internal_temp_3V3(10);  // 8 msec, Temp only if VCC = 3.3V
}

//---------- MAIN ------------
void setup() {
	ltx_lib_setup();
}
void loop() {
	ltx_lib_loop();
}
// **

