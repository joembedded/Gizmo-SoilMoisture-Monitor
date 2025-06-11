// ltx_rak3172_user.h - Define Device specific Params - SIMPLE NODE with Extendted AT

#ifndef LTX_CONFIG_H
#define LTX_CONFIG_H

//#define USAGE_EXT_AT     // USE as AT modem with extended commands
//#define USAGE_STANDALONE // Single-Chip Stand alone solution
//#define LED_PIN PA4      // Standard LED vs. VCC - This is PIN for State-LED (if defined)

// Define the used LoRa-Energy scheme (see ltx_globaldefs.h):
//#define STD_ENERGY_RAK3172LSIP_3V3 
//#define STD_ENERGY_RAK3172MODULE_3V3

#ifndef DEVICE_TYPE
 #define DEVICE_TYPE 8000         // Typically 8000: Basic, choose your own value for your application.
#endif
#ifndef DEV_FAMILY
 #define DEV_FAMILY "LTX-RAK3172-BASIC" // Name of the application
#endif
#ifndef DEVICE_FW_VERSION
 #define DEVICE_FW_VERSION 0      // Steps of 0.1 (e.g. 32 is Firmware V3.2)
#endif

// Good combi: Both 1 or both 0: (1:Server periodically automatically adjusts DR with ADR set, else 0:use always maximum energy)
// May be changed manually with AT-commands
#define LORA_AUTO_DATARATE_REDUCTION 0     // 1:Init with ADR enabled (fix device), 0:Init with ADR disabled(movin device)
#define LORA_CONFIRM_MODE 0                // 1:Confirm TX, 0:Send unconfirmed
#define USER_CREDENTIAL_SEED (get_mac_l()) // Dynamic/Random or Static, Macro to initialize (indiviodual) credentials generation

#if defined(USAGE_STANDALONE)
//#define HK_FLAGS 11        // HK values 1:BatVolt 2:Temp (4:Humidity) 8:Energy (16:Barometer)
//#define ANZ_KOEFF 8        // Number of coefficients for this application
//#define MAX_CHANNELS 4     // Maximum number of channels. For F16: 20 channels occupy 2+40 bytes payload plus optionally 1+3 bytes per HK channel
#ifndef START_DELAY_SEC
#define START_DELAY_SEC 30 // Start LoRaWAN after x seconds after PowerOn (Use >= 30 sec)
#endif
// 3V3: LTX calculates the consumed energy in mAH. Values for RAK3172L-SIP:
//#define MESSEN_ENERGY 1000 // uC - Assume 10mA for 100 msec (= 1 mC)
#endif

// LoRa Energy Scheme
#if !defined(STD_ENERGY_RAK3172MODULE_3V3) || !defined(STD_ENERGY_RAK3172LSIP_3V3)
 #define STD_ENERGY_RAK3172MODULE_3V3
#endif
// Energy also dependant from Antenna Matching! Bad match: lower energy.
#ifdef STD_ENERGY_RAK3172LSIP_3V3 // LowPower L-SIP
#define SOC_NAME "RAK3172L-SIP LowPower"
#define JOIN_ENERGY 100000        // uC For 3172-L at 3V3 @ DR0
#define TX_PER_BYTE_ENERGY 1490   // uC/Byte For 3172-L at 3V3 @ DR0
#define TX_FIX_ENERGY 53511       // uC For 3172-L at 3V3 @ DR0
#endif

#ifdef STD_ENERGY_RAK3172MODULE_3V3 // NormalPower Module, Eval:-T/-SIP
#define SOC_NAME "RAK3172 StdPower"
#define JOIN_ENERGY 160000          // uC For 3172-E/T at 3V3 @ DR0
#define TX_PER_BYTE_ENERGY 3404     // uC/Byte For 3172-E/T at 3V3 @ DR0
#define TX_FIX_ENERGY 116596        // uC For 3172-E/T at 3V3 @ DR0
#endif

#define PACKET_ENERGY (((TX_PER_BYTE_ENERGY * mlora_info.txframe.txanz) + TX_FIX_ENERGY) / (api.lorawan.dr.get() * 6 + 1))

#endif
//**