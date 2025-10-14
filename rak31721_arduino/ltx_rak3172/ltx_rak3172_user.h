// ltx_rak3172_user.h - Define Device specific Params - SIMPLE NODE with Extendted AT

#ifndef LTX_CONFIG_H
#define LTX_CONFIG_H

//--- User defines first, then headers ---
//#define USAGE_STANDALONE // If defined: Single-Chip STANDALONE solution, else: HOSTED
#define EXT_BAUDRATE  9600  // Default is 115200, if defined, use this Baudrate (< 115200 recommended if used as external devive)
#define USAGE_EXT_AT     // optionally USE as AT modem with extended commands
#define LED_PIN PA4      // optionally Standard LED vs. VCC - This is PIN for State-LED (if defined)

//#define STD_ENERGY_RAK3172MODULE_3V3 // Standard and E/T/SIP
#define STD_ENERGY_RAK3172LSIP_3V3 // Low Power-SIP only

#define DEVICE_TYPE 8002         // Typically 8000: Basic, choose your own value for your application.
#define DEV_FAMILY "LTX-RAK3172-EXT_MODEM" // Name of the application
#define DEVICE_FW_VERSION 1      // Steps of 0.1 (e.g. 32 is Firmware V3.2)

//#define HK_FLAGS 11        // HK values 1:BatVolt 2:Temp (4:Humidity) 8:Energy (16:Barometer)
//#define ANZ_KOEFF 8        // Number of coefficients for this application
//#define MAX_CHANNELS 4     // Maximum number of channels. For F16: 20 channels occupy 2+40 bytes payload plus optionally 1+3 bytes per HK channel
// 3V3: LTX calculates the consumed energy in mAH. Values for RAK3172L-SIP:
//#define MESSEN_ENERGY 1000 // uC - Assume 10mA for 100 msec (= 1 mC)
//#define START_DELAY_SEC 30  // Start LoRaWAN after x seconds after PowerOn (Use >= 30 sec)


// Good combi: Both 1 or both 0: (1:Server periodically automatically adjusts DR with ADR set, else 0:use always maximum energy)
// May be changed manually with AT-commands
#define LORA_AUTO_DATARATE_REDUCTION 0      // 1:Init with ADR enabled (fix device), 0:Init with ADR disabled(movin device)
#define LORA_CONFIRM_MODE 0                 // 1:Confirm TX, 0:Send unconfirmed
#define USER_CREDENTIAL_SEED (get_mac_l())  // Dynamic/Random or Static, Macro to initialize (individual) credentials generation

// LoRa Energy Scheme

#endif
//**