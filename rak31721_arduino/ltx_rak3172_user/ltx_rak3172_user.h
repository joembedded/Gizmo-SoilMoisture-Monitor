// ltx_rak3172_user.h - Define Device specific Params
#define DEVICE_TYPE 8000              // Typically 8000: Basic System, choose your own value for your application.
#define DEV_FAMILY "LTX-RAK3172"      // Name of the application
#define DEVICE_FW_VERSION 1           // Steps of 0.1 (e.g. 32 is Firmware V3.2)
#define HK_FLAGS 11                   // HK values 1:BatVolt 2:Temp (4:Humidity) 8:Energy (16:Barometer)
#define ANZ_KOEFF 8                   // Number of coefficients for this application
#define MAX_CHANNELS 4                // Maximum number of channels. For F16: 20 channels occupy 2+40 bytes payload plus optionally 1+3 bytes per HK channel
#define START_DELAY_SEC 30            // Start LoRaWAN after x seconds after PowerOn
#define AUTO_DATARATE_REDUCTION 0     // 1:Init with ADR enabled (fix device), 0:Init with ADR disabled(movin device)
#define USER_CREDENTIAL_SEED    (get_mac_l())  // Dynamic/Random or Static, Macro to initialize credentials generation

// 3V3: LTX calculates the consumed energy in mAH. This is for a RAK3172 module without stepper. RAK3172L requires less.
#define MESSEN_ENERGY 1000  // Assume 1mA for 1000 msec
#define PER_BYTE_ENERGY 90  // For 3172-E at 3V3, adjust for -L
#define JOIN_ENERGY 180000  // For 3172-E at 3V3, and with previous stepper

// This is PIN for State-LED
#define LED_PIN PA4  // Standard LED vs. VCC

//++