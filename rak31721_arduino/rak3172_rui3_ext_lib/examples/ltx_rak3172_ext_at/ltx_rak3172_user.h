// ltx_rak3172_user.h - Define Device specific Params - Only extended AT

#define USAGE_EXT_AT     // USE as AT modem with extended commands
//#define USAGE_STANDALONE // Single-Chip Stand alone solution
#define LED_PIN PA4      // Standard LED vs. VCC - This is PIN for State-LED (if defined)

#define DEVICE_TYPE 8100         // Typically 8100: Basic 'Extended AT', choose your own value for your application.
#define DEV_FAMILY "LTX-RAK3172-ExtAT" // Name of the application
#define DEVICE_FW_VERSION 2      // Steps of 0.1 (e.g. 32 is Firmware V3.2)
// Good combi: Both 1 or both 0: (1:Server periodically automatically adjusts DR with ADR set, else 0:use always maximum energy)
// May be changed manually with AT-commands
#define LORA_AUTO_DATARATE_REDUCTION 0     // 1:Init with ADR enabled (fix device), 0:Init with ADR disabled(movin device)
#define LORA_CONFIRM_MODE 0                // 1:Confirm TX, 0:Send unconfirmed
#define USER_CREDENTIAL_SEED (get_mac_l()) // Dynamic/Random or Static, Macro to initialize (indiviodual) credentials generation

// Define used LoRa-Energy scheme (see ltx_globaldefs.h):
#define STD_ENERGY_RAK3172LSIP_3V3
//#define STD_ENERGY_RAK3172MODULE_3V3

//**