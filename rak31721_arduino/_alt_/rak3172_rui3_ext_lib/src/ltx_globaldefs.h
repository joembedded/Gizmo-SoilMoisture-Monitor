// LTX_GLOBALDEFS.H - Common Headers for LTX

/* Energy:
 *
 * RAK3172L-SIP - ARDUINO-SW
 * Join: 90 mC (DR0)
 * 1 Byte Upload DR0 55 mC
 * 48 Bytes Upload DR0 125 mC
 * => 1.490 mC/Byte + 53.511 mC(Fix)
 * RAK3172(Modul) - ARDUINO-SW
 * Join: 160 mC (DR0)
 * 1 Byte Upload DR0 120mC
 * 48 Bytes Upload DR0 280 mC
 * => 3.404 mC/Byte + 116.596mC(Fix)
 *
 * Info: 1 mAh = 3600 mC = 3600.000 uC
 * Payload 10 Bytes on L-SIP @ DR0: ca. 80 mC => ca. 45 Transmissions per 1 mAh => 1500 mAh Battery allows 67500 Transmissions
 * Payload 10 Bytes on Module @ DR0: ca. 150 mC => ca. 24 Transmissions per 1 mAh => 1500 mAh Battery allows 36000 Transmissions
 * => If possible (for fixed Devices) enable ADR (Auto Data Rate Reduction).
 */

extern void ltx_lib_setup(void);	// May call user_setup() and ..measure() if stand-alone
extern void ltx_lib_loop(void);		// Call to RUN

// Definitions
#if defined(USAGE_STANDALONE)
#define MAX_MESSCMD 79
#define _PMAGIC (0xCAFFEBA0 + DEVICE_TYPE * ANZ_KOEFF) // Magic for valid parameters
typedef struct
{
  uint32_t _pmagic;
  char messcmd[MAX_MESSCMD + 1]; // Defines the measurement command 'cmd'
  uint32_t period;               // (60..3599), 3600, xxx sec 'p'
  uint16_t hk_reload;            // If >=1: transmit with housekeeping 'hkr'
  // If >= 1 (-199) LoRaPort, upper 1000 included for ASL 1:F16, 0:F32
  // e.g. 11: rh/T(F32) or 1011(F16)
  uint16_t sensor_profile; // 'profile'
  bool use_watchdog;       // 'wd'

  // End: Coefficients [MUL, OFFSET, MUL, OFFSET, ..]
  float koeff[ANZ_KOEFF];
} PARAM;
extern PARAM param;                       // Supplied by User
extern const char *koeff_desc[ANZ_KOEFF]; // Description for the coefficients - Supplied by user

typedef union
{ // Conversion FLOAT->Binary
  uint32_t ulval;
  float fval;
} FXVAL;

#define NO_ERROR 0
// Representation of a general floating point number with optional error:
typedef struct
{
  uint16_t errno; // errno 0: No ERROR (1-1023 asl error code usable)
  float floatval; // Value only relevant if errno 0, otherwise ignored
} FE_ZAHL;

typedef struct
{
  FE_ZAHL fe; // The measured value
  // Optional space for meta-data, e.g. type-specific units
  const char *unit; // For "e": unit for display or NULL
} CHANNEL_VALUE;

extern CHANNEL_VALUE channel_value[MAX_CHANNELS];
extern uint16_t anz_values; // Number of measured values

// User Interface
extern void user_setup(void);
extern int16_t user_measure_values(int16_t ival);
extern float user_measure_hk_battery(void);
extern float user_measure_hk_temperature(void);

// Analog
extern void analog_setup(void);
extern int16_t analog_read_single(uint32_t apin);
extern float analog_read_average(uint32_t pin, uint16_t manz);
extern float analog_read_internal_temp_3V3(uint16_t manz);

// Measure
extern void hk_add_energy(uint32_t mAmSec);

// Get MAC h/l of Device
extern uint32_t get_mac_l(void);
extern uint32_t get_mac_h(void);
#endif

//**