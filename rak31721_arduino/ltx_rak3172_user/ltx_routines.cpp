/*
 * @file ltx_routines.cpp
 * @brief LoRaWAN Sensor background routines
 * @version 1.02
 * @author JoEmbedded.de
 
 Dieses Modul implementiert einen kompletten LoRaWAN Node.
 Je nach definierte USAGE_ (siehe ltx_rak3172_user.h) kann das Modem
 Standalone oder als Modem (ggfs. auch beide) eingesetzt werden.

 ltx_rak3172_user.h:
 ===================
- Definiere USAGE

- Wenn LED vorhanden: Definiere LED

- Device-Parameter festlegen
  Energieschema, Parameter, etc..

 - Definiere Modem:
   #define STD_ENERGY_RAK3172LSIP_3V3
    oder
   #define STD_ENERGY_RAK3172MODULE_3V3


Zusätzliche Kommandos zur RUI3:
===============================
 Es werden diverse Kommandos unterstützt, alle über 'ATC+LTX' ansprechbar:

 Immer:
 ------
 initeu868   -  initialisiert Modem auf EU868 und sett Default DEVEUI/APPEUI/APPKEY-Werte
 cred        -  Zeigt die KEYs an ("Credentials")
 ?           -  Infos zum System/Modem
 stat        -  Statistik ueber letzten ausgehenden Frame
 reset       -  Resettet Modem (erhalten bleibt Energieverbrauch)

 Nur für USAGE_STANDALONE:
 -------------------------
 send        - 'port, hex..' sendet frame 'smart'(nur bei Bedarf confirmation wenn CFM=0)

 Nur für USAGE_EXT_ATX:
 ----------------------
 Dieser Modus erweitert die RAK RUI3 um einige Kommandos, können z.B. eingehende Frames
 gemerkt werden (Abwarten der Meldung "+EVT_RECEIVED" nicht noetig) oder die Energie der
 verbrauchten Aktion. Kommandos können auch vom Server im Klartext gesendet werden

 write        - Speichert Setup/Koeffizienten nach Änderungen im NVM
 factoryreset - Loescht alle User-Daten
 cmd=         - Setzt das Messkommando 'cmd'
 p=           - Setzt Messperiode in Sekunden
 hkr=         - Setzt HK-Counter, nach HK werden hkr Daten ohhne HK übertragen, spart Energie
 profile=     - 1-223 (Messwerte als F32 übertragen) oder 1001-1223 (Messwerte als F16 übertragen) 
 wd=          - 0/1 fuer false/true: Watchdog verwenden (empfohlen, kostet ca. 3uA mehr)
 k            - Koeffizienten setzen fuer Verrechnung der Messwerte
 e            - Testmessung ausführen und anzeigen
 i            - Testmessung mit Test-Übertragung. Ggfs. LoRaWAN Netzwerk joinen

 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <arduino.h>

#include "ltx_rak3172_user.h"
#include "ltx_globaldefs.h"

// Watchdog jo_wdt: Timeout 32sec fixed.
// Watchdog should always be fed if something takes longer than 2 sec
#include "uhal_wdt.h"  // Watchdog
#include "udrv_adc.h"  // Analog

// Magic Nv.RAM for energy (also after reset)
#define RAM_MAGIC (0xBADC0FFEUL)
uint32_t _tb_novo[4] __attribute__((section(".non_init")));
#define hk_batperc_h _tb_novo[1]
#define hk_batperc_h_neg _tb_novo[2]
#define hk_batperc_sum32 _tb_novo[3]

// Internal Defines - Infos
#define MAX_PAYBUF_ANZ 51  // Maximum number of bytes to receive/send
typedef struct
{
  struct
  {                    // Connection stuff
    bool device_init;  // true if device initialized (DEVEUI, APPEUI, APPKEY, BAND)
    uint32_t join_runtime;
    uint32_t last_server_reply_runtime;  // When was the last time something was heard from the server? Reply
    uint32_t last_server_time_runtime;   // If set; Runtime of lats TIME update
  } con;
  struct
  {                                        // Set parameters
    bool tx_with_conf;                     // (intern) If true override CFM
    uint8_t txanz;                         // Message to send if >0
    uint8_t txport;                        // Port to send, 1-223 allowed
    uint8_t txbytes[MAX_PAYBUF_ANZ + 32];  // Always binary, but technical reserve
  } txframe;
  struct
  {
    bool in_transfer;       // TRUE during RADIO transfer
    bool sth_received;      // TRUE if someting received, FALSE before send
    uint32_t frame_energy;  // Energy eines externen Frames
#if defined(USAGE_STANDALONE)
    int16_t flags;  // 15 Bits Flags or if <0: Error / "Reason"
                    // &128:RESET
                    // (&64:Alarm) unused here
                    // (&32:oldAlarm) unused here
                    // &15: Reason: 2:Auto, 3 Manual, Rest n.d.

    uint8_t hk_dcnt;  // If 0: always measure HK
    bool hk_valid;    // True if valid for this measurement
#endif
  } stat;
} MLORA_INFO;  // Modem Lora Info

// Globals
MLORA_INFO mlora_info;

/* _tb_novo[] implements an NV-RAM that retains data in RAM even after reset
 * used here for the energy counter. _tb_novo[1] and [2] must
 * contain complementary values and _tb_novo[0] a magic number to be valid */
void ltxtb_init(void) {
  // Check RAM and init counter
  if (_tb_novo[0] != RAM_MAGIC || (_tb_novo[1] != ~_tb_novo[2])) {
    _tb_novo[0] = RAM_MAGIC;  // Init Non-Volatile Vars
    _tb_novo[1] = 0;          // Reserved for User Energycounter H
    _tb_novo[2] = ~0;         // ~Reserved for User Energycounter H Neg. Value
    _tb_novo[3] = 0;          // Reserved for User Energycounter L
  }
}

// DirtyFlag Parameter
bool param_dirty = false;

#if defined(USAGE_STANDALONE)
// Periodic
uint32_t next_periodic_runtime;  // Measure when runtime >= next_periodic_runtime
#endif

// --- SEC_TIMER. allgemein ---
uint32_t dbg_until_runtime;  // As long as > now_runtime: Output
uint32_t now_runtime = 0;    // Runtime in sec since reset

uint32_t _omil = 0;  // Overflow every 49d!
// Bring runtime up to date
void update_runtime(void) {
  uint32_t _nmil = millis();
  int32_t nsec = ((int32_t)(_nmil - _omil)) / 1000;
  now_runtime += nsec;
  _omil = _nmil;
}

#if defined(USAGE_STANDALONE)
bool wdt_in_use;  // True if used/enabled

#define WDT_TIMER_PERIOD_SEC 30  // for auto_feed Max. up to TIMER_PERIOD 30

#define ENERGY_PERIODIC_WDT 190    // Avg. 10 mA for 10 msec + 3uA * 30 sec
#define ENERGY_PERIODIC_NOWDT 420  // Avg. 8 mA for 8 msec +3uA * 120 sec

#define IWDG_WINDOW 0xFFF  // Feeding too fast triggers WDT-Window!
#define IWDG_RELOAD 0xFFF
static IWDG_HandleTypeDef hiwdg;
void jo_wdt_feed(void) {
  HAL_IWDG_Refresh(&hiwdg);
}
int16_t jo_wdt_init(void) {
  wdt_in_use = true;
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = IWDG_WINDOW;
  hiwdg.Init.Reload = IWDG_RELOAD;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    return -1000;
  return 0;
}
#endif  // USAGE_STANDALONE

#if defined(USAGE_STANDALONE)
/* --- Analog routines, see AN2668 -----
 * extern analog PINs:
 *   PB3   PB4  PB2  A10  A15
 * internal analog channels:
 *   UDRV_ADC_CHANNEL_TEMPSENSOR
 *   UDRV_ADC_CHANNEL_VREFINT
 *   UDRV_ADC_CHANNEL_VBAT (see Datasheet 3.18.3)*/

void analog_setup(void) {
  udrv_adc_set_resolution(UDRV_ADC_RESOLUTION_12BIT);  // STM32 has ADC12-Bit
}
// Single measurement, approx. 800uSec - Result: 0..4095 corresponds to 0..3600 mV (?)
int16_t analog_read_single(uint32_t apin) {
  int16_t val16;
  udrv_adc_read(apin, &val16);
  return val16;
}
/* Multiple measurements. Possibly feed WD!
 * Result: 0.0 .. 4095.0 corresponds to 0..3600 mV (?) */
float analog_read_average(uint32_t pin, uint16_t manz) {  // manz: >0!
  int32_t temp32 = 0;
  for (uint16_t i = 0; i < manz; i++) {
    int16_t temp16;
    udrv_adc_read(pin, &temp16);  // approx. 800usec/Sample
    temp32 += temp16;
  }
  return (float)temp32 / (float)manz;
}
/* Read internal Temperature for VCC = 3.3V , Recommended: manz 4-10
 * CPU comes with calibration Coeffs () (+/- 5°C!)
 * Only valid if Module Voltage VCC is 3.3 Volt! */
float analog_read_internal_temp_3V3(uint16_t manz) {
  float tempadf = analog_read_average(UDRV_ADC_CHANNEL_TEMPSENSOR, manz);
  int16_t tcal30 = *(uint16_t *)0x1FFF75A8;   // approx. 950 from datasheet 3.18.1
  int16_t tcal130 = *(uint16_t *)0x1FFF75C8;  // approx. 1200
  if (tcal130 > tcal30)
    return (100.0 / (float)(tcal130 - tcal30)) * (tempadf - (float)tcal30) + 30.0;
  else
    return -99.0;
}
// Analog-End
#endif

#if defined(LED_PIN)
// Blinker
#define BLINK_MAX 10
static uint16_t ablink_cnt;
static uint16_t blink_level;
static uint16_t blink_running;
void apptimer_blink_handler(void *data) {
  if (!ablink_cnt) {
    if (!blink_running) {
      api.system.timer.stop(RAK_TIMER_1);
      return;
    }
    blink_running--;
    ablink_cnt = BLINK_MAX;
    digitalWrite(LED_PIN, LOW);  // LED on
  } else {
    if (ablink_cnt == blink_level)
      digitalWrite(LED_PIN, HIGH);  // LED off
    ablink_cnt--;
  }
}
// Set,start blink timer at 8Hz, val= 0..8, if !running: run_cnt
// 1: Long ON - 5: 50% - 10: Short ON, each run: 1.25 sec
void blink_timer_setvalrun(uint16_t val, uint16_t runs) {
  blink_level = val;
  if (runs && !blink_running) {
    ablink_cnt = 0;
    blink_running = runs;
    api.system.timer.start(RAK_TIMER_1, 125 /*msec*/, (void *)0);  // Not system relevant
  } else {                                                         // Re-Trigger or stop (if 0)
    blink_running = runs;
  }
}
int16_t blink_init(void) {
  if (api.system.timer.create(RAK_TIMER_1, (RAK_TIMER_HANDLER)apptimer_blink_handler, RAK_TIMER_PERIODIC) != true)
    return -1003;  // Timer1 Fail1
  return 0;        // OK
}
#endif  // LED_PIN

#if defined(USAGE_STANDALONE)
/* Context of the handler is "normal"
 * Note: the timer has a frame of 9msec @ approx. 10mA msec and itself, average thus approx. 3->7uA
 * if called only every 10 sec, the current consumption increases to avg. 15uA!
 * Timer is used for opt. WD and periodic, since apparently only 1 timer runs cleanly, or the WD-Window may be triggered
 * If no WD, interval max. 120 sec, otherwise 30 sec due to WD
 * Important: Feed watchdog every <= 32 seconds!*/
static int16_t measure(int16_t ival);  // Forward Decl.
static int16_t lora_transfer(void);    // Forward Decl.
void apptimer_timer_handler(void *data) {
#if defined(LED_PIN)
  digitalWrite(LED_PIN, LOW);  // LED ON
#endif
  if (wdt_in_use)
    jo_wdt_feed();  // Verified: WD triggers cleanly 32 sec after last feed
  update_runtime();
  int32_t delta2next = next_periodic_runtime - now_runtime;
  if (delta2next > 0 || (!mlora_info.con.device_init)) {
    if (dbg_until_runtime > now_runtime) {
      if (wdt_in_use)
        Serial.printf("[WDT]");  // Wake with Watchdog <= 30sec
      else
        Serial.printf("[Wake]");  // Wake without Watchdog <= 120sec for even less I_q
      if (!mlora_info.con.device_init)
        Serial.printf(" - *** Device not init! ***\n");
      else
        Serial.printf(" - Next Transfer Slot in %d sec\n", delta2next);
    }
#if (HK_FLAGS & 8)
    hk_add_energy(wdt_in_use ? ENERGY_PERIODIC_WDT : ENERGY_PERIODIC_NOWDT);  // Energy for 1 Tick
#endif
  } else {
    Serial.printf("[PERIODIC] - Auto Transfer...\n");
    mlora_info.stat.flags |= 0x12;  // Start measurement and transfer. At least AUTO
    int16_t res = measure(0);       // Can also return general error
    if (res >= 0) {
      res = lora_transfer();
    }

    if (res)
      Serial.printf("ERROR: %d\n", res);

    while (next_periodic_runtime <= now_runtime)
      next_periodic_runtime += param.period;
  }
#if defined(LED_PIN)
  digitalWrite(LED_PIN, HIGH);  // LED OFF
#endif
}
#endif  // USAGE_STANDALONE

//------ JoEmb Toolbox---------
// universal String
#define UNI_LINE_SIZE 199
char uni_line[UNI_LINE_SIZE + 1];

// universal Byte-Buffer
#define UNI_BUF_SIZE 256
uint8_t uni_buf[UNI_BUF_SIZE];

// Access 64 Bit unique ID
uint32_t get_mac_l(void) {
  return *(uint32_t *)0x1FFF7580;
}
uint32_t get_mac_h(void) {
  return *(uint32_t *)0x1FFF7584;
}

// Own pseudo-random generator 16-Bit - e.g. for LoRa-Keys
// Advisable to initialize!
static uint32_t _seed = 0;
void my_seed(uint32_t s) {
  _seed = s;
}
uint16_t my_rand16(void) {
  _seed = (_seed * 1664525 + 1013904223);  // LCG formula (Wikipedia)
  return (uint16_t)(_seed >> 8);           // Result from the middle
}
void set_my_rbytes(uint8_t *pu, int16_t anz) {
  while (anz--) {
    *pu = (my_rand16() & 15) << 4;
    *pu++ |= (my_rand16() & 15);
  }
}
// String-Matcher Match "what" in *pstr
bool str_cmatch(const char *what, const char *pstr) {
  uint8_t c;
  for (;;) {
    c = *what++;
    if (!c)
      return true;  // All equal until finish!
    if (c != (*pstr++))
      return false;  // Not equal
  }
}

#if defined(USAGE_STANDALONE)
// Parameter Save/Load Flash
int16_t parameter_nvm_load(void) {
  uint32_t _ptest = 0;
  api.system.flash.get(0, (uint8_t *)&_ptest, sizeof(uint32_t));
  if (_ptest != _PMAGIC)
    return -2002;  // No Params
  param._pmagic = 0;
  api.system.flash.get(0, (uint8_t *)&param, sizeof(param));
  if (param._pmagic != _PMAGIC)
    return -2003;  // Read Error!!!
  return 0;        // OK
}
int16_t parameter_nvm_save(void) {  // set param._pmagic to 0 for Flash-Clear
  if (!api.system.flash.set(0, (uint8_t *)&param, sizeof(param)))
    return -2004;  // Write error?
  param_dirty = false;
  return 0;
}

// Measurement START
// F16-Helper START
// Simulates Float16 on ARDUINO - Only as one-way conversion for UPLINK!
uint16_t float_to_half(float f) {
  union {
    float f;
    uint32_t u;
  } u = { f };
  uint32_t f_bits = u.u;
  uint32_t sign = (f_bits >> 16) & 0x8000;  // sign shift to 16-bit position
  uint32_t exponent = (f_bits >> 23) & 0xFF;
  uint32_t mantissa = f_bits & 0x7FFFFF;
  if (exponent == 255) {  // Inf or NaN
    if (mantissa)
      return sign | 0x7E00;  // NaN
    else
      return sign | 0x7C00;  // Inf
  }
  int16_t new_exp = exponent - 127 + 15;
  if (new_exp >= 31) {
    return sign | 0x7C00;  // Overflow → Inf
  } else if (new_exp <= 0) {
    if (new_exp < -10) {
      return sign;  // Too small → Zero
    }
    // Subnormal number
    mantissa = (mantissa | 0x800000) >> (1 - new_exp);
    return sign | (mantissa >> 13);
  }
  return sign | (new_exp << 10) | (mantissa >> 13);
}
// F16-Helper End

#define HK_MAXANZ 4
#define HK90_VBAT 0                // Bat in mV Index
#define HK91_TEMP 1                // Temp in 0.1 degree
#define HK92_HUM 1                 // Humidity in 0.1 percent if available
#define HK93_MAH 2                 // Energy
extern float hk_value[HK_MAXANZ];  // old: int16!

CHANNEL_VALUE channel_value[MAX_CHANNELS];
uint16_t anz_values;        // Number of measured values
float hk_value[HK_MAXANZ];  // New: HL as float

// Minimal HK
static void measure_hk(void) {
#if HK_FLAGS & 1
  hk_value[HK90_VBAT] = user_measure_hk_battery();
#endif
#if HK_FLAGS & 2
  hk_value[HK91_TEMP] = user_measure_hk_temperature();
#endif
#if HK_FLAGS & 8
  uint32_t h;
  // Direct conversion to mAH with adjustment
  h = (hk_batperc_sum32 & 0xFFC00000);  // 1.165mAH-overflow? (0-4194304).16
  if (h) {
    hk_batperc_sum32 -= h;
    hk_batperc_h += (h / 4194304);     // 22 Bits
    hk_batperc_h_neg = ~hk_batperc_h;  // Change all approx. 1 mAh NEG
  }
  hk_value[HK93_MAH] = (hk_batperc_h * 1.1650844) + (hk_batperc_sum32 * 277.778e-9);
#endif
#if HK_FLAGS
  mlora_info.stat.hk_valid = true;
#endif
}
#if HK_FLAGS & 8
// Idea: add energy in mAmsec and regularly shift mA-bit, 1mAms correspond to 1uC
// hk_add_energy(1*35000) for 1 sec 35000uA)
void hk_add_energy(uint32_t mAmSec) {
  hk_batperc_sum32 += mAmSec;
}
#endif

/* Type-specific measurement.
 * Several possibilities, with ival>0 always measure HK
 * "Regular" measurement ival=0
 * Feeding the Watchdog required if takes >1 sec */

static int16_t measure(int16_t ival) {
  // First User Measure
  int16_t ures = user_measure_values(ival);
  // Optionally HK
  if (!mlora_info.stat.hk_dcnt || ival) {
    mlora_info.stat.hk_dcnt = param.hk_reload;
    measure_hk();
  } else {
    mlora_info.stat.hk_dcnt--;
    mlora_info.stat.hk_valid = 0;
  }
#if (HK_FLAGS & 8)
  hk_add_energy(MESSEN_ENERGY);  // Energy for 1 measurement
#endif
  return ures;
}
// Measurement End
#endif  // USAGE_STANDALONE

// Lora Default Credentials generation band: 4:EU868
// Credentials Example: AT+DEVEUI=0080E1150032D493 => AT+APPEUI=02EF62D6B9DD3A29 => AT+APPKEY=290DDBCD739E45B2857F87F86092838F
int16_t lora_setup(uint8_t band) {
  int16_t res = 0;  // Assume OK
  // Set keys and init LoRaWAN credentials
  my_seed(USER_CREDENTIAL_SEED);  // Always the same sequence, depends on MAC
  uint32_t h = get_mac_h();       // LTX: Big Endian Style
  uni_buf[0] = (uint8_t)(h >> 24);
  uni_buf[1] = (uint8_t)(h >> 16);
  uni_buf[2] = (uint8_t)(h >> 8);
  uni_buf[3] = (uint8_t)(h);
  h = get_mac_l();
  uni_buf[4] = (uint8_t)(h >> 24);
  uni_buf[5] = (uint8_t)(h >> 16);
  uni_buf[6] = (uint8_t)(h >> 8);
  uni_buf[7] = (uint8_t)(h);
  api.lorawan.deui.set(uni_buf, 8);
  set_my_rbytes(uni_buf, 8);
  api.lorawan.appeui.set(uni_buf, 8);
  set_my_rbytes(uni_buf, 16);
  api.lorawan.appkey.set(uni_buf, 16);

  if (!api.lorawan.band.set(band))
    res = -2101;            // 4:868 MHz, ..
  api.lorawan.njm.set(1);   // 1: OTAA
  api.lorawan.rety.set(0);  // No RX confirmation repeat
  api.lorawan.dr.set(0);    // In case still > 0 from before

  api.lorawan.cfm.set(LORA_CONFIRM_MODE);             // But confirm TX
  api.lorawan.adr.set(LORA_AUTO_DATARATE_REDUCTION);  // !!!1:Automatic Data Reduction On, not useful for moving devices!!!
  return res;
}

#if defined(USAGE_STANDALONE)

// lora_transfer() and related
// Payload_flags
#define PLWITH_ALL 3  // Combo
#define PLWITH_VALUES 1
#define PLWITH_HK 2

uint8_t *measure_payoff_lorahdr(void) {
  uint8_t *pu = mlora_info.txframe.txbytes;  //  MAX_PYBUF_AND+32 space
  *pu++ = mlora_info.stat.flags;
  return pu;
}
/******************************************************************************************
 * Generate compressed data -BYTES
 * uint8_t* measure_payoff(uint8_t *pu)
 * Optimized version for LTX-Payload. Assembling the payload is a bit more complicated,
 * but very space-saving. F16-Check: https://evanw.github.io/float-toy/
 *
 * Resolution F16/32 via +1000 in fPort == param.sensor_profile (but then for all)
 **************************************************************************************/
uint8_t *measure_payoff_mess(uint8_t *pu, uint8_t payload_flags) {
  /* Channels 0..89: "cursor"= channel implicit at 0
   * Then 0Fxxxxxx 6 Bits count  F:0:Float32, F:1:Float16 xx: number following, at 0: cursor follows
   *      1bbbbbbb 7 Bits HK, starts at least at 90, if more than 7 HK: 2*
   * Here only max. 25 channels possible due to LoRa payload length <= 51 bytes
   */
  if (payload_flags & PLWITH_VALUES) {
    uint16_t mcursor = 0;
    // ---F16---
    if ((param.sensor_profile / 1000) == 1) {  // 1k-Flag in port: All as F16, else F32
      uint16_t anzf16 = (uint8_t)anz_values;   //
      *pu++ = (anzf16 | 64);                   // number of F16 follows
      while (anzf16--) {
        uint16_t fx16vu;

        if (channel_value[mcursor].fe.errno == NO_ERROR) {
          fx16vu = float_to_half(channel_value[mcursor].fe.floatval);
        } else {
          fx16vu = 0xFC00 | (channel_value[mcursor].fe.errno & 1023);
        }
        *pu++ = (uint8_t)(fx16vu >> 8);
        *pu++ = (uint8_t)(fx16vu);
        mcursor++;
      }
      // ---F32---
    } else {
      uint16_t anzf32 = anz_values;
      *pu++ = (uint8_t)anzf32;  // number of F32 follows
      while (anzf32--) {
        FXVAL fxv;
        if (channel_value[mcursor].fe.errno == NO_ERROR) {
          fxv.fval = channel_value[mcursor].fe.floatval;
        } else {
          // all 16k errors representable
          fxv.ulval = 0xFF800000 | channel_value[mcursor].fe.errno;
        }
        *pu++ = (uint8_t)(fxv.ulval >> 24);
        *pu++ = (uint8_t)(fxv.ulval >> 16);
        *pu++ = (uint8_t)(fxv.ulval >> 8);
        *pu++ = (uint8_t)(fxv.ulval);

        mcursor++;
      }
    }
  }

  if (payload_flags & PLWITH_HK) {
    *pu++ = (uint8_t)(0x80 | (HK_FLAGS & 127));  // HK is FIX for now
    float fval;
    uint16_t fx16vu;
    // For simple gateway HKs fixed
#if HK_FLAGS & 1
    fx16vu = float_to_half(hk_value[HK90_VBAT]);
    *pu++ = (uint8_t)(fx16vu >> 8);
    *pu++ = (uint8_t)(fx16vu);

#endif
#if HK_FLAGS & 2
    fx16vu = float_to_half(hk_value[HK91_TEMP]);
    *pu++ = (uint8_t)(fx16vu >> 8);
    *pu++ = (uint8_t)(fx16vu);
#endif
#if HK_FLAGS & 4
    x  // Humidity still missing *todo*
#endif
#if HK_FLAGS & 8
      fx16vu = float_to_half(hk_value[HK93_MAH]);
    *pu++ = (uint8_t)(fx16vu >> 8);
    *pu++ = (uint8_t)(fx16vu);
#endif
  }
  return pu;
}
#endif  // USAGE_STANDALONE

/******* Lora-Transfer and Service ****************/
#if defined(USAGE_STANDALONE)
// Before the callbacks
static int16_t parse_ltx_cmd(char *pc);

/* Parse incoming payload and treat as CMD, 0-terminated */
static void menu_parse_payload(char *pc) {
  while (*pc) {
    char *pc0 = pc++;
    while (*pc > ' ')
      pc++;
    char c = *pc;  // c can be 0 (end) or <= ' ' (more to come)
    *pc++ = 0;     // Read until whitespace and terminate string, possibly skip whitespace

    parse_ltx_cmd(pc0);

    if (!c)
      break;
  }
  // Possibly save
  if (param_dirty == true) {
    parse_ltx_cmd((char *)"write");  // (char*) is C++ peculiarity
  }
}
#endif  // USAGE_STANDALONE

// Payload already formated and stored, reset received Flag
static int16_t send_txpayload(void) {
  if (!mlora_info.txframe.txanz) {
    Serial.println("ERROR: Nothing to send");
    return -2011;
  }
  mlora_info.stat.in_transfer = true;
  mlora_info.stat.sth_received = false;
  bool sres;
  if (mlora_info.txframe.tx_with_conf) {  // request confirmation
    sres = api.lorawan.send(mlora_info.txframe.txanz, mlora_info.txframe.txbytes, mlora_info.txframe.txport, true);
  } else {
    sres = api.lorawan.send(mlora_info.txframe.txanz, mlora_info.txframe.txbytes, mlora_info.txframe.txport);
  }
  if (!sres) {
    Serial.println("ERROR: Send failed");  // Bool - e.g. because of Duty Cycle limitations
    mlora_info.stat.in_transfer = false;
    return -2010;
  }
  return 0;
}

uint16_t rejoins = 0;  // Retry-Counter
void join_cb(int32_t status) {
  if (status == RAK_LORAMAC_STATUS_OK) {
    rejoins = 0;
    Serial.println("Network joined");
#if defined(LED_PIN)
    blink_timer_setvalrun(5, 8);  // 8 times 50% blink: JOINED
#endif
    send_txpayload();  // Should be there
    mlora_info.con.join_runtime = now_runtime;
    uint32_t used_energy = PACKET_ENERGY;
    mlora_info.stat.frame_energy += used_energy;  // Add
#if HK_FLAGS & 8
    hk_add_energy(used_energy);
#endif
  } else {
    Serial.printf("ERROR: %d, Join failed\n", status);
    if (rejoins) {
      Serial.printf("Retry(%u) Join Network...\n", rejoins);
      rejoins--;
      api.lorawan.join();
      uint32_t used_energy = JOIN_ENERGY;
      mlora_info.stat.frame_energy += used_energy;  // Add
#if HK_FLAGS & 8
      hk_add_energy(JOIN_ENERGY);
#endif
    } else {
#if defined(LED_PIN)
      blink_timer_setvalrun(10, 8);  // 8 times short blink: ERROR
#endif
      mlora_info.stat.in_transfer = false;
    }
  }
}

// Lora Callbacks -> SERVICE_LORA_RECEIVE_T https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/lorawan#service_lora_receive_t
void recv_cb(SERVICE_LORA_RECEIVE_T *data) {
#if defined(USAGE_STANDALONE)
  mlora_info.stat.flags = 0;  // Arrived at server
#endif
  mlora_info.con.last_server_reply_runtime = now_runtime;
  uint8_t rlen = data->BufferSize;
  if (dbg_until_runtime > now_runtime) {
    Serial.printf("Received %u Bytes on Port %u\n", rlen, data->Port);
    Serial.printf("RxDR:%u RSSI:%d SNR:%d DL-Counter:%u\n", data->RxDatarate, data->Rssi, data->Snr, data->DownLinkCounter);

    if (rlen) {
      mlora_info.stat.sth_received = true;

      Serial.printf("Data: ");
      for (int i = 0; i < rlen; i++) {
        Serial.printf("%02x", data->Buffer[i]);
      }
      Serial.print(" '");
      for (int i = 0; i < rlen; i++) {
        uint8_t c = data->Buffer[i];
        if (c >= ' ' && c < 128)
          Serial.printf("%c");
        else
          Serial.printf("<%u>", c);
      }
      Serial.print("'\n");
    }
  }
  // Evaluate payload:
  // LTX only Port 10! And is a string with space as separator ('p=600 cmd=Hello')
  if (rlen && data->Port == 10) {
    data->Buffer[rlen] = 0;  // Terminate ASCII
#if defined(USAGE_STANDALONE)
    menu_parse_payload((char *)data->Buffer);
#endif
  }
}

void timereq_cb(int32_t status) {
  if (status == GET_DEVICE_TIME_OK) {
    mlora_info.con.last_server_time_runtime = now_runtime;
    char local_time[30] = { 0 };
    service_lora_get_local_time(local_time);
    Serial.printf("Received Time:'%s'\n", local_time);  // '23h59m54s on 05/24/2025' Note:24.5.2025 (=US-Format)
  } else if (status == GET_DEVICE_TIME_FAIL) {
    Serial.println("Receive Time failed");
  }
}

/* Send-Callback: Usually just says 0, approx. 3 sec. after send ??? Good maybe for an LED or detect TCXO ok */
void send_cb(int32_t status) {
  if (status != RAK_LORAMAC_STATUS_OK)
    Serial.printf("ERROR: Send status: %d\n", status);
  mlora_info.stat.in_transfer = false;
}
// ---CB End

/* Globally available
 *  Payload TX-Data Packet pre-formated:
 *  mlora_info.txframe.txanz
 *  mlora_info.txframe.txport
 *  mlora_info.txframe.txanz
 *
 * handles CFM=0/1
 */
int16_t lora_send_packet(void) {
  if (dbg_until_runtime > now_runtime) {  // Show Payload
    Serial.printf("P[%u]:", mlora_info.txframe.txport);
    for (uint16_t i = 0; i < mlora_info.txframe.txanz; i++)
      Serial.printf("%02X", mlora_info.txframe.txbytes[i]);
    Serial.printf("\n");
  }

  // Auto-Request confirmater after 6h, if nothing heard after 12h -(param.period*6) or 1d: Re-Join
  int32_t last_contact_sec = now_runtime - mlora_info.con.last_server_reply_runtime;
  mlora_info.txframe.tx_with_conf = (last_contact_sec >= 21600) ? true : false;  // >6h always request confirmation
  int32_t last_contact_limit = 43200;                                            // Each 12h reply expected
#if defined(USAGE_STANDALONE)
  if (param.period > 7200)
    last_contact_limit = param.period * 6;  // or more
#else
  last_contact_limit = 86400;  // If not Standalone 1d
#endif
  if ((last_contact_sec > last_contact_limit) || !api.lorawan.njs.get()) {
    Serial.printf("Join Network...\n");  // Join or Rejoin
    rejoins = 3;                         // 1+3 attempts
    mlora_info.stat.in_transfer = true;
    api.lorawan.join();
    uint32_t used_energy = JOIN_ENERGY;
    mlora_info.stat.frame_energy = used_energy;  // Set
#if HK_FLAGS & 8
    hk_add_energy(used_energy);
#endif
  } else {
    send_txpayload();
    uint32_t used_energy = PACKET_ENERGY;
    mlora_info.stat.frame_energy = used_energy;  // Set
#if HK_FLAGS & 8
    hk_add_energy(used_energy);
#endif
  }
  return 0;
}

#if defined(USAGE_STANDALONE)
// Start Transfer via ATC+LTX+i handler or times Automatic
static int16_t lora_transfer(void) {
  uint8_t *phu = measure_payoff_lorahdr();
  phu = measure_payoff_mess(phu, mlora_info.stat.hk_valid ? PLWITH_ALL : PLWITH_VALUES);
  int16_t paylen = (phu - mlora_info.txframe.txbytes);

  Serial.printf("LoRa-Transfer%s (%d Bytes)\n", mlora_info.stat.hk_valid ? " with HK" : "", paylen);
  if (paylen > MAX_PAYBUF_ANZ) {
    Serial.printf("ERROR: Payload too large, clipped to %u Bytes\n", MAX_PAYBUF_ANZ);
    paylen = MAX_PAYBUF_ANZ;  // Better something than nothing
  }
  mlora_info.txframe.txanz = (uint8_t)paylen;

  mlora_info.txframe.txport = (param.sensor_profile % 1000) & 255;
  if (mlora_info.txframe.txport < 1 || mlora_info.txframe.txport > 223) {
    Serial.printf("ERROR: Invalid fPort %d: set to 1\n", mlora_info.txframe.txport);
    mlora_info.txframe.txport = 1;
  }

  int16_t res = lora_send_packet();
  return res;
}
#endif  // USAGE_STANDALONE

// Show Credentials. Returns 0 if not initialized (all 0)
int16_t show_credentials(void) {
  uint8_t buff[16];
  int32_t bsum = 0;
  Serial.printf("DEVEUI: ");
  api.lorawan.deui.get(buff, 8);
  for (uint16_t i = 0; i < 8; i++) {
    Serial.printf("%02X", buff[i]);
    bsum += buff[i];
  }
  Serial.printf("\nAPPEUI: ");
  api.lorawan.appeui.get(buff, 8);
  for (uint16_t i = 0; i < 8; i++) {
    Serial.printf("%02X", buff[i]);
    bsum += buff[i];
  }
  Serial.printf("\nAPPKEY: ");
  api.lorawan.appkey.get(buff, 16);
  for (uint16_t i = 0; i < 16; i++) {
    Serial.printf("%02X", buff[i]);
    bsum += buff[i];
  }
  Serial.printf("\nBand: ");
  uint16_t band = api.lorawan.band.get();
  Serial.printf("%d ", band);
  switch (band) {
    case 4:
      Serial.printf("(*EU868)\n");  // Recom
      break;
    default:
      Serial.printf("(unknown)\n");
      bsum = 0;
  }
  Serial.printf("Activation: (0:ABP/*1:OTAA): %d\n", api.lorawan.njm.get());  // 1 Recom
  Serial.printf("Retrans.Conf.RX(*0): %d\n", api.lorawan.rety.get());         // 0 Recom
  Serial.printf("Confirm TX(*1): %d\n", api.lorawan.cfm.get());               // 1 Recom
  Serial.printf("Adaptive Datarate(*1): %d\n", api.lorawan.adr.get());        // 1 Recom (but !!!Automatic Data Reduction On, not useful for moving devices!!!)
  mlora_info.con.device_init = (bsum > 0);
  return bsum;
}

#if defined(USAGE_STANDALONE)
// Deliver a format string
const char *getLTXErrorfmt(uint16_t errno) {
  switch (errno) {
    case 1:
      return "NoValue";
    case 2:
      return "NoReply";
    case 3:
      return "OldValue";
    // 4,5 unused
    case 6:
      return "ErrorCRC";
    case 7:
      return "DataError";
    case 8:
      return "NoCachedValue";
    default:
      return "Err%u";
  }
}
#endif  // USAGE_STANDALONE
#if defined(USAGE_EXT_AT)
/*****************************************************************************
 * fpool_get_hex - get a value in HEX representatin from a pointer in BE Format
 * Reads max. max chars, so a Byte 1 is max=2 (and max<=8 of course)
 * Only from RAM
 * This can be used to store/read binary blocks in Files.
 ****************************************************************************/
uint32_t fpool_get_hex(char **ppc, uint8_t max) {
  char *pc, c, anz = 0;
  uint32_t val = 0;

  pc = *ppc;
  for (;;) {
    c = *pc;
    if (c < 32)
      break;  // End of String detected? STOP
    pc++;
    if ((c < '0' || c > '9') && (c < 'a' || c > 'f') && (c < 'A' || c > 'F')) {
      if (!anz)
        continue;  // ignore leading Whitespace or Non-HEX
      else
        break;  // but exit after the first
    }
    if (c >= 'a')
      c -= ('a' - 10);  // kleinbuchstaben hoechste!
    else if (c >= 'A')
      c -= ('A' - 10);
    else
      c -= '0';

    val <<= 4;
    ;
    val += c;
    anz++;
    if (anz == max)
      break;  // limit length
  }
  *ppc = pc;  // Save End-Pointer
  return val;
}
#endif

// Command handler only takes strings, Ret: 0: OK. No WS!
static int16_t parse_ltx_cmd(char *pc) {
  int16_t res = 0;  // Assume OK

#if defined(LED_PIN)
  digitalWrite(LED_PIN, LOW);  // LED ON
#endif
  update_runtime();

  if (!strcasecmp(pc, "initeu868")) {
    res = lora_setup(4);  // Band 4:868 MHz
    show_credentials();
  } else if (str_cmatch("cred", pc)) {  // cred.. : Credentials and Setup
    show_credentials();
  } else if (str_cmatch("?", pc)) {  // ?: Pure info function or ?k coefficients
    Serial.printf("Info:\n");
    Serial.printf("DEVICE_TYPE: %u\n", DEVICE_TYPE);
    if (!mlora_info.con.device_init) {
      Serial.printf("Device not init!\n");
    } else {
      Serial.printf("Runtime: %d sec\n", now_runtime);
      if (api.lorawan.njs.get()) {
        uint8_t buff[4];
        api.lorawan.daddr.get(buff, 4);
        Serial.printf("Joined (DEVADDR: %02X%02X%02X%02X) %d sec ago\n", buff[0], buff[1], buff[2], buff[3], now_runtime - mlora_info.con.join_runtime);
        Serial.printf("LastServerReply: ");
        if (mlora_info.con.last_server_reply_runtime)
          Serial.printf("%d sec ago\n", now_runtime - mlora_info.con.last_server_reply_runtime);
        else
          Serial.printf("Never!\n");

        Serial.printf("LastServerTime: ");
        if (mlora_info.con.last_server_time_runtime)
          Serial.printf("%d sec ago\n", now_runtime - mlora_info.con.last_server_time_runtime);
        else
          Serial.printf("Never!\n");

        Serial.printf("Datarate: %d\n", api.lorawan.dr.get());
      } else {
        Serial.printf("No Network!\n");
      }
    }
#if defined(USAGE_STANDALONE)  // Additional Info only if Standalone
    if (param._pmagic != _PMAGIC) {
      Serial.printf("ERROR: Parameter invalid\n");
    } else {
      Serial.printf("Cmd('cmd='): '%s'\n", param.messcmd);
      Serial.printf("Period('=p'): %u sec\n", param.period);
      Serial.printf("HK-Reload('hkr='): %u\n", param.hk_reload);
      Serial.printf("Profile('profile='): %u\n", param.sensor_profile);
      Serial.printf("Use Watchdog('wd='): %u\n", param.use_watchdog);

      if (param_dirty)
        Serial.printf("INFO: Parameter not saved!\n");
    }
#endif
    // Statistics
  } else if (!strcasecmp(pc, "stat")) {
    Serial.printf("intr.F%u rec.R%u ener.G%u", mlora_info.stat.in_transfer, mlora_info.stat.sth_received, mlora_info.stat.frame_energy);  // Fuer Transfer
    if (mlora_info.con.last_server_time_runtime) {
      int32_t tage = now_runtime - mlora_info.con.last_server_time_runtime;
      Serial.printf(" t.T%u", tage);  // Age of LoRa-Time
    }
    Serial.printf("\n");
  }
#if defined(USAGE_STANDALONE)
  else if (!strcasecmp(pc, "write")) {  // Write Parameter
    Serial.printf("Write Parameter\n");
    res = parameter_nvm_save();
  } else if (!strcasecmp(pc, "factoryreset")) {  // FactoryReset
    param._pmagic = 0;
    parameter_nvm_save();
    Serial.printf("Factory Reset...\n");
    delay(50);
    api.system.restoreDefault();
    api.system.reboot();
  } else if (str_cmatch("cmd=", pc)) {
    pc += 4;
    strncpy(param.messcmd, pc, MAX_MESSCMD);  // DSN
    param_dirty = true;
    Serial.printf("Cmd: '%s'\n", param.messcmd);
  } else if (str_cmatch("p=", pc)) {
    uint32_t np = atoi(pc + 2);
    if (np < 10) {  // Ignore nonsensical periods
      res = -2007;  // Min. 10 sec period
    } else if (np > 86400) {
      res = -2008;  // Max. 1/day
    } else {
      param.period = np;
      param_dirty = true;
      Serial.printf("Period: %u sec\n", param.period);

      uint32_t int_sec = param.period;
      if (int_sec > 120)
        int_sec = 120;  // Max every 2 minutes
      if (wdt_in_use && (int_sec > WDT_TIMER_PERIOD_SEC)) {
        int_sec = WDT_TIMER_PERIOD_SEC;  // Or less if WD active
        jo_wdt_feed();
      }
      next_periodic_runtime = now_runtime + 30;  // Traditionally after 30 sec it starts
      if (api.system.timer.stop(RAK_TIMER_0) != true)
        res = -2005;  // Timer0 Fail1b
      else if (api.system.timer.start(RAK_TIMER_0, (int_sec * 1000), (void *)0) != true)
        res = -2006;  // Timer0 Fail2b
    }
  } else if (str_cmatch("hkr=", pc)) {
    uint32_t nhk = atoi(pc + 4);
    if (nhk < 0)
      nhk = 0;  // Reload
    param.hk_reload = nhk;
    mlora_info.stat.hk_dcnt = 0;
    param_dirty = true;
    Serial.printf("HK-Reload: %u\n", param.hk_reload);
  } else if (str_cmatch("profile=", pc)) {
    uint32_t nprof = atoi(pc + 8);
    if (nprof < 1)
      nprof = 1;  // Prof 1-xxx (1000: F16)
    param.sensor_profile = nprof;
    param_dirty = true;
    Serial.printf("Profile: %u\n", param.sensor_profile);
  } else if (str_cmatch("wd=", pc)) {
    param.use_watchdog = atoi(pc + 3) ? true : false;
    param_dirty = true;
    Serial.printf("Use Watchdog: %u\n", param.use_watchdog);
  } else if (str_cmatch("k", pc)) {
    if (*(pc + 1)) {
      uint16_t idx = strtoul(pc + 1, &pc, 10);
      if (idx < 0 || idx >= ANZ_KOEFF)
        res = -2002;  // Illegal Index
      else if (*pc == '=') {
        float nkf = strtof(pc + 1, &pc);
        if (*pc)
          res = -2004;  // Format
        else {
          param.koeff[idx] = nkf;
          param_dirty = true;
          Serial.printf("koeff[%u]: %f\n", idx, nkf);
        }
      }
    } else {  // Without anything just show coefficients with description
      for (uint16_t i = 0; i < ANZ_KOEFF; i++) {
        Serial.printf("koeff[%u]: %f %s\n", i, param.koeff[i], koeff_desc[i]);
      }
    }
  } else if (str_cmatch("e", pc)) {            // e Measure
    uint16_t ival = strtoul(pc + 1, &pc, 10);  // Measurement param
    Serial.printf("Measure(%u):\n", ival);
    if (!ival)
      ival = 1;           // >0 Always with HK, only 0 is "without"
    res = measure(ival);  // Can also return general error
    if (res >= 0) {
      CHANNEL_VALUE *pkv = channel_value;
      for (int16_t i = 0; i < anz_values; i++) {
        Serial.printf("#%u: ", i);
        if (pkv->fe.errno == NO_ERROR) {
          Serial.printf("%f ", pkv->fe.floatval);
        } else {
          Serial.printf(getLTXErrorfmt(pkv->fe.errno), pkv->fe.errno);  // Unknown as number, known as text
        }
        Serial.printf("%s\n", (pkv->unit) ? pkv->unit : "");  // Optional unit
        pkv++;
      }
      res = 0;
    }
#if HK_FLAGS & 1
    Serial.printf("H90:BatV: %.3f V(Bat)\n", hk_value[HK90_VBAT]);
#endif
#if HK_FLAGS & 2
    Serial.printf("H91:Temp: %.1f "
                  "\xc2\xb0"
                  "C(int)\n",
                  hk_value[HK91_TEMP]);
#endif
#if HK_FLAGS & 4
    x  // *todo* HUM
#endif
#if HK_FLAGS & 8
      Serial.printf("H93:Energy: %.3f mAh\n", hk_value[HK93_MAH]);
#endif
    // Some info
    if (param_dirty)
      Serial.printf("INFO: Parameter not saved!\n");
  } else if (str_cmatch("i", pc)) {            // Transmit - Manually always as 'e1'
    uint16_t ival = strtoul(pc + 1, &pc, 10);  // Measurement param
    Serial.printf("Manual Transfer(%u)...\n", ival);
    if (ival)
      mlora_info.stat.hk_dcnt = 0;  // Parameter always with HK
    mlora_info.stat.flags &= 0xF0;  // flags, forget rest
    mlora_info.stat.flags |= 3;     // 3 Manual Transfer, overwrites evAlarms
    res = measure(ival);            // Can also return general error
    if (res >= 0) {
      res = lora_transfer();
    }
    mlora_info.stat.flags &= 0xF0;  // Clear manual
  }
#endif  // USAGE_STANDALONE
#if defined(USAGE_EXT_AT)
  else if (str_cmatch("send ", pc)) {
    uint16_t port = strtoul(pc + 5, &pc, 10);
    if (port < 1 || port > 223) res = -2012;

    else if (*pc != ',') res = -2013;  // ':' not possible ( == new argument)
    else {
      pc++;
      mlora_info.txframe.txanz = 0;
      mlora_info.txframe.txport = port;
      for (;;) {
        char c = *pc;
        if (*pc == 0) break;
        if ((c < '0' || c > '9') && (c < 'a' || c > 'f') && (c < 'A' || c > 'F')) {
          res = -2015;  // Illegal chars
          break;
        }
        c = *(pc + 1);
        if ((c < '0' || c > '9') && (c < 'a' || c > 'f') && (c < 'A' || c > 'F')) {
          res = -2016;  // Odd number
          break;
        }
        if (mlora_info.txframe.txanz >= MAX_PAYBUF_ANZ) {
          res = -2014;
          break;
        }
        mlora_info.txframe.txbytes[mlora_info.txframe.txanz++] = fpool_get_hex(&pc, 2);
      }
      if (!mlora_info.txframe.txanz) res = -2011;  // Nothing to send

      if (!res) {
        res = lora_send_packet();
      }
    }
  }
#endif
  else if (!strcasecmp(pc, "reset")) {  // Reset Board
    Serial.printf("Reset...\n");
    delay(50);
    api.system.reboot();
  } else
    res = -2001;  // Command Unknown
#if defined(LED_PIN)
  digitalWrite(LED_PIN, HIGH);  // LED OFF
#endif
  return res;  // OK
}

/* LTX own AT-commands: Call e.g. 'ATC+LTX' => argc:0,  'ATC+LTX=1:2' => argc: 2: '1','2' cmd contains 'ATC+LTX'
 * Also calls like 'atc+ltx test=33' possible => arc: 1: 'test=33' separator ONLY colon!
 * Testhandler:
 *  Serial.printf("cmd:%s argc:%d ", cmd, param->argc);
 *   for (int16_t i = 0; i < param->argc; i++) {
 *     Serial.printf(" [%d]:'%s'", i, param->argv[i]);
 *   }
 *   Serial.printf("\n");
 */
int ltx_cmd_handler(SERIAL_PORT _port, char *cmd, stParam *param) {
  int16_t res;

  if (param->argc != 1)
    res = -2009;
  else {
    dbg_until_runtime = now_runtime + 600;  // Debug for 10 min after commands
#if defined(USAGE_STANDALONE)
    int32_t delta2next = next_periodic_runtime - now_runtime;
    if (delta2next < 60)
      next_periodic_runtime = now_runtime + 60;  // No transmission for 1 min. after commands
#endif
    res = parse_ltx_cmd(param->argv[0]);
  }
  if (res) {
    Serial.printf("ERROR: %d\n", res);
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}
bool init_ltx_ats(void) {
  bool bres = api.system.atMode.add((char *)"LTX",
                                    (char *)"LTX Commands",
                                    (char *)"LTX", ltx_cmd_handler,
                                    RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
  return !bres;
}

// ---- Setup() -------------
extern const char *sw_version;
void setup() {
  int16_t res = 0;

  dbg_until_runtime = 600;  // 10 minutes debug after reset

  ltxtb_init();

  Serial.begin(Serial.getBaudrate());
  Serial.printf("*** " DEV_FAMILY " (C)JoEmbedded.de ***\n");
  Serial.printf("MAC:%08X%08X DEVICE_TYPE:%u V%u.%u\n", get_mac_h(), get_mac_l(), DEVICE_TYPE, DEVICE_FW_VERSION / 10, DEVICE_FW_VERSION % 10);
  Serial.printf("Version: %s\n\n", sw_version);

#if defined(USAGE_STANDALONE)
  Serial.printf("Usage: STANDALONE\n");
#endif
#if defined(USAGE_EXT_AT)
  Serial.printf("Usage: EXTENDED_AT\n");
#endif

  if (api.system.lpmlvl.get() != 2)
    api.system.lpmlvl.set(2);  // Enable LowPowerMode
  if (api.system.lpm.get() != 1)
    api.system.lpm.set(1);  // Enable Low Power

#if defined(USAGE_STANDALONE)
  parameter_nvm_load();
  if (param.period < 10)
    param.period = 10;                      // Min. 10 sec period
  next_periodic_runtime = START_DELAY_SEC;  // Earliest after xx seconds 1st measurement
  mlora_info.stat.flags = 128;              // mtimes.flags: RESET signal and next UE probably AUTOMATIC

  if (param.use_watchdog) {
    res = jo_wdt_init();
    Serial.printf("Watchdog enabled\n");
  }
#endif  // #if USAGE_STANDALONE

#if defined(LED_PIN)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // LED ON
#endif

  if (!res)
    res = init_ltx_ats();

#if defined(USAGE_STANDALONE)
  // WDT_TIMER (#0) - Enable WDT Background Feed
  if (!res) {
    uint32_t int_sec = param.period;
    if (int_sec > 120)
      int_sec = 120;  // Max every 2 minutes
    if (wdt_in_use && (int_sec > WDT_TIMER_PERIOD_SEC))
      int_sec = WDT_TIMER_PERIOD_SEC;  // Or less if WD active
    if (api.system.timer.create(RAK_TIMER_0, (RAK_TIMER_HANDLER)apptimer_timer_handler, RAK_TIMER_PERIODIC) != true)
      res = -1001;  // Timer0 Fail1
    else if (api.system.timer.start(RAK_TIMER_0, (int_sec * 1000), (void *)0) != true)
      res = -1002;  // Timer0 Fail2
    Serial.printf("Period: Transfer/Wakeup: %u/%u sec\n", param.period, int_sec);
  }
#endif  // USAGE_STANDALONE

#if defined(LED_PIN)
  if (!res)
    res = blink_init();  // A timer for LED Signals
#endif

  show_credentials();

  api.lorawan.registerJoinCallback(join_cb);
  api.lorawan.registerRecvCallback(recv_cb);
  api.lorawan.registerTimereqCallback(timereq_cb);
  api.lorawan.registerSendCallback(send_cb);

#if defined(LED_PIN)
  // 0: OK
  blink_timer_setvalrun(10, 8);  // 8 times short blink
#endif

  if (res) {
    Serial.printf("ERROR: setup():%d, Reset...\n", res);
    delay(10000);
    api.system.reboot();
  }

#if defined(USAGE_STANDALONE)
  user_setup();
#endif
}
// Useless Idle-Task, due to timer
void loop() {
  api.system.scheduler.task.destroy();
}
// ***
