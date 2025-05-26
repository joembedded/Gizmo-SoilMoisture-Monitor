/*
RAK3272_SIP_Basis_WD_Timer - joembedded@gmail.com

Dieser Sketch implementiert einen kompletten LTX-Lora-Knoten auf einem RAK3172-E/T/L/SIP
Dazu das entsprechende Board im Arduino Board-Manager eintragen.
Ist noch einiges zu tun (ADC, Finetuning, ..), aber Uplink, Watchdog, Payload, Download klappt bereits 1a

Beispiel C: c:\dokus\Lora\rak\rui3\cores\STM32WLE\component\service\battery\service_battery.c
driver: C:\dokus\Lora\rak\rui3\cores\STM32WLE\component\core\mcu\stm32wle5xx\uhal\uhal_adc.c
Oversampling wohl nicht vorgesehen oder implementiert in RUI3


      LTX-Payload-Decoder: https://github.com/joembedded/payload-decoder

      RUI3 Doku: https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/arduino-api
        WD: https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/watchdog/
        C: C:\dokus\Lora\rak\rui3\cores\STM32WLE\component\core\mcu\stm32wle5xx\uhal\uhal_wdt.c
        Demo: https://github.com/RAKWireless/RUI3-Best-Practice/blob/main/RUI3-LowPower-Example/RUI3-LowPower-Example.ino
        Bei alten Boards  erstmal volle FW-Upgrade vor aufspieln einer neuer Version!
        Lauft auf RAK3172-E(T) und -(L)SIP, einstellen als Board!
*/


// Globals
//#define DEVICE_TYPE 8000          // 8000: Basic System
//#define DEV_FAMILY "LTX-RAK3172"  //
#define DEVICE_TYPE 8001               // 8001: E/T-Modul
#define DEV_FAMILY "LTX-RAK3172-E(T)"  //
#define DEVICE_FW_VERSION 1            // STep of 0.1
#define HK_FLAGS 11                    // 1:V 2:T 4:H: 8:E Spaeter noch evtl. rH/T
#define ANZ_KOEFF 8                    // Koefficients for Measure, Minimum 4*2
#define MAX_CHANNELS 4                 // Macht Sinn das mit ANZ_KOEFFS zu kombinieren
#define START_DELAY_SEC 60             // Start LoRa after x sec

// 3V3:
#define MESSEN_ENERGY 1000  // Annahme erstmal 1mA fuer 1000 msec
#define PER_BYTE_ENERGY 90  // Fuer 3172-E an 3V3, Anpassen an -L
#define JOIN_ENERGY 180000  // Fuer 3172-E an 3V3, und und vorh. Stepper


// Magic Nv.RAM fuer Energie (auch nach Reset)
#define RAM_MAGIC (0xBADC0FFEUL)
uint32_t _tb_novo[4] __attribute__((section(".non_init")));
#define hk_batperc_h _tb_novo[1]
#define hk_batperc_h_neg _tb_novo[2]
#define hk_batperc_sum32 _tb_novo[3]

#define LED_PIN PA4  // Standard LED gg. VCC


// Init _tb_novo-nonvolatile RAM
void tb_init(void) {
  // Check RAM and init counter
  if (_tb_novo[0] != RAM_MAGIC || (_tb_novo[1] != ~_tb_novo[2])) {
    _tb_novo[0] = RAM_MAGIC;  // Init Non-Volatile Vars
    _tb_novo[1] = 0;          // Reserviert fuer User Energycounter H
    _tb_novo[2] = ~0;         // ~Reserviert fuer User Energycounter H Neg. Wert
    _tb_novo[3] = 0;          // Reserviert fuer User Energycounter L
  }
}
/* Auswertung, z.B. im cmd-handler
  ... } else if (str_cmatch("xxx", pc)) {  // DEBUG: xxx fuer internes
    Serial.printf("---xxx---\n");
    for (uint16_t i = 0; i < 4; i++) {
      Serial.printf("t[%u]: %x %d\n", i, _tb_novo[i], _tb_novo[i]);
    }
*/


#define MAX_PAYBUF_ANZ 51  // Anzahl maximal zu empfangender/sender Bytes
typedef struct {
  struct {             // Connection Sachen
    bool device_init;  // true wenn device init
    uint32_t join_runtime;
    // Da sich der Server abundan auch mit Servicezeugs meldet, kann das als Lebenszeichen verw. werden
    uint32_t last_server_reply_runtime;  // Wann zum letzten Mal was vom Server vernommen? Reply

    // Hier z.B. noch RSSI / SNR vom RX-CB
  } con;
  struct {                                 // Gesetzte Parameter
    uint8_t txanz;                         // Zu sendende Nachricht wenn >0
    uint8_t txport;                        // Zu sendender Port, 1-223 erlaubt
    uint8_t txbytes[MAX_PAYBUF_ANZ + 32];  // Immer Binaer, aber technische Reserve
  } par;
} MLORA_INFO;                  // Modem Lora Info
extern MLORA_INFO mlora_info;  // Modem Lora Info
MLORA_INFO mlora_info;


#define MAX_MESSCMD 79
#define _PMAGIC (0xCAFFEBA0 + DEVICE_TYPE * ANZ_KOEFF)  // Magic for valid parameters
typedef struct {
  uint32_t _pmagic;
  char messcmd[MAX_MESSCMD + 1];  // Definiert das Messkommando
  uint32_t period;                // (60..3599), 3600, xxx sec
  uint16_t hk_reload;             // Wenn >=1: mit HK mit uebertragen
  // Bei >= 1 (-199) LoraPort, obere 1000 enthalten fuer ASL 1:F16, 0:F32
  // also z.B. 11: rh/T(F32) oder dto 1011(F16)
  uint16_t sensor_profile;
  bool use_watchdog;

  // End: Koefficients [MUL, OFFSET, MUL, OFFSET, ..]
  float koeff[ANZ_KOEFF];
} PARAM;
extern PARAM param;
extern bool param_dirty;                   // True wenn dirty
extern const char *koeff_desc[ANZ_KOEFF];  // Description for the coefficients

/* Watchdog wird empfohlen, wenn aus: Raster max. 120 sec, sonst Raster max. 30*/
PARAM param = { _PMAGIC, "", 3600, 6, 1, /*WD*/ true, { 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0 } };
const char *koeff_desc[ANZ_KOEFF] = {
  "Multi Channel #0", "Offset Channel #0",
  "Multi Channel #1", "Offset Channel #1",
  "unused", "unused",
  "unused", "unused"
};

bool param_dirty = false;

typedef struct {
  int16_t flags;  // 15 Bits Flags oder wenn <0: Error / "Reason"
                  // &128:RESET
                  // (&64:Alarm) unused here
                  // (&32alterAlarm) unused here
                  // &15: Reason: 2:Auto, 3 Manual, Rest n.d.

  uint8_t hk_dcnt;  // Bei 0: in jedem Fall HK Mitmessen
  bool hk_valid;    // True wenn Valid fuer diese Messung
} MTIMES;
extern MTIMES mtimes;  // Measure Intern
MTIMES mtimes;         // Measure Intern


// Periodic
uint32_t next_periodic_runtime;  // Messen wenn runtime >= next_periodic_runtime

// --- SEC_TIMER ---
uint32_t dbg_until_runtime;  // Solange > now_runtime: Output
uint32_t now_runtime = 0;    // Runtime in sec seit Reset
// Locals
static uint32_t _omil = 0;  // Overflow alle 49d!
// Runtime auf aktuellen Stand bringen
void update_runtime(void) {
  uint32_t _nmil = millis();
  int32_t nsec = ((int32_t)(_nmil - _omil)) / 1000;
  now_runtime += nsec;
  _omil = _nmil;
}

// Watchdog jo_wdt: Timeout 32sec fix!.
// Watchdog sollte immer gefeeded werden wenn etwas laenger als 2 sec dauert
#include "uhal_wdt.h"

extern bool wdt_in_use;
bool wdt_in_use;  // True if used/enabled

#define WDT_TIMER_PERIOD_SEC 30  // for auto_feed Max. bis TIMER_PERIOD 30
#define ENERGY_WDT 100           // Avg. 10 mA fuer 10 msec, gibt ca. zus. 3.5uAh wenn an

#define IWDG_WINDOW 0xFFF  // Zu schnelles Feed loest WDT-Window aus!
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
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) return -1000;
  return 0;
}

/* --- Analog-Routinen -----
* extern analog PINs:
*   PB3   PB4  PB2  A10  A15
* intern analog channels:
*   UDRV_ADC_CHANNEL_TEMPSENSOR
*   UDRV_ADC_CHANNEL_VREFINT
*   UDRV_ADC_CHANNEL_VBAT (see Datasheet 3.18.3)*/
#include "udrv_adc.h"
void analog_setup(void) {
  udrv_adc_set_resolution(UDRV_ADC_RESOLUTION_12BIT);  // STM32 hat ADC12-Bit
}
// Einzelmessung, ca. 800uSec - Result: 0..4095 entspr 0..3600 mV (?)
int16_t analog_read_single(uint32_t apin) {
  int16_t val16;
  udrv_adc_read(apin, &val16);
  return val16;
}
/* Mehrfachmessung. Evtl. WD bedienen!
* Result: 0.0 .. 4095.0 entspr 0..3600 mV (?) */
float analog_read_multi(uint32_t pin, uint16_t manz) {  // manz: >0!
  int32_t temp32 = 0;
  for (uint16_t i = 0; i < manz; i++) {
    int16_t temp16;
    udrv_adc_read(pin, &temp16);  // ca. 800usec/Sample
    temp32 += temp16;
  }
  return (float)temp32 / (float)manz;
}
/* Read internal Temperature, Recommended: manz 4-10
* CPU comes with calibration Coeffs () (+/- 5°C!) */
float analog_read_internal_temp(uint16_t manz) {
  float tempadf = analog_read_multi(UDRV_ADC_CHANNEL_TEMPSENSOR, manz);
  int16_t tcal30 = *(uint16_t *)0x1FFF75A8;   // ca. 950 Ausm Datenblatt 3.18.1
  int16_t tcal130 = *(uint16_t *)0x1FFF75C8;  // ca. 1200
  if (tcal130 > tcal30) return (100.0 / (float)(tcal130 - tcal30)) * (tempadf - (float)tcal30) + 30.0;
  else return -99.0;
}
// Analog-End

/* Kontext des Handlers ist "normal"
* Achtung: der Timer hat einen Rahmen von 9msec @ ca. 10mA msec und selbst, Schnitt also ca. 3->7uA
* bei Aufruf nur alle 10 sec steigt der Stromverbrauch auf Avg. 15uA!
* Timer wird fuer opt. WD und Periodic verwendet, da anscheinend nur 1 Timer sauber laeuft, bzw. das WD-Window evtl. ausloest
* Wenn keine WD ist Intervall max. 120 Sek. sonst 30 Sekunde wg. WD
* Wichtig: Watchdog alle <= 32 Sekunden feeden!*/
void apptimer_timer_handler(void *data) {


  if (wdt_in_use) jo_wdt_feed();  // Verifiziert: WD loest sauber 32 sec nach letztem feed aus
  update_runtime();
  if (now_runtime < next_periodic_runtime || (!mlora_info.con.device_init)) {
    if (dbg_until_runtime > now_runtime) {
      if (wdt_in_use) Serial.printf("[WDT %u]\n", now_runtime);
      else Serial.printf("[Wake %u]\n", now_runtime);  // Wachen ohne Watchdog
    }
#if (HK_FLAGS & 8)
    hk_add_energy(ENERGY_WDT);  // Energy fuer 1 Watchdog
#endif
  } else {
    Serial.printf("[PERIODIC %u] - Auto Transfer...\n", now_runtime);
    mtimes.flags |= 0x12;      // Messung und Transfer starten. Mind mal AUTO
    int16_t res = measure(0);  // Kann auch allg. Fehler lefern
    if (res >= 0) {
      res = lora_transfer();
    }

    if (res) Serial.printf("ERROR: %d\n", res);

    while (next_periodic_runtime <= now_runtime) next_periodic_runtime += param.period;
  }
}

//------ JoEmb Toolbox---------
// universeller String
#define UNI_LINE_SIZE 199
char uni_line[UNI_LINE_SIZE + 1];

// universeller Byte-Buffer
#define UNI_BUF_SIZE 256
uint8_t uni_buf[UNI_BUF_SIZE];

// Access 64 Bit unique ID
uint32_t get_mac_l(void) {
  return *(uint32_t *)0x1FFF7580;
}
uint32_t get_mac_h(void) {
  return *(uint32_t *)0x1FFF7584;
}

// Ein eigener Pseude-Zufallsgenerator 16-Bit - Z.B- fuer LoRa-Keys
// Ratsam zu initialisieren!
static uint32_t _seed = 0;
void my_seed(uint32_t s) {
  _seed = s;
}
uint16_t my_rand16(void) {
  _seed = (_seed * 1664525 + 1013904223);  // LCG-Formel (Wikipedia)
  return (uint16_t)(_seed >> 8);           // Ergebnis aus der Mitte
}
void set_my_rbytes(uint8_t *pu, int16_t anz) {
  while (anz--) {
    *pu = (my_rand16() & 15) << 4;
    *pu++ |= (my_rand16() & 15);
  }
}
// String-Matcher Match "what" ind *pstr
bool str_cmatch(const char *what, char *pstr) {
  uint8_t c;
  for (;;) {
    c = *what++;
    if (!c)
      return true;  // All equal until finish!
    if (c != (*pstr++))
      return false;  // Not equal
  }
}

// Parameter Save/Load Flash
int16_t parameter_nvm_load(void) {
  uint32_t _ptest = 0;
  api.system.flash.get(0, (uint8_t *)&_ptest, sizeof(uint32_t));
  if (_ptest != _PMAGIC) return -2002;  // No Params
  param._pmagic = 0;
  api.system.flash.get(0, (uint8_t *)&param, sizeof(param));
  if (param._pmagic != _PMAGIC) return -2003;  // Read Error!!!
  return 0;                                    // OK
}
int16_t parameter_nvm_save(void) {                                               // set param._pmagic to 0 for Flash-Clear
  if (!api.system.flash.set(0, (uint8_t *)&param, sizeof(param))) return -2004;  // Schreibfehler?
  param_dirty = false;
  return 0;
}

// Messen START
// F16-Helper START
// Simuliert Float16 auf ARDUINO - Nur als One-Way-Konversion fuer UPLINK vorgesehen!
uint16_t float_to_half(float f) {
  union {
    float f;
    uint32_t u;
  } u = { f };
  uint32_t f_bits = u.u;
  uint32_t sign = (f_bits >> 16) & 0x8000;  // sign shift to 16-bit position
  uint32_t exponent = (f_bits >> 23) & 0xFF;
  uint32_t mantissa = f_bits & 0x7FFFFF;
  if (exponent == 255) {                 // Inf or NaN
    if (mantissa) return sign | 0x7E00;  // NaN
    else return sign | 0x7C00;           // Inf
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
// F16-Helper Ende


typedef union {  // Umwandlung FLOAT->Binaer
  uint32_t ulval;
  float fval;
} FXVAL;

#define NO_ERROR 0
// Darstellung einer allgemeinen Fliesskommazahl mit Option zum Fehler:
typedef struct {
  uint16_t errno;  // errno 0: No ERROR (1-1023 asl Fehlercode verwendbar)
  float floatval;  // Wert nur bei errno 0 ob relevant, wird sonst ignoriert
} FE_ZAHL;

typedef struct {
  FE_ZAHL fe;  // Der Messwert
  // Optional Platz fuer Meta-Daten, z.B. Typ-spec Einheiten
  char *unit;  // Fuer "e": Einheit zum Anzeigen oder NULL
} CHANNEL_VALUE;

extern CHANNEL_VALUE channel_value[MAX_CHANNELS];
extern uint16_t anz_values;  // Anzahl der Messwerte


#define HK_MAXANZ 4
#define HK90_VBAT 0                // Bat in mV Index
#define HK91_TEMP 1                // Temp in 0.1 Grad
#define HK92_HUM 1                 // Feucht in 0.1 Proc falls vers
#define HK93_MAH 2                 // Energie
extern float hk_value[HK_MAXANZ];  // alt: int16!

CHANNEL_VALUE channel_value[MAX_CHANNELS];
uint16_t anz_values;        // Anzahl der Messwerte
float hk_value[HK_MAXANZ];  // Neu: HL als Float

// User-Measure-Routinen
float user_measure_battery(void) {
  return api.system.bat.get();
}
float user_measure_temperature(void) {
  return analog_read_internal_temp(10);  // 8 msec Auslagern
}

// Minimal HK
void measure_hk(void) {
#if HK_FLAGS & 1
  hk_value[HK90_VBAT] = user_measure_battery();
#endif
#if HK_FLAGS & 2
  hk_value[HK91_TEMP] = user_measure_temperature();
#endif
#if HK_FLAGS & 8
  uint32_t h;
  // Direktes Umrechnen im mAH mit Justieren
  h = (hk_batperc_sum32 & 0xFFC00000);  // 1.165mAH-Ueberlauf? (0-4194304).16
  if (h) {
    hk_batperc_sum32 -= h;
    hk_batperc_h += (h / 4194304);     // 22 Bits
    hk_batperc_h_neg = ~hk_batperc_h;  // Alle ca. 1 mAh NEG aendern
  }
  hk_value[HK93_MAH] = (hk_batperc_h * 1.1650844) + (hk_batperc_sum32 * 277.778e-9);
#endif
#if HK_FLAGS
  mtimes.hk_valid = true;
#endif
}
#if HK_FLAGS & 8
// Idee Energie in mAmsec addieren und regelmaessig mA-Bit shiften, 1mAms entsprechen 1uC
// hk_add_energy(1*35000) fuer 1 sec 35000uA)
void hk_add_energy(uint32_t mAmSec) {
  hk_batperc_sum32 += mAmSec;
}
#endif


/*  -- Simulation-- Typspezifisch messen. 
* Mehrere Moeglichkeiten, bei ival>0 immer HK Mitmessen
* "Regulaere" Messung ival=0
* Feeding the Watchdg required if takes >1 sec */

int16_t measure(int16_t ival) {
  if (!mtimes.hk_dcnt || ival) {
    measure_hk();
    mtimes.hk_dcnt = param.hk_reload;
  } else {
    mtimes.hk_dcnt--;
    mtimes.hk_valid = 0;
  }

  // Simulierte Messung
  int16_t bat16;
  udrv_adc_read(UDRV_ADC_CHANNEL_VBAT, &bat16);
  int16_t ref16;
  udrv_adc_read(UDRV_ADC_CHANNEL_VREFINT, &ref16);

  // Evtl. linearisieren, z.B. .floatval = (fval * param.koeff[x]) - param.koeff[x+1]
  channel_value[0].fe.errno = 0;  // Im Fehlerfall Code
  channel_value[0].fe.floatval = (float)bat16;
  channel_value[0].unit = "(bat16)";  // Fuer Display
  channel_value[1].fe.errno = 0;
  channel_value[1].fe.floatval = (float)ref16;
  channel_value[1].unit = "(ref16)";  // Fuer Display
  anz_values = 2;                     // global

#if (HK_FLAGS & 8)
  hk_add_energy(MESSEN_ENERGY);  // Energy fuer 1 Messung
#endif
  return anz_values;
}
// Messen Ende




// Lora Default Credentials erzeugen band: 4:EU868
// Credentials Bsp: AT+DEVEUI=0080E1150032D493 => AT+APPEUI=02EF62D6B9DD3A29 => AT+APPKEY=290DDBCD739E45B2857F87F86092838F
int16_t lora_setup(uint8_t band) {
  int16_t res = 0;  // Assume OK
  // Keys setzen und init  LoRaWAN credentials
  my_seed(get_mac_l());      // Immer die selbe Sequenz, abh. von MAC
  uint32_t h = get_mac_h();  // LTX: Big Endian Style
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

  if (!api.lorawan.band.set(band)) res = -2101;  // 4:868 MHz, ..
  api.lorawan.njm.set(1);                        // 1: OTAA
  api.lorawan.rety.set(0);                       // Kein RX-Best. Wdh
  api.lorawan.cfm.set(1);                        // Aber TX bestaetigen lassen
  api.lorawan.dr.set(0);                         // Falls evtl. noch > 0 von zuvor
  api.lorawan.adr.set(1);                        // !!!Automatic Data Reduction An, nicht sinnvoll fuer Moving Devices!!!
  return res;
}

// lora_transfer() und Zubehoer
// Payload_flags
#define PLWITH_ALL 3  // Kombi
#define PLWITH_VALUES 1
#define PLWITH_HK 2

uint8_t *measure_payoff_lorahdr(void) {
  uint8_t *pu = mlora_info.par.txbytes;  //  MAX_PYBUF_AND+32 Platz
  *pu++ = mtimes.flags;
  return pu;
}
/******************************************************************************************
 * Komprimierte Daten generieren -BYTES
 * uint8_t* measure_payoff(uint8_t *pu) 
 * Optimierte Version fuer LTX-Payload. Zusammenbau der Payload etwas komplizierter,
 * dafuer sehr platzsparend . F16-Check: https://evanw.github.io/float-toy/
 *
 * Aufloesung F16/32 via +1000 im fPort == param.sensor_profile (aber dann halt fuer alle)
 **************************************************************************************/
uint8_t *measure_payoff_mess(uint8_t *pu, uint8_t payload_flags) {
  /* Kanaele 0..89: "cursor"= Kanal implizit bei 0
   * Dann 0Fxxxxxx 6 Bits Anzahl  F:0:Float32, F:1:Float16 xx: Anzahl folgend, bei 0: cursor folgt
   *      1bbbbbbb 7 Bits HK, startet mnd. ab 90, wenn mehr als 7 HK: 2*
   * Hier eh nur max. 25 Kanale mgl. wg. LoRa Payload-Laenge <= 51 Bytes
   */
  if (payload_flags & PLWITH_VALUES) {
    uint16_t mcursor = 0;
    // ---F16---
    if ((param.sensor_profile / 1000) == 1) {  // 1k-Flag im potz: Alles auf F16, sonst F32
      uint16_t anzf16 = (uint8_t)anz_values;   //
      *pu++ = (anzf16 | 64);                   // anz F16 folgen
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
      *pu++ = (uint8_t)anzf32;  // anz F32 folgen
      while (anzf32--) {
        FXVAL fxv;
        if (channel_value[mcursor].fe.errno == NO_ERROR) {
          fxv.fval = channel_value[mcursor].fe.floatval;
        } else {
          // alle 16k Fehler darstellbar
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
    *pu++ = (uint8_t)(0x80 | (HK_FLAGS & 127));  // HK ist FIX erstmal
    float fval;
    uint16_t fx16vu;
    // Fuer Simple-Gateway HKs fix
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
    x  // Hum fehlt noch *todo*
#endif
#if HK_FLAGS & 8
      fx16vu = float_to_half(hk_value[HK93_MAH]);
    *pu++ = (uint8_t)(fx16vu >> 8);
    *pu++ = (uint8_t)(fx16vu);
#endif
  }
  return pu;
}

/******* Lora-Transfer und Service ****************
* Gemessen fuer Standard-Modul mit 3V3
 * Energie:
 * 1 Bytes DR0  140mC
 * 48 Bytes DR0  285mC
 * 48 Bytes DR5  15mC
 * Join: 180mC
 * ca. 3mC/Byte in DR 0, ca. 0.16 in DR5 - So ungefaehr ausgependelt
 * energy += (paylen) + 90) * 1500) / (data_rate * 3 + 1);
 */

// Vor den Callbacks
extern int16_t parse_ltx_cmd(char *pc);

/* Eingehende Payload zerlegen und wie CMD behandeln, 0-terminiert */
void menu_parse_payload(char *pc) {
  while (*pc) {
    char *pc0 = pc++;
    while (*pc > ' ')
      pc++;
    char c = *pc;  // c kann 0 (Ende) oder <= ' ' sein (kommt noch was)
    *pc++ = 0;     // Lesen bis leerzeichen und String terminieren, evtl. LZ ueberlesen

    parse_ltx_cmd(pc0);

    if (!c)
      break;
  }
  // Evtl. speichern
  if (param_dirty == true) {
    parse_ltx_cmd("write");
  }
}

int16_t send_txplayload(void) {
  if (!mlora_info.par.txanz) {
    Serial.println("ERROR: Nothing to send");
    return -2011;
  }
  if (!api.lorawan.send(mlora_info.par.txanz, mlora_info.par.txbytes, mlora_info.par.txport)) {
    Serial.println("ERROR: Send failed");  // Bool
    return -2010;
  }
  return 0;
}

uint16_t rejoins = 0;  // Retry-Counter
void join_cb(int32_t status) {
  if (status == RAK_LORAMAC_STATUS_OK) {
    rejoins = 0;
    Serial.println("Network joined");
    send_txplayload();  // Sollte da sein
    mlora_info.con.join_runtime = now_runtime;
#if HK_FLAGS & 8
    hk_add_energy(((mlora_info.par.txanz + PER_BYTE_ENERGY) * 1500) / (api.lorawan.dr.get() * 3 + 1));  // PACKET ENERGY
#endif
  } else {
    Serial.printf("ERROR: %d, Join failed\n", status);
    if (rejoins) {
      Serial.printf("Retry(%u) Join Network...\n", rejoins);
      rejoins--;
      api.lorawan.join();
#if HK_FLAGS & 8
      hk_add_energy(JOIN_ENERGY);
#endif
    }
  }
}

// Lora Callbacks -> SERVICE_LORA_RECEIVE_T https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/lorawan#service_lora_receive_t
void recv_cb(SERVICE_LORA_RECEIVE_T *data) {
  mtimes.flags = 0;  // Beim Server angekommen
  mlora_info.con.last_server_reply_runtime = now_runtime;
  uint8_t rlen = data->BufferSize;
  if (dbg_until_runtime > now_runtime) {
    Serial.printf("Received %u Bytes on Port %u\n", rlen, data->Port);
    Serial.printf("RxDR:%u RSSI:%d SNR:%d DL-Counter:%u\n", data->RxDatarate, data->Rssi, data->Snr, data->DownLinkCounter);

    if (rlen) {
      Serial.printf("Data: ");
      for (int i = 0; i < rlen; i++) {
        Serial.printf("%02x", data->Buffer[i]);
      }
      Serial.print(" '");
      for (int i = 0; i < rlen; i++) {
        uint8_t c = data->Buffer[i];
        if (c >= ' ' && c < 128) Serial.printf("%c");
        else Serial.printf("<%u>", c);
      }
      Serial.print("'\n");
    }
  }
  // Payload auswerten:
  // LTX nur Port 10! Und ist ein String mit Space als Trenner ('p=600 cmd=Hallo')
  if (rlen && data->Port == 10) {
    data->Buffer[rlen] = 0;  // Terminate ASCII
    menu_parse_payload((char *)data->Buffer);
  }
}

/* Send-Callback: Sagt i.d.R. einfach nur 0, ca. 3 sec. nach send ??? Gut vlt. fuer ne LED? */
/*
void send_cb(int32_t status) {  
  if (status != RAK_LORAMAC_STATUS_OK) Serial.printf("ERROR: Send status: %d\n", status);
}
*/

/* Linkcheck wird eigtl. nicht verwendet, nur Platzhalter */
/*
void linkcheck_cb(SERVICE_LORA_LINKCHECK_T *data) {
  Serial.println("Linkcheck:");
  Serial.printf("State:%u ", data->State); // 0: Success 
  Serial.printf("DemodMargin:%u ", data->DemodMargin);
  Serial.printf("NbGateways:%u ", data->NbGateways);
  Serial.printf("RSSI:%u ", data->Rssi);
  Serial.printf("SNR:%u\n", data->Snr);
}
*/

/* Timereq wird nicht verwednet, nur Platzhalter */
/*
void timereq_cb(int32_t status) {
  Serial.println("TIMEREQ-CB");
  if (status == GET_DEVICE_TIME_OK) {
    Serial.println("Get device time success");
  } else if (status == GET_DEVICE_TIME_FAIL) {
    Serial.println("Get device time fail");
  }
}
*/
/* Das scheint nicht zu klappen
    struct tm * localtime;
    api.lorawan.ltime.get(localtime);
    Serial.printf("Current local time : %d:%d:%d\n", localtime->tm_hour, localtime->tm_min, localtime->tm_sec);
*/
/* Das hier klappt
    char local_time[30] = { 0 };
    service_lora_get_local_time(local_time);
    Serial.printf("LTime:'%s'\n", local_time);  // '23h59m54s on 05/24/2025' Anm.:24.5.2025 (=US-Format)
*/
// ---CB End


int16_t lora_transfer(void) {
  uint8_t *phu = measure_payoff_lorahdr();
  phu = measure_payoff_mess(phu, mtimes.hk_valid ? PLWITH_ALL : PLWITH_VALUES);
  int16_t paylen = (phu - mlora_info.par.txbytes);
  Serial.printf("LoRa-Transfer%s (%d Bytes)\n", mtimes.hk_valid ? " with HK" : "", paylen);
  if (paylen > MAX_PAYBUF_ANZ) {
    Serial.printf("ERROR: Payload too large, clipped to %u Bytes\n", MAX_PAYBUF_ANZ);
    paylen = MAX_PAYBUF_ANZ;  // Better soemthing than nothing
  }
  mlora_info.par.txanz = (uint8_t)paylen;
  mlora_info.par.txport = (param.sensor_profile % 1000) & 255;
  if (mlora_info.par.txport < 1 || mlora_info.par.txport > 223) {
    Serial.printf("ERROR: Invalid fPort %d: set to 1\n", mlora_info.par.txport);
    mlora_info.par.txport = 1;
  }
  if (dbg_until_runtime > now_runtime) {  // Show Payload

    Serial.printf("P[%u]:", mlora_info.par.txport);
    for (uint16_t i = 0; i < paylen; i++) Serial.printf("%02X", mlora_info.par.txbytes[i]);
    Serial.printf("\n");
  }
  // Nothing heard from Server after 12h: Re-Join!
  int32_t last_contact_sec = now_runtime - mlora_info.con.last_server_reply_runtime;
  if ((last_contact_sec > 43200) || !api.lorawan.njs.get()) {
    Serial.printf("Join Network...\n");
    rejoins = 3;  // 1+3 Versuche
    api.lorawan.join();
#if HK_FLAGS & 8
    hk_add_energy(JOIN_ENERGY);
#endif
  } else {
    send_txplayload();
#if HK_FLAGS & 8
    hk_add_energy(((mlora_info.par.txanz + PER_BYTE_ENERGY) * 1500) / (api.lorawan.dr.get() * 3 + 1));  // PACKET ENERGY
#endif
  }
  return 0;
}
// Show Credentials. Gibt 0 zurueck wenn nicht init (alles 0)
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
  Serial.printf("Adaptive Datarate(*1): %d\n", api.lorawan.adr.get());        // 1 Recom (aber !!!Automatic Data Reduction An, nicht sinnvoll fuer Moving Devices!!!)
  mlora_info.con.device_init = (bsum > 0);
  return bsum;
}


// Kommando-Handler nimmt nur Strings, Ret: 0: OK. Ohne WS!
int16_t parse_ltx_cmd(char *pc) {
  int16_t res = 0;  // Assume OK

  if (!strcasecmp(pc, "initeu868")) {
    res = lora_setup(4);                // Band 4:868 MHz
    show_credentials();
  } else if (str_cmatch("cred", pc)) {  // cred.. : Credentials and Setup
    show_credentials();
  } else if (str_cmatch("?", pc)) {  // ?: Reine Info-Fkt oder ?k Koeficients
    Serial.printf("Info:\n");
    Serial.printf("DEVICE_TYPE: %u\n", DEVICE_TYPE);
    update_runtime();
    if (!mlora_info.con.device_init) {
      Serial.printf("Device not init!\n");
    } else {
      Serial.printf("Runtime: %d sec\n", now_runtime);
      if (api.lorawan.njs.get()) {
        uint8_t buff[4];
        api.lorawan.daddr.get(buff, 4);
        Serial.printf("Joined (DEVADDR: %02X%02X%02X%02X) %d sec ago\n", buff[0], buff[1], buff[2], buff[3], now_runtime - mlora_info.con.join_runtime);
        Serial.printf("LastServerReply: ");
        if (mlora_info.con.last_server_reply_runtime) {
          Serial.printf("%d sec ago\n", now_runtime - mlora_info.con.last_server_reply_runtime);
        } else Serial.printf("Never!\n");
        Serial.printf("Datarate: %d\n", api.lorawan.dr.get());

      } else {
        Serial.printf("No Network!\n");
      }
    }
    if (param._pmagic != _PMAGIC) {
      Serial.printf("ERROR: Parameter invalid\n");
    } else {
      Serial.printf("Cmd: '%s'\n", param.messcmd);
      Serial.printf("Period: %u sec\n", param.period);
      Serial.printf("HK-Reload: %u\n", param.hk_reload);
      Serial.printf("Profile: %u\n", param.sensor_profile);
      Serial.printf("Use Watchdog: %u\n", param.use_watchdog);

      if (param_dirty) Serial.printf("INFO: Parameter not saved!\n");
    }
  } else if (!strcasecmp(pc, "write")) {  // Write Parameter
    Serial.printf("Write Parameter\n");
    res = parameter_nvm_save();
  } else if (!strcasecmp(pc, "factoryreset")) {  // FactoryReset
    param._pmagic = 0;
    parameter_nvm_save();
    Serial.printf("Factory Reset...\n");
    delay(50);
    api.system.restoreDefault();
    api.system.reboot();
  } else if (!strcasecmp(pc, "reset")) {  // Reset Board
    Serial.printf("Reset...\n");
    delay(50);
    api.system.reboot();
  } else if (str_cmatch("cmd=", pc)) {
    pc += 4;
    strncpy(param.messcmd, pc, MAX_MESSCMD);  // DSN
    param_dirty = true;
    Serial.printf("Cmd: '%s'\n", param.messcmd);
  } else if (str_cmatch("p=", pc)) {
    uint32_t np = atoi(pc + 2);
    if (np < 10) {  // Bloedsinnige Perioden ignorieren
      res = -2007;  // Min. 10 sec period
    } else if (np > 86400) {
      res = -2008;  // Max. 1/day
    } else {
      param.period = np;
      param_dirty = true;
      Serial.printf("Period: %u sec\n", param.period);

      uint32_t int_sec = param.period;
      if (int_sec > 120) int_sec = 120;  // Alle 2 Minuten Maximal
      if (wdt_in_use && (int_sec > WDT_TIMER_PERIOD_SEC)) {
        int_sec = WDT_TIMER_PERIOD_SEC;  // Oder weniger wenn WD aktiv
        jo_wdt_feed();
      }
      next_periodic_runtime = now_runtime + 30;                                                        // Traditionell nach 30 Sek geht es los
      if (api.system.timer.stop(RAK_TIMER_0) != true) res = -2005;                                     // Timer0 Faile1b
      else if (api.system.timer.start(RAK_TIMER_0, (int_sec * 1000), (void *)0) != true) res = -2006;  // Timer0 Fail2b
    }
  } else if (str_cmatch("hk=", pc)) {
    uint32_t nhk = atoi(pc + 3);
    if (nhk < 0) nhk = 0;  // Reload
    param.hk_reload = nhk;
    mtimes.hk_dcnt = 0;
    param_dirty = true;
    Serial.printf("HK-Reload: %u\n", param.hk_reload);
  } else if (str_cmatch("profile=", pc)) {
    uint32_t nprof = atoi(pc + 8);
    if (nprof < 1) nprof = 1;  // Prof 1-xxx (1000: F16)
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
      if (idx < 0 || idx >= ANZ_KOEFF) res = -2002;  // Ilegal Index
      else if (*pc == '=') {
        float nkf = strtof(pc + 1, &pc);
        if (*pc) res = -2004;  // Format
        else {
          param.koeff[idx] = nkf;
          param_dirty = true;
          Serial.printf("koeff[%u]: %f\n", idx, nkf);
        }
      }
    } else {  // Ohne was einfach nur Koeffs anzeigen mit Description
      for (uint16_t i = 0; i < ANZ_KOEFF; i++) {
        Serial.printf("koeff[%u]: %f %s\n", i, param.koeff[i], koeff_desc[i]);
      }
    }
  } else if (str_cmatch("e", pc)) {            // e Messen
    uint16_t ival = strtoul(pc + 1, &pc, 10);  // Mess-Param
    Serial.printf("Measure(%u):\n", ival);
    if (!ival) ival = 1;  // >0 In jedem Fall mit HK, nur 0 ist "ohne"
    res = measure(ival);  // Kann auch allg. Fehler lefern
    if (res >= 0) {
      CHANNEL_VALUE *pkv = channel_value;
      for (int16_t i = 0; i < res; i++) {
        Serial.printf("#%u: ", i);
        if (pkv->fe.errno == NO_ERROR) {
          Serial.printf("%f ", pkv->fe.floatval);
        } else {
          Serial.printf("Err%d ", pkv->fe.errno);  // Unbekannte als Zahl
        }
        Serial.printf(" %s\n", (pkv->unit) ? pkv->unit : "");  // Optional Unit dazu
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
    // Bissl Blabla
    if (param_dirty) Serial.printf("INFO: Parameter not saved!\n");

  } else if (str_cmatch("i", pc)) {            // Transmit - Manuell immer als 'e1'
    uint16_t ival = strtoul(pc + 1, &pc, 10);  // Mess-Param
    Serial.printf("Manual Transfer(%u)...\n", ival);
    if (ival) mtimes.hk_dcnt = 0;  // Parameter in jedem Fall mit HK
    mtimes.flags &= 0xF0;          // flags, Rest vergessen
    mtimes.flags |= 3;             // 3 Manual Transfer, ueberschreibt evAlarme
    res = measure(ival);           // Kann auch allg. Fehler liefern
    if (res >= 0) {
      res = lora_transfer();
    }
    mtimes.flags &= 0xF0;  // Manual loeschen
  } else res = -2001;      // Command Unknown
  return res;              // OK
}



/* LTX Eigene AT-Kommandos: Aufruf z.B. 'ATC+LTX' => argc:0,  'ATC+LTX=1:2' => argc: 2: '1','2' cmd enth 'ATC+LTX'
* Auch Aufrufe wie 'atc+ltx test=33' mgl. => arc: 1: 'test=33' Trenner NUR Doppelpunkt!
* Testhandler:   
*  Serial.printf("cmd:%s argc:%d ", cmd, param->argc);
*   for (int16_t i = 0; i < param->argc; i++) {
*     Serial.printf(" [%d]:'%s'", i, param->argv[i]);
*   }
*   Serial.printf("\n");
*/
int ltx_cmd_handler(SERIAL_PORT _port, char *cmd, stParam *param) {
  int16_t res;

  if (param->argc != 1) res = -2009;
  else {
    dbg_until_runtime = now_runtime + 600;  // Fuer 10 Min
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

  dbg_until_runtime = 600;  // 10 Minuten lang Debug nach Reset

  tb_init();

  Serial.begin(Serial.getBaudrate());
  Serial.printf("*** " DEV_FAMILY " (C)JoEmbedded.de ***\n");
  Serial.printf("MAC:%08X%08X DEVICE_TYPE:%u V%u.%u\n", get_mac_h(), get_mac_l(), DEVICE_TYPE, DEVICE_FW_VERSION / 10, DEVICE_FW_VERSION % 10);
  Serial.printf("Version: %s\n\n", sw_version);

  if (api.system.lpmlvl.get() != 2) api.system.lpmlvl.set(2);  // Enable LowPowerMode
  if (api.system.lpm.get() != 1) api.system.lpm.set(1);        // Enable Low Power

  parameter_nvm_load();
  if (param.period < 10) param.period = 10;  // Min. 10 sec period
  next_periodic_runtime = START_DELAY_SEC;   // Fruehestens nach xx Sekunden 1. Messung
  mtimes.flags = 128;                        // mtimes.flags: RESET Signalisieren und naechste UE vmtl. AUTOMATISCH

  if (param.use_watchdog) {
    res = jo_wdt_init();
    Serial.printf("Watchdog enabled\n");
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED turn on when input pin value is LOW
  analog_setup();               // Enable analog Routines

  if (!res) res = init_ltx_ats();

  // WDT_TIMER (#0) - Enable WDT Background Feed
  if (!res) {
    uint32_t int_sec = param.period;
    if (int_sec > 120) int_sec = 120;                                                                                              // Alle 2 Minuten Maximal
    if (wdt_in_use && (int_sec > WDT_TIMER_PERIOD_SEC)) int_sec = WDT_TIMER_PERIOD_SEC;                                            // Oder weniger wenn WD aktiv
    if (api.system.timer.create(RAK_TIMER_0, (RAK_TIMER_HANDLER)apptimer_timer_handler, RAK_TIMER_PERIODIC) != true) res = -1001;  // Timer0 Faile1
    else if (api.system.timer.start(RAK_TIMER_0, (int_sec * 1000), (void *)0) != true) res = -1002;                                // Timer0 Fail2
    Serial.printf("Period: Transfer/Wakup: %u/%u sec\n", param.period, int_sec);
  }
  show_credentials();

  api.lorawan.registerJoinCallback(join_cb);
  api.lorawan.registerRecvCallback(recv_cb);

  /* 3 unwichtige Callbacks, im Quellcode daktiviert
  api.lorawan.registerSendCallback(send_cb);
  api.lorawan.registerTimereqCallback(timereq_cb);
  api.lorawan.registerLinkCheckCallback(linkcheck_cb);
  */

  // 0: OK
  if (res) {
    Serial.printf("ERROR: setup():%d, Reset...\n", res);
    delay(10000);
    api.system.reboot();
  }
}
// Nutzloser Idle-Task, wg. Timer
void loop() {
  api.system.scheduler.task.destroy();
}
// ***