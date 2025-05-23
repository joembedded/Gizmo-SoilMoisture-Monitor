// RAK3272_SIP_Basis_WD_Timer
// RUI3 Doku: https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/arduino-api
//            WD: https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/watchdog/
//            C: C:\dokus\Lora\rak\rui3\cores\STM32WLE\component\core\mcu\stm32wle5xx\uhal\uhal_wdt.c
//            Demo: https://github.com/RAKWireless/RUI3-Best-Practice/blob/main/RUI3-LowPower-Example/RUI3-LowPower-Example.ino


// Globals
#define DEVICE_TYPE 8000          // 8000: Basic System
#define DEV_FAMILY "LTX-RAK3172"  //
#define DEVICE_FW_VERSION 1       // STep of 0.1
#define HK_FLAGS 11               // 1:V 2:T 4:H: 8:E Spaeter noch evtl. rH/T
#define ANZ_KOEFF 8               // Koefficients for Measure, Minimum 4*2
#define MAX_CHANNELS 4            // Macht Sinn das mit ANZ_KOEFFS zu kombinieren
#define MESSEN_ENERGY 1000        // Annahme erstmal 1mA fuer 1000 msec

/* wie JesFs: _tb_novo is valid if _tb_novo[0]==RAM_MAGIC and _tb_novo[1] = ~_tb_novo[2];  
*  _tb_novo[1] holds the RTC, _tb_novo[3] the GUARD-Value
*  _tb_novo[4,5,6,7 frei] (4,5 for Energy-Management)
*/
#define RAM_MAGIC (0xBADC0FFEUL)
uint32_t _tb_novo[8] __attribute__((section(".non_init")));
#define _tb_bootcode _tb_novo[3]  // Bootcode for post-mortem analysis
uint32_t _tb_bootcode_backup;     // Holds initial Bootcode
#define hk_batperc_h _tb_novo[4]
#define hk_batperc_sum32 _tb_novo[5]
// Time erstmal auf ARDUINO noch nicht verwendet
void tb_init(void) {
  /* Minimum Required Basic Block ---START--- */
  // Init _tb_novo-nonvolatile RAM
  // Check RAM and init counter
  if (_tb_novo[0] != RAM_MAGIC || (_tb_novo[1] != ~_tb_novo[2])) {
    _tb_novo[0] = RAM_MAGIC;  // Init Non-Volatile Vars
    _tb_novo[1] = 0;          // Time, Times <(01.01.2020 are "unknown")
    _tb_novo[2] = ~0;         // ~Time
    // _tb_novo[3] === _tb_bootcode
    _tb_novo[4] = 0;                     // Reserviert fuer User Energycounter H (measure.c)
    _tb_novo[5] = 0;                     // Reserviert fuer User Energycounter L (measure.c)
    _tb_novo[6] = 0;                     // noch FREI
    _tb_novo[7] = 0;                     // noch FREI
  } else {                               // Valid Time
    _tb_bootcode_backup = _tb_bootcode;  // Save Bootcode
    // tb_time_set(_tb_novo[1]); // Ersmal keine RTC auf ARDUINO
  }
}

#define LED_PIN PA4  // Standard LED gg. VCC

#define MAX_RXANZ 51  // Anzahl maximal zu empfangender Bytes
typedef struct {
  struct {                               // Connection Sachen
    uint32_t last_server_reply_runtime;  // Wann zum letzten Mal was vom Server vernommen? Joind oder Reply
    bool dbg;                            // Manuell setzen fuer maximal Blabla
  } con;
  struct {           // Gesetzte Parameter
    uint8_t rxport;  // Msg vom Server
    uint8_t rxanz;   // Hier die Nachricht
    uint8_t rxbytes[MAX_RXANZ + 1];
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
extern bool param_dirty;  // True wenn dirty

/* Watchdog wird empfohlen, wenn aus: Raster max. 120 sec, sonst Raster max. 30*/
PARAM param = { _PMAGIC, "", 3600, 6, 1, /*WD*/ true, { 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0 } };
bool param_dirty = false;

// Periodic
uint32_t next_periodic_runtime;  // Messen wenn runtime >= next_periodic_runtime

// --- SEC_TIMER ---
uint32_t now_runtime = 0;  // Runtime in sec seit Reset
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

/* Kontext des Handlers ist "normal"
* Achtung: der Timer hat einen Rahmen von 9msec @ ca. 10mA msec und selbst, Schnitt also ca. 3->7uA
* bei Aufruf nur alle 10 sec steigt der Stromverbrauch auf Avg. 15uA!
* Timer wird fuer opt. WD und Periodic verwendet, da anscheinend nur 1 Timer sauber laeuft, bzw. das WD-Window evtl. ausloest
* Wenn keine WD ist Intervall max. 120 Sek. sonst 30 Sekunde wg. WD
* Wichtig: Watchdog alle <= 32 Sekunden feeden!*/
void apptimer_timer_handler(void *data) {
  if (wdt_in_use) jo_wdt_feed();  // Verifiziert: WD loest sauber 32 sec nach letztem feed aus
  update_runtime();
  if (now_runtime < next_periodic_runtime) {
    if (wdt_in_use) Serial.printf("[WDT %u]\n", now_runtime);
    else Serial.printf("[??? %u]\n", now_runtime);  // Wachen ohne Watchdog
#if (HK_FLAGS & 8)
    hk_add_energy(ENERGY_WDT);  // Energy fuer 1 Watchdog
#endif
  } else {
    Serial.printf("[PERIODIC %u]\n", now_runtime);
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
#define __fp16 uint16_t
__fp16 float_to_half(float f) {
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

typedef union {  // Umwandlung FLOAT->FLOAT16->Binaer
  uint16_t uval;
  __fp16 f16val;
} FX16VAL;

#define NO_ERROR 0
// Darstellung einer allgemeinen Fliesskommazahl mit Option zum Fehler:
typedef struct {
  uint16_t errno;  // 0: No ERROR
  float floatval;
} FE_ZAHL;

typedef struct {
  FE_ZAHL fe;  // Der Messwert
  // Optional Platz fuer Meta-Daten, z.B. Typ-spec Einheiten
} CHANNEL_VALUE;

extern CHANNEL_VALUE channel_value[MAX_CHANNELS];
extern uint16_t anz_values;  // Anzahl der Messwerte

typedef struct {
  uint8_t hk_dcnt;  // Bei 0: in jedem Fall HK Mitmessen
  bool hk_valid;    // True wenn Valid fuer diese Messung
} MTIMES;
extern MTIMES mtimes;  // Measure Intern

#define HK_MAXANZ 4
#define HK90_VBAT 0                // Bat in mV Index
#define HK91_TEMP 1                // Temp in 0.1 Grad
#define HK92_HUM 1                 // Feucht in 0.1 Proc falls vers
#define HK93_MAH 2                 // Energie
extern float hk_value[HK_MAXANZ];  // alt: int16!

CHANNEL_VALUE channel_value[MAX_CHANNELS];
uint16_t anz_values;        // Anzahl der Messwerte
MTIMES mtimes;              // Measure Intern
float hk_value[HK_MAXANZ];  // Neu: HL als Float

// Minimal HK
void measure_hk(void) {
#if HK_FLAGS & 1
  hk_value[HK90_VBAT] = 3.217654321;  // Simulierte mV
#endif
#if HK_FLAGS & 2

  hk_value[HK91_TEMP] = (float)25.123;  // Simulierte Temperatur
#endif
#if HK_FLAGS & 8
  uint32_t h;
  // Direktes Umrechnen im mAH mit Justieren
  h = (hk_batperc_sum32 & 0xFFC00000);  // 1.165mAH-Ueberlauf? (0-4194304).16
  if (h) {
    hk_batperc_sum32 -= h;
    hk_batperc_h += (h / 4194304);  // 22 Bits
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

// Typspezifisch messen -- Simulation--
int16_t measure(int16_t ival) {
  if (!mtimes.hk_dcnt || ival) {
    measure_hk();
    mtimes.hk_dcnt = param.hk_reload;
  } else {
    mtimes.hk_dcnt--;
    mtimes.hk_valid = 0;
  }

  // Simulierte Messung
  channel_value[0].fe.errno = 0;
  channel_value[0].fe.floatval = 1.12345;
  channel_value[1].fe.errno = 0;
  channel_value[1].fe.floatval = 2.23456;
  anz_values = 2;  // global

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
  api.system.lpmlvl.set(2);                      // Enable LowPowerMode
  api.system.lpm.set(1);                         // Enable Low Power
  return res;
}

// Kommando-Handler nimmt nur Strings, Ret: 0: OK. Ohne WS!
int16_t parse_ltx_cmd(char *pc) {
  int16_t res = 0;  // Assume OK
  if (!strcasecmp(pc, "initeu868")) {
    res = lora_setup(4);              // Band 4:868 MHz
  } else if (!strcasecmp(pc, "?")) {  // ?: Reine Info
    Serial.printf("DEVICE_TYPE: %u\n", DEVICE_TYPE);

    update_runtime();
    Serial.printf("Runtime: %d sec\n", now_runtime);
    if (mlora_info.con.last_server_reply_runtime) Serial.printf("LastServerReply:%d sec ago\n", now_runtime - mlora_info.con.last_server_reply_runtime);
    else Serial.printf("No Network!\n");
    if (param._pmagic != _PMAGIC) {
      Serial.printf("ERROR: Parameter invalid\n");
    } else {
      Serial.printf("Cmd: '%s'\n", param.messcmd);
      Serial.printf("Period: %u sec\n", param.period);
      Serial.printf("HK-Reload: %u\n", param.hk_reload);
      Serial.printf("Profile: %u\n", param.sensor_profile);
      Serial.printf("Use Watchdog: %u\n", param.use_watchdog);
      for (uint16_t i = 0; i < ANZ_KOEFF; i++) {
        Serial.printf("koeff[%u]: %f\n", i, param.koeff[i]);
      }
      if (param_dirty) Serial.printf("INFO: Parameter not saved!\n");
    }
  } else if (!strcasecmp(pc, "write")) {  // Write Parameter
    Serial.printf("Write\n");
    res = parameter_nvm_save();
  } else if (!strcasecmp(pc, "factoryreset")) {  // FactoryReset
    param._pmagic = 0;
    parameter_nvm_save();
    Serial.printf("Factory Reset...\n");
    delay(50);
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
        Serial.printf("\n");
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
      Serial.printf("H93:Energy: %.4f mAh\n", hk_value[HK93_MAH]);
#endif
    // Bissl Blabla
    if (param_dirty) Serial.printf("INFO: Parameter not saved!\n");
  } else if (str_cmatch("xxx", pc)) {  // DEBUG: xxx fuer internes
    Serial.printf("---xxx---\n");
    for (uint16_t i = 0; i < 8; i++) {
      Serial.printf("t[%u]: %x %d\n", i, _tb_novo[i], _tb_novo[i]);
    }
  } else res = -2001;  // Command Unknown
  return res;          // OK
}


// --- LTX-eigene CMDs ---
int ltx_cmd_handler(SERIAL_PORT _port, char *cmd, stParam *param) {
  int16_t res;
  if (param->argc != 1) res = -2009;
  else res = parse_ltx_cmd(param->argv[0]);
  if (res) {
    Serial.printf("ERROR: %d\n",res);
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}
/* Aufruf z.B. 'ATC+LTX' => argc:0,  'ATC+LTX=1:2' => argc: 2: '1','2' cmd enth 'ATC+LTX'
* Auch Aufrufe wie 'atc+ltx test=33' mgl. => arc: 1: 'test=33' Trenner NUR Doppelpunkt!
* Testhandler:   
*  Serial.printf("cmd:%s argc:%d ", cmd, param->argc);
*   for (int16_t i = 0; i < param->argc; i++) {
*     Serial.printf(" [%d]:'%s'", i, param->argv[i]);
*   }
*   Serial.printf("\n");
*/
bool init_ltx_ats(void) {
  bool bres = api.system.atMode.add((char *)"LTX",
                                    (char *)"LTX Commands",
                                    (char *)"LTX", ltx_cmd_handler,
                                    RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
  return !bres;
}

// Periodic
#define ENERGY_PERIODIC 100  // Minimum ist 100
void periodic(void) {
  /* Test */
  digitalWrite(LED_PIN, LOW);  // LED turn off when input pin value is HIGH
  Serial.printf("[PERIODIC Runtime:%u]\n", now_runtime);
  digitalWrite(LED_PIN, HIGH);  // LED turn on when input pin value is LOW

#if (HK_FLAGS & 8)
  hk_add_energy(ENERGY_PERIODIC);  // Energy fuer 1 Watchdog
#endif
}


// ---- Setup() -------------
void setup() {
  int16_t res = 0;

  tb_init();

  Serial.begin(Serial.getBaudrate());
  Serial.printf("*** " DEV_FAMILY " (C)JoEmbedded.de ***\n");
  Serial.printf("MAC:%08X%08X DEVICE_TYPE:%u V%u.%u\n", get_mac_h(), get_mac_l(), DEVICE_TYPE, DEVICE_FW_VERSION / 10, DEVICE_FW_VERSION % 10);
  Serial.printf("RUI3-Version %s\n\n", api.system.firmwareVersion.get().c_str());

  parameter_nvm_load();
  if (param.period < 10) param.period = 10;  // Min. 10 sec period
  next_periodic_runtime = 30;                // Fruehestens nach 30 Sekunden 1. Messung

  if (param.use_watchdog) {
    res = jo_wdt_init();
    Serial.printf("- Watchdog enabled\n");
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED turn on when input pin value is LOW

  if (!res) res = init_ltx_ats();

  // WDT_TIMER (#0) - Enable WDT Background Feed
  if (!res) {
    uint32_t int_sec = param.period;
    if (int_sec > 120) int_sec = 120;                                                                                              // Alle 2 Minuten Maximal
    if (wdt_in_use && (int_sec > WDT_TIMER_PERIOD_SEC)) int_sec = WDT_TIMER_PERIOD_SEC;                                            // Oder weniger wenn WD aktiv
    if (api.system.timer.create(RAK_TIMER_0, (RAK_TIMER_HANDLER)apptimer_timer_handler, RAK_TIMER_PERIODIC) != true) res = -1001;  // Timer0 Faile1
    else if (api.system.timer.start(RAK_TIMER_0, (int_sec * 1000), (void *)0) != true) res = -1002;                                // Timer0 Fail2
  }

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