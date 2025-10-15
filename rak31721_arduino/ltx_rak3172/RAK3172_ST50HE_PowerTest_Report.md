
# RAK3172LP-/ST50HE-Power-Test – 12.10.2025

## Ziel
Test der Sendeleistung mit Fixpaketen. Am TX-Ausgang ist ein **Hameg HMS3000** angeschlossen.

---

# Firmware RAK (V4.2.1)
Pfad: `C:\dokus\Lora\rak\rak3172\firmware`  
Aufspielen über **J-Link** (Setup, HEX).

> Der **RAK3172LP** verwendet andere AT-Kommandos als der **ST50HE**, obwohl beide denselben Chip besitzen.  
> Da der ST50HE jedoch **kein CW-Kommando** unterstützt, ist die Messung mit CW (Continuous Wave) wesentlich einfacher.

---

## UART-Einstellungen
Baudrate: **115200 Bd**

```
ATZ
ATE (optional)
AT+BAND=4
AT+LPMLVL=2
AT+LPM=1
```

---

## Testpakete (CW-Mode)

| Frequenz (Hz) | TX Power (dBm) | Zeit (s) |
|----------------|----------------|----------|
| 868500000 | 5–15 | 5 |

Beispiele:
```
AT+CW=868500000:5:5
AT+CW=868500000:6:5
...
AT+CW=868500000:15:5
```

---

## Messergebnisse (RAK3172LP)

| dBm (Set) | Hameg (dBm) | Strom (mA) |
|------------|--------------|-------------|
| 5 | 4.43 | 19 |
| 8 | 7.30 | 23 |
| 10 | 9.19 | 25 |
| 12 | 10.74 | 28 |
| 14 | 12.29 | 31 |
| 15 | 12.84 | 37 *(leicht über Soll)* |

---

## Alternativ: LoRa-Test mit TCONF/TTX

> **TCONF-Parameter:**  
> FRQ, Power, Bandwidth (125 kHz), SF=12, CR=4/5, Mod=LoRa, PayloadLen=50 B, FreqDev=25 kHz

**Beispiele:**
```
5 dBm:
AT+TCONF=868500000:5:0:12:1:0:0:1:50:25000:0:0
14 dBm:
AT+TCONF=868500000:14:0:12:1:0:0:1:50:25000:0:0
AT+TTX=4  → sendet 4 Pakete
```

**Ergebnisse:**

| Power | Hameg (dBm) | Strom (mA) |
|--------|--------------|-------------|
| 5 dBm | 4.46 | 19 |
| 14 dBm | 12.35 | 31 |

---

# Vergleich: ST50HE

## Firmware
Aufspielen per **J-Link**:  
`C:\dokus\Lora\acip_st50hx\ST50HE_LoRaWAN_ATSlave_v1.2.7`

---

## Grundkonfiguration
```
AT+BAND=?      → 5: EU868
AT+SAVE        → Speichert Setup dauerhaft
```

---

## TCONF (Beispielparameter)

| Nr | Parameter | Wert |
|----|------------|------|
| 1 | Frequency | 868500000 Hz |
| 2 | Power | 14 dBm |
| 3 | Bandwidth | 125 kHz |
| 4 | SF | 12 |
| 5 | CR | 4/5 |
| 6 | LNA | 0 |
| 7 | PA Boost | 0 |
| 8 | Modulation | LoRa |
| 12 | Preamble | 12 |
| 13 | CRC | 1 |
| 14 | IQ Inversion | 0 |

**Set-Kommandos:**
```
5 dBm:
AT+TCONF=868500000:5:125000:12:4/5:0:0:1:25000:2:3:12:1:0
14 dBm:
AT+TCONF=868500000:14:125000:12:4/5:0:0:1:25000:2:3:12:1:0
```

**Senden:**
```
AT+TTLRA=4:0:A55AA55AA55A... (50 Byte Daten)
```

---

## Messergebnisse (ST50HE)

| Power | Hameg (dBm) | Strom (mA) |
|--------|--------------|-------------|
| 5 dBm | 4.45 | 16 |
| 14 dBm | 12.37 | 31 |

---

## Fazit
Beide Module – **RAK3172LP-SIP** und **ST50HE** – zeigen bei identischen Bedingungen sehr ähnliche Sendeleistungen und Stromaufnahmen.  
Der **ST50HE** benötigt geringfügig weniger Strom bei gleicher Ausgangsleistung.

> **Ergebnis:** Elektrisch nahezu identisch, Funktionsverhalten konsistent.
