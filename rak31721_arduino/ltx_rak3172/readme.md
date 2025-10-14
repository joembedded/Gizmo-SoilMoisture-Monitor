# 📡 RAK3172 Arduino-Code

Dieses Repository enthält Arduino-Code für **RAK3172-Module** mit LoRa (Versionen **-E**, **-T**, **-LP-SIP** und **-SIP**). 

Je nach Konfiguration kann die LoRa-CPU entweder als **eigenständige Anwendung** oder als **peripheres Modem** für einen externen Host eingesetzt werden.

## 🔧 Funktionsweise

Der Arduino-Code basiert auf der **RUI3** und ergänzt diese mit eigenen Erweiterungen (`ATC+LTX...`). Alle 'normalen' RUI3-Befehle können weiterhin verwendet werden.

> **Hinweis:** Die zugehörige Platine befindet sich im parallelen Verzeichnis. Die Schaltplandaten liegen im **KiCad-Format (Version 9)** vor.

---

## 📚 RUI3-Firmware (von RAK)

**Aktuell verwendet:** RUI3 Version **V4.2.2**

📖 [AT Command Manual](https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/at-command-manual/)  

> **Tipp für Tests:** Duty-Cycle deaktivieren mit `AT+DCS=0`

---

## 🚀 Verfügbare ATC+LTX-Kommandos

> **Syntax-Hinweis:** Das Zeichen direkt nach `ATC+LTX` kann `=` oder ` ` (Leerzeichen) sein. Lediglich `:` ist im Kommando nicht erlaubt.


### 🌍 Immer verfügbare Kommandos

Diese Kommandos stehen in allen Modi zur Verfügung:

**`ATC+LTX=initeu868`**  
→ Initialisiert das Modem auf EU868 und setzt Standard DEVEUI/APPEUI/APPKEY  
→ Verwendet die MAC-Adresse des LoRa-Modems

**`ATC+LTX=cred`**  
→ Zeigt die Schlüssel ("Credentials") an

**`ATC+LTX=?`**  
→ Zeigt ausführliche Informationen über System/Modem an

**`ATC+LTX=stat`**  
→ Statistiken über den letzten (ausgehenden) Frame und (optionale) Server-Antwort  
→ Beispiel-Ausgabe: `+F0 N0 R0 E0 L119 C0 S0`
- **F:** Transfer-Result (≥1: Busy, 0: OK, <0: Error)
- **N:** Joined-Flag
- **R:** Anzahl empfangener Bytes (RX-Frame)
- **E:** Frame-Energie (µC)
- **L:** Letzter Kontakt (Sekunden)
- **C:** Confirm-Flag
- **S:** TX-Frame-Counter

**`ATC+LTX=reset`**  
→ Setzt das Modem zurück (Energieverbrauch bleibt erhalten)  
→ Identisch zu `ATZ`

**`ATC+LTX=echo`**  
→ Echo-Test für Kommunikation  
→ Beispiel: `atc+ltx echoHalloWelt` → `Echo(9)'HalloWelt'`

**`ATC+LTX=dbg`**  
→ Aktiviert Debug-Ausgabe für 1800 Sekunden

**`ATC+LTX=ndbg`**  
→ Deaktiviert Debug-Ausgabe

---

### 📊 Kommandos für `USAGE_STANDALONE`

Diese Kommandos sind verfügbar, wenn das Modul als eigenständiger LoRa-Knoten betrieben wird:

**`ATC+LTX=write`**  
→ Speichert Setup/Koeffizienten nach Änderungen im NVM

**`ATC+LTX=factoryreset`**  
→ Löscht alle Benutzerdaten und setzt auf Werkseinstellungen zurück

**`ATC+LTX=cmd=<command>`**  
→ Setzt den Messbefehl

**`ATC+LTX=p=<seconds>`**  
→ Setzt die Messperiode in Sekunden  
→ Minimum: 10 Sekunden, Maximum: 86400 Sekunden (24 Stunden)

**`ATC+LTX=hkr=<value>`**  
→ Setzt HK-Zähler  
→ Nach HK werden `hkr` Daten ohne HK übertragen (spart Energie)

**`ATC+LTX=profile=<value>`**  
→ Setzt das Sensor-Profil
- **1–223:** Messwerte als F32 (32-Bit Float)
- **1001–1223:** Messwerte als F16 (16-Bit Float)

**`ATC+LTX=wd=<0|1>`**  
→ Watchdog-Verwendung einstellen  
→ `0` = deaktiviert, `1` = aktiviert

**`ATC+LTX=k<index>=<value>`**  
→ Setzt Koeffizienten für die Berechnung der Messwerte  
→ Beispiel: `ATC+LTX=k0=1.23`

**`ATC+LTX=k`**  
→ Zeigt alle Koeffizienten mit Beschreibung an

**`ATC+LTX=e<value>`**  
→ Führt eine Testmessung durch und zeigt die Ergebnisse an

**`ATC+LTX=i<value>`**  
→ Testmessung mit Testübertragung  
→ Falls erforderlich, wird ein LoRaWAN Join durchgeführt


---

### 🔌 Kommandos für `USAGE_EXT_AT`

Diese Kommandos sind verfügbar, wenn das Modul als externes LoRa-Modem für einen Host verwendet wird:

Die Verwaltung von Netzwerk-Joins, gelegentlichen Server-Acknowledges als "Lebenszeichen" oder eingehenden Kommandos vom Server wird durch diese Befehle stark vereinfacht. 

> **Vorteil:** Der ganze LoRa-Verkehr läuft asynchron im Hintergrund ab – der Host muss sich um nichts kümmern!

**`ATC+LTX=send <port>`**  
→ Sendet Frame 'smart' (Bestätigung wird nur bei Bedarf vom Server angefordert, wenn `CFM=0`)  
→ `<port>`: LoRaWAN Port-Nummer (1-223)

**`ATC+LTX=tput <hex_data>`**  
→ Legt TX-Paket in den Puffer
- **Ohne `*`:** Startet neuen Puffer bei Position 0
- **Mit `*`:** Fügt Daten an bestehenden Puffer an

**Beispiele:**

```bash
# Erste 16 Bytes in den Puffer legen
atc+ltx=tput 00112233445566778899aabbccddeeff

# Weitere 16 Bytes anhängen
atc+ltx=tput *00112233445566778899aabbccddeeff

# Ein einzelnes Byte
atc+ltx=tput 31

# Maximum im EU868-Band (51 Bytes)
atc+ltx tput 303132333435363738396162636465666768696A303132333435363738396162636465666768696A3031323334353637383945
```

Mit `ATC+LTX=stat` kann das Ergebnis der Übertragung abgefragt werden.

---

## 🛠️ Helpers

**51 Zeichen in HEX (Maximum für EU868):**
```
ASCII:  0123456789abcdefghij0123456789abcdefghij0123456789E
HEX:    303132333435363738396162636465666768696A303132333435363738396162636465666768696A3031323334353637383945
```

---

## 📝 Lizenz & Copyright

© JoEmbedded.de

---
