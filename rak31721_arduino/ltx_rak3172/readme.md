# RAK3172 Arduino-Code
Dieses Repository enthält Arduino-Code für RAK3172-Module mit LoRa (Versionen **-E**, **-T**, **-LP-SIP** und **-SIP**). Je nach Konfiguration kann die LoRa-CPU entweder als eigenständige Anwendung oder als peripheres Modem für einen externen Host eingesetzt werden.

> **Aktuell verwendete Firmware:**  
> **RUI3** Version **V4.2.2**

**Hinweis:** Die zugehörige Platine befindet sich im parallelen Verzeichnis. Die Schaltplandaten liegen im KiCad-Format (Version 9) vor.

---

## Verfügbare ATC+LTX Kommandos

### Immer verfügbar

| Befehl                        | Beschreibung                                                                                  |
|-------------------------------|----------------------------------------------------------------------------------------------|
| `ATC+LTX=initeu868`           | Initialisiert das Modem auf EU868 und setzt Standard DEVEUI/APPEUI/APPKEY                    |
| `ATC+LTX=cred`                | Zeigt die Schlüssel ("Credentials") an                                                       |
| `ATC+LTX=?`                   | Info über System/Modem                                                                       |
| `ATC+LTX=stat`                | Statistiken über den letzten (ausgehenden) Frame und (optionale) Server-Antwort              |
| `ATC+LTX=reset`               | Setzt das Modem zurück (Energieverbrauch bleibt erhalten)                                    |
| `ATC+LTX=echo`                | Echo-Test für Kommunikation                                                                  |

### Nur für `USAGE_STANDALONE`

| Befehl                        | Beschreibung                                                                                  |
|-------------------------------|----------------------------------------------------------------------------------------------|
| `ATC+LTX=write`               | Speichert Setup/Koeffizienten nach Änderungen im NVM                                         |
| `ATC+LTX=factoryreset`        | Löscht alle Benutzerdaten                                                                    |
| `ATC+LTX=cmd=<command>`       | Setzt den Messbefehl                                                                         |
| `ATC+LTX=p=<seconds>`         | Setzt die Messperiode in Sekunden (Min. 10, Max. 86400)                                     |
| `ATC+LTX=hkr=<value>`         | Setzt HK-Zähler, nach HK werden hkr Daten ohne HK übertragen (spart Energie)                |
| `ATC+LTX=profile=<value>`     | 1–223 (Messwerte als F32), 1001–1223 (Messwerte als F16)                                    |
| `ATC+LTX=wd=<0\|1>`           | 0/1 für false/true: Watchdog verwenden                                                      |
| `ATC+LTX=k<index>=<value>`    | Setzt Koeffizienten für die Berechnung der Messwerte                                         |
| `ATC+LTX=k`                   | Zeigt alle Koeffizienten mit Beschreibung an                                                 |
| `ATC+LTX=e<value>`            | Führt eine Testmessung durch und zeigt sie an                                                |
| `ATC+LTX=i<value>`            | Testmessung mit Testübertragung, möglicherweise LoRaWAN Join                                 |

### Nur für `USAGE_EXT_AT`

| Befehl                        | Beschreibung                                                                                  |
|-------------------------------|----------------------------------------------------------------------------------------------|
| `ATC+LTX=send <port>`         | Sendet Frame 'smart' (Bestätigung nur wenn nötig bei CFM=0)                                  |
| `ATC+LTX=tput <hex_data>`     | Legt TX-Paket in den Puffer (`*`: hinzufügen, sonst bei 0 starten)                           |