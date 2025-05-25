# RAK3272_modul_lora_lp_test.ino - DEUTSCH

Ein einfacher, ultra-niedrigstrom LoRa-Knoten-Sketch für das RAK3172-Modul unter Verwendung der RUI3-Bibliothek.  
**Autor:** joembedded@gmail.com  
**Repository:** [Gizmo-SoilMoisture-Monitor](https://github.com/joembedded/Gizmo-SoilMoisture-Monitor)  
**Hardware:** RAK3172-E/T/L/SIP Modul  
**Bibliothek:** RUI3 Arduino API

---

## Zweck

Dieser Sketch implementiert einen vollständigen LoRaWAN (LTX) Knoten, optimiert für ultra-niedrigen Stromverbrauch. Er ist für die Bodenfeuchteüberwachung und ähnliche Sensormessungen konzipiert, kann aber für andere Low-Power-LoRa-Anwendungen angepasst werden. Der Knoten kann einem LoRaWAN-Netzwerk beitreten, Sensordaten messen, periodische Übertragungen durchführen, Energiewerte verwalten und den Hardware-Watchdog nutzen.

---

## Hauptfunktionen

- **LoRaWAN-Konnektivität:** Beitritt, Uplink und Downlink über die RUI3-APIs.
- **Ultra-Niedrigstrom:** Nutzt Low-Power-Modi und Hardware-Watchdog für Zuverlässigkeit und minimalen Batterieverbrauch.
- **Sensorik und Housekeeping:** Simuliert und verwaltet Sensordaten, Batteriespannung, Temperatur und Energiezähler.
- **Nichtflüchtiger Speicher:** Parameter und Energiezähler werden in Flash/NVRAM gespeichert und überstehen einen Reset.
- **Kommando-Handler:** Konfiguration und Steuerung über serielle Schnittstelle oder LoRa-Downlink.
- **Flexible Nutzlast:** Sensor- und Housekeeping-Daten werden effizient für LoRaWAN-Payloads gepackt (unterstützt float16/float32).
- **Erweiterbar:** Modulare Struktur erlaubt Anpassungen für andere Sensortypen oder Kommunikationswege.

---

## Hauptbestandteile

### 1. Globale Konstanten & Konfiguration

- `DEVICE_TYPE`, `DEV_FAMILY`, `DEVICE_FW_VERSION` — Gerätekennungen.
- `HK_FLAGS` — Steuert, welche Housekeeping-Werte übertragen werden (Spannung, Temperatur, usw.).
- `ANZ_KOEFF`, `MAX_CHANNELS` — Anzahl Kalibrierkoeffizienten und unterstützte Kanäle.
- Energieverbrauchsparameter für Messungen und LoRa-Operationen.

### 2. Nichtflüchtiger Speicher

- Verwendet Magic Numbers und reservierte RAM-Bereiche zur Speicherung von Zählern über Resets hinweg.
- Funktionen wie `tb_init()` und Makros für Batterie-Zähler.

### 3. LoRaWAN-Stack

- Alle LoRaWAN-Zugangsdaten und Konfigurationen werden über die RUI3-APIs gehandhabt.
- Unterstützt OTAA-Join, Credential-Setup, Adaptive Data Rate, bestätigte Uplinks, sowie Join-, Send- und Receive-Callbacks.

### 4. Watchdog-Implementierung

- Nutzt den STM32 Hardware-Watchdog (`uhal_wdt.h`) zur Erhöhung der Zuverlässigkeit.
- Kann über Parameter und Kommandos aktiviert oder deaktiviert werden.

### 5. Messlogik

- Simuliert Sensordaten

.....
<hr>

# RAK3272_modul_lora_lp_test.ino  - ENGLISH

A simple, ultra-low-power LoRa node sketch for the RAK3172 module, using the RUI3 library.  
**Author:** joembedded@gmail.com  
**Repository:** [Gizmo-SoilMoisture-Monitor](https://github.com/joembedded/Gizmo-SoilMoisture-Monitor)  
**Hardware:** Any RAK3172-E/T/L/SIP module , but L preferred (EU)


---

## Purpose

This sketch implements a complete LoRaWAN (LTX) node, optimized for ultra-low-power operation. It is designed for soil moisture monitoring and related sensor tasks, but can be adapted for other low-power LoRa applications. The node can join a LoRaWAN network, measure sensor values, handle periodic transmissions, manage energy counters, and make use of hardware watchdog timers.

---

## Key Features

- **LoRaWAN Connectivity**: Handles join, uplink, and downlink using RUI3 APIs.
- **Ultra-Low-Power**: Uses low-power modes and hardware watchdog for reliability and minimal battery consumption.
- **Sensor and Housekeeping**: Simulates and manages sensor values, battery voltage, temperature, and energy counters.
- **Non-Volatile Storage**: Maintains parameters and energy counters in flash/NVRAM, surviving resets.
- **Command Handler**: Accepts commands via serial or LoRa downlink to configure operation or trigger actions.
- **Flexible Payload**: Packs sensor and housekeeping data efficiently for LoRaWAN payloads (supports float16/float32).
- **Extensible**: Modular structure allows for easy adaptation to other sensor types or communication needs.

---

## Main Components

### 1. Global Constants & Configuration

- `DEVICE_TYPE`, `DEV_FAMILY`, `DEVICE_FW_VERSION` — Device identifiers.
- `HK_FLAGS` — Controls which housekeeping values are included (voltage, temperature, etc.).
- `ANZ_KOEFF`, `MAX_CHANNELS` — Number of calibration coefficients and supported channels.
- Energy consumption parameters for measurements and LoRa operations.

### 2. Non-Volatile Memory

- Uses magic numbers and reserved RAM sections to retain counters across resets.
- Functions `tb_init()`, and macros for battery counters.

### 3. LoRaWAN Stack

- All LoRaWAN credentials and configuration handled via RUI3 APIs.
- Supports OTAA join, credentials setup, adaptive data rate, confirmed uplinks, and callbacks for join, send, and receive.

### 4. Watchdog Implementation

- Uses STM32 hardware watchdog (`uhal_wdt.h`) to ensure reliability.
- Can be enabled/disabled via parameters and commands.

### 5. Measurement Logic

- Simulates sensor readings for demonstration (soil moisture, battery voltage, etc.).
- Periodic measurement and transfer using a timer callback.
- Housekeeping functions update battery, temperature, energy, etc.

### 6. Payload Construction

- Supports efficient packing of sensor and housekeeping data.
- Dynamic selection of float precision (float16/float32) for compact payloads.

### 7. Command Interface

- Serial and LoRa downlink commands for configuration and control:
  - `initeu868` — Setup for EU868 band.
  - `cred` — Show credentials.
  - `?` — Show device and parameter info.
  - `write` — Save parameters to flash.
  - `factoryreset`, `reset` — Restore defaults or reboot.
  - `cmd=`, `p=`, `hk=`, `profile=`, `wd=`, `k<idx>=` — Set measurement logic, period, housekeeping, profile, watchdog, coefficients.
  - `e`, `i` — Trigger measurement and manual transfer.

### 8. Setup and Main Loop

- `setup()` initializes parameters, hardware, watchdog, timers, and LoRaWAN stack.
- `loop()` disables scheduler tasks (idle).

---

## Usage

1. **Board Setup**: Add the relevant RAK3172 board in Arduino IDE via Board Manager.
2. **Flashing**: Ensure firmware is up-to-date before uploading a new version.
3. **Configuration**:
   - Use the serial interface to send commands for setup and debugging.
   - Use LoRaWAN downlink to remotely update parameters.
4. **Operation**:
   - The node joins the LoRaWAN network and sends periodic sensor data.
   - Watchdog ensures the node recovers from hangups.
   - Energy counters help estimate battery usage.

---

## References

- RUI3 Arduino API: https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/arduino-api
- Watchdog: https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/watchdog/
- Example: https://github.com/RAKWireless/RUI3-Best-Practice/blob/main/RUI3-LowPower-Example/RUI3-LowPower-Example.ino

---

## Notes

- The sketch contains many features for advanced monitoring and debugging; adapt and streamline as needed for production.
- Housekeeping and sensor reading functions should be connected to real sensors for full deployment.
- Review and adjust energy calculation constants for your specific hardware.

---
