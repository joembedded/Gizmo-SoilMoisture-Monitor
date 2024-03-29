# Gizmo - 'GiessMonitor' #
**Ein LowCost IoT Sensor zur Messung der Bodenfeuchte mit LoRa und/oder BluetoothLE**

Es gibt unzählige Ultra-LowCost-Sensoren zur Messung der Bodenfeuchte.
Die meisten davon taugen aber oft nicht einmal für Hobby-Zwecke.

Das Projekt Gizmo - 'GiessMonitor' möchte einen einfachen, aber trotzdem
für die Praxis geeigneten IoT Sensor entwickeln. Final soll ein LoRa-Modul
eingesetzt werden, so dass Gizmo auch über grosse Entfernungen und selbst mit
einfachen Batterien jahrelang arbeiten kann. 

_*Gizmo ist ein 100% Open Source Projekt!*_
_*Wenn alles klappt, könnte daraus ein spannender und preisgünstiger Sensor für die Allgemeinheit werden.*_
_*'Collaborteure' sind jederzeit herzlich willkommen!*_

---
**Gizmo_0V1 - Boilerplate und Vor-Prototyp**

Im ersten Schritt geht es aber erst einmal darum das Messverfahren festzulegen.

Es bietet sich an die recht hohe Dielektrizitätskonstante von Wasser auszunutzen. 
Diese ist bis ca. 1 GHz recht konstant (bei ca. 80), hat aber den Nachteil, dass z.B. Salze bei
niedereren Frequenzen (unter ca. 1-10 Mhz) das Signal stark verfälschen.
(Details dazu: [hier](https://de.wikipedia.org/wiki/Permittivit%C3%A4t). Die Permitivität setzt sich aus der 
Dielektrizitätskonstante (Realteil) und dem störenden Leitwert (Imaginärteil) zusammen).

(Technische Anmerkung: Die Menge des vorhandenen Wassers im Boden ist allerdings trotzdem auch immer nur ein indirektes Mass für den 'Giess-Bedarf',
denn die Art des Bodens (z.B. sandig oder eher lehmig) spielt eine fast noch größere Rolle. Aber da diese sich lokal normalerweise nicht ändert,
ist der Wassergehalt als Messwert absolut OK. Gizmo soll ja auch kein wissenschaftliches Messgerät, sondern nur ein (hoffentlich) zuverlässiger Giess-Monitor werden).

Es bietet sich also an, Frequenzen im Bereich > 10 Mhz und < 1Ghz zu verwenden. Als Sensorelement wird die Kapazität einer kleinen 
Kupferfläche (die später in einer korrosionsgeschützten Innenlage der PCB verschwinden wird) verwendet und ganz grob ca. 10pF - 100pF beträgt.
Je stärker das Anregungssignal ist, desto günstiger wird auch die Messung. Aber schnell kann daraus auch ein nicht CE-konformer Sender werden.
Daher sind auf dem Gizmo_0V1 für die ersten Tests mehrere verschiedene Mess- und Anregungs-Systeme vorgesehen.

In Europa gelten auf diesen Frequenzen etwas lockerere Vorschriften:
- 26.957 - 27.283 MHz: CB-Band
- 40.660 – 40.700 MHz: ISM-Band
- 169.400 – 169.475 MHz: ISM-Band

Die Geometrie der Sensorfläche muss für einen kapazitiven Sensor aber jedezeit unter der Lambda/4-Länge liegen, bei einer max. Dielektrizitätskonstante von 80 wären das bei 40 Mhz ca. 21 cm, bei 169 Mhz aber nur ca. 5 cm. Das 40 MHz Band (speziell die Bandmitte 40.680 MHz) erscheint daher als ein guter Ausgangspunkt, wo auch Quarze leicht erhältlich sind.

[Hier](./docu/sensorsim01.png) die mit FEMM auf 10 cm simulierte Geometrie. Später sollen die Leiter korrosionsgeschützt in einer Innenlage verschwinden. In der weiteren Simulation und ersten Tests ergab sich auch, dass ein Abstand der Sensorfläche zu GND von 2.2 mm und eine Länge von 10 cm noch absolut ausreichend für ein gutes Signal sind. (Details dazu folgen). 

![Gizmo_0V1](./docu/pgizmo01.png)

[Gizmo_0V1 - PCB, Layouts und Schaltpläne im Ordner 'docu']

---
**Gizmo - der Plan**
- Tests und Festlegung des Messsystems: Dazu PCB mit LowCost BLE Modul und Speicher (als Logger) layoutet.
- Erste Software (basierend auf Open-SDI12-Blue-Libs und JesFs)
- Finalisierung einer PCB V1.0 (Multilayer mit korrosiongeschützter Sensorfläche), LoRa-Modul und 3D-gedrucktem Batteriefach
- Anbindung an z.B. das Community LoRa-Netz TTN
- Kleine APP (basierend auf BLX.JS) für die praktische Nutzung
- ...

---
## Changelog  ##
- 01.04.2023 V0.1 Gizmo Prototyp als PCB in Fertigung gegeben
- 07.04.2023 Erste elektrische Tests der PCB V0.1
---

