# Netzgekoppelter Wechselrichter für ein Modellhaus

**Erstellt von:** Gaby Fotio
**Hochschule:** Technische Hochschule Mittelhessen (THM), Campus Gießen, Fachbereich MNI
**Rahmen:** Interne Projektphase
**Betreuung:** Prof. Dr. Florian von Zabiensky

Ein STM32F407-Mikrocontroller und die H-Brücke BTS7960 erzeugen aus 24 V Gleichspannung eine sinusförmige Wechselspannung von 12 V eff. bei 50 Hz. Damit wird eine 12-V-Lampe in einem Modellhaus versorgt. Der Mikrocontroller misst Strom, Spannung und Frequenz am Ausgang selbst.

## Ergebnisse

| Größe | Referenzmessung | STM32 | Abweichung |
| --- | --- | --- | --- |
| Ausgangsspannung (eff.) | 11,801 V (Oszilloskop) | 11,91 V | +0,9 % |
| Frequenz | 50,072 Hz (Oszilloskop) | 50,26 Hz | +0,4 % |
| Lampenstrom (eff.) | 0,100 A (Nennwert) | 0,1005 A | +0,5 % |

Berechnet waren 11,88 V eff. (24 V · 0,70 / √2). Die Oszilloskop-Messung zeigt einen sauberen 50-Hz-Sinus (`Dokumentation/WaveForms_Ausgangsspannung_50Hz.png`).

## Hardware

| Bauteil | Typ | Aufgabe |
| --- | --- | --- |
| Mikrocontroller | STM32F4DISCOVERY (STM32F407VGT6, 168 MHz, VDDA = 3,0 V) | SPWM-Erzeugung, Messung |
| H-Brücke | BTS7960 (Modul IBT-2) | Schaltet 24 V mit 10 kHz |
| Drossel | Ringkerndrossel 1 mH, 5 A | LC-Ausgangsfilter |
| Kondensator | 7 × 1 µF parallel (geplant: 10 µF MKP) | LC-Ausgangsfilter, Grenzfrequenz 1,9 kHz |
| Last | Lampe 12 V / 1,2 W, E10-Fassung | Verbraucher im Modellhaus |
| Stromsensor | ACS712-20A | Lampenstrom |
| Spannungssensor | ZMPT101B | Lampenspannung |
| Versorgung | Labornetzteil 24 V mit Strombegrenzung | Eingangsspannung |

### Pinbelegung

| STM32-Pin | Funktion | Anschluss |
| --- | --- | --- |
| PA8 | TIM1_CH1 | BTS7960 RPWM |
| PB13 | TIM1_CH1N | BTS7960 LPWM |
| PC0 | GPIO-Ausgang | BTS7960 R_EN |
| PC1 | GPIO-Ausgang | BTS7960 L_EN |
| PA0 | ADC1_IN0 | ACS712 OUT (empfohlen: über 1 kΩ) |
| PA1 | ADC1_IN1 | ZMPT101B OUT (empfohlen: über 1 kΩ) |

Leistungspfad: Netzteil + → BTS7960 B+ · BTS7960 M+ → Drossel → Klemme A → ACS712 → Lampe → Klemme B → BTS7960 M−. Der Kondensator liegt zwischen Klemme A und B. Alle Massen laufen an einem Punkt zusammen.

## Software

Der gesamte eigene Code steht in `Core/Src/main.c` innerhalb der `USER CODE`-Bereiche von STM32CubeMX.

- **Sinus-PWM:** TIM1 läuft mit 168 MHz bis ARR = 16 799 (10 kHz). Eine Tabelle mit 200 Tastverhältnissen (Modulationsgrad 0,70) wird per DMA (DMA2 Stream 5, TIM1_UP, zirkulär) bei jedem Timer-Update nach CCR1 kopiert. Ergebnis: 50 Hz ohne CPU-Last. CH1 und CH1N arbeiten komplementär mit 119 ns Totzeit (bipolare SPWM).
- **Strommessung:** Effektivwert über 100 ms, Nullpunkt beim Start gemessen, Sensorrauschen herausgerechnet.
- **Spannungsmessung:** digitaler Tiefpass 200 Hz (2 × 1. Ordnung, Filterfaktor aus der echten Abtastzeit), Effektivwert des Wechselanteils, kalibriert mit dem Oszilloskop (11,801 V ↔ 87,62 ADC-Stufen).
- **Frequenzmessung:** steigende Nulldurchgänge mit Hysterese (±30 ADC-Stufen) und 10 ms Sperrzeit, Zeitstempel über den Zykluszähler DWT (168 MHz).
- **Kurvenanzeige:** alle 500 µs wird die Momentanspannung in `scopeMillivolts` geschrieben und kann im Debugger mit dem SWV Data Trace Timeline Graph angezeigt werden.
- **Fehlerbehandlung:** `Error_Handler()` schaltet die H-Brücke ab und hält an.

### Variablen im Debugger (Live Expressions)

| Variable | Bedeutung |
| --- | --- |
| `currentAmps` | Lampenstrom eff. in A |
| `voltageVolts` | Lampenspannung eff. in V |
| `frequencyHz` | Ausgangsfrequenz in Hz |
| `scopeMillivolts` | Momentanspannung für den SWV-Graphen |
| `voltageRmsRaw`, `voltageFundRmsRaw`, `voltageOffsetRaw`, `adcPairsPerSecond` | Diagnose |

## Bauen und Flashen

1. Projekt in **STM32CubeIDE 1.18** importieren (File → Import → Existing Projects into Workspace).
2. Die HAL-Treiber werden aus dem lokalen STM32Cube-Repository eingebunden (**STM32Cube FW_F4 V1.28.3**). Fehlen sie, die `.ioc`-Datei in STM32CubeMX öffnen und den Code neu generieren; der eigene Code in den `USER CODE`-Bereichen bleibt dabei erhalten.
3. Bauen und mit dem ST-LINK des Discovery-Boards flashen.
4. Inbetriebnahme: zuerst nur USB (Sensor-Ruhewerte prüfen), dann 24 V mit Strombegrenzung am Netzteil.

**Sicherheit:** Am Brückenausgang nie die Masseklemme eines geerdeten Oszilloskops anschließen. Die Ausgangsspannung mit zwei Kanälen gegen Masse messen und die Differenz bilden.

## Dokumentation

Im Ordner `Dokumentation/`:

- `Wechselrichter_Modellhaus_Dokumentation.pdf` / `.docx`: ausführliche Projektdokumentation
- `Wechselrichter_Modellhaus_Praesentation_THM.pptx`: Präsentation der Projektphase mit Sprechertext
- `WaveForms_Ausgangsspannung_50Hz.png`: Oszilloskop-Messung der Ausgangsspannung

## Ausblick

Das Projekt wird in einer Bachelorarbeit fortgeführt: *Vergleich von Strommessverfahren in einem STM32-basierten H-Brücken-Wechselrichter für ein Modellhaus: Hall-Sensor (ACS712) gegenüber integrierter Strommessung (IS) des BTS7960 im Insel- und Einspeisebetrieb.*

---

© 2026 Gaby Fotio
