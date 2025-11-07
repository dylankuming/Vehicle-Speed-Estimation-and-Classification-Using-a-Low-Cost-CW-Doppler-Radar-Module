# Phase 2 Teensyduino Code (Teensy 4.1 + SGTL5000)

This folder contains the **Teensy/Arduino firmware** used in **Phase 2** of the MSc dissertation (Chapter 4) to acquire baseband Doppler data directly from the 24 GHz CW radar module.  
The program performs **real-time FFT logging** of the radar’s signal to the Teensy 4.1’s onboard SD card for subsequent MATLAB analysis.

---

## Overview

The firmware continuously records short-time power spectra by computing **4096-point, 50 %-overlap FFTs** using the *OpenAudio F32* library.  
Each frame is saved as raw 32-bit floating-point values to a `.BIN` file on the SD card.

**Main features**

1. Real-time 4096-point complex FFT using the **OpenAudio F32** library  
2. Continuous data streaming to the SD card (onboard slot of Teensy 4.1)  
3. Automatic filename generation with run-counter using EEPROM(`REC0001.BIN`, `REC0002.BIN`, …)  
4. Adjustable recording duration (`RECORD_MS`)  
---

## Requirements

* **Arduino IDE v2.0.4 or later**  
* **TeensyDuino 1.59 or later**  
* **[OpenAudio_ArduinoLibrary](https://github.com/chipaudette/OpenAudio_ArduinoLibrary/tree/master?tab=readme-ov-file#openaudio-library-for-teensy)** by Chip Audette

---

## How It Works

1. On startup, a persistent **run counter** is read from EEPROM and incremented.  
2. A unique filename is created (e.g., `REC0007.BIN`).  
3. The SGTL5000 audio input is configured (`AUDIO_INPUT_LINEIN`).  
4. FFT processing runs continuously; each frame (2048 bins × 4 bytes) is streamed to the SD card.  
5. Recording stops automatically after the specified duration (`RECORD_MS`).  
6. The resulting `.BIN` file can be transferred to a PC and processed in MATLAB using the **[Phase 2 MATLAB code](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/tree/main/Phase%202%20MATLAB%20code)** .

---

## Key Parameters (Editable)

| Parameter          | Default         | Description                          |
|--------------------|-----------------|--------------------------------------|
| `RECORD_MS`        | 1800 ms (30 min)       | Recording duration                   |
| `FFT size`         | 4096            | Number of FFT points                 |
| `windowFunction`   | Hanning         | FFT window                           |
| `setOutputType()`  | `FFT_POWER`     | Output mode (`POWER`, `RMS`, `DBFS`) |
| `lineInLevel()`    | 15              | ADC full-scale input (0 – 15)                  |
| `flushCounter`     | Every 64 frames (~3 sec) | SD flush interval                    |

---
