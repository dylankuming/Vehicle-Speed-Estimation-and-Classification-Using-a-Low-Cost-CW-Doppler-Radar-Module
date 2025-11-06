# CW Doppler Radar: Embedded Data Logger (Teensy 4.1 + SGTL5000)

This folder contains the **Teensy/Arduino firmware** used in **Phase 2** of the MSc dissertation to acquire baseband Doppler data directly from the 24 GHz CW radar module.
The program performs **real-time FFT logging** of the radar’s I/Q signal to the Teensy 4.1’s onboard SD card for later MATLAB analysis.

---

## Overview

The firmware continuously records short-time power spectra by computing **4096-point, 50 % overlap FFTs** using the *OpenAudio F32* library.
Each frame is saved as raw 32-bit floating-point values to a `.BIN` file on the SD card.

**Main features:**

1. Real-time 4096-point complex FFT using the **OpenAudio F32** library
2. Continuous data streaming to SD (onboard slot of Teensy 4.1)
3. Unique auto-incremented filename for every power-spectrogram capture
4. Adjustable recording duration (`RECORD_MS`)
5. EEPROM-stored run counter for reproducible logging sessions

---

## Hardware Setup

| Component                 | Description / Purpose                                |
| ------------------------- | ---------------------------------------------------- |
| **Teensy 4.1**            | 600 MHz ARM Cortex-M7 MCU with built-in SD slot      |
| **SGTL5000 Audio Shield** | Analogue front-end providing line-in interface       |
| **InnoSenT IPM-365**      | 24 GHz CW Doppler radar module (baseband I/Q output) |
| **Radar Amplifier**       | AD620 or LM358 instrumentation amplifier             |
| **SD Card**               | FAT32-formatted (Class 10 or higher recommended)     |
| **Power Supply**          | 5 V USB or regulated source                          |

Connections:

* Radar I output → Audio Line-In L
* Radar Q output (if unused) → Audio Line-In R (left unconnected in this firmware)
* Audio Shield mounted on Teensy 4.1

---

## Software Requirements

* **Arduino IDE 1.8+** or **Teensy IDE**
* **TeensyDuino 1.59+**
* **OpenAudio_ArduinoLibrary** by Chip Audette
* **AudioStream_F32** (included with OpenAudio)
* **SD** and **EEPROM** libraries (built-in)

---

## How It Works

1. On startup, a persistent **run counter** is read from EEPROM and incremented.
2. A unique filename is created, e.g. `REC0007.BIN`.
3. The SGTL5000 audio input is configured (`AUDIO_INPUT_LINEIN`).
4. FFT processing runs continuously; each frame (2048 bins × 4 bytes) is streamed to the SD card.
5. Recording stops automatically after the specified duration (`RECORD_MS`).
6. The resulting `.BIN` file can be transferred to a PC and processed in MATLAB using the Phase 2 pipeline.

---

## Example Configuration

```cpp
const unsigned long RECORD_MS = 20UL * 1000;  // 20 s recording duration
#define SDCARD_CS_PIN BUILTIN_SDCARD          // Teensy 4.1 onboard SD
AudioAnalyzeFFT4096_IQ_F32 FFT4096iq1;        // 4096-point FFT
```

To extend recording time, simply increase `RECORD_MS` or loop the process in code.

---

## Output File Format

* **Filename:** `RECXXXX.BIN` (auto-incremented)
* **Content:** 2048 float values per FFT frame (power spectrum)
* **Data type:** IEEE 754 float (4 bytes per value)
* **Order:** sequential frames, no header

Each file can be read in MATLAB or Python with:

```matlab
fid = fopen('REC0007.BIN', 'rb');
data = fread(fid, 'single');
fclose(fid);
```

---

## Key Parameters (Editable)

| Parameter         | Default         | Description                      |
| ----------------- | --------------- | -------------------------------- |
| `RECORD_MS`       | 20000 ms        | Recording duration               |
| `FFT size`        | 4096            | Number of FFT points             |
| `windowFunction`  | Hanning         | FFT window                       |
| `setOutputType()` | `FFT_POWER`     | Output mode (POWER / RMS / DBFS) |
| `lineInLevel()`   | 15              | Input gain (0–31)                |
| `volume()`        | 0.5             | SGTL5000 volume                  |
| `flushCounter`    | every 64 frames | SD flush interval                |

---

## Example Serial Output

```
Starting run #7 → opening file REC0007.BIN
SD init… OK
→ setup complete, recording starts now
Flushed 64 frames to SD.
Flushed 128 frames to SD.
Time elapsed—recording stopped, file closed.
```

---

## Notes

* Use a **Class 10** or faster SD card for sustained writes.
* Do not power down during a recording; always wait for the “file closed” message.
* Each `.BIN` file can later be converted or analysed in MATLAB.
* FFT4096IQ_F32 computes 2048 bins per frame; half the FFT is stored since the signal is real.

---

This firmware forms the **embedded data-acquisition** component of the low-cost CW Doppler radar system described in Chapter 4 of the MSc dissertation.

