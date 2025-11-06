# Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar

This repository contains all code and resources developed for the MSc dissertation
**“Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar System”**
by **Dylan Kuming**, University of Cape Town (2025).

The project demonstrates a complete **low-cost radar-based vehicle monitoring system**, combining real-time data acquisition, signal processing, multi-target tracking, and vehicle classification — all implemented using accessible hardware and MATLAB toolchains.

---

## Repository Overview

The repository is organized into three main components:

| Folder                          | Description                                                                                                                                     |
| --------------------------------| ----------------------------------------------------------------------------------------------------------------------------------------------- |
| **[Phase 1 MATLAB code](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/tree/main/Phase%201%20MATLAB%20code)**  | Offline MATLAB processing and validation (Chapter 3) — performs detection, clustering, tracking, and classification on WAV recordings.          |
| **[Phase 2 MATLAB code](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/tree/main/Phase%202%20MATLAB%20code)**  | Embedded-system validation (Chapter 4) — applies the same processing pipeline to data acquired directly from the Teensy 4.1 logger.             |
| **[Phase 2 Teensyduino code](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/tree/main/Phase%202%20Teensyduino%20code)** | C++ firmware for the Teensy 4.1 + SGTL5000 audio shield — captures Doppler radar signals, performs real-time FFTs, and logs results to SD card. |

Each folder contains its own README detailing setup, requirements, parameters, and usage.

---

## System Overview

### 1. Hardware

| Component                                 | Function                                                                    |
| ------------------------------------------| --------------------------------------------------------------------------- |
| **InnoSenT IPM-365 24 GHz CW Radar**      | Provides the baseband Doppler signal proportional to target radial velocity |
| **AD620 / LM358 Amplifier**               | Boosts baseband signal for ADC input                                        |
| **Teensy 4.1 + SGTL5000 Audio Shield**    | Handles real-time digitisation and spectrogram logging                      |
| **PC / MATLAB**                           | Performs signal processing, detection, tracking, and classification         |

---

### 2. Software Pipeline

1. **Signal Acquisition** – Raw baseband I/Q data recorded via Teensy 4.1
2. **Spectrogram Generation** – Short-Time Fourier Transform (STFT)
3. **Target Detection** – Ordered-Statistic CFAR (OS-CFAR)
4. **Clustering** – 1-D DBSCAN to group detections per frame
5. **Tracking** – JPDA + CA Kalman filter for multi-target tracking
6. **Track Merging** – Overlap and same-velocity gap merging
7. **Classification** – Rule-based using normalised Doppler ramp width

---

## Requirements

* **MATLAB R2022b** or later

  * Signal Processing Toolbox
  * Statistics and Machine Learning Toolbox
  * Sensor Fusion and Tracking Toolbox
  * Image Processing Toolbox
* **Arduino IDE v2.0.4+**
* **TeensyDuino 1.59.0+**
* **[OpenAudio_ArduinoLibrary](https://github.com/chipaudette/OpenAudio_ArduinoLibrary/tree/master?tab=readme-ov-file#openaudio-library-for-teensy)** by Chip Audette
* OS: Windows / macOS / Linux

---

## Quick Start

1. **Phase 1 (Offline MATLAB)**

   * Open [`runPhase1.m`](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/blob/main/Phase%201%20MATLAB%20code/runPhase1.m)
   * Select a WAV file (e.g., `05_Control_2_Motorcycle_Car_towards.wav`)
   * Set `direction = "towards"` or `"away"` and run the script

2. **Phase 2 (Embedded Validation)**

   * Open `Phase_2_MATLAB_Code/runPhase2.m`
   * Select a Teensy-recorded WAV file and run to reproduce Chapter 4 results

3. **Teensy Firmware**

   * Open `Teensy_Embedded_Code/TeensyRadarLogger.ino`
   * Flash to Teensy 4.1 using TeensyDuino
   * Recorded `.BIN` files will be saved on the SD card as `RECXXXX.BIN`


---

## Publications

Portions of this work were presented at **IEEE AFRICON 2025** under the title:
*“Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar System.”*

---

## License

This repository is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.
All MATLAB and firmware code are released for academic and research use.

---

## Acknowledgements

Developed by **Dylan Kuming**
Supervised by **Dr Yunus Abdul Gaffar**
*Radar Remote Sensing Group, University of Cape Town*

This project forms part of ongoing research into **low-cost intelligent traffic sensing** using CW Doppler radar technology.
