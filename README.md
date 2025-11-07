# Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar

This repository contains all code and resources developed for the MSc dissertation
**“Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar”**
by **Dylan Kuming**, University of Cape Town (2025).

---

## Repository Overview

The repository is organised into three main components:

| Folder                          | Description                                                                                                                                     |
| --------------------------------| ----------------------------------------------------------------------------------------------------------------------------------------------- |
| **[Phase 1 MATLAB code](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/tree/main/Phase%201%20MATLAB%20code)**  | Offline MATLAB processing and validation (Chapter 3) — performs spectrogram generation, detection, clustering, tracking, and classification on `.WAV` recordings.          |
| **[Phase 2 Teensyduino code](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/tree/main/Phase%202%20Teensyduino%20code)** | C++ firmware for the Teensy 4.1 + SGTL5000 audio codec — performs real-time 4096-point FFTs and logs consecutive FFT power spectra (2048 float bins per frame) to the SD card as a binary `.BIN` file for offline spectrogram reconstruction and analysis. |
| **[Phase 2 MATLAB code](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/tree/main/Phase%202%20MATLAB%20code)**  |Embedded-system validation (Chapter 4) — reconstructs the spectrogram from the `.BIN` file before re-applying the Phase 1 processing pipeline, incorporating refinements to clustering and classification.       |


Each folder contains its own README detailing setup, requirements, parameters, and usage.

---


## Requirements

* **MATLAB R2022b** or later

  * Signal Processing Toolbox
  * Symbolic Math Toolbox
  * Statistics and Machine Learning Toolbox
  * Sensor Fusion and Tracking Toolbox
  * Image Processing Toolbox
* **Arduino IDE v2.0.4+**
* **TeensyDuino 1.59.0+**
* **[OpenAudio_ArduinoLibrary](https://github.com/chipaudette/OpenAudio_ArduinoLibrary/tree/master?tab=readme-ov-file#openaudio-library-for-teensy)** by Chip Audette


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
*“Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar.”*

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
