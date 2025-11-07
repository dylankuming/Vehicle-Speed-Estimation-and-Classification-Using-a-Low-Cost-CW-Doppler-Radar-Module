# Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar

This repository contains all code and resources developed for the MSc dissertation
**“Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar”**
by **Dylan Kuming**, University of Cape Town (2025).

---

## Repository Overview

The repository is organised into three main components:

| Folder                          | Description                                                                                                                                     |
| --------------------------------| ----------------------------------------------------------------------------------------------------------------------------------------------- |
| **[Phase 1 MATLAB code](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/tree/main/Phase%201%20MATLAB%20code)**  | Offline MATLAB processing and validation (Chapter 3) — performs spectrogram generation, detection, clustering, tracking, and classification on `.wav` recordings.          |
| **[Phase 2 Teensyduino code](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/tree/main/Phase%202%20Teensyduino%20code)** | C++ firmware for the Teensy 4.1 + SGTL5000 audio codec — performs real-time 4096-point FFTs and logs consecutive FFT power spectra (2048 float bins per frame) to the SD card as a binary `.bin` file for offline spectrogram reconstruction and analysis. |
| **[Phase 2 MATLAB code](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar-Module/tree/main/Phase%202%20MATLAB%20code)**  |Embedded-system validation (Chapter 4) — reconstructs the spectrogram from the `.bin` file before re-applying the Phase 1 processing pipeline, incorporating refinements to clustering and classification.       |


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

   * Open [`runPhase1.m`](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar/blob/main/Phase%201%20MATLAB%20code/runPhase1.m)
   * Select a `.wav` file (e.g., `05_Control_2_Motorcycle_Car_towards.wav`)
   * Set `direction = "towards"` or `"away"` and run the script

2. **Teensy Firmware**

   * Open [`Phase2_Teensy_FFTLogger_4096.ino`](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar/blob/main/Phase%202%20Teensyduino%20code/Phase2_Teensy_FFTLogger_4096/Phase2_Teensy_FFTLogger_4096.ino) in the Arduino IDE.  
   * Compile and upload the sketch to the **Teensy 4.1** with the **SGTL5000 audio shield** attached.  
   * During operation, the firmware logs real-time FFT power spectra to the onboard SD card.  
   * Each recording is saved as a binary file (`RECxxxx.bin`) for later reconstruction in MATLAB (see [`runPhase2.m`](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar/blob/main/Phase%202%20MATLAB%20code/runPhase2.m)).
  
 3. **Phase 2 (Embedded Validation)**

   * Open [`runPhase2.m`](https://github.com/dylankuming/Vehicle-Speed-Estimation-and-Classification-Using-a-Low-Cost-CW-Doppler-Radar/blob/main/Phase%202%20MATLAB%20code/runPhase2.m)
   * Select a `.bin` file (e.g., `02_Phase2_uncontrol_30min_towards.bin`)
   * Set `direction = "towards"` or `"away"` and run the script


---

## Publications

Portions of this work were presented at the [IEEE AFRICON 2025 CONFERENCE](https://www.ul.ac.za/ieee-africon-2025-conference/) under the title:<br>
*“Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar.”*

---

## License

This repository is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.<br>
All MATLAB and firmware code are released for academic and research use.

---

## Acknowledgements

Developed by **Dylan Kuming**<br>
Supervised by **Dr Yunus Abdul Gaffar**<br>
*Radar Remote Sensing Group, University of Cape Town*

This project forms part of ongoing research into **low-cost intelligent traffic sensing** using CW Doppler radar technology.
