# CW Doppler Radar: Speed, Tracking & Classification (MATLAB)
This repository contains MATLAB code for processing continuous-wave (CW) Doppler radar recordings to perform **vehicle speed estimation**, **multi-target tracking (JPDA + CA Kalman filter)**, and **rule-based vehicle classification**.

The script runs on **any valid radar WAV file**, provided it contains a baseband Doppler signal from a 24 GHz CW radar.  
Eight reference recordings are included in the repository — the same examples used for the demonstrations, testing, and results presented in **Chapter 3** of the MSc dissertation. Running them will reproduce the figures and behaviour described there.

---

## Overview

The full radar signal-processing pipeline includes:
1. **Spectrogram generation** using the short-time Fourier transform (STFT)
2. **Target detection** via Ordered-Statistic CFAR (OS-CFAR)
3. **Per-frame clustering** using 1-D DBSCAN
4. **Multi-target tracking (MTT)** with a Joint Probabilistic Data Association (JPDA) tracker and a Constant-Acceleration (CA) Kalman filter
5. **Track merging** (overlap + same-velocity short-gap)
6. **Speed estimation** with cosine-angle correction
7. **Vehicle classification** using normalised Doppler-ramp width
  * `< 3` → *Motorbike/Bicycle*
  * `3–9.5` → *Car/Minibus*
  * `> 9.5` → *Bus/Truck*

---

## Requirements

* MATLAB R2022b or later (earlier may work, but tested on recent releases)
* Toolboxes:

  * **Signal Processing Toolbox** (STFT, windows)
  * **Statistics and Machine Learning Toolbox** (`dbscan`)
  * **Sensor Fusion and Tracking Toolbox** (`trackerJPDA`, `objectDetection`)
  * **Image Processing Toolbox** (`bwareaopen`)
* OS: Windows/macOS/Linux

---

### Reference WAV Recordings (Examples)

| File                                    | Description                  | Suggested `direction` |
| --------------------------------------- | ---------------------------- | --------------------- |
| 01_LM358_Car_away.wav                   | Early LM358 prototype test   | "away"                |
| 02_AD620_Car_away.wav                   | Improved amplifier test      | "away"                |
| 03_Motorbike_Car_towards.wav            | Controlled two-vehicle trial | "towards"             |
| 04_Control_1_Car_Motorcycle_away.wav    | Controlled trial             | "away"                |
| 05_Control_2_Motorcycle_Car_towards.wav | Controlled trial             | "towards"             |
| 06_Uncontrol_1_Bus_away.wav             | Uncontrolled roadside trial  | "away"                |
| 07_Uncontrol_2_4Cars_away.wav           | Uncontrolled roadside trial  | "away"                |
| 08_Uncontrol_3_2Cars_towards.wav        | Uncontrolled roadside trial  | "towards"             |

> These eight recordings were analysed in Chapter 3 of the dissertation, but the code will process **any** valid radar WAV input.

---

### Key MATLAB Scripts

| Script                              | Purpose                                                        |
| ----------------------------------- | -------------------------------------------------------------- |
| **demo_main.m**                     | Main end-to-end pipeline                                       |
| **powerSpectrogram.m**              | Computes STFT → power spectrogram                              |
| **oscfarDetector.m**                | Column-wise OS-CFAR detection                                  |
| **solveAlphaOS.m**                  | Solves OS-CFAR scaling factor                                  |
| **initCAKF.m**                      | Constant-Acceleration Kalman filter initialiser                |
| **mergeTracksAllPassesAvg.m**       | Overlap-based track merge                                      |
| **mergeTracksSameVelocityGap.m**    | Same-velocity short-gap merge                                  |
| **computeVehicleSpeedWithExtrap.m** | Calculates representative speed and extrapolated zero-crossing |
| **runPhase1.m**                     |**main script**                               |

---

## How to Run

1. Open MATLAB in this repository folder.

2. Open  **runPhase1.m**  .

3. At the top of the file, set the WAV filename and direction:

   ```matlab
   wavFile   = '08_Uncontrol_3_2Cars_towards.wav';
   direction = "towards";   % or "away"
   ```

4. Run the script.

The program will:

* Generate and display multiple figures (spectrograms, detections, clusters, tracks, classification)
* Print a summary table of detected vehicles with estimated speeds and classes

---

## Example Output

```
Vehicle  Speed (m/s)  Speed (km/h)  Time. Width  Norm. Width  Direction  Class
1        12.34        44.42         1.25         8.65         towards    Car/Minibus
2        6.11         22.00         0.85         2.50         towards    Motorbike/Bicycle

```

---

## Core Parameters (Editable in Script)

* **Spectrogram:**
  `frameLength = 4096`, `overlap = 0.5 * frameLength`, `windowFunction = hanning(frameLength)`

* **CFAR:**
  `PFA = 1e-6`, `RefWindow = 100`, `NumGuardCells = 4`, `NumRefCells = 2 * RefWindow`, `k = floor(NumRefCells / 3)`

* **DBSCAN:**
  `epsilon = 0.5` m/s (≈ 1.8 km/h), `minPts = 2`

* **JPDA Tracker:**
  `AssignmentThreshold = 5`, `ConfirmationThreshold = [4 6]`, `DeletionThreshold = [8 16]`

* **Track Merging (Overlap-Based):**
  `maxTimeGap = 1.5 s`, `maxSpeedGap = 0.5 m/s`

* **Track Merging (Same-Velocity Gap):**
  `maxTimeGap = 4 s`, `maxSpeedGap = 0.5 m/s`

* **Track Discard:**
  `T_min = 2.25 s` (minimum track duration retained)

* **Classification:**
  `threshold_dB = -22 dB`, `timeWindow = ±1.5 s`, `sMin = 3 m/s`, `sMax = 5 m/s`


---

## Notes

* **Direction ("towards" / "away")** affects how the zero-speed crossing is extrapolated.
* **Cosine correction** assumes a 2 m lateral offset (`D_perp = 2 m`); adjust if geometry differs.
* **Units:** internal speeds are in m/s; all plots display km/h.
* Works with any properly sampled radar IF signal (typically 44.1 kHz, 16-bit PCM).

---

## Citation

If you use this repository, please cite:

> Kuming, D. (2025). *Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar.* MSc Dissertation, University of Cape Town.

---

## License

This project is licensed under the **MIT License** — see the [`LICENSE`](LICENSE) file for details.

---


## Acknowledgements

Developed under the supervision of **Dr Yunus Abdul Gaffar** (University of Cape Town).
This MATLAB implementation extends and validates the CW Doppler radar system presented in Chapter 3 of the dissertation.


```
