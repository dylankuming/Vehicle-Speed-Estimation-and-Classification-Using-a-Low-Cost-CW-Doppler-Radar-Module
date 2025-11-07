# Phase 2 MATLAB Code

This folder contains MATLAB code used in **Phase 2** of the project — corresponding to **Chapter 4** of the MSc dissertation — which validates the embedded CW Doppler radar platform under extended real-time roadside conditions.

The same end-to-end radar processing pipeline from Phase 1 is applied here, but the recordings are obtained directly from the **Teensy 4.1 + SGTL5000** embedded acquisition system.
Phase 2 verifies that the real-time implementation maintains comparable speed-estimation, tracking, and classification performance over longer durations and denser traffic scenes.

---

## Overview

The processing stages are identical to Phase 1:

1. **Spectrogram generation** using the short-time Fourier transform (STFT)
2. **Target detection** via Ordered-Statistic CFAR (OS-CFAR)
3. **Per-frame clustering** using 1-D DBSCAN
4. **Multi-target tracking (MTT)** with a JPDA tracker and Constant-Acceleration (CA) Kalman filter
5. **Track merging** (overlap + same-velocity gap)
6. **Speed estimation** with cosine-angle correction
7. **Vehicle classification** using normalised Doppler-ramp width

   * `< 3` → *Motorbike / Bicycle*
   * `3 – 9.5` → *Car / Minibus*
   * `> 9.5` → *Bus / Truck*

---

## Requirements

* MATLAB R2022b or newer
* Toolboxes

  * **Signal Processing Toolbox**
  * **Statistics and Machine Learning Toolbox**
  * **Sensor Fusion and Tracking Toolbox**
  * **Image Processing Toolbox**
* OS: Windows / macOS / Linux

---

### Example Recordings

The WAV files were recorded directly from the embedded Teensy 4.1 radar logger during Phase 2 field tests.
Each file contains raw baseband Doppler data sampled at 44.1 kHz.

| File                                    | Description            | Suggested `direction` |
| --------------------------------------- | ---------------------- | --------------------- |
| 01_Car_away.bin                         | Early embedded test    | "away"                |


> These examples reproduce the results and figures presented in **Chapter 4**.

---

### Key MATLAB Scripts

| Script                              | Purpose                                                       |
| ----------------------------------- | ------------------------------------------------------------- |
| **runPhase2.m**                     | Main end-to-end processing script (real-time data validation) |
| **oscfarDetector.m**                | Ordered-Statistic CFAR detector                               |
| **solveAlphaOS.m**                  | Solves OS-CFAR scaling constant                               |
| **initCAKF.m**                      | Constant-Acceleration Kalman filter initialiser               |
| **mergeTracksAllPassesAvg.m**       | Merges overlapping tracks                                     |
| **mergeTracksSameVelocityGap.m**    | Merges adjacent tracks with same velocity                     |
| **computeVehicleSpeedWithExtrap.m** | Computes representative vehicle speed with extrapolation      |

---

## How to Run

1. Open MATLAB in the **Phase 2 MATLAB code** directory.

2. Open **runPhase2.m**.

3. At the top of the file, set the WAV filename and motion direction:

   ```matlab
   wavFile   = '08_Uncontrol_3_2Cars_towards.bin';
   direction = "towards";   % or "away"
   ```

4. Run the script.

The script will:

* Generate multiple figures (spectrograms, detections, clusters, tracks, classifications)
* Print a formatted table of estimated speeds and classes

---

## Example Output

```
Vehicle  Speed (m/s)  Speed (km/h)  Time. Width  Norm. Width  Direction  Class
1        14.12        50.83         1.35         8.91         towards    Car/Minibus
2         7.22        26.00         0.72         2.15         towards    Motorbike/Bicycle
```

---

## Core Parameters (Editable in Script)

* **Spectrogram:**
  `frameLength = 4096`, `overlap = 0.5 * frameLength`, `windowFunction = hanning(frameLength)`

* **CFAR:**
  `PFA = 1e-6`, `RefWindow = 100`, `NumGuardCells = 4`, `NumRefCells = 2 * RefWindow`, `k = floor(NumRefCells / 3)`

* **DBSCAN:**
  `epsilon = 0.5 m/s (≈ 1.8 km/h)`, `minPts = 2`

* **JPDA Tracker:**
  `AssignmentThreshold = 5`, `ConfirmationThreshold = [4 6]`, `DeletionThreshold = [8 16]`

* **Track Merging (Overlap-Based):**
  `maxTimeGap = 1.5 s`, `maxSpeedGap = 0.5 m/s`

* **Track Merging (Same-Velocity Gap):**
  `maxTimeGap = 4 s`, `maxSpeedGap = 0.5 m/s`

* **Track Discard:**
  `T_min = 2.25 s` (minimum duration retained)

* **Classification:**
  `threshold_dB = -22`, `timeWindow = ±1.5 s`, `sMin = 3 m/s`, `sMax = 5 m/s`

---

## Notes

* **Direction** (`"towards"` / `"away"`) controls the extrapolation toward 0 m/s.
* **Cosine correction** assumes 2 m lateral offset (`D_perp = 2 m`).
* **Units:** internal = m/s, plots = km/h.
* Works with any valid radar binfile recorded from the Teensy embedded logger (44.1 kHz, 16-bit PCM).
* All figures correspond to results in **Chapter 4** – real-time validation.

---
