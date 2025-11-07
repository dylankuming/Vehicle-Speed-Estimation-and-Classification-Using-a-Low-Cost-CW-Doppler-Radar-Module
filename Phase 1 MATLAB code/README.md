# Phase 1 MATLAB Code

This folder contains MATLAB scripts for processing **continuous-wave (CW) Doppler radar recordings** to perform **vehicle speed estimation**, **multi-target tracking**, and **rule-based vehicle classification**.

The pipeline converts a baseband Doppler signal from a 24 GHz CW radar (stored as a `.wav` file) into continuous vehicle trajectories and final class labels. It integrates short-time Fourier transform (STFT) spectrogram generation, Ordered-Statistic CFAR detection, 1-D DBSCAN clustering, JPDA-based multi-target tracking with a Constant-Acceleration (CA) Kalman filter, and ramp-width-based classification.

Eight sample recordings are included—identical to those used in **Chapter 3** of the MSc dissertation—allowing direct reproduction of the results and figures presented there.

---

## Processing Pipeline

1. **Spectrogram generation** — STFT-based time–frequency analysis
2. **Target detection** — Ordered-Statistic CFAR (OS-CFAR)
3. **Clustering** — 1-D DBSCAN per time frame
4. **Multi-target tracking** — JPDA with a CA Kalman filter
5. **Track merging** — overlap- and velocity-gap-based consolidation
6. **Speed estimation** — including cosine-angle correction
7. **Classification** — rule-based using normalised Doppler-ramp width

   * `< 3` → *Motorcycle/Bicycle*
   * `3 – 9.5` → *Car/Minibus*
   * `> 9.5` → *Bus/Truck*

---

## Requirements

* MATLAB R2022b or later (earlier may work, but tested on recent releases)
* Toolboxes:

  * **Signal Processing Toolbox** (STFT, windows)
  * **Symbolic Math Toolbox** (`vpasolve`) 
  * **Statistics and Machine Learning Toolbox** (`dbscan`)
  * **Sensor Fusion and Tracking Toolbox** (`trackerJPDA`, `objectDetection`)
  * **Image Processing Toolbox** (`bwareaopen`)


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
|  **runPhase1.m**                    | Main end-to-end pipeline                                       |
| **powerSpectrogram.m**              | Computes STFT → power spectrogram                              |
| **oscfarDetector.m**                | Column-wise OS-CFAR detection                                  |
| **solveAlphaOS.m**                  | Solves OS-CFAR scaling factor                                  |
| **initCAKF.m**                      | Constant-Acceleration Kalman filter initialiser                |
| **mergeTracksAllPassesAvg.m**       | Overlap-based track merge                                      |
| **mergeTracksSameVelocityGap.m**    | Same-velocity short-gap merge                                  |
| **computeVehicleSpeedWithExtrap.m** | Calculates representative speed and extrapolated zero-crossing |


---

## How to Run

1. Open MATLAB in this repository folder.

2. Open  **runPhase1.m**  .

3. At the top of the file, set the WAV filename and direction:

   ```matlab
   wavFile   = '05_Control_2_Motorcycle_Car_towards.wav';
   direction = "towards";   % or "away"
   ```

4. Run the script.

The program will:

* Generate and display multiple figures (spectrograms, detections, clusters, tracks, classification)
* Print a summary table of detected vehicles with estimated speeds and classes

---

## Example Output

```
Total Vehicles Detected: 2

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

---
