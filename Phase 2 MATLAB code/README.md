 # Phase 2 MATLAB Code

This folder contains MATLAB scripts for processing **Teensy 4.1 binary logs** to perform **vehicle speed estimation**, **multi-target tracking**, and **rule-based vehicle classification**.

The pipeline reconstructs a power spectrogram from a `.bin` file written by the Teensy firmware (4096-point, 50% overlap FFT frames), then applies Ordered-Statistic CFAR detection, **gap-based** 1-D clustering (per time frame),JPDA-based multi-target tracking with a Constant-Acceleration (CA) Kalman filter, and **locally normalised** ramp-width-based classification.

Three sample `.bin` recordings are included—identical to those used in **Chapter 4** of the MSc dissertation—allowing direct reproduction of the results and figures presented there.

---


## Processing Pipeline

1. **Spectrogram reconstruction** — reshape Teensy `.bin` (per-frame power) to `[2048 × N]`, build time/frequency axes, convert frequency → speed
2. **Target detection** — Ordered-Statistic CFAR (OS-CFAR)
3. **Clustering** — **gap-based** 1-D clustering per time frame (split where Δspeed > ε; keep segments with ≥ minPts)
4. **Multi-target tracking** — JPDA with a CA Kalman filter
5. **Track merging** — overlap- and same-velocity short-gap consolidation
6. **Speed estimation** — including cosine-angle correction
7. **Classification** — rule-based using **locally normalised** Doppler-ramp width within each vehicle’s ramp window

   * `< 3` → *Motorcycle/Bicycle*
   * `3 – 9.5` → *Car/Minibus*
   * `> 9.5` → *Bus/Truck*

---

## Requirements

* MATLAB R2022b or later (earlier may work, but tested on recent releases)
* Toolboxes:

  * **Symbolic Math Toolbox** (`vpasolve`) 
  * **Statistics and Machine Learning Toolbox** (`dbscan`)
  * **Sensor Fusion and Tracking Toolbox** (`trackerJPDA`, `objectDetection`)
  * **Image Processing Toolbox** (`bwareaopen`)

---

---
### Accessing the Reference BIN Recordings

Due to GitHub’s file-size limit, the three reference `.bin` recordings used in Phase 2 are hosted externally on OneDrive.

📂 **Download all recordings here:**  
[Open OneDrive folder (Radar Recordings – Phase 2)](https://1drv.ms/f/c/e7b851d77e25a67e/Ety4YTov5G5HuUIxn-jq9TwBO-RcUHev4Lw48Tf9dcHDjQ?e=oUw4Rd)

These files correspond to the examples listed below and can be used directly with the provided MATLAB scripts.

### Reference BIN Recordings (Examples)

| File                                  | Description                         | Suggested `direction` |
| ------------------------------------- | ----------------------------------- | --------------------- |
| 01_Phase2_Car_away.bin                | Controlled short run of a car       | "away"                |
| 02_Phase2_uncontrol_30min_towards.bin | 30-min uncontrolled test (towards)  | "towards"             |
| 03_Phase2_uncontrol_30min_away.bin    | 30-min uncontrolled test (away)     | "away"                |


### Reference BIN Recordings (Examples)

| File                                  | Description                         | Suggested `direction` |
| ------------------------------------- | ----------------------------------- | --------------------- |
| 01_Phase2_Car_away.bin                | Controlled short run of a car       | "away"                |
| 02_Phase2_uncontrol_30min_towards.bin | 30-min uncontrolled test (towards)  | "towards"             |
| 03_Phase2_uncontrol_30min_away.bin    | 30-min uncontrolled test (away)     | "away"                |

> Any valid Teensy-generated `.bin` with 4096-pt, 50%-overlap FFT frames will work, provided the **script parameters match the firmware** (`fs`, `frameLength`, `overlap`).

---

### Key MATLAB Scripts

| Script                              | Purpose                                                                                    |
| ----------------------------------- | ------------------------------------------------------------------------------------------ |
| **runPhase2.m**                     | Main end-to-end pipeline (BIN → spectrogram → detect → cluster → track → merge → classify) |
| **solveAlphaOS.m**                  | Solves OS-CFAR scaling factor                                                              |
| **oscfarDetector.m**                | Column-wise OS-CFAR detection                                  |
| **solveAlphaOS.m**                  | Solves OS-CFAR scaling factor                                  |
| **initCAKF.m**                      | Constant-Acceleration Kalman filter initialiser                |
| **mergeTracksAllPassesAvg.m**       | Overlap-based track merge                                      |
| **mergeTracksSameVelocityGap.m**    | Same-velocity short-gap merge                                  |
| **computeVehicleSpeedWithExtrap.m** | Calculates representative speed and extrapolated zero-crossing |
---

## How to Run

1. Open MATLAB in this repository folder.

2. Open  **runPhase2.m**  .

3. At the top of the file, set the `.bin` filename and direction:

   ```matlab
   fileName    = '02_Phase2_uncontrol_30min_towards.bin';
   direction = "towards";   % or "away"
   ```

4. Run the script.

The program will:

* Generate and display multiple figures (spectrograms, detections, clusters, tracks, classification)
* Print a summary table of detected vehicles with estimated speeds and classes

---

## Example Output

```

Total Vehicles Detected: 92

Vehicle  Speed (m/s)     Speed (km/h)    Time. Width (s)     Norm. Width (m)    Direction    Class       
----------------------------------------------------------------------------------------------------
1        13.39           48.21           0.33                4.35                towards      Car/Minibus 
2        8.20            29.53           0.74                6.10                towards      Car/Minibus 
3        9.67            34.79           0.51                4.94                towards      Car/Minibus
...
90       8.25            29.68           1.58               13.02                towards      Bus/Truck   
91       6.90            24.85           0.74               5.13                 towards      Car/Minibus 
92       10.76           38.72           0.37               4.00                 towards      Car/Minibus 
```     
---

## Core Parameters (Editable in Script)

* **Spectrogram reconstruction (must match firmware):**
  `fs = 44.1e3`, `frameLength = 4096`, `overlap = frameLength*0.5`

* **CFAR:**
  `PFA = 1e-6`, `RefWindow = 100`, `NumGuardCells = 4`, `NumRefCells = 2 * RefWindow`, `k = floor(NumRefCells / 3)`

* **Gap-based clustering (per time frame, in m/s):**
  `epsilon = 0.5` (split clusters where adjacent Δspeed > ε)
  `minPts  = 2`  (reject tiny segments)

* **JPDA Tracker:**
  `AssignmentThreshold = 5`, `ConfirmationThreshold = [4 6]`, `DeletionThreshold = [8 16]`
  Measurement noise variance: `R_meas = 0.04`

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
* For long recordings (e.g., **30 min**), ensure sufficient RAM for the `[2048 × N]` power matrix.
* Keep the **bin-to-axes parameters** in sync with Teensy firmware settings to avoid axis distortion.

---
