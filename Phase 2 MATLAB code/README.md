 # Phase 2 MATLAB Code

This folder contains MATLAB scripts for processing **Teensy 4.1 spectral logs** to perform **vehicle speed estimation**, **multi-target tracking**, and **rule-based vehicle classification**.

The pipeline reconstructs a power spectrogram from a `.bin` file written by the Teensy firmware (4096-point, 50% overlap FFT frames), then applies Ordered-Statistic CFAR detection, **gap-based** 1-D clustering (per time frame), JPDA-based multi-target tracking with a Constant-Acceleration (CA) Kalman filter, track merging, and **locally normalised** ramp-width classification.

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

  * **Signal Processing Toolbox** (windows/utilities)
  * **Sensor Fusion and Tracking Toolbox** (`trackerJPDA`, `objectDetection`)
  * **Image Processing Toolbox** (`bwareaopen`)
  * **Symbolic Math Toolbox** (`vpasolve`, if your `solveAlphaOS` uses it)

---

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
| **oscfarDetector.m**                | Column-wise OS-CFAR detection                                                              |
| **initCAKF.m**                      | Constant-Acceleration Kalman filter initialiser                                            |
| **mergeTracksAllPassesAvg.m**       | Overlap-based track merge                                                                  |
| **mergeTracksSameVelocityGap.m**    | Same-velocity short-gap merge                                                              |
| **computeVehicleSpeedWithExtrap.m** | Representative speed + extrapolated zero-crossing                                          |

---

## How to Run

1. Open MATLAB in this repository folder.
2. Open **runPhase2.m**.
3. At the top of the file, set the BIN filename, direction, and confirm firmware-matching parameters:

   ```matlab
   fileName   = '03_Phase2_uncontrol_30min_away.bin';
   direction  = "away";          % or "towards"

   fs          = 44.1e3;         % Teensy sample rate
   frameLength = 4096;           % FFT size on Teensy
   overlap     = frameLength*0.5;  % 50% overlap
   ```
4. Run the script.

The program will:

* Reconstruct and display the **spectrogram** (speed–time),
* Show **OS-CFAR detections**, **gap-based clusters**, **JPDA tracks**, **merged tracks**, and **classification** overlays,
* Print a summary table of detected vehicles with estimated speeds and classes.

---

## Example Output

```
Vehicle  Speed (m/s)  Speed (km/h)  Time. Width  Norm. Width  Direction  Class
1        12.34        44.42         1.25         8.65         away       Car/Minibus
2        6.11         22.00         0.70         2.15         away       Motorbike/Bicycle
```

---

## Core Parameters (Editable in Script)

* **Spectrogram reconstruction (must match firmware):**
  `fs = 44.1e3`, `frameLength = 4096`, `overlap = frameLength*0.5`

* **CFAR:**
  `PFA = 1e-6`, `RefWindow = 100`, `NumGuardCells = 4`,
  `NumRefCells = 2 * RefWindow`, `k = floor(NumRefCells / 3)`

* **Gap-based clustering (per time frame, in m/s):**
  `epsilon = 0.5` (split clusters where adjacent Δspeed > ε)
  `minPts  = 2`  (reject tiny segments)

* **JPDA Tracker:**
  `AssignmentThreshold = 5`, `ConfirmationThreshold = [4 6]`, `DeletionThreshold = [8 16]`
  Measurement noise variance: `R_meas = 0.04`

* **Track Merging:**
  Overlap-based thresholds (inside `mergeTracksAllPassesAvg`)
  Same-velocity gap merge: `maxTimeGap = 4 s`, `maxSpeedGap = 0.5 m/s`
  Track discard: `T_min = 2.25 s` (minimum retained duration)

* **Classification (local normalisation):**
  `timeWindow = ±1.5 s` around the 0 m/s crossing
  `sMin = 3 m/s`, `sMax = 5 m/s`
  Local threshold: `threshold_dB = -22 dB` (relative to **local** max inside the ramp window)
  Over-coverage guard: if detection coverage `> 0.85` → mark `"Noise"` (dropped before printing)
  Class thresholds (normalised width = row-span time × representative speed):
  `< 3` → *Motorbike/Bicycle*; `3–9.5` → *Car/Minibus*; `> 9.5` → *Bus/Truck*

---

## Notes

* **Direction ("towards"/"away")** affects how the zero-speed crossing is extrapolated.
* **Cosine correction** assumes a 2 m lateral offset (`D_perp = 2 m`); adjust for your geometry.
* **Units:** Internal speeds are in m/s; all plots display km/h.
* For long recordings (e.g., **30 min**), ensure sufficient RAM for the `[2048 × N]` power matrix.
* Keep the **bin-to-axes parameters** in sync with Teensy firmware settings to avoid axis distortion.

---
