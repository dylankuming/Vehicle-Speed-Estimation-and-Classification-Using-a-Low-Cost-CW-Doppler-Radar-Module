# CW Doppler Radar: Speed, Tracking & Classification (MATLAB)

End-to-end MATLAB pipeline for roadside vehicle **speed estimation, multi-target tracking (JPDA + CA-KF), and rule-based classification** using a **24 GHz CW Doppler** sensor.
The demo runs on eight reference recordings (WAV files) from **Chapter 3** of the accompanying MSc dissertation and reproduces the figures/behaviour described there.

---

## What this repo does

* Builds a **speed–time spectrogram** from audio (radar IF) logs
* Detects targets with **OS-CFAR**
* Consolidates hits per frame with **1-D DBSCAN**
* Tracks multiple vehicles with **trackerJPDA** (custom **CA Kalman** filter)
* Merges fragmented tracks (overlap + same-velocity short-gap logic)
* Picks a representative speed per vehicle (cosine-corrected)
* Classifies vehicles by **normalised ramp width**:

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

## Folder contents (core files)

* **WAV recordings (8 options; numbered as in Dissertation Chapter 3):**

  1. `01_LM358_Car_away.wav`
  2. `02_AD620_Car_away.wav`
  3. `03_Motorbike_Car_towards.wav`
  4. `04_Control_1_Car_Motorcycle_away.wav`
  5. `05_Control_2_Motorcycle_Car_towards.wav`
  6. `06_Uncontrol_1_Bus_away.wav`
  7. `07_Uncontrol_2_4Cars_away.wav`
  8. `08_Uncontrol_3_2Cars_towards.wav`
* **Pipeline & helpers**

  * `powerSpectrogram.m` – STFT → power spectrogram (|STFT|²)
  * `oscfarDetector.m` – OS-CFAR per column
  * `solveAlphaOS.m` – OS-CFAR scaling factor (given PFA, N, k)
  * `initCAKF.m` – custom Constant-Acceleration Kalman filter init
  * `mergeTracksAllPassesAvg.m` – overlap-based merge (avg speed diff)
  * `mergeTracksSameVelocityGap.m` – same-velocity short-gap merge
  * `computeVehicleSpeedWithExtrap.m` – representative speed + 0 m/s extrapolation
  * `runPhase1.m` – (optional) alternative script used during Phase 1 work
* **Main demo script (this README targets it):** your provided script block (save as e.g. `demo_main.m`)

> The screenshot in the repo shows these files together with the eight WAVs.

---

## Quick start

1. Open MATLAB in this folder.
2. Edit the top of the main script to choose **one** recording and its **direction**:

```matlab
% --- Select ONE of the 8 WAV files (see list above) ---
wavFile  = '08_Uncontrol_3_2Cars_towards.wav';

% --- Must match the recording geometry ---
direction = "towards";   % or "away"
```

3. Run the script.
   It will generate several figures (spectrograms, detections, clustering, tracking, merged tracks) and print a summary table like:

```
Vehicle  Speed (m/s)   Speed (km/h)  Time. Width   Norm. Width   Direction   Class
1        12.34         44.42         1.25          8.65          towards     Car/Minibus
...
```

---

## Typical outputs (figures)

* **Normalised Power Spectrogram (10–70 km/h)**
* **Spectrogram with OS-CFAR detections**
* **DBSCAN clustering:** cluster points, noise points, and representative detections
* **Tracker results:** JPDA + CA-KF estimated speeds per track
* **Merged tracks:** overlap-only, then overlap + same-velocity (final)
* **Binary mask** used for ramp-width measurement
* **Final merged tracks with representative radial-speed marker** and ramp-detection overlay

---

## Key parameters you may tune

* **Spectrogram**

  * `frameLength = 4096;`
  * `overlap = 0.5 * frameLength;`
  * `windowFunction = hanning(frameLength);`
* **Crop range:** `10–70 km/h`
* **OS-CFAR**

  * `PFA = 1e-6;`
  * `RefWindow = 100;`
  * `NumGuardCells = 4;`
  * `k = floor((2*RefWindow)/3);`
* **DBSCAN (1-D over speed in m/s)**

  * `epsilon = 0.5;`  *(≈1.8 km/h)*
  * `minPts  = 2;`
* **Track merging**

  * overlap merge thresholds inside `mergeTracksAllPassesAvg.m`
  * gap merge: `maxTimeGap = 4;  maxSpeedGap = 0.5;` *(m/s)*
* **Classification**

  * Bounding box: `timeWindow = ±1.5 s`, speed band `3–5 m/s`
  * Size thresholds: `<3`, `3–9.5`, `>9.5` (*normalised width = median time width × representative speed*)

---

## Notes & conventions

* **Direction matters.**
  Set `direction = "towards"` when vehicles approach the radar; `"away"` when moving away.
  This affects how the 0 m/s crossing is extrapolated and how representative speed is chosen.
* **Cosine correction** is applied using the default lateral offset `D_perp = 2 m`. Adjust in the script if your geometry differs.
* **Wavelength:** fixed at `λ = c / 24e9` for a 24 GHz radar.
* All speeds shown on plots are **km/h**; internal processing is **m/s**.

---

## Reproducing Chapter 3 order

Run the eight WAVs **in the numbered order** above to mirror the progression in **Chapter 3** (prototype → improved amp → controlled → uncontrolled). Direction to set:

| File                                    | Suggested `direction` |
| --------------------------------------- | --------------------- |
| 01_LM358_Car_away.wav                   | `"away"`              |
| 02_AD620_Car_away.wav                   | `"away"`              |
| 03_Motorbike_Car_towards.wav            | `"towards"`           |
| 04_Control_1_Car_Motorcycle_away.wav    | `"away"`              |
| 05_Control_2_Motorcycle_Car_towards.wav | `"towards"`           |
| 06_Uncontrol_1_Bus_away.wav             | `"away"`              |
| 07_Uncontrol_2_4Cars_away.wav           | `"away"`              |
| 08_Uncontrol_3_2Cars_towards.wav        | `"towards"`           |

(If your recordings differ, set the direction to match your scene.)

---

## Troubleshooting

* **“No detections found”** → Raise signal levels (check WAV), relax CFAR (higher PFA or smaller `RefWindow`), or verify the crop band (10–70 km/h) covers your speeds.
* **Too many false alarms** → Lower PFA, increase guard cells, or increase detection threshold via OS-CFAR tuning (`k`, `RefWindow`).
* **Fragmented tracks** → Loosen `AssignmentThreshold`, increase `DeletionThreshold`, or rely more on the two-stage merge settings.
* **Classification = ‘Unknown’** → Ensure bounding box overlaps actual ramp (adjust `timeWindow`, `sMin/sMax`) and that the binary mask threshold (`threshold_dB`) isn’t too strict.

---

## Citing

If you use this code or the dataset, please cite the dissertation:

> Kuming, D. *Vehicle Speed Estimation and Classification Using a Low-Cost Continuous-Wave Doppler Radar*. MSc Dissertation, University of Cape Town, 2025.

---

## License

Choose a license (e.g., MIT) and place it in `LICENSE`.

---

## Acknowledgements

Built on MATLAB toolboxes listed above and inspired by classic CFAR/JPDA literature. Huge thanks to **Dr. Yunus Abdul Gaffar** for supervision and guidance throughout the project.

---

## Quick FAQ

* **Can I add my own WAVs?** Yes—drop them in the folder and set `wavFile` + `direction`. Keep sampling rate metadata intact.
* **Do I need ground truth?** Not for the demo, but the dissertation validates speed against GPS in controlled trials.
* **Can this run in real-time?** This repo shows the **offline** MATLAB back-end. The embedded real-time STFT logger (Teensy 4.1 + SGTL5000) is covered in Phase 2; post-processing code paths are present here.

