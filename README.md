# Adaptive Traffic Control System — Project README

> **Last updated:** 2026-05-04 by Antigravity AI assistant
> **Update policy:** This file is updated every time a working session makes a change to the project.

---

## Project Overview

A **SUMO-based Adaptive Traffic Control System** with GPS Emergency Preemption and YOLOv11-based vehicle counting. Built as a college project.

### Core Goals
- Simulate a 4-way junction with realistic adaptive traffic light control
- Use computer vision (YOLO) to count vehicles and dynamically adjust green light durations
- Preempt the normal light cycle when an ambulance/emergency vehicle is within 150m of the junction
- Run on both local GPU (WSL Ubuntu) and headless CPU (Google Colab)

---

## Tech Stack

| Component | Tool/Library | Version |
|---|---|---|
| Traffic Simulation | SUMO | 1.26.0 |
| Simulation Bridge | TraCI (Python) | bundled with SUMO |
| Primary Detector | YOLOv11 Medium | `yolo11m.pt` |
| Secondary Detector | YOLO26 Medium | `yolo26m.pt` |
| OS / Environment | WSL Ubuntu on Windows 11 | — |
| Python Env | Conda `traffic_control` | — |
| Cloud Backup | Google Colab (headless/CPU) | — |

---

## Project Structure

```
~/traffic_control/
├── adaptive_traffic_traci_bridge.py   # MAIN — connects YOLO to SUMO via TraCI
├── yolo_detector.py                   # YOLO dual-model detector + FrameSource
├── visual_enhancements.py             # GUI helpers (camera tracking, headless-safe)
├── cleanup.sh                         # Run once to delete old grid-network files
├── frames/                            # Drop intersection images here for YOLO input
│   └── README.txt                     # Explains how to name and place images
├── models/                            # ← GITIGNORED — put .pt files here
│   ├── yolo11m.pt                     # YOLOv11 Medium weights
│   └── yolo26m.pt                     # YOLO26 Medium weights
└── sumo_network/
    ├── single_junction.net.xml        # The 4-way junction network
    ├── single_junction.rou.xml        # Vehicle routes + flows (all 4 directions)
    ├── single_junction.sumocfg        # SUMO config — entry point
    ├── gui-settings.xml               # SUMO GUI visual settings
    └── additional.add.xml             # Extra SUMO definitions
```

> **Note:** `models/` is in `.gitignore`. The `.pt` files are large and must be kept locally. The code will **never auto-download** them — it will raise a clear error if they are missing.

---

## How to Run

### Normal (with GUI)
```bash
conda activate traffic_control
cd ~/traffic_control
python adaptive_traffic_traci_bridge.py --gui
```

### Diagnostic Mode (shows live phase table in terminal)
```bash
python adaptive_traffic_traci_bridge.py --gui --test
```

### Headless (Google Colab / no display)
```bash
python adaptive_traffic_traci_bridge.py
```

---

## How SUMO and YOLO Work Together

These are **two independent systems** that communicate through the bridge:

| System | Role | Data Source |
|---|---|---|
| **SUMO** | Physics engine — spawns and moves cars | `single_junction.rou.xml` |
| **YOLO** | Vision — counts waiting vehicles per approach | Images in `frames/` folder |
| **Bridge** | Brain — feeds YOLO counts → adjusts SUMO green times | `adaptive_traffic_traci_bridge.py` |

> ⚠️ YOLO images do **not** spawn cars in SUMO. They only affect green light durations.

### Green Time Formula
```
YOLO count for busiest approach = N
Green duration = base_green + (N // 5) × 15s
Example: 12 cars → 15 + (2 × 15) = 45s
```

---

## Traffic Light Cycle (Verified)

| Phase | State (first 10 chars) | Meaning | Duration |
|---|---|---|---|
| 0 | `GGGggrrrrrr` | North/South GREEN | 41s (adaptive) |
| 1 | `yyyyyrrrrr…` | North/South YELLOW | 3s (fixed) |
| 2 | `rrrrrrrrrr…` | ALL RED | 1s (fixed) |
| 3 | `rrrrrGGGgg…` | East/West GREEN ← ev0 preempts here | 41s (adaptive) |
| 4 | `rrrrryyyyy…` | East/West YELLOW | 3s (fixed) |
| 5 | `rrrrrrrrrr…` | ALL RED | 1s (fixed) |

**Full cycle = 90 seconds.**
Emergency vehicle `ev0` travels West→East, so phase **3** is the correct preemption target.

---

## Reading the Diagnostic Output

When running with `--test`, you see:
```
t=  46.0s | TLS=junction | ph=3 | state=rrrrrGGGgg… | remain=41.0s | preempt=no | EV=--- | YOLO [N:0 S:0 E:0 W:0]
```

| Field | Meaning |
|---|---|
| `t=46.0s` | Simulation time in seconds |
| `TLS=junction` | Traffic light ID in the network |
| `ph=3` | Current phase index (3 = East/West green) |
| `state=rrrrrGGGgg…` | First 10 chars of the 20-char light state |
| `remain=41.0s` | Seconds left before phase changes |
| `preempt=no` | Emergency preemption not active |
| `EV=---` | No emergency vehicle detected |
| `YOLO [N:0 ...]` | YOLO vehicle counts per approach |

---

## Bugs Fixed (History)

| Date | Bug | Fix |
|---|---|---|
| 2026-05-04 | **Stuck TLS** — `setPhaseDuration` called every second, resetting the countdown | Added `_last_phase` tracking — only fires on phase *transition* |
| 2026-05-04 | **Duplicate preemption** — `preemption_logic()` in visual_enhancements fighting `EmergencyPreemptor` | Removed the duplicate `preemption_logic(step)` call from the main loop |
| 2026-05-04 | **Flicker on release** — `setPhaseDuration(0)` caused rapid flickering after ambulance passed | Replaced with `setProgram("0")` to restore the default TLS program |
| 2026-05-04 | **Wrong preemption phase** — emergency phase set to 0 (N/S green) instead of 3 (E/W green) | Fixed `tls_emergency_phase["junction"] = 3` |
| 2026-05-04 | **Hidden stuck TLS** — `reset_phase_tracking()` called every second, re-breaking the phase fix | Removed the every-second external call; `apply()` manages `_last_phase` internally |
| 2026-05-04 | **Model auto-download** — YOLO downloaded `.pt` files at runtime instead of using local `models/` | Added `_resolve_model_path()` that checks `models/` folder and raises `FileNotFoundError` if missing (never downloads) |

---

## Model Loading Rules

The `.pt` model files live in `models/` which is **gitignored** (never pushed to GitHub).

Search order when loading:
1. Exact path as given (e.g. an absolute path)
2. `models/<filename>` relative to the project root

If neither is found → **hard error** with a clear message. No auto-download.

```
FileNotFoundError: Model weights not found: 'models/yolo11m.pt'
  Checked : /home/anaghashree/traffic_control/models/yolo11m.pt
  Fix     : Place your .pt files inside the 'models/' folder.
```

---

## Feeding YOLO Images

```bash
# Copy your intersection images into the frames folder
cp /mnt/c/Users/Anaghashree/Desktop/my_images/*.jpg ~/traffic_control/frames/

# Name them sequentially so they sort and loop correctly:
# frame_0001.jpg, frame_0002.jpg, frame_0003.jpg ...
```

YOLO reads one frame per second and maps detections to ROI zones:
```
north: (0,   0,   640, 180)   ← top strip
south: (0,   460, 640, 640)   ← bottom strip
east:  (460, 0,   640, 640)   ← right strip
west:  (0,   0,   180, 640)   ← left strip
```
Adjust these pixel coordinates in `DetectorConfig` to match your camera angle.

---

## Open Tasks / Next Steps

- [ ] Answer 5 y/n questions about model loading behavior (1-y, 2-y, 3-y, 4-n, 5-y ✅ answered)
- [ ] Feed actual intersection images into `frames/` and tune the ROI zones
- [ ] Run `bash cleanup.sh` to delete leftover old grid-network files from `sumo_network/`
- [ ] Test emergency preemption: verify `preempt=yes` appears at `t=100s` when `ev0` spawns

---

*This README is maintained by the Antigravity AI assistant and updated after each working session.*
