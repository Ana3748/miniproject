# Adaptive Traffic Control System (ATCS)

An intelligent traffic management system that combines Computer Vision (YOLO) with Adaptive Simulation (SUMO/TraCI) to optimize signal timings based on real-time vehicle density and emergency vehicle priority.

## 🚀 Overview

This project consists of two primary modules:
1.  **Vision Module**: Uses YOLOv8/v11 to detect and count vehicles from video feeds, categorizing them into Indian vehicle classes (Hatchback, Bus, Two-wheeler, etc.).
2.  **Simulation Module**: A SUMO-based environment that implements an adaptive traffic signal algorithm using TraCI. It features dynamic green extensions, early release for empty approaches, and emergency vehicle preemption.

---

## 🛠️ Project Structure

```text
.
├── simulation/           # Core simulation logic and TraCI bridge
│   ├── core/             # Configuration and utility functions
│   ├── interfaces/       # Vehicle count providers (YOLO/Hardcoded)
│   └── logic/            # Adaptive control and Preemption algorithms
├── vision/               # Computer Vision pipeline
│   ├── detector.py       # YOLO inference logic
│   ├── pipeline.py       # Video processing and count exporting
│   └── utils.py          # Visualization and image processing
├── sumo_network/         # SUMO network files (.net.xml, .rou.xml, .sumocfg)
├── assets/               # Videos and images for testing
├── outputs/              # Exported counts and detection frames
└── ALGORITHM.md          # Detailed explanation of the signal logic
```

---

## 🚦 Key Features

### 1. Adaptive Signal Control
The system dynamically adjusts green light durations based on **Passenger Car Unit (PCU)** density.
- **Base Green**: 15s initial duration.
- **Dynamic Extensions**: Adds 15s (up to 3 times) if the current approach has higher demand.
- **Early Release**: Immediately switches phases if the current approach becomes empty.

### 2. Emergency Preemption
A high-priority system that detects emergency vehicles (e.g., Ambulances) and forces a green signal for their specific approach until they clear the junction.

### 3. Multi-Class Vehicle Detection
Optimized for the Indian context, the vision pipeline detects:
- **Large**: Bus, Truck, Mini-bus, Tempo-traveller
- **Medium**: Hatchback, Sedan, SUV, MUV, LCV, Van
- **Small**: Three-wheeler, Two-wheeler, Bicycle

---

## 📦 Setup & Installation

1.  **Install SUMO**: Ensure [SUMO](https://eclipse.dev/sumo/) is installed and `SUMO_HOME` is set in your environment variables.
2.  **Virtual Environment**:
    ```bash
    python -m venv .venv
    source .venv/bin/activate  # Linux
    pip install -r requirements.txt
    ```
3.  **Download Models**: The vision module expects YOLO weights in the `models/` directory.

---

## 🏃 Usage

### Running the Vision Pipeline
To process video files and export vehicle counts:
```bash
python -m vision.pipeline
```

### Running the Adaptive Simulation
To start the SUMO simulation with adaptive logic:
```bash
python -m simulation.runner
```
*Optional flags:*
- `--gui`: Enable SUMO GUI.
- `--emergency`: Simulate emergency vehicle arrivals.

---

## 📖 Documentation

For a deep dive into the math and logic behind the signal switching, see:
👉 **[ALGORITHM.md](./ALGORITHM.md)**
