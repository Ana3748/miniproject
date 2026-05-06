from dataclasses import dataclass, field

@dataclass
class Config:
    # --- SUMO ---
    sumo_cfg: str = "your_network.sumocfg"
    use_gui: bool = False
    step_length: float = 0.1          # seconds per simulation step

    # --- Emergency Preemption ---
    use_emergency: bool = False       # Set via --emergency flag
    preemption_radius_m: float = 150.0
    emergency_vehicle_type: str = "emergency"
    preemption_green_duration: float = 30.0
    tls_emergency_phase: dict = field(default_factory=lambda: {
        "junction": 3,
    })

    # --- Adaptive Phase Tuning ---
    min_green: float = 5.0
    max_green: float = 60.0
    base_green: float = 15.0
    vehicles_per_unit: int = 5

    # --- GPS Lag / Inaccuracy ---
    gps_lag_steps: int = 10
    gps_noise_sigma_m: float = 3.0
    gps_dropout_prob: float = 0.02
