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

    # --- Demand Source / Dynamic Spawning ---
    demand_source: str = "static_xml"      # static_xml | dynamic_python
    dynamic_sumo_cfg: str = "sumo_network/single_junction_dynamic.sumocfg"
    spawn_provider_mode: str = "hardcoded" # hardcoded | random
    spawn_interval_s: float = 10.0
    spawn_max_per_class_per_lane: int = 10

    # --- Indian Vehicle Classes & PCU Mapping ---
    vehicle_classes: list[str] = field(default_factory=lambda: [
        "Hatchback", "Sedan", "SUV", "MUV", "Bus", "Truck",
        "Three-wheeler", "Two-wheeler", "LCV", "Mini-bus",
        "Tempo-traveller", "Bicycle", "Van", "Others"
    ])
    
    pcu_weights: dict[str, float] = field(default_factory=lambda: {
        "Hatchback": 1.0, "Sedan": 1.0, "SUV": 1.0, "MUV": 1.0,
        "Bus": 3.0, "Truck": 3.0, "Three-wheeler": 0.7, "Two-wheeler": 0.5,
        "LCV": 1.0, "Mini-bus": 3.0, "Tempo-traveller": 3.0,
        "Bicycle": 0.5, "Van": 1.0, "Others": 1.0
    })

    # Mapping to SUMO vClass
    sumo_vclass_map: dict[str, str] = field(default_factory=lambda: {
        "Hatchback": "passenger", "Sedan": "passenger", "SUV": "passenger", "MUV": "passenger",
        "Bus": "bus", "Truck": "truck", "Three-wheeler": "moped", "Two-wheeler": "motorcycle",
        "LCV": "truck", "Mini-bus": "bus", "Tempo-traveller": "bus",
        "Bicycle": "bicycle", "Van": "passenger", "Others": "passenger"
    })

    # --- Junction Structure ---
    # Focus on single junction for now
    junction_id: str = "junction"
    approaches: list[str] = field(default_factory=lambda: ["north_in", "south_in", "east_in", "west_in"])
    
    # Paired approaches for signaling (N+S vs E+W)
    paired_approaches: list[tuple[str, str]] = field(default_factory=lambda: [
        ("north_in", "south_in"),
        ("east_in", "west_in")
    ])

    # --- GPS Lag / Inaccuracy ---
    gps_lag_steps: int = 10
    gps_noise_sigma_m: float = 3.0
    gps_dropout_prob: float = 0.02
