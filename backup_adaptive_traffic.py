"""
Adaptive Traffic Control with GPS Emergency Preemption
=======================================================
BACKUP / STANDALONE SCRIPT
  — Runs the full adaptive + preemption algorithm WITHOUT requiring YOLO.
  — By default uses RANDOM vehicle counts (great for testing the logic).
  — Pass --yolo to enable real YOLO-based vehicle detection.

System Specs:
  - Vision:     YOLOv11 Medium (Primary), YOLOv8 Medium (Secondary)  [optional]
  - Simulation: SUMO via TraCI

Usage:
  # Run with random counts (no YOLO needed):
  python backup_adaptive_traffic.py --config path/to/your.sumocfg

  # Run with real YOLO detection:
  python backup_adaptive_traffic.py --config path/to/your.sumocfg --yolo

  # With GUI:
  python backup_adaptive_traffic.py --config path/to/your.sumocfg --gui
"""
import traci
import traci.constants as tc
import math
import random
import time
import argparse
import logging
from dataclasses import dataclass, field
from typing import Optional

# ──────────────────────────────────────────────────────────────────────────────
# LOGGING
# ──────────────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("TraCI-Bridge-Backup")


# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class Config:
    # --- SUMO ---
    sumo_cfg: str = "your_network.sumocfg"
    use_gui: bool = False
    step_length: float = 0.1          # seconds per simulation step

    # --- Emergency Preemption ---
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

    # --- YOLO Toggle ---
    use_yolo: bool = False            # Set via --yolo flag at runtime


# ──────────────────────────────────────────────────────────────────────────────
# VEHICLE COUNT PROVIDER
# Abstracts YOLO vs random — the rest of the code never needs to know which.
# ──────────────────────────────────────────────────────────────────────────────
class VehicleCountProvider:
    """
    Provides per-approach vehicle counts to the adaptive controller.

    Modes:
      - RANDOM (default): returns random counts per direction, no dependencies.
      - YOLO  (--yolo):   delegates to yolo_detector.get_yolo_vehicle_counts().
    """

    APPROACHES = ["north", "south", "east", "west"]

    def __init__(self, use_yolo: bool):
        self.use_yolo = use_yolo
        self._yolo_init = False
        self._get_counts_fn = None

        if use_yolo:
            try:
                from yolo_detector import init_yolo, get_yolo_vehicle_counts
                init_yolo("models/yolo11m.pt", "models/yolo26m.pt", video_source="frames/")
                self._get_counts_fn = get_yolo_vehicle_counts
                self._yolo_init = True
                log.info("✅ YOLO initialised — using real vehicle counts.")
            except Exception as exc:
                log.error("❌ YOLO failed to initialise (%s). Falling back to random counts.", exc)
                self.use_yolo = False

        if not self.use_yolo:
            log.info("🎲 YOLO disabled — using RANDOM vehicle counts.")

    def get_counts(self, tls_id: str) -> dict[str, int]:
        """
        Returns a dict like {"north": 7, "south": 2, "east": 5, "west": 9}.
        Uses YOLO if initialised, else random counts.
        """
        if self.use_yolo and self._get_counts_fn:
            try:
                return self._get_counts_fn(tls_id)
            except Exception as exc:
                log.warning("YOLO count failed (%s), using random fallback.", exc)

        # ── Random fallback ──────────────────────────────────────────────────
        return {direction: random.randint(0, 20) for direction in self.APPROACHES}


# ──────────────────────────────────────────────────────────────────────────────
# GPS SIMULATION
# ──────────────────────────────────────────────────────────────────────────────
class GPSSimulator:
    """
    Wraps real SUMO positions with:
      1. Lag    – returns a position from N steps ago.
      2. Noise  – adds Gaussian error in X and Y (metres).
      3. Dropout– occasionally returns None (signal lost).
    """

    def __init__(self, cfg: Config):
        self.lag = cfg.gps_lag_steps
        self.sigma = cfg.gps_noise_sigma_m
        self.dropout = cfg.gps_dropout_prob
        self._history: dict[str, list] = {}

    def update(self, vehicle_id: str, true_x: float, true_y: float) -> None:
        if vehicle_id not in self._history:
            self._history[vehicle_id] = []
        self._history[vehicle_id].append((true_x, true_y))
        max_history = self.lag + 1
        if len(self._history[vehicle_id]) > max_history:
            self._history[vehicle_id].pop(0)

    def get_reported_position(self, vehicle_id: str) -> Optional[tuple[float, float]]:
        if random.random() < self.dropout:
            log.debug("GPS DROPOUT for %s", vehicle_id)
            return None
        history = self._history.get(vehicle_id, [])
        if not history:
            return None
        lag_idx = max(0, len(history) - 1 - self.lag)
        lagged_x, lagged_y = history[lag_idx]
        noisy_x = lagged_x + random.gauss(0, self.sigma)
        noisy_y = lagged_y + random.gauss(0, self.sigma)
        return noisy_x, noisy_y


# ──────────────────────────────────────────────────────────────────────────────
# HELPER UTILITIES
# ──────────────────────────────────────────────────────────────────────────────
def euclidean_distance(pos1: tuple, pos2: tuple) -> float:
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)


def get_tls_junction_position(tls_id: str) -> Optional[tuple[float, float]]:
    try:
        controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
        if not controlled_lanes:
            return None
        positions = []
        for lane in controlled_lanes:
            shape = traci.lane.getShape(lane)
            if shape:
                positions.append(shape[-1])
        if not positions:
            return None
        cx = sum(p[0] for p in positions) / len(positions)
        cy = sum(p[1] for p in positions) / len(positions)
        return cx, cy
    except traci.TraCIException:
        return None


# ──────────────────────────────────────────────────────────────────────────────
# PREEMPTION LOGIC
# ──────────────────────────────────────────────────────────────────────────────
class EmergencyPreemptor:
    """
    Monitors emergency vehicles and overrides TLS phases when they are within
    `preemption_radius_m` of a controlled junction.

    State machine per TLS:
        NORMAL  →  PREEMPTED  →  NORMAL
    """

    def __init__(self, cfg: Config, gps_sim: GPSSimulator):
        self.cfg = cfg
        self.gps = gps_sim
        self._preempted: dict[str, bool] = {}
        self._preempt_start: dict[str, int] = {}

    def step(self, sim_step: int) -> None:
        active_vehicles = [
            v for v in traci.vehicle.getIDList()
            if traci.vehicle.getTypeID(v) == self.cfg.emergency_vehicle_type
        ]
        for vid in active_vehicles:
            true_pos = traci.vehicle.getPosition(vid)
            self.gps.update(vid, true_pos[0], true_pos[1])

        for tls_id in traci.trafficlight.getIDList():
            junction_pos = get_tls_junction_position(tls_id)
            if junction_pos is None:
                continue

            nearest_ev, nearest_dist = self._nearest_emergency(active_vehicles, junction_pos)
            already_preempted = self._preempted.get(tls_id, False)

            if nearest_ev is not None and nearest_dist <= self.cfg.preemption_radius_m:
                if not already_preempted:
                    self._activate_preemption(tls_id, nearest_ev, sim_step)
            elif already_preempted:
                elapsed = (sim_step - self._preempt_start[tls_id]) * self.cfg.step_length
                if elapsed >= self.cfg.preemption_green_duration:
                    self._release_preemption(tls_id)

    def _nearest_emergency(self, vehicles: list[str], junction_pos: tuple) -> tuple[Optional[str], float]:
        nearest_id = None
        nearest_dist = float("inf")
        for vid in vehicles:
            reported = self.gps.get_reported_position(vid)
            if reported is None:
                continue
            dist = euclidean_distance(reported, junction_pos)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_id = vid
        return nearest_id, nearest_dist

    def _activate_preemption(self, tls_id: str, vehicle_id: str, sim_step: int) -> None:
        emergency_phase = self.cfg.tls_emergency_phase.get(tls_id, 0)
        try:
            traci.trafficlight.setPhase(tls_id, emergency_phase)
            traci.trafficlight.setPhaseDuration(tls_id, self.cfg.preemption_green_duration)
            self._preempted[tls_id] = True
            self._preempt_start[tls_id] = sim_step
            log.warning(
                "🚨 PREEMPTION ACTIVATED  TLS=%s  vehicle=%s  phase=%d",
                tls_id, vehicle_id, emergency_phase,
            )
        except traci.TraCIException as exc:
            log.error("Preemption failed for %s: %s", tls_id, exc)

    def _release_preemption(self, tls_id: str) -> None:
        try:
            traci.trafficlight.setProgram(tls_id, "0")
            self._preempted[tls_id] = False
            log.info("✅ Preemption released for TLS=%s — normal program restored", tls_id)
        except traci.TraCIException as exc:
            log.error("Release failed for %s: %s", tls_id, exc)


# ──────────────────────────────────────────────────────────────────────────────
# ADAPTIVE PHASE CONTROL
# ──────────────────────────────────────────────────────────────────────────────
class AdaptiveController:
    """
    Maps vehicle counts → dynamic green phase durations via setPhaseDuration.

    KEY FIX: setPhaseDuration is called ONLY ONCE per phase transition to
    prevent the timer constantly resetting (the old stuck-TLS bug).
    """

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._last_phase: dict[str, int] = {}

    def compute_green_duration(self, vehicle_count: int) -> float:
        extra_units = vehicle_count // self.cfg.vehicles_per_unit
        duration = self.cfg.base_green + extra_units * self.cfg.base_green
        return max(self.cfg.min_green, min(self.cfg.max_green, duration))

    def reset_phase_tracking(self, tls_id: str) -> None:
        self._last_phase.pop(tls_id, None)

    def apply(self, tls_id: str, yolo_counts: dict[str, int], preemptor: EmergencyPreemptor) -> None:
        if preemptor._preempted.get(tls_id, False):
            return
        if not yolo_counts:
            return
        try:
            current_phase = traci.trafficlight.getPhase(tls_id)
            state = traci.trafficlight.getRedYellowGreenState(tls_id)
            is_green = "G" in state or "g" in state
            phase_just_changed = self._last_phase.get(tls_id, -1) != current_phase

            if is_green and phase_just_changed:
                busiest_approach = max(yolo_counts, key=yolo_counts.get)
                count = yolo_counts[busiest_approach]
                new_duration = self.compute_green_duration(count)
                traci.trafficlight.setPhaseDuration(tls_id, new_duration)
                self._last_phase[tls_id] = current_phase
                log.info(
                    "Adaptive TLS=%s phase=%d approach=%s count=%d → %.1fs green",
                    tls_id, current_phase, busiest_approach, count, new_duration,
                )
            elif not is_green:
                self._last_phase.pop(tls_id, None)

        except traci.TraCIException as exc:
            log.error("Adaptive control failed for %s: %s", tls_id, exc)


# ──────────────────────────────────────────────────────────────────────────────
# DIAGNOSTIC TABLE
# ──────────────────────────────────────────────────────────────────────────────
def _print_diagnostic_table(
    step: int,
    cfg: Config,
    preemptor: EmergencyPreemptor,
    gps_sim: GPSSimulator,
    yolo_cache: dict,
) -> None:
    sim_time = step * cfg.step_length
    for tls_id in traci.trafficlight.getIDList():
        try:
            phase   = traci.trafficlight.getPhase(tls_id)
            state   = traci.trafficlight.getRedYellowGreenState(tls_id)
            remain  = traci.trafficlight.getNextSwitch(tls_id) - traci.simulation.getTime()
            preempt = "🚨YES" if preemptor._preempted.get(tls_id, False) else " no "
        except traci.TraCIException:
            continue

        ev_dist = "---"
        for vid in traci.vehicle.getIDList():
            if traci.vehicle.getTypeID(vid) == cfg.emergency_vehicle_type:
                reported = gps_sim.get_reported_position(vid)
                if reported:
                    jpos = get_tls_junction_position(tls_id)
                    if jpos:
                        ev_dist = f"{euclidean_distance(reported, jpos):6.1f}m"

        counts = yolo_cache.get(tls_id, {})
        counts_str = " ".join(f"{k[0].upper()}:{v}" for k, v in counts.items())

        print(
            f"  t={sim_time:7.1f}s | TLS={tls_id} | ph={phase} "
            f"| state={state[:10]}… | remain={remain:5.1f}s "
            f"| preempt={preempt} | EV={ev_dist} | counts [{counts_str}]"
        )


# ──────────────────────────────────────────────────────────────────────────────
# MAIN SIMULATION LOOP
# ──────────────────────────────────────────────────────────────────────────────
def run_simulation(cfg: Config, test_mode: bool = False) -> None:
    binary = "sumo-gui" if cfg.use_gui else "sumo"
    sumo_cmd = [
        binary,
        "-c", cfg.sumo_cfg,
        "--step-length", str(cfg.step_length),
        "--no-warnings",
        "--collision.action", "warn",
    ]

    log.info("Starting SUMO: %s", " ".join(sumo_cmd))
    traci.start(sumo_cmd)

    # Vehicle count provider — YOLO or random, decided at startup
    count_provider = VehicleCountProvider(use_yolo=cfg.use_yolo)

    gps_sim   = GPSSimulator(cfg)
    preemptor = EmergencyPreemptor(cfg, gps_sim)
    adaptive  = AdaptiveController(cfg)

    yolo_cache: dict[str, dict] = {}

    if test_mode:
        mode_label = "YOLO" if cfg.use_yolo else "RANDOM COUNTS"
        print(f"\n── TEST / DIAGNOSTIC MODE ACTIVE  [{mode_label}] ──────────────────────────────")
        print("  Columns: sim_time | TLS | phase | state(10) | remain | preempted | EV dist | counts")
        print("─" * 80)

    step = 0
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            has_ev = any(
                traci.vehicle.getTypeID(v) == "emergency"
                for v in traci.vehicle.getIDList()
            )

            # ── 1. Emergency preemption (every step) ────────────────────────
            preemptor.step(step)

            # ── 2. Adaptive control (once per second) ───────────────────────
            steps_per_second = int(1.0 / cfg.step_length)
            if step % steps_per_second == 0:
                for tls_id in traci.trafficlight.getIDList():
                    if not preemptor._preempted.get(tls_id, False):
                        counts = count_provider.get_counts(tls_id)
                        yolo_cache[tls_id] = counts
                        adaptive.apply(tls_id, counts, preemptor)

                if test_mode:
                    _print_diagnostic_table(step, cfg, preemptor, gps_sim, yolo_cache)

            # ── 3. EV GPS debug logging ─────────────────────────────────────
            for vid in traci.vehicle.getIDList():
                if traci.vehicle.getTypeID(vid) == cfg.emergency_vehicle_type:
                    true_pos  = traci.vehicle.getPosition(vid)
                    speed_mps = traci.vehicle.getSpeed(vid)
                    reported  = gps_sim.get_reported_position(vid)
                    if reported:
                        error_m = euclidean_distance(true_pos, reported)
                        log.debug(
                            "EV %s | true=(%.1f,%.1f) reported=(%.1f,%.1f)"
                            " err=%.2fm speed=%.1fm/s",
                            vid, true_pos[0], true_pos[1],
                            reported[0], reported[1], error_m, speed_mps,
                        )

            step += 1

    except KeyboardInterrupt:
        log.info("Simulation interrupted by user.")
    finally:
        traci.close()
        log.info("Simulation complete. Total steps: %d", step)


# ──────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Adaptive Traffic Control + GPS Emergency Preemption (Standalone Backup)"
    )
    parser.add_argument(
        "--config",
        default="sumo_network/single_junction.sumocfg",
        help="Path to the SUMO .sumocfg file",
    )
    parser.add_argument(
        "--gui", action="store_true",
        help="Launch sumo-gui instead of headless sumo",
    )
    parser.add_argument(
        "--radius", type=float, default=150.0,
        help="Emergency preemption trigger radius in metres (default: 150)",
    )
    parser.add_argument(
        "--lag", type=int, default=10,
        help="GPS lag in simulation steps (default: 10)",
    )
    parser.add_argument(
        "--noise", type=float, default=3.0,
        help="GPS Gaussian noise std-dev in metres (default: 3.0)",
    )
    parser.add_argument(
        "--yolo", action="store_true",
        help="Enable YOLO-based vehicle detection (default: random counts)",
    )
    parser.add_argument(
        "--test", action="store_true",
        help="Enable diagnostic table output every second",
    )
    args = parser.parse_args()

    cfg = Config(
        sumo_cfg=args.config,
        use_gui=args.gui,
        preemption_radius_m=args.radius,
        gps_lag_steps=args.lag,
        gps_noise_sigma_m=args.noise,
        use_yolo=args.yolo,
    )
    run_simulation(cfg, test_mode=args.test)
