"""
Adaptive Traffic Control with GPS Emergency Preemption
=======================================================
TraCI Bridge: Connects YOLO vehicle detection logic to SUMO simulation.

System Specs:
  - Vision:     YOLOv11 Medium (Primary), YOLOv8 Medium (Secondary)
  - Training:   NVIDIA Titan GPU (real-world + Kaggle datasets)
  - Simulation: SUMO via TraCI

Usage:
  python adaptive_traffic_traci_bridge.py --config path/to/your.sumocfg
  python adaptive_traffic_traci_bridge.py --config path/to/your.sumocfg --gui
"""
from visual_enhancements import set_vehicle_colors, track_emergency_vehicle, preemption_logic, update_gui_schema
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
log = logging.getLogger("TraCI-Bridge")


# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION  ← Tune these to match your network & requirements
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class Config:
    # --- SUMO ---
    sumo_cfg: str = "your_network.sumocfg"
    use_gui: bool = False
    step_length: float = 0.1          # seconds per simulation step

    # --- Emergency Preemption ---
    preemption_radius_m: float = 150.0   # X metres  — trigger threshold
    emergency_vehicle_type: str = "emergency"   # vType id in SUMO routes file
    preemption_green_duration: float = 30.0     # seconds to hold green
    # phase index that gives green to the emergency approach (per TLS id)
    # Phase 3 = state "rrrrrGGGggrrrrrGGGgg" → east_in + west_in GREEN
    # ev0 travels west_in → east_out, so phase 3 is correct.
    tls_emergency_phase: dict = field(default_factory=lambda: {
        "junction": 3,
    })

    # --- Adaptive Phase Tuning ---
    min_green: float = 5.0             # minimum green duration (s)
    max_green: float = 60.0            # maximum green duration (s)
    base_green: float = 15.0           # green per vehicle 'unit'
    vehicles_per_unit: int = 5         # YOLO count threshold per unit

    # --- GPS Lag / Inaccuracy (Part 3) ---
    gps_lag_steps: int = 10            # steps of positional delay
    gps_noise_sigma_m: float = 3.0     # Gaussian noise std-dev in metres
    gps_dropout_prob: float = 0.02     # probability of a full dropout per step


# ──────────────────────────────────────────────────────────────────────────────
# GPS SIMULATION  (Part 3)
# ──────────────────────────────────────────────────────────────────────────────
class GPSSimulator:
    """
    Wraps real SUMO positions with:
      1. Lag    – returns a position from N steps ago (buffer-based).
      2. Noise  – adds Gaussian error in X and Y (metres).
      3. Dropout– occasionally returns None (signal lost).

    Why this matters:
      Real emergency vehicles report GPS at ~1 Hz with 2–5 m CEP accuracy
      and occasional dead-zones (tunnels, tall buildings).  Modelling this
      prevents your preemption logic from being unrealistically omniscient.
    """

    def __init__(self, cfg: Config):
        self.lag = cfg.gps_lag_steps
        self.sigma = cfg.gps_noise_sigma_m
        self.dropout = cfg.gps_dropout_prob
        # vehicle_id → deque of (x, y) history
        self._history: dict[str, list] = {}

    def update(self, vehicle_id: str, true_x: float, true_y: float) -> None:
        """Feed the current true position into the lag buffer."""
        if vehicle_id not in self._history:
            self._history[vehicle_id] = []
        self._history[vehicle_id].append((true_x, true_y))
        # Keep only what we need
        max_history = self.lag + 1
        if len(self._history[vehicle_id]) > max_history:
            self._history[vehicle_id].pop(0)

    def get_reported_position(
        self, vehicle_id: str
    ) -> Optional[tuple[float, float]]:
        """
        Return the 'reported' GPS position for a vehicle, with:
          - dropout  → None
          - lag      → position from N steps ago
          - noise    → Gaussian perturbation
        """
        # 1. Dropout
        if random.random() < self.dropout:
            log.debug("GPS DROPOUT for %s", vehicle_id)
            return None

        history = self._history.get(vehicle_id, [])
        if not history:
            return None

        # 2. Lag  — use oldest available sample up to self.lag steps old
        lag_idx = max(0, len(history) - 1 - self.lag)
        lagged_x, lagged_y = history[lag_idx]

        # 3. Noise
        noisy_x = lagged_x + random.gauss(0, self.sigma)
        noisy_y = lagged_y + random.gauss(0, self.sigma)
        return noisy_x, noisy_y


# ──────────────────────────────────────────────────────────────────────────────
# HELPER UTILITIES
# ──────────────────────────────────────────────────────────────────────────────
def euclidean_distance(pos1: tuple, pos2: tuple) -> float:
    """2-D Euclidean distance between two (x, y) SUMO coordinate pairs."""
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)


def get_tls_junction_position(tls_id: str) -> Optional[tuple[float, float]]:
    """Return the (x, y) centroid of a traffic-light-controlled junction."""
    try:
        controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
        if not controlled_lanes:
            return None
        positions = []
        for lane in controlled_lanes:
            shape = traci.lane.getShape(lane)
            if shape:
                positions.append(shape[-1])  # end-point of the lane
        if not positions:
            return None
        cx = sum(p[0] for p in positions) / len(positions)
        cy = sum(p[1] for p in positions) / len(positions)
        return cx, cy
    except traci.TraCIException:
        return None


# ──────────────────────────────────────────────────────────────────────────────
# PREEMPTION LOGIC  (Part 1 – core requirement)
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
        # tls_id → True if currently preempted
        self._preempted: dict[str, bool] = {}
        # tls_id → step at which preemption started
        self._preempt_start: dict[str, int] = {}

    def step(self, sim_step: int) -> None:
        """Called once per simulation step."""
        # Find all active emergency vehicles
        active_vehicles = [
            v for v in traci.vehicle.getIDList()
            if traci.vehicle.getTypeID(v) == self.cfg.emergency_vehicle_type
        ]

        # Update GPS histories for every emergency vehicle
        for vid in active_vehicles:
            true_pos = traci.vehicle.getPosition(vid)
            self.gps.update(vid, true_pos[0], true_pos[1])

        # Check each TLS
        for tls_id in traci.trafficlight.getIDList():
            junction_pos = get_tls_junction_position(tls_id)
            if junction_pos is None:
                continue

            nearest_ev, nearest_dist = self._nearest_emergency(
                active_vehicles, junction_pos
            )

            already_preempted = self._preempted.get(tls_id, False)

            # ── TRIGGER PREEMPTION ──────────────────────────────────────────
            if nearest_ev is not None and nearest_dist <= self.cfg.preemption_radius_m:
                if not already_preempted:
                    self._activate_preemption(tls_id, nearest_ev, sim_step)

            # ── RELEASE PREEMPTION ──────────────────────────────────────────
            elif already_preempted:
                elapsed = (
                    sim_step - self._preempt_start[tls_id]
                ) * self.cfg.step_length
                if elapsed >= self.cfg.preemption_green_duration:
                    self._release_preemption(tls_id)

    def _nearest_emergency(
        self, vehicles: list[str], junction_pos: tuple
    ) -> tuple[Optional[str], float]:
        """Return the closest emergency vehicle and its *reported* GPS distance."""
        nearest_id = None
        nearest_dist = float("inf")
        for vid in vehicles:
            reported = self.gps.get_reported_position(vid)
            if reported is None:
                continue   # GPS dropout — cannot use this vehicle
            dist = euclidean_distance(reported, junction_pos)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_id = vid
        return nearest_id, nearest_dist

    def _activate_preemption(
        self, tls_id: str, vehicle_id: str, sim_step: int
    ) -> None:
        """Override TLS to green for the emergency vehicle's approach."""
        emergency_phase = self.cfg.tls_emergency_phase.get(tls_id, 0)
        try:
            # Switch to the designated emergency green phase immediately
            traci.trafficlight.setPhase(tls_id, emergency_phase)
            # Lock this phase for the full preemption window
            traci.trafficlight.setPhaseDuration(
                tls_id, self.cfg.preemption_green_duration
            )
            self._preempted[tls_id] = True
            self._preempt_start[tls_id] = sim_step
            log.warning(
                "🚨 PREEMPTION ACTIVATED  TLS=%s  vehicle=%s  phase=%d",
                tls_id, vehicle_id, emergency_phase,
            )
        except traci.TraCIException as exc:
            log.error("Preemption failed for %s: %s", tls_id, exc)

    def _release_preemption(self, tls_id: str) -> None:
        """Restore normal TLS program after preemption window expires."""
        try:
            # Restore the default program so SUMO resumes its normal
            # phase schedule from wherever it was (no flicker / rapid advance).
            traci.trafficlight.setProgram(tls_id, "0")
            self._preempted[tls_id] = False
            log.info("✅ Preemption released for TLS=%s — normal program restored", tls_id)
        except traci.TraCIException as exc:
            log.error("Release failed for %s: %s", tls_id, exc)


# ──────────────────────────────────────────────────────────────────────────────
# ADAPTIVE PHASE CONTROL  (Part 2)
# ──────────────────────────────────────────────────────────────────────────────
class AdaptiveController:
    """
    Maps YOLO vehicle counts → dynamic green phase durations via setPhaseDuration.

    Integration pattern
    ───────────────────
    Your YOLO pipeline runs on a camera frame and returns a dict such as:
        {
          "north": 12,
          "south":  4,
          "east":   7,
          "west":   9,
        }

    This controller translates those counts to adjusted green windows so that
    busier approaches receive proportionally longer service.

    Formula (see compute_green_duration):
        green = clamp(base_green + (count // vehicles_per_unit) * base_green,
                      min_green, max_green)

    BUG FIX (was: stuck TLS)
    ─────────────────────────
    setPhaseDuration() RESTARTS the phase timer from scratch every call.
    Calling it every second means the green phase never expires → TLS freezes.
    Fix: track _last_phase per TLS and only call setPhaseDuration ONCE,
    on the step when the phase first becomes green (i.e., phase index changed).
    """

    def __init__(self, cfg: Config):
        self.cfg = cfg
        # tls_id → last phase index we applied an adaptive duration to
        self._last_phase: dict[str, int] = {}

    def compute_green_duration(self, vehicle_count: int) -> float:
        """
        Convert a single-approach YOLO count to a green duration in seconds.

          count=0  →  min_green  (never cut to zero)
          count=5  →  base_green * 2   (one extra unit)
          count=10 →  base_green * 3   (two extra units)
          ...       capped at max_green
        """
        extra_units = vehicle_count // self.cfg.vehicles_per_unit
        duration = self.cfg.base_green + extra_units * self.cfg.base_green
        return max(self.cfg.min_green, min(self.cfg.max_green, duration))

    def reset_phase_tracking(self, tls_id: str) -> None:
        """Called by EmergencyPreemptor after releasing so next green gets adapted."""
        self._last_phase.pop(tls_id, None)

    def apply(
        self,
        tls_id: str,
        yolo_counts: dict[str, int],
        preemptor: EmergencyPreemptor,
    ) -> None:
        """
        Adjust the CURRENT green phase of `tls_id` based on the approach with
        the highest YOLO count.

        KEY FIX: setPhaseDuration is called ONLY ONCE per phase transition
        (when phase index changes), NOT every second. This prevents the timer
        from being constantly reset, which was causing the TLS to freeze.
        """
        if preemptor._preempted.get(tls_id, False):
            return   # hands-off during emergency preemption

        if not yolo_counts:
            return

        try:
            current_phase = traci.trafficlight.getPhase(tls_id)
            state = traci.trafficlight.getRedYellowGreenState(tls_id)

            # Only act on green phases, and only on the FIRST step of that phase
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
                # Reset tracking on non-green phases so we catch the next green
                self._last_phase.pop(tls_id, None)

        except traci.TraCIException as exc:
            log.error("Adaptive control failed for %s: %s", tls_id, exc)


# ──────────────────────────────────────────────────────────────────────────────
# YOLO INTERFACE  — replace stub with your actual pipeline
# ──────────────────────────────────────────────────────────────────────────────
from yolo_detector import init_yolo, get_yolo_vehicle_counts


# ──────────────────────────────────────────────────────────────────────────────
# MAIN SIMULATION LOOP
# ──────────────────────────────────────────────────────────────────────────────
def _print_diagnostic_table(
    step: int,
    cfg: Config,
    preemptor: EmergencyPreemptor,
    gps_sim: GPSSimulator,
    yolo_cache: dict,
) -> None:
    """
    Prints a live one-line diagnostic row every second when --test is active.
    Columns: step | sim_time | TLS_id | phase | phase_state | preempted | EV_dist | YOLO counts
    """
    sim_time = step * cfg.step_length
    for tls_id in traci.trafficlight.getIDList():
        try:
            phase   = traci.trafficlight.getPhase(tls_id)
            state   = traci.trafficlight.getRedYellowGreenState(tls_id)
            remain  = traci.trafficlight.getNextSwitch(tls_id) - traci.simulation.getTime()
            preempt = "🚨YES" if preemptor._preempted.get(tls_id, False) else " no "
        except traci.TraCIException:
            continue

        # Find nearest EV distance
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
            f"| preempt={preempt} | EV={ev_dist} | YOLO [{counts_str}]"
        )


def run_simulation(cfg: Config, test_mode: bool = False) -> None:
    import os

    # Video/image source for YOLO — place your traffic images in frames/
    # For video : VIDEO_OR_IMAGE_SOURCE = "traffic.mp4"
    # For images: VIDEO_OR_IMAGE_SOURCE = "frames/"   (folder of .jpg/.png files)
    VIDEO_OR_IMAGE_SOURCE = "frames/"

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
    init_yolo("yolo11m.pt", "yolo26m.pt", video_source=VIDEO_OR_IMAGE_SOURCE)

    gps_sim   = GPSSimulator(cfg)
    preemptor = EmergencyPreemptor(cfg, gps_sim)
    adaptive  = AdaptiveController(cfg)

    # Cache latest YOLO counts for diagnostic display
    yolo_cache: dict[str, dict] = {}

    if test_mode:
        print("\n── TEST / DIAGNOSTIC MODE ACTIVE ──────────────────────────────────────────")
        print("  Columns: sim_time | TLS | phase | state(10) | remain | preempted | EV dist | YOLO")
        print("─" * 80)

    step = 0
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            # ── Visual enhancements (GUI-safe) ──────────────────────────────
            set_vehicle_colors()
            track_emergency_vehicle()
            # NOTE: preemption_logic() from visual_enhancements is intentionally
            # NOT called here — EmergencyPreemptor below is the authoritative
            # handler (GPS-aware, per-TLS state machine). Calling both caused
            # conflicts (Bug 2 — fixed).

            has_ev = any(
                traci.vehicle.getTypeID(v) == "emergency"
                for v in traci.vehicle.getIDList()
            )
            update_gui_schema(has_ev)

            # ── 1. Emergency preemption (every step) ────────────────────────
            preemptor.step(step)

            # ── 2. Adaptive control (once per second = camera FPS) ──────────
            steps_per_second = int(1.0 / cfg.step_length)
            if step % steps_per_second == 0:
                for tls_id in traci.trafficlight.getIDList():
                    if not preemptor._preempted.get(tls_id, False):
                        yolo_counts = get_yolo_vehicle_counts(tls_id)
                        yolo_cache[tls_id] = yolo_counts
                        adaptive.apply(tls_id, yolo_counts, preemptor)
                        # NOTE: adaptive.apply() manages _last_phase internally.
                        # reset_phase_tracking() is NOT called here — doing so
                        # every second would clear phase memory and re-trigger
                        # setPhaseDuration every step (the original stuck-TLS bug).

                # ── Diagnostic table (--test flag) ──────────────────────────
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
        description="Adaptive Traffic Control + GPS Emergency Preemption (TraCI)"
    )
    parser.add_argument(
        "--config",
        default="sumo_network/single_junction.sumocfg",
        help="Path to the SUMO .sumocfg file (default: sumo_network/single_junction.sumocfg)",
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
        "--test", action="store_true",
        help="Enable diagnostic table: prints TLS phase, timer, EV distance and YOLO counts every second",
    )
    args = parser.parse_args()

    cfg = Config(
        sumo_cfg=args.config,
        use_gui=args.gui,
        preemption_radius_m=args.radius,
        gps_lag_steps=args.lag,
        gps_noise_sigma_m=args.noise,
    )
    run_simulation(cfg, test_mode=args.test)
