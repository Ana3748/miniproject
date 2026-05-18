import traci
import argparse
import sys
import os

# Add parent directory to sys.path to ensure 'simulation' package is findable
# if running directly as 'python simulation/runner.py'
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from simulation.core.config import Config
from simulation.core.utils import setup_logging
from simulation.core.gps import GPSSimulator
from simulation.interfaces.providers import VehicleCountProvider
from simulation.logic.preemption import EmergencyPreemptor
from simulation.logic.adaptive import AdaptiveController
from simulation.logic.spawner import DynamicSpawner
from simulation.logic.diagnostics import _print_diagnostic_table
from simulation.logic.gui_overlay import VehicleCountOverlay

log = setup_logging("TraCI-Bridge-Modular")

def run_simulation(cfg: Config, test_mode: bool = False) -> None:
    active_sumo_cfg = cfg.sumo_cfg
    dynamic_enabled = cfg.demand_source == "dynamic_python"
    if dynamic_enabled:
        active_sumo_cfg = cfg.dynamic_sumo_cfg

    binary = "sumo-gui" if cfg.use_gui else "sumo"
    sumo_cmd = [
        binary,
        "-c", active_sumo_cfg,
        "--step-length", str(cfg.step_length),
        "--no-warnings",
        "--collision.action", "warn",
    ]

    log.info("Starting SUMO: %s", " ".join(sumo_cmd))
    traci.start(sumo_cmd)

    count_provider = VehicleCountProvider(mode=cfg.spawn_provider_mode)
    spawner = DynamicSpawner(cfg) if dynamic_enabled else None

    gps_sim   = GPSSimulator(cfg)
    preemptor = EmergencyPreemptor(cfg, gps_sim)
    adaptive  = AdaptiveController(cfg)

    # Cache latest counts for diagnostic display
    counts_cache: dict[str, dict] = {}
    spawn_cache: dict[str, dict[str, int]] = {}
    
    # Setup GUI Overlay for counts if using GUI
    gui_overlay = VehicleCountOverlay(junction_id="junction") if cfg.use_gui else None
    

    if test_mode:
        print("\n── TEST / DIAGNOSTIC MODE ACTIVE ──────────────────────────────────────────")
        print("  Columns: sim_time | TLS | phase | state(10) | remain | preempted | EV dist | counts")
        print("─" * 80)

    step = 0
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            if gui_overlay is not None:
                gui_overlay.update_counts(count_provider.get_total_counts())

            # ── 1. Emergency preemption (optional) ──────────────────────────
            if cfg.use_emergency:
                has_ev = False
                for vid in traci.vehicle.getIDList():
                    if traci.vehicle.getTypeID(vid) == cfg.emergency_vehicle_type:
                        has_ev = True
                        if cfg.use_gui:
                            try:
                                # Visual highlight only, no zoom/lock
                                traci.vehicle.setColor(vid, (255, 0, 0, 255))
                            except traci.TraCIException:
                                pass
                        break
                preemptor.step(step)

            # ── 2. Adaptive control (once per second) ───────────────────────
            steps_per_second = max(1, int(1.0 / cfg.step_length))
            steps_per_spawn = max(1, int(cfg.spawn_interval_s / cfg.step_length))
            if step % steps_per_second == 0:
                for tls_id in traci.trafficlight.getIDList():
                    if not preemptor._preempted.get(tls_id, False):
                        lane_class_counts = count_provider.get_lane_class_counts(tls_id)
                        counts = count_provider.totals_from_lane_class_counts(lane_class_counts)
                        counts_cache[tls_id] = counts
                        adaptive.apply(tls_id, counts, preemptor)

                        if dynamic_enabled and spawner is not None and step % steps_per_spawn == 0:
                            spawn_stats = spawner.spawn_for_tls(tls_id, lane_class_counts, step)
                            spawn_cache[tls_id] = spawn_stats

                if test_mode:
                    _print_diagnostic_table(step, cfg, preemptor, gps_sim, counts_cache, spawn_cache)

            # ── 3. EV GPS debug logging ─────────────────────────────────────
            if cfg.use_emergency:
                for vid in traci.vehicle.getIDList():
                    if traci.vehicle.getTypeID(vid) == cfg.emergency_vehicle_type:
                        true_pos  = traci.vehicle.getPosition(vid)
                        speed_mps = traci.vehicle.getSpeed(vid)
                        reported  = gps_sim.get_reported_position(vid)
                        if reported:
                            error_m = ((true_pos[0]-reported[0])**2 + (true_pos[1]-reported[1])**2)**0.5
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Adaptive Traffic Control + GPS Emergency Preemption (Modularized)"
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
        "--emergency", action="store_true",
        help="Enable emergency vehicle preemption logic",
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
        help="Enable diagnostic table output every second",
    )
    parser.add_argument(
        "--demand-source",
        choices=["static_xml", "dynamic_python"],
        default="static_xml",
        help="Traffic demand source mode",
    )
    parser.add_argument(
        "--provider-mode",
        choices=["hardcoded", "random"],
        default="hardcoded",
        help="Input provider mode for dynamic spawning",
    )
    parser.add_argument(
        "--dynamic-config",
        default="sumo_network/single_junction_dynamic.sumocfg",
        help="SUMO config used when demand source is dynamic_python",
    )
    parser.add_argument(
        "--spawn-interval",
        type=float,
        default=1.0,
        help="Dynamic spawn interval in seconds",
    )
    parser.add_argument(
        "--spawn-cap",
        type=int,
        default=30,
        help="Max vehicles per class per lane per spawn tick",
    )
    args = parser.parse_args()

    cfg = Config(
        sumo_cfg=args.config,
        use_gui=args.gui,
        use_emergency=args.emergency,
        preemption_radius_m=args.radius,
        gps_lag_steps=args.lag,
        gps_noise_sigma_m=args.noise,
        demand_source=args.demand_source,
        dynamic_sumo_cfg=args.dynamic_config,
        spawn_provider_mode=args.provider_mode,
        spawn_interval_s=args.spawn_interval,
        spawn_max_per_class_per_lane=args.spawn_cap,
    )
    run_simulation(cfg, test_mode=args.test)
