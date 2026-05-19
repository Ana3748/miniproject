import traci
import argparse
import sys
import os
import threading
import logging

# Add parent directory to sys.path to ensure 'simulation' package is findable
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from simulation.core.config import Config
from simulation.core.utils import setup_logging
from simulation.interfaces.providers import VehicleCountProvider
from simulation.logic.preemption import EmergencyPreemptor
from simulation.logic.adaptive import AdaptiveController
from simulation.logic.spawner import DynamicSpawner
from simulation.logic.gui_overlay import VehicleCountOverlay
from vision.single_frame import get_yolo_counts

log = setup_logging("TraCI-Bridge-Modular")

def keyboard_listener(preemptor: EmergencyPreemptor, cfg: Config):
    """
    Listens for 'n', 's', 'e', 'w' in the terminal to spawn ambulances.
    """
    print("\n" + "="*45)
    print(" 🚑 EMERGENCY ON-DEMAND SYSTEM READY")
    print(" Type 'n', 's', 'e', or 'w' and press ENTER")
    print(" to spawn an ambulance from that direction.")
    print("="*45 + "\n")
    
    direction_map = {'n': 'north', 's': 'south', 'e': 'east', 'w': 'west'}
    
    while True:
        try:
            line = sys.stdin.readline().strip().lower()
            if not line:
                continue
                
            for char in line:
                if char in direction_map:
                    dir_name = direction_map[char]
                    approach = f"{dir_name}_in"
                    veh_id = f"EMERGENCY_{dir_name.upper()}_{int(traci.simulation.getTime())}"
                    
                    try:
                        # Ensure emergency type exists
                        if cfg.emergency_vehicle_type not in traci.vehicletype.getIDList():
                            traci.vehicletype.copy("car", cfg.emergency_vehicle_type)
                            traci.vehicletype.setColor(cfg.emergency_vehicle_type, (255, 0, 0, 255))
                            traci.vehicletype.setShapeClass(cfg.emergency_vehicle_type, "emergency")

                        traci.vehicle.add(
                            vehID=veh_id,
                            routeID=f"rt_{dir_name}_straight",
                            typeID=cfg.emergency_vehicle_type,
                            depart="now",
                            departLane="best",
                            departSpeed="max",
                        )
                        preemptor.add_emergency(veh_id)
                        log.warning("Spawned emergency vehicle: %s", veh_id)
                    except traci.TraCIException as e:
                        log.error("Failed to spawn emergency on %s: %s", approach, e)
        except EOFError:
            break

def run_simulation(cfg: Config, yolo_payload: dict = None) -> None:
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

    count_provider = VehicleCountProvider(
        mode=cfg.spawn_provider_mode,
        yolo_payload=yolo_payload
    )
    spawner = DynamicSpawner(cfg) if dynamic_enabled else None
    preemptor = EmergencyPreemptor(cfg)
    adaptive  = AdaptiveController(cfg)
    
    # Start keyboard listener for on-demand emergencies
    input_thread = threading.Thread(target=keyboard_listener, args=(preemptor, cfg), daemon=True)
    input_thread.start()

    # Setup GUI Overlay
    gui_overlay = VehicleCountOverlay(junction_id=cfg.junction_id) if cfg.use_gui else None
    
    step = 0
    try:
        # Loop indefinitely to allow on-demand spawning even if net is empty
        while True:
            try:
                traci.simulationStep()
            except traci.FatalTraCIError:
                break
            except traci.TraCIException as e:
                if "connection closed" in str(e).lower() or "not connected" in str(e).lower():
                    break
                raise e

            # 1. Update Preemption State
            preemptor.step(step)
            is_preempted = preemptor.is_preempted(cfg.junction_id)

            # 2. Update Adaptive Control & GUI
            # Note: We update metrics every step for smooth GUI, but adaptive logic 
            # might have its own internal timing.
            metrics = adaptive.get_live_metrics()
            if gui_overlay:
                gui_overlay.update_metrics(metrics)

            adaptive.apply(cfg.junction_id, is_preempted, step)

            # 3. Dynamic Spawning (if enabled)
            steps_per_spawn = max(1, int(cfg.spawn_interval_s / cfg.step_length))
            if dynamic_enabled and spawner is not None and step % steps_per_spawn == 0:
                lane_class_counts = count_provider.get_lane_class_counts(cfg.junction_id)
                spawner.spawn_for_tls(cfg.junction_id, lane_class_counts, step)

            step += 1

    except KeyboardInterrupt:
        log.info("Simulation interrupted by user.")
    except Exception as e:
        log.error("Simulation error: %s", e, exc_info=True)
    finally:
        try:
            traci.close()
        except:
            pass
        log.info("Simulation complete. Total steps: %d", step)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Indian Traffic System: Live PCU Control + On-Demand Preemption"
    )
    parser.add_argument("--gui", action="store_true", help="Launch sumo-gui")
    parser.add_argument("--config", default="sumo_network/single_junction.sumocfg", help="SUMO config")
    parser.add_argument("--dynamic", action="store_true", help="Enable dynamic python spawning")
    parser.add_argument("--yolo", action="store_true", help="Use YOLO to detect initial traffic demand")
    args = parser.parse_args()

    # Create config based on arguments
    cfg = Config(
        sumo_cfg=args.config,
        use_gui=args.gui,
        demand_source="dynamic_python" if (args.dynamic or args.yolo) else "static_xml",
        spawn_provider_mode="yolo" if args.yolo else "hardcoded"
    )
    
    yolo_payload = None
    if args.yolo:
        yolo_payload = get_yolo_counts()
    
    run_simulation(cfg, yolo_payload=yolo_payload)
