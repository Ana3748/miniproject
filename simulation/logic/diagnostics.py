import traci
from simulation.core.config import Config
from simulation.logic.preemption import EmergencyPreemptor
from simulation.core.gps import GPSSimulator
from simulation.core.utils import euclidean_distance, get_tls_junction_position

def _print_diagnostic_table(
    step: int,
    cfg: Config,
    preemptor: EmergencyPreemptor,
    gps_sim: GPSSimulator,
    counts_cache: dict,
    spawn_cache: dict | None = None,
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

        counts = counts_cache.get(tls_id, {})
        counts_str = " ".join(f"{k[0].upper()}:{v}" for k, v in counts.items())
        spawn_stats = (spawn_cache or {}).get(tls_id, {})
        spawn_str = (
            f"spawn[R:{spawn_stats.get('requested', 0)} "
            f"I:{spawn_stats.get('inserted', 0)} "
            f"F:{spawn_stats.get('failed', 0)}]"
        )

        print(
            f"  t={sim_time:7.1f}s | TLS={tls_id} | ph={phase} "
            f"| state={state[:10]}… | remain={remain:5.1f}s "
            f"| preempt={preempt} | EV={ev_dist} | counts [{counts_str}] | {spawn_str}"
        )
