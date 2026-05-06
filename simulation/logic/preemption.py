import traci
import logging
from typing import Optional
from simulation.core.config import Config
from simulation.core.gps import GPSSimulator
from simulation.core.utils import euclidean_distance, get_tls_junction_position

log = logging.getLogger("TraCI-Bridge-Backup")

class EmergencyPreemptor:
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
        try:
            edge_id = traci.vehicle.getRoadID(vehicle_id)
            if edge_id in ["north_in", "south_in"]:
                emergency_phase = 0
            elif edge_id in ["east_in", "west_in"]:
                emergency_phase = 3
            else:
                emergency_phase = self.cfg.tls_emergency_phase.get(tls_id, 0)
                
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
