import traci
import logging
from simulation.core.config import Config
from simulation.logic.preemption import EmergencyPreemptor

log = logging.getLogger("TraCI-Bridge-Backup")

class AdaptiveController:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._last_phase: dict[str, int] = {}

    def compute_green_duration(self, vehicle_count: int) -> float:
        extra_units = vehicle_count // self.cfg.vehicles_per_unit
        duration = self.cfg.base_green + extra_units * self.cfg.base_green
        return max(self.cfg.min_green, min(self.cfg.max_green, duration))

    def reset_phase_tracking(self, tls_id: str) -> None:
        self._last_phase.pop(tls_id, None)

    def apply(self, tls_id: str, vehicle_counts: dict[str, int], preemptor: EmergencyPreemptor) -> None:
        if preemptor._preempted.get(tls_id, False):
            return
        if not vehicle_counts:
            return
        try:
            current_phase = traci.trafficlight.getPhase(tls_id)
            state = traci.trafficlight.getRedYellowGreenState(tls_id)
            is_green = "G" in state or "g" in state
            phase_just_changed = self._last_phase.get(tls_id, -1) != current_phase

            if is_green and phase_just_changed:
                busiest_approach = max(vehicle_counts, key=vehicle_counts.get)
                count = vehicle_counts[busiest_approach]
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
