import traci
import logging
from simulation.core.config import Config

log = logging.getLogger("TraCI-Bridge-Backup")

class EmergencyPreemptor:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._queue: list[str] = [] # List of active emergency vehicle IDs
        self._current_tls_preempted: dict[str, str] = {} # tls_id -> veh_id
        
    def add_emergency(self, veh_id: str):
        """Adds a new emergency vehicle to the FCFS queue."""
        if veh_id not in self._queue:
            self._queue.append(veh_id)
            log.info("🚨 Emergency vehicle %s added to FCFS queue", veh_id)

    def is_preempted(self, tls_id: str) -> bool:
        """Returns True if the traffic light is currently under emergency preemption."""
        return tls_id in self._current_tls_preempted

    def step(self, sim_step: int) -> None:
        """
        Main step logic for handling the emergency queue and releasing preemption.
        """
        all_vehs = traci.vehicle.getIDList()
        
        # Cleanup queue for vehicles that might have been removed from simulation
        self._queue = [vid for vid in self._queue if vid in all_vehs]
        
        tls_id = self.cfg.junction_id
        
        # 1. Check if the current preempting vehicle has passed the junction
        if tls_id in self._current_tls_preempted:
            target_veh = self._current_tls_preempted[tls_id]
            
            if target_veh not in all_vehs:
                self._release_preemption(tls_id)
            else:
                try:
                    curr_edge = traci.vehicle.getRoadID(target_veh)
                    # Release as soon as it enters an outgoing edge
                    if curr_edge.endswith("_out"):
                        self._release_preemption(tls_id)
                except traci.TraCIException:
                    self._release_preemption(tls_id)
        
        # 2. If no preemption is active but the queue has vehicles, activate for the first one
        if tls_id not in self._current_tls_preempted and self._queue:
            next_veh = self._queue[0]
            self._activate_preemption(tls_id, next_veh)

    def _activate_preemption(self, tls_id: str, vehicle_id: str) -> None:
        """Forces the signal to green for the emergency vehicle's approach."""
        try:
            edge_id = traci.vehicle.getRoadID(vehicle_id)
            
            # Determine correct green phase based on approach direction
            if "north" in edge_id or "south" in edge_id:
                emergency_phase = 0 # N+S Green
            elif "east" in edge_id or "west" in edge_id:
                emergency_phase = 3 # E+W Green
            else:
                emergency_phase = 0 # Fallback
                
            traci.trafficlight.setPhase(tls_id, emergency_phase)
            # Use a very long duration; we will manually release when it passes
            traci.trafficlight.setPhaseDuration(tls_id, 9999) 
            
            self._current_tls_preempted[tls_id] = vehicle_id
            log.warning(
                "🚨 PREEMPTION ACTIVATED  TLS=%s  vehicle=%s  phase=%d",
                tls_id, vehicle_id, emergency_phase,
            )
        except traci.TraCIException as exc:
            log.error("Preemption failed for %s: %s", tls_id, exc)

    def _release_preemption(self, tls_id: str) -> None:
        """Releases the preemption and restores normal adaptive control."""
        try:
            veh_id = self._current_tls_preempted.pop(tls_id, None)
            if veh_id and veh_id in self._queue:
                self._queue.remove(veh_id)
            
            # Restore normal program timing
            traci.trafficlight.setPhaseDuration(tls_id, self.cfg.base_green)
            log.info("✅ Preemption released for TLS=%s — vehicle %s passed junction", tls_id, veh_id)
        except traci.TraCIException as exc:
            log.error("Release failed for %s: %s", tls_id, exc)
