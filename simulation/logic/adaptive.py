import traci
import logging
from simulation.core.config import Config

log = logging.getLogger("TraCI-Bridge-Backup")

class AdaptiveController:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._last_phase_change_step = 0
        self.live_metrics = {}

    def get_live_metrics(self) -> dict:
        """
        Queries TraCI for live vehicle counts and calculates PCU per approach.
        """
        metrics = {}
        for approach in self.cfg.approaches:
            try:
                veh_ids = traci.edge.getLastStepVehicleIDs(approach)
            except traci.TraCIException:
                veh_ids = []
                
            counts = {cls: 0 for cls in self.cfg.vehicle_classes}
            total_pcu = 0.0
            
            for vid in veh_ids:
                try:
                    type_id = traci.vehicle.getTypeID(vid)
                except traci.TraCIException:
                    continue
                    
                # Map safe type_id back to actual class name
                matched_cls = None
                for actual_cls in self.cfg.vehicle_classes:
                    if actual_cls.replace(" ", "") == type_id:
                        matched_cls = actual_cls
                        break
                
                if matched_cls:
                    counts[matched_cls] += 1
                    total_pcu += self.cfg.pcu_weights.get(matched_cls, 1.0)
                else:
                    # Fallback for emergency or unknown vehicles
                    total_pcu += 1.0
            
            metrics[approach] = {
                "counts": counts,
                "pcu": total_pcu,
                "total": len(veh_ids)
            }
        self.live_metrics = metrics
        return metrics

    def apply(self, tls_id: str, preempted: bool, step: int) -> None:
        """
        Main logic for switching signals based on PCU density.
        """
        if preempted:
            return

        metrics = self.get_live_metrics()
        
        # NS Pair: north_in + south_in
        ns_pcu = metrics.get("north_in", {}).get("pcu", 0) + metrics.get("south_in", {}).get("pcu", 0)
        # EW Pair: east_in + west_in
        ew_pcu = metrics.get("east_in", {}).get("pcu", 0) + metrics.get("west_in", {}).get("pcu", 0)
        
        try:
            current_phase = traci.trafficlight.getPhase(tls_id)
            # Standard transition logic: 
            # Phase 0: NS Green
            # Phase 1: NS Yellow
            # Phase 2: Red
            # Phase 3: EW Green
            # Phase 4: EW Yellow
            # Phase 5: Red
            
            elapsed = (step - self._last_phase_change_step) * self.cfg.step_length
            
            if current_phase == 0: # NS is Green
                # Switch if NS is empty and EW is not, OR EW higher and min_green passed
                if (ns_pcu == 0 and ew_pcu > 0) or (ew_pcu > ns_pcu and elapsed >= self.cfg.min_green):
                    traci.trafficlight.setPhase(tls_id, 1) # Switch to Yellow -> EW
                    self._last_phase_change_step = step
                    log.info("Adaptive: Switching NS -> EW | NS_PCU=%.1f EW_PCU=%.1f", ns_pcu, ew_pcu)
            
            elif current_phase == 3: # EW is Green
                # Switch if EW is empty and NS is not, OR NS higher and min_green passed
                if (ew_pcu == 0 and ns_pcu > 0) or (ns_pcu > ew_pcu and elapsed >= self.cfg.min_green):
                    traci.trafficlight.setPhase(tls_id, 4) # Switch to Yellow -> NS
                    self._last_phase_change_step = step
                    log.info("Adaptive: Switching EW -> NS | EW_PCU=%.1f NS_PCU=%.1f", ew_pcu, ns_pcu)
            
            # Reset timer if we just transitioned to a main green phase
            if current_phase in [0, 3] and traci.trafficlight.getSpentDuration(tls_id) < self.cfg.step_length:
                 self._last_phase_change_step = step

        except traci.TraCIException as exc:
            log.error("Adaptive control failed for %s: %s", tls_id, exc)
