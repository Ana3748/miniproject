import traci
import logging
from simulation.core.config import Config

log = logging.getLogger("TraCI-Bridge-Backup")

class AdaptiveController:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._last_phase_change_step = 0
        self._extension_count = 0
        self._was_preempted = False
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
        # Detect release of preemption to restart timers
        if not preempted and self._was_preempted:
            log.info("Adaptive: Preemption released, restarting timers.")
            self._last_phase_change_step = step
            self._extension_count = 0
            
        self._was_preempted = preempted

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
            # Phase 2: All Red
            # Phase 3: EW Green
            # Phase 4: EW Yellow
            # Phase 5: All Red
            
            elapsed = (step - self._last_phase_change_step) * self.cfg.step_length
            
            # 1. Respect Yellow/Red Phases: Do not interrupt a transition
            if current_phase in [1, 2, 4, 5]:
                return

            # Current threshold for decision
            # After base_green + (extension_count * extend_green), we decide to extend or switch
            threshold = self.cfg.base_green + (self._extension_count * self.cfg.extend_green)

            if current_phase == 0: # NS is Green
                # A) Early release: NS empty, EW has traffic
                if ns_pcu == 0 and ew_pcu > 0:
                    traci.trafficlight.setPhase(tls_id, 1) # NS Yellow
                    self._last_phase_change_step = step
                    self._extension_count = 0
                    log.info("Adaptive: Early Switch NS -> EW | NS_PCU=0 EW_PCU=%.1f", ew_pcu)
                    return

                # B) Decision time: reached the threshold
                if elapsed >= threshold:
                    # Extension check: Current > Opposing and not at limit
                    if ns_pcu > ew_pcu and self._extension_count < self.cfg.extend_n_times:
                        self._extension_count += 1
                        log.info("Adaptive: Extending NS Green (%d/%d) | NS_PCU=%.1f EW_PCU=%.1f", 
                                 self._extension_count, self.cfg.extend_n_times, ns_pcu, ew_pcu)
                    # Switch check: Opposing traffic present
                    elif ew_pcu > 0:
                        traci.trafficlight.setPhase(tls_id, 1) # NS Yellow
                        self._last_phase_change_step = step
                        self._extension_count = 0
                        log.info("Adaptive: Switching NS -> EW | NS_PCU=%.1f EW_PCU=%.1f Elapsed=%.1fs", 
                                 ns_pcu, ew_pcu, elapsed)
            
            elif current_phase == 3: # EW is Green
                # A) Early release: EW empty, NS has traffic
                if ew_pcu == 0 and ns_pcu > 0:
                    traci.trafficlight.setPhase(tls_id, 4) # EW Yellow
                    self._last_phase_change_step = step
                    self._extension_count = 0
                    log.info("Adaptive: Early Switch EW -> NS | EW_PCU=0 NS_PCU=%.1f", ns_pcu)
                    return

                # B) Decision time: reached the threshold
                if elapsed >= threshold:
                    # Extension check: Current > Opposing and not at limit
                    if ew_pcu > ns_pcu and self._extension_count < self.cfg.extend_n_times:
                        self._extension_count += 1
                        log.info("Adaptive: Extending EW Green (%d/%d) | EW_PCU=%.1f NS_PCU=%.1f", 
                                 self._extension_count, self.cfg.extend_n_times, ew_pcu, ns_pcu)
                    # Switch check: Opposing traffic present
                    elif ns_pcu > 0:
                        traci.trafficlight.setPhase(tls_id, 4) # EW Yellow
                        self._last_phase_change_step = step
                        self._extension_count = 0
                        log.info("Adaptive: Switching EW -> NS | EW_PCU=%.1f NS_PCU=%.1f Elapsed=%.1fs", 
                                 ew_pcu, ns_pcu, elapsed)
            
            # Reset timer if we just transitioned to a main green phase from something else
            if current_phase in [0, 3] and traci.trafficlight.getSpentDuration(tls_id) < self.cfg.step_length + 0.001:
                if self._last_phase_change_step != step:
                    self._last_phase_change_step = step
                    self._extension_count = 0 # Ensure reset on actual phase start

        except traci.TraCIException as exc:
            log.error("Adaptive control failed for %s: %s", tls_id, exc)
