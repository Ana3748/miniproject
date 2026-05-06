import random
import logging
from typing import Optional
from simulation.core.config import Config

log = logging.getLogger("TraCI-Bridge-Backup")

class GPSSimulator:
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
