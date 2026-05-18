import random
import logging

from simulation.interfaces.count_provider import CountInputProvider

log = logging.getLogger("TraCI-Bridge-Modular")


class VehicleCountProvider:

    APPROACHES = ("north", "south", "east", "west")
    VEHICLE_CLASSES = ("car", "3 wheeler", "truck", "2 wheeler", "auto")

    def __init__(self, mode: str = "hardcoded", max_random_per_class: int = 6):

        self.mode = mode
        self.max_random_per_class = max_random_per_class

        self._hardcoded_payload: dict[str, dict[str, int]] = {

            "north": {
                "car": 1,
                "3 wheeler": 0,
                "truck": 0,
                "2 wheeler": 1,
                "auto": 0
            },

            "south": {
                "car": 0,
                "3 wheeler": 0,
                "truck": 0,
                "2 wheeler": 0,
                "auto": 0
            },

            "east": {
                "car": 1,
                "3 wheeler": 0,
                "truck": 0,
                "2 wheeler": 1,
                "auto": 6
            },

            "west": {
                "car": 1,
                "3 wheeler": 1,
                "truck": 1,
                "2 wheeler": 0,
                "auto": 0
            },
        }

    def _current_payload(self) -> dict[str, dict[str, int]]:
        if self.mode == "random":
            return {
                approach: {
                    vehicle_class: random.randint(0, self.max_random_per_class)
                    for vehicle_class in self.VEHICLE_CLASSES
                }
                for approach in self.APPROACHES
            }

        return self._hardcoded_payload

    def get_payload(self):

        return self._current_payload()

    def get_total_counts(self):

        payload = self._current_payload()

        return {

            "North": sum(
                payload["north"].values()
            ),

            "South": sum(
                payload["south"].values()
            ),

            "East": sum(
                payload["east"].values()
            ),

            "West": sum(
                payload["west"].values()
            ),
        }

    def _normalize_payload(self, payload: dict) -> dict[str, dict[str, int]]:
        normalized: dict[str, dict[str, int]] = {}
        for approach in self.APPROACHES:
            class_counts = payload.get(approach, {})
            normalized[approach] = {}
            for vehicle_class in self.VEHICLE_CLASSES:
                value = int(class_counts.get(vehicle_class, 0))
                normalized[approach][vehicle_class] = max(0, value)
        return normalized

    def get_lane_class_counts(self, tls_id: str) -> dict[str, dict[str, int]]:
        return self._normalize_payload(self._current_payload())

    def get_counts(self, tls_id: str) -> dict[str, int]:
        """
        Returns per-approach totals for adaptive signal timing.
        """
        payload = self.get_lane_class_counts(tls_id)
        return self.totals_from_lane_class_counts(payload)

    def totals_from_lane_class_counts(
        self,
        payload: dict[str, dict[str, int]],
    ) -> dict[str, int]:
        return {
            approach: sum(payload[approach].values())
            for approach in self.APPROACHES
        }
