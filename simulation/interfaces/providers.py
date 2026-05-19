import random
import logging

from simulation.interfaces.count_provider import CountInputProvider

log = logging.getLogger("TraCI-Bridge-Modular")


class VehicleCountProvider:

    APPROACHES = ("north", "south", "east", "west")
    VEHICLE_CLASSES = (
        "Hatchback", "Sedan", "SUV", "MUV", "Bus", "Truck",
        "Three-wheeler", "Two-wheeler", "LCV", "Mini-bus",
        "Tempo-traveller", "Bicycle", "Van", "Others"
    )

    def __init__(self, mode: str = "hardcoded", max_random_per_class: int = 1):

        self.mode = mode
        self.max_random_per_class = max_random_per_class

        self._hardcoded_payload: dict[str, dict[str, int]] = {
            approach: {cls: 0 for cls in self.VEHICLE_CLASSES}
            for approach in self.APPROACHES
        }
        
        # Reduced variety for lower density
        self._hardcoded_payload["north"].update({"Hatchback": 1, "Two-wheeler": 2})
        self._hardcoded_payload["south"].update({"Sedan": 1, "Three-wheeler": 1})
        self._hardcoded_payload["east"].update({"SUV": 1, "Bicycle": 1})
        self._hardcoded_payload["west"].update({"MUV": 1, "LCV": 1})

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
