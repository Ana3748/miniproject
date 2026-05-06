import random
import logging

log = logging.getLogger("TraCI-Bridge-Modular")

class VehicleCountProvider:
    """
    Provides per-approach vehicle counts to the adaptive controller.
    Returns random counts per direction for simulation/testing purposes.
    """

    APPROACHES = ["north", "south", "east", "west"]

    def __init__(self):
        log.info("🎲 Using RANDOM vehicle counts provider.")

    def get_counts(self, tls_id: str) -> dict[str, int]:
        """
        Returns a dict like {"north": 7, "south": 2, "east": 5, "west": 9}.
        """
        return {direction: random.randint(0, 20) for direction in self.APPROACHES}
