from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict


class CountInputProvider(ABC):
    """Abstract interface for providers that supply lane-class counts
    and per-approach totals. Implement this to plug in YOLO or other
    external pipelines without changing spawner logic.
    """

    @abstractmethod
    def get_lane_class_counts(self, tls_id: str) -> Dict[str, Dict[str, int]]:
        """Return a mapping: approach -> {class: count}.
        Example: {"north": {"car": 3, "3 wheeler": 1, "truck": 0}, ...}
        """

    @abstractmethod
    def get_counts(self, tls_id: str) -> Dict[str, int]:
        """Return per-approach totals used by adaptive control.
        Example: {"north": 4, "south": 0, "east": 5, "west": 2}
        """
