import random
import logging
import traci

from simulation.core.config import Config

log = logging.getLogger("TraCI-Bridge-Modular")


class DynamicSpawner:
    TYPE_MAP = {
        "car": "car",
        "3 wheeler": "threewheeler",
        "truck": "truck",
        "2 wheeler": "twowheeler",
        "auto":"autorickshaw"
    }

    ROUTE_POOLS = {
        "north": [
            ("rt_north_straight", ["north_in", "south_out"]),
            ("rt_north_turn", ["north_in", "east_out"]),
        ],
        "south": [
            ("rt_south_straight", ["south_in", "north_out"]),
            ("rt_south_turn", ["south_in", "west_out"]),
        ],
        "east": [
            ("rt_east_straight", ["east_in", "west_out"]),
            ("rt_east_turn", ["east_in", "north_out"]),
        ],
        "west": [
            ("rt_west_straight", ["west_in", "east_out"]),
            ("rt_west_turn", ["west_in", "south_out"]),
        ],
    }

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._seq = 0
        self._routes_ready = False

    def _ensure_routes(self) -> None:
        if self._routes_ready:
            return

        existing_routes = set(traci.route.getIDList())
        for route_options in self.ROUTE_POOLS.values():
            for route_id, edges in route_options:
                if route_id not in existing_routes:
                    traci.route.add(route_id, edges)
        self._routes_ready = True

    def _next_vehicle_id(self, vehicle_class: str, step: int) -> str:
        self._seq += 1
        safe_cls = vehicle_class.replace(" ", "")
        return f"dyn_{safe_cls}_{step}_{self._seq}"

    def _choose_route_id(self, approach: str) -> str:
        route_options = self.ROUTE_POOLS[approach]
        return random.choice(route_options)[0]

    def spawn_for_tls(
        self,
        tls_id: str,
        lane_class_counts: dict[str, dict[str, int]],
        step: int,
    ) -> dict[str, dict[str, int]]:
        self._ensure_routes()

        stats = {
            "requested": 0,
            "inserted": 0,
            "failed": 0,
        }

        for approach, class_counts in lane_class_counts.items():
            if approach not in self.ROUTE_POOLS:
                continue

            for vehicle_class, requested_raw in class_counts.items():
                requested = max(0, min(int(requested_raw), self.cfg.spawn_max_per_class_per_lane))
                if requested == 0:
                    continue

                type_id = self.TYPE_MAP.get(vehicle_class)
                if not type_id:
                    log.warning("Skipping unknown class '%s' for %s", vehicle_class, tls_id)
                    continue

                for _ in range(requested):
                    stats["requested"] += 1
                    veh_id = self._next_vehicle_id(vehicle_class, step)
                    route_id = self._choose_route_id(approach)
                    try:
                        traci.vehicle.add(
                            vehID=veh_id,
                            routeID=route_id,
                            typeID=type_id,
                            depart="now",
                            departLane="best",
                            departSpeed="max",
                        )
                        stats["inserted"] += 1
                    except traci.TraCIException:
                        stats["failed"] += 1

        return stats
