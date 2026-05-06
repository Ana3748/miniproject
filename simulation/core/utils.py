import logging
import math
import traci
from typing import Optional

def setup_logging(name: str):
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )
    return logging.getLogger(name)

def euclidean_distance(pos1: tuple, pos2: tuple) -> float:
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

def get_tls_junction_position(tls_id: str) -> Optional[tuple[float, float]]:
    try:
        controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
        if not controlled_lanes:
            return None
        positions = []
        for lane in controlled_lanes:
            shape = traci.lane.getShape(lane)
            if shape:
                positions.append(shape[-1])
        if not positions:
            return None
        cx = sum(p[0] for p in positions) / len(positions)
        cy = sum(p[1] for p in positions) / len(positions)
        return cx, cy
    except traci.TraCIException:
        return None
