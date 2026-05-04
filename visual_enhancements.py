"""
ADDITIONS for adaptive_traffic_traci_bridge.py
================================================
Paste these functions into the bridge script.
Call them from inside the simulation loop.
"""

import traci
import random
import math

# ── Junction position (must match your nodes.nod.xml) ──
JUNCTION_POS = (0.0, 0.0)
PREEMPTION_RADIUS = 150.0       # metres
EMERGENCY_TLS_PHASE = 0         # phase index that gives green to west→east
EMERGENCY_GREEN_HOLD = 30.0     # seconds

# ── Track preemption state ──
_preempted = False
_tracking_ev = False


def set_vehicle_colors() -> None:
    """
    On each step:
      - Emergency vehicles → bright red + white outline
      - Standard vehicles  → keep their vType color (already set in routes)
    Called once per simulation step.
    """
    for vid in traci.vehicle.getIDList():
        vtype = traci.vehicle.getTypeID(vid)
        if vtype == "emergency":
            traci.vehicle.setColor(vid, (255, 0, 0, 255))      # vivid red
        # Standard cars keep their vType color — no override needed


def track_emergency_vehicle() -> None:
    """
    Once an emergency vehicle appears, lock the camera onto it.
    Uses traci.gui.trackVehicle so the view follows the ambulance.
    """
    global _tracking_ev
    for vid in traci.vehicle.getIDList():
        if traci.vehicle.getTypeID(vid) == "emergency" and not _tracking_ev:
            try:
                traci.gui.trackVehicle("View #0", vid)
                print(f"📷 Camera locked on emergency vehicle: {vid}")
            except traci.TraCIException:
                pass  # headless mode — GUI not available, safe to ignore
            _tracking_ev = True


def preemption_logic(sim_step: int) -> None:
    """
    Green Wave preemption:
      1. Find all emergency vehicles.
      2. Calculate Euclidean distance to junction.
      3. If within PREEMPTION_RADIUS → force TLS to emergency green phase.
      4. Release after EMERGENCY_GREEN_HOLD seconds.
    """
    global _preempted

    emergency_vehicles = [
        v for v in traci.vehicle.getIDList()
        if traci.vehicle.getTypeID(v) == "emergency"
    ]

    if not emergency_vehicles:
        return

    for tls_id in traci.trafficlight.getIDList():
        nearest_dist = float("inf")
        for vid in emergency_vehicles:
            pos = traci.vehicle.getPosition(vid)
            dist = math.sqrt(
                (pos[0] - JUNCTION_POS[0]) ** 2 +
                (pos[1] - JUNCTION_POS[1]) ** 2
            )
            nearest_dist = min(nearest_dist, dist)

        if nearest_dist <= PREEMPTION_RADIUS and not _preempted:
            # Force green for emergency approach
            traci.trafficlight.setPhase(tls_id, EMERGENCY_TLS_PHASE)
            traci.trafficlight.setPhaseDuration(tls_id, EMERGENCY_GREEN_HOLD)
            _preempted = True
            print(f"🚨 PREEMPTION ACTIVATED — EV is {nearest_dist:.1f}m away")

        elif nearest_dist > PREEMPTION_RADIUS and _preempted:
            # Release — let SUMO resume normal program
            traci.trafficlight.setPhaseDuration(tls_id, 0)
            _preempted = False
            print("✅ Preemption released — resuming normal TLS program")


def update_gui_schema(has_emergency: bool) -> None:
    """
    Switch the GUI color schema based on emergency status.
    NOTE: traci.gui does not support text overlays natively.
    We simulate visual feedback by changing the view schema name.
    For a real text overlay, use SUMO's --device.ssm.file or
    a separate Tkinter/PyQt overlay window on top of sumo-gui.
    """
    try:
        if has_emergency:
            traci.gui.setSchema("View #0", "real world")
        else:
            traci.gui.setSchema("View #0", "standard")
    except traci.TraCIException:
        pass   # schema may not exist — safe to ignore


# ══════════════════════════════════════════════════════════════
# HOW TO INTEGRATE INTO run_simulation() LOOP
# ══════════════════════════════════════════════════════════════
# Inside the while loop in run_simulation(), add:
#
#   set_vehicle_colors()
#   track_emergency_vehicle()
#   preemption_logic(step)
#
#   has_ev = any(
#       traci.vehicle.getTypeID(v) == "emergency"
#       for v in traci.vehicle.getIDList()
#   )
#   update_gui_schema(has_ev)
#
# ══════════════════════════════════════════════════════════════