import traci

class VehicleCountOverlay:
    """
    Displays live vehicle counts and PCU metrics in SUMO GUI
    using colored quadrant boxes + text labels.
    """

    def __init__(self, junction_id: str = "junction"):
        self.junction_id = junction_id

        # Default position
        self.junction_x = 300.0
        self.junction_y = 300.0
        self.last_pcu = {}

        # Get junction position
        try:
            pos = traci.junction.getPosition(junction_id)
            self.junction_x = pos[0]
            self.junction_y = pos[1]
            print(f"[GUI Overlay] Junction '{junction_id}' found at ({self.junction_x}, {self.junction_y})")
        except Exception as e:
            print(f"[GUI Overlay] Could not get junction position: {e}")
            print(f"[GUI Overlay] Using default coordinates ({self.junction_x}, {self.junction_y})")

        # Quadrant box definitions
        self.directions = {
            "North": {
                "shape": [
                    (self.junction_x - 300, self.junction_y + 120),
                    (self.junction_x - 40, self.junction_y + 120),
                    (self.junction_x - 40, self.junction_y + 70),
                    (self.junction_x - 300, self.junction_y + 70),
                ],
                "text_pos": (self.junction_x - 200, self.junction_y + 100),
                "color": (0, 150, 255, 180),
            },
            "South": {
                "shape": [
                    (self.junction_x + 40, self.junction_y - 70),
                    (self.junction_x + 300, self.junction_y - 70),
                    (self.junction_x + 300, self.junction_y - 120),
                    (self.junction_x + 40, self.junction_y - 120),
                ],
                "text_pos": (self.junction_x + 200, self.junction_y - 95),
                "color": (255, 100, 0, 180),
            },
            "East": {
                "shape": [
                    (self.junction_x + 70, self.junction_y + 120),
                    (self.junction_x + 300, self.junction_y + 120),
                    (self.junction_x + 300, self.junction_y + 70),
                    (self.junction_x + 70, self.junction_y + 70),
                ],
                "text_pos": (self.junction_x + 150, self.junction_y + 95),
                "color": (0, 255, 100, 180),
            },
            "West": {
                "shape": [
                    (self.junction_x - 300, self.junction_y - 70),
                    (self.junction_x - 70, self.junction_y - 70),
                    (self.junction_x - 70, self.junction_y - 120),
                    (self.junction_x - 300, self.junction_y - 120),
                ],
                "text_pos": (self.junction_x - 200, self.junction_y - 95),
                "color": (255, 220, 0, 180),
            },
        }

        self._create_overlay()

    def _create_overlay(self):
        for direction, data in self.directions.items():
            polygon_id = f"direction_{direction}"
            text_id = f"text_{direction}"

            try:
                traci.polygon.remove(polygon_id)
            except Exception: pass
            try:
                traci.poi.remove(text_id)
            except Exception: pass

            try:
                traci.polygon.add(polygon_id, data["shape"], data["color"], fill=True, layer=98)
                traci.poi.add(text_id, data["text_pos"][0], data["text_pos"][1], (255, 255, 255, 255), layer=99)
                traci.poi.setParameter(text_id, "text", f"{direction}: 0.0 PCU")
            except Exception as e:
                print(f"[GUI Overlay] Failed creating overlay for {direction}: {e}")

    def update_metrics(self, metrics: dict):
        """
        Updates the overlay text with PCU and individual class counts.
        """
        approach_to_dir = {
            "north_in": "North",
            "south_in": "South",
            "east_in": "East",
            "west_in": "West"
        }

        for approach, data in metrics.items():
            direction = approach_to_dir.get(approach)
            if not direction:
                continue

            pcu = data.get("pcu", 0.0)
            counts = data.get("counts", {})
            
            text_id = f"text_{direction}"
            polygon_id = f"direction_{direction}"

            # Format breakdown text (e.g., "Sedan:2, Bus:1")
            breakdown = ", ".join([f"{cls}:{count}" for cls, count in counts.items() if count > 0])
            if not breakdown:
                breakdown = "Empty"
            
            display_text = f"{direction}: {pcu:.1f} PCU | {breakdown}"

            try:
                traci.poi.setParameter(text_id, "text", display_text)
                
                # Dynamic color brightness based on PCU
                brightness = min(255, int(100 + (pcu * 10)))
                if direction == "North": new_color = (0, brightness, 255, 180)
                elif direction == "South": new_color = (255, brightness, 0, 180)
                elif direction == "East": new_color = (0, 255, brightness, 180)
                else: new_color = (255, 255, brightness, 180)
                
                traci.polygon.setColor(polygon_id, new_color)
            except Exception:
                pass

            if direction not in self.last_pcu or abs(self.last_pcu[direction] - pcu) > 0.1:
                self.last_pcu[direction] = pcu
