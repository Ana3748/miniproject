import traci


class VehicleCountOverlay:
    """
    Displays live vehicle counts in SUMO GUI
    using colored quadrant boxes + text labels.
    """

    def __init__(self, junction_id: str = "junction"):

        self.junction_id = junction_id

        # Default position
        self.junction_x = 300.0
        self.junction_y = 300.0

        self.last_counts = {}

        # Get junction position
        try:
            pos = traci.junction.getPosition(junction_id)

            self.junction_x = pos[0]
            self.junction_y = pos[1]

            print(
                f"[GUI Overlay] Junction '{junction_id}' found at "
                f"({self.junction_x}, {self.junction_y})"
            )

        except Exception as e:

            print(f"[GUI Overlay] Could not get junction position: {e}")

            print(
                f"[GUI Overlay] Using default coordinates "
                f"({self.junction_x}, {self.junction_y})"
            )

        # Quadrant box definitions
        self.directions = {

            "North": {
                "shape": [
                    (self.junction_x - 120, self.junction_y + 120),
                    (self.junction_x - 40, self.junction_y + 120),
                    (self.junction_x - 40, self.junction_y + 70),
                    (self.junction_x - 120, self.junction_y + 70),
                ],

                "text_pos": (
                    self.junction_x - 105,
                    self.junction_y + 95
                ),

                "color": (0, 150, 255, 180),
            },

            "South": {
                "shape": [
                    (self.junction_x + 40, self.junction_y - 70),
                    (self.junction_x + 120, self.junction_y - 70),
                    (self.junction_x + 120, self.junction_y - 120),
                    (self.junction_x + 40, self.junction_y - 120),
                ],

                "text_pos": (
                    self.junction_x + 55,
                    self.junction_y - 95
                ),

                "color": (255, 100, 0, 180),
            },

            "East": {
                "shape": [
                    (self.junction_x + 70, self.junction_y + 120),
                    (self.junction_x + 150, self.junction_y + 120),
                    (self.junction_x + 150, self.junction_y + 70),
                    (self.junction_x + 70, self.junction_y + 70),
                ],

                "text_pos": (
                    self.junction_x + 85,
                    self.junction_y + 95
                ),

                "color": (0, 255, 100, 180),
            },

            "West": {
                "shape": [
                    (self.junction_x - 150, self.junction_y - 70),
                    (self.junction_x - 70, self.junction_y - 70),
                    (self.junction_x - 70, self.junction_y - 120),
                    (self.junction_x - 150, self.junction_y - 120),
                ],

                "text_pos": (
                    self.junction_x - 135,
                    self.junction_y - 95
                ),

                "color": (255, 220, 0, 180),
            },
        }

        # Create GUI elements
        self._create_overlay()

        print("[GUI Overlay] Initialized successfully")

    def _create_overlay(self):

        for direction, data in self.directions.items():

            polygon_id = f"direction_{direction}"
            text_id = f"text_{direction}"

            # Remove old polygon if exists
            try:
                traci.polygon.remove(polygon_id)
            except Exception:
                pass

            # Remove old text if exists
            try:
                traci.poi.remove(text_id)
            except Exception:
                pass

            # Create colored box
            try:

                traci.polygon.add(
                    polygon_id,
                    data["shape"],
                    data["color"],
                    fill=True,
                    layer=98
                )

                print(f"[GUI Overlay] Created polygon for {direction}")

            except Exception as e:

                print(
                    f"[GUI Overlay] Failed creating polygon "
                    f"for {direction}: {e}"
                )

            # Create text label
            try:

                traci.poi.add(
                    text_id,
                    data["text_pos"][0],
                    data["text_pos"][1],
                    (255, 255, 255, 255),
                    layer=99
                )

                traci.poi.setParameter(
                    text_id,
                    "text",
                    f"{direction}: 0"
                )

                print(f"[GUI Overlay] Created text for {direction}")

            except Exception as e:

                print(
                    f"[GUI Overlay] Failed creating text "
                    f"for {direction}: {e}"
                )

    def update_counts(self, counts: dict[str, int]):

        try:

            for direction, count in counts.items():

                if direction not in self.directions:
                    continue

                polygon_id = f"direction_{direction}"
                text_id = f"text_{direction}"

                base_color = self.directions[direction]["color"]

                # Dynamic brightness
                brightness = min(255, 100 + (count * 8))

                if direction == "North":

                    new_color = (
                        0,
                        brightness,
                        255,
                        200
                    )

                elif direction == "South":

                    new_color = (
                        255,
                        brightness,
                        0,
                        200
                    )

                elif direction == "East":

                    new_color = (
                        0,
                        255,
                        brightness,
                        200
                    )

                else:

                    new_color = (
                        255,
                        255,
                        brightness,
                        200
                    )

                # Update box color
                try:

                    traci.polygon.setColor(
                        polygon_id,
                        new_color
                    )

                except Exception as e:

                    print(
                        f"[GUI Overlay] Failed updating polygon "
                        f"{polygon_id}: {e}"
                    )

                # Update displayed text
                try:

                    traci.poi.setParameter(
                        text_id,
                        "text",
                        f"{direction}: {count}"
                    )

                except Exception as e:

                    print(
                        f"[GUI Overlay] Failed updating text "
                        f"{text_id}: {e}"
                    )

                # Console output only if changed
                if (
                    direction not in self.last_counts
                    or
                    self.last_counts[direction] != count
                ):

                    print(
                        f"[Traffic Counts] "
                        f"{direction}: {count} vehicles"
                    )

                    self.last_counts[direction] = count

        except Exception as e:

            print(f"[GUI Overlay] Error updating counts: {e}")