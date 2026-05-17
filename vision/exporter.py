# vision/exporter.py
import json
import os
import logging
from vision import config

log = logging.getLogger("Vision-Exporter")

def export_counts(counts_dict: dict[str, int]):
    """
    Writes the provided counts to a JSON file.
    Ensures the output directory exists.
    """
    if not os.path.exists(config.OUTPUTS_DIR):
        os.makedirs(config.OUTPUTS_DIR)
        
    # Ensure all directions are present in the export, even if 0
    full_payload = {
        "north": counts_dict.get("north", 0),
        "south": counts_dict.get("south", 0),
        "east": counts_dict.get("east", 0),
        "west": counts_dict.get("west", 0)
    }
    
    try:
        with open(config.EXPORT_FILE, "w") as f:
            json.dump(full_payload, f, indent=2)
        log.info(f"Snapshot exported to {config.EXPORT_FILE}: {full_payload}")
        return True
    except Exception as e:
        log.error(f"Failed to export counts: {e}")
        return False
