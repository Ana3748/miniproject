# vision/check_classes.py
import os
import sys
from ultralytics import YOLO

# Add parent directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from vision import config

def main():
    model_path = os.path.join(config.MODELS_DIR, config.MODEL_WEIGHTS)
    
    if not os.path.exists(model_path):
        print(f"ERROR: Model not found at {model_path}")
        return

    print(f"Loading model: {model_path}...")
    model = YOLO(model_path)
    
    print("\n--- MODEL CLASS LIST ---")
    names = model.names
    for idx, name in names.items():
        print(f"Index {idx}: {name}")
    print("------------------------\n")
    
    print("Update the 'target_classes' list in vision/detector.py with the indices you want to track.")

if __name__ == "__main__":
    main()
