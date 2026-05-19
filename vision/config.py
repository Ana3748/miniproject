# vision/config.py
import os

# --- Path Configuration ---
# Resolves paths relative to the project root
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODELS_DIR = os.path.join(PROJECT_ROOT, "models")
ASSETS_DIR = os.path.join(PROJECT_ROOT, "assets", "videos")
OUTPUTS_DIR = os.path.join(PROJECT_ROOT, "outputs")

# --- Model Configuration ---
# The .pt file must exist in the models/ directory
MODEL_WEIGHTS = "yolo11m.pt"
# --- Pipeline Configuration ---
TARGET_FPS = 0.1         # How often to run YOLO inference
MIN_FRAME_DELAY_MS = 1    # Minimum wait between frames to keep UI responsive

# --- UI Configuration ---
WINDOW_NAME = "YOLO Vehicle Counter - Vision Pipeline"
TARGET_HEIGHT = 480       # Fix height, width will be dynamic to preserve aspect ratio
FONT_SCALE = 0.8
FONT_COLOR = (0, 255, 0)  # Green
FONT_THICKNESS = 2

# --- Export Configuration ---
EXPORT_FILE = os.path.join(OUTPUTS_DIR, "counts.json")
