# vision/detector.py
import os
import logging
from ultralytics import YOLO
from vision import config

log = logging.getLogger("Vision-Detector")

class VehicleDetector:
    """
    Wrapper for YOLO models to handle initialization and inference.
    """
    def __init__(self, model_name: str = None):
        self.model_name = model_name or config.MODEL_WEIGHTS
        model_path = os.path.join(config.MODELS_DIR, self.model_name)
        
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"YOLO weights not found at: {model_path}")
            
        log.info(f"Loading YOLO model: {self.model_name}")
        self.model = YOLO(model_path)
        
        # By default, we track all classes found in the model metadata.
        # This is safer for custom-trained models where indices differ from COCO.
        self.target_classes = list(self.model.names.keys())
        log.info(f"Tracking {len(self.target_classes)} classes: {self.model.names}")

    def detect(self, frame):
        """
        Runs inference on a single frame and returns counts and annotated frame.
        """
        # We track target_classes defined in __init__
        results = self.model(frame, classes=self.target_classes, verbose=False)
        result = results[0]
        
        # Count occurrences of all detected objects in target classes
        counts = len(result.boxes)
        
        # Get annotated frame (drawing bounding boxes)
        annotated_frame = result.plot()
        
        return counts, annotated_frame
