# vision/detector.py
import os
import logging
import cv2
import numpy as np
from ultralytics import YOLO
from vision import config, utils

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
        self.target_classes = list(self.model.names.keys())
        log.info(f"Tracking {len(self.target_classes)} classes: {self.model.names}")

    def detect(self, frame, roi_polygon: list[tuple[int, int]] = None):
        """
        Runs inference on a single frame and returns class-wise counts and annotated frame.
        Applies ROI masking and center-point filtering if roi_polygon is provided.
        """
        inference_frame = frame
        
        if roi_polygon:
            # 1. Create mask
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            poly_np = np.array(roi_polygon, dtype=np.int32)
            cv2.fillPoly(mask, [poly_np], 255)
            
            # 2. Apply mask to frame (for YOLO to ignore background)
            inference_frame = cv2.bitwise_and(frame, frame, mask=mask)

        # 3. Run Inference
        results = self.model(inference_frame, classes=self.target_classes, verbose=False)
        result = results[0]
        
        valid_boxes = []
        for box in result.boxes:
            # Get box coordinates (x1, y1, x2, y2)
            coords = box.xyxy[0].cpu().numpy()
            center_x = (coords[0] + coords[2]) / 2
            center_y = (coords[1] + coords[3]) / 2
            
            # 4. Filter by Center Point
            if roi_polygon:
                if utils.is_inside_poly((int(center_x), int(center_y)), roi_polygon):
                    valid_boxes.append(box)
            else:
                valid_boxes.append(box)
        
        # 5. Count by Class
        class_counts = {}
        for box in valid_boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            class_counts[cls_name] = class_counts.get(cls_name, 0) + 1
            
        # 6. Visual Feedback: Draw ROI and filtered detections
        annotated_frame = result.plot() 
        if roi_polygon:
             poly_np = np.array(roi_polygon, dtype=np.int32)
             cv2.polylines(annotated_frame, [poly_np], True, (0, 255, 255), 2)

        return class_counts, annotated_frame
