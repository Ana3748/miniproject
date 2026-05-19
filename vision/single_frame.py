# vision/single_frame.py
import cv2
import logging
import os
from vision import utils, detector, config

log = logging.getLogger("Vision-SingleFrame")

def get_yolo_counts() -> dict[str, dict[str, int]]:
    """
    Performs a single YOLO inference pass on the first frame of directional videos.
    Prompts for ROI selection and displays a verification grid.
    Returns: {direction: {vehicle_class: count}}
    """
    log.info("Starting single-frame YOLO inference...")
    sources = utils.get_video_sources()
    
    if not sources:
        log.error("No valid video files found in assets/videos/.")
        return {}
        
    try:
        v_detector = detector.VehicleDetector()
    except Exception as e:
        log.error(f"Initialization Error: {e}")
        return {}
        
    first_frames = {}
    rois = {}
    ordered_directions = ["north", "south", "east", "west"]
    
    # 1. Capture first frames and get ROIs
    for dir_name in ordered_directions:
        if dir_name in sources:
            cap = cv2.VideoCapture(sources[dir_name])
            ret, frame = cap.read()
            if ret:
                first_frames[dir_name] = frame
                # User draws ROI for this direction
                poly = utils.select_roi(dir_name, frame)
                rois[dir_name] = poly
            cap.release()
            
    if not first_frames:
        log.error("Failed to read any frames from sources.")
        return {}

    # 2. Perform Inference
    final_counts = {}
    annotated_frames = {}
    
    for dir_name, frame in first_frames.items():
        poly = rois.get(dir_name)
        counts, annotated = v_detector.detect(frame, roi_polygon=poly)
        final_counts[dir_name] = counts
        annotated_frames[dir_name] = annotated
        
        total = sum(counts.values())
        log.info(f"Detected in {dir_name.upper()}: {total} vehicles.")

    # 3. Save Verification Display
    if annotated_frames:
        canvas = utils.create_grid(annotated_frames, final_counts)
        output_path = os.path.join(config.OUTPUTS_DIR, "yolo_initial_detection.jpg")
        
        if not os.path.exists(config.OUTPUTS_DIR):
            os.makedirs(config.OUTPUTS_DIR)
            
        cv2.imwrite(output_path, canvas)
        log.info(f"YOLO verification image saved to: {output_path}")
        
    return final_counts
