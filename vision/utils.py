# vision/utils.py
import os
import cv2
import numpy as np
from vision import config

def get_video_sources() -> dict[str, str]:
    """
    Scans the assets/videos directory for files following the naming convention.
    Returns: {"north": "path/to/north_video.mp4", ...}
    """
    sources = {}
    valid_directions = ["north", "south", "east", "west"]
    
    if not os.path.exists(config.ASSETS_DIR):
        os.makedirs(config.ASSETS_DIR)
        return sources

    for filename in os.listdir(config.ASSETS_DIR):
        if filename.startswith(".") or os.path.isdir(os.path.join(config.ASSETS_DIR, filename)):
            continue
            
        parts = filename.lower().split("_")
        direction = parts[0]
        
        if direction in valid_directions and direction not in sources:
            sources[direction] = os.path.join(config.ASSETS_DIR, filename)
            
    return sources

def create_grid(frames_dict: dict[str, np.ndarray], counts_dict: dict[str, int]) -> np.ndarray:
    """
    Arranges 1-4 frames into a single canvas (1x1, 1x2, or 2x2).
    Preserves aspect ratio based on config.TARGET_HEIGHT.
    """
    directions = list(frames_dict.keys())
    num_feeds = len(directions)
    
    if num_feeds == 0:
        # Default placeholder width 640
        blank = np.zeros((config.TARGET_HEIGHT, 640, 3), dtype=np.uint8)
        cv2.putText(blank, "No Videos Found in assets/videos/", (50, config.TARGET_HEIGHT // 2), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return blank

    processed_frames = []
    for dir_name in directions:
        frame = frames_dict[dir_name]
        count = counts_dict[dir_name]
        
        # Preserve aspect ratio
        h, w = frame.shape[:2]
        aspect_ratio = w / h
        target_width = int(config.TARGET_HEIGHT * aspect_ratio)
        
        resized = cv2.resize(frame, (target_width, config.TARGET_HEIGHT))
        
        # Draw overlay background for text
        cv2.rectangle(resized, (0, 0), (min(200, target_width), 40), (0, 0, 0), -1)
        
        label = f"{dir_name.upper()}: {count}"
        cv2.putText(resized, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    config.FONT_SCALE, config.FONT_COLOR, config.FONT_THICKNESS)
        
        processed_frames.append(resized)

    # Grid arrangement
    if num_feeds == 1:
        return processed_frames[0]
    
    elif num_feeds == 2:
        # Note: If widths differ, hstack won't work perfectly. We assume similar cameras.
        # To be safe, we pad the narrower one if needed.
        w1 = processed_frames[0].shape[1]
        w2 = processed_frames[1].shape[1]
        if w1 != w2:
            max_w = max(w1, w2)
            processed_frames[0] = _pad_to_width(processed_frames[0], max_w)
            processed_frames[1] = _pad_to_width(processed_frames[1], max_w)
        return np.hstack(processed_frames)
    
    else:
        # 2x2 Grid (pad all to same width)
        max_w = max(f.shape[1] for f in processed_frames)
        padded = [_pad_to_width(f, max_w) for f in processed_frames]
        
        while len(padded) < 4:
            padded.append(np.zeros((config.TARGET_HEIGHT, max_w, 3), dtype=np.uint8))
            
        top_row = np.hstack([padded[0], padded[1]])
        bottom_row = np.hstack([padded[2], padded[3]])
        return np.vstack([top_row, bottom_row])

def _pad_to_width(frame: np.ndarray, target_width: int) -> np.ndarray:
    h, w = frame.shape[:2]
    if w == target_width:
        return frame
    
    diff = target_width - w
    # Add black padding to the right
    padding = np.zeros((h, diff, 3), dtype=np.uint8)
    return np.hstack([frame, padding])
