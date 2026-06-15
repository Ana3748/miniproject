# vision/pipeline.py
import cv2
import time
import logging
import sys
from vision import config, utils, detector, exporter

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("Vision-Pipeline")

def main():
    log.info("Starting Vision Pipeline...")
    sources = utils.get_video_sources()
    
    if not sources:
        log.error("No valid video files found in assets/videos/. Check your naming convention.")
        sys.exit(1)
        
    try:
        v_detector = detector.VehicleDetector()
    except Exception as e:
        log.error(f"Initialization Error: {e}")
        sys.exit(1)
        
    # --- ROI Setup Phase ---
    rois = {}
    ordered_directions = ["north", "south", "east", "west"]
    
    for dir_name in ordered_directions:
        if dir_name in sources:
            cap = cv2.VideoCapture(sources[dir_name])
            ret, frame = cap.read()
            if ret:
                poly = utils.select_roi(dir_name, frame)
                rois[dir_name] = poly
            cap.release()
            
    # --- Main Inference Loop ---
    caps = {dir_name: cv2.VideoCapture(path) for dir_name, path in sources.items()}
    current_counts = {dir_name: {} for dir_name in sources}
    last_inference_time = 0
    inference_interval = 1.0 / config.TARGET_FPS
    
    log.info(f"Pipeline running. Controls: [C]apture, [Q]uit. Target: {config.TARGET_FPS} FPS")

    try:
        while True:
            frames_to_display = {}
            current_time = time.time()
            run_inference = (current_time - last_inference_time) >= inference_interval
            
            for dir_name, cap in caps.items():
                ret, frame = cap.read()
                if not ret:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = cap.read()
                
                if ret:
                    if run_inference:
                        # Pass ROI to detector
                        poly = rois.get(dir_name)
                        counts, annotated = v_detector.detect(frame, roi_polygon=poly)
                        
                        # Check for changes and log to terminal
                        if counts != current_counts[dir_name]:
                            active = {k: v for k, v in counts.items() if v > 0}
                            total = sum(active.values())
                            print(f"\n[CHANGE] {dir_name.upper()} | Total: {total}")
                            for cls, count in active.items():
                                print(f"  - {cls}: {count}")
                            
                            current_counts[dir_name] = counts
                            
                        frames_to_display[dir_name] = annotated
            
            if run_inference:
                last_inference_time = current_time
                if frames_to_display:
                    canvas = utils.create_grid(frames_to_display, current_counts)
                    cv2.imshow(config.WINDOW_NAME, canvas)

                # Keep the JSON snapshot in sync with the latest visible counts.
                total_counts = {d: sum(c.values()) for d, c in current_counts.items()}
                exporter.export_counts(total_counts)

            key = cv2.waitKey(config.MIN_FRAME_DELAY_MS) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c') or key == ord(' '):
                # For backward compatibility with exporter, we'll send total counts
                total_counts = {d: sum(c.values()) for d, c in current_counts.items()}
                exporter.export_counts(total_counts)
                
    except KeyboardInterrupt:
        pass
    finally:
        for cap in caps.values():
            cap.release()
        cv2.destroyAllWindows()
        log.info("Pipeline shut down.")

if __name__ == "__main__":
    main()
