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
    
    try:
        v_detector = detector.VehicleDetector()
    except Exception as e:
        log.error(f"Initialization Error: {e}")
        sys.exit(1)
        
    caps = {dir_name: cv2.VideoCapture(path) for dir_name, path in sources.items()}
    current_counts = {dir_name: 0 for dir_name in sources}
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
                        count, annotated = v_detector.detect(frame)
                        current_counts[dir_name] = count
                        frames_to_display[dir_name] = annotated
            
            if run_inference:
                last_inference_time = current_time
                if frames_to_display:
                    canvas = utils.create_grid(frames_to_display, current_counts)
                    cv2.imshow(config.WINDOW_NAME, canvas)

            key = cv2.waitKey(config.MIN_FRAME_DELAY_MS) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c') or key == ord(' '):
                exporter.export_counts(current_counts)
                
    except KeyboardInterrupt:
        pass
    finally:
        for cap in caps.values():
            cap.release()
        cv2.destroyAllWindows()
        log.info("Pipeline shut down.")

if __name__ == "__main__":
    main()
