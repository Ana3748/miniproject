# frames/ — YOLO Input Image Folder
# =====================================
# Drop your traffic intersection images here.
# Supported formats: .jpg  .jpeg  .png
#
# How FrameSource reads this folder (yolo_detector.py):
#   - Files are sorted alphabetically and read in order.
#   - When all images are consumed, get_yolo_vehicle_counts() returns zeros.
#   - To loop images, convert your folder into a video first:
#       ffmpeg -r 10 -pattern_type glob -i '*.jpg' traffic_loop.mp4
#     Then set VIDEO_OR_IMAGE_SOURCE = "traffic_loop.mp4" in the bridge.
#
# Naming tip: name images sequentially so they sort correctly:
#   frame_0001.jpg, frame_0002.jpg, ...
#
# ROI layout expected by yolo_detector.py (640x640 frame assumed):
#   north : top strip     (0,   0,   640, 180)
#   south : bottom strip  (0,   460, 640, 640)
#   east  : right strip   (460, 0,   640, 640)
#   west  : left strip    (0,   0,   180, 640)
#
# Adjust DetectorConfig.approach_rois in yolo_detector.py if your
# camera resolution or layout differs.
