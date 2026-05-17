# Vision Pipeline: YOLO Vehicle Counter

This module runs a standalone YOLO object detection pipeline on pre-recorded intersection videos.

## 1. Adding Videos
Place videos in `assets/videos/`. Use `<direction>_<name>.mp4`.

## 2. Configuration
Edit `vision/config.py` to change models or FPS.

## 3. Usage
`python -m vision.pipeline`

## 4. Controls
- `c` or `Space`: Capture counts to `outputs/counts.json`.
- `q`: Quit.
