"""
yolo_detector.py
================
Dual-model YOLO vehicle detector for Adaptive Traffic Control.

Primary   : YOLOv11 Medium  (yolo11m.pt)   — high accuracy, main detections
Secondary : YOLO26 Medium   (yolo26m.pt)   — NMS-free, catches what v11 misses

How the two models work together
─────────────────────────────────
1. Every frame is run through YOLOv11 first.
2. Any detection with confidence < CONF_HANDOFF is passed to YOLO26 for a
   second opinion.  If YOLO26 is confident enough, the detection is kept.
3. The merged detections are counted per approach direction and returned to
   the TraCI bridge as a simple dict {"north": int, "south": int, ...}.

Why this split?
  • YOLOv11 gives strong per-class accuracy on dense traffic scenes.
  • YOLO26 is NMS-free (end-to-end) so it handles crowded/occluded vehicles
    and small objects better (STAL loss), acting as a safety net.
"""

import cv2
import numpy as np
import logging
import torch
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional
from ultralytics import YOLO

log = logging.getLogger("YOLODetector")

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class DetectorConfig:
    # Model weights — place .pt files in the same folder as this script,
    # or provide full paths.
    primary_weights: str   = "yolo11m.pt"    # YOLOv11 Medium
    secondary_weights: str = "yolo26m.pt"    # YOLO26 Medium

    # Inference settings
    imgsz: int             = 640
    device: str            = "cuda"          # "cuda" or "cpu"
    primary_conf: float    = 0.25            # min confidence for YOLOv11
    secondary_conf: float  = 0.20            # min confidence for YOLO26
    conf_handoff: float    = 0.45            # detections below this go to YOLO26

    # COCO vehicle class IDs (used when models are pretrained on COCO)
    # car=2, motorcycle=3, bus=5, truck=7
    vehicle_classes: list = field(default_factory=lambda: [2, 3, 5, 7])

    # Emergency vehicle class ID in YOUR custom-trained model
    # Set this to whatever class index your dataset uses for ambulances/firetrucks
    emergency_class_id: int = 8

    # Region-of-interest polygons per approach direction.
    # Keys must match what the TraCI bridge expects.
    # Values are (x1,y1,x2,y2) bounding boxes in pixel coordinates — adjust
    # these to match your actual camera frame layout.
    approach_rois: dict = field(default_factory=lambda: {
        "north": (0,   0,   640, 180),
        "south": (0,   460, 640, 640),
        "east":  (460, 0,   640, 640),
        "west":  (0,   0,   180, 640),
    })


# ──────────────────────────────────────────────────────────────────────────────
# DETECTION RESULT
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float
    class_id: int
    source: str   # "primary" or "secondary"

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2

    def is_in_roi(self, roi: tuple) -> bool:
        rx1, ry1, rx2, ry2 = roi
        return rx1 <= self.cx <= rx2 and ry1 <= self.cy <= ry2


# ──────────────────────────────────────────────────────────────────────────────
# DUAL-MODEL DETECTOR
# ──────────────────────────────────────────────────────────────────────────────
class DualYOLODetector:
    """
    Wraps YOLOv11 (primary) and YOLO26 (secondary) into a single interface.

    Usage
    ─────
        detector = DualYOLODetector(DetectorConfig())
        frame    = cv2.imread("intersection.jpg")
        counts, has_emergency = detector.detect(frame)
        # counts = {"north": 4, "south": 2, "east": 7, "west": 1}
    """

    def __init__(self, cfg: DetectorConfig):
        self.cfg = cfg
        log.info("Loading YOLOv11 Medium from '%s' ...", cfg.primary_weights)
        self.primary   = YOLO(cfg.primary_weights)

        log.info("Loading YOLO26 Medium from '%s' ...", cfg.secondary_weights)
        self.secondary = YOLO(cfg.secondary_weights)

        log.info("Both models loaded on device='%s'", cfg.device)

    # ── Public API ────────────────────────────────────────────────────────────
    def detect(
        self,
        frame: np.ndarray,
    ) -> tuple[dict[str, int], bool]:
        """
        Run dual-model inference on a single frame.

        Returns
        ───────
        counts        : {"north": int, "south": int, "east": int, "west": int}
        has_emergency : True if an emergency vehicle was detected anywhere
        """
        # 1. Primary pass — YOLOv11
        primary_dets = self._run_primary(frame)

        # 2. Secondary pass — YOLO26 on low-confidence detections
        low_conf = [d for d in primary_dets if d.conf < self.cfg.conf_handoff]
        high_conf = [d for d in primary_dets if d.conf >= self.cfg.conf_handoff]

        if low_conf:
            secondary_dets = self._run_secondary(frame)
            # Merge: keep secondary detections that don't overlap high-conf ones
            merged = high_conf + self._merge(high_conf, secondary_dets)
        else:
            merged = primary_dets

        # 3. Count per approach
        counts = self._count_per_approach(merged)

        # 4. Emergency detection
        has_emergency = any(
            d.class_id == self.cfg.emergency_class_id for d in merged
        )

        return counts, has_emergency

    # ── Private helpers ───────────────────────────────────────────────────────
    def _run_primary(self, frame: np.ndarray) -> list[Detection]:
        results = self.primary(
            frame,
            imgsz=self.cfg.imgsz,
            conf=self.cfg.primary_conf,
            device=self.cfg.device,
            verbose=False,
        )
        return self._parse_results(results, source="primary")

    def _run_secondary(self, frame: np.ndarray) -> list[Detection]:
        results = self.secondary(
            frame,
            imgsz=self.cfg.imgsz,
            conf=self.cfg.secondary_conf,
            device=self.cfg.device,
            verbose=False,
        )
        return self._parse_results(results, source="secondary")

    def _parse_results(self, results, source: str) -> list[Detection]:
        dets = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0])
                # Keep only vehicle classes OR emergency class
                if cls_id not in self.cfg.vehicle_classes and \
                   cls_id != self.cfg.emergency_class_id:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                dets.append(Detection(x1, y1, x2, y2, conf, cls_id, source))
        return dets

    def _merge(
        self,
        primary: list[Detection],
        secondary: list[Detection],
        iou_threshold: float = 0.45,
    ) -> list[Detection]:
        """
        Return secondary detections that do NOT overlap with any primary one
        (simple IoU-based deduplication).
        """
        new_dets = []
        for sd in secondary:
            overlaps = any(
                self._iou(sd, pd) > iou_threshold for pd in primary
            )
            if not overlaps:
                new_dets.append(sd)
        return new_dets

    @staticmethod
    def _iou(a: Detection, b: Detection) -> float:
        ix1 = max(a.x1, b.x1)
        iy1 = max(a.y1, b.y1)
        ix2 = min(a.x2, b.x2)
        iy2 = min(a.y2, b.y2)
        inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
        if inter == 0:
            return 0.0
        area_a = (a.x2 - a.x1) * (a.y2 - a.y1)
        area_b = (b.x2 - b.x1) * (b.y2 - b.y1)
        return inter / (area_a + area_b - inter)

    def _count_per_approach(
        self, dets: list[Detection]
    ) -> dict[str, int]:
        counts = {k: 0 for k in self.cfg.approach_rois}
        for d in dets:
            if d.class_id == self.cfg.emergency_class_id:
                continue   # don't count emergency vehicles in traffic counts
            for direction, roi in self.cfg.approach_rois.items():
                if d.is_in_roi(roi):
                    counts[direction] += 1
                    break  # one vehicle → one approach
        return counts


# ──────────────────────────────────────────────────────────────────────────────
# CAMERA / VIDEO SOURCE
# ──────────────────────────────────────────────────────────────────────────────
class FrameSource:
    """
    Thin wrapper around OpenCV VideoCapture.

    Supports:
      - Live camera  : FrameSource(0)
      - Video file   : FrameSource("traffic.mp4")
      - Image folder : FrameSource("frames/")   (sorted glob *.jpg)
    """

    def __init__(self, source):
        if isinstance(source, str) and Path(source).is_dir():
            import glob
            self._frames = sorted(glob.glob(f"{source}/*.jpg") +
                                  glob.glob(f"{source}/*.png"))
            self._idx = 0
            self._cap = None
        else:
            self._cap = cv2.VideoCapture(source)
            self._frames = None

    def read(self) -> Optional[np.ndarray]:
        if self._cap:
            ok, frame = self._cap.read()
            return frame if ok else None
        if self._frames and self._idx < len(self._frames):
            frame = cv2.imread(self._frames[self._idx])
            self._idx += 1
            return frame
        return None

    def release(self):
        if self._cap:
            self._cap.release()


# ──────────────────────────────────────────────────────────────────────────────
# INTEGRATION FUNCTION — plug this into adaptive_traffic_traci_bridge.py
# ──────────────────────────────────────────────────────────────────────────────
_detector: Optional[DualYOLODetector] = None
_source:   Optional[FrameSource]      = None


def init_yolo(
    primary_weights: str   = "yolo11m.pt",
    secondary_weights: str = "yolo26m.pt",
    video_source             = 0,
    device: str            = "auto",
) -> None:
    """
    Call this ONCE before the simulation loop starts.

    device="auto" selects CUDA if available, otherwise falls back to CPU.
    This makes the code work both locally (with GPU) and on Google Colab.

    Example in adaptive_traffic_traci_bridge.py:
        from yolo_detector import init_yolo, get_yolo_vehicle_counts
        init_yolo("yolo11m.pt", "yolo26m.pt", "frames/")
    """
    global _detector, _source

    # Auto-detect device: CUDA if available, else CPU
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    log.info("YOLO running on device: %s", device)

    cfg = DetectorConfig(
        primary_weights=primary_weights,
        secondary_weights=secondary_weights,
        device=device,
    )
    _detector = DualYOLODetector(cfg)
    _source   = FrameSource(video_source)
    log.info("YOLO detector initialised.")


def get_yolo_vehicle_counts(tls_id: str) -> dict[str, int]:
    """
    Drop-in replacement for the stub in adaptive_traffic_traci_bridge.py.

    Returns vehicle counts per approach direction for the given TLS.
    (tls_id is accepted for API compatibility but camera-to-TLS mapping
    should be added here when you have multiple cameras.)
    """
    if _detector is None or _source is None:
        log.warning("YOLO not initialised — returning zeros.")
        return {"north": 0, "south": 0, "east": 0, "west": 0}

    frame = _source.read()
    if frame is None:
        log.warning("No frame available — returning zeros.")
        return {"north": 0, "south": 0, "east": 0, "west": 0}

    counts, has_emergency = _detector.detect(frame)
    if has_emergency:
        log.warning("🚨 Emergency vehicle detected by YOLO at TLS=%s", tls_id)
    return counts