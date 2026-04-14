#!/usr/bin/env python3
import os
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class Detection:
    kind: str
    family: str | None = None
    center: tuple[float, float] | None = None
    confidence: float | None = None
    tag_id: int | None = None
    hamming: int | None = None
    corners: list[tuple[float, float]] = field(default_factory=list)
    raw: Any = None


class BaseDetector:
    name = "none"

    def detect(self, frame):
        return []

    def info(self):
        return {"name": self.name}


class NoOpDetector(BaseDetector):
    pass


class AprilTagDetector(BaseDetector):
    name = "apriltag"

    def __init__(
        self,
        family: str | None = None,
        nthreads: int | None = None,
        max_hamming: int | None = None,
        decimate: float | None = None,
        blur: float | None = None,
        refine_edges: bool | None = None,
        debug: bool | None = None,
    ):
        try:
            import apriltag
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "AprilTag detector is unavailable. Install python3-apriltag on the Pi first."
            ) from exc

        self.family = family or os.environ.get("APRILTAG_FAMILY", "tag36h11")
        self.nthreads = int(nthreads or os.environ.get("APRILTAG_NTHREADS", "2"))
        self.max_hamming = int(
            max_hamming or os.environ.get("APRILTAG_MAX_HAMMING", "1")
        )
        self.decimate = float(
            decimate or os.environ.get("APRILTAG_DECIMATE", "1.0")
        )
        self.blur = float(blur or os.environ.get("APRILTAG_BLUR", "0.0"))
        self.refine_edges = _env_bool(
            "APRILTAG_REFINE_EDGES",
            True if refine_edges is None else refine_edges,
        )
        self.debug = _env_bool("APRILTAG_DEBUG", False if debug is None else debug)
        self._apriltag = apriltag
        self.backend = _detect_backend(apriltag)
        if self.backend == "detector_options":
            self.detector = apriltag.Detector(
                apriltag.DetectorOptions(
                    families=self.family,
                    nthreads=self.nthreads,
                    quad_decimate=self.decimate,
                    quad_sigma=self.blur,
                    refine_edges=int(self.refine_edges),
                )
            )
        elif self.backend == "legacy_builtin":
            self.detector = apriltag.apriltag(
                self.family,
                self.nthreads,
                self.max_hamming,
                self.decimate,
                self.blur,
                int(self.refine_edges),
                int(self.debug),
            )
        else:
            raise RuntimeError("Unsupported apriltag Python API")

    def detect(self, frame):
        gray = _to_grayscale(frame)
        results = []
        for item in self.detector.detect(gray):
            results.append(self._convert_detection(item))
        return results

    def info(self):
        return {
            "name": self.name,
            "family": self.family,
            "backend": self.backend,
            "nthreads": self.nthreads,
            "max_hamming": self.max_hamming,
            "decimate": self.decimate,
            "blur": self.blur,
            "refine_edges": self.refine_edges,
            "debug": self.debug,
        }

    def _convert_detection(self, item):
        if isinstance(item, dict):
            corners = [
                _as_point(item.get("lb-rb-rt-lt", ())[index])
                for index in range(len(item.get("lb-rb-rt-lt", ())))
            ]
            return Detection(
                kind=self.name,
                family=self.family,
                center=_as_point(item.get("center")),
                confidence=float(item.get("margin", 0.0)),
                tag_id=int(item.get("id", -1)),
                hamming=int(item.get("hamming", -1)),
                corners=corners,
                raw=item,
            )

        return Detection(
            kind=self.name,
            family=self.family,
            center=(float(item.center[0]), float(item.center[1])),
            confidence=float(getattr(item, "decision_margin", 0.0)),
            tag_id=int(getattr(item, "tag_id", -1)),
            hamming=int(getattr(item, "hamming", -1)),
            corners=[(float(x), float(y)) for x, y in item.corners],
            raw=item,
        )


def _to_grayscale(frame):
    array = np.asarray(frame)
    if array.ndim == 2:
        return array
    if array.ndim == 3:
        rgb = array[..., :3].astype(np.float32)
        gray = 0.114 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.299 * rgb[..., 2]
        return gray.astype(np.uint8)
    raise ValueError("Unsupported frame format for detection")


def build_detector(name=None):
    if not name or name == "none":
        return NoOpDetector()
    if name == "apriltag":
        return AprilTagDetector()
    raise ValueError(f"Unsupported detector: {name}")


def _env_bool(name, default):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _detect_backend(module):
    if hasattr(module, "DetectorOptions") and hasattr(module, "Detector"):
        return "detector_options"
    if hasattr(module, "apriltag"):
        return "legacy_builtin"
    return "unknown"


def _as_point(value):
    if value is None:
        return None
    return (float(value[0]), float(value[1]))
