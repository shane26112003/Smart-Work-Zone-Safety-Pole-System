"""
Computer Vision & Multi-Object Tracking Package.
"""
from .detector import ObjectDetector, Detection
from .tracker import MultiObjectTracker, TrackedObject

__all__ = ["ObjectDetector", "Detection", "MultiObjectTracker", "TrackedObject"]
