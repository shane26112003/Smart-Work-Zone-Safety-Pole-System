"""
Camera Manager Module: Dual-Camera Feed Capture & Synthetic Simulator.
Supports:
 1. Raspberry Pi 5 MIPI CSI Camera Module (picam2)
 2. USB Webcams / V4L2 devices
 3. Video file playback (looping)
 4. High-fidelity synthetic work-zone renderer for cross-platform simulation
"""

import time
import math
import cv2
import numpy as np
import threading
from typing import Optional, Tuple, Dict, Any
import logging

from ..config import CONFIG

logger = logging.getLogger("CameraManager")


class CameraFeed:
    """Represents a single threaded camera capture stream."""

    def __init__(self, name: str, source: Any, width: int = 640, height: int = 480):
        self.name = name
        self.source = source
        self.width = width
        self.height = height
        
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_count = 0
        self._is_synthetic = False
        self._is_picam2 = False
        self._picam2 = None
        self._cap = None

        # Synthetic animation state (for Road feed)
        self._sim_veh_y = 50.0   # distance in meters
        self._sim_veh_speed = 12.0 # m/s (~43 km/h)
        self._sim_wrk_x = -3.0   # work zone x
        self._sim_wrk_y = 5.0
        self._sim_wrk_vx = 0.0

    def start(self):
        """Starts background frame reader."""
        if self.running:
            return
        self.running = True

        # Check if source is requested as synthetic/mock
        if str(self.source).lower() in ["mock", "synthetic", "sim", "-1"]:
            self._is_synthetic = True
            logger.info(f"Camera '{self.name}' initialized in Synthetic Mode.")
        else:
            # 1. First attempt native Picamera2 if requested or on Raspberry Pi 5 MIPI CSI
            if (CONFIG.camera.USE_PICAMERA2 or str(self.source).lower() == "picam2") and self.name == "Road_Camera":
                try:
                    from picamera2 import Picamera2
                    self._picam2 = Picamera2()
                    # Configure preview/video stream for OV5647
                    config = self._picam2.create_video_configuration(
                        main={"size": (self.width, self.height), "format": "BGR888"}
                    )
                    self._picam2.configure(config)
                    self._picam2.start()
                    # Configure camera tuning controls for OV5647
                    try:
                        self._picam2.set_controls({"AeEnable": True, "AwbEnable": True})
                    except Exception:
                        pass
                    self._is_picam2 = True
                    logger.info(f"Camera '{self.name}' opened via Picamera2 (Rev 1.3 OV5647 CSI).")
                except Exception as e:
                    logger.warning(f"Picamera2 init failed ({e}). Falling back to OpenCV V4L2.")
                    self._is_picam2 = False

            # 2. Fallback to OpenCV VideoCapture
            if not self._is_picam2:
                try:
                    src = int(self.source) if str(self.source).isdigit() else self.source
                    self._cap = cv2.VideoCapture(src)
                    if not self._cap.isOpened():
                        logger.warning(f"Could not open camera '{self.name}' at source '{self.source}'. Falling back to Synthetic Mode.")
                        self._is_synthetic = True
                    else:
                        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                        logger.info(f"Camera '{self.name}' opened device successfully.")
                except Exception as e:
                    logger.warning(f"Error opening camera source: {e}. Using Synthetic Mode.")
                    self._is_synthetic = True

        self._thread = threading.Thread(target=self._capture_loop, daemon=True, name=f"Cam_{self.name}")
        self._thread.start()

    def stop(self):
        """Stops camera stream."""
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._picam2:
            try:
                self._picam2.stop()
                self._picam2.close()
            except Exception:
                pass
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
        logger.info(f"Camera '{self.name}' stopped.")

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Returns the most recent captured frame (copy)."""
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
            return None

    def update_simulation_actors(self, veh_y: float, veh_speed: float, wrk_x: float, wrk_y: float):
        """Allows scenario simulator to drive synthetic camera rendering."""
        with self._lock:
            self._sim_veh_y = veh_y
            self._sim_veh_speed = veh_speed
            self._sim_wrk_x = wrk_x
            self._sim_wrk_y = wrk_y

    def _capture_loop(self):
        fps_delay = 1.0 / CONFIG.camera.TARGET_FPS

        while self.running:
            t0 = time.time()
            frame = None
            if self._is_synthetic:
                frame = self._render_synthetic_scene()
            elif self._is_picam2 and self._picam2 is not None:
                try:
                    frame = self._picam2.capture_array()
                except Exception as e:
                    logger.warning(f"Picamera2 capture error: {e}. Using synthetic fallback.")
                    frame = self._render_synthetic_scene()
            else:
                ret, frame = self._cap.read()
                if not ret or frame is None:
                    # If video file ended, loop back
                    if isinstance(self.source, str) and not self.source.isdigit():
                        self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ret, frame = self._cap.read()
                    if not ret or frame is None:
                        frame = self._render_synthetic_scene()
                else:
                    frame = cv2.resize(frame, (self.width, self.height))

            # Robust frame normalization for AI inference:
            # Picamera2 / libcamera frequently returns 4-channel XBGR8888 or RGBA8888
            if frame is not None:
                if frame.ndim == 3 and frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                elif frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                frame = np.ascontiguousarray(frame, dtype=np.uint8)

            with self._lock:
                self._latest_frame = frame
                self._frame_count += 1

            elapsed = time.time() - t0
            sleep_time = max(0.001, fps_delay - elapsed)
            time.sleep(sleep_time)

    def _render_synthetic_scene(self) -> np.ndarray:
        """
        Renders a photorealistic synthetic road perspective with:
        - Horizon, sky, asphalt road, lane lines
        - Work-zone safety cones dividing lanes
        - High-vis construction worker
        - Approaching vehicle with perspective scaling
        """
        w, h = self.width, self.height
        frame = np.zeros((h, w, 3), dtype=np.uint8)

        # 1. Sky & Ground
        horizon_y = int(h * 0.40)
        frame[:horizon_y, :] = (180, 160, 120) # Sky (BGR)
        frame[horizon_y:, :] = (55, 55, 55)    # Asphalt ground

        # 2. Road Perspective Lines
        # Vanishing point
        vp_x, vp_y = int(w * 0.55), horizon_y
        
        # Road lane boundaries
        pts_road = np.array([
            [vp_x - 30, vp_y],
            [vp_x + 120, vp_y],
            [w, h],
            [int(w * 0.35), h]
        ], np.int32)
        cv2.fillPoly(frame, [pts_road], (45, 45, 45))

        # Lane dashed lines
        for i in range(5):
            t = (i + (time.time() * 3.0) % 1.0) / 5.0
            ly = int(vp_y + t * (h - vp_y))
            lx = int(vp_x + 40 + t * (w * 0.7 - (vp_x + 40)))
            dash_len = int(8 + t * 25)
            cv2.line(frame, (lx, ly), (lx, min(h - 1, ly + dash_len)), (230, 230, 230), max(1, int(t * 4)))

        # Work-zone surface (left side of road)
        pts_workzone = np.array([
            [0, vp_y],
            [vp_x - 30, vp_y],
            [int(w * 0.35), h],
            [0, h]
        ], np.int32)
        cv2.fillPoly(frame, [pts_workzone], (65, 75, 70)) # Gravel / work surface

        # 3. Safety Cones along divider
        for i in range(6):
            t = (i + 1) / 7.0
            cy = int(vp_y + t * (h - vp_y))
            cx = int(vp_x - 30 + t * (int(w * 0.35) - (vp_x - 30)))
            cone_h = int(12 + t * 35)
            cone_w = int(cone_h * 0.6)
            # Orange cone body
            cone_pts = np.array([[cx, cy - cone_h], [cx - cone_w // 2, cy], [cx + cone_w // 2, cy]], np.int32)
            cv2.fillPoly(frame, [cone_pts], (0, 140, 255))
            # White reflective band
            band_y1 = cy - int(cone_h * 0.5)
            band_y2 = cy - int(cone_h * 0.35)
            cv2.line(frame, (cx - cone_w // 4, band_y1), (cx + cone_w // 4, band_y1), (255, 255, 255), max(1, int(cone_h * 0.1)))

        # 4. Render Worker (High-vis vest & hard hat)
        wx, wy = self._sim_wrk_x, self._sim_wrk_y
        # Transform (wx, wy) in meters to image pixels
        norm_y = max(1.0, min(50.0, wy))
        scale = 10.0 / norm_y
        wrk_img_y = int(horizon_y + (1.0 - math.exp(-wy * 0.05)) * (h - horizon_y))
        wrk_img_x = int(vp_x + (wx * 35.0 * scale))
        
        wrk_h = int(70 * scale)
        wrk_w = int(wrk_h * 0.45)
        if 20 <= wrk_img_y < h and 0 <= wrk_img_x < w:
            # Person body (dark trousers)
            cv2.rectangle(frame, 
                          (wrk_img_x - wrk_w // 3, wrk_img_y - wrk_h // 2), 
                          (wrk_img_x + wrk_w // 3, wrk_img_y), 
                          (60, 40, 30), -1)
            # High-vis orange/yellow vest
            cv2.rectangle(frame, 
                          (wrk_img_x - wrk_w // 2, wrk_img_y - wrk_h + int(wrk_h * 0.25)), 
                          (wrk_img_x + wrk_w // 2, wrk_img_y - wrk_h // 2), 
                          (0, 215, 255), -1)
            # Yellow hard hat / head
            head_radius = max(3, int(wrk_h * 0.12))
            cv2.circle(frame, (wrk_img_x, wrk_img_y - wrk_h + head_radius), head_radius, (0, 255, 255), -1)

        # 5. Render Approaching Vehicle
        vy = self._sim_veh_y
        if 2.0 <= vy <= 60.0:
            scale_v = 12.0 / vy
            car_img_y = int(horizon_y + (1.0 - math.exp(-vy * 0.045)) * (h - horizon_y))
            car_img_x = int(vp_x + 60 + (3.0 * 25.0 * scale_v))
            
            car_w = int(140 * scale_v)
            car_h = int(90 * scale_v)
            
            if 0 <= car_img_y < h and 0 <= car_img_x < w:
                # Car body (Metallic blue/red)
                car_top = car_img_y - car_h
                car_left = car_img_x - car_w // 2
                cv2.rectangle(frame, (car_left, car_top + car_h // 3), (car_left + car_w, car_img_y), (180, 50, 40), -1)
                # Windshield / Cabin
                cabin_pts = np.array([
                    [car_left + int(car_w * 0.18), car_top + car_h // 3],
                    [car_left + int(car_w * 0.28), car_top],
                    [car_left + int(car_w * 0.72), car_top],
                    [car_left + int(car_w * 0.82), car_top + car_h // 3]
                ], np.int32)
                cv2.fillPoly(frame, [cabin_pts], (90, 80, 70))
                # Headlights
                hl_r = max(2, int(car_h * 0.1))
                cv2.circle(frame, (car_left + int(car_w * 0.15), car_img_y - hl_r * 2), hl_r, (255, 255, 200), -1)
                cv2.circle(frame, (car_left + int(car_w * 0.85), car_img_y - hl_r * 2), hl_r, (255, 255, 200), -1)

        # HUD Watermark
        cv2.putText(frame, f"{self.name.upper()} [LIVE]", (15, 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, time.strftime("%H:%M:%S"), (w - 110, 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        return frame


class CameraManager:
    """Aggregates and coordinates the dual cameras (Road & Work-zone)."""

    def __init__(self, road_cam_id: Any = None, workzone_cam_id: Any = None):
        rc_id = road_cam_id if road_cam_id is not None else CONFIG.camera.ROAD_CAM_ID
        wz_id = workzone_cam_id if workzone_cam_id is not None else CONFIG.camera.WORKZONE_CAM_ID
        self.road_cam = CameraFeed("Road_Camera", rc_id, 
                                   CONFIG.camera.FRAME_WIDTH, CONFIG.camera.FRAME_HEIGHT)
        self.workzone_cam = CameraFeed("Workzone_Camera", wz_id, 
                                       CONFIG.camera.FRAME_WIDTH, CONFIG.camera.FRAME_HEIGHT)

    def start(self):
        self.road_cam.start()
        self.workzone_cam.start()
        logger.info("CameraManager started both camera feeds.")

    def stop(self):
        self.road_cam.stop()
        self.workzone_cam.stop()
        logger.info("CameraManager stopped.")

    def get_road_frame(self) -> Optional[np.ndarray]:
        return self.road_cam.get_latest_frame()

    def get_workzone_frame(self) -> Optional[np.ndarray]:
        return self.workzone_cam.get_latest_frame()

    def is_road_synthetic(self) -> bool:
        return self.road_cam._is_synthetic if self.road_cam else True

    def update_simulation_actors(self, veh_y: float, veh_speed: float, wrk_x: float, wrk_y: float):
        self.road_cam.update_simulation_actors(veh_y, veh_speed, wrk_x, wrk_y)
        self.workzone_cam.update_simulation_actors(veh_y, veh_speed, wrk_x, wrk_y)
