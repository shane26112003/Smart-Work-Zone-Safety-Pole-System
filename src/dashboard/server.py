"""
Real-Time Tactical Dashboard Server (Tornado Web & WebSockets).
Streams unified work-zone state, radar coordinates, dual video feeds, and actuator status
to web browsers at 20+ FPS.
"""

import os
import json
import base64
import cv2
import time
import asyncio
import threading
from typing import Set, Optional, Dict, Any
import logging

import tornado.web
import tornado.websocket
import tornado.ioloop

from ..config import CONFIG
from ..risk.risk_engine import RiskAssessment
from ..fusion.ekf_fusion import FusedEntity
from ..sensors.lidar_tsd20 import LidarReading
from ..sensors.worker_receiver import WorkerState

logger = logging.getLogger("DashboardServer")

# Global set of connected WebSocket clients
connected_clients: Set[tornado.websocket.WebSocketHandler] = set()

# Global state reference for broadcast
latest_dashboard_payload: Dict[str, Any] = {}
payload_lock = threading.Lock()
scenario_switch_callback = None


class WebSocketHandler(tornado.websocket.WebSocketHandler):
    """Handles bidirectional real-time telemetry streaming with browsers."""

    def check_origin(self, origin):
        return True # Allow cross-origin for local development

    def open(self):
        connected_clients.add(self)
        logger.info(f"Dashboard client connected: {self.request.remote_ip} (Total: {len(connected_clients)})")
        # Send initial snapshot immediately
        with payload_lock:
            if latest_dashboard_payload:
                self.write_message(json.dumps(latest_dashboard_payload))

    def on_close(self):
        connected_clients.discard(self)
        logger.info(f"Dashboard client disconnected. (Remaining: {len(connected_clients)})")

    def on_message(self, message):
        """Handle incoming commands from dashboard (e.g. scenario trigger)."""
        try:
            cmd = json.loads(message)
            action = cmd.get("action")
            if action == "set_scenario" and scenario_switch_callback:
                scen_name = cmd.get("scenario")
                scenario_switch_callback(scen_name)
        except Exception as e:
            logger.debug(f"WS message error: {e}")


class MainHandler(tornado.web.RequestHandler):
    """Serves the primary Single-Page Application (SPA)."""
    def get(self):
        self.render("index.html")


class ApiScenarioHandler(tornado.web.RequestHandler):
    """REST API endpoint to trigger simulation scenarios."""
    def post(self):
        try:
            data = json.loads(self.request.body.decode("utf-8"))
            scen = data.get("scenario", "normal")
            if scenario_switch_callback:
                scenario_switch_callback(scen)
                self.write({"status": "ok", "scenario": scen})
            else:
                self.write({"status": "no_simulator"})
        except Exception as e:
            self.set_status(400)
            self.write({"error": str(e)})


class DashboardServer:
    """Manages the background Tornado HTTP & WebSocket server."""

    def __init__(self, port: Optional[int] = None):
        self.port = port or CONFIG.network.DASHBOARD_PORT
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self._ioloop: Optional[tornado.ioloop.IOLoop] = None

    def start(self, scenario_callback=None):
        """Starts the web server in a dedicated background thread."""
        global scenario_switch_callback
        scenario_switch_callback = scenario_callback

        if self.running:
            return
        self.running = True

        self._thread = threading.Thread(target=self._run_server, daemon=True, name="DashboardServer")
        self._thread.start()
        logger.info(f"Dashboard Web Server active at http://localhost:{self.port}")

    def stop(self):
        """Stops web server."""
        self.running = False
        if self._ioloop:
            self._ioloop.add_callback(self._ioloop.stop)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        logger.info("Dashboard Web Server stopped.")

    def update_telemetry(self, 
                         assessment: RiskAssessment, 
                         entities: list, 
                         lidar: LidarReading, 
                         workers: list, 
                         road_frame: Optional[any] = None,
                         workzone_frame: Optional[any] = None,
                         actuators: Optional[dict] = None):
        """
        Packs complete system state and broadcasts to all connected WebSockets.
        """
        global latest_dashboard_payload

        # Encode camera frames to JPEG Base64
        road_b64 = None
        workzone_b64 = None
        
        try:
            if road_frame is not None:
                # Downsample slightly for ultra-fast network transmission (e.g. 480x360)
                small_road = cv2.resize(road_frame, (480, 360))
                _, buf1 = cv2.imencode('.jpg', small_road, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                road_b64 = base64.b64encode(buf1).decode('utf-8')

            if workzone_frame is not None:
                small_wz = cv2.resize(workzone_frame, (480, 360))
                _, buf2 = cv2.imencode('.jpg', small_wz, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                workzone_b64 = base64.b64encode(buf2).decode('utf-8')
        except Exception as e:
            logger.debug(f"Image encode error: {e}")

        # Serialize entities
        serialized_entities = []
        for e in entities:
            serialized_entities.append({
                "id": e.entity_id,
                "type": e.entity_type,
                "x": e.x,
                "y": e.y,
                "vx": e.vx,
                "vy": e.vy,
                "speed_kmh": e.speed_kmh,
                "heading_deg": e.heading_deg,
                "trajectory": e.predicted_trajectory,
                "lidar_verified": e.lidar_verified,
                "lidar_dist": e.lidar_distance_m,
                "worker_state": {
                    "motion_state": e.worker_state.motion_state,
                    "svm": round(e.worker_state.svm, 2),
                    "battery": round(e.worker_state.battery_v, 2),
                    "rssi": e.worker_state.rssi
                } if e.worker_state else None
            })

        # Serialize workers
        serialized_workers = []
        for w in workers:
            serialized_workers.append({
                "worker_id": w.worker_id,
                "motion_state": w.motion_state,
                "svm": round(w.svm, 2),
                "battery": round(w.battery_v, 2),
                "rssi": w.rssi,
                "est_dist_m": round(w.estimated_dist_m, 1),
                "last_seen_sec": round(time.time() - w.last_seen, 1)
            })

        # Build payload
        payload = {
            "timestamp": time.time(),
            "overall_level": assessment.overall_level,
            "min_ttc_sec": assessment.min_ttc_sec,
            "min_distance_m": assessment.min_distance_m,
            "critical_pair": assessment.critical_pair,
            "warning_type": assessment.warning_type,
            "warning_active": assessment.warning_active,
            "reasons": [r for p in assessment.active_pairs if p.risk_level == assessment.overall_level for r in p.reasons],
            "entities": serialized_entities,
            "workers": serialized_workers,
            "lidar": {
                "distance_m": round(lidar.distance_m, 2),
                "range_rate_mps": round(lidar.range_rate_mps, 2),
                "valid": lidar.valid,
                "strength": lidar.signal_strength
            },
            "actuators": actuators or {
                "caution_led": False,
                "danger_led": False,
                "siren": False,
                "buzzer": False
            },
            "guidance": {
                "target_worker": assessment.guidance.target_worker_id,
                "action": assessment.guidance.action_code,
                "directive": assessment.guidance.directive_text,
                "urgency": assessment.guidance.urgency,
                "escape_dx": assessment.guidance.escape_vector[0],
                "escape_dy": assessment.guidance.escape_vector[1],
                "safe_x": assessment.guidance.target_safe_pos[0],
                "safe_y": assessment.guidance.target_safe_pos[1],
                "haptic": assessment.guidance.haptic_pattern
            } if assessment.guidance else None,
            "road_image": road_b64,
            "workzone_image": workzone_b64
        }

        with payload_lock:
            latest_dashboard_payload = payload

        # Broadcast via tornado ioloop thread-safely
        if self._ioloop and connected_clients:
            msg_str = json.dumps(payload)
            self._ioloop.add_callback(self._broadcast, msg_str)

    def _broadcast(self, msg_str: str):
        dead_clients = set()
        for client in connected_clients:
            try:
                client.write_message(msg_str)
            except Exception:
                dead_clients.add(client)
        connected_clients.difference_update(dead_clients)

    def _run_server(self):
        asyncio.set_event_loop(asyncio.new_event_loop())
        self._ioloop = tornado.ioloop.IOLoop.current()

        static_path = os.path.join(os.path.dirname(__file__), "static")
        app = tornado.web.Application([
            (r"/", MainHandler),
            (r"/ws", WebSocketHandler),
            (r"/api/scenario", ApiScenarioHandler),
            (r"/static/(.*)", tornado.web.StaticFileHandler, {"path": static_path})
        ], template_path=static_path, static_path=static_path, debug=False)

        app.listen(self.port, address=CONFIG.network.HOST)
        self._ioloop.start()
