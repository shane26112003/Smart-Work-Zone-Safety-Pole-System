"""
Hardware Warning Actuator Controller.
Controls:
 - Directional Amber Caution LED Flasher
 - High-Intensity Red Danger Strobe LED
 - 12V Smart Siren / Horn Relay
 - Directional Alert Audio Buzzer
Supports native Raspberry Pi 5 GPIO pins as well as mock software simulation.
"""

import time
import threading
import logging
from typing import Optional

from ..config import CONFIG
from ..risk.risk_engine import RiskAssessment

logger = logging.getLogger("AlertController")


class AlertController:
    """
    Manages physical warning outputs on the Safety Pole.
    Ensures fail-safe state transitions and thread-safe actuator strobing.
    """

    def __init__(self):
        self.use_real_gpio = CONFIG.warning.USE_REAL_GPIO
        self._gpio_initialized = False

        # State
        self.current_risk_level = "SAFE"
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Actuator states
        self.caution_led_on = False
        self.danger_led_on = False
        self.siren_active = False
        self.buzzer_active = False

        self._init_gpio()

    def _init_gpio(self):
        """Initializes Raspberry Pi GPIO pins if available."""
        if not self.use_real_gpio:
            logger.info("AlertController initialized in Simulation/Mock Mode.")
            return

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(CONFIG.warning.PIN_CAUTION_LED, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(CONFIG.warning.PIN_DANGER_LED, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(CONFIG.warning.PIN_SIREN_RELAY, GPIO.OUT, initial=GPIO.HIGH) # Active LOW relay
            GPIO.setup(CONFIG.warning.PIN_BUZZER, GPIO.OUT, initial=GPIO.LOW)
            self._gpio_initialized = True
            logger.info("Raspberry Pi 5 GPIO initialized successfully.")
        except Exception as e:
            logger.warning(f"RPi.GPIO not accessible ({e}). Running in Software Mock Mode.")
            self._gpio_initialized = False

    def start(self):
        """Starts background actuator strobe loop."""
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._strobe_loop, daemon=True, name="ActuatorLoop")
        self._thread.start()
        logger.info("AlertController background loop active.")

    def stop(self):
        """Disables all actuators and stops thread."""
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._set_hardware(caution=False, danger=False, siren=False, buzzer=False)
        logger.info("AlertController stopped.")

    def update(self, assessment: RiskAssessment):
        """Updates target risk level."""
        with self._lock:
            self.current_risk_level = assessment.overall_level

    def _set_hardware(self, caution: bool, danger: bool, siren: bool, buzzer: bool):
        self.caution_led_on = caution
        self.danger_led_on = danger
        self.siren_active = siren
        self.buzzer_active = buzzer

        if self._gpio_initialized:
            try:
                import RPi.GPIO as GPIO
                GPIO.output(CONFIG.warning.PIN_CAUTION_LED, GPIO.HIGH if caution else GPIO.LOW)
                GPIO.output(CONFIG.warning.PIN_DANGER_LED, GPIO.HIGH if danger else GPIO.LOW)
                # Active LOW relay for 12V siren: LOW = ON, HIGH = OFF
                GPIO.output(CONFIG.warning.PIN_SIREN_RELAY, GPIO.LOW if siren else GPIO.HIGH)
                GPIO.output(CONFIG.warning.PIN_BUZZER, GPIO.HIGH if buzzer else GPIO.LOW)
            except Exception as e:
                logger.debug(f"GPIO output write error: {e}")

    def _strobe_loop(self):
        """Continuous strobe pattern generator based on risk level."""
        strobe_counter = 0

        while self.running:
            with self._lock:
                level = self.current_risk_level

            strobe_counter = (strobe_counter + 1) % 20

            if level == "SAFE":
                self._set_hardware(caution=False, danger=False, siren=False, buzzer=False)
                time.sleep(0.1)

            elif level == "CAUTION":
                # 1 Hz Amber blink
                blink = (strobe_counter % 10 < 5)
                self._set_hardware(caution=blink, danger=False, siren=False, buzzer=False)
                time.sleep(0.1)

            elif level == "HIGH RISK":
                # Rapid 5 Hz alternating strobe + intermittent buzzer
                blink = (strobe_counter % 2 == 0)
                buzz = (strobe_counter % 4 < 2)
                self._set_hardware(caution=blink, danger=(not blink), siren=False, buzzer=buzz)
                time.sleep(0.05)

            elif level == "CRITICAL":
                # Full emergency mode: High speed 10 Hz strobe + continuous 12V Siren + Buzzer
                blink = (strobe_counter % 2 == 0)
                self._set_hardware(caution=True, danger=blink, siren=True, buzzer=True)
                time.sleep(0.05)

            else:
                time.sleep(0.1)
