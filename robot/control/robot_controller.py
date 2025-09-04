"""
robot_controller.py  (patched)
Controller principale che integra visione, controllo e comunicazione

Modifiche principali:
- Uso reale dei PID: pid_forward (forward projection) + pid_angular (errore angolare)
- PID con anti-windup tramite clamp dell'integrale e derivata filtrata
- Calcolo dt esplicito in control_loop_step
- Mapping coerente v,omega -> joystick_x/joystick_y (joystick_y = forward, joystick_x = rotation)
- Rimosso swap/bug degli assi
- Evita chiamate ripetute a emergency_stop usando flag _in_emergency
- Fallback ai valori di ControlConfig se mancano alcune costanti
"""

from ast import Dict
import asyncio
import time
import math
import logging
from typing import Optional, Tuple
from dataclasses import dataclass

from config import ControlConfig, SystemConfig, VisionConfig
from robot.comm.robot_communication import RobotCommunication, joystick_to_differential_drive


@dataclass
class RobotState:
    """Stato corrente del robot"""
    x: float = 0.0              # Posizione X arena [0-100]
    y: float = 0.0              # Posizione Y arena [0-100]
    theta: float = 0.0          # Orientamento gradi [0-360]
    timestamp: float = 0.0      # Timestamp ultima misura
    tracking_lost: bool = True  # True se tracking perso


@dataclass
class TargetState:
    """Stato target desiderato"""
    x: float = 50.0             # Posizione X target [0-100]
    y: float = 50.0             # Posizione Y target [0-100]
    theta: float = 0.0          # Orientamento target gradi [0-360] (opzionale)
    tolerance: float = ControlConfig.POSITION_TOLERANCE  # Tolleranza arrivo



class PIDController:
    """PID controller migliorato con anti-windup e derivata filtrata.

    update(error, dt) -> output

    Parametri:
        kp, ki, kd: guadagni
        max_output: saturazione dell'output (valore positivo)
        integral_limit: massimo valore assoluto consentito per l'integrale (se None calcolato automaticamente)
        d_filter_alpha: fattore per filtrare la derivata [0..1], 0=no filtering (uso raw), 1=derivata sempre precedente
    """
    def __init__(self, kp: float, ki: float, kd: float, max_output: float = 1.0,
                 integral_limit: Optional[float] = None, d_filter_alpha: float = 0.8):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = abs(max_output) if max_output is not None else 1.0

        # anti-windup limit for integral
        if integral_limit is None:
            # se ki==0 non vogliamo dividere per zero
            eps = 1e-8
            self.integral_limit = self.max_output / max(abs(self.ki), eps)
        else:
            self.integral_limit = abs(integral_limit)

        # derivata filtrata
        self.d_filter_alpha = float(d_filter_alpha)

        # stato interno
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        # P
        p = self.kp * error

        # I con anti-windup tramite clamp dell'integrale
        self.integral += error * dt
        # clamp
        if self.integral_limit is not None:
            if self.integral > self.integral_limit:
                self.integral = self.integral_limit
            elif self.integral < -self.integral_limit:
                self.integral = -self.integral_limit
        i = self.ki * self.integral

        # D (derivata dell'errore) con filtro esponenziale
        raw_d = (error - self.prev_error) / dt
        d = self.d_filter_alpha * self.prev_derivative + (1.0 - self.d_filter_alpha) * raw_d
        self.prev_derivative = d
        d_term = self.kd * d

        # output e saturazione
        out = p + i + d_term
        out = max(-self.max_output, min(self.max_output, out))

        # salva
        self.prev_error = error
        return out

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0


class RobotController:
    """Controller principale del robot"""

    def __init__(self):
        # Stati
        self.robot_state = RobotState()
        self.target_state = TargetState()

        # Comunicazione
        self.communication = RobotCommunication()

        # Config / fallback
        self.max_speed = ControlConfig.MAX_SPEED
        self.vector_distance_scale = ControlConfig.VECTOR_DISTANCE_SCALE
        self.max_omega = ControlConfig.MAX_OMEGA

       # Controllori PID
        self.pid_forward = PIDController(
            ControlConfig.PID_FORWARD_KP,
            ControlConfig.PID_FORWARD_KI, 
            ControlConfig.PID_FORWARD_KD,
            ControlConfig.MAX_SPEED
        )
        
        self.pid_angular = PIDController(
            ControlConfig.PID_ANGULAR_KP,
            ControlConfig.PID_ANGULAR_KI,
            ControlConfig.PID_ANGULAR_KD,
            ControlConfig.MAX_SPEED
        )

        # Stato controllo
        self.control_enabled = False
        self.last_control_time = time.time()
        self.tracking_lost_count = 0

        # Flag per emergenza (evita chiamate ripetute a emergency_stop)
        self._in_emergency = False

        # Filtro per smoothing velocità angolare (opzionale, non usato direttamente ora)
        self.last_angular_speed = 0.0
        self.angular_smooth_factor = 0.7  # Fattore di smoothing [0-1]

        # Variabili per debug e visualizzazione vettore
        self.last_vector_intensity = 0.0
        self.last_vector_angle = 0.0  # In radianti, sistema robot
        self.last_target_distance = 0.0

        # Logger
        self.logger = logging.getLogger(__name__)

        # Setup callbacks comunicazione
        self.communication.on_connected = self._on_robot_connected
        self.communication.on_disconnected = self._on_robot_disconnected
        self.communication.on_error = self._on_robot_error

    # ---------------- communication callbacks (placeholder) ----------------
    def _on_robot_connected(self):
        self.logger.info("Robot connected")

    def _on_robot_disconnected(self):
        self.logger.warning("Robot disconnected")

    def _on_robot_error(self, err):
        self.logger.error(f"Robot communication error: {err}")

    # ---------------- init / connection ----------------
    async def initialize(self) -> bool:
        self.logger.info("Inizializzando robot controller...")
        success = await self.communication.connect()
        if not success:
            self.logger.error("Impossibile connettersi al robot")
            return False
        self.logger.info("Controller inizializzato con successo")
        return True

    # ---------------- state updates ----------------
    def update_robot_state(self, x: float, y: float, theta: float, tracking_valid: bool = True):
        """Aggiorna stato robot da sistema di visione"""
        self.robot_state.x = x
        self.robot_state.y = y
        self.robot_state.theta = theta % 360
        self.robot_state.timestamp = time.time()
        self.robot_state.tracking_lost = not tracking_valid

        if tracking_valid:
            self.tracking_lost_count = 0
            # clear emergency if had been triggered
            if self._in_emergency:
                self._in_emergency = False
        else:
            prev = self.tracking_lost_count
            self.tracking_lost_count += 1
            # Se entriamo nello stato di emergenza, chiama emergency_stop una sola volta
            if prev <= VisionConfig.MAX_TRACKING_LOSS and self.tracking_lost_count > VisionConfig.MAX_TRACKING_LOSS:
                self.logger.warning("Tracking perso per troppo tempo, stop sicurezza")
                # create_task così non blocca il thread corrente
                asyncio.create_task(self.emergency_stop())
                self._in_emergency = True

    # ---------------- target ----------------
    def set_target_position(self, x: float, y: float, theta: float = None):
        """Imposta posizione target con clamp nei limiti dell'arena"""
        margin = ControlConfig.BOUNDARY_MARGIN
        margin = margin if margin is not None else 0.0
        self.target_state.x = max(margin, min(100 - margin, x))
        self.target_state.y = max(margin, min(100 - margin, y))

        if theta is not None:
            self.target_state.theta = theta % 360

        self.logger.info(f"Nuovo target: ({self.target_state.x:.1f}, {self.target_state.y:.1f}, {self.target_state.theta:.1f}°)")

        # Reset PID per nuovo target
        self.pid_forward.reset()
        self.pid_angular.reset()

        # Reset anche filtro smoothing angolare
        self.last_angular_speed = 0.0

    # ---------------- control computation ----------------
    def calculate_control_output(self, dt: float) -> Tuple[float, float]:
        """
        Calcola joystick (x: rotazione, y: avanti) normalizzati in [-1,1].
        - Usa PID su forward projection (errore lungo asse avanti del robot)
        - Usa PID su errore angolare
        """
        if self.robot_state.tracking_lost:
            return 0.0, 0.0

        # vettore verso target nel mondo
        ex = self.target_state.x - self.robot_state.x
        ey = self.target_state.y - self.robot_state.y
        distance = math.hypot(ex, ey)
        if distance < self.target_state.tolerance:
            return 0.0, 0.0

        # angolo verso target (mondo) e angolo robot
        target_angle = math.atan2(ey, ex)
        robot_theta = math.radians(self.robot_state.theta)
        # errore angolare in [-pi,pi]
        angle_error = target_angle - robot_theta
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        # proiezione della distanza sull'asse frontale del robot
        forward_error = distance * math.cos(angle_error)

        # PID per forward e angular
        v = self.pid_forward.update(forward_error, dt)   # output in range [-max_speed, +max_speed]
        omega = self.pid_angular.update(angle_error, dt) # output in range [-max_omega, +max_omega]

        # Map to normalized joystick [-1,1]
        j_y = 0.0
        j_x = 0.0
        if self.max_speed != 0:
            j_y = v / self.max_speed
        if self.max_omega != 0:
            j_x = omega / self.max_omega

        # final clamp
        j_y = max(-1.0, min(1.0, j_y))
        j_x = max(-1.0, min(1.0, j_x))

        # salva per debug
        self.last_vector_intensity = abs(j_y)
        self.last_vector_angle = angle_error
        self.last_target_distance = distance

        return j_x, j_y

    # ---------------- main control loop step ----------------
    async def control_loop_step(self):
        """Singolo step del loop di controllo"""
        if not self.control_enabled or not self.communication.is_connected():
            return

        now = time.time()
        dt = now - getattr(self, 'last_control_time', now)
        # protezione su dt
        if dt <= 0:
            dt = 1e-3
        self.last_control_time = now

        joystick_x, joystick_y = self.calculate_control_output(dt)

        # Log/debug con rate
        self._debug_counter = getattr(self, '_debug_counter', 0) + 1
        if self._debug_counter % 10 == 0:
            ex = self.target_state.x - self.robot_state.x
            ey = self.target_state.y - self.robot_state.y
            distance = math.hypot(ex, ey)
            self.logger.info(f"🎮 Vector Control: dist={distance:.2f}, joystick_x={joystick_x:.3f}, joystick_y={joystick_y:.3f}")

        # Invia comando (communication si aspetta normalized [-1,1])
        success = await self.communication.send_movement_command(joystick_x, joystick_y)

        if not success:
            self.logger.warning("Fallito invio comando movimento")

    # ---------------- helpers / checks ----------------
    def is_at_target(self) -> bool:
        if self.robot_state.tracking_lost:
            return False
        ex = self.target_state.x - self.robot_state.x
        ey = self.target_state.y - self.robot_state.y
        distance = math.hypot(ex, ey)
        return distance <= self.target_state.tolerance

    # ---------------- high level motion primitives ----------------
    async def go_to_position(self, x: float, y: float, timeout: float = 30.0) -> bool:
        self.set_target_position(x, y)
        self.control_enabled = True
        start_time = time.time()

        try:
            while time.time() - start_time < timeout:
                await self.control_loop_step()
                if self.is_at_target():
                    self.logger.info(f"Target raggiunto: ({x:.1f}, {y:.1f})")
                    await self.stop()
                    return True
                await asyncio.sleep(1.0 / getattr(SystemConfig, 'CONTROL_LOOP_RATE', 30))

            self.logger.warning(f"Timeout raggiungimento target: ({x:.1f}, {y:.1f})")
            await self.stop()
            return False

        except Exception as e:
            self.logger.error(f"Errore durante movimento: {e}")
            await self.emergency_stop()
            return False

    async def start_continuous_control(self):
        self.control_enabled = True
        self.logger.info("Controllo continuo avviato")

    async def stop(self):
        self.control_enabled = False
        await self.communication.send_stop_command()
        self.logger.info("Robot fermato")

    async def emergency_stop(self):
        if self._in_emergency:
            # se già in emergenza non eseguire doppio
            return
        self._in_emergency = True
        self.control_enabled = False
        await self.communication.send_emergency_stop()
        self.logger.warning("STOP DI EMERGENZA")

    async def send_manual_command(self, joystick_x: float, joystick_y: float):
        """Invia comando manuale diretto per calibrazione"""
        if not self.communication.is_connected():
            self.logger.warning("Robot non connesso per comando manuale")
            return False

        # Applica limiti di sicurezza normalized [-1,1]
        joystick_x = max(-1.0, min(1.0, joystick_x))
        joystick_y = max(-1.0, min(1.0, joystick_y))

        success = await self.communication.send_movement_command(joystick_x, joystick_y)

        if success:
            self.logger.debug(f"🎮 Comando manuale: x={joystick_x:.2f}, y={joystick_y:.2f}")
        else:
            self.logger.warning("Fallito invio comando manuale")

        return success

    def get_vector_info(self) -> Dict:
            """Ritorna informazioni del vettore di controllo per visualizzazione"""
            return {
                "intensity": getattr(self, 'last_vector_intensity', 0.0),
                "angle_rad": getattr(self, 'last_vector_angle', 0.0),
                "distance": getattr(self, 'last_target_distance', 0.0),
                "control_enabled": self.control_enabled
            }
        
    def get_status(self) -> Dict:
        """Ritorna stato completo del sistema"""
        return {
            "robot_state": {
                "position": (self.robot_state.x, self.robot_state.y),
                "theta": self.robot_state.theta,
                "tracking_ok": not self.robot_state.tracking_lost,
                "timestamp": self.robot_state.timestamp
            },
            "target_state": {
                "position": (self.target_state.x, self.target_state.y), 
                "theta": self.target_state.theta,
                "tolerance": self.target_state.tolerance
            },
            "control": {
                "enabled": self.control_enabled,
                "at_target": self.is_at_target(),
                "tracking_lost_count": self.tracking_lost_count
            },
            "communication": self.communication.get_connection_info()
        }