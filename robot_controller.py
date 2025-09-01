"""
robot_controller.py
Controller principale che integra visione, controllo e comunicazione
"""

import asyncio
import time
import math
import logging
from typing import Optional, Tuple, Dict, List
from dataclasses import dataclass

from config import ControlConfig, SystemConfig, VisionConfig
from robot_communication import RobotCommunication, joystick_to_differential_drive

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
    """Controllo PID per posizione e orientamento"""
    
    def __init__(self, kp: float, ki: float, kd: float, max_output: float = 1.0):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        self.max_output = max_output
        
        # Stato interno PID
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = 0.0
        
    def update(self, error: float, dt: float) -> float:
        """Calcola output PID dato errore e delta tempo"""
        if dt <= 0:
            return 0.0
            
        # Termine proporzionale
        proportional = self.kp * error
        
        # Termine integrativo (con anti-windup)
        self.integral += error * dt
        # Anti-windup: limita integrale se output è saturato
        if abs(proportional + self.ki * self.integral) > self.max_output:
            self.integral -= error * dt  # Annulla ultimo contributo
        integral_term = self.ki * self.integral
        
        # Termine derivativo
        derivative = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        
        # Output totale
        output = proportional + integral_term + derivative
        output = max(-self.max_output, min(self.max_output, output))
        
        # Salva per prossima iterazione
        self.prev_error = error
        
        return output
    
    def reset(self):
        """Reset stato PID"""
        self.prev_error = 0.0
        self.integral = 0.0

class RobotController:
    """Controller principale del robot"""
    
    def __init__(self):
        # Stati
        self.robot_state = RobotState()
        self.target_state = TargetState()
        
        # Comunicazione
        self.communication = RobotCommunication()
        
        # Controllori PID
        self.pid_linear = PIDController(
            ControlConfig.PID_LINEAR_KP,
            ControlConfig.PID_LINEAR_KI, 
            ControlConfig.PID_LINEAR_KD,
            ControlConfig.MAX_LINEAR_SPEED
        )
        
        self.pid_angular = PIDController(
            ControlConfig.PID_ANGULAR_KP,
            ControlConfig.PID_ANGULAR_KI,
            ControlConfig.PID_ANGULAR_KD, 
            ControlConfig.MAX_ANGULAR_SPEED
        )
        
        # Stato controllo
        self.control_enabled = False
        self.last_control_time = 0.0
        self.tracking_lost_count = 0
        
        # Filtro per smoothing velocità angolare (anti-oscillazione)
        self.last_angular_speed = 0.0
        self.angular_smooth_factor = 0.7  # Fattore di smoothing [0-1]
        
        # Logger
        self.logger = logging.getLogger(__name__)
        
        # Setup callbacks comunicazione
        self.communication.on_connected = self._on_robot_connected
        self.communication.on_disconnected = self._on_robot_disconnected
        self.communication.on_error = self._on_robot_error
    
    async def initialize(self) -> bool:
        """Inizializza il controller e la connessione"""
        self.logger.info("Inizializzando robot controller...")
        
        # Connetti al robot
        success = await self.communication.connect()
        if not success:
            self.logger.error("Impossibile connettersi al robot")
            return False
            
        self.logger.info("Controller inizializzato con successo")
        return True
    
    def update_robot_state(self, x: float, y: float, theta: float, tracking_valid: bool = True):
        """Aggiorna stato robot da sistema di visione"""
        self.robot_state.x = x
        self.robot_state.y = y
        self.robot_state.theta = theta
        self.robot_state.timestamp = time.time()
        self.robot_state.tracking_lost = not tracking_valid
        
        if tracking_valid:
            self.tracking_lost_count = 0
        else:
            self.tracking_lost_count += 1
            
        # Se tracking perso per troppo tempo, stop di sicurezza
        if self.tracking_lost_count > VisionConfig.MAX_TRACKING_LOSS:
            self.logger.warning("Tracking perso per troppo tempo, stop sicurezza")
            asyncio.create_task(self.emergency_stop())
    
    def set_target_position(self, x: float, y: float, theta: float = None):
        """Imposta posizione target"""
        # Clamp target dentro arena con margini di sicurezza
        margin = ControlConfig.BOUNDARY_MARGIN
        self.target_state.x = max(margin, min(100 - margin, x))
        self.target_state.y = max(margin, min(100 - margin, y))
        
        if theta is not None:
            self.target_state.theta = theta % 360
            
        self.logger.info(f"Nuovo target: ({self.target_state.x:.1f}, {self.target_state.y:.1f}, {self.target_state.theta:.1f}°)")
        
        # Reset PID per nuovo target
        self.pid_linear.reset()
        self.pid_angular.reset()
        
        # Reset anche filtro smoothing angolare
        self.last_angular_speed = 0.0
    
    def calculate_control_output(self) -> Tuple[float, float]:
        """Calcola output di controllo (linear_speed, angular_speed)"""
        if self.robot_state.tracking_lost:
            return 0.0, 0.0
            
        # Calcola errori di posizione
        error_x = self.target_state.x - self.robot_state.x
        error_y = self.target_state.y - self.robot_state.y
        distance_error = math.sqrt(error_x**2 + error_y**2)
        
        # Calcola angolo verso target
        target_angle = math.degrees(math.atan2(error_y, error_x))
        
        # Calcola errore angolare (con wraparound)
        angle_error = target_angle - self.robot_state.theta
        # Normalizza errore angolare tra -180 e +180
        while angle_error > 180:
            angle_error -= 360
        while angle_error < -180:
            angle_error += 360
        
        # Controllo PID
        dt = time.time() - self.last_control_time if self.last_control_time > 0 else 0.033
        self.last_control_time = time.time()
        
        # Velocità lineare basata su distanza
        linear_speed = self.pid_linear.update(distance_error, dt)
        
        # Velocità angolare basata su errore di orientamento
        angular_speed = self.pid_angular.update(angle_error, dt)
        
        # ANTI-OSCILLAZIONE: Dead zone per piccoli errori angolari
        if abs(angle_error) < 5:  # Dead zone di ±5°
            angular_speed = 0.0  # Non correggere, vai dritto
            
        # LOGICA MIGLIORATA: Prima orienta, poi muovi
        elif abs(angle_error) > 45:  # Se errore > 45°
            linear_speed = 0.0  # Stop movimento, solo rotazione
            self.logger.debug(f"Orientamento: errore {angle_error:.1f}°, solo rotazione")
            
        # Se errore angolare medio, riduci velocità lineare ma continua
        elif abs(angle_error) > 20:  # Se errore > 20°
            linear_speed *= 0.4  # Riduci velocità lineare
            angular_speed *= 0.7  # Riduci anche correzione angolare
            
        # Se errore piccolo, vai quasi dritto con correzioni minime
        elif abs(angle_error) > 10:  # Se errore > 10°
            angular_speed *= 0.3  # Correzioni molto dolci
            
        # Se siamo vicini al target, riduci velocità
        if distance_error < self.target_state.tolerance * 2:
            linear_speed *= 0.3
            angular_speed *= 0.5
            
        # Applica soglia minima
        if abs(linear_speed) < ControlConfig.MIN_SPEED_THRESHOLD:
            linear_speed = 0.0
        if abs(angular_speed) < ControlConfig.MIN_SPEED_THRESHOLD:
            angular_speed = 0.0
            
        # SMOOTH ANGOLARE: Filtra cambi improvvisi di velocità angolare
        if hasattr(self, 'last_angular_speed'):
            angular_speed = (self.angular_smooth_factor * self.last_angular_speed + 
                           (1.0 - self.angular_smooth_factor) * angular_speed)
            self.last_angular_speed = angular_speed
        
        return linear_speed, angular_speed
    
    async def control_loop_step(self):
        """Singolo step del loop di controllo"""
        if not self.control_enabled or not self.communication.is_connected():
            return
            
        # Calcola controllo
        linear_speed, angular_speed = self.calculate_control_output()
        
        # Salva errori per debug
        error_x = self.target_state.x - self.robot_state.x
        error_y = self.target_state.y - self.robot_state.y
        distance_error = math.sqrt(error_x**2 + error_y**2)
        target_angle = math.degrees(math.atan2(error_y, error_x))
        angle_error = target_angle - self.robot_state.theta
        while angle_error > 180: angle_error -= 360
        while angle_error < -180: angle_error += 360
        
        # Converti in comandi joystick (cinematica inversa)
        # Per differential drive: 
        # - linear_speed positiva = avanti
        # - angular_speed positiva = gira a destra
        
        # Calcola velocità motori per differential drive
        # Formula corretta: 
        # left_motor = linear_speed - angular_speed * wheel_base/2
        # right_motor = linear_speed + angular_speed * wheel_base/2
        left_motor = linear_speed - angular_speed   
        right_motor = linear_speed + angular_speed
        
        # Saturazione
        max_motor = max(abs(left_motor), abs(right_motor))
        if max_motor > 1.0:
            scale = 1.0 / max_motor
            left_motor *= scale
            right_motor *= scale
        
        # Applica limiti di velocità
        left_motor = max(-ControlConfig.MAX_LINEAR_SPEED, min(ControlConfig.MAX_LINEAR_SPEED, left_motor))
        right_motor = max(-ControlConfig.MAX_LINEAR_SPEED, min(ControlConfig.MAX_LINEAR_SPEED, right_motor))
        
        # Converti a coordinate joystick per il robot server
        # CORREZIONE: Il robot server si aspetta: y positivo = avanti, x positivo = destra
        joystick_x = (right_motor - left_motor) / 2.0  # Rotazione: positivo = destra
        joystick_y = (left_motor + right_motor) / 2.0  # Traslazione: positivo = avanti
        
        # Invia comando
        success = await self.communication.send_movement_command(joystick_x, joystick_y)
        
        # Debug dettagliato ogni 10 comandi
        if self.control_enabled and hasattr(self, '_debug_counter'):
            self._debug_counter = getattr(self, '_debug_counter', 0) + 1
            if self._debug_counter % 10 == 0:
                self.logger.info(f"🎮 Control: dist={distance_error:.2f}, angle={angle_error:.1f}°, "
                               f"linear={linear_speed:.3f}, angular={angular_speed:.3f}, "
                               f"joy=({joystick_x:.3f},{joystick_y:.3f})")
        
        if not success:
            self.logger.warning("Fallito invio comando movimento")
    
    def is_at_target(self) -> bool:
        """Verifica se il robot ha raggiunto il target"""
        if self.robot_state.tracking_lost:
            return False
            
        error_x = self.target_state.x - self.robot_state.x
        error_y = self.target_state.y - self.robot_state.y
        distance = math.sqrt(error_x**2 + error_y**2)
        
        return distance <= self.target_state.tolerance
    
    async def go_to_position(self, x: float, y: float, timeout: float = 30.0) -> bool:
        """
        Muove il robot verso una posizione specifica
        
        Args:
            x, y: Coordinate target [0-100]
            timeout: Timeout in secondi
            
        Returns:
            bool: True se posizione raggiunta con successo
        """
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
                
                await asyncio.sleep(1.0 / SystemConfig.CONTROL_LOOP_RATE)
            
            # Timeout raggiunto
            self.logger.warning(f"Timeout raggiungimento target: ({x:.1f}, {y:.1f})")
            await self.stop()
            return False
            
        except Exception as e:
            self.logger.error(f"Errore durante movimento: {e}")
            await self.emergency_stop()
            return False
    
    async def start_continuous_control(self):
        """Avvia controllo continuo (per uso con interfaccia)"""
        self.control_enabled = True
        self.logger.info("Controllo continuo avviato")
    
    async def stop(self):
        """Ferma il robot normalmente"""
        self.control_enabled = False
        await self.communication.send_stop_command()
        self.logger.info("Robot fermato")
    
    async def emergency_stop(self):
        """Stop di emergenza"""
        self.control_enabled = False
        await self.communication.send_emergency_stop()
        self.logger.warning("STOP DI EMERGENZA")
    
    async def send_manual_command(self, joystick_x: float, joystick_y: float):
        """Invia comando manuale diretto per calibrazione"""
        if not self.communication.is_connected():
            self.logger.warning("Robot non connesso per comando manuale")
            return False
            
        # Applica limiti di sicurezza
        joystick_x = max(-1.0, min(1.0, joystick_x))
        joystick_y = max(-1.0, min(1.0, joystick_y))
        
        # Invia comando diretto
        success = await self.communication.send_movement_command(joystick_x, joystick_y)
        
        if success:
            self.logger.debug(f"🎮 Comando manuale: x={joystick_x:.2f}, y={joystick_y:.2f}")
        else:
            self.logger.warning("Fallito invio comando manuale")
            
        return success
    
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
    
    # Callbacks comunicazione
    def _on_robot_connected(self):
        self.logger.info("Robot connesso")
    
    def _on_robot_disconnected(self):
        self.logger.warning("Robot disconnesso")
        self.control_enabled = False
    
    def _on_robot_error(self, error_msg: str):
        self.logger.error(f"Errore robot: {error_msg}")

# Test del controller
if __name__ == "__main__":
    import asyncio
    
    async def test_controller():
        print("=== Test Robot Controller ===")
        
        controller = RobotController()
        
        # Inizializza
        if not await controller.initialize():
            print("❌ Inizializzazione fallita")
            return
        
        print("✅ Controller inizializzato")
        
        # Simula update da sistema visione
        controller.update_robot_state(30.0, 40.0, 45.0, tracking_valid=True)
        
        print("🎯 Movimento verso target (70, 60)...")
        success = await controller.go_to_position(70.0, 60.0, timeout=10.0)
        
        if success:
            print("✅ Target raggiunto!")
        else:
            print("❌ Target non raggiunto")
        
        # Stato finale
        status = controller.get_status()
        print(f"📊 Stato finale: {status}")
        
        await controller.communication.disconnect()
    
    try:
        asyncio.run(test_controller())
    except KeyboardInterrupt:
        print("\nTest interrotto")
