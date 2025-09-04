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
        self.pid_forward = PIDController(
            ControlConfig.PID_FORWARD_KP,
            ControlConfig.PID_FORWARD_KI, 
            ControlConfig.PID_FORWARD_KD,
            ControlConfig.MAX_SPEED
        )
        
        self.pid_lateral = PIDController(
            ControlConfig.PID_LATERAL_KP,
            ControlConfig.PID_LATERAL_KI,
            ControlConfig.PID_LATERAL_KD, 
            ControlConfig.MAX_SPEED
        )
        
        # Stato controllo
        self.control_enabled = False
        self.last_control_time = 0.0
        self.tracking_lost_count = 0
        
        # Filtro per smoothing velocità angolare (anti-oscillazione)
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
        self.pid_forward.reset()
        self.pid_lateral.reset()
        
        # Reset anche filtro smoothing angolare
        self.last_angular_speed = 0.0
    
    def calculate_control_output(self) -> Tuple[float, float]:
        """
        Calcola output di controllo
        Output: x,y come coordinate JOYSTICK per robot
        """
        if self.robot_state.tracking_lost:
            return 0.0, 0.0
            
        # Calcola vettore verso target
        error_x = self.target_state.x - self.robot_state.x
        error_y = self.target_state.y - self.robot_state.y
        distance = math.sqrt(error_x**2 + error_y**2)
        
        # Se già al target, ferma
        if distance < self.target_state.tolerance:
            return 0.0, 0.0
        
        # VETTORE VELOCITÀ: intensità e direzione
        # Intensità: proporzionale alla distanza (con saturazione)
        max_intensity = ControlConfig.MAX_SPEED
        intensity = min(distance / ControlConfig.VECTOR_DISTANCE_SCALE, max_intensity)
        
        # Direzione: angolo verso target
        target_angle_rad = math.atan2(error_y, error_x)
        
        # Componenti vettore velocità nel sistema mondo
        velocity_world_x = intensity * math.cos(target_angle_rad)
        velocity_world_y = intensity * math.sin(target_angle_rad)
        
        # Trasforma dal sistema mondo al sistema robot
        robot_angle_rad = math.radians(self.robot_state.theta)
        
        # Rotazione inversa: da coordinate mondo a coordinate robot
        velocity_robot_x = (velocity_world_x * math.cos(robot_angle_rad) + 
                           velocity_world_y * math.sin(robot_angle_rad))
        velocity_robot_y = (-velocity_world_x * math.sin(robot_angle_rad) + 
                           velocity_world_y * math.cos(robot_angle_rad))
        
        # Applica limiti
        joystick_x = max(-max_intensity, min(max_intensity, velocity_robot_x))
        joystick_y = max(-ControlConfig.MAX_SPEED, 
                          min(ControlConfig.MAX_SPEED, velocity_robot_y))
        
        # SALVA INFORMAZIONI VETTORE per visualizzazione
        self.last_vector_intensity = intensity
        self.last_vector_angle = math.atan2(velocity_robot_y, velocity_robot_x)  # Angolo nel sistema robot
        self.last_target_distance = distance

        #print(f"Joystick Output: x={joystick_x:.2f}, y={joystick_y:.2f}")
        return joystick_y, joystick_x  # Nota: invertito per joystick (y avanti, x laterale)
    
    async def control_loop_step(self):
        """Singolo step del loop di controllo"""
        if not self.control_enabled or not self.communication.is_connected():
            return
            
        # Calcola controllo
        joystick_x, joystick_y = self.calculate_control_output()

        # Salva errori per debug
        error_x = self.target_state.x - self.robot_state.x
        error_y = self.target_state.y - self.robot_state.y
        distance_error = math.sqrt(error_x**2 + error_y**2)
        target_angle = math.degrees(math.atan2(error_y, error_x))
        angle_error = target_angle - self.robot_state.theta
        while angle_error > 180: angle_error -= 360
        while angle_error < -180: angle_error += 360

        # Saturazione
        max_motor = max(abs(joystick_x), abs(joystick_y))
        if max_motor > 1.0:
            scale = 1.0 / max_motor
            joystick_x *= scale
            joystick_y *= scale

        # Applica limiti di velocità
        joystick_x = max(-ControlConfig.MAX_SPEED, min(ControlConfig.MAX_SPEED, joystick_x))
        joystick_y = max(-ControlConfig.MAX_SPEED, min(ControlConfig.MAX_SPEED, joystick_y))

        # Invia comando
        success = await self.communication.send_movement_command(joystick_x, joystick_y)

        # Salva errori e variabili per debug
        error_x = self.target_state.x - self.robot_state.x
        error_y = self.target_state.y - self.robot_state.y
        distance = math.sqrt(error_x**2 + error_y**2)
        
        # Debug dettagliato ogni 10 comandi
        if self.control_enabled and hasattr(self, '_debug_counter'):
            self._debug_counter = getattr(self, '_debug_counter', 0) + 1
            if self._debug_counter % 10 == 0:
                self.logger.info(f"🎮 Vector Control: dist={distance:.2f}, "
                                 f"joystick_x={joystick_x:.2f}, joystick_y={joystick_y:.2f}")

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
