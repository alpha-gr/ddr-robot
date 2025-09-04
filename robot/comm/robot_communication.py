"""
robot_communication.py
Modulo per la comunicazione WebSocket con il robot Raspberry Pi
"""

import asyncio
import json
import time
import websockets
from typing import Optional, Dict, Tuple, Callable
import logging

from config import CommunicationConfig

class RobotCommStatus:
    """Stati di connessione possibili"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"  
    CONNECTED = "connected"
    ERROR = "error"

class RobotCommunication:
    """Gestisce la comunicazione WebSocket con il robot"""
    
    def __init__(self, host: str = None, port: int = None):
        self.host = host or CommunicationConfig.WS_HOST
        self.port = port or CommunicationConfig.WS_PORT
        self.websocket = None
        self.status = RobotCommStatus.DISCONNECTED
        self.last_command_time = 0
        self.last_response = None
        self.retry_count = 0
        
        # Callbacks per eventi
        self.on_connected: Optional[Callable] = None
        self.on_disconnected: Optional[Callable] = None
        self.on_error: Optional[Callable] = None
        self.on_response: Optional[Callable] = None
        
        # Logger
        self.logger = logging.getLogger(__name__)
        
    async def connect(self) -> bool:
        """Stabilisce connessione WebSocket con il robot"""
        if self.status == RobotCommStatus.CONNECTED:
            return True
            
        try:
            self.status = RobotCommStatus.CONNECTING
            self.logger.info(f"Connettendo a ws://{self.host}:{self.port}")
            
            self.websocket = await asyncio.wait_for(
                websockets.connect(f"ws://{self.host}:{self.port}"),
                timeout=CommunicationConfig.CONNECTION_TIMEOUT
            )
            
            self.status = RobotCommStatus.CONNECTED
            self.retry_count = 0
            self.logger.info("Connessione stabilita con successo")
            
            if self.on_connected:
                self.on_connected()
                
            return True
            
        except asyncio.TimeoutError:
            self.logger.error("Timeout connessione")
            self.status = RobotCommStatus.ERROR
            return False
        except Exception as e:
            self.logger.error(f"Errore connessione: {e}")
            self.status = RobotCommStatus.ERROR
            if self.on_error:
                self.on_error(f"Connection failed: {e}")
            return False
    
    async def disconnect(self):
        """Chiude la connessione WebSocket"""
        if self.websocket:
            await self.websocket.close()
            self.websocket = None
        
        self.status = RobotCommStatus.DISCONNECTED
        self.logger.info("Disconnesso dal robot")
        
        if self.on_disconnected:
            self.on_disconnected()
    
    async def send_movement_command(self, x: float, y: float) -> bool:
        """
        Invia comando di movimento al robot
        
        Args:
            x: Controllo laterale [-1, 1] (negativo = sinistra)
            y: Controllo avanti/indietro [-1, 1] (negativo = indietro)
            
        Returns:
            bool: True se comando inviato con successo
        """
        if self.status != RobotCommStatus.CONNECTED:
            if not await self.connect():
                return False
        
        try:
            command = {
                "x": float(x),
                "y": float(y),
                "timestamp": int(time.time() * 1000)
            }
            
            await asyncio.wait_for(
                self.websocket.send(json.dumps(command)),
                timeout=CommunicationConfig.COMMAND_TIMEOUT
            )
            
            self.last_command_time = time.time()
            self.logger.debug(f"Comando inviato: x={x:.3f}, y={y:.3f}")
            
            # Attendi risposta (opzionale, non blocking)
            try:
                response = await asyncio.wait_for(
                    self.websocket.recv(),
                    timeout=0.1  # Timeout breve per non bloccare
                )
                self.last_response = response
                if self.on_response:
                    self.on_response(response)
            except asyncio.TimeoutError:
                pass  # Non è critico se non riceviamo risposta immediata
                
            return True
            
        except websockets.ConnectionClosed:
            self.logger.warning("Connessione chiusa durante invio comando")
            self.status = RobotCommStatus.DISCONNECTED
            return False
        except Exception as e:
            self.logger.error(f"Errore invio comando: {e}")
            if self.on_error:
                self.on_error(f"Send failed: {e}")
            return False
    
    async def send_stop_command(self) -> bool:
        """Invia comando di stop al robot"""
        return await self.send_movement_command(0.0, 0.0)
    
    async def send_emergency_stop(self) -> bool:
        """Invia comando di stop di emergenza"""
        if self.status != RobotCommStatus.CONNECTED:
            return False
            
        try:
            command = {"cmd": "stop"}
            await self.websocket.send(json.dumps(command))
            self.logger.warning("Stop di emergenza inviato")
            return True
        except Exception as e:
            self.logger.error(f"Errore stop emergenza: {e}")
            return False
    
    def is_connected(self) -> bool:
        """Verifica se la connessione è attiva"""
        return self.status == RobotCommStatus.CONNECTED
    
    def get_connection_info(self) -> Dict:
        """Ritorna informazioni sullo stato della connessione"""
        return {
            "status": self.status,
            "host": self.host,
            "port": self.port,
            "last_command": self.last_command_time,
            "last_response": self.last_response,
            "retry_count": self.retry_count
        }
    
    async def heartbeat_loop(self):
        """Loop per mantenere la connessione attiva"""
        while True:
            await asyncio.sleep(CommunicationConfig.HEARTBEAT_INTERVAL)
            
            if self.status == RobotCommStatus.CONNECTED:
                # Verifica se è passato troppo tempo dall'ultimo comando
                if time.time() - self.last_command_time > CommunicationConfig.COMMAND_TIMEOUT * 2:
                    # Invia heartbeat (movimento nullo)
                    await self.send_stop_command()

# Funzioni di utilità per conversioni cinematiche
def differential_drive_to_joystick(left_speed: float, right_speed: float) -> Tuple[float, float]:
    """
    Converte velocità motori differenziali in coordinate joystick
    
    Args:
        left_speed: Velocità motore sinistro [-1, 1]
        right_speed: Velocità motore destro [-1, 1]
        
    Returns:
        Tuple[float, float]: (x, y) per joystick
    """
    # Cinematica inversa differenziale
    forward = (left_speed + right_speed) / 2.0  # Velocità media = avanti
    rotation = (right_speed - left_speed) / 2.0  # Differenza = rotazione
    
    # Conversione a coordinate joystick (y negativo = avanti)
    x = -rotation  # Rotazione: negativo = sinistra
    y = -forward   # Avanti: negativo = avanti per robot
    
    return x, y

def joystick_to_differential_drive(x: float, y: float) -> Tuple[float, float]:
    """
    Converte coordinate joystick in velocità motori differenziali
    
    Args:
        x: Controllo laterale [-1, 1] (negativo = sinistra)  
        y: Controllo avanti/indietro [-1, 1] (negativo = avanti)
        
    Returns:
        Tuple[float, float]: (left_speed, right_speed)
    """
    # Conversione da coordinate joystick: y negativo = avanti
    forward = -y  # Inverti y: negativo diventa positivo per "avanti"
    rotation = -x  # Inverti x: negativo diventa positivo per "sinistra"
    
    # Cinematica differenziale
    left_speed = forward + rotation   # Sinistra = avanti + rotazione sinistra
    right_speed = forward - rotation  # Destra = avanti - rotazione sinistra
    
    # Saturazione per evitare valori > 1
    max_speed = max(abs(left_speed), abs(right_speed))
    if max_speed > 1.0:
        scale = 1.0 / max_speed
        left_speed *= scale
        right_speed *= scale
    
    return left_speed, right_speed

# Test e debug
if __name__ == "__main__":
    import asyncio
    
    async def test_communication():
        """Test di base della comunicazione"""
        print("=== Test Robot Communication ===")
        
        # Crea oggetto comunicazione
        robot_comm = RobotCommunication()
        
        # Setup callbacks per debug
        robot_comm.on_connected = lambda: print("✅ Robot connesso!")
        robot_comm.on_disconnected = lambda: print("❌ Robot disconnesso!")
        robot_comm.on_error = lambda msg: print(f"⚠️  Errore: {msg}")
        robot_comm.on_response = lambda resp: print(f"📨 Risposta: {resp}")
        
        # Test connessione
        print("Tentativo connessione...")
        success = await robot_comm.connect()
        
        if success:
            print("Test movimento...")
            # Test movimento: avanti lento
            await robot_comm.send_movement_command(0.0, -0.3)
            await asyncio.sleep(1)
            
            # Stop
            await robot_comm.send_stop_command()
            await asyncio.sleep(0.5)
            
            # Test rotazione
            await robot_comm.send_movement_command(-0.5, 0.0)  
            await asyncio.sleep(1)
            
            # Stop finale
            await robot_comm.send_stop_command()
            
        await robot_comm.disconnect()
        print("Test completato!")
    
    # Test conversioni cinematiche
    print("\n=== Test Conversioni Cinematiche ===")
    test_cases = [
        (0.0, -0.5),   # Avanti
        (0.0, 0.5),    # Indietro
        (-0.5, 0.0),   # Sinistra
        (0.5, 0.0),    # Destra
        (-0.3, -0.3),  # Avanti + Sinistra
    ]
    
    for x, y in test_cases:
        left, right = joystick_to_differential_drive(x, y)
        print(f"Joystick({x:+.1f}, {y:+.1f}) → Motors(L:{left:+.2f}, R:{right:+.2f})")
    
    # Esegui test se non siamo in modalità simulazione
    try:
        asyncio.run(test_communication())
    except KeyboardInterrupt:
        print("\nTest interrotto da utente")
