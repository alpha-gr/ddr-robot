# Sistema DDR Robot - Configurazione Centralizzata
"""
config.py
Parametri di configurazione centralizzati per il sistema di controllo robotico
"""

import cv2

class MapConfig:
    """Configurazione mappa per compatibilità con software qak basicrobot"""
    MAP_TO_TARGET = {
        (0, 0): "Home",
        (0, 4): "Pickup",
        (1, 1): "Slot_1",
        (4, 1): "Slot_2",
        (1, 3): "Slot_3",
        (4, 3): "Slot_4",
        (5, 2): "Slot_5",
    }

class RobotConfig:
    """Configurazione hardware e fisica del robot"""
    # Dimensioni fisiche (in cm, per reference)
    ROBOT_WIDTH = 15.0        # Larghezza robot
    ROBOT_LENGTH = 20.0       # Lunghezza robot
    WHEEL_DIAMETER = 6.5      # Diametro ruote
    WHEEL_BASE = 12.0         # Distanza tra ruote
    
    # Limiti di velocità fisica
    MAX_RPM = 100             # RPM massimo motori
    MAX_LINEAR_VELOCITY = 0.3 # m/s velocità lineare massima
    MAX_ANGULAR_VELOCITY = 1.5 # rad/s velocità angolare massima

class VisionConfig:
    """Configurazione sistema di visione"""
    # Camera settings
    CAMERA_INDEX = 0
    FRAME_WIDTH = 1280
    FRAME_HEIGHT = 720
    CAPTURE_FLAGS = cv2.CAP_DSHOW
    MAX_FPS = 30              # FPS massimo per thread vision

    # ArUco settings
    ARUCO_DICT = "DICT_4X4_50"
    MARKER_SIZE = 0.05        # Dimensione marker in metri (per calibrazione futura)
    
    # Arena markers
    ARENA_MARKER_IDS = [1, 2, 3, 4]
    TARGETS ={
        5 : "Home",
        6 : "Pickup",
    }
    TARGETS_MARKER_IDS = list(TARGETS.keys())
    # gli slots sono sia ostacoli che target
    SLOTS = {
        11 : "Slot_1",
        12 : "Slot_2",
        13 : "Slot_3",
        14 : "Slot_4",
        15 : "Slot_5",
    }
    SLOTS_MARKER_IDS = list(SLOTS.keys())
    ROBOT_MARKER_ID = 0

    TARGET_OFFSET_DISTANCE = 100
    
    # Tracking settings
    MIN_MARKER_AREA = 100     # Pixel minimi per marker valido
    MAX_TRACKING_LOSS = 30    # Frame massimi senza tracking prima di stop

class ControlConfig:
    """Configurazione algoritmi di controllo"""
    # PID Parameters
    # P: if you’re not where you want to be, get there.
    # I: if you haven’t been where you want to be for a long time, get there faster.
    # D: if you’re getting close to where you want to be, slow down.

    # Forward
    PID_FORWARD_KP = 1   # Risposta rapida alle distanze
    PID_FORWARD_KI = 0.0  # Elimina errori di posizionamento
    PID_FORWARD_KD = 0.0   # Smorzamento overshoot

    # Lateral  
    PID_LATERAL_KP = 0.4   # Correzione angolare immediata
    PID_LATERAL_KI = 0.005  # Correzione derive/bias
    PID_LATERAL_KD = 0.4  # Stabilità rotazioni

    # Limiti controllo
    MAX_SPEED = 1 
    
    # Controllo vettoriale
    VECTOR_DISTANCE_SCALE = 10.0
    
    # Tolleranze
    POSITION_TOLERANCE = 3.0  # Aumentato da 2.0 - meno preciso ma più stabile
    ANGLE_TOLERANCE = 10.0    # Aumentato da 5.0 - meno oscillazioni angolari
    
    # Safety
    BOUNDARY_MARGIN = 5.0     # Margine dai bordi arena (unità arena)
    EMERGENCY_STOP_DISTANCE = 3.0 # Distanza minima dai bordi per stop
    
    # Calibrazione manuale
    MANUAL_LINEAR_SPEED = 0.3   # Velocità lineare per controlli manuali
    MANUAL_ANGULAR_SPEED = 0.4  # Velocità angolare per controlli manuali
    

class PathfindingConfig:
    """Configurazione per il pathfinding"""
    # Griglia di pathfinding
    GRID_RESOLUTION = 1     # Risoluzione griglia (unità arena per cella) 
    GRID_SIZE = int(100 / GRID_RESOLUTION)
    
    # Ostacoli - AUMENTATI per test
    OBSTACLE_INFLATION_RADIUS = 20  # Raggio inflazione ostacoli (unità arena)
    ROBOT_SAFETY_RADIUS = 4         # Raggio sicurezza robot (unità arena)
    
    # Algoritmo A*
    DIAGONAL_COST = 1.41421356       # sqrt(2) per movimenti diagonali
    STRAIGHT_COST = 1.0              # Costo movimenti cardinali
    
    # Ottimizzazione percorso - CONFIGURATE per più punti
    ENABLE_PATH_SMOOTHING = False
    SMOOTHING_ITERATIONS = 1         
    MIN_WAYPOINT_DISTANCE = 5.0      
    MAX_WAYPOINT_DISTANCE = 10.0     
    
    # Vincoli di navigazione
    MAX_PATH_LENGTH = 10000          
    BOUNDARY_MARGIN = ControlConfig.BOUNDARY_MARGIN 

    # Ricalcolo percorso
    PATHFINDING_RECALC_DISTANCE = 3.0 # Distanza minima cambiamento posizione robot per ricalcolo
    
    # Waypoint dinamico per controllo fluido
    DYNAMIC_WAYPOINT_RADIUS = 15.0   # Distanza avanti al robot per waypoint dinamico
    MIN_ADVANCE_DISTANCE = 2.0       # Distanza minima per avanzare lungo il percorso
    WAYPOINT_SMOOTHING = 0.8         # Fattore smoothing per movimento waypoint (0-1)


class CommunicationConfig:
    """Configurazione comunicazione"""
    # WebSocket settings - CONFIGURARE CON IP RASPBERRY PI REALE
    WS_HOST = "192.168.1.9"
    WS_PORT = 8765
    WS_TIMEOUT = 1.0          # Timeout connessione
    
    # Message format
    HEARTBEAT_INTERVAL = 0.5  # Secondi tra heartbeat
    MAX_RETRY_ATTEMPTS = 3    # Tentativi di riconnessione
    
    # Safety timeouts
    COMMAND_TIMEOUT = 1.0     # Timeout comando singolo
    CONNECTION_TIMEOUT = 5.0  # Timeout connessione totale
    
    # Per testing locale senza hardware
    SIMULATION_MODE = True    # True = non invia comandi reali

class UIConfig:
    """Configurazione interfaccia utente"""
    # Window settings
    WINDOW_NAME = "DDR Robot Control System"
    DEFAULT_WINDOW_WIDTH = 1600
    DEFAULT_WINDOW_HEIGHT = 900
    
    # Colors (BGR format)
    COLOR_ROBOT = (0, 255, 255)        # Giallo
    COLOR_TARGET = (255, 0, 255)       # Magenta
    COLOR_OBSTACLES = (0, 0, 255)      # Rosso per ostacoli
    COLOR_TARGETS = (0, 165, 255)      # Arancione per target
    COLOR_ARENA_MARKERS = {
        1: (255, 0, 0),                 # Blu
        2: (0, 255, 0),                 # Verde
        3: (0, 0, 255),                 # Rosso  
        4: (255, 255, 0)                # Ciano
    }
    COLOR_TRAJECTORY = (128, 255, 128) # Verde chiaro
    COLOR_PATH = (255, 128, 0)         # Arancione per percorso pathfinding
    COLOR_WAYPOINT = (0, 165, 255)     # Arancione scuro per waypoint
    COLOR_CURRENT_WAYPOINT = (0, 255, 255) # Giallo per waypoint corrente
    COLOR_BOUNDARY = (255, 255, 255)   # Bianco
    COLOR_VECTOR = (255, 0, 255)       # Viola (Magenta)
    
    # Text settings
    FONT_SCALE = 0.4
    FONT_THICKNESS = 1
    INFO_PANEL_ALPHA = 0.6

# Configurazione globale del sistema
class SystemConfig:
    """Configurazione generale del sistema"""
    # Loop timing
    MAIN_LOOP_RATE = 30       # Hz - frequenza loop principale
    CONTROL_LOOP_RATE = 20    # Hz - frequenza controllo motori
    UI_UPDATE_RATE = 30       # Hz - frequenza aggiornamento UI
    
    # Logging
    LOG_LEVEL = "INFO"        # DEBUG, INFO, WARNING, ERROR
    SAVE_TRAJECTORY_DATA = True
    TRAJECTORY_LOG_FILE = "robot_trajectory_{timestamp}.json"
    
    # Modalità operativa
    SIMULATION_MODE = False   # True per test senza hardware
    DEBUG_VISUALIZATION = True # Mostra debug info on screen
    PERFORMANCE_MONITORING = True # Traccia metriche performance

# Funzioni di utilità per configurazione
def get_config_summary():
    """Ritorna un riassunto della configurazione attuale"""
    return {
        "robot": {
            "dimensions": f"{RobotConfig.ROBOT_WIDTH}x{RobotConfig.ROBOT_LENGTH}cm",
            "max_speed": f"{RobotConfig.MAX_LINEAR_VELOCITY}m/s"
        },
        "vision": {
            "resolution": f"{VisionConfig.FRAME_WIDTH}x{VisionConfig.FRAME_HEIGHT}",
            "markers": f"Arena: {VisionConfig.ARENA_MARKER_IDS}, Robot: {VisionConfig.ROBOT_MARKER_ID}"
        },
        "control": {
            "pid_linear": f"Kp={ControlConfig.PID_LINEAR_KP}, Ki={ControlConfig.PID_LINEAR_KI}, Kd={ControlConfig.PID_LINEAR_KD}",
            "tolerances": f"Pos: ±{ControlConfig.POSITION_TOLERANCE}, Ang: ±{ControlConfig.ANGLE_TOLERANCE}°"
        },
        "communication": {
            "target": f"ws://{CommunicationConfig.WS_HOST}:{CommunicationConfig.WS_PORT}",
            "timeouts": f"Cmd: {CommunicationConfig.COMMAND_TIMEOUT}s, Conn: {CommunicationConfig.CONNECTION_TIMEOUT}s"
        }
    }

def validate_config():
    """Valida la coerenza della configurazione"""
    issues = []
    
    # Validazioni di base
    if ControlConfig.POSITION_TOLERANCE <= 0:
        issues.append("POSITION_TOLERANCE deve essere > 0")
    
    if ControlConfig.MAX_LINEAR_SPEED > 1.0:
        issues.append("MAX_LINEAR_SPEED non può essere > 1.0")
        
    if VisionConfig.FRAME_WIDTH < 640:
        issues.append("Risoluzione camera troppo bassa")
    
    return issues

if __name__ == "__main__":
    # Test configurazione
    print("=== Configurazione Sistema DDR Robot ===")
    
    config_summary = get_config_summary()
    for section, params in config_summary.items():
        print(f"\n[{section.upper()}]")
        for key, value in params.items():
            print(f"  {key}: {value}")
    
    # Validazione
    issues = validate_config()
    if issues:
        print(f"\nPROBLEMI CONFIGURAZIONE:")
        for issue in issues:
            print(f"  - {issue}")
    else:
        print(f"\n✅ Configurazione valida!")
