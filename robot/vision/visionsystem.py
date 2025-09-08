from ast import Dict
import logging
import time
from typing import Optional, Tuple
import cv2
import numpy as np

from config import VisionConfig


class VisionSystem:
    """Sistema di visione basato sui marker ArUco con correzione prospettica"""
    
    def __init__(self, coords=None):
        self.cap = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.coords = coords

        # Stati
        self.arena_markers = {}
        self.robot_position = None
        self.robot_orientation = None
        self.last_detection_time = 0
        
        # Correzione prospettica
        self.perspective_matrix = None
        self.inverse_perspective_matrix = None
        self.corrected_frame_size = (800, 600)  # Dimensioni frame corretto
        self.perspective_calibrated = False
        
        self.logger = logging.getLogger(__name__)
    
    def initialize_camera(self) -> bool:
        """Inizializza la webcam"""
        try:
            self.cap = cv2.VideoCapture(VisionConfig.CAMERA_INDEX, VisionConfig.CAPTURE_FLAGS)
            
            if not self.cap.isOpened():
                self.logger.error("Impossibile aprire la webcam")
                return False
            
            # Imposta risoluzione
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VisionConfig.FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VisionConfig.FRAME_HEIGHT)
            
            # Verifica risoluzione effettiva
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.logger.info(f"Camera inizializzata: {actual_width}x{actual_height}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Errore inizializzazione camera: {e}")
            return False
    
    def calibrate_perspective(self, frame) -> bool:
        """
        Calibra la trasformazione prospettica usando i 4 marker dell'arena
        
        Returns:
            True se la calibrazione è riuscita, False altrimenti
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        if ids is None or len(ids) < 4:
            return False
        
        # Cerca i 4 marker dell'arena
        arena_corners = {}
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in VisionConfig.ARENA_MARKER_IDS:
                corner = corners[i][0]
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))
                arena_corners[marker_id] = (center_x, center_y)
        
        if len(arena_corners) != 4:
            self.logger.warning(f"Solo {len(arena_corners)}/4 marker arena trovati per calibrazione")
            return False
        
        # Ordina i marker in senso orario partendo da top-left
        # Assumendo che i marker siano: 1=top-left, 2=top-right, 3=bottom-right, 4=bottom-left
        try:
            source_points = np.float32([
                arena_corners[1],  # Top-left
                arena_corners[2],  # Top-right  
                arena_corners[3],  # Bottom-right
                arena_corners[4],  # Bottom-left
            ])
        except KeyError as e:
            self.logger.error(f"Marker arena mancante per calibrazione: {e}")
            return False
        
        # Punti destinazione: rettangolo perfetto nell'immagine corretta
        margin = 50  # Margine dai bordi
        dest_width = self.corrected_frame_size[0] - 2 * margin
        dest_height = self.corrected_frame_size[1] - 2 * margin
        
        destination_points = np.float32([
            [margin, margin],                           # Top-left
            [margin + dest_width, margin],              # Top-right
            [margin + dest_width, margin + dest_height], # Bottom-right
            [margin, margin + dest_height],             # Bottom-left
        ])
        
        # Calcola matrice di trasformazione prospettica
        self.perspective_matrix = cv2.getPerspectiveTransform(source_points, destination_points)
        self.inverse_perspective_matrix = cv2.getPerspectiveTransform(destination_points, source_points)
        
        self.perspective_calibrated = True
        self.logger.info("✅ Calibrazione prospettica completata!")
        self.logger.debug(f"Source points: {source_points}")
        self.logger.debug(f"Dest points: {destination_points}")
        
        return True
    
    def apply_perspective_correction(self, frame):
        """
        Applica la correzione prospettica al frame
        
        Returns:
            Frame corretto, None se non calibrato
        """
        if not self.perspective_calibrated or self.perspective_matrix is None:
            return None
        
        corrected_frame = cv2.warpPerspective(
            frame, 
            self.perspective_matrix, 
            self.corrected_frame_size
        )
        
        return corrected_frame
    
    def transform_point_to_corrected(self, point):
        """
        Trasforma un punto dal frame originale al frame corretto
        
        Args:
            point: (x, y) nel frame originale
            
        Returns:
            (x, y) nel frame corretto, None se non calibrato
        """
        if not self.perspective_calibrated or self.perspective_matrix is None:
            return None
        
        # OpenCV richiede il punto in formato [[[x, y]]]
        point_array = np.array([[[point[0], point[1]]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point_array, self.perspective_matrix)
        
        return (int(transformed[0][0][0]), int(transformed[0][0][1]))
    
    def transform_point_to_original(self, point):
        """
        Trasforma un punto dal frame corretto al frame originale
        
        Args:
            point: (x, y) nel frame corretto
            
        Returns:
            (x, y) nel frame originale, None se non calibrato
        """
        if not self.perspective_calibrated or self.inverse_perspective_matrix is None:
            return None
        
        point_array = np.array([[[point[0], point[1]]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point_array, self.inverse_perspective_matrix)
        
        return (int(transformed[0][0][0]), int(transformed[0][0][1]))

    def process_frame(self, frame) -> Dict:
        """Processa un frame per rilevare marker e calcolare posizioni"""
        # Se non abbiamo ancora calibrato la prospettiva, prova a farlo
        if not self.perspective_calibrated:
            self.calibrate_perspective(frame)
        
        # Usa il frame corretto se disponibile, altrimenti quello originale
        working_frame = frame
        if self.perspective_calibrated:
            corrected_frame = self.apply_perspective_correction(frame)
            if corrected_frame is not None:
                working_frame = corrected_frame
        
        gray = cv2.cvtColor(working_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        result = {
            "robot_found": False,
            "robot_x": 0.0,
            "robot_y": 0.0, 
            "robot_theta": 0.0,
            "arena_markers": {},
            "arena_valid": False,
            "obstacles": {},
            "targets": {},
            "perspective_corrected": self.perspective_calibrated,
            "corrected_frame": working_frame if self.perspective_calibrated else None
        }
        
        if ids is None:
            return result
        
        # Processo marker rilevati
        arena_markers = {}
        robot_data = None
        obstacles = {}
        targets = {}
        
        for i, marker_id in enumerate(ids.flatten()):
            # Calcola centro marker
            corner = corners[i][0]
            center_x = int(np.mean(corner[:, 0]))
            center_y = int(np.mean(corner[:, 1]))
            center = (center_x, center_y)
            
            if marker_id == VisionConfig.ROBOT_MARKER_ID:
                # Marker robot
                robot_data = {
                    "center": center,
                    "corners": corner
                }

            elif marker_id in VisionConfig.TARGETS_MARKER_IDS:
                # Marker target 
                targetName = VisionConfig.TARGETS.get(marker_id)
                # Se stiamo usando il frame corretto, i calcoli sono più precisi
                if self.perspective_calibrated:
                    # Nel frame corretto, le coordinate sono già "dritte"
                    target_pos = self.coords.calculate_target_position_from_marker(
                        center, corner, offset_distance=VisionConfig.TARGET_OFFSET_DISTANCE, target_side="left"
                    )
                else:
                    # Frame originale - usa il metodo standard
                    target_pos = self.coords.calculate_target_position_from_marker(
                        center, corner, offset_distance=VisionConfig.TARGET_OFFSET_DISTANCE, target_side="left"
                    )
                targets[targetName] = target_pos

            elif marker_id in VisionConfig.ARENA_MARKER_IDS:
                # Marker arena - nel frame corretto sono ai bordi perfetti
                arena_markers[marker_id] = center
                            
            elif marker_id > 4:
                # Marker ostacoli (ID > 4)
                obstacles[marker_id] = center
                if marker_id in VisionConfig.SLOTS_MARKER_IDS:
                    # Gli slot sono sia ostacoli che target
                    slotName = VisionConfig.SLOTS.get(marker_id)
                    if self.perspective_calibrated:
                        target_pos = self.coords.calculate_target_position_from_marker(
                            center, corner, offset_distance=VisionConfig.TARGET_OFFSET_DISTANCE, target_side="left"
                        )
                    else:
                        target_pos = self.coords.calculate_target_position_from_marker(
                            center, corner, offset_distance=VisionConfig.TARGET_OFFSET_DISTANCE, target_side="left"
                        )
                    targets[slotName] = target_pos

        # Processa marker arena
        if self.perspective_calibrated:
            # Nel frame corretto, i marker arena definiscono automaticamente l'arena
            if len(arena_markers) >= 4:
                result["arena_markers"] = arena_markers
                result["arena_valid"] = True
                self.arena_markers = arena_markers
        else:
            # Frame originale - usa la logica standard
            if len(arena_markers) >= 2:
                result["arena_markers"] = arena_markers
                result["arena_valid"] = True
                self.arena_markers = arena_markers
        
        # Processa ostacoli
        result["obstacles"] = obstacles

        # Processa targets e slots
        result["targets"] = targets
        
        # Processa robot se trovato e arena valida
        if robot_data and result["arena_valid"]:
            if self.perspective_calibrated:
                # Nel frame corretto, calcola coordinate direttamente dai margini
                margin = 50  # Stesso margine usato nella calibrazione
                arena_width = self.corrected_frame_size[0] - 2 * margin
                arena_height = self.corrected_frame_size[1] - 2 * margin
                
                # Coordinate normalizzate (0-100) basate sulla posizione nell'area arena
                robot_x = ((robot_data["center"][0] - margin) / arena_width) * 100
                robot_y = ((robot_data["center"][1] - margin) / arena_height) * 100
                
                # Clamp ai limiti e inverti Y (OpenCV ha Y crescente verso il basso)
                robot_x = max(0, min(100, robot_x))  
                robot_y = max(0, min(100, 100 - robot_y))  # Inverti Y
                
                result["robot_found"] = True
                result["robot_x"] = robot_x
                result["robot_y"] = robot_y
                result["robot_theta"] = self._calculate_orientation(robot_data["corners"])
                self.last_detection_time = time.time()
                
                self.logger.debug(f"Robot coords (perspective corrected): ({robot_x:.1f}, {robot_y:.1f})")
            else:
                # Frame originale - usa il sistema esistente
                robot_arena_coords = self.coords._transform_to_arena_coordinates(
                    robot_data["center"], arena_markers
                )
                
                if robot_arena_coords:
                    result["robot_found"] = True
                    result["robot_x"], result["robot_y"] = robot_arena_coords
                    result["robot_theta"] = self._calculate_orientation(robot_data["corners"])
                    self.last_detection_time = time.time()
                    
                    self.logger.debug(f"Robot coords (original frame): ({result['robot_x']:.1f}, {result['robot_y']:.1f})")
        
        return result
    
    def _calculate_orientation(self, corners) -> float:
        """Calcola orientamento del robot in gradi"""
        # Vettore dal primo al secondo corner
        dx = corners[1, 0] - corners[0, 0]
        dy = corners[1, 1] - corners[0, 1]
        angle_rad = np.arctan2(dy, dx)
        angle_deg = np.degrees(angle_rad)
        
        # Normalizza 0-360
        if angle_deg < 0:
            angle_deg += 360
            
        return angle_deg -90
    
    def get_perspective_info(self) -> dict:
        """Ritorna informazioni sulla calibrazione prospettica"""
        return {
            "calibrated": self.perspective_calibrated,
            "corrected_size": self.corrected_frame_size,
            "has_matrix": self.perspective_matrix is not None
        }
    
    def reset_perspective_calibration(self):
        """Reset della calibrazione prospettica"""
        self.perspective_matrix = None
        self.inverse_perspective_matrix = None
        self.perspective_calibrated = False
        self.logger.info("🔄 Calibrazione prospettica resettata")
    
    def save_perspective_calibration(self, filepath: str) -> bool:
        """Salva la calibrazione prospettica su file"""
        if not self.perspective_calibrated:
            return False
        
        try:
            np.savez(filepath, 
                    perspective_matrix=self.perspective_matrix,
                    inverse_perspective_matrix=self.inverse_perspective_matrix,
                    corrected_frame_size=self.corrected_frame_size)
            self.logger.info(f"💾 Calibrazione salvata in {filepath}")
            return True
        except Exception as e:
            self.logger.error(f"Errore salvataggio calibrazione: {e}")
            return False
    
    def load_perspective_calibration(self, filepath: str) -> bool:
        """Carica la calibrazione prospettica da file"""
        try:
            data = np.load(filepath)
            self.perspective_matrix = data['perspective_matrix']
            self.inverse_perspective_matrix = data['inverse_perspective_matrix'] 
            self.corrected_frame_size = tuple(data['corrected_frame_size'])
            self.perspective_calibrated = True
            self.logger.info(f"📁 Calibrazione caricata da {filepath}")
            return True
        except Exception as e:
            self.logger.error(f"Errore caricamento calibrazione: {e}")
            return False

    def release(self):
        """Rilascia le risorse della camera"""
        if self.cap:
            self.cap.release()
