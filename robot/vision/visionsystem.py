from ast import Dict
import logging
import time
from typing import Optional, Tuple
import cv2
import numpy as np

from config import VisionConfig


class VisionSystem:
    """Sistema di visione basato sui marker ArUco"""
    
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
    
    def process_frame(self, frame) -> Dict:
        """Processa un frame per rilevare marker e calcolare posizioni"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        result = {
            "robot_found": False,
            "robot_x": 0.0,
            "robot_y": 0.0, 
            "robot_theta": 0.0,
            "arena_markers": {},
            "arena_valid": False,
            "obstacles": {},
            "targets": {}
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
                # Usa il nuovo metodo per posizionare il target basato sull'orientamento
                target_pos = self.coords.calculate_target_position_from_marker(
                    center, corner, offset_distance=VisionConfig.TARGET_OFFSET_DISTANCE, target_side="left"
                )
                targets[targetName] = target_pos

            elif marker_id in VisionConfig.ARENA_MARKER_IDS:
                # Marker arena
                arena_markers[marker_id] = center
                            
            elif marker_id > 4:
                # Marker ostacoli (ID > 4)
                obstacles[marker_id] = center
                if marker_id in VisionConfig.SLOTS_MARKER_IDS:
                    # Gli slot sono sia ostacoli che target
                    slotName = VisionConfig.SLOTS.get(marker_id)
                    # Usa il nuovo metodo per posizionare il target basato sull'orientamento
                    target_pos = self.coords.calculate_target_position_from_marker(
                        center, corner, offset_distance=VisionConfig.TARGET_OFFSET_DISTANCE, target_side="left"
                    )
                    targets[slotName] = target_pos

        # Processa marker arena
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
            robot_arena_coords = self.coords._transform_to_arena_coordinates(
                robot_data["center"], arena_markers
            )
            
            if robot_arena_coords:
                result["robot_found"] = True
                result["robot_x"], result["robot_y"] = robot_arena_coords
                result["robot_theta"] = self._calculate_orientation(robot_data["corners"])
                self.last_detection_time = time.time()
        
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
    
    def release(self):
        """Rilascia le risorse della camera"""
        if self.cap:
            self.cap.release()
