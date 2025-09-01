"""
integrated_robot_system.py
Sistema integrato che combina visione, controllo e interfaccia utente
"""

import asyncio
import cv2
import math
import numpy as np
import time
import logging
from typing import Optional, Tuple, Dict
from datetime import datetime

# Import dei moduli del nostro sistema
from robot_controller import RobotController, RobotState, TargetState
from config import VisionConfig, SystemConfig, UIConfig, ControlConfig

# Setup logging
logging.basicConfig(level=getattr(logging, SystemConfig.LOG_LEVEL))

class VisionSystem:
    """Sistema di visione basato sui marker ArUco"""
    
    def __init__(self):
        self.cap = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
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
            "arena_valid": False
        }
        
        if ids is None:
            return result
        
        # Processo marker rilevati
        arena_markers = {}
        robot_data = None
        
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
                
            elif marker_id in VisionConfig.ARENA_MARKER_IDS:
                # Marker arena
                arena_markers[marker_id] = center
        
        # Processa marker arena
        if len(arena_markers) >= 2:
            result["arena_markers"] = arena_markers
            result["arena_valid"] = True
            self.arena_markers = arena_markers
        
        # Processa robot se trovato e arena valida
        if robot_data and result["arena_valid"]:
            robot_arena_coords = self._transform_to_arena_coordinates(
                robot_data["center"], arena_markers
            )
            
            if robot_arena_coords:
                result["robot_found"] = True
                result["robot_x"], result["robot_y"] = robot_arena_coords
                result["robot_theta"] = self._calculate_orientation(robot_data["corners"])
                self.last_detection_time = time.time()
        
        return result
    
    def _transform_to_arena_coordinates(self, robot_center, arena_markers) -> Optional[Tuple[float, float]]:
        """Trasforma coordinate pixel in coordinate arena [0-100]"""
        if len(arena_markers) < 2:
            return None
        
        arena_points = list(arena_markers.values())
        all_x = [p[0] for p in arena_points]
        all_y = [p[1] for p in arena_points]
        
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        
        if max_x == min_x or max_y == min_y:
            return None
        
        norm_x = (robot_center[0] - min_x) / (max_x - min_x)
        norm_y = (robot_center[1] - min_y) / (max_y - min_y)
        
        arena_x = norm_x * 100
        arena_y = norm_y * 100
        
        return arena_x, arena_y
    
    def _transform_arena_to_screen(self, arena_point, arena_markers, frame_width, frame_height):
        """Trasforma coordinate arena [0-100] in coordinate schermo"""
        if len(arena_markers) < 2:
            return None
        
        arena_points = list(arena_markers.values())
        all_x = [p[0] for p in arena_points]
        all_y = [p[1] for p in arena_points]
        
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        
        if max_x == min_x or max_y == min_y:
            return None
        
        # Trasformazione inversa: arena [0-100] -> schermo [pixel]
        norm_x = arena_point[0] / 100.0
        norm_y = arena_point[1] / 100.0
        
        screen_x = int(min_x + norm_x * (max_x - min_x))
        screen_y = int(min_y + norm_y * (max_y - min_y))
        
        return screen_x, screen_y
    
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

class IntegratedRobotSystem:
    """Sistema integrato completo"""
    
    def __init__(self):
        # Sottosistemi
        self.vision = VisionSystem()
        self.controller = RobotController()
        
        # Stati
        self.running = False
        self.manual_control = False
        self.target_set = False
        
        # Performance monitoring
        self.frame_count = 0
        self.fps_start_time = time.time()
        self.current_fps = 0.0
        
        # Mouse callback per setting target
        self.mouse_target = None
        self.last_target_sent = None  # Traccia ultimo target inviato
        
        # Follow mouse mode
        self.follow_mouse_mode = False
        self.current_mouse_pos = None  # Posizione corrente del mouse
        
        self.logger = logging.getLogger(__name__)
    
    def mouse_callback(self, event, x, y, flags, param):
        """Callback per mouse - gestisce target fisso o follow mode"""
        # Aggiorna sempre la posizione corrente del mouse
        if len(self.vision.arena_markers) >= 2:
            arena_coords = self.vision._transform_to_arena_coordinates((x, y), self.vision.arena_markers)
            if arena_coords:
                self.current_mouse_pos = arena_coords
        
        # Click per target fisso (solo se non in follow mode)
        if event == cv2.EVENT_LBUTTONDOWN and not self.follow_mouse_mode:
            if self.current_mouse_pos:
                arena_x, arena_y = self.current_mouse_pos
                new_target = (arena_x, arena_y)
                
                # Evita aggiornamenti troppo frequenti per click vicini
                if (self.mouse_target is None or 
                    abs(new_target[0] - self.mouse_target[0]) > 1.0 or 
                    abs(new_target[1] - self.mouse_target[1]) > 1.0):
                    
                    self.mouse_target = new_target
                    self.logger.info(f"🎯 Target fisso: ({arena_x:.1f}, {arena_y:.1f})")
                else:
                    self.logger.debug("Target troppo vicino al precedente, ignorato")
            else:
                self.logger.warning("Impossibile calcolare coordinate arena per il target")
    
    async def initialize(self) -> bool:
        """Inizializza tutto il sistema"""
        self.logger.info("Inizializzando sistema integrato...")
        
        # Inizializza visione
        if not self.vision.initialize_camera():
            return False
        
        # Inizializza controller
        if not await self.controller.initialize():
            return False
            
        self.logger.info("Sistema integrato inizializzato con successo")
        return True
    
    def draw_overlay(self, frame, vision_data) -> np.ndarray:
        """Disegna overlay informativo sul frame"""
        overlay_frame = frame.copy()
        frame_height, frame_width = frame.shape[:2]
        
        # Disegna marker arena
        for marker_id, center in vision_data["arena_markers"].items():
            color = UIConfig.COLOR_ARENA_MARKERS.get(marker_id, (128, 128, 128))
            cv2.circle(overlay_frame, center, 8, color, -1)
            cv2.putText(overlay_frame, str(marker_id), 
                       (center[0] + 12, center[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, UIConfig.FONT_SCALE, color, UIConfig.FONT_THICKNESS)
        
        # Disegna robot se trovato
        if vision_data["robot_found"]:
            # Usa la trasformazione corretta arena -> schermo
            robot_screen_pos = self.vision._transform_arena_to_screen(
                (vision_data["robot_x"], vision_data["robot_y"]), 
                vision_data["arena_markers"], frame_width, frame_height
            )
            
            if robot_screen_pos:
                robot_screen_x, robot_screen_y = robot_screen_pos
                
                # Cambia colore robot se ha raggiunto il target
                if self.mouse_target and self.controller.is_at_target():
                    robot_color = (0, 255, 0)  # Verde = target raggiunto
                    cv2.circle(overlay_frame, (robot_screen_x, robot_screen_y), 15, robot_color, 3)  # Bordo verde
                    cv2.putText(overlay_frame, "STOP", 
                               (robot_screen_x - 20, robot_screen_y - 25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, robot_color, 2)
                else:
                    robot_color = UIConfig.COLOR_ROBOT
                
                cv2.circle(overlay_frame, (robot_screen_x, robot_screen_y), 12, robot_color, -1)
                
                # Disegna orientamento robot (freccia rossa)
                theta_rad = np.radians(vision_data["robot_theta"])
                end_x = int(robot_screen_x + 30 * np.cos(theta_rad))
                end_y = int(robot_screen_y + 30 * np.sin(theta_rad))
                cv2.arrowedLine(overlay_frame, (robot_screen_x, robot_screen_y), (end_x, end_y), (0, 0, 255), 3)
                
                # DISEGNA VETTORE DI CONTROLLO (viola)
                vector_info = self.controller.get_vector_info()
                if vector_info["control_enabled"] and vector_info["intensity"] > 0.01:
                    # Converti angolo vettore dal sistema robot al sistema mondo
                    # Il vettore è nel sistema robot, quindi aggiungi l'orientamento del robot
                    vector_world_angle = theta_rad + vector_info["angle_rad"]
                    
                    # Scala lunghezza in base all'intensità (max 50 pixel)
                    vector_length = min(vector_info["intensity"] * 100, 50)
                    
                    # Calcola punto finale del vettore
                    vector_end_x = int(robot_screen_x + vector_length * np.cos(vector_world_angle))
                    vector_end_y = int(robot_screen_y + vector_length * np.sin(vector_world_angle))
                    
                    # Disegna vettore viola
                    cv2.arrowedLine(overlay_frame, (robot_screen_x, robot_screen_y), 
                                   (vector_end_x, vector_end_y), UIConfig.COLOR_VECTOR, 2, tipLength=0.3)
                    
                    # Aggiungi testo con intensità
                    cv2.putText(overlay_frame, f"v={vector_info['intensity']:.2f}", 
                               (vector_end_x + 5, vector_end_y - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, UIConfig.COLOR_VECTOR, 1)
        
        # Disegna target basato sulla modalità
        if self.follow_mouse_mode:
            # FOLLOW MODE: Disegna cursore mouse come target
            if self.current_mouse_pos:
                target_screen_pos = self.vision._transform_arena_to_screen(
                    self.current_mouse_pos, vision_data["arena_markers"], frame_width, frame_height
                )
                
                if target_screen_pos:
                    target_screen_x, target_screen_y = target_screen_pos
                    # Cursore più dinamico per follow mode
                    cv2.circle(overlay_frame, (target_screen_x, target_screen_y), 8, (0, 255, 255), 2)  # Giallo
                    cv2.circle(overlay_frame, (target_screen_x, target_screen_y), 3, (0, 255, 255), -1)  # Centro
                    cv2.putText(overlay_frame, "FOLLOW", 
                               (target_screen_x + 15, target_screen_y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        else:
            # TARGET MODE: Disegna target fisso
            if self.mouse_target:
                target_screen_pos = self.vision._transform_arena_to_screen(
                    self.mouse_target, vision_data["arena_markers"], frame_width, frame_height
                )
                
                if target_screen_pos:
                    target_screen_x, target_screen_y = target_screen_pos
                    cv2.circle(overlay_frame, (target_screen_x, target_screen_y), 15, UIConfig.COLOR_TARGET, 3)
                    cv2.putText(overlay_frame, "TARGET", 
                               (target_screen_x + 20, target_screen_y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, UIConfig.COLOR_TARGET, 2)
        
        # Disegna perimetro arena
        if len(vision_data["arena_markers"]) >= 3:
            points = list(vision_data["arena_markers"].values())
            # Ordina punti per angolo
            center = np.mean(points, axis=0)
            angles = [np.arctan2(p[1] - center[1], p[0] - center[0]) for p in points]
            sorted_points = [p for _, p in sorted(zip(angles, points))]
            
            for i in range(len(sorted_points)):
                start = sorted_points[i]
                end = sorted_points[(i + 1) % len(sorted_points)]
                cv2.line(overlay_frame, start, end, UIConfig.COLOR_BOUNDARY, 3)
        
        # Panel informazioni
        self._draw_info_panel(overlay_frame, vision_data)
        
        return overlay_frame
    
    def _draw_info_panel(self, frame, vision_data):
        """Disegna pannello informazioni"""
        frame_height, frame_width = frame.shape[:2]
        
        info_lines = [
            f"FPS: {self.current_fps:.1f}",
            f"Arena: {len(vision_data['arena_markers'])}/4",
            f"Robot: {'OK' if vision_data['robot_found'] else 'LOST'}",
            f"Control: {'ON' if self.controller.control_enabled else 'OFF'}",
            f"Mode: {'🐭 FOLLOW' if self.follow_mouse_mode else '🎯 TARGET'}",
            "--- CALIBRAZIONE ---",
            "WASD per muoversi",
            "SPACE per fermarsi"
        ]
        
        if vision_data["robot_found"]:
            info_lines.extend([
                f"Pos: ({vision_data['robot_x']:.1f}, {vision_data['robot_y']:.1f})",
                f"theta: {vision_data['robot_theta']:.0f}"
            ])
            
            # Aggiungi informazioni vettore di controllo
            vector_info = self.controller.get_vector_info()
            if vector_info["control_enabled"]:
                vector_angle_deg = math.degrees(vector_info["angle_rad"])
                info_lines.extend([
                    f"--- VETTORE CONTROLLO ---",
                    f"Intensità: {vector_info['intensity']:.3f}",
                    f"Angolo: {vector_angle_deg:.1f}°",
                    f"Distanza: {vector_info['distance']:.1f}"
                ])
        # Informazioni target basate sulla modalità
        if self.follow_mouse_mode:
            if self.current_mouse_pos and vision_data["robot_found"]:
                # Distanza dal cursore mouse
                error_x = self.current_mouse_pos[0] - vision_data["robot_x"]
                error_y = self.current_mouse_pos[1] - vision_data["robot_y"]
                distance = math.sqrt(error_x**2 + error_y**2)
                info_lines.append(f"Mouse: ({self.current_mouse_pos[0]:.1f}, {self.current_mouse_pos[1]:.1f})")
                info_lines.append(f"Dist: {distance:.1f} unità")
            elif self.current_mouse_pos:
                info_lines.append(f"Mouse: ({self.current_mouse_pos[0]:.1f}, {self.current_mouse_pos[1]:.1f})")
                info_lines.append("Dist: ? (robot perso)")
            else:
                info_lines.append("Mouse: fuori arena")
        else:
            # Target fisso
            if self.mouse_target:
                if vision_data["robot_found"]:
                    # Calcola distanza dal target
                    error_x = self.mouse_target[0] - vision_data["robot_x"]
                    error_y = self.mouse_target[1] - vision_data["robot_y"]
                    distance = math.sqrt(error_x**2 + error_y**2)
                    
                    if self.controller.is_at_target():
                        info_lines.append(f"Target: 🎉 RAGGIUNTO!")
                    else:
                        info_lines.append(f"Target: ({self.mouse_target[0]:.1f}, {self.mouse_target[1]:.1f})")
                        info_lines.append(f"Dist: {distance:.1f} unità")
                else:
                    info_lines.append(f"Target: ({self.mouse_target[0]:.1f}, {self.mouse_target[1]:.1f})")
                    info_lines.append("Dist: ? (robot perso)")
            else:
                info_lines.append("Target: Non impostato")
        
        # Sfondo pannello
        panel_width = 250
        panel_height = len(info_lines) * 20 + 10
        start_x = frame_width - panel_width - 10
        start_y = 10
        
        overlay = frame.copy()
        cv2.rectangle(overlay, (start_x, start_y), 
                     (start_x + panel_width, start_y + panel_height), 
                     (0, 0, 0), -1)
        cv2.addWeighted(overlay, UIConfig.INFO_PANEL_ALPHA, frame, 1 - UIConfig.INFO_PANEL_ALPHA, 0, frame)
        
        # Testo
        y_offset = start_y + 18
        for line in info_lines:
            cv2.putText(frame, line, (start_x + 5, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, UIConfig.FONT_SCALE, (255, 255, 255), UIConfig.FONT_THICKNESS)
            y_offset += 18
    
    async def main_loop(self):
        """Loop principale del sistema"""
        self.running = True
        
        # Setup finestra
        window_name = UIConfig.WINDOW_NAME
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, UIConfig.DEFAULT_WINDOW_WIDTH, UIConfig.DEFAULT_WINDOW_HEIGHT)
        
        # Setup controllo asincrono
        control_task = None
        
        self.logger.info("Sistema avviato - Controlli: 'q'=quit, 'c'=toggle control, click=set target")
        
        try:
            while self.running:
                # Cattura frame
                ret, frame = self.vision.cap.read()
                if not ret:
                    self.logger.error("Errore lettura frame")
                    break
                
                # Processa visione
                vision_data = self.vision.process_frame(frame)
                
                # Aggiorna controller con dati visione
                if vision_data["robot_found"]:
                    self.controller.update_robot_state(
                        vision_data["robot_x"],
                        vision_data["robot_y"], 
                        vision_data["robot_theta"],
                        tracking_valid=True
                    )
                else:
                    self.controller.update_robot_state(0, 0, 0, tracking_valid=False)
                
                # Gestisci target - differenzia tra follow mode e target fisso
                current_target = None
                
                if self.follow_mouse_mode:
                    # FOLLOW MOUSE: usa posizione corrente del mouse come target continuo
                    if self.current_mouse_pos:
                        current_target = self.current_mouse_pos
                        # Invia target solo se cambiato significativamente
                        if (not self.last_target_sent or 
                            abs(current_target[0] - self.last_target_sent[0]) > 2.0 or 
                            abs(current_target[1] - self.last_target_sent[1]) > 2.0):
                            
                            self.controller.set_target_position(current_target[0], current_target[1])
                            self.last_target_sent = current_target
                else:
                    # TARGET FISSO: usa target da click mouse
                    if self.mouse_target and self.mouse_target != self.last_target_sent:
                        current_target = self.mouse_target
                        self.logger.info(f"🎯 Nuovo target fisso: {current_target}")
                        self.controller.set_target_position(current_target[0], current_target[1])
                        self.last_target_sent = current_target
                
                # Auto-stop solo per target fissi (non in follow mode)
                if (not self.follow_mouse_mode and 
                    self.mouse_target and  # Usa self.mouse_target invece di current_target
                    vision_data["robot_found"] and
                    self.controller.is_at_target()):
                    
                    self.logger.info(f"🎉 Target raggiunto! Auto-stop e reset target")
                    await self.controller.stop()
                    self.mouse_target = None
                    self.last_target_sent = None
                
                # Disegna overlay
                display_frame = self.draw_overlay(frame, vision_data)
                
                # Setup mouse callback
                cv2.setMouseCallback(window_name, self.mouse_callback, display_frame)
                
                # Mostra frame
                cv2.imshow(window_name, display_frame)
                
                # Calcola FPS
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    current_time = time.time()
                    self.current_fps = 30 / (current_time - self.fps_start_time)
                    self.fps_start_time = current_time
                
                # Gestisci input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    # Toggle controllo
                    if self.controller.control_enabled:
                        await self.controller.stop()
                        if control_task:
                            control_task.cancel()
                            control_task = None
                        self.logger.info("Controllo disattivato")
                    else:
                        await self.controller.start_continuous_control()
                        control_task = asyncio.create_task(self._control_loop())
                        self.logger.info("Controllo attivato")
                elif key == ord('s'):
                    # Stop di emergenza
                    await self.controller.emergency_stop()
                    if control_task:
                        control_task.cancel()
                        control_task = None
                elif key == ord('r'):
                    # Reset target
                    self.mouse_target = None
                    self.last_target_sent = None  # Reset anche il tracking
                    await self.controller.stop()
                    self.logger.info("🔄 Target resettato")
                elif key == ord('f'):
                    # Toggle Follow Mouse Mode
                    self.follow_mouse_mode = not self.follow_mouse_mode
                    if self.follow_mouse_mode:
                        self.mouse_target = None  # Resetta target fisso
                        self.last_target_sent = None
                        self.logger.info("🐭 FOLLOW MOUSE attivato - Il robot seguirà il cursore")
                    else:
                        await self.controller.stop()
                        self.logger.info("🎯 FOLLOW MOUSE disattivato - Torna a target fissi")
                    
                # CONTROLLI MANUALI per calibrazione
                elif key == ord('w'):
                    # Avanti - CORRETTO: y positivo = avanti
                    await self.controller.send_manual_command(0.0, ControlConfig.MANUAL_LINEAR_SPEED)
                    self.logger.info("⬆️ Movimento manuale: AVANTI")
                elif key == ord('s') and cv2.waitKey(1) & 0xFF != ord('s'):  # Evita conflitto con stop
                    pass  # Skip, 's' è già usato per stop
                elif key == ord('x'):
                    # Indietro - CORRETTO: y negativo = indietro
                    await self.controller.send_manual_command(0.0, -ControlConfig.MANUAL_LINEAR_SPEED)
                    self.logger.info("⬇️ Movimento manuale: INDIETRO")
                elif key == ord('a'):
                    # Sinistra (rotazione) - TEST: se va a destra, invertire segno
                    await self.controller.send_manual_command(-ControlConfig.MANUAL_ANGULAR_SPEED, 0.0)
                    self.logger.info("⬅️ Movimento manuale: SINISTRA (TEST)")
                elif key == ord('d'):
                    # Destra (rotazione) - TEST: se va a sinistra, invertire segno
                    await self.controller.send_manual_command(ControlConfig.MANUAL_ANGULAR_SPEED, 0.0)
                    self.logger.info("➡️ Movimento manuale: DESTRA (TEST)")
                elif key == ord(' '):
                    # Stop immediato (barra spazio)
                    await self.controller.send_manual_command(0.0, 0.0)
                    self.logger.info("⏹️ Stop manuale")
                
                await asyncio.sleep(1.0 / SystemConfig.UI_UPDATE_RATE)
                
        except Exception as e:
            self.logger.error(f"Errore nel main loop: {e}")
        finally:
            # Cleanup
            if control_task:
                control_task.cancel()
            await self.controller.stop()
            await self.controller.communication.disconnect()
            self.vision.release()
            cv2.destroyAllWindows()
            self.logger.info("Sistema arrestato")
    
    async def _control_loop(self):
        """Loop di controllo asincrono"""
        try:
            while self.controller.control_enabled:
                await self.controller.control_loop_step()
                await asyncio.sleep(1.0 / SystemConfig.CONTROL_LOOP_RATE)
        except asyncio.CancelledError:
            pass

# Esecuzione principale
async def main():
    print("=== Sistema DDR Robot Integrato ===")
    
    system = IntegratedRobotSystem()
    
    if not await system.initialize():
        print("❌ Errore inizializzazione sistema")
        return
    
    print("✅ Sistema inizializzato")
    print("🎮 Controlli:")
    print("   === NAVIGAZIONE AUTOMATICA ===")
    print("   - Click mouse: Imposta target fisso")
    print("   - 'f': Toggle FOLLOW MOUSE mode")
    print("   - 'c': Toggle controllo automatico")
    print("   - 'r': Reset target")
    print("   === CONTROLLO MANUALE (CALIBRAZIONE) ===")
    print("   - 'w': Avanti")
    print("   - 'x': Indietro")
    print("   - 'a': Sinistra (rotazione)")
    print("   - 'd': Destra (rotazione)")
    print("   - SPAZIO: Stop immediato")
    print("   === SISTEMA ===")
    print("   - 's': Stop emergenza")
    print("   - 'q': Quit")
    
    await system.main_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nSistema interrotto da utente")
