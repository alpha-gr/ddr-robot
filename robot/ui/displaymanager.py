import math
import cv2
import numpy as np

from config import UIConfig


class DisplayManager:

    def __init__(self, vision_system=None, controller=None, coords=None, pathfinding=None, system_ref=None):
        self.coords = coords
        
        # Riferimenti ai sistemi esterni
        self.vision = vision_system
        self.controller = controller
        self.pathfinding = pathfinding
        
        # NUOVO: Riferimento al sistema principale per accedere al state condiviso
        self.system = system_ref
        
        # Aggiungi logger per i messaggi
        import logging
        self.logger = logging.getLogger(__name__)

    def mouse_callback(self, event, x, y, flags, param):
        """Callback per mouse - gestisce target fisso o follow mode"""
        # Aggiorna sempre la posizione corrente del mouse
        if len(self.vision.arena_markers) >= 2:
            arena_coords = self.coords._transform_to_arena_coordinates((x, y), self.vision.arena_markers)
            if arena_coords:
                self.system.current_mouse_pos = arena_coords
        
        # Click per target fisso (solo se non in follow mode)
        if event == cv2.EVENT_LBUTTONDOWN and not self.system.follow_mouse_mode:
            if self.system.current_mouse_pos:
                arena_x, arena_y = self.system.current_mouse_pos
                new_target = (arena_x, arena_y)
                
                # Evita aggiornamenti troppo frequenti per click vicini
                if (self.system.mouse_target is None or 
                    abs(new_target[0] - self.system.mouse_target[0]) > 1.0 or 
                    abs(new_target[1] - self.system.mouse_target[1]) > 1.0):
                    
                    self.system.mouse_target = new_target
                    self.logger.info(f"🎯 Target fisso: ({arena_x:.1f}, {arena_y:.1f})")
                else:
                    self.logger.debug("Target troppo vicino al precedente, ignorato")
            else:
                self.logger.warning("Impossibile calcolare coordinate arena per il target")

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
            
            # Disegna ostacoli (marker ID > 4)
            for marker_id, center in vision_data["obstacles"].items():
                color = UIConfig.COLOR_OBSTACLES
                # Disegna ostacolo come quadrato rosso più grande
                cv2.rectangle(overlay_frame, 
                            (center[0] - 12, center[1] - 12), 
                            (center[0] + 12, center[1] + 12), 
                            color, -1)
                # Bordo nero per visibilità
                cv2.rectangle(overlay_frame, 
                            (center[0] - 12, center[1] - 12), 
                            (center[0] + 12, center[1] + 12), 
                            (0, 0, 0), 2)
                # ID dell'ostacolo
                cv2.putText(overlay_frame, f"OBS{marker_id}", 
                        (center[0] + 15, center[1] + 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, UIConfig.FONT_SCALE, color, UIConfig.FONT_THICKNESS)
            
            # Disegna robot se trovato
            if vision_data["robot_found"]:
                # Usa la trasformazione corretta arena -> schermo
                robot_screen_pos = self.coords._transform_arena_to_screen(
                    (vision_data["robot_x"], vision_data["robot_y"]), 
                    vision_data["arena_markers"], frame_width, frame_height
                )
                
                if robot_screen_pos:
                    robot_screen_x, robot_screen_y = robot_screen_pos
                    
                    # Cambia colore robot se ha raggiunto il target
                    if self.system.mouse_target and self.controller.is_at_target():
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
            if self.system.follow_mouse_mode:
                # FOLLOW MODE: Disegna cursore mouse come target
                if self.system.current_mouse_pos:
                    target_screen_pos = self.coords._transform_arena_to_screen(
                        self.system.current_mouse_pos, vision_data["arena_markers"], frame_width, frame_height
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
                if self.system.mouse_target:
                    target_screen_pos = self.coords._transform_arena_to_screen(
                        self.system.mouse_target, vision_data["arena_markers"], frame_width, frame_height
                    )
                    
                    if target_screen_pos:
                        target_screen_x, target_screen_y = target_screen_pos
                        cv2.circle(overlay_frame, (target_screen_x, target_screen_y), 15, UIConfig.COLOR_TARGET, 3)
                        cv2.putText(overlay_frame, "TARGET", 
                                (target_screen_x + 20, target_screen_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, UIConfig.COLOR_TARGET, 2)
            
            # NUOVO: Disegna percorso pathfinding
            if self.system.pathfinding_mode and vision_data["arena_valid"]:
                self._draw_pathfinding_overlay(overlay_frame, vision_data, frame_width, frame_height)
            
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
            f"FPS: {self.system.current_fps:.1f}",
            f"Arena: {len(vision_data['arena_markers'])}/4",
            f"Obstacles: {len(vision_data['obstacles'])}",
            f"Robot: {'OK' if vision_data['robot_found'] else 'LOST'}",
            f"Control: {'ON' if self.controller.control_enabled else 'OFF'}",
            f"Mode: {'🐭 FOLLOW' if self.system.follow_mouse_mode else '🎯 TARGET'}",
            f"Navigation: {'🗺️ PATHFINDING' if self.system.pathfinding_mode else '➡️ DIRECT'}",
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
        if self.system.follow_mouse_mode:
            if self.system.current_mouse_pos and vision_data["robot_found"]:
                # Distanza dal cursore mouse
                error_x = self.system.current_mouse_pos[0] - vision_data["robot_x"]
                error_y = self.system.current_mouse_pos[1] - vision_data["robot_y"]
                distance = math.sqrt(error_x**2 + error_y**2)
                info_lines.append(f"Mouse: ({self.system.current_mouse_pos[0]:.1f}, {self.system.current_mouse_pos[1]:.1f})")
                info_lines.append(f"Dist: {distance:.1f} unità")
            elif self.system.current_mouse_pos:
                info_lines.append(f"Mouse: ({self.system.current_mouse_pos[0]:.1f}, {self.system.current_mouse_pos[1]:.1f})")
                info_lines.append("Dist: ? (robot perso)")
            else:
                info_lines.append("Mouse: fuori arena")
        else:
            # Target fisso
            if self.system.mouse_target:
                if vision_data["robot_found"]:
                    # Calcola distanza dal target
                    error_x = self.system.mouse_target[0] - vision_data["robot_x"]
                    error_y = self.system.mouse_target[1] - vision_data["robot_y"]
                    distance = math.sqrt(error_x**2 + error_y**2)
                    
                    if self.controller.is_at_target():
                        info_lines.append(f"Target: 🎉 RAGGIUNTO!")
                    else:
                        info_lines.append(f"Target: ({self.system.mouse_target[0]:.1f}, {self.system.mouse_target[1]:.1f})")
                        info_lines.append(f"Dist: {distance:.1f} unità")
                else:
                    info_lines.append(f"Target: ({self.system.mouse_target[0]:.1f}, {self.system.mouse_target[1]:.1f})")
                    info_lines.append("Dist: ? (robot perso)")
            else:
                info_lines.append("Target: Non impostato")
        
        # Informazioni ostacoli
        if vision_data["obstacles"]:
            info_lines.append("--- OSTACOLI ---")
            for obs_id in sorted(vision_data["obstacles"].keys()):
                # Calcola coordinate arena dell'ostacolo se possibile
                if vision_data["arena_valid"]:
                    obs_center = vision_data["obstacles"][obs_id]
                    obs_arena_coords = self.coords._transform_to_arena_coordinates(
                        obs_center, vision_data["arena_markers"]
                    )
                    if obs_arena_coords:
                        info_lines.append(f"OBS{obs_id}: ({obs_arena_coords[0]:.1f}, {obs_arena_coords[1]:.1f})")
                    else:
                        info_lines.append(f"OBS{obs_id}: rilevato")
                else:
                    info_lines.append(f"OBS{obs_id}: rilevato")
        
        # NUOVO: Informazioni pathfinding
        if self.system.pathfinding_mode:
            path_info = self.pathfinding.get_path_info()
            info_lines.append("--- PATHFINDING ---")
            if path_info["has_path"]:
                info_lines.extend([
                    f"Waypoints: {path_info['current_waypoint']}/{path_info['total_waypoints']}",
                    f"Rimangono: {path_info['waypoints_remaining']}",
                    f"Status: {'COMPLETE' if path_info['is_complete'] else 'NAVIGATING'}"
                ])
                
                # Info waypoint corrente
                current_wp = self.pathfinding.get_current_waypoint()
                if current_wp and vision_data["robot_found"]:
                    wp_distance = math.sqrt(
                        (current_wp.x - vision_data["robot_x"])**2 + 
                        (current_wp.y - vision_data["robot_y"])**2
                    )
                    info_lines.append(f"Next WP: ({current_wp.x:.1f}, {current_wp.y:.1f})")
                    info_lines.append(f"WP Dist: {wp_distance:.1f}")
            else:
                info_lines.append("Path: Calcolo...")
        
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
    
    def _draw_pathfinding_overlay(self, frame, vision_data, frame_width, frame_height):
        """Disegna overlay del pathfinding (percorso e waypoint)"""
        if not self.system.pathfinding_mode:
            return
            
        path_info = self.pathfinding.get_path_info()
        if not path_info["has_path"]:
            return
        
        full_path = self.pathfinding.get_full_path()
        if len(full_path) < 2:
            return
            
        # Disegna linee del percorso
        prev_screen_pos = None
        for i, path_point in enumerate(full_path):
            screen_pos = self.coords._transform_arena_to_screen(
                (path_point.x, path_point.y), vision_data["arena_markers"], frame_width, frame_height
            )
            
            if screen_pos and prev_screen_pos:
                # Linea del percorso
                cv2.line(frame, prev_screen_pos, screen_pos, UIConfig.COLOR_PATH, 3)
            
            prev_screen_pos = screen_pos
        
        # Disegna waypoint
        current_waypoint_index = path_info["current_waypoint"]
        
        for i, path_point in enumerate(full_path):
            screen_pos = self.coords._transform_arena_to_screen(
                (path_point.x, path_point.y), vision_data["arena_markers"], frame_width, frame_height
            )
            
            if screen_pos:
                if i == current_waypoint_index:
                    # Waypoint corrente - più grande e giallo
                    cv2.circle(frame, screen_pos, 8, UIConfig.COLOR_CURRENT_WAYPOINT, -1)
                    cv2.circle(frame, screen_pos, 10, (0, 0, 0), 2)  # Bordo nero
                    cv2.putText(frame, f"WP{i}", 
                               (screen_pos[0] + 12, screen_pos[1] - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, UIConfig.COLOR_CURRENT_WAYPOINT, 1)
                elif i < current_waypoint_index:
                    # Waypoint già visitati - grigi
                    cv2.circle(frame, screen_pos, 4, (128, 128, 128), -1)
                else:
                    # Waypoint futuri - arancioni
                    cv2.circle(frame, screen_pos, 6, UIConfig.COLOR_WAYPOINT, -1)
                    cv2.circle(frame, screen_pos, 7, (0, 0, 0), 1)  # Bordo nero sottile