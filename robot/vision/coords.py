from typing import Optional, Tuple, Dict
import numpy as np
import cv2


class Coordinates:
    def _order_arena_corners(self, arena_markers) -> Optional[list]:
        """Ordina i 4 marker dell'arena in senso orario: top-left, top-right, bottom-right, bottom-left"""
        if len(arena_markers) != 4:
            return None
        
        # Estrai i punti
        points = []
        for marker_id in sorted(arena_markers.keys()):
            points.append(arena_markers[marker_id])
        
        # Converte in numpy array per facilità di calcolo
        points = np.array(points)
        
        # Trova il centro
        center = np.mean(points, axis=0)
        
        # Ordina i punti in base all'angolo rispetto al centro
        # Calcola angoli rispetto al centro
        angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
        
        # Ordina per angolo (senso antiorario da -π a π)
        sorted_indices = np.argsort(angles)
        
        # Trova il punto più in alto a sinistra come punto di partenza
        top_left_idx = None
        min_sum = float('inf')
        
        for i in sorted_indices:
            point_sum = points[i][0] + points[i][1]  # x + y (più piccolo = più in alto a sinistra)
            if point_sum < min_sum:
                min_sum = point_sum
                top_left_idx = i
        
        # Riordina partendo dal top-left
        start_pos = np.where(sorted_indices == top_left_idx)[0][0]
        ordered_indices = np.roll(sorted_indices, -start_pos)
        
        # Ritorna i punti ordinati: top-left, top-right, bottom-right, bottom-left (senso orario)
        ordered_points = points[ordered_indices].tolist()
        
        return ordered_points

    def _transform_to_arena_coordinates_perspective(self, robot_center, arena_markers) -> Optional[Tuple[float, float]]:
        """Trasforma coordinate pixel in coordinate arena [0-100] usando trasformazione prospettica"""
        if len(arena_markers) != 4:
            # Fallback al metodo precedente se non abbiamo 4 marker
            return self._transform_to_arena_coordinates_legacy(robot_center, arena_markers)
        
        # Ordina i marker come corner dell'arena
        arena_corners = self._order_arena_corners(arena_markers)
        if arena_corners is None:
            return None
        
        # Definisci i 4 angoli dell'arena normalizzata [0-100]
        arena_rect = np.array([
            [0, 0],      # top-left
            [100, 0],    # top-right  
            [100, 100],  # bottom-right
            [0, 100]     # bottom-left
        ], dtype=np.float32)
        
        try:
            # Calcola la matrice di trasformazione omografica
            M = cv2.getPerspectiveTransform(
                np.array(arena_corners, dtype=np.float32),
                arena_rect
            )
            
            # Applica la trasformazione al punto del robot
            robot_point = np.array([[[robot_center[0], robot_center[1]]]], dtype=np.float32)
            arena_point = cv2.perspectiveTransform(robot_point, M)
            
            arena_x = float(arena_point[0][0][0])
            arena_y = float(arena_point[0][0][1])
            
            # Clamp ai limiti dell'arena
            arena_x = max(0, min(100, arena_x))
            arena_y = max(0, min(100, arena_y))
            
            return arena_x, arena_y
            
        except cv2.error:
            # Se la trasformazione fallisce, usa il metodo legacy
            return self._transform_to_arena_coordinates_legacy(robot_center, arena_markers)

    def _transform_to_arena_coordinates_legacy(self, robot_center, arena_markers) -> Optional[Tuple[float, float]]:
        """Metodo originale per trasformare coordinate pixel in coordinate arena [0-100]"""
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
    
    def _transform_to_arena_coordinates(self, robot_center, arena_markers) -> Optional[Tuple[float, float]]:
        """Trasforma coordinate pixel in coordinate arena [0-100]"""
        if len(arena_markers) == 4:
            # Usa trasformazione prospettica se abbiamo tutti e 4 i marker
            return self._transform_to_arena_coordinates_perspective(robot_center, arena_markers)
        else:
            # Fallback al metodo legacy per < 4 marker
            return self._transform_to_arena_coordinates_legacy(robot_center, arena_markers)
    
    def get_obstacle_arena_coordinates(self, obstacles_dict, arena_markers) -> Dict:
        """Trasforma le coordinate degli ostacoli in coordinate arena [0-100]"""
        obstacle_arena_coords = {}
        
        for obs_id, obs_center in obstacles_dict.items():
            arena_coords = self._transform_to_arena_coordinates(obs_center, arena_markers)
            if arena_coords:
                obstacle_arena_coords[obs_id] = arena_coords
                
        return obstacle_arena_coords
    
    def _transform_arena_to_screen(self, arena_point, arena_markers, frame_width, frame_height):
        """Trasforma coordinate arena [0-100] in coordinate schermo"""
        if len(arena_markers) == 4:
            # Usa trasformazione prospettica inversa se abbiamo 4 marker
            return self._transform_arena_to_screen_perspective(arena_point, arena_markers)
        else:
            # Fallback al metodo legacy
            return self._transform_arena_to_screen_legacy(arena_point, arena_markers, frame_width, frame_height)
    
    def _transform_arena_to_screen_perspective(self, arena_point, arena_markers):
        """Trasforma coordinate arena [0-100] in coordinate schermo usando trasformazione prospettica"""
        if len(arena_markers) != 4:
            return None
        
        # Ordina i marker come corner dell'arena
        arena_corners = self._order_arena_corners(arena_markers)
        if arena_corners is None:
            return None
        
        # Definisci i 4 angoli dell'arena normalizzata [0-100]
        arena_rect = np.array([
            [0, 0],      # top-left
            [100, 0],    # top-right  
            [100, 100],  # bottom-right
            [0, 100]     # bottom-left
        ], dtype=np.float32)
        
        try:
            # Calcola la matrice di trasformazione omografica inversa
            M = cv2.getPerspectiveTransform(
                arena_rect,
                np.array(arena_corners, dtype=np.float32)
            )
            
            # Applica la trasformazione al punto dell'arena
            arena_point_array = np.array([[[arena_point[0], arena_point[1]]]], dtype=np.float32)
            screen_point = cv2.perspectiveTransform(arena_point_array, M)
            
            screen_x = int(screen_point[0][0][0])
            screen_y = int(screen_point[0][0][1])
            
            return screen_x, screen_y
            
        except cv2.error:
            # Se la trasformazione fallisce, usa il metodo legacy
            return self._transform_arena_to_screen_legacy(arena_point, arena_markers, None, None)
    
    def _transform_arena_to_screen_legacy(self, arena_point, arena_markers, frame_width, frame_height):
        """Metodo originale per trasformare coordinate arena [0-100] in coordinate schermo"""
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
    
    def calculate_target_position_from_marker(self, marker_center, marker_corners, offset_distance=20, target_side="left"):
        """
        Calcola la posizione del target basandosi sull'orientamento del marker
        
        Args:
            marker_center: Centro del marker (x, y)
            marker_corners: Array dei 4 corner del marker
            offset_distance: Distanza in pixel dal marker
            target_side: Lato dove posizionare il target ("left", "right", "top", "bottom", "front", "back")
        
        Returns:
            Tuple (x, y) della posizione del target
        """
        if len(marker_corners) < 4:
            # Fallback: posizione di default a sinistra
            return (marker_center[0] - offset_distance, marker_center[1])
        
        # Calcola l'orientamento del marker
        # Usa il vettore dal primo al secondo corner per determinare l'orientamento
        corner1 = marker_corners[0]
        corner2 = marker_corners[1]
        
        # Vettore di orientamento del marker
        dx = corner2[0] - corner1[0]
        dy = corner2[1] - corner1[1]
        
        # Normalizza il vettore
        length = np.sqrt(dx*dx + dy*dy)
        if length == 0:
            return (marker_center[0] - offset_distance, marker_center[1])
        
        dx_norm = dx / length
        dy_norm = dy / length
        
        # Calcola il vettore perpendicolare (ruotato di 90 gradi)
        perp_dx = -dy_norm
        perp_dy = dx_norm
        
        # Posiziona il target in base al lato richiesto
        if target_side == "left":
            # Lato sinistro rispetto all'orientamento del marker
            target_x = marker_center[0] + perp_dx * offset_distance
            target_y = marker_center[1] + perp_dy * offset_distance
        elif target_side == "right":
            # Lato destro rispetto all'orientamento del marker
            target_x = marker_center[0] - perp_dx * offset_distance
            target_y = marker_center[1] - perp_dy * offset_distance
        elif target_side == "front":
            # Davanti al marker (nella direzione di orientamento)
            target_x = marker_center[0] + dx_norm * offset_distance
            target_y = marker_center[1] + dy_norm * offset_distance
        elif target_side == "back":
            # Dietro al marker (opposto alla direzione di orientamento)
            target_x = marker_center[0] - dx_norm * offset_distance
            target_y = marker_center[1] - dy_norm * offset_distance
        elif target_side == "top":
            # Sopra il marker (sempre verso l'alto nello schermo)
            target_x = marker_center[0]
            target_y = marker_center[1] - offset_distance
        elif target_side == "bottom":
            # Sotto il marker (sempre verso il basso nello schermo)
            target_x = marker_center[0]
            target_y = marker_center[1] + offset_distance
        else:
            # Default: lato sinistro
            target_x = marker_center[0] + perp_dx * offset_distance
            target_y = marker_center[1] + perp_dy * offset_distance
        
        return (int(target_x), int(target_y))