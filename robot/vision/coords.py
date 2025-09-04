from typing import Optional, Tuple, Dict


class Coordinates:
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