"""
pathfinding.py
Sistema di pathfinding per il robot DDR con evitamento ostacoli
"""

import math
import numpy as np
import heapq
from typing import List, Tuple, Dict, Optional, Set
from dataclasses import dataclass
from enum import Enum
import logging

from config import ControlConfig, PathfindingConfig

# Setup logging
logger = logging.getLogger(__name__)

class CellType(Enum):
    """Tipi di celle nella griglia"""
    FREE = 0
    OBSTACLE = 1
    ROBOT = 2
    TARGET = 3
    BOUNDARY = 4

@dataclass
class PathPoint:
    """Punto del percorso con coordinate arena e metadati"""
    x: float
    y: float
    cell_type: CellType = CellType.FREE
    distance_to_target: float = 0.0
    
    def __eq__(self, other):
        if not isinstance(other, PathPoint):
            return False
        return abs(self.x - other.x) < 0.1 and abs(self.y - other.y) < 0.1
    
    def __hash__(self):
        return hash((round(self.x, 1), round(self.y, 1)))

class DynamicWaypoint:
    """
    Waypoint dinamico che si muove continuamente lungo il percorso,
    restando sempre a una distanza fissa davanti al robot
    """
    
    def __init__(self, radius: float = None):
        self.radius = radius or PathfindingConfig.DYNAMIC_WAYPOINT_RADIUS
        self.current_position = None  # PathPoint corrente del waypoint
        self.path_progress = 0.0      # Progresso lungo il percorso (0.0 - 1.0)
        self.path = []                # Percorso completo
        
        self.logger = logging.getLogger(__name__ + ".DynamicWaypoint")
    
    def set_path(self, path: List[PathPoint]):
        """Imposta un nuovo percorso per il waypoint dinamico"""
        self.path = path.copy()
        self.path_progress = 0.0
        self.current_position = None
        self.logger.debug(f"Nuovo percorso impostato con {len(path)} punti")
    
    def update_position(self, robot_x: float, robot_y: float) -> Optional[PathPoint]:
        """
        Aggiorna la posizione del waypoint dinamico basandosi sulla posizione del robot
        
        Returns:
            PathPoint del waypoint corrente, None se non c'è percorso valido
        """
        if not self.path or len(self.path) < 2:
            return None
        
        # Trova il punto del percorso più vicino al robot
        closest_progress = self._find_closest_point_on_path(robot_x, robot_y)
        
        # Calcola la posizione target del waypoint (avanti di radius dal robot)
        target_progress = self._calculate_target_progress(closest_progress, robot_x, robot_y)
        
        # Smooth del movimento del waypoint per evitare scatti
        if self.path_progress == 0.0:
            # Prima volta: posiziona direttamente
            self.path_progress = target_progress
        else:
            # Smooth movement
            smoothing = PathfindingConfig.WAYPOINT_SMOOTHING
            self.path_progress = (smoothing * self.path_progress + 
                                (1.0 - smoothing) * target_progress)
        
        # Clamp progress tra 0 e 1
        self.path_progress = max(0.0, min(1.0, self.path_progress))
        
        # Calcola posizione interpolata lungo il percorso
        self.current_position = self._interpolate_position_on_path(self.path_progress)
        
        self.logger.debug(f"Waypoint aggiornato: progress={self.path_progress:.3f}, "
                         f"pos=({self.current_position.x:.1f}, {self.current_position.y:.1f})")
        
        return self.current_position
    
    def _find_closest_point_on_path(self, robot_x: float, robot_y: float) -> float:
        """Trova il punto del percorso più vicino al robot e ritorna il progress (0-1)"""
        min_distance = float('inf')
        closest_progress = 0.0
        
        # Controlla ogni segmento del percorso
        for i in range(len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i + 1]
            
            # Proiezione del robot sul segmento
            t, distance = self._project_point_on_segment(robot_x, robot_y, p1, p2)
            
            if distance < min_distance:
                min_distance = distance
                # Converti posizione locale del segmento in progress globale
                segment_progress = i / (len(self.path) - 1)
                segment_length = 1.0 / (len(self.path) - 1)
                closest_progress = segment_progress + t * segment_length
        
        return closest_progress
    
    def _project_point_on_segment(self, px: float, py: float, 
                                 p1: PathPoint, p2: PathPoint) -> Tuple[float, float]:
        """
        Proietta un punto su un segmento e ritorna (t, distance)
        t = 0 per p1, t = 1 per p2
        """
        # Vettore segmento
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        segment_length_sq = dx * dx + dy * dy
        
        if segment_length_sq < 1e-6:  # Segmento degenere
            distance = math.sqrt((px - p1.x)**2 + (py - p1.y)**2)
            return 0.0, distance
        
        # Proiezione
        t = max(0.0, min(1.0, ((px - p1.x) * dx + (py - p1.y) * dy) / segment_length_sq))
        
        # Punto proiettato
        proj_x = p1.x + t * dx
        proj_y = p1.y + t * dy
        
        # Distanza
        distance = math.sqrt((px - proj_x)**2 + (py - proj_y)**2)
        
        return t, distance
    
    def _calculate_target_progress(self, current_progress: float, 
                                  robot_x: float, robot_y: float) -> float:
        """Calcola dove dovrebbe essere il waypoint (avanti di radius dal robot)"""
        # Percorso totale
        total_length = self._calculate_path_length()
        if total_length < 1e-6:
            return current_progress
        
        # Distanza target dal robot
        target_distance_along_path = self.radius
        
        # Converti in incremento di progress
        progress_increment = target_distance_along_path / total_length
        
        # Nuovo progress target
        target_progress = current_progress + progress_increment
        
        return min(1.0, target_progress)  # Non superare la fine
    
    def _calculate_path_length(self) -> float:
        """Calcola la lunghezza totale del percorso"""
        if len(self.path) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i + 1]
            segment_length = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            total_length += segment_length
        
        return total_length
    
    def _interpolate_position_on_path(self, progress: float) -> PathPoint:
        """Interpola una posizione lungo il percorso basata sul progress (0-1)"""
        if not self.path:
            return PathPoint(0, 0)
        
        if len(self.path) == 1:
            return self.path[0]
        
        if progress <= 0.0:
            return self.path[0]
        
        if progress >= 1.0:
            return self.path[-1]
        
        # Trova il segmento corrispondente
        segment_count = len(self.path) - 1
        segment_progress = progress * segment_count
        segment_index = int(segment_progress)
        segment_t = segment_progress - segment_index
        
        # Clamp per sicurezza
        segment_index = min(segment_index, segment_count - 1)
        
        # Interpola tra i due punti del segmento
        p1 = self.path[segment_index]
        p2 = self.path[segment_index + 1]
        
        interpolated_x = p1.x + segment_t * (p2.x - p1.x)
        interpolated_y = p1.y + segment_t * (p2.y - p1.y)
        
        return PathPoint(interpolated_x, interpolated_y)
    
    def get_current_waypoint(self) -> Optional[PathPoint]:
        """Ritorna la posizione corrente del waypoint"""
        return self.current_position
    
    def is_path_complete(self) -> bool:
        """Verifica se il waypoint ha raggiunto la fine del percorso"""
        return self.path_progress >= 1.0
    
    def get_progress_info(self) -> Dict:
        """Ritorna informazioni sul progresso del waypoint"""
        return {
            "progress": self.path_progress,
            "progress_percent": self.path_progress * 100.0,
            "has_path": len(self.path) > 0,
            "is_complete": self.is_path_complete(),
            "current_position": self.current_position
        }

class GridMap:
    """Griglia per pathfinding con gestione ostacoli"""
    
    def __init__(self):
        self.grid_size = PathfindingConfig.GRID_SIZE
        self.resolution = PathfindingConfig.GRID_RESOLUTION
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=int)
        
        # Cache per ottimizzazioni
        self._obstacles_cache = {}
        self._last_obstacle_hash = None
        
    def arena_to_grid(self, arena_x: float, arena_y: float) -> Tuple[int, int]:
        """Converte coordinate arena [0-100] in coordinate griglia"""
        grid_x = int(arena_x / self.resolution)
        grid_y = int(arena_y / self.resolution)
        
        # Clamp ai limiti della griglia
        grid_x = max(0, min(self.grid_size - 1, grid_x))
        grid_y = max(0, min(self.grid_size - 1, grid_y))
        
        return grid_x, grid_y
    
    def grid_to_arena(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Converte coordinate griglia in coordinate arena [0-100]"""
        arena_x = (grid_x + 0.5) * self.resolution
        arena_y = (grid_y + 0.5) * self.resolution
        return arena_x, arena_y
    
    def update_obstacles(self, obstacles: Dict[int, Tuple[float, float]]):
        """Aggiorna la griglia con gli ostacoli rilevati"""
        # Hash degli ostacoli per cache
        obstacle_hash = hash(str(sorted(obstacles.items())))
        if obstacle_hash == self._last_obstacle_hash:
            return  # Nessun cambiamento
        
        # Reset griglia solo se ci sono cambiamenti
        self.grid.fill(0)
        
        self._last_obstacle_hash = obstacle_hash
        logger.debug(f"Aggiornamento griglia con {len(obstacles)} ostacoli")
        
        # Aggiungi boundaries
        self._add_boundaries()
        
        # Aggiungi ostacoli con inflazione
        for obs_id, (obs_x, obs_y) in obstacles.items():
            self._add_inflated_obstacle(obs_x, obs_y)
    
    def _add_boundaries(self):
        """Aggiunge i boundaries della arena alla griglia"""
        margin_cells = int(PathfindingConfig.BOUNDARY_MARGIN / self.resolution)
        
        # Top/bottom boundaries
        for x in range(self.grid_size):
            for y in range(margin_cells):
                self.grid[x, y] = CellType.BOUNDARY.value
                self.grid[x, self.grid_size - 1 - y] = CellType.BOUNDARY.value
        
        # Left/right boundaries  
        for y in range(self.grid_size):
            for x in range(margin_cells):
                self.grid[x, y] = CellType.BOUNDARY.value
                self.grid[self.grid_size - 1 - x, y] = CellType.BOUNDARY.value
    
    def _add_inflated_obstacle(self, obs_x: float, obs_y: float):
        """Aggiunge un ostacolo con inflazione per sicurezza"""
        center_grid_x, center_grid_y = self.arena_to_grid(obs_x, obs_y)
        
        # Raggio inflazione in celle
        inflation_cells = int(PathfindingConfig.OBSTACLE_INFLATION_RADIUS / self.resolution)
        
        logger.debug(f"Aggiungendo ostacolo in arena ({obs_x:.1f}, {obs_y:.1f}) -> grid ({center_grid_x}, {center_grid_y})")
        logger.debug(f"Raggio inflazione: {inflation_cells} celle")
        
        # Aggiungi ostacolo inflated
        obstacles_added = 0
        for dx in range(-inflation_cells, inflation_cells + 1):
            for dy in range(-inflation_cells, inflation_cells + 1):
                x, y = center_grid_x + dx, center_grid_y + dy
                
                # Check limiti griglia
                if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                    # Distanza euclidea per forma circolare
                    distance = math.sqrt(dx*dx + dy*dy)
                    if distance <= inflation_cells:
                        self.grid[x, y] = CellType.OBSTACLE.value
                        obstacles_added += 1
        
        logger.debug(f"Aggiunte {obstacles_added} celle ostacolo")
    
    def is_free(self, grid_x: int, grid_y: int) -> bool:
        """Verifica se una cella è libera"""
        if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
            return self.grid[grid_x, grid_y] == CellType.FREE.value
        return False
    
    def print_grid_debug(self, start=None, goal=None):
        """Stampa la griglia per debug (solo per griglia piccola)"""
        if self.grid_size > 30:
            print(f"Griglia {self.grid_size}x{self.grid_size} troppo grande per debug")
            return
            
        print(f"\nGriglia {self.grid_size}x{self.grid_size} (0=libero, 1=ostacolo, 4=boundary):")
        print("   ", end="")
        for x in range(self.grid_size):
            print(f"{x%10:2d}", end="")
        print()
        
        for y in range(self.grid_size):
            print(f"{y:2d} ", end="")
            for x in range(self.grid_size):
                cell_value = self.grid[x, y]
                
                # Simboli speciali per start/goal
                if start and (x, y) == start:
                    print(" S", end="")
                elif goal and (x, y) == goal:
                    print(" G", end="")
                elif cell_value == CellType.FREE.value:
                    print(" .", end="")
                elif cell_value == CellType.OBSTACLE.value:
                    print(" #", end="")
                elif cell_value == CellType.BOUNDARY.value:
                    print(" B", end="")
                else:
                    print(f"{cell_value:2d}", end="")
            print()
        print()
    
    def get_neighbors(self, grid_x: int, grid_y: int) -> List[Tuple[int, int, float]]:
        """Ritorna vicini validi con costo movimento"""
        neighbors = []
        
        # 8-connectivity (include diagonali)
        directions = [
            (-1, -1, PathfindingConfig.DIAGONAL_COST),  # NW
            (-1,  0, PathfindingConfig.STRAIGHT_COST),  # N
            (-1,  1, PathfindingConfig.DIAGONAL_COST),  # NE
            ( 0, -1, PathfindingConfig.STRAIGHT_COST),  # W
            ( 0,  1, PathfindingConfig.STRAIGHT_COST),  # E
            ( 1, -1, PathfindingConfig.DIAGONAL_COST),  # SW
            ( 1,  0, PathfindingConfig.STRAIGHT_COST),  # S
            ( 1,  1, PathfindingConfig.DIAGONAL_COST),  # SE
        ]
        
        for dx, dy, cost in directions:
            new_x, new_y = grid_x + dx, grid_y + dy
            if self.is_free(new_x, new_y):
                neighbors.append((new_x, new_y, cost))
        
        return neighbors

class AStarPathfinder:
    """Algoritmo A* per pathfinding"""
    
    def __init__(self):
        self.grid_map = GridMap()
    
    def find_path(self, start_x: float, start_y: float, 
                  goal_x: float, goal_y: float,
                  obstacles: Dict[int, Tuple[float, float]]) -> List[PathPoint]:
        """
        Trova il percorso ottimale da start a goal evitando ostacoli
        
        Args:
            start_x, start_y: Posizione iniziale (coordinate arena)
            goal_x, goal_y: Posizione target (coordinate arena)  
            obstacles: Dizionario {obs_id: (x, y)} ostacoli
            
        Returns:
            Lista di PathPoint che formano il percorso, vuota se non trovato
        """
        # Aggiorna griglia con ostacoli
        self.grid_map.update_obstacles(obstacles)
        
        # Converti a coordinate griglia
        start_grid = self.grid_map.arena_to_grid(start_x, start_y)
        goal_grid = self.grid_map.arena_to_grid(goal_x, goal_y)
        
        logger.debug(f"Pathfinding: ({start_x:.1f},{start_y:.1f}) -> ({goal_x:.1f},{goal_y:.1f})")
        logger.debug(f"Grid coords: {start_grid} -> {goal_grid}")
        logger.debug(f"Grid size: {self.grid_map.grid_size}x{self.grid_map.grid_size}")
        
        # DEBUG: Stampa griglia se piccola
        if self.grid_map.grid_size <= 30:
            self.grid_map.print_grid_debug(start_grid, goal_grid)
        
        # Verifica che start e goal siano validi
        if not self.grid_map.is_free(*start_grid):
            logger.warning(f"Posizione start occupata: {start_grid}")
            # Trova cella libera più vicina
            start_grid = self._find_nearest_free_cell(*start_grid)
            if start_grid is None:
                logger.error("Impossibile trovare posizione start valida")
                return []
        
        if not self.grid_map.is_free(*goal_grid):
            logger.warning(f"Posizione goal occupata: {goal_grid}")
            # Trova cella libera più vicina
            goal_grid = self._find_nearest_free_cell(*goal_grid)
            if goal_grid is None:
                logger.error("Impossibile trovare posizione goal valida")
                return []
        
        # Esegui A*
        path_grid = self._astar(start_grid, goal_grid)
        
        if not path_grid:
            logger.warning("Nessun percorso trovato")
            return []
        
        # Converti a coordinate arena
        path_points = []
        for grid_x, grid_y in path_grid:
            arena_x, arena_y = self.grid_map.grid_to_arena(grid_x, grid_y)
            
            # Calcola distanza al target per metadati
            dist_to_target = math.sqrt((arena_x - goal_x)**2 + (arena_y - goal_y)**2)
            
            point = PathPoint(
                x=arena_x,
                y=arena_y,
                distance_to_target=dist_to_target
            )
            path_points.append(point)
        
        logger.debug(f"Path grezzo: {len(path_points)} punti")
        for i, p in enumerate(path_points[:5]):  # Mostra primi 5
            logger.debug(f"  {i}: ({p.x:.1f}, {p.y:.1f})")
        
        # Ottimizza percorso
        if PathfindingConfig.ENABLE_PATH_SMOOTHING and len(path_points) > 2:
            path_points = self._smooth_path(path_points)
            logger.debug(f"Dopo smoothing: {len(path_points)} punti")
        
        path_points = self._reduce_waypoints(path_points)
        logger.debug(f"Dopo riduzione: {len(path_points)} punti")
        
        logger.info(f"Percorso trovato con {len(path_points)} waypoint")
        return path_points
    
    def _astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Implementazione algoritmo A* ottimizzata"""
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        closed_set = set()
        nodes_explored = 0
        
        logger.debug(f"A* iniziato: {start} -> {goal}")
        
        while open_set:
            nodes_explored += 1
            
            if nodes_explored > PathfindingConfig.MAX_PATH_LENGTH:
                logger.warning(f"A*: Raggiunta lunghezza massima percorso dopo {nodes_explored} nodi")
                break
                
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                # Ricostruisci percorso
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path_result = path[::-1]  # Inverti per ottenere ordine start->goal
                logger.debug(f"A* completato: {len(path_result)} passi, {nodes_explored} nodi esplorati")
                return path_result
            
            closed_set.add(current)
            
            # Esamina vicini
            for neighbor_x, neighbor_y, move_cost in self.grid_map.get_neighbors(*current):
                neighbor = (neighbor_x, neighbor_y)
                
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    
                    # Aggiungi a open set se non presente
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        logger.error(f"A* fallito dopo {nodes_explored} nodi esplorati")
        return []  # Nessun percorso trovato
    
    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euristica con bias diagonale per A*"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        
        # Distanza con fattore diagonale
        return PathfindingConfig.STRAIGHT_COST * (dx + dy) + \
               (PathfindingConfig.DIAGONAL_COST - 2 * PathfindingConfig.STRAIGHT_COST) * min(dx, dy)
    
    def _find_nearest_free_cell(self, grid_x: int, grid_y: int) -> Optional[Tuple[int, int]]:
        """Trova la cella libera più vicina a una posizione"""
        max_search_radius = 20  # celle massime di ricerca
        
        for radius in range(1, max_search_radius):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:  # Solo perimetro
                        candidate_x = grid_x + dx
                        candidate_y = grid_y + dy
                        
                        if self.grid_map.is_free(candidate_x, candidate_y):
                            return candidate_x, candidate_y
        
        return None
    
    def _smooth_path(self, path: List[PathPoint]) -> List[PathPoint]:
        """Smooth del percorso per renderlo più naturale - VERSIONE CONSERVATIVA"""
        if len(path) < 3:
            logger.debug("Path troppo corto per smoothing")
            return path
        
        smoothed = path.copy()  # Inizia con una copia completa
        
        # Smoothing molto conservativo - solo 1 iterazione con fattore basso
        for iteration in range(1):  # Solo 1 iterazione
            new_smoothed = [smoothed[0]]  # Mantieni primo punto
            
            for i in range(1, len(smoothed) - 1):
                prev_point = smoothed[i - 1]
                curr_point = smoothed[i]
                next_point = smoothed[i + 1]
                
                # Smoothing molto leggero (fattore 0.1 invece di 0.25)
                new_x = 0.1 * prev_point.x + 0.8 * curr_point.x + 0.1 * next_point.x
                new_y = 0.1 * prev_point.y + 0.8 * curr_point.y + 0.1 * next_point.y
                
                # Verifica che il punto smoothed sia ancora valido
                grid_x, grid_y = self.grid_map.arena_to_grid(new_x, new_y)
                if self.grid_map.is_free(grid_x, grid_y):
                    new_smoothed.append(PathPoint(new_x, new_y))
                else:
                    new_smoothed.append(curr_point)  # Mantieni originale se smoothed non valido
            
            new_smoothed.append(smoothed[-1])  # Mantieni ultimo punto
            smoothed = new_smoothed
        
        logger.debug(f"Smoothing: {len(path)} -> {len(smoothed)} punti")
        return smoothed
    
    def _reduce_waypoints(self, path: List[PathPoint]) -> List[PathPoint]:
        """Riduce waypoint ma interpola se troppo lontani per controllo fine"""
        if len(path) <= 2:
            logger.debug("Path troppo corto per riduzione")
            return path
        
        reduced = [path[0]]  # Inizia sempre con il primo punto
        current_idx = 0
        
        while current_idx < len(path) - 1:
            # Trova il punto più lontano raggiungibile in linea retta
            farthest_idx = current_idx + 1
            
            for test_idx in range(current_idx + 2, len(path)):
                if self._is_line_clear(path[current_idx], path[test_idx]):
                    distance = math.sqrt(
                        (path[test_idx].x - path[current_idx].x)**2 + 
                        (path[test_idx].y - path[current_idx].y)**2
                    )
                    # Accetta il punto se la distanza è ragionevole
                    if distance >= PathfindingConfig.MIN_WAYPOINT_DISTANCE:
                        farthest_idx = test_idx
                    # Non fare break, continua a cercare il punto più lontano
                else:
                    break  # Se non è chiaro, ferma la ricerca
            
            # Aggiungi il punto più lontano raggiungibile
            if farthest_idx < len(path):
                target_point = path[farthest_idx]
                current_point = path[current_idx]
                
                # Calcola distanza
                distance = math.sqrt(
                    (target_point.x - current_point.x)**2 + 
                    (target_point.y - current_point.y)**2
                )
                
                # NUOVO: Se la distanza è troppo grande, interpola punti intermedi
                if distance > PathfindingConfig.MAX_WAYPOINT_DISTANCE:
                    # Calcola quanti punti intermedi servono
                    num_segments = int(math.ceil(distance / PathfindingConfig.MAX_WAYPOINT_DISTANCE))
                    
                    # Aggiungi punti intermedi interpolati
                    for i in range(1, num_segments):
                        t = i / num_segments
                        interp_x = current_point.x + t * (target_point.x - current_point.x)
                        interp_y = current_point.y + t * (target_point.y - current_point.y)
                        
                        # Verifica che il punto interpolato sia sicuro
                        grid_x, grid_y = self.grid_map.arena_to_grid(interp_x, interp_y)
                        if self.grid_map.is_free(grid_x, grid_y):
                            interp_point = PathPoint(interp_x, interp_y)
                            reduced.append(interp_point)
                        else:
                            logger.warning(f"Punto interpolato non sicuro: ({interp_x:.1f}, {interp_y:.1f})")
                
                # Aggiungi il punto target finale
                reduced.append(target_point)
                current_idx = farthest_idx
            else:
                break
        
        # Assicurati che l'ultimo punto sia sempre incluso
        if len(reduced) == 0 or reduced[-1] != path[-1]:
            reduced.append(path[-1])
        
        logger.debug(f"Waypoint: {len(path)} grezzo -> {len(reduced)} finale (con interpolazione)")
        return reduced
    
    def _is_line_clear(self, start: PathPoint, end: PathPoint) -> bool:
        """Verifica se la linea tra due punti è libera da ostacoli"""
        steps = int(max(abs(end.x - start.x), abs(end.y - start.y)) / self.grid_map.resolution) + 1
        
        for i in range(steps + 1):
            t = i / steps if steps > 0 else 0
            
            check_x = start.x + t * (end.x - start.x)
            check_y = start.y + t * (end.y - start.y)
            
            grid_x, grid_y = self.grid_map.arena_to_grid(check_x, check_y)
            if not self.grid_map.is_free(grid_x, grid_y):
                return False
        
        return True

class PathfindingSystem:
    """Sistema completo di pathfinding per il robot con waypoint dinamico"""
    
    def __init__(self):
        self.pathfinder = AStarPathfinder()
        self.current_path = []
        self.dynamic_waypoint = DynamicWaypoint()
        
        # Cache per evitare ricalcoli
        self._last_obstacles = {}
        self._last_target = None
        self._last_robot_pos = None
        
        self.logger = logging.getLogger(__name__)
    
    def update_path(self, robot_x: float, robot_y: float,
                   target_x: float, target_y: float,
                   obstacles: Dict[int, Tuple[float, float]]) -> bool:
        """
        Aggiorna il percorso se necessario
        
        Returns:
            True se il percorso è stato aggiornato/calcolato, False se usa quello esistente
        """
        # Verifica se serve ricalcolare
        target_changed = self._last_target != (target_x, target_y)
        obstacles_changed = self._last_obstacles != obstacles
        robot_moved_significantly = (
            self._last_robot_pos is None or
            abs(robot_x - self._last_robot_pos[0]) > 8.0 or  # AUMENTATO: meno ricalcoli
            abs(robot_y - self._last_robot_pos[1]) > 8.0
        )
        
        # Se abbiamo un percorso valido e i cambiamenti sono minimi, mantienilo
        if (len(self.current_path) > 0 and 
            not target_changed and 
            not obstacles_changed and 
            not robot_moved_significantly):
            return False  # Usa percorso esistente
        
        self.logger.info(f"🔄 Ricalcolo percorso: target_changed={target_changed}, "
                        f"obstacles_changed={obstacles_changed}, robot_moved={robot_moved_significantly}")
        
        # Calcola nuovo percorso
        new_path = self.pathfinder.find_path(
            robot_x, robot_y, target_x, target_y, obstacles
        )
        
        if new_path:
            # NUOVO PERCORSO TROVATO
            self.current_path = new_path
            self.dynamic_waypoint.set_path(new_path)  # Imposta il nuovo percorso nel waypoint dinamico
            
            # Aggiorna cache
            self._last_target = (target_x, target_y)
            self._last_obstacles = obstacles.copy()
            self._last_robot_pos = (robot_x, robot_y)
            
            self.logger.info(f"🗺️ Nuovo percorso con {len(new_path)} waypoint")
            return True
        else:
            # PERCORSO NON TROVATO: mantieni quello vecchio se esiste
            if len(self.current_path) > 0:
                self.logger.warning("⚠️ Nuovo percorso non trovato, mantengo quello esistente")
                return False  # Usa percorso esistente
            else:
                self.logger.error("❌ Impossibile calcolare percorso e nessuno esistente")
                return False
    
    def get_current_waypoint(self, robot_x: float, robot_y: float) -> Optional[PathPoint]:
        """
        Ritorna il waypoint dinamico corrente basato sulla posizione del robot
        
        IMPORTANTE: Questo metodo ora richiede la posizione del robot per aggiornare
        il waypoint dinamico continuamente!
        """
        if not self.has_valid_path():
            return None
        
        # Aggiorna la posizione del waypoint dinamico
        return self.dynamic_waypoint.update_position(robot_x, robot_y)
    
    def has_valid_path(self) -> bool:
        """Verifica se esiste un percorso valido"""
        return len(self.current_path) > 0
    
    def is_path_complete(self, robot_x: float, robot_y: float, tolerance: float = None) -> bool:
        """
        Verifica se il percorso è stato completato
        
        Args:
            robot_x, robot_y: Posizione corrente robot
            tolerance: Tolleranza per considerare raggiunto il target
        """
        if not self.has_valid_path():
            return True  # Nessun percorso = completato
        
        if tolerance is None:
            tolerance = ControlConfig.POSITION_TOLERANCE
        
        # Controlla se siamo vicini al target finale
        final_target = self.current_path[-1]
        distance_to_final = math.sqrt(
            (robot_x - final_target.x)**2 + (robot_y - final_target.y)**2
        )
        
        path_complete = distance_to_final <= tolerance
        
        # DEBOUNCE del log per evitare spam
        if path_complete and not hasattr(self, '_completion_logged'):
            self.logger.info("🎯 Target finale raggiunto!")
            self._completion_logged = True
        elif not path_complete and hasattr(self, '_completion_logged'):
            # Reset flag se non siamo più al target
            delattr(self, '_completion_logged')
        
        return path_complete
    
    def get_path_info(self) -> Dict:
        """Ritorna informazioni sul percorso corrente"""
        waypoint_info = self.dynamic_waypoint.get_progress_info()
        
        return {
            "has_path": len(self.current_path) > 0,
            "total_waypoints": len(self.current_path),
            "path_progress": waypoint_info["progress"],
            "path_progress_percent": waypoint_info["progress_percent"],
            "is_complete": waypoint_info["is_complete"],
            "waypoint_position": waypoint_info["current_position"]
        }
    
    def get_full_path(self) -> List[PathPoint]:
        """Ritorna il percorso completo per visualizzazione"""
        return self.current_path.copy()
    
    def get_dynamic_waypoint_position(self) -> Optional[PathPoint]:
        """Ritorna la posizione corrente del waypoint dinamico"""
        return self.dynamic_waypoint.get_current_waypoint()
    
    def clear_path(self):
        """Cancella il percorso corrente"""
        self.current_path = []
        self.dynamic_waypoint.set_path([])
        self._last_target = None
        self._last_robot_pos = None
        self._last_obstacles = {}
        
        # Reset flag completion per evitare spam
        if hasattr(self, '_completion_logged'):
            delattr(self, '_completion_logged')
            
        self.logger.info("Percorso cancellato")
    
    # Metodi legacy per compatibilità (DEPRECATI)
    def advance_waypoint(self, robot_x: float, robot_y: float, 
                        tolerance: float = None) -> bool:
        """
        DEPRECATO: Il waypoint ora si muove automaticamente!
        Questo metodo è mantenuto solo per compatibilità.
        """
        self.logger.warning("advance_waypoint() è deprecato con waypoint dinamico!")
        return not self.is_path_complete(robot_x, robot_y, tolerance)

# Esempio di utilizzo e test
if __name__ == "__main__":
    # Setup logging per debug
    logging.basicConfig(level=logging.INFO, 
                       format='%(levelname)s: %(message)s')
    
    # Test del sistema di pathfinding con waypoint dinamico
    print("=== Test Sistema Pathfinding con Waypoint Dinamico ===")
    
    # Crea sistema
    pathfinding_system = PathfindingSystem()
    
    # Parametri test - SEMPLIFICATI per debug
    robot_pos = [20, 20]  # Lista per simulare movimento
    target_pos = (80, 80)
    obstacles = {
        5: (50, 50),  # Ostacolo centrale che DEVE essere evitato!
    }
    
    print(f"Robot start: {robot_pos}")
    print(f"Target: {target_pos}")
    print(f"Obstacles: {obstacles}")
    print(f"Waypoint radius: {PathfindingConfig.DYNAMIC_WAYPOINT_RADIUS} unità arena")
    
    # Calcola percorso iniziale
    path_updated = pathfinding_system.update_path(
        robot_pos[0], robot_pos[1],
        target_pos[0], target_pos[1],
        obstacles
    )
    
    if path_updated:
        path_info = pathfinding_system.get_path_info()
        print(f"\n✅ Percorso calcolato:")
        print(f"  Waypoint totali: {path_info['total_waypoints']}")
        
        # Simula movimento del robot lungo il percorso
        print(f"\n=== Simulazione Movimento Robot ===")
        simulation_steps = 10
        
        for step in range(simulation_steps):
            # Simula movimento del robot verso il waypoint corrente
            current_waypoint = pathfinding_system.get_current_waypoint(robot_pos[0], robot_pos[1])
            
            if current_waypoint is None:
                print(f"Step {step}: Nessun waypoint disponibile")
                break
            
            # Muovi robot verso waypoint (simulazione semplice)
            direction_x = current_waypoint.x - robot_pos[0]
            direction_y = current_waypoint.y - robot_pos[1]
            distance = math.sqrt(direction_x**2 + direction_y**2)
            
            if distance > 0:
                # Movimento di 5 unità verso il waypoint
                move_distance = min(5.0, distance)
                robot_pos[0] += (direction_x / distance) * move_distance
                robot_pos[1] += (direction_y / distance) * move_distance
            
            # Verifica se percorso completato
            if pathfinding_system.is_path_complete(robot_pos[0], robot_pos[1]):
                print(f"Step {step}: 🎯 TARGET RAGGIUNTO!")
                print(f"  Robot finale: ({robot_pos[0]:.1f}, {robot_pos[1]:.1f})")
                break
            
            # Mostra stato corrente
            path_info = pathfinding_system.get_path_info()
            print(f"Step {step}: Robot=({robot_pos[0]:.1f}, {robot_pos[1]:.1f}), "
                  f"Waypoint=({current_waypoint.x:.1f}, {current_waypoint.y:.1f}), "
                  f"Progress={path_info['path_progress_percent']:.1f}%")
        
        print(f"\n✅ Test waypoint dinamico completato!")
        
        # Mostra confronto con sistema classico
        print(f"\n=== Confronto Sistemi ===")
        print(f"🆚 Sistema CLASSICO: Robot va da punto fisso a punto fisso")
        print(f"   - Movimento a scatti")
        print(f"   - Fermate ad ogni waypoint")
        print(f"   - Controllo PID su target fissi")
        print(f"")
        print(f"🆕 Sistema DINAMICO: Waypoint si muove continuamente")
        print(f"   - Movimento fluido")
        print(f"   - Nessuna fermata")
        print(f"   - Controllo PID su target mobile a distanza fissa")
        print(f"   - Radius waypoint: {PathfindingConfig.DYNAMIC_WAYPOINT_RADIUS} unità")
        
    else:
        print(f"\n❌ Impossibile calcolare percorso")
        
        # Debug aggiuntivo in caso di fallimento
        print("\n=== DEBUG INFO ===")
        pathfinder = pathfinding_system.pathfinder
        pathfinder.grid_map.update_obstacles(obstacles)
        
        start_grid = pathfinder.grid_map.arena_to_grid(*robot_pos)
        goal_grid = pathfinder.grid_map.arena_to_grid(*target_pos)
        
        print(f"Start grid: {start_grid}, libera: {pathfinder.grid_map.is_free(*start_grid)}")
        print(f"Goal grid: {goal_grid}, libera: {pathfinder.grid_map.is_free(*goal_grid)}")
        
        pathfinder.grid_map.print_grid_debug(start_grid, goal_grid)
