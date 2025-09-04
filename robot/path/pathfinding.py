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

from config import ControlConfig, VisionConfig

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

class PathfindingConfig:
    """Configurazione per il pathfinding"""
    # Griglia di pathfinding
    GRID_RESOLUTION = 1     # Risoluzione griglia (unità arena per cella) 
    GRID_SIZE = int(100 / GRID_RESOLUTION)  # 50x50 grid per arena 100x100
    
    # Ostacoli - AUMENTATI per test
    OBSTACLE_INFLATION_RADIUS = 12  # Raggio inflazione ostacoli (unità arena)
    ROBOT_SAFETY_RADIUS = 4         # Raggio sicurezza robot (unità arena)
    
    # Algoritmo A*
    DIAGONAL_COST = 1.41421356       # sqrt(2) per movimenti diagonali
    STRAIGHT_COST = 1.0              # Costo movimenti cardinali
    
    # Ottimizzazione percorso - CONFIGURATE per più punti
    ENABLE_PATH_SMOOTHING = False
    SMOOTHING_ITERATIONS = 1         
    MIN_WAYPOINT_DISTANCE = 3.0      
    MAX_WAYPOINT_DISTANCE = 8.0     
    
    # Vincoli di navigazione
    MAX_PATH_LENGTH = 10000          
    BOUNDARY_MARGIN = ControlConfig.BOUNDARY_MARGIN 

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
        """Euristica Manhattan con bias diagonale per A*"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        
        # Distanza Manhattan con fattore diagonale
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
    """Sistema completo di pathfinding per il robot"""
    
    def __init__(self):
        self.pathfinder = AStarPathfinder()
        self.current_path = []
        self.current_waypoint_index = 0
        
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
            self.current_waypoint_index = 0
            
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
    
    def get_current_waypoint(self) -> Optional[PathPoint]:
        """Ritorna il waypoint corrente da raggiungere"""
        if (self.current_waypoint_index < len(self.current_path)):
            return self.current_path[self.current_waypoint_index]
        return None
    
    def has_valid_path(self) -> bool:
        """Verifica se esiste un percorso valido"""
        return len(self.current_path) > 0 and self.current_waypoint_index < len(self.current_path)
    
    def advance_waypoint(self, robot_x: float, robot_y: float, 
                        tolerance: float = None) -> bool:
        """
        Avanza al prossimo waypoint se quello corrente è stato raggiunto
        
        Returns:
            True se avanzato, False se non ci sono più waypoint
        """
        if tolerance is None:
            tolerance = ControlConfig.POSITION_TOLERANCE
        
        current_waypoint = self.get_current_waypoint()
        if current_waypoint is None:
            return False
        
        # Verifica se waypoint raggiunto
        distance = math.sqrt(
            (robot_x - current_waypoint.x)**2 + (robot_y - current_waypoint.y)**2
        )
        
        if distance <= tolerance:
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index < len(self.current_path):
                next_waypoint = self.current_path[self.current_waypoint_index]
                self.logger.info(f"Waypoint {self.current_waypoint_index}: "
                               f"({next_waypoint.x:.1f}, {next_waypoint.y:.1f})")
                return True
            else:
                self.logger.info("🎯 Percorso completato!")
                return False
        
        return True
    
    def get_path_info(self) -> Dict:
        """Ritorna informazioni sul percorso corrente"""
        return {
            "has_path": len(self.current_path) > 0,
            "total_waypoints": len(self.current_path),
            "current_waypoint": self.current_waypoint_index,
            "waypoints_remaining": max(0, len(self.current_path) - self.current_waypoint_index),
            "is_complete": self.current_waypoint_index >= len(self.current_path)
        }
    
    def get_full_path(self) -> List[PathPoint]:
        """Ritorna il percorso completo per visualizzazione"""
        return self.current_path.copy()
    
    def clear_path(self):
        """Cancella il percorso corrente"""
        self.current_path = []
        self.current_waypoint_index = 0
        self._last_target = None
        self._last_robot_pos = None
        self._last_obstacles = {}
        self.logger.info("Percorso cancellato")

# Esempio di utilizzo e test
if __name__ == "__main__":
    # Setup logging per debug
    logging.basicConfig(level=logging.DEBUG, 
                       format='%(levelname)s: %(message)s')
    
    # Test del sistema di pathfinding
    print("=== Test Sistema Pathfinding ===")
    
    # Crea sistema
    pathfinding_system = PathfindingSystem()
    
    # Parametri test - SEMPLIFICATI per debug
    robot_pos = (20, 20)
    target_pos = (60, 60)
    obstacles = {
        5: (40, 40),  # Ostacolo centrale che DEVE essere evitato!
    }
    
    print(f"Robot: {robot_pos}")
    print(f"Target: {target_pos}")
    print(f"Obstacles: {obstacles}")
    print(f"Inflazione ostacolo: {PathfindingConfig.OBSTACLE_INFLATION_RADIUS} unità arena")
    
    # Calcola percorso SENZA ottimizzazioni
    path_updated = pathfinding_system.update_path(
        robot_pos[0], robot_pos[1],
        target_pos[0], target_pos[1],
        obstacles
    )
    
    if path_updated:
        path_info = pathfinding_system.get_path_info()
        print(f"\n✅ Percorso calcolato:")
        print(f"  Waypoint totali: {path_info['total_waypoints']}")
        
        # Mostra TUTTI i waypoint
        full_path = pathfinding_system.get_full_path()
        for i, point in enumerate(full_path):
            print(f"  {i}: ({point.x:.1f}, {point.y:.1f})")
        
        print(f"\n✅ Test completato con successo!")
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
