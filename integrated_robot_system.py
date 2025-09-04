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
from robot.ui.displaymanager import DisplayManager
from robot.vision.coords import Coordinates
from robot.vision.visionsystem import VisionSystem
from robot.control.robot_controller import RobotController, RobotState, TargetState
from config import VisionConfig, SystemConfig, UIConfig, ControlConfig
from robot.path.pathfinding import PathfindingSystem, PathPoint

# Setup logging
logging.basicConfig(level=getattr(logging, SystemConfig.LOG_LEVEL))

class IntegratedRobotSystem:
    """Sistema integrato completo"""
    
    def __init__(self):
        # Sottosistemi
        self.coords = Coordinates()
        self.controller = RobotController()
        self.pathfinding = PathfindingSystem()
        self.vision = VisionSystem(self.coords)
        self.display = DisplayManager(
            self.vision, 
            self.controller, 
            self.coords, 
            self.pathfinding,
            system_ref=self  # Passa riferimento al sistema
        )
        
        # Stati
        self.running = False
        self.manual_control = False
        self.target_set = False
        
        # Modalità di navigazione
        self.pathfinding_mode = False
        
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
        
        # Obstacle tracking
        self._last_obstacle_count = 0
        
        self.logger = logging.getLogger(__name__)
    
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
                
                # Log ostacoli quando vengono rilevati per la prima volta
                if vision_data["obstacles"] and hasattr(self, '_last_obstacle_count'):
                    if len(vision_data["obstacles"]) != self._last_obstacle_count:
                        obstacle_ids = list(vision_data["obstacles"].keys())
                        self.logger.info(f"🚧 Ostacoli rilevati: {obstacle_ids}")
                elif vision_data["obstacles"]:
                    obstacle_ids = list(vision_data["obstacles"].keys())
                    self.logger.info(f"🚧 Ostacoli rilevati: {obstacle_ids}")
                    
                self._last_obstacle_count = len(vision_data["obstacles"])
                
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
                
                # NUOVO: Gestione target con pathfinding
                current_target = None
                
                if self.pathfinding_mode:
                    # === MODALITÀ PATHFINDING ===
                    await self._handle_pathfinding_mode(vision_data)
                else:
                    # === MODALITÀ CONTROLLO DIRETTO ===
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
                display_frame = self.display.draw_overlay(frame, vision_data)
                
                # Setup mouse callback
                cv2.setMouseCallback(window_name, self.display.mouse_callback, display_frame)
                
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
                    self.pathfinding.clear_path()  # NUOVO: Pulisci anche pathfinding
                    await self.controller.stop()
                    self.logger.info("🔄 Target e percorso resettati")
                elif key == ord('p'):
                    # NUOVO: Toggle Pathfinding Mode
                    self.pathfinding_mode = not self.pathfinding_mode
                    if self.pathfinding_mode:
                        self.pathfinding.clear_path()  # Reset pathfinding
                        self.logger.info("🗺️ PATHFINDING attivato - Il robot userà A* per evitare ostacoli")
                    else:
                        await self.controller.stop()
                        self.logger.info("➡️ PATHFINDING disattivato - Torna a controllo diretto")
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
    
    async def _handle_pathfinding_mode(self, vision_data):
        """Gestisce la modalità pathfinding"""
        if not vision_data["robot_found"]:
            return
            
        # Preparar coordinate ostacoli per pathfinding
        obstacles_arena = {}
        if vision_data["obstacles"] and vision_data["arena_valid"]:
            for obs_id, obs_center in vision_data["obstacles"].items():
                obs_arena_coords = self.coords._transform_to_arena_coordinates(
                    obs_center, vision_data["arena_markers"]
                )
                if obs_arena_coords:
                    obstacles_arena[obs_id] = obs_arena_coords
        
        # Determina target per pathfinding
        target_pos = None
        if self.follow_mouse_mode and self.current_mouse_pos:
            target_pos = self.current_mouse_pos
        elif not self.follow_mouse_mode and self.mouse_target:
            target_pos = self.mouse_target
            
        if not target_pos:
            # NESSUN TARGET: Se esiste un percorso valido, continua a seguirlo
            # altrimenti ferma il robot
            if not self.pathfinding.has_valid_path():
                await self.controller.stop()
                return
        else:
            # AGGIORNA PATHFINDING: Prova a calcolare nuovo percorso
            # MA mantieni quello vecchio se il calcolo fallisce
            path_updated = self.pathfinding.update_path(
                vision_data["robot_x"], vision_data["robot_y"],
                target_pos[0], target_pos[1],
                obstacles_arena
            )
            
            if path_updated:
                self.logger.info(f"🗺️ Pathfinding: Nuovo percorso calcolato")
            else:
                self.logger.debug(f"🗺️ Pathfinding: Usando percorso esistente")
        
        # USA IL PERCORSO ESISTENTE (nuovo o vecchio)
        current_waypoint = self.pathfinding.get_current_waypoint()
        if current_waypoint:
            # Imposta waypoint come target del controller SOLO se diverso
            if (not self.last_target_sent or 
                abs(current_waypoint.x - self.last_target_sent[0]) > 1.0 or 
                abs(current_waypoint.y - self.last_target_sent[1]) > 1.0):
                
                self.controller.set_target_position(current_waypoint.x, current_waypoint.y)
                self.last_target_sent = (current_waypoint.x, current_waypoint.y)
                self.logger.debug(f"🎯 Nuovo waypoint: ({current_waypoint.x:.1f}, {current_waypoint.y:.1f})")
            
            # Verifica se waypoint corrente è stato raggiunto
            if self.controller.is_at_target():
                waypoint_advanced = self.pathfinding.advance_waypoint(
                    vision_data["robot_x"], vision_data["robot_y"]
                )
                
                if not waypoint_advanced:
                    # Percorso completato!
                    path_info = self.pathfinding.get_path_info()
                    if path_info["is_complete"]:
                        self.logger.info("🎉 Percorso pathfinding completato!")
                        await self.controller.stop()
                        
                        # Reset target se non in follow mode
                        if not self.follow_mouse_mode:
                            self.mouse_target = None
                            self.last_target_sent = None
        else:
            # NESSUN PERCORSO VALIDO: Ferma il robot
            self.logger.warning("⚠️ Nessun percorso valido, robot fermo")
            await self.controller.stop()
    
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
    print("   - 'p': Toggle PATHFINDING mode (A* con evitamento ostacoli)")
    print("   - 'c': Toggle controllo automatico")
    print("   - 'r': Reset target e percorso")
    print("   === CONTROLLO MANUALE (CALIBRAZIONE) ===")
    print("   - 'w': Avanti")
    print("   - 'x': Indietro")
    print("   - 'a': Sinistra (rotazione)")
    print("   - 'd': Destra (rotazione)")
    print("   - SPAZIO: Stop immediato")
    print("   === SISTEMA ===")
    print("   - 's': Stop emergenza")
    print("   - 'q': Quit")
    print("")
    print("🗺️ PATHFINDING: Algoritmo A* con evitamento ostacoli intelligente")
    print("   - Modalità PATHFINDING: Il robot calcola il percorso ottimale")
    print("   - Modalità DIRECT: Il robot va diretto al target (vecchio comportamento)")
    print("🚧 OSTACOLI: I marker con ID > 4 sono considerati ostacoli")
    print("   - Visualizzati come quadrati rossi sullo schermo")
    print("   - Coordinate mostrate nel pannello informazioni")
    
    await system.main_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nSistema interrotto da utente")
