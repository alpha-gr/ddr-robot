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
import traceback
from typing import Any, Optional, Tuple, Dict
from datetime import datetime

# Import dei moduli del nostro sistema
from robot.ui.displaymanager import DisplayManager
from robot.comm.robot_websocketserver import RobotWebSocketServer
from robot.vision.coords import Coordinates
from robot.vision.visionsystem import VisionSystem
from robot.control.robot_controller import RobotController, RobotState, TargetState
from config import VisionConfig, SystemConfig, UIConfig, ControlConfig, MapConfig, PathfindingConfig
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
        
        # WebSocket per controllo remoto
        self.websocket = RobotWebSocketServer()
        self.websocket.set_message_handler(self._handle_remote_message)

        # Stati
        self.running = False
        self.manual_control = False
        self.target_set = False
        
        # Stato controllo remoto
        self.remote_control_enabled = False
        self.remote_movement_active = False  # NUOVO: traccia movimenti remoti attivi
        self.remote_target_id = None  # NUOVO: ID del movimento per tracking
        
        # Debounce per evitare spam di stop
        self.last_stop_call = 0  # NUOVO: timestamp ultimo stop
        
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

        self.targets = {}
    
    async def _initialize_perspective_correction(self):
        """Inizializza la correzione prospettica automaticamente"""
        calibration_file = "perspective_calibration.npz"
        
        # Prima prova a caricare calibrazione salvata
        if self.vision.load_perspective_calibration(calibration_file):
            self.logger.info("✅ Calibrazione prospettica caricata automaticamente")
            return
        
        # Se non esiste calibrazione salvata, prova a calibrare automaticamente
        self.logger.info("🔄 Calibrazione prospettica non trovata, provo calibrazione automatica...")
        
        max_attempts = 30  # 3 secondi di tentativi
        attempt = 0
        
        while attempt < max_attempts:
            ret, frame = self.vision.cap.read()
            if not ret:
                attempt += 1
                await asyncio.sleep(0.1)
                continue
            
            # Prova calibrazione automatica
            if self.vision.calibrate_perspective(frame):
                self.logger.info("✅ Calibrazione prospettica automatica completata!")
                
                # Salva la calibrazione per la prossima volta
                if self.vision.save_perspective_calibration(calibration_file):
                    self.logger.info("💾 Calibrazione salvata automaticamente")
                
                return
            
            attempt += 1
            await asyncio.sleep(0.1)
        
        self.logger.warning("⚠️ Calibrazione prospettica automatica fallita")
        self.logger.warning("   Assicurati che tutti e 4 i marker arena (ID 1,2,3,4) siano visibili")
        self.logger.warning("   Il sistema continuerà senza correzione prospettica")
    
    async def initialize(self) -> bool:
        """Inizializza tutto il sistema"""
        self.logger.info("Inizializzando sistema integrato...")
        
        # Inizializza visione
        if not self.vision.initialize_camera():
            return False
        
        # NUOVO: Calibrazione automatica prospettiva
        await self._initialize_perspective_correction()
        
        # Inizializza controller
        if not await self.controller.initialize():
            return False
        
        # NUOVO: Inizializza WebSocket server
        try:
            await self.websocket.start()
            self.logger.info("🌐 WebSocket server avviato su localhost:8765")
        except Exception as e:
            self.logger.error(f"Errore avvio WebSocket server: {e}")
            return False
            
        self.logger.info("Sistema integrato inizializzato con successo")
        return True
    
    async def _handle_remote_message(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """Gestisce messaggi remoti"""
        try:
            command = message.get("command")
            
            if command == "engage":
                return await self._handle_engage()
            elif command == "disengage":
                return await self._handle_disengage()
            elif command == "moverobot":
                x = message.get("x")
                y = message.get("y")
                if x is None or y is None:
                    return {"status": "error", "message": "x e y sono richiesti"}
                return await self._handle_move_robot(x, y)
            elif command == "movetosquare":
                x = message.get("x")
                y = message.get("y")
                if x is None or y is None:
                    return {"status": "error", "message": "x e y sono richiesti"}
                return await self._handle_move_square(x, y)
            elif command == "gototarget":
                target_name = message.get("target")
                if not target_name:
                    return {"status": "error", "message": "target è richiesto"}
                return await self._handle_goto_target(target_name)
            else:
                return {"status": "error", "message": f"Comando sconosciuto: {command}"}
                
        except Exception as e:
            self.logger.error(f"Errore gestione messaggio remoto: {e}")
            return {"status": "error", "message": str(e)}
        
    async def _handle_goto_target(self, target_name: str) -> Dict[str, Any]:
        # Verifica se il target esiste
        if not self.targets:
            return {"status": "error", "message": "Nessun target definito"}
        if target_name not in self.targets:
            return {"status": "error", "message": f"Target sconosciuto: {target_name}"}
        
        target_pos = self.targets[target_name]
        target_pos = self.coords._transform_to_arena_coordinates(target_pos, arena_markers=self.vision.arena_markers)
        print(f"Moving to target '{target_name}' at position {target_pos}")
        return await self._handle_move_robot(target_pos[0], target_pos[1])

    async def _handle_move_square(self, x: int, y: int) -> Dict[str, Any]:
        """Muove il robot in uno dei target mappati su una griglia - per compatibilità con sistema qak basicrobot"""
        target = MapConfig.MAP_TO_TARGET.get((x, y))
        if not target:
            return {"status": "error", "message": f"Nessun target mappato per ({x}, {y})"}
        return await self._handle_goto_target(target)

    async def _handle_engage(self) -> Dict[str, Any]:
        """Attiva controllo remoto"""
        try:
            if not self.controller.control_enabled:
                await self.controller.start_continuous_control()
                self.pathfinding_mode = True  # Abilita pathfinding per controllo remoto
                
                # MODIFICATO: Avvia loop di controllo solo se non esiste già un task attivo
                # Il main loop potrebbe già avere avviato control_task, quindi non creiamo duplicati
                self.logger.info("🔄 Controllo automatico sarà gestito dal main loop")

            self.remote_control_enabled = True
            self.logger.info("🔗 Controllo remoto ATTIVATO")
            
            return {
                "status": "success",
                "message": "Controllo remoto attivato",
                "remote_control": True
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    async def _handle_disengage(self) -> Dict[str, Any]:
        """Disattiva controllo remoto"""
        try:
            await self.controller.stop()
            self.remote_control_enabled = False
            self.remote_movement_active = False  # NUOVO: reset movimento remoto
            self.remote_target_id = None  # NUOVO: reset ID movimento
            self.mouse_target = None
            self.last_target_sent = None
            
            # MODIFICATO: Non cancelliamo task che non abbiamo creato noi
            # Il main loop gestirà il proprio control_task
            self.logger.info("🔄 Controllo automatico fermato dal controller")
            
            self.logger.info("🔗 Controllo remoto DISATTIVATO")
            
            return {
                "status": "success", 
                "message": "Controllo remoto disattivato",
                "remote_control": False
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    async def _handle_move_robot(self, x: float, y: float) -> Dict[str, Any]:
        """Muove robot verso coordinate specificate"""
        try:
            if not self.remote_control_enabled:
                return {"status": "error", "message": "Controllo remoto non attivo"}
            
            # Genera ID univoco per il movimento
            import time
            import traceback
            movement_id = f"move_{int(time.time() * 1000)}"
            
            # Imposta target e inizia tracking movimento remoto
            self.mouse_target = (x, y)
            self.last_target_sent = None  # Forza aggiornamento
            self.remote_movement_active = True  # NUOVO: inizia tracking
            self.remote_target_id = movement_id  # NUOVO: salva ID movimento

            # se controllo non è abilitato, abilitalo
            if not self.controller.control_enabled:
                await self.controller.start_continuous_control()
                self.pathfinding_mode = True  # Abilita pathfinding per controllo remoto
                self.logger.info("🔄 Controllo remoto attivato")
            
            self.logger.info(f"🎯 Target remoto impostato: ({x:.1f}, {y:.1f}) [ID: {movement_id}]")
            
            return {
                "status": "success",
                "message": f"Target impostato a ({x}, {y})",
                "target": {"x": x, "y": y},
                "movement_id": movement_id  # NUOVO: restituisce ID movimento
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    async def _send_movement_done_notification(self):
        """Invia notifica di movimento completato ai client remoti"""
        try:
            if self.remote_target_id and self.websocket.clients:
                notification = {
                    "command": "moverobotdone",
                    "movement_id": self.remote_target_id,
                    "status": "success",
                    "message": "Movimento completato con successo"
                }
                
                await self.websocket.broadcast(notification)
                self.logger.info(f"✅ Notifica movimento completato inviata [ID: {self.remote_target_id}]")
                
                # Reset tracking movimento remoto
                self.remote_movement_active = False
                self.remote_target_id = None
                
                # NUOVO: Reset target anche qui per evitare loop
                if not self.follow_mouse_mode:
                    self.mouse_target = None
                    self.last_target_sent = None
                
        except Exception as e:
            self.logger.error(f"Errore invio notifica movimento completato: {e}")
    
    
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

                if vision_data["targets"]:
                    self.targets = vision_data["targets"]

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
                
                # NUOVO: Auto-avvia controllo se necessario
                if (self.controller.control_enabled and 
                    (not control_task or control_task.done())):
                    control_task = asyncio.create_task(self._control_loop())
                    self.logger.debug("🔄 Auto-avvio loop di controllo")
                
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
                
                # CALIBRAZIONE PROSPETTIVA
                elif key == ord('v'):
                    # Re-calibra prospettiva
                    self.logger.info("🔄 Re-calibrazione prospettiva...")
                    if self.vision.calibrate_perspective(frame):
                        self.logger.info("✅ Re-calibrazione completata!")
                        # Salva automaticamente
                        self.vision.save_perspective_calibration("perspective_calibration.npz")
                    else:
                        self.logger.warning("❌ Re-calibrazione fallita")
                elif key == ord('n'):
                    # Reset calibrazione prospettiva
                    self.vision.reset_perspective_calibration()
                    self.logger.info("🔄 Calibrazione prospettiva resettata")
                    
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
            self.logger.error(f"Errore nel main loop: {e}\n{traceback.format_exc()}")
        finally:
            # Cleanup
            if control_task:
                control_task.cancel()
            await self.controller.stop()
            await self.controller.communication.disconnect()
            # NUOVO: Ferma WebSocket server
            await self.websocket.stop()
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
                if vision_data.get("perspective_corrected", False):
                    # Frame corretto: calcola coordinate direttamente dai margini
                    margin = 50  # Stesso margine usato nella calibrazione
                    arena_width = self.vision.corrected_frame_size[0] - 2 * margin
                    arena_height = self.vision.corrected_frame_size[1] - 2 * margin
                    
                    obs_x = ((obs_center[0] - margin) / arena_width) * 100
                    obs_y = ((obs_center[1] - margin) / arena_height) * 100
                    
                    # Clamp e inverti Y
                    obs_x = max(0, min(100, obs_x))
                    obs_y = max(0, min(100, 100 - obs_y))
                    
                    obstacles_arena[obs_id] = (obs_x, obs_y)
                else:
                    # Frame originale: usa il sistema esistente
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
            if not self.pathfinding.has_valid_path():
                current_time = time.time()
                if current_time - self.last_stop_call > 1.0:  # Debounce di 1 secondo
                    await self.controller.stop()
                    self.last_stop_call = current_time
                return
        else:
            # NUOVO: Verifica se siamo già al target e non serve ricalcolare
            if hasattr(self, '_target_reached') and self._target_reached:
                # Verifica se il target è cambiato da quando l'abbiamo raggiunto
                if (hasattr(self, '_last_pathfinding_target') and 
                    target_pos == self._last_pathfinding_target):
                    # Stesso target già raggiunto, non fare nulla
                    return
                else:
                    # Nuovo target, resetta il flag
                    self._target_reached = False
            
            # AGGIORNA PATHFINDING SOLO SE NECESSARIO
            # Non ricalcolare il percorso ad ogni frame per evitare reset PID continui
            needs_recalc = False
            
            # Verifica se il target è cambiato significativamente
            if not hasattr(self, '_last_pathfinding_target'):
                needs_recalc = True
                self._last_pathfinding_target = target_pos
            else:
                dist_change = math.sqrt(
                    (target_pos[0] - self._last_pathfinding_target[0])**2 + 
                    (target_pos[1] - self._last_pathfinding_target[1])**2
                )
                if dist_change > 5.0:  # Solo se target cambia di più di 5 unità
                    needs_recalc = True
                    self._last_pathfinding_target = target_pos

            # Verifica se il robot si è spostato significativamente
            robot_pos = (vision_data["robot_x"], vision_data["robot_y"])
            if not hasattr(self, '_last_pathfinding_robot_pos'):
                needs_recalc = True
                self._last_pathfinding_robot_pos = robot_pos
            else:
                dist_robot_change = math.sqrt(
                    (robot_pos[0] - self._last_pathfinding_robot_pos[0])**2 + 
                    (robot_pos[1] - self._last_pathfinding_robot_pos[1])**2
                )
                if dist_robot_change > PathfindingConfig.PATHFINDING_RECALC_DISTANCE:
                    needs_recalc = True
                    self._last_pathfinding_robot_pos = robot_pos
            
            # Verifica se gli ostacoli sono cambiati
            if not hasattr(self, '_last_obstacles_hash'):
                self._last_obstacles_hash = str(sorted(obstacles_arena.keys()))
                needs_recalc = True
            else:
                current_obstacles_hash = str(sorted(obstacles_arena.keys()))
                if current_obstacles_hash != self._last_obstacles_hash:
                    self._last_obstacles_hash = current_obstacles_hash
                    needs_recalc = True
                    self.logger.info("🚧 Ostacoli cambiati, ricalcolo percorso")
            
            # Ricalcola percorso SOLO se necessario
            if needs_recalc:
                path_updated = self.pathfinding.update_path(
                    vision_data["robot_x"], vision_data["robot_y"],
                    target_pos[0], target_pos[1],
                    obstacles_arena
                )
                
                if path_updated:
                    self.logger.info(f"🗺️ Pathfinding: Nuovo percorso calcolato (target o ostacoli cambiati)")
                    # RESET TARGET SENT per forzare aggiornamento del prossimo waypoint
                    self.last_target_sent = None
                else:
                    self.logger.debug(f"🗺️ Pathfinding: Mantengo percorso esistente")
        
        # USA IL WAYPOINT DINAMICO
        # IMPORTANTE: Ora get_current_waypoint richiede la posizione del robot!
        current_waypoint = self.pathfinding.get_current_waypoint(
            vision_data["robot_x"], vision_data["robot_y"]
        )
        
        if current_waypoint:
            # Il waypoint dinamico si muove continuamente, quindi dobbiamo sempre aggiornare il target
            # Usa una tolleranza piccola per un controllo fluido
            waypoint_tolerance = 1.0  # Tolleranza ridotta per movimento fluido
            
            if (not self.last_target_sent or 
                abs(current_waypoint.x - self.last_target_sent[0]) > waypoint_tolerance or 
                abs(current_waypoint.y - self.last_target_sent[1]) > waypoint_tolerance):
                
                self.controller.set_target_position(current_waypoint.x, current_waypoint.y)
                self.last_target_sent = (current_waypoint.x, current_waypoint.y)
                
                # Log più dettagliato per il waypoint dinamico
                path_info = self.pathfinding.get_path_info()
                self.logger.debug(f"🎯 Waypoint dinamico aggiornato: ({current_waypoint.x:.1f}, {current_waypoint.y:.1f}) "
                                f"[Progress: {path_info['path_progress_percent']:.1f}%]")
            
            # Verifica se l'intero percorso è stato completato (non più singolo waypoint!)
            if self.pathfinding.is_path_complete(vision_data["robot_x"], vision_data["robot_y"]):
                # Percorso completato!
                path_info = self.pathfinding.get_path_info()
                if path_info["is_complete"]:
                    self.logger.info("🎉 Percorso pathfinding completato!")
                    await self.controller.stop()
                    
                    # IMPORTANTE: Pulisci il percorso per evitare loop infiniti
                    self.pathfinding.clear_path()
                    
                    # Invia notifica movimento completato se era remoto
                    if self.remote_movement_active and self.remote_target_id:
                        await self._send_movement_done_notification()
                    else:
                        # Reset target solo se non era un movimento remoto
                        if not self.follow_mouse_mode:
                            self.mouse_target = None
                            self.last_target_sent = None
                            # Reset anche variabili pathfinding
                            if hasattr(self, '_last_pathfinding_target'):
                                del self._last_pathfinding_target
                            
                            # NUOVO: Flag per indicare che il target è stato raggiunto
                            self._target_reached = True
                    
                    return  # Esci dalla funzione per evitare ulteriori elaborazioni
        else:
            # NESSUN PERCORSO VALIDO: Ferma il robot (con debounce)
            current_time = time.time()
            if current_time - self.last_stop_call > 1.0:  # Debounce di 1 secondo
                self.logger.warning("⚠️ Nessun percorso valido, robot fermo")
                await self.controller.stop()
                self.last_stop_call = current_time
    
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
