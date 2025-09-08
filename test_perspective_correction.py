#!/usr/bin/env python3
"""
Test della correzione prospettica
Mostra il confronto tra frame originale e frame corretto
"""

import cv2
import numpy as np
import logging
from robot.vision.visionsystem import VisionSystem
from robot.vision.coords import Coordinates
from config import VisionConfig

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

def main():
    print("=== Test Correzione Prospettica ===")
    print("Istruzioni:")
    print("- Posiziona tutti e 4 i marker dell'arena (ID 1,2,3,4) nel campo visivo")
    print("- Premi 'c' per calibrare la prospettiva")
    print("- Premi 's' per salvare la calibrazione")
    print("- Premi 'l' per caricare calibrazione salvata")
    print("- Premi 'r' per resettare calibrazione")
    print("- Premi 'q' per uscire")
    print("")
    
    # Inizializza sistema
    coords = Coordinates()
    vision = VisionSystem(coords)
    
    if not vision.initialize_camera():
        logger.error("Impossibile inizializzare camera")
        return
    
    calibration_file = "perspective_calibration.npz"
    
    # Prova a caricare calibrazione esistente
    if vision.load_perspective_calibration(calibration_file):
        print("✅ Calibrazione caricata automaticamente!")
    
    try:
        while True:
            ret, frame = vision.cap.read()
            if not ret:
                logger.warning("Impossibile leggere frame")
                continue
            
            # Processa frame
            vision_data = vision.process_frame(frame)
            
            # Crea display
            display_frame = frame.copy()
            
            # Info calibrazione
            perspective_info = vision.get_perspective_info()
            status_color = (0, 255, 0) if perspective_info["calibrated"] else (0, 0, 255)
            status_text = "CALIBRATO" if perspective_info["calibrated"] else "NON CALIBRATO"
            
            cv2.putText(display_frame, f"Prospettiva: {status_text}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # Mostra marker rilevati
            if vision_data["arena_valid"]:
                arena_count = len(vision_data["arena_markers"])
                cv2.putText(display_frame, f"Arena markers: {arena_count}/4", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            if vision_data["robot_found"]:
                cv2.putText(display_frame, f"Robot: ({vision_data['robot_x']:.1f}, {vision_data['robot_y']:.1f})", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Mostra frame originale
            cv2.imshow("Frame Originale", display_frame)
            
            # Mostra frame corretto se disponibile
            if vision_data["perspective_corrected"] and vision_data["corrected_frame"] is not None:
                corrected_display = vision_data["corrected_frame"].copy()
                
                # Disegna griglia sul frame corretto per riferimento
                h, w = corrected_display.shape[:2]
                
                # Griglia 10x10
                for i in range(1, 10):
                    x = int(w * i / 10)
                    y = int(h * i / 10)
                    cv2.line(corrected_display, (x, 0), (x, h), (100, 100, 100), 1)
                    cv2.line(corrected_display, (0, y), (w, y), (100, 100, 100), 1)
                
                # Bordi arena
                margin = 50
                cv2.rectangle(corrected_display, 
                            (margin, margin), 
                            (w - margin, h - margin), 
                            (0, 255, 0), 2)
                
                cv2.putText(corrected_display, "FRAME CORRETTO", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow("Frame Corretto", corrected_display)
            
            # Gestione input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('c'):
                print("🔄 Calibrando prospettiva...")
                if vision.calibrate_perspective(frame):
                    print("✅ Calibrazione completata!")
                else:
                    print("❌ Calibrazione fallita - assicurati che tutti e 4 i marker arena siano visibili")
            elif key == ord('s'):
                if vision.save_perspective_calibration(calibration_file):
                    print("💾 Calibrazione salvata!")
                else:
                    print("❌ Impossibile salvare - calibra prima")
            elif key == ord('l'):
                if vision.load_perspective_calibration(calibration_file):
                    print("📁 Calibrazione caricata!")
                else:
                    print("❌ Impossibile caricare calibrazione")
            elif key == ord('r'):
                vision.reset_perspective_calibration()
                print("🔄 Calibrazione resettata")
    
    except KeyboardInterrupt:
        print("\nInterrotto dall'utente")
    
    finally:
        vision.release()
        cv2.destroyAllWindows()
        print("🏁 Test completato")

if __name__ == "__main__":
    main()
