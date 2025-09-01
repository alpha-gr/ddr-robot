import cv2
import cv2.aruco as aruco
import numpy as np
import math

def calculate_marker_orientation(corners):
    """Calcola l'orientamento del marker in radianti"""
    corner = corners[0]  # Prende i 4 corner del marker
    # Calcola il vettore dal primo al secondo corner (lato superiore del marker)
    dx = corner[1, 0] - corner[0, 0]
    dy = corner[1, 1] - corner[0, 1]
    angle = math.atan2(dy, dx)
    return angle

def draw_robot_coordinate_system(img, center, angle, scale=40):
    """Disegna il sistema di coordinate del robot"""
    cx, cy = center
    
    # Calcola le coordinate degli assi
    end_x = int(cx + scale * math.cos(angle))
    end_y = int(cy + scale * math.sin(angle))
    
    end_x_perp = int(cx + scale * math.cos(angle + math.pi/2))
    end_y_perp = int(cy + scale * math.sin(angle + math.pi/2))
    
    # Disegna asse X (rosso) - direzione "avanti" del robot
    cv2.arrowedLine(img, (cx, cy), (end_x, end_y), (0, 0, 255), 3)
    # Disegna asse Y (verde) - direzione "sinistra" del robot
    cv2.arrowedLine(img, (cx, cy), (end_x_perp, end_y_perp), (0, 255, 0), 2)

def transform_robot_to_arena_coordinates(robot_center, arena_markers):
    """Trasforma le coordinate del robot nel sistema di riferimento dell'arena"""
    if len(arena_markers) < 2:
        return None, None
    
    # Ottieni tutti i punti dell'arena
    arena_points = list(arena_markers.values())
    
    # Calcola il bounding box dell'arena
    all_x = [p[0] for p in arena_points]
    all_y = [p[1] for p in arena_points]
    
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    
    # Evita divisione per zero
    if max_x == min_x or max_y == min_y:
        return None, None
    
    # Coordinate normalizzate del robot nell'arena (0-1)
    norm_x = (robot_center[0] - min_x) / (max_x - min_x)
    norm_y = (robot_center[1] - min_y) / (max_y - min_y)
    
    # Scala a coordinate 0-100 per maggiore leggibilità
    arena_x = norm_x * 100
    arena_y = norm_y * 100
    
    return arena_x, arena_y

def order_points_clockwise(points):
    """Ordina i punti in senso orario partendo dal punto più in alto a sinistra"""
    # Trova il centroide
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    
    # Ordina per angolo rispetto al centroide
    def angle_from_center(point):
        return math.atan2(point[1] - cy, point[0] - cx)
    
    return sorted(points, key=angle_from_center)

def main():
    # Apri la webcam con risoluzione alta
    print("accedo alla webcam")
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print("Errore: impossibile aprire la webcam")
        return
    print("webcam aperta")

    # Imposta risoluzione alta della webcam
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    # Verifica risoluzione effettiva
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Risoluzione webcam: {actual_width}x{actual_height}")

    # Dizionario ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    print("Test multi-marker - Premi 'q' per uscire")
    print("Marker 0: Robot (giallo)")
    print("Marker 1,2,3,4: Arena (colorati)")

    # Crea finestra ridimensionabile con dimensioni proporzionate
    window_name = "DDR Robot Tracker - HD"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1600, 900)  # Dimensione HD iniziale
    
    # Aggiungi controlli sulla finestra
    print("Controlli:")
    print("  'q' - Esci")
    print("  'f' - Fullscreen/Finestra")
    print("  's' - Screenshot")
    print("  Trascina i bordi per ridimensionare")
    
    is_fullscreen = False
    
    # Per calcolare FPS
    import time
    fps_counter = 0
    fps_start_time = time.time()
    current_fps = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Calcola FPS
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= 1.0:  # Aggiorna ogni secondo
            current_fps = fps_counter / (current_time - fps_start_time)
            fps_counter = 0
            fps_start_time = current_time

        # Converti in grigio
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Rileva marker
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)

        # Dizionari per organizzare i marker
        arena_markers = {}  # Per marker 1,2,3,4
        robot_center = None
        robot_theta = None
        robot_arena_x = None
        robot_arena_y = None
        
        # Se trova marker
        if ids is not None:
            # Disegna i bordi dei marker rilevati
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                # Calcola il centro del marker - METODO ORIGINALE CHE FUNZIONAVA
                corner = corners[i][0]  # Prende i 4 corner del marker
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))
                center = (center_x, center_y)
                
                if marker_id == 0:
                    # Robot marker - Giallo
                    robot_center = center
                    
                    # Calcola l'orientamento del robot
                    robot_theta_rad = calculate_marker_orientation(corners[i])
                    robot_theta = math.degrees(robot_theta_rad)
                    
                    # Disegna il sistema di coordinate del robot
                    draw_robot_coordinate_system(frame, center, robot_theta_rad)
                    
                    # Cerchio giallo per il robot
                    cv2.circle(frame, center, 12, (0, 255, 255), -1)
                    cv2.putText(frame, f"R", 
                               (center[0] + 18, center[1] + 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                elif marker_id in [1, 2, 3, 4]:
                    # Arena markers - Colori diversi
                    arena_markers[marker_id] = center
                    colors = {
                        1: (255, 0, 0),    # Blu
                        2: (0, 255, 0),    # Verde  
                        3: (0, 0, 255),    # Rosso
                        4: (255, 0, 255)   # Magenta
                    }
                    color = colors[marker_id]
                    
                    cv2.circle(frame, center, 8, color, -1)
                    cv2.putText(frame, f"{marker_id}", 
                               (center[0] + 12, center[1] + 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 2)

        # Calcola le coordinate del robot nell'arena
        if robot_center and len(arena_markers) >= 2:
            robot_arena_x, robot_arena_y = transform_robot_to_arena_coordinates(robot_center, arena_markers)

        # Disegna il rettangolo dell'arena se abbiamo almeno 3 marker
        if len(arena_markers) >= 3:
            # Ottieni i punti dell'arena
            arena_points = list(arena_markers.values())
            
            # Ordinali per formare un rettangolo corretto
            ordered_points = order_points_clockwise(arena_points)
            
            # Disegna le linee del rettangolo
            for i in range(len(ordered_points)):
                start_point = ordered_points[i]
                end_point = ordered_points[(i + 1) % len(ordered_points)]
                
                # Linea spessa bianca per il perimetro
                cv2.line(frame, start_point, end_point, (255, 255, 255), 4)
                
                # Aggiungi numero al centro della linea per debug (più piccolo)
                mid_x = (start_point[0] + end_point[0]) // 2
                mid_y = (start_point[1] + end_point[1]) // 2
                cv2.putText(frame, str(i+1), (mid_x-5, mid_y-5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1)
            
            # Se abbiamo tutti e 4 i marker, riempi leggermente l'area
            if len(arena_markers) == 4:
                overlay = frame.copy()
                points_array = np.array(ordered_points, np.int32)
                cv2.fillPoly(overlay, [points_array], (100, 100, 100))
                cv2.addWeighted(overlay, 0.2, frame, 0.8, 0, frame)
        
        # Informazioni sullo schermo con testo compatto
        frame_height, frame_width = frame.shape[:2]
        
        # Testo più piccolo e posizionato in alto a destra per non coprire l'arena
        info_lines = [
            f"FPS: {current_fps:.1f}",
            f"Arena: {len(arena_markers)}/4",
            f"Robot: {'OK' if robot_center else 'NO'}",
            f"IDs: {sorted(arena_markers.keys()) if arena_markers else '[]'}"
        ]
        
        # Aggiungi informazioni del robot se disponibili
        if robot_center:
            info_lines.append(f"Pixel: ({robot_center[0]}, {robot_center[1]})")
            if robot_theta is not None:
                info_lines.append(f"theta: {robot_theta:.0f}")
            if robot_arena_x is not None and robot_arena_y is not None:
                info_lines.append(f"Arena: ({robot_arena_x:.0f}, {robot_arena_y:.0f})")
            else:
                info_lines.append("Arena: <non disponibile>")
        
        # Posiziona il pannello info in alto a destra, testo piccolo
        info_panel_width = 250
        info_panel_height = len(info_lines) * 30 + 10
        start_x = frame_width - info_panel_width - 10
        start_y = 10
        
        # Sfondo semi-trasparente compatto
        overlay = frame.copy()
        cv2.rectangle(overlay, (start_x, start_y), 
                     (start_x + info_panel_width, start_y + info_panel_height), 
                     (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        # Testo piccolo e compatto
        y_offset = start_y + 30
        for line in info_lines:
            cv2.putText(frame, line, (start_x + 5, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            y_offset += 30

        # Mostra il frame nella finestra ridimensionabile
        cv2.imshow(window_name, frame)

        # Gestione input avanzata
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('f'):
            # Toggle fullscreen
            is_fullscreen = not is_fullscreen
            if is_fullscreen:
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                print("Modalità fullscreen attivata")
            else:
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(window_name, 1200, 800)
                print("Modalità finestra attivata")
        elif key == ord('s'):
            # Screenshot
            from datetime import datetime
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"robot_tracking_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Screenshot salvato: {filename}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
