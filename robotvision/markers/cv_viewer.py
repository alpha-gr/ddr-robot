import cv2
import cv2.aruco as aruco
import numpy as np
import math

def calculate_marker_center(corners):
    """Calcola il centro di un marker dai suoi corner"""
    corner = corners[0]  # corners è una lista, prendiamo il primo elemento
    center_x = np.mean(corner[:, 0])
    center_y = np.mean(corner[:, 1])
    return int(center_x), int(center_y)

def calculate_marker_orientation(corners):
    """Calcola l'orientamento del marker in radianti"""
    corner = corners[0]
    # Prendiamo il vettore dal primo al secondo corner (lato superiore)
    dx = corner[1, 0] - corner[0, 0]
    dy = corner[1, 1] - corner[0, 1]
    angle = math.atan2(dy, dx)
    return angle

def draw_coordinate_system(img, center, angle, scale=50):
    """Disegna un sistema di coordinate al centro del marker"""
    cx, cy = center
    
    # Calcola le coordinate degli assi
    end_x = int(cx + scale * math.cos(angle))
    end_y = int(cy + scale * math.sin(angle))
    
    end_x_perp = int(cx + scale * math.cos(angle + math.pi/2))
    end_y_perp = int(cy + scale * math.sin(angle + math.pi/2))
    
    # Disegna asse X (rosso)
    cv2.arrowedLine(img, (cx, cy), (end_x, end_y), (0, 0, 255), 3)
    # Disegna asse Y (verde)
    cv2.arrowedLine(img, (cx, cy), (end_x_perp, end_y_perp), (0, 255, 0), 3)

def transform_coordinates(robot_center, arena_corners):
    """Trasforma le coordinate del robot nel sistema di riferimento dell'arena"""
    if len(arena_corners) < 4:
        return None, None
    
    # Assumiamo che i marker siano ordinati: 1=top-left, 2=top-right, 3=bottom-right, 4=bottom-left
    # Se non sono in questo ordine, potremmo dover riordinare
    
    # Per ora, usiamo una trasformazione semplice basata sui marker visibili
    # Calcoliamo il bounding rectangle dei marker dell'arena
    all_x = [pos[0] for pos in arena_corners.values()]
    all_y = [pos[1] for pos in arena_corners.values()]
    
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    
    # Coordinate normalizzate del robot nell'arena (0-1)
    norm_x = (robot_center[0] - min_x) / (max_x - min_x) if max_x != min_x else 0
    norm_y = (robot_center[1] - min_y) / (max_y - min_y) if max_y != min_y else 0
    
    # Convertiamo in coordinate dell'arena (ad esempio 0-100)
    arena_x = norm_x * 100
    arena_y = norm_y * 100
    
    return arena_x, arena_y

def main():
    # Apri la webcam
    cap = cv2.VideoCapture(0)  # usa 1 se hai più webcam

    if not cap.isOpened():
        print("Errore: impossibile aprire la webcam")
        return

    # Dizionario ArUco (stesso usato per generare i marker)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Converti in grigio
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Rileva marker
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)

        # Variabili per tracciare i marker dell'arena e del robot
        arena_markers = {}  # dizionario per marker 1,2,3,4
        robot_marker = None  # per marker 0
        robot_corners = None
        robot_angle = None

        # Disegna i marker trovati e classifica
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                center = calculate_marker_center([corners[i]])
                
                if marker_id in [1, 2, 3, 4]:
                    # Marker dell'arena
                    arena_markers[marker_id] = center
                    cv2.circle(frame, center, 5, (255, 0, 0), -1)  # Punto blu per arena
                    cv2.putText(frame, f"Arena {marker_id}", (center[0]+10, center[1]-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                elif marker_id == 0:
                    # Marker del robot
                    robot_marker = center
                    robot_corners = corners[i]
                    robot_angle = calculate_marker_orientation([corners[i]])
                    
                    # Disegna sistema di coordinate del robot
                    draw_coordinate_system(frame, center, robot_angle)
                    cv2.circle(frame, center, 8, (0, 255, 255), -1)  # Punto giallo per robot
                    cv2.putText(frame, "Robot", (center[0]+10, center[1]+20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Disegna il perimetro dell'arena se abbiamo abbastanza marker
        if len(arena_markers) >= 2:
            # Ordina i marker dell'arena per disegnare il perimetro
            sorted_markers = sorted(arena_markers.items())
            points = [pos for _, pos in sorted_markers]
            
            # Disegna linee tra i marker dell'arena
            for i in range(len(points)):
                start_point = points[i]
                end_point = points[(i + 1) % len(points)]
                cv2.line(frame, start_point, end_point, (255, 0, 0), 2)

        # Calcola e mostra le coordinate del robot
        info_text = []
        if robot_marker is not None:
            # Coordinate del robot in pixel
            robot_x, robot_y = robot_marker
            # Angolo in gradi
            robot_theta_deg = math.degrees(robot_angle) if robot_angle is not None else 0
            
            info_text.append(f"Robot Pixel: ({robot_x}, {robot_y})")
            info_text.append(f"Robot Theta: {robot_theta_deg:.1f}°")
            
            # Se abbiamo marker dell'arena, calcola coordinate trasformate
            if len(arena_markers) >= 2:
                arena_x, arena_y = transform_coordinates(robot_marker, arena_markers)
                if arena_x is not None:
                    info_text.append(f"Arena Coords: ({arena_x:.1f}, {arena_y:.1f})")
        
        # Mostra informazioni sui marker dell'arena
        info_text.append(f"Arena markers: {len(arena_markers)}/4")
        for marker_id in sorted(arena_markers.keys()):
            pos = arena_markers[marker_id]
            info_text.append(f"  Marker {marker_id}: ({pos[0]}, {pos[1]})")

        # Disegna le informazioni sullo schermo
        y_offset = 30
        for text in info_text:
            cv2.putText(frame, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.6, (255, 255, 255), 2)
            y_offset += 25

        # Mostra feed
        cv2.imshow("DDR Robot Arena Tracking", frame)

        # Esci con 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


if __name__ == "__main__":
    main()
