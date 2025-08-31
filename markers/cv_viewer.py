import cv2
import cv2.aruco as aruco

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

        # Disegna i marker trovati
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, corner in enumerate(corners):
                c = corner[0]  # 4 angoli del marker
                print(f"ID {ids[i][0]} -> angoli: {c}")

        # Mostra feed
        cv2.imshow("Riconoscimento ArUco", frame)

        # Esci con 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
