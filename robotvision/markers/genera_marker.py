# genera_marker.py
import cv2
import cv2.aruco as aruco

def main():
    # Dizionario ArUco corretto
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    # Generiamo i marker per il progetto DDR robot
    # Marker 0: Robot
    # Marker 1,2,3,4: Corner dell'arena
    for id in [0, 1, 2, 3, 4]:
        img = aruco.generateImageMarker(aruco_dict, id, 300)  # 300 px per lato
        filename = f"aruco_{id}.png"
        cv2.imwrite(filename, img)
        if id == 0:
            print(f"Salvato marker ROBOT (ID {id}) in {filename}")
        else:
            print(f"Salvato marker ARENA (ID {id}) in {filename}")

if __name__ == "__main__":
    main()
