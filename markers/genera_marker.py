# genera_marker.py
import cv2
import cv2.aruco as aruco

def main():
    # Dizionario ArUco corretto
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    # Generiamo i marker
    for id in range(4, 8):
        img = aruco.generateImageMarker(aruco_dict, id, 300)  # 300 px per lato
        filename = f"aruco_{id}.png"
        cv2.imwrite(filename, img)
        print(f"Salvato marker ID {id} in {filename}")

if __name__ == "__main__":
    main()
