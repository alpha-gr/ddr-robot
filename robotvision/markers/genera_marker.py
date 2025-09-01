# genera_marker.py
import cv2
import cv2.aruco as aruco

def main():
    # Dizionario ArUco corretto
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    for id in range(8, 16):
        img = aruco.generateImageMarker(aruco_dict, id, 300)  # 300 px per lato
        filename = f"aruco_{id}.png"
        cv2.imwrite(filename, img)
        # aggiungo quiet zone bianca di 50 px
        #img_with_border = cv2.copyMakeBorder(img, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=[255, 255, 255])
        # cv2.imwrite(filename, img_with_border)
        print(f"Marker salvato: {filename}")

if __name__ == "__main__":
    main()
