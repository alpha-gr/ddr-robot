from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import cm

def images_to_pdf(image_paths, output_pdf):
    c = canvas.Canvas(output_pdf, pagesize=A4)
    page_width, page_height = A4

    # Posizioni su una griglia 2x2 centrata
    image_size = 8*cm
    # Calcola margini per centrare le immagini
    margin_x = (page_width - 2 * image_size) / 3  # 3 spazi: sx, centro, dx
    margin_y = (page_height - 2 * image_size) / 3  # 3 spazi: su, centro, giù
    
    positions = [
        (margin_x, page_height - margin_y - image_size),  # in alto a sinistra
        (margin_x * 2 + image_size, page_height - margin_y - image_size),  # in alto a destra
        (margin_x, margin_y),  # in basso a sinistra
        (margin_x * 2 + image_size, margin_y)  # in basso a destra
    ]

    for pos, img in zip(positions, image_paths):
        x, y = pos
        c.drawImage(img, x, y, width=8*cm, height=8*cm)  # lato 8 cm

    c.save()


if __name__ == "__main__":
    # Inserisci qui i path alle 4 immagini generate
    # image_files = ["aruco_0.png", "aruco_1.png", "aruco_2.png", "aruco_3.png"]
    image_files = ["aruco_4.png", "aruco_5.png", "aruco_6.png", "aruco_7.png"]
    #image_files = ["aruco_8.png", "aruco_9.png", "aruco_10.png", "aruco_11.png"]
    #image_files = ["aruco_12.png", "aruco_13.png", "aruco_14.png", "aruco_15.png"]
    output_pdf = "aruco_markers4-7.pdf"

    images_to_pdf(image_files, output_pdf)
    print(f"PDF generato: {output_pdf}")
