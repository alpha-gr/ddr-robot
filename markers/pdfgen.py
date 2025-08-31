from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import cm

def images_to_pdf(image_paths, output_pdf):
    c = canvas.Canvas(output_pdf, pagesize=A4)
    page_width, page_height = A4

    # Posizioni su una griglia 2x2
    positions = [
        (2*cm, page_height/2 + 2*cm),
        (page_width/2 + 2*cm, page_height/2 + 2*cm),
        (2*cm, 2*cm),
        (page_width/2 + 2*cm, 2*cm)
    ]

    for pos, img in zip(positions, image_paths):
        x, y = pos
        c.drawImage(img, x, y, width=8*cm, height=8*cm)  # lato 8 cm

    c.save()


if __name__ == "__main__":
    # Inserisci qui i path alle 4 immagini generate
    # image_files = ["aruco_0.png", "aruco_1.png", "aruco_2.png", "aruco_3.png"]
    image_files = ["aruco_4.png", "aruco_5.png", "aruco_6.png", "aruco_7.png"]
    output_pdf = "aruco_markers.pdf"

    images_to_pdf(image_files, output_pdf)
    print(f"PDF generato: {output_pdf}")
