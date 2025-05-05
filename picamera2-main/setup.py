import cv2
from picamera2 import Picamera2
import time

def main():
    # Inicialización y configuración de la cámara
    picam2 = Picamera2()
    config = picam2.create_still_configuration()
    picam2.configure(config)
    picam2.start()
    
    # Pequeña espera para que la cámara se estabilice
    time.sleep(2)
    
    # Capturamos una imagen y la obtenemos como un array de numpy
    imagen_rgb = picam2.capture_array()
    
    # Convertimos la imagen de RGB a HSV usando OpenCV
    imagen_hsv = cv2.cvtColor(imagen_rgb, cv2.COLOR_RGB2HSV)
    
    # Mostramos las imágenes en ventanas separadas
    cv2.imshow("Imagen RGB", imagen_rgb)
    cv2.imshow("Imagen HSV", imagen_hsv)
    
    print("Presiona cualquier tecla en la ventana de la imagen para cerrar...")
    cv2.waitKey(0)
    
    # Cerramos las ventanas y detenemos la cámara
    cv2.destroyAllWindows()
    picam2.stop()

if __name__ == "__main__":
    main()
