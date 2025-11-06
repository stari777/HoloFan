from PIL import Image
import numpy as np
import math

size_xy = 64

img = Image.open("rgb_image.png").convert("RGB") # Bild laden
img = img.resize((size_xy, size_xy)) # Bild komprimieren auf LED-Matrix
arr = np.array(img, dtype=np.uint8) # In Array umwandeln

# Koordinaten und RGB-Werte auslesen
pixels = []
for y in range(arr.shape[0]):   # Höhe
    for x in range(arr.shape[1]):  # Breite
        r, g, b = arr[y, x]
        pixels.append((x, y, r, g, b))

def file_write(pixel):
    f = open("test_rgb", "w")
    f.write(pixel)

def CordToPol(pixel_list):
    with open("CoordToPolar", "w") as f:
        for (x,y,r,g,b) in pixel_list:
            r = math.sqrt(x**2 + y**2)
            phi = math.atan2(y, x) * (180/math.pi) # Phi in Grad
            f.write(f"x:{x} y:{y} | r:{r:0.02f} Phi:{phi:0.02f}\n")  


def main():
    #file_write(str(pixels[::])) # :: alle Stellen ausgegeben
    CordToPol(pixels)

if __name__ == "__main__":
    main()