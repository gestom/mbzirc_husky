import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

z = -0.4
ztol = 0.4
scale = 0.05

fn = "/home/george/datasets/MBZIRC_pointcloud/tower2.pts"

img = np.zeros((4000, 4000, 3), dtype=np.uint8)
origin = (2000, 2000)

linen = 0

with open(fn, "r") as f:
    while True:
        line = f.readline()
        if not line:
            break
        try:
            height = float(line.split(" ")[2])
        except:
            continue
        if height > (z - ztol) and height < (z + ztol):
            x = int(float(line.split(" ")[0]) / scale)
            y = int(float(line.split(" ")[1]) / scale)
            img[x + origin[0]][y + origin[1]] = [255, 255, 255]


        linen += 1
        if linen % 100000 == 0:
            print(linen)

img[origin[0]][origin[1]] = [255,255,255]

img = Image.fromarray(img, 'RGB')
img.save('arena.png')
