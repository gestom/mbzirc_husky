#!/usr/bin/env python
import cv2
import time
import numpy as np
import math

filename = "../maps/romance.pgm"
mapRes = 0.1
x = 2000
y = 2000
w = 60
h = 40
r = 0
maxObservableDistance = 4

rotAmount = 0.02
mvAmount = 10

running = True
mapmul = 1/mapRes
w = w * mapmul
h = h * mapmul

if __name__ == "__main__":

    rawimg = cv2.imread(filename)
    cv2.namedWindow("Map")

    while running:

        k = cv2.waitKey(33)
        print("Key: " + str(k))

        if k == 27:
            #esc
            running = False
        elif k == 97: 
            #a
            x -= mvAmount
        elif k == 100:
            #d
            x += mvAmount
        elif k == 119:
            #w
            y -= mvAmount
        elif k == 115:
            #s
            y += mvAmount
        elif k == 113:
            #q
            r -= rotAmount
        elif k == 101:
            #e
            r += rotAmount
        elif k == 114:
            #r
            w += 1
        elif k == 116:
            #t
            w -= 1
        elif k == 102:
            #f
            h += 1
        elif k == 103:
            #g
            h -= 1

        img = rawimg.copy()

        width = 5
        colour = (0, 255, 255)
        tl = (int(x - (w * math.cos(r)) + (h * math.sin(r))), int(y - (h * math.cos(r)) - (w * math.sin(r))))
        tr = (int(x + (w * math.cos(r)) + (h * math.sin(r))), int(y - (h * math.cos(r)) + (w * math.sin(r))))
        bl = (int(x - (w * math.cos(r)) - (h * math.sin(r))), int(y + (h * math.cos(r)) - (w * math.sin(r))))
        br = (int(x + (w * math.cos(r)) - (h * math.sin(r))), int(y + (h * math.cos(r)) + (w * math.sin(r))))

        img = cv2.line(img, tl, tr, colour, width)
        img = cv2.line(img, tr, br, colour, width)
        img = cv2.line(img, br, bl, colour, width)
        img = cv2.line(img, bl, tl, colour, width)

        cv2.imshow("Map", img)

        if k == 13 or k == 32:
            print("Saving map")
            folder = '/'.join(filename.split("/")[:-1])
            newFilename = filename.split("/")[-1].split(".")[0]
            cv2.imwrite(folder + "/" + newFilename + "-annotation.png", img)

            img2 = np.ones(img.shape)
            img2 = img2 * 255
            width = 3
            colour = (0, 0, 0)
        
            img2 = cv2.line(img2, tl, tr, colour, width)
            img2 = cv2.line(img2, tr, br, colour, width)
            img2 = cv2.line(img2, br, bl, colour, width)
            img2 = cv2.line(img2, bl, tl, colour, width)
            #img2 = cv2.floodFill(img2, None, (x, y), colour)

            cv2.imwrite(folder + "/" + newFilename + "-nogo.png", img2) 

            wp = []
            wp.append((x, y))

            M
             


            with open(folder + "/" + newFilename + "-waypoints.txt", "w") as f:
                for i in wp:
                    f.write(str(i[0]) + " " + str(i[1]) + "\n")


    cv2.destroyAllWindows()
