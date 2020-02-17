#!/usr/bin/env python
import cv2
import time
import numpy as np
import math

filename = "../maps/tennis-left-2/map.yaml"
maxObservableDistance = 4.0

rotAmount = 0.02
mvAmount = 1
resizeAmount = 0.5
nogoboundary = 6

#don't need to adjust vars below
foldername = "/".join(filename.split("/")[:-1])
#to account for diagonal distance of radius (1 over root 2)
diagMaxObservableDistance = maxObservableDistance * (1/(2.0**0.5))
imagefn = ""
resolution = 1
origin = [0, 0]
boxX = 0
boxY = 0
boxW = 30
boxH = 20
boxR = 0
running = True

def worldToDraw(x, y):
    #real map size
    mapSize = abs(origin[0]) * 2
    #real map size in pxs
    mapSize *= (1/resolution)

    x *= (1/resolution)
    y *= (1/resolution)
    y *= -1

    x += (mapSize / 2)
    y += (mapSize / 2)

    return int(x), int(y)

if __name__ == "__main__":

    mapdata = []
    with open(filename, "r") as f:
        mapdata = f.read()
    mapdata = mapdata.split("\n")

    for line in mapdata:
        line = filter(None, line).replace(" ", "").split(":")
        if "image" in line[0]:
            imagefn = line[1]
        elif "resolution" in line[0]:
            resolution = float(line[1])
        elif "origin" in line[0]:
            line[1] = line[1].replace("[", "")
            line[1] = line[1].replace("]", "")
            origin[0] = float(line[1].split(",")[0])
            origin[1] = float(line[1].split(",")[1])

    fullfn = foldername + "/" + imagefn
    print("Opening: " + fullfn)
    rawimg = cv2.imread(fullfn)
    cv2.namedWindow("Map")
    
    boxX = 0
    boxY = 0

    while running:

        k = cv2.waitKey(33)
        
        if k == 27:
            #esc
            running = False
        elif k == 97: 
            #a
            boxX -= mvAmount
        elif k == 100:
            #d
            boxX += mvAmount
        elif k == 119:
            #w
            boxY -= mvAmount
        elif k == 115:
            #s
            boxY += mvAmount
        elif k == 113:
            #q
            boxR -= rotAmount
        elif k == 101:
            #e
            boxR += rotAmount
        elif k == 114:
            #r
            boxW += resizeAmount
        elif k == 116:
            #t
            boxW -= resizeAmount
        elif k == 102:
            #f
            boxH += resizeAmount
        elif k == 103:
            #g
            boxH -= resizeAmount

        img = rawimg.copy()

        tl = (boxX - ((boxW/2) * math.cos(boxR)) + ((boxH/2) * math.sin(boxR)), boxY - ((boxH/2) * math.cos(boxR)) - ((boxW/2) * math.sin(boxR)))
        tr = (boxX + ((boxW/2) * math.cos(boxR)) + ((boxH/2) * math.sin(boxR)), boxY - ((boxH/2) * math.cos(boxR)) + ((boxW/2) * math.sin(boxR)))
        bl = (boxX - ((boxW/2) * math.cos(boxR)) - ((boxH/2) * math.sin(boxR)), boxY + ((boxH/2) * math.cos(boxR)) - ((boxW/2) * math.sin(boxR)))
        br = (boxX + ((boxW/2) * math.cos(boxR)) - ((boxH/2) * math.sin(boxR)), boxY + ((boxH/2) * math.cos(boxR)) + ((boxW/2) * math.sin(boxR)))
      
        tl = worldToDraw(tl[0], tl[1])
        tr = worldToDraw(tr[0], tr[1])
        bl = worldToDraw(bl[0], bl[1])
        br = worldToDraw(br[0], br[1])

        lineWidth = 5
        colour = (0, 255, 255)
        img = cv2.line(img, tl, tr, colour, lineWidth)
        img = cv2.line(img, tr, br, colour, lineWidth)
        img = cv2.line(img, br, bl, colour, lineWidth)
        img = cv2.line(img, bl, tl, colour, lineWidth)
        
        nWaypointsX = int(((boxW-nogoboundary)/(2*diagMaxObservableDistance)) + 1) 
        nWaypointsY = int(((boxH-nogoboundary)/(2*diagMaxObservableDistance)) + 1) 

        xOffset = (((2*diagMaxObservableDistance) * nWaypointsX) - boxW) / 2
        yOffset = (((2*diagMaxObservableDistance) * nWaypointsY) - boxH) / 2

        colour = (255, 0, 255)
        lineWidth = 20

        wp = []
        for i in range(nWaypointsY):
            for j in range(nWaypointsX):
                px = (2 * diagMaxObservableDistance * j) + (diagMaxObservableDistance - xOffset)
                py = (2 * diagMaxObservableDistance * i) + (diagMaxObservableDistance - yOffset)
                px -= boxW/2
                py -= boxH/2
                rx = (px * math.cos(boxR)) - (py * math.sin(boxR))
                ry = (px * math.sin(boxR)) + (py * math.cos(boxR))
                rx += boxX
                ry += boxY
                
                wp.append((rx, ry))
                pos = worldToDraw(rx, ry)
                img = cv2.line(img, pos, pos, colour, lineWidth)
                img = cv2.circle(img, pos, int(maxObservableDistance * (1/resolution)), (0, 125, 0), thickness=2, lineType=8, shift=0)

        cv2.imshow("Map", img)

        if k == 13 or k == 32:
            print("Saving map")
            folder = '/'.join(filename.split("/")[:-1])
            newFilename = filename.split("/")[-1].split(".")[0]
            cv2.imwrite(folder + "/" + newFilename + "-annotation.png", img)

            img2 = np.ones(img.shape)
            img2 = img2 * 255
            lineWidth = 3
            colour = (0, 0, 0)
        
            img2 = cv2.line(img2, tl, tr, colour, lineWidth)
            img2 = cv2.line(img2, tr, br, colour, lineWidth)
            img2 = cv2.line(img2, br, bl, colour, lineWidth)
            img2 = cv2.line(img2, bl, tl, colour, lineWidth)

            cv2.imwrite(folder + "/" + newFilename + "-nogo.png", img2) 

            yaml = ""
            with open(folder + "/" + newFilename + ".yaml", "r") as f:
                yaml = f.read()
            yaml = yaml.replace(newFilename + ".pgm", newFilename + "-nogo.png")
            with open(folder + "/" + newFilename + "-nogo.yaml", "w") as f:
                f.write(yaml)

            with open(folder + "/" + newFilename + "-waypoints.txt", "w") as f:
                for i in wp:
                    f.write(str(i[0]) + " " + str(i[1]) + "\n")

    cv2.destroyAllWindows()
