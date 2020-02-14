#!/usr/bin/env python
import cv2
import time
import numpy as np
import math

filename = "../maps/romance/romance.yaml"
maxObservableDistance = 4.0

rotAmount = 0.02
mvAmount = 10
resizeAmount = 10

#don't need to adjust vars below
foldername = "/".join(filename.split("/")[:-1])
#to account for diagonal distance of radius (1 over root 2)
maxObservableDistance = maxObservableDistance * (1/(2.0**0.5))
imagefn = ""
resolution = 1
origin = [0, 0]
boxX = 0
boxY = 0
boxW = 60
boxH = 40
boxR = 0
running = True
mapmul = 0

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
            resolution = line[1]
            mapmul = 1/float(resolution)
        elif "origin" in line[0]:
            line[1] = line[1].replace("[", "")
            line[1] = line[1].replace("]", "")
            origin[0] = float(line[1].split(",")[0])
            origin[1] = float(line[1].split(",")[1])

    fullfn = foldername + "/" + imagefn
    print("Opening: " + fullfn)
    rawimg = cv2.imread(fullfn)
    cv2.namedWindow("Map")
    
    boxW = boxW * mapmul
    boxH = boxH * mapmul
    boxX = (rawimg.shape[0]/2)
    boxY = (rawimg.shape[1]/2)

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

        tl = (int(boxX - ((boxW/2) * math.cos(boxR)) + ((boxH/2) * math.sin(boxR))), int(boxY - ((boxH/2) * math.cos(boxR)) - ((boxW/2) * math.sin(boxR))))
        tr = (int(boxX + ((boxW/2) * math.cos(boxR)) + ((boxH/2) * math.sin(boxR))), int(boxY - ((boxH/2) * math.cos(boxR)) + ((boxW/2) * math.sin(boxR))))
        bl = (int(boxX - ((boxW/2) * math.cos(boxR)) - ((boxH/2) * math.sin(boxR))), int(boxY + ((boxH/2) * math.cos(boxR)) - ((boxW/2) * math.sin(boxR))))
        br = (int(boxX + ((boxW/2) * math.cos(boxR)) - ((boxH/2) * math.sin(boxR))), int(boxY + ((boxH/2) * math.cos(boxR)) + ((boxW/2) * math.sin(boxR)))) 
        
        lineWidth = 5
        colour = (0, 255, 255)
        img = cv2.line(img, tl, tr, colour, lineWidth)
        img = cv2.line(img, tr, br, colour, lineWidth)
        img = cv2.line(img, br, bl, colour, lineWidth)
        img = cv2.line(img, bl, tl, colour, lineWidth)
            
        nWaypointsY = 0
        nWaypointsX = 0
        xDistCovered = 0
        yDistCovered = 0

        while xDistCovered < (boxW/mapmul):
            nWaypointsX += 1
            xDistCovered += maxObservableDistance * 2
        while yDistCovered < (boxH/mapmul):
            nWaypointsY += 1
            yDistCovered += maxObservableDistance * 2

        xOffset = (boxW/mapmul) / float(nWaypointsX + 1)
        yOffset = (boxH/mapmul) / float(nWaypointsY + 1)

        wp = []
        for i in range(nWaypointsY):
            for j in range(nWaypointsX):
                px = (xOffset * j) + xOffset
                py = (yOffset * i) + yOffset
                px -= (boxW/mapmul)/2
                py -= (boxH/mapmul)/2
                rx = (px * math.cos(boxR)) - (py * math.sin(boxR))
                ry = (px * math.sin(boxR)) + (py * math.cos(boxR))
                rx += (boxX / mapmul)
                ry += (boxY / mapmul)
                wp.append((rx, ry))

        colour = (255, 0, 255)
        lineWidth = 20
        for i in wp:
            pos = (int(i[0]*mapmul), int(i[1]*mapmul))
            img = cv2.line(img, pos, pos, colour, lineWidth)

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
