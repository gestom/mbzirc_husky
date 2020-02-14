#!/usr/bin/env python
import rospy
import math
from mbzirc_husky.srv import addInventory, addInventoryResponse
from mbzirc_husky.srv import getInventory, getInventoryResponse
from mbzirc_husky.srv import removeInventory, removeInventoryResponse
from mbzirc_husky.srv import accessibleInventory, accessibleInventoryResponse
from mbzirc_husky.srv import nextBrickPlacement, nextBrickPlacementResponse
from mbzirc_husky.srv import brickBuilt, brickBuiltResponse
from mbzirc_husky.srv import debugFillInventory, debugFillInventoryResponse

blueprint = []
bricksCompleted = []
invBricks = []
invLayers = []
invPositions = []

def readBlueprint():
    global blueprint, bricksCompleted

    file = None
    with open("../bricks/bricks-difficult.txt", "r") as f:
        file = f.read()
    file = file.split("\n")
    file = filter(None, file)

    for line in file:
        line = list(line)
        blueprint.append(line)
        completedLine = []
        for i in line:
            completedLine.append(False)
        bricksCompleted.append(completedLine)

def addToInventoryCB(req):
    global invBricks, invLayers, invPositions
    if req.brickType < 0 or req.brickType > 3 or math.isnan(req.brickType):
        print("Bad brick type added to inventory, ignoring")
        return
    if math.isnan(req.position) or math.isnan(req.layer):
        print("NaN added to inventory position/layer, ignoring")
        return
    for i in range(len(invBricks)):
        if invLayers[i] == req.layer:
            if invPositions[i] == req.position:
                print("Warning: layer and position already full, ignoring")
                return
    invBricks.append(req.brickType)
    invLayers.append(req.layer)
    invPositions.append(req.position)
    print("Brick added t-%f l-%f p-%f", req.brickType, req.layer, req.position) 
    return addInventoryResponse()

def getInventoryCB(req):
    global invBricks, invLayers, invPositions
    reply = getInventoryResponse(brickTypes=invBricks,position=invPositions,layer=invLayers)
    print("Request for inventory received")
    return reply

def accessibleInventoryCB(req):
    global invBricks, invLayers, invPositions
    #test if we have it, if not return -1
    if req.brickType < 0 or req.brickType > 3:
        print("Bad brick type for accessiblity test")
        return
    if req.brickType not in invBricks:
        print("Brick accessibility test t-%f: not present", req.brickType)
        return accessibleInventoryResponse(accessible=-1)

    if req.brickType == 2:
        #blue brick
        print("Brick accessibility test t-%f: accessible", req.brickType)
        return accessibleInventoryResponse(accessible=1)

    if 2 in invBricks:
        print("Brick accessibility test t-%f: inaccessible", req.brickType)
        return accessibleInventoryResponse(accessible=0)
    else:
        print("Brick accessibility test t-%f: accessible", req.brickType)
        return accessibleInventoryResponse(accessible=1)

def removeFromInventoryCB(req):
    global invBricks, invLayers, invPositions
    delIndex = -1
    for i in range(len(invBricks)):
        if invLayers[i] == req.layer:
            if invPositions[i] == req.position:
                delIndex = i
    try:
        del invBricks[delIndex]
        del invLayers[delIndex]
        del invPositions[delIndex]
    except:
        pass
    return removeInventoryResponse()

def nextBrickPlacementCB(req):
    global invBricks, invLayers, invPositions

    print("Sending next brick to be placed")

    if len(invBricks) < 1:
        return nextBrickPlacementResponse(brickType=-1, position=-1, layer=-1, wallOriginOffset=-1, wallLayer=-1, wallIndex=-1)

    if 2 in invBricks:
        #if a blue is present, put this down first
        bluepos = -1
        for i in range(len(invBricks)):
            if invBricks[i] == 2:
                bluepos = i
        for lineIdx in range(len(blueprint)):
            wallOffset = 0
            for brickIdx in range(len(blueprint[lineIdx])):
                if blueprint[lineIdx][brickIdx] == "B":
                    #test if its already been put down
                    if bricksCompleted[lineIdx][brickIdx] == False:
                        wallOffset += 0.6
                        return nextBrickPlacementResponse(brickType=2, position=invPositions[bluepos], layer=invLayers[bluepos], wallOriginOffset=wallOffset, wallLayer=lineIdx, wallIndex=brickIdx)
                    else:
                        wallOffset += 1.2 + req.offset
                else:
                    if blueprint[lineIdx][brickIdx] == "R":
                        wallOffset += 0.3 + req.offset
                    elif blueprint[lineIdx][brickIdx] == "G":
                        wallOffset += 0.6 + req.offset
                    else:
                        print("Unrecornigsed brick type")
        
    #find top layer bricks
    elif 1 in invLayers:
        brickPos = -1
        for i in range(len(invBricks)):
            if invLayers[i] == 1:
                brickpos = i
        brickType = invBricks[i]
        for lineIdx in range(len(blueprint)):
            wallOffset = 0
            for brickIdx in range(len(blueprint[lineIdx])):
                if (blueprint[lineIdx][brickIdx] == "R" and brickType == 0) or (blueprint[lineIdx][brickIdx] == "B" and brickType == 1):
                    #test if its already been put down
                    if bricksCompleted[lineIdx][brickIdx] == False:
                        if brickType == 0:
                            wallOffset += 0.15
                        else:
                            wallOffset += 0.3
                        return nextBrickPlacementResponse(brickType=brickType, position=invPositions[brickpos], layer=invLayers[brickpos], wallOriginOffset=wallOffset, wallLayer=lineIdx, wallIndex=brickIdx)
                    else:
                        if brickType == 0:
                            wallOffset += 0.15 + req.offset
                        else:
                            wallOffset += 0.3 + req.offset
                else:
                    if blueprint[lineIdx][brickIdx] == "R":
                        wallOffset += 0.3 + req.offset
                    elif blueprint[lineIdx][brickIdx] == "G":
                        wallOffset += 0.6 + req.offset
                    else:
                        print("Unrecornigsed brick type")

    #handle bottom layer of bricks
    elif 0 in invLayers:
        brickPos = -1
        for i in range(len(invBricks)):
            if invLayers[i] == 0:
                brickpos = i
        brickType = invBricks[i]
        for lineIdx in range(len(blueprint)):
            wallOffset = 0
            for brickIdx in range(len(blueprint[lineIdx])):
                if (blueprint[lineIdx][brickIdx] == "R" and brickType == 0) or (blueprint[lineIdx][brickIdx] == "B" and brickType == 1):
                    #test if its already been put down
                    if bricksCompleted[lineIdx][brickIdx] == False:
                        if brickType == 0:
                            wallOffset += 0.15
                        else:
                            wallOffset += 0.3
                        return nextBrickPlacementResponse(brickType=brickType, position=invPositions[brickpos], layer=invLayers[brickpos], wallOriginOffset=wallOffset, wallLayer=lineIdx, wallIndex=brickIdx)
                    else:
                        if brickType == 0:
                            wallOffset += 0.15 + req.offset
                        else:
                            wallOffset += 0.3 + req.offset
                else:
                    if blueprint[lineIdx][brickIdx] == "R":
                        wallOffset += 0.3 + req.offset
                    elif blueprint[lineIdx][brickIdx] == "G":
                        wallOffset += 0.6 + req.offset
                    else:
                        print("Unrecornigsed brick type")
    else:
        print("Error checking layer variables")

def brickBuiltCB(req):
    global invBricks, invLayers, invPositions
   
    print("Brick Completed")

    #delete from inventory
    delIndex = None
    for i in range(len(invBricks)):
        if invLayers[i] == req.fromLayer:
            if invPositions[i] == req.fromPosition:
                delIndex = i
    try:
        del invBricks[delIndex]
        del invLayers[delIndex]
        del invPositions[delIndex]
    except:
        print("Exception deleteing brick from inv on completion")

    bricksCompleted[req.buildLayer][req.buildIndex] = True 
    return brickBuiltResponse()

def debugFillInventoryCB(req):
    global invBricks, invLayers, invPositions

    print("Debug filling inventory with all bricks")

    invBricks = [0, 1, 0, 0, 1, 0, 2]
    invLayers = [0, 0, 0, 1, 1, 1, 2]
    invPositions = [0, 2, 1, 0, 2, 1, 3]
    
    return debugFillInventoryResponse()

if __name__ == "__main__":
    rospy.init_node("brickInventory")
    readBlueprint()
    addToInventoryService = rospy.Service("/inventory/add", addInventory, addToInventoryCB)
    accessibleInventoryService = rospy.Service("/inventory/accessible", accessibleInventory, accessibleInventoryCB)
    getInventoryService = rospy.Service("/inventory/get", getInventory, getInventoryCB)
    removeFromInventoryService = rospy.Service("/inventory/remove", removeInventory, removeFromInventoryCB)
    nextBrickPlacementService = rospy.Service("/inventory/nextBrickPlacement", nextBrickPlacement, nextBrickPlacementCB)
    brickBuiltService = rospy.Service("/inventory/brickBuilt", brickBuilt, brickBuiltCB)
    debugFillInventoryService = rospy.Service("/inventory/debugFillInventory", debugFillInventory, debugFillInventoryCB)
    rospy.spin()
