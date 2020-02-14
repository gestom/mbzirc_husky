#!/usr/bin/env python
import rospy
import math
from mbzirc_husky.srv import addInventory, addInventoryResponse
from mbzirc_husky.srv import getInventory, getInventoryResponse
from mbzirc_husky.srv import removeInventory, removeInventoryResponse
from mbzirc_husky.srv import accessibleInventory, accessibleInventoryResponse

blueprint = []
invBricks = []
invLayers = []
invPositions = []

def readBlueprint():

    file = None

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

if __name__ == "__main__":
    rospy.init_node("brickInventory")
    readBlueprint()
    addToInventoryService = rospy.Service("/inventory/add", addInventory, addToInventoryCB)
    accessibleInventoryService = rospy.Service("/inventory/accessible", accessibleInventory, accessibleInventoryCB)
    getInventoryService = rospy.Service("/inventory/get", getInventory, getInventoryCB)
    removeFromInventoryService = rospy.Service("/inventory/remove", removeInventory, removeFromInventoryCB)
    rospy.spin()
