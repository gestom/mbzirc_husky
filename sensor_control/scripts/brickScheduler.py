#!/usr/bin/env python
import rospy
from mbzirc_husky_msgs.msg import brickInventory as invMsg 
from mbzirc_husky_msgs.msg import brickPickedUp as pickupMsg 

blueprint = None

brickInvType = []
brickInvLvl = []
brickInvSpace = []
inventoryPub = None

def publishInventory():

    msg = invMsg()
    msg.type = brickInvType
    msg.level = brickInvLvl
    msg.space = brickInvSpace
    inventoryPub.pub(msg)

def pickupCallback(msg):

    #sanitize input
    if msg.type < 0 or msg.type > 3:
        print("Invalid brick type received by scheduler")
    if msg.level < 0:
        print("Invalid brick level received by scheduler")
    if msg.space < 0:
        print("Invalid brick space received by scheduler")

    brickInvType.append(msg.type)
    brickInvLvl.append(msg.level)
    brickInvSpace.append(msg.space)

def readBlueprint():

    file = None
    with open("../bricks/bricks-difficult.txt", "r") as f:
        file = f.read()
    file = file.split("\n")
    file = filter(None, file)

    for line in file:
        line = list(line)
        blueprint.append(line)

if __name__ == "__main__":
    rospy.init_node("brickScheduler")

    readBlueprint()

    inventoryPub = rospy.Publisher("/brickScheduler/inventory", invMsg, queue_size=1)
    rospy.Subscriber("/brickScheduler/pickedUp", pickupMsg, pickupCallback)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

        publishInventory()

        rate.sleep()
