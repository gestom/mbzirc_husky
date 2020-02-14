#!/usr/bin/env python
import rospy
from mbzirc_husky_msgs.msg import brickRequirements as reqmsg
from mbzirc_husky_msgs.msg import brickCompleted as finmsg

#assumes 2 reds = 1 green, 2 greens = 1 blue
#robot capabilities:
spans = {"R": 1, "G": 2, "B": 4}
capabilities = {"R": True, "G": True, "B": False}
inventory = (3, 1)
brickRequirements = []

bricks = []
completed = []
capable = []
changed = True

def deleteTree(rowIdx, brickIdx):
	#as we're working top to bottom, if this tree is already done, quit
	if capable[rowIdx][brickIdx] == False:
		return

	#invalidate brick
	capable[rowIdx][brickIdx] = False

	if rowIdx+1 >= len(bricks):
		return

	#get abs index from one side	
	absIdx = 0
	for i in range(brickIdx):
		currentType = bricks[rowIdx][i]
		absIdx += spans[currentType]

	#get brick type so we cant work out its span
	brickType = bricks[rowIdx][brickIdx]

	#now we have a range of absolute columns to invalidate above, so we will convert them to real bricks,
	#then call this function recursively on them
	#as we worked top down on the wall, if the brick is already invalidated, we can skip that tree
	for i in range(absIdx, absIdx + spans[brickType]):

		for currentBrickIdx in range(len(bricks[rowIdx])):
			# print(bricks[rowIdx+1])
			currentType = bricks[rowIdx+1][currentBrickIdx]
			i -= spans[currentType]
			if i < 0:
				deleteTree(rowIdx+1, currentBrickIdx)
				break

def calcCapable():
	#row of bricks top to bottom
	for rowIdx in range(len(bricks)-1, -1, -1):
		#for each brick in row
		for brickIdx in range(len(bricks[rowIdx])):
			brickType = bricks[rowIdx][brickIdx]
			if capabilities[brickType] == False:
				#delete brick and all above
				deleteTree(rowIdx, brickIdx)

def recalc():
	global brickRequirements, changed
	currentInventory = list(inventory)
	brickRequirements = []
	for rowIdx in range(len(bricks)):
		for brickIdx in range(len(bricks[rowIdx])):
			#if it's already been done or is impossible, skip
			if completed[rowIdx][brickIdx] == True:
				continue
			if capable[rowIdx][brickIdx] == False:
				continue

			#if the candidate can't fit in the robot, skip
			brickType = bricks[rowIdx][brickIdx]
			brickSize = spans[brickType]

			for i in range(len(currentInventory)):
				if currentInventory[i] >= brickSize:
					currentInventory[i] -= brickSize
					brickRequirements.append((brickType, i, rowIdx, brickIdx))
					break
	changed = False

def brickFinishedCallback(msg):
	global changed
	
	if msg.wallRow < 0 or msg.wallRow > 4:
		print("ERROR!!!!! Your wall row field is out of bounds, ignoring, but this is unsafe....")
		return
	if msg.wallColumn < 0 or msg.wallColumn > 6:
		print("ERROR!!!!! Your wall col field is out of bounds, ignoring, but this is unsafe....")
		return

	row = msg.wallRow
	col = msg.wallColumn

	completed[row][col] = True
	displayGrid(completed, "completed (t->b)")
	changed = True

def displayGrid(grid, title):
	print("=============" + title + "=============")
	for i in grid:
		print(i)
	print("")

if __name__ == "__main__":

	rospy.init_node("brickScheduler")

	with open("../bricks/bricks-difficult.txt", "r") as f:
		file = f.read()
	file = file.split("\n")
	file = filter(None, file)

	for line in file:
		line = list(line)
		bricks.append(line)
		lineCompleted = []
		lineCapable = []
		for i in line:
			lineCompleted.append(False)
			lineCapable.append(True)
		completed.append(lineCompleted)
		capable.append(lineCapable)
	calcCapable()

	rate = rospy.Rate(2)
	rospy.Subscriber("/brickScheduler/completed", finmsg, brickFinishedCallback)
	pub = rospy.Publisher("/brickScheduler/goals", reqmsg, queue_size=1)

	while not rospy.is_shutdown():

		if changed:

			recalc()

			print("====================" + "Current State" + "====================")
			displayGrid(bricks, "blueprint (t->b)")
			displayGrid(capable, "robot capability (t->b)")
			displayGrid(completed, "completed (t->b)")
			displayGrid(brickRequirements, "brick goals")

		brickTypes = []
		inventoryIDs = []
		wallRows = []
		wallColumns = []

		for i in brickRequirements:
			if i[0] == "R":
				brickTypes.append(0)
			elif i[0] == "G":
				brickTypes.append(1)
			elif i[0] == "B":
				brickTypes.append(2)
			inventoryIDs.append(i[1])
			wallRows.append(i[2])
			wallColumns.append(i[3])

		msg = reqmsg()
		msg.brickType = brickTypes
		msg.inventoryID = inventoryIDs
		msg.wallRow = wallRows
		msg.wallColumn = wallColumns

		pub.publish(msg)

		rate.sleep()
