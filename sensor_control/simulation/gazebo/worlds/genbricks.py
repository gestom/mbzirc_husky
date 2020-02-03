import random


partA = """
	  <model name='mbzirc_blue_brick_0'>
		<pose frame=''>-4.85759 13.9253 -5.3e-05 -0.000475 -0 0.00017</pose>
		<scale>1 1 1</scale>
		<link name='link'>
		  <pose frame=''>-4.85759 13.9253 0.099947 -0.000475 -0 0.00017</pose>
		  <velocity>-1e-06 0.014832 -0.013411 -0.14881 -4e-06 -2e-06</velocity>
		  <acceleration>0.004687 3.006 -4.7995 1.29349 0.043577 -0.007035</acceleration>
		  <wrench>0.001786 1.14529 -1.82861 0 -0 0</wrench>
		</link>
	  </model>
"""

partB = """
	<model name='mbzirc_blue_brick_0'>
	  <static>1</static>
	  <link name='link'>
		<pose frame=''>0 0 0.1 0 -0 0</pose>
		<inertial>
		  <mass>0.381</mass>
		  <inertia>
			<ixx>0.00252984</ixx>
			<ixy>0</ixy>
			<ixz>0</ixz>
			<iyy>0.046802</iyy>
			<iyz>0</iyz>
			<izz>0.046802</izz>
		  </inertia>
		</inertial>
		<visual name='main_box'>
		  <geometry>
			<box>
			  <size>1.2 0.2 0.2</size>
			</box>
		  </geometry>
		  <material>
			<script>
			  <uri>file://media/materials/scripts/gazebo.material</uri>
			  <name>Gazebo/Blue</name>
			</script>
		  </material>
		</visual>
		<visual name='white_plate'>
		  <pose frame=''>0 0 0.1 0 -0 0</pose>
		  <geometry>
			<box>
			  <size>0.3 0.15 0.006</size>
			</box>
		  </geometry>
		  <material>
			<script>
			  <uri>file://media/materials/scripts/gazebo.material</uri>
			  <name>Gazebo/White</name>
			</script>
		  </material>
		</visual>
		<collision name='collision'>
		  <pose frame=''>0 0 0 0 -0 0</pose>
		  <geometry>
			<box>
				<size>1.2 0.2 0.2</size>
			</box>
		  </geometry>
		  <max_contacts>10</max_contacts>
		  <surface>
			<contact>
			  <ode/>
			</contact>
			<bounce/>
			<friction>
			  <ode/>
			  <torsional>
				<ode/>
			  </torsional>
			</friction>
		  </surface>
		</collision>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<gravity>1</gravity>
		<enable_wind>0</enable_wind>
	  </link>
	  <pose frame=''>-4.85759 13.9253 0 0 -0 0</pose>
	</model>
"""
randomN = 0
import sys

def genABrick(colour, x, y, z):
	msg = partA
	rot = str(random.uniform(-0.015, 0.015))
	msg = msg.replace('mbzirc_blue_brick_0', 'brick_' + colour + '_' + str(randomN))
	msg = msg.replace('-4.85759 13.9253 -5.3e-05 -0.000475 -0 0.00017', str(x) + ' ' + str(y) + ' ' + str(z) + ' 0 0 ' + rot)
	msg = msg.replace('-4.85759 13.9253 0.099947 -0.000475 -0 0.00017', str(x) + ' ' + str(y) + ' ' + str(z) + ' 0 0 ' + rot)
	return msg

def genBBrick(colour, x, y, z):
	msg = partB
	msg = msg.replace('mbzirc_blue_brick_0', 'brick_' + colour + '_' + str(randomN))
	if colour == 'red':
		msg = msg.replace('<size>1.2 0.2 0.2</size>', '<size>0.3 0.2 0.2</size>')
		msg = msg.replace('Gazebo/Blue', 'Gazebo/Red')
		msg = msg.replace('<mass>0.381</mass>', '<mass>1.0</mass>')
	if colour == 'green':
		msg = msg.replace('<size>1.2 0.2 0.2</size>', '<size>0.6 0.2 0.2</size>')
		msg = msg.replace('Gazebo/Blue', 'Gazebo/Green')
		msg = msg.replace('<mass>0.381</mass>', '<mass>1.0</mass>')
	if colour == 'blue':
		msg = msg.replace('<size>1.2 0.2 0.2</size>', '<size>1.2 0.2 0.2</size>')
		msg = msg.replace('Gazebo/Blue', 'Gazebo/Blue')
		msg = msg.replace('<mass>0.381</mass>', '<mass>1.5</mass>')
	if colour == 'orange':
		msg = msg.replace('<size>1.2 0.2 0.2</size>', '<size>1.8 0.2 0.2</size>')
		msg = msg.replace('Gazebo/Blue', 'Gazebo/Orange')
		msg = msg.replace('<mass>0.381</mass>', '<mass>2.0</mass>')
	return msg

dataA = []
dataB = []

#red bricks
startX = 0 + random.uniform(-0.1, 0.1)
startY = 0 + random.uniform(-0.1, 0.1)
for ix in range(4):
	for iy in range(3):
		if iy == 0 and (ix == 0 or ix == 3):
			continue
		for iz in range(2):
			positionX = startX - (ix * (0.3 + 0.1 + random.uniform(-0.02, 0.02)))
			positionY = startY - (iy * (0.2 + 0.1 + random.uniform(-0.02, 0.02)))
			positionZ = (iz * 0.202) + 0.1 
			dataA.append(genABrick('red', -positionX, positionY, positionZ))
			dataB.append(genBBrick('red', -positionX, positionY, positionZ))
			randomN += 1
#green bricks
startX = -2.15 + random.uniform(-0.1, 0.1)
startY = 0 + random.uniform(-0.1, 0.1)
for ix in range(2):
	for iy in range(3):
		for iz in range(2):
			if (iz == 1) and iy == 0:
				continue
			positionX = startX - (ix * (0.6 + 0.1 + random.uniform(-0.02, 0.02)))
			positionY = startY - (iy * (0.2 + 0.1 + random.uniform(-0.02, 0.02)))
			positionZ = (iz * 0.202) + 0.1
			dataA.append(genABrick('green', -positionX, positionY, positionZ))
			dataB.append(genBBrick('green', -positionX, positionY, positionZ))
			randomN += 1
#blue bricks
startX = -4.25 + random.uniform(-0.1, 0.1)
startY = 0 + random.uniform(-0.1, 0.1)
for ix in range(1):
	for iy in range(3):
		for iz in range(2):
			if iy == 0 and iz == 1:
				continue
			positionX = startX - (ix * (1.2 + 0.1 + random.uniform(-0.02, 0.02)))
			positionY = startY - (iy * (0.2 + 0.1 + random.uniform(-0.02, 0.02)))
			positionZ = (iz * 0.202) + 0.1
			dataA.append(genABrick('blue', -positionX, positionY, positionZ))
			dataB.append(genBBrick('blue', -positionX, positionY, positionZ))
			randomN += 1
#orange bricks
startX = -6.25 + random.uniform(-0.1, 0.1)
startY = 0 + random.uniform(-0.1, 0.1)
for ix in range(1):
	for iy in range(3):
		for iz in range(4):
			if iz == 3 and (iy == 0 or iy == 2):
				continue
			positionX = startX - (ix * (1.8 + 0.1 + random.uniform(-0.02, 0.02)))
			positionY = startY - (iy * (0.2 + 0.1 + random.uniform(-0.02, 0.02)))
			positionZ = (iz * 0.202) + 0.1
			dataA.append(genABrick('orange', -positionX, positionY, positionZ))
			dataB.append(genBBrick('orange', -positionX, positionY, positionZ))
			randomN += 1

file = ""
with open('bricks.world.template', 'r') as f:
	file = f.read()

file = file.replace('###A###', '\n'.join(dataA))
file = file.replace('###B###', '\n'.join(dataB))

with open('bricks.world', "w") as f:
	f.write(file)
