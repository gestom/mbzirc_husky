
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
	  <static>0</static>
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
int randomN = 0

def genABrick(colour, x, y, z):
	msg = partA
	msg.replace('mbzirc_blue_brick_0', 'brick_' + colour + '_' + str(randomN))
	randomN += 1
	msg.replace('-4.85759 13.9253 -5.3e-05 -0.000475 -0 0.00017', x + ' ' + y + ' ' + z + ' 0 0 0')
	msg.replace('-4.85759 13.9253 0.099947 -0.000475 -0 0.00017', x + ' ' + y + ' ' + z + ' 0 0 0')
	return msg



dataA = []
dataB = []

#red bricks
startX = 0
startY = 0
for ix in range(4):
	for iy in range(3):
		if iy == 0 and (ix == 0 or ix == 3):
			continue
		for iz in range(2):
			positionX = (ix * (0.3 + 0.1))
			positionY = (iy * (0.2 + 0.1))
			positionZ = (iz * 0.21) + 0.0 
			dataA.append(genABrick('red', positionX, positionY, positionZ))
			dataB.append(genBBrick('red', positionX, positionY, positionZ))
#green bricks
startX = 8
startY = 0
for ix in range(2):
	for iy in range(3):
		for iz in range(2):
			if (iz == 1) and iy == 0:
				continue
			positionX = (ix * (0.6 + 0.1))
			positionY = (iy * (0.2 + 0.1))
			positionZ = (iz * 0.21) + 0.0 
			dataA.append(genABrick('green', positionX, positionY, positionZ))
			dataB.append(genBBrick('green', positionX, positionY, positionZ))
#blue bricks
startX = 16
startY = 0
for ix in range(1):
	for iy in range(3):
		for iz in range(2):
			if iy == 0 and iz == 1:
				continue
			positionX = (ix * (1.2 + 0.1))
			positionY = (iy * (0.2 + 0.1))
			positionZ = (iz * 0.21) + 0.0 
			dataA.append(genABrick('blue', positionX, positionY, positionZ))
			dataB.append(genBBrick('blue', positionX, positionY, positionZ))
#orange bricks
startX = 24
startY = 0
for ix in range(1):
	for iy in range(3):
		for iz in range(4):
			if iz == 3 and (iy == 0 or iy == 2):
				continue
			positionX = (ix * (1.8 + 0.1))
			positionY = (iy * (0.2 + 0.1))
			positionZ = (iz * 0.21) + 0.0 
			dataA.append(genABrick('orange', positionX, positionY, positionZ))
			dataB.append(genBBrick('orange', positionX, positionY, positionZ))