import pybullet as p
import time
import math
#p.connect(p.UDP,"192.168.86.100")


cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	cid = p.connect(p.GUI_SERVER)
p.resetSimulation()
#disable rendering during loading makes it much faster
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("samurai.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("pr2_gripper.urdf", 0.500000,0.300006,0.700000,-0.000000,-0.000000,-0.000031,1.000000)]
pr2_gripper = objects[0]
print ("pr2_gripper=")
print (pr2_gripper)

jointPositions=[ 0.550569, 0.000000, 0.549657, 0.000000 ]
for jointIndex in range (p.getNumJoints(pr2_gripper)):
	p.resetJointState(pr2_gripper,jointIndex,jointPositions[jointIndex])

pr2_cid = p.createConstraint(pr2_gripper,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0.2,0,0],[0.500000,0.300006,0.700000])
print ("pr2_cid")
print (pr2_cid)

objects = [p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000,-0.200000,0.600000,0.000000,0.000000,0.000000,1.000000)]
kuka = objects[0]
jointPositions=[ -0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001 ]
for jointIndex in range (p.getNumJoints(kuka)):
	p.resetJointState(kuka,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(kuka,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)

objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.700000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.800000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("lego/lego.urdf", 1.000000,-0.200000,0.900000,0.000000,0.000000,0.000000,1.000000)]
objects = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")
kuka_gripper = objects[0]
print ("kuka gripper=")
print(kuka_gripper)

p.resetBasePositionAndOrientation(kuka_gripper,[0.923103,-0.200000,1.250036],[-0.000000,0.964531,-0.000002,-0.263970])
jointPositions=[ 0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000 ]
for jointIndex in range (p.getNumJoints(kuka_gripper)):
	p.resetJointState(kuka_gripper,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(kuka_gripper,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)


kuka_cid = p.createConstraint(kuka,   6,  kuka_gripper,0,p.JOINT_FIXED, [0,0,0], [0,0,0.05],[0,0,0])

pr2_cid2 = p.createConstraint(kuka_gripper,4,kuka_gripper,6,jointType=p.JOINT_GEAR,jointAxis =[1,1,1],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(pr2_cid2,gearRatio=-1, erp=0.5, relativePositionTarget=0, maxForce=100)


objects = [p.loadURDF("jenga/jenga.urdf", 1.300000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 1.200000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 1.100000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 1.000000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 0.900000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("jenga/jenga.urdf", 0.800000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
objects = [p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)]
objects = p.loadSDF("kiva_shelf/model.sdf")
ob = objects[0]
p.resetBasePositionAndOrientation(ob,[0.000000,1.000000,1.204500],[0.000000,0.000000,0.000000,1.000000])
objects = [p.loadURDF("teddy_vhacd.urdf", -0.100000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("sphere_small.urdf", -0.100000,0.955006,1.169706,0.633232,-0.000000,-0.000000,0.773962)]
objects = [p.loadURDF("cube_small.urdf", 0.300000,0.600000,0.850000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("table_square/table_square.urdf", -1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
ob = objects[0]
jointPositions=[ 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

objects = [p.loadURDF("husky/husky.urdf", 2.000000,-5.000000,1.000000,0.000000,0.000000,0.000000,1.000000)]
ob = objects[0]
jointPositions=[ 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

p.setGravity(0.000000,0.000000,0.000000)
p.setGravity(0,0,-10)

##show this for 10 seconds
#now = time.time()
#while (time.time() < now+10):
#	p.stepSimulation()
p.setRealTimeSimulation(1)

# add control stuff here
################################################################3
kuka = 3
kuka_gripper = 7
POSITION = 1
ORIENTATION = 2
ANALOG=3
BUTTONS = 6


THRESHOLD = .5
LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
JOINT_DAMP = [0.1]*10
REST_JOINT_POS = [-0., -0., 0., 1.570793, 0., -1.036725, 0.000001]
MAX_FORCE = 500
KUKA_GRIPPER_REST_POS = [0., -0.011130, -0.206421, 0.205143, -0.009999, 0., -0.010055, 0.]
KUKA_GRIPPER_CLOZ_POS = [0.0, -0.047564246423083795, 0.6855956234759611, -0.7479294372303137, 0.05054599996976922, 0.0, 0.049838105678835724, 0.0]

def euc_dist(posA, posB):
	dist = 0.
	for i in range(len(posA)):
		dist += (posA[i] - posB[i]) ** 2
	return dist

controllers = [e[0] for e in p.getVREvents()]

for j in range(p.getNumJoints(kuka_gripper)):
	print(p.getJointInfo(kuka_gripper,j))

while (1):
	p.setGravity(0,0,-10)

	# control stuff
	###################################################################
	events = p.getVREvents()
	for e in (events):

		# Only use one controller
		###########################################
		# This is important: make sure there's only one VR Controller!
		# if e[0] == controllers[0]:
			# break

		sq_len = euc_dist(p.getLinkState(kuka, 6)[0], e[POSITION])

		# A simplistic version of gripper control
		#@TO-DO: Add slider for the gripper

		print(e[ANALOG])

		#for i in range(p.getNumJoints(kuka_gripper)):
		i=4
		p.setJointMotorControl2(kuka_gripper, i, p.POSITION_CONTROL, targetPosition=e[ANALOG]*0.05, force=10)
		i=6
		p.setJointMotorControl2(kuka_gripper, i, p.POSITION_CONTROL, targetPosition=e[ANALOG]*0.05, force=10)


		if sq_len < THRESHOLD * THRESHOLD:
			eef_pos = e[POSITION]
			eef_orn = p.getQuaternionFromEuler([0,-math.pi,0])
			joint_pos = p.calculateInverseKinematics(kuka, 6, eef_pos, eef_orn,
				lowerLimits=LOWER_LIMITS, upperLimits=UPPER_LIMITS,
				jointRanges=JOINT_RANGE, restPoses=REST_POSE, jointDamping=JOINT_DAMP)


			for i in range(len(joint_pos)):
				p.setJointMotorControl2(kuka, i, p.POSITION_CONTROL,
					targetPosition=joint_pos[i], targetVelocity=0, positionGain=0.15,
					velocityGain=1.0, force=MAX_FORCE)


		else:
			# Set back to original rest pose
			for jointIndex in range(p.getNumJoints(kuka)):
				p.setJointMotorControl2(kuka, jointIndex, p.POSITION_CONTROL,
					REST_JOINT_POS[jointIndex], 0)

p.disconnect()
