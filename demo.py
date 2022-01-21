import pybullet as p
import time
import pybullet_data
import pathlib
import os

##############################################
# Start environment and set physics settings #
##############################################

#Start our physics client with the GUI on
physicsClient = p.connect(p.GUI)
#Setting physics settings within the environment
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0,0,-10)

#########################################
# Load in the models to our environment #
#########################################

#Load in a plane for our robot arm to sit on
planeId = p.loadURDF("plane.urdf")

#the data path to the provided Kuka arm in pybullet
kuka_arm_path = "kuka_iiwa/model.urdf"
data_path = os.path.join(pybullet_data.getDataPath(), kuka_arm_path)
#Loading in the urdf of our Kuka arm at the desired origin position and orientation
time.sleep(10)
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robot_id = p.loadURDF(data_path,startPos, startOrientation, useFixedBase=True)
time.sleep(10)

#######################################################
# Move the arm into first position using Joint Angles #
#######################################################

#Gets number of joints on the Kuka robot
num_joints = p.getNumJoints(robot_id)
#Sets each joint motor controller on the arm to the value we wish to move to 
for i in range(num_joints):
    #We are using a position controller with a maxVelocity of 1 
    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=1.5, maxVelocity=1)
#We then step the simulator for 500 timesteps to allow the robot arm to move to the desired location
for i in range (500):
    p.stepSimulation()
    time.sleep(1./240.)

###############################################################
# Move the arm back into original position using Joint Angles #
###############################################################

#Repeat of above code, but moving the arm back into the original position
for i in range(num_joints):
    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=0, maxVelocity=1)
#We then step the simulator for 500 timesteps to move the arm back into the start position
for i in range (500):
    p.stepSimulation()
    time.sleep(1./240.)

#########################################################################
# Move the end effector to a point in 3D space using Inverse Kinematics #
#########################################################################

#Given a point in 3D space we want to move the end effector to that point 
#This commands calculates the needed joint angles to do so
ik_joint_positions = p.calculateInverseKinematics(robot_id, num_joints-1, [.3,.3,.1])
#We then set those joint angles to the motor controller in each joint like before
for i in range(num_joints):
    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=ik_joint_positions[i], maxVelocity=1)

#We then step the simulator for 500 timesteps to move the arm into the calculated position
for i in range (600):
    p.stepSimulation()
    time.sleep(1./240.)

#Disconnect from the physics client
p.disconnect()