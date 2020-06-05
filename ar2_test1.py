#!/usr/bin/env python3
import pybullet
import time
import pybullet_data
import os

pybullet.connect(pybullet.GUI)
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybuplane = pybullet.loadURDF("plane.urdf")
os.system("git clone https://github.com/Chris-Annin/ROS_AR2_urdf.git ar2")
robot = pybullet.loadURDF("ar2/urdf/ar2.urdf", [0, 0, 0], useFixedBase=1)
pybullet.resetDebugVisualizerCamera(1.40, -53.0, -39.0, (0.53, 0.21, -0.24))
time.sleep(2)

pybullet.setTimeStep(0.0001)
pybullet.setRealTimeSimulation(0)
steps = 400
fixedTimeStep = 1.5 / steps

for jointNumber in range(6):
    for posJoint in range(0, steps+1):
        pybullet.setJointMotorControl2(robot, jointNumber, pybullet.POSITION_CONTROL, -posJoint/steps)
        pybullet.stepSimulation()
        time.sleep(fixedTimeStep)
    for posJoint in reversed(range(0, steps+1)):
        pybullet.setJointMotorControl2(robot, jointNumber, pybullet.POSITION_CONTROL, -posJoint/steps)
        pybullet.stepSimulation()
        time.sleep(fixedTimeStep)
    for posJoint in range(0, steps+1):
        pybullet.setJointMotorControl2(robot, jointNumber, pybullet.POSITION_CONTROL, posJoint/steps)
        pybullet.stepSimulation()
        time.sleep(fixedTimeStep)
    for posJoint in reversed(range(0, steps+1)):
        pybullet.setJointMotorControl2(robot, jointNumber, pybullet.POSITION_CONTROL, posJoint/steps)
        pybullet.stepSimulation()
        time.sleep(fixedTimeStep)
