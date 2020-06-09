#!/usr/bin/env python3

#  Starting the simulation
import pybullet

# pour pouvoir utiliser les délias (time.sleep)
import time

import math

import pybullet_data

from datetime import datetime

# pour pouvoir utiliser la comemande système git clone
import os


pybullet.connect(pybullet.GUI)

# pour remettre à zéro la simulation
pybullet.resetSimulation()

pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# plane est le sol il est contenu dans l'import pybullet_data
pybuplane = pybullet.loadURDF("plane.urdf")

#os.system("git clone https://github.com/Chris-Annin/ROS_AR2_urdf.git ar2")

# on récpère le bras robot depuiq le dossier git qu'on a récupéré
# c'est important de lui mettre une useFixedBase à 1 pour qu'il ne tombe pas (avec la gravité par exemple)
robot = pybullet.loadURDF("ar2/urdf/ar2.urdf", [0, 0, 0], useFixedBase=1)

# importation du socle
# socle = pybullet.loadURDF("socle.urdf", [50, 0, 0], useFixedBase=1)


pybullet.resetDebugVisualizerCamera(1.40, -53.0, -39.0, (0.53, 0.21, -0.24))
numJoints = pybullet.getNumJoints(robot)
time.sleep(2)

pybullet.setTimeStep(0.0001)
pybullet.setRealTimeSimulation(0)
steps = 400
fixedTimeStep = 1.5 / steps

robotEndEffectorIndex = numJoints - 1
pybullet.setGravity(0, 0,0)
t = 0.
base = pybullet.getBasePositionAndOrientation(robot)
print(base)

#trailDuration = 15 #duration after debug lines will be removed, 0 for no-removal
def descendre():
    for jointNumber in range(numJoints):
        pybullet.setJointMotorControl2(bodyIndex=robot, jointIndex=jointNumber, controlMode=pybullet.POSITION_CONTROL, targetPosition=jointPoses[jointNumber], targetVelocity=0, force=500, positionGain=0.03, velocityGain=1)
        time.sleep(0.001)

def monter():
    for jointNumber in range(numJoints):
        pybullet.setJointMotorControl2(bodyIndex=robot, jointIndex=jointNumber, controlMode=pybullet.POSITION_CONTROL, targetPosition=jointPosesBase[jointNumber], targetVelocity=0, force=500, positionGain=0.03, velocityGain=1)
        time.sleep(0.001)


i=0
while 1:
    #i +=1
    #dt = datetime.now()
    t += 0.01
    pybullet.stepSimulation()

    pos = [0.005, -0.46, 0.11]
    #posbase = pybullet.getEulerFromQuaternion(base)
    orn = pybullet.getQuaternionFromEuler([0, -math.pi, 0])

    jointPosesBase = pybullet.calculateInverseKinematics(robot, robotEndEffectorIndex, [-4.6947e-06, 0.054174, 0.038824], [0, 0, 0, 1])
    jointPoses = pybullet.calculateInverseKinematics(robot, robotEndEffectorIndex, pos, orn)

    #permet de selectionner quel joint on va utiliser (JointNumber)
    descendre()
    monter()

    #for jointNumber2 in range(numJoints):
        #for posJoint in reversed(range(0, steps+1)):
        #pybullet.setJointMotorControl2(bodyIndex=robot, jointNumber2, pybullet.POSITION_CONTROL, jointPosesBase[jointNumber2])
        #pybullet.resetJointState(robot, jointNumber2, jointPoses[jointNumber2])
        #pybullet.stepSimulation()
        #time.sleep(0.01)
    #ls = pybullet.getLinkState(robot, robotEndEffectorIndex)

        #for posJoint in range(0, steps+1):
        #    pybullet.setJointMotorControl2(robot, jointNumber, pybullet.POSITION_CONTROL, posJoint/steps)
        #    pybullet.stepSimulation()
        #    time.sleep(fixedTimeStep)
        #for posJoint in reversed(range(0, steps+1)):
        #    pybullet.setJointMotorControl2(robot, jointNumber, pybullet.POSITION_CONTROL, posJoint/steps)
        #    pybullet.stepSimulation()
        #    time.sleep(fixedTimeStep)
