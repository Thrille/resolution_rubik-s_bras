#!/usr/bin/env python3

#  Starting the simulation
import pybullet

# pour pouvoir utiliser les délias (time.sleep)
import time

import pybullet_data

# pour pouvoir utiliser la comemande système git clone
import os


pybullet.connect(pybullet.GUI)

# pour remettre à zéro la simulation
pybullet.resetSimulation()

pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# plane est le sol il est contenu dans l'import pybullet_data
pybuplane = pybullet.loadURDF("plane.urdf")

os.system("git clone https://github.com/Chris-Annin/ROS_AR2_urdf.git ar2")

pybullet.setGravity(0,0,-9.8)

# on récpère le bras robot depuiq le dossier git qu'on a récupéré
# c'est important de lui mettre une useFixedBase à 1 pour qu'il ne tombe pas (avec la gravité par exemple)
robot = pybullet.loadURDF("ar2/urdf/ar2.urdf", [0, 0, 0], useFixedBase=1)

# importation du socle
socle = pybullet.loadURDF("socle.urdf", [0, -0.5, 0], useFixedBase=1)


# pour faire l'orientation du cub
orientation = pybullet.getQuaternionFromEuler([73,0,0])
# importation du cube
cube = pybullet.loadURDF("cube.urdf", [0.05, -0.46, 0.11], orientation)


pybullet.resetDebugVisualizerCamera(1.40, -53.0, -39.0, (0.53, 0.21, -0.24))
time.sleep(2)

pybullet.setTimeStep(0.0001)
pybullet.setRealTimeSimulation(0)
steps = 400
fixedTimeStep = 1.5 / steps

#permet de selectionner quel joint on va utiliser (JointNumber)
for jointNumber in range(6):
    #permet d'effectuer les steps du mouvement
    for posJoint in range(0, steps+1):
        #controle le joint choisis(robot est l'objet robot, jointNumber le joint choisis, je sais pas, déterminer l'angle effectuer grace au pos joints et au step)
        pybullet.setJointMotorControl2(robot, jointNumber, pybullet.POSITION_CONTROL, -posJoint/steps)
        #effectue le mouvement
        pybullet.stepSimulation()
        time.sleep(fixedTimeStep)
    #fait le mouvement précédent dans l'autre sens pour revenir à la position initiale
    for posJoint in reversed(range(0, steps+1)):
        pybullet.setJointMotorControl2(robot, jointNumber, pybullet.POSITION_CONTROL, -posJoint/steps)
        pybullet.stepSimulation()
        time.sleep(fixedTimeStep)
    #for posJoint in range(0, steps+1):
    #    pybullet.setJointMotorControl2(robot, jointNumber, pybullet.POSITION_CONTROL, posJoint/steps)
    #    pybullet.stepSimulation()
    #    time.sleep(fixedTimeStep)
    #for posJoint in reversed(range(0, steps+1)):
    #    pybullet.setJointMotorControl2(robot, jointNumber, pybullet.POSITION_CONTROL, posJoint/steps)
    #    pybullet.stepSimulation()
    #    time.sleep(fixedTimeStep)
