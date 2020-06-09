#!/usr/bin/env python3

#  Starting the simulation
import pybullet
# pour pouvoir utiliser les délias (time.sleep)
import time

import pybullet_data

pybullet.connect(pybullet.GUI)

# pour remettre à zéro la simulation
pybullet.resetSimulation()

pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# plane est le sol il est contenu dans l'import pybullet_data
pybuplane = pybullet.loadURDF("plane.urdf")

# importation du socle
socle = pybullet.loadURDF("cube.urdf", [0, 0, 0], useFixedBase=1)

time.sleep(2000)
