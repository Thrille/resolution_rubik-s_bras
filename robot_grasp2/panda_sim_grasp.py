import time
import numpy as np
import math

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11  # 8
pandaNumDofs = 7

ll = [-7] * pandaNumDofs
# upper limits for null space (todo: set them to proper range)
ul = [7] * pandaNumDofs
# joint ranges for null space (todo: set them to proper range)
jr = [7] * pandaNumDofs
# restposes for null space
jointPositions = [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
rp = jointPositions


class PandaSim(object):
    def __init__(self, bullet_client, offset):
        self.bullet_client = bullet_client
        self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
        self.offset = np.array(offset)

        # print("offset=",offset)
        flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES

        self.plane = self.bullet_client.loadURDF("plane.urdf", [0 + offset[0], 0 + offset[1], 0 + offset[2]],
                                                 self.bullet_client.getQuaternionFromEuler([-math.pi/2, 0, 0]),
                                                 useFixedBase=True,
                                                 flags=flags)
        #self.bullet_client.loadURDF("tray/traybox.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]],
        #                            [-0.5, -0.5, -0.5, 0.5], flags=flags)
        #self.cube = self.bullet_client.loadURDF("cube_small.urdf", np.array([0.05, 0.4, -0.7-0.025]) + self.offset, flags=flags)
        self.socle = self.bullet_client.loadURDF("SocleRubiks.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]],self.bullet_client.getQuaternionFromEuler([-math.pi/2, 0, 0]), flags=flags, useFixedBase=True)
        #self.socle = self.bullet_client.loadSDF("SocleRubiks.sdf")
        #self.bullet_client.resetBasePositionAndOrientation(self.socle[0], [0, 0, 0], self.bullet_client.getQuaternionFromEuler([0, 0, 0]))
        #self.noyau = self.bullet_client.loadURDF("noyau.urdf", np.array([0, 0.5, -0.7]) + self.offset, flags=flags)
        print("q=",self.bullet_client.getQuaternionFromEuler([-math.pi/4, 0, 0]))
        self.rubiks = self.bullet_client.loadURDF("../cubeV2.urdf",
        #                                          [0 + offset[0] + 0.00001, 0.028575 + offset[1] + 0.02 + 0.03, - 0.6 + offset[2]],
                                                  [0.045 + offset[0], 0.1 + offset[1], - 0.635 + offset[2]],self.bullet_client.getQuaternionFromEuler([73,0,0]),
                                                  flags=flags)
        #self.noyau = self.bullet_client.loadURDF("noyau.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]], useFixedBase=True, flags=flags)
        #self.centre01 = self.bullet_client.loadURDF("centre01.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]], useFixedBase=True, flags=flags)
        #self.centre02 = self.bullet_client.loadURDF("centre02.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]], useFixedBase=True, flags=flags)
        #self.centre03 = self.bullet_client.loadURDF("centre03.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]],
        #                                         useFixedBase=True, flags=flags)
        #self.centre04 = self.bullet_client.loadURDF("centre04.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]],
        #                                         useFixedBase=True, flags=flags)
        #self.centre05 = self.bullet_client.loadURDF("centre05.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]],
        #                                         useFixedBase=True, flags=flags)
        #self.centre06 = self.bullet_client.loadURDF("centre06.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]],
        #                                         useFixedBase=True, flags=flags)

        orn = [-0.707107, 0.0, 0.0, 0.707107]  # p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
        eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])
        self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0, 0, 0]) + self.offset, orn,
                                                 useFixedBase=True, flags=flags)
        #revolution of hand by joint between link6 and link7
        index = 0
        self.state = 0
        self.control_dt = 1. / 240.
        self.finger_target = 0
        self.gripper_height = 0.2
        self.target_pos = []
        # create a constraint to keep the fingers centered
        c = self.bullet_client.createConstraint(self.panda,
                                                9,
                                                self.panda,
                                                10,
                                                jointType=self.bullet_client.JOINT_GEAR,
                                                jointAxis=[1, 0, 0],
                                                parentFramePosition=[0, 0, 0],
                                                childFramePosition=[0, 0, 0])
        self.bullet_client.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

        for j in range(self.bullet_client.getNumJoints(self.panda)):
            self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
            info = self.bullet_client.getJointInfo(self.panda, j)
            # print("info=",info)
            jointName = info[1]
            jointType = info[2]
            if (jointType == self.bullet_client.JOINT_PRISMATIC):
                self.bullet_client.resetJointState(self.panda, j, jointPositions[index])
                index = index + 1
            if (jointType == self.bullet_client.JOINT_REVOLUTE):
                self.bullet_client.resetJointState(self.panda, j, jointPositions[index])
                index = index + 1
        self.t = 0.
        rubiks = self.bullet_client.getBasePositionAndOrientation(self.rubiks)
        self.target_pos = [rubiks[0][0], rubiks[0][1], rubiks[0][2]]
        self.steps = 400
        self.fixedTimeStep = 1.5 / self.steps
        print(rubiks)
        print(rubiks[0])
        print(rubiks[1])
        print(rubiks[0][0])
        print(rubiks[0][1])
        print(rubiks[0][2])
        print(self.target_pos)
        #print(rubiks(0))
        #print(rubiks(1))

    def reset(self):
        pass

    def update_state(self):
        keys = self.bullet_client.getKeyboardEvents()
        if len(keys) > 0:
            for k, v in keys.items():
                if v & self.bullet_client.KEY_WAS_TRIGGERED:
                    if (k == 38):   # key 1
                        self.state = 5  # Open grip
                    if (k == 233):  # key 2
                        self.state = 6  # Close grip
                    if (k == 34):   # key 3
                        self.state = 3  # Goto target up
                    if (k == 39):   # key 4
                        self.state = 4  # Goto target down
                    if (k == 40):   # key 5
                        self.state = 2  # Circle up
                    if (k == 45):   # key 6
                        self.state = 1  # Circle down
                if v & self.bullet_client.KEY_WAS_RELEASED:
                    self.state = 0
        self.state = 20
        #print("state=", self.state)

    def test(self):
        for jointNumber in range(self.bullet_client.getNumJoints(self.panda)):
            for posJoint in range(0, self.steps+1):
                self.bullet_client.setJointMotorControl2(self.panda, jointNumber, self.bullet_client.POSITION_CONTROL, -posJoint/self.steps)
                self.bullet_client.stepSimulation()
                time.sleep(self.fixedTimeStep)
            for posJoint in reversed(range(0, self.steps+1)):
                self.bullet_client.setJointMotorControl2(self.panda, jointNumber, self.bullet_client.POSITION_CONTROL, -posJoint/self.steps)
                self.bullet_client.stepSimulation()
                time.sleep(self.fixedTimeStep)
            for posJoint in range(0, self.steps+1):
                self.bullet_client.setJointMotorControl2(self.panda, jointNumber, self.bullet_client.POSITION_CONTROL, posJoint/self.steps)
                self.bullet_client.stepSimulation()
                time.sleep(self.fixedTimeStep)
            for posJoint in reversed(range(0, self.steps+1)):
                self.bullet_client.setJointMotorControl2(self.panda, jointNumber, self.bullet_client.POSITION_CONTROL, posJoint/self.steps)
                self.bullet_client.stepSimulation()
                time.sleep(self.fixedTimeStep)

    def step(self):

        print("test1")
        if self.state == 6:
            self.finger_target = 0.005
        if self.state == 5:
            self.finger_target = 0.04
        self.bullet_client.submitProfileTiming("step")
        #print("self.state=",self.state)
        #print("self.finger_target=",self.finger_target)
        alpha = 0.9  # 0.99
        self.update_state()
        print("test2")
        print(self.state)
        if self.state == 0:
            panda = self.bullet_client.getBasePositionAndOrientation(self.rubiks)
            posb = [panda[0][0], panda[0][1], panda[0][2]]
            #self.prev_pos = posb
        if self.state == 1 or self.state == 2 or self.state == 3 or self.state == 4 or self.state == 7 or self.state == 8 or self.state == 9 or self.state == 10 or self.state == 11 or self.state == 12:
            # gripper_height = 0.034
            self.gripper_height = alpha * self.gripper_height + (1. - alpha) * 0.03
            if self.state == 2 or self.state == 3 or self.state == 7:
                self.gripper_height = alpha * self.gripper_height + (1. - alpha) * 0.2

            t = self.t
            self.t += self.control_dt
            #pos = [self.offset[0] + 0.2 * math.sin(1.5 * t), self.offset[1] + self.gripper_height,
                   #self.offset[2] + -0.6 + 0.1 * math.cos(1.5 * t)]
            #orn = self.bullet_client.getQuaternionFromEuler([3 * math.pi / 4 , 0., 0.])
            if self.state == 3 or self.state == 4:
                pos, o = self.bullet_client.getBasePositionAndOrientation(self.rubiks)
                pos = [pos[0], self.gripper_height, pos[2]]
                self.prev_pos = pos
            if self.state == 7:
                pos = self.prev_pos
                diffX = pos[0] - self.offset[0]
                diffZ = pos[2] - (self.offset[2] - 0.6)
                self.prev_pos = [self.prev_pos[0] - diffX * 0.1, self.prev_pos[1], self.prev_pos[2] - diffZ * 0.1]
            #première position
            if self.state == 8:
                print("test3")
                rubiks = self.bullet_client.getBasePositionAndOrientation(self.panda)
                print(rubiks)
                ora = self.bullet_client.getEulerFromQuaternion(rubiks[1])
                print(ora)
                print(self.prev_pos)
                pos = self.prev_pos
                orn = self.prev_orn
            #baisse à la hauteur du cube
            if self.state == 9:
                print("test9")
                pos = [self.prev_pos[0], self.prev_pos[1] - 0.13/240, self.prev_pos[2]]
                orn = self.prev_orn
                self.prev_pos = pos
            #remonte
            if self.state == 10:
                print("test10")
                pos = [self.prev_pos[0], self.prev_pos[1] + 0.13/240, self.prev_pos[2]]
                orn = self.prev_orn
                self.prev_pos = pos
            #met la pince à l'horizontale
            if self.state == 11:
                print("test11")
                pos = self.prev_pos
                orn = [self.prev_orn[0], self.prev_orn[1], self.prev_orn[2]]
            #    self.prev_orn = orn
            #    self.nouv_orn_pince = 1
            #à l'horizontale dans l'autre sens
            if self.state == 12:
                print("test12")
                pos = self.prev_pos
                orn = [self.prev_orn[0], self.prev_orn[1], self.prev_orn[2]]
            #    self.prev_orn = orn
            #    self.nouv_orn_pince = 0

            self.bullet_client.submitProfileTiming("IK")
            if self.state == 9 or self.state == 10:
                print(pos)
                jointPoses = self.bullet_client.calculateInverseKinematics(bodyUniqueId=self.panda,
                                                                            endEffectorLinkIndex=pandaEndEffectorIndex,
                                                                            targetPosition=pos,
                                                                            lowerLimits=ll,
                                                                            upperLimits=ul,
                                                                            jointRanges=jr,
                                                                            restPoses=rp,
                                                                            maxNumIterations=20)
                self.prev_pos = pos
            else:
                jointPoses = self.bullet_client.calculateInverseKinematics(self.panda, pandaEndEffectorIndex, pos, orn, ll,
                                                                            ul, jr, rp, maxNumIterations=20)

            self.bullet_client.submitProfileTiming()
            for i in range(pandaNumDofs):
                if i == 6:
                    if self.state == 11:
                        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, math.pi / 4, force=10)
                        pan = self.bullet_client.getBasePositionAndOrientation(self.panda)
                        print(pan)
                        ori = self.bullet_client.getEulerFromQuaternion(pan[1])
                        print(ori)
                    elif self.state == 12:
                        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, 3 * math.pi / 4, force=5 * 240.)
                else:
                    self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=5 * 240.)
            # target for fingers
        for i in [9, 10]:
            self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,
                                                     self.finger_target, force=1000)
        self.bullet_client.submitProfileTiming()


class PandaSimAuto(PandaSim):
    def __init__(self, bullet_client, offset):
        PandaSim.__init__(self, bullet_client, offset)

        self.state_t = 0
        self.cur_state = 0
        rubiks = self.bullet_client.getBasePositionAndOrientation(self.rubiks)

        self.prev_pos = [rubiks[0][0], rubiks[0][1] + 0.1, rubiks[0][2] + 0.01]
        self.prev_orn = self.bullet_client.getQuaternionFromEuler([3* math.pi/4 , 0, 0])
        #self.prev_orn_pince = 0 #0 pour verticale
        #self.nouv_orn_pince = 1 #1 pour horizontale
        #self.states = [0, 3, 5, 4, 6, 3, 7]
        #self.state_durations = [1, 1, 1, 2, 1, 1, 2]

        #self.states = [0, 8, 5, 9, 6, 10, 12, 9, 5, 10]
        #self.state_durations = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

        self.states = [0, 8, 11, 5, 9, 6, 10, 12, 9, 5, 10, 11]
        self.state_durations = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

    def update_state(self):
        self.state_t += self.control_dt
        if self.state_t > self.state_durations[self.cur_state]:
            self.cur_state += 1
            if self.cur_state >= len(self.states):
                self.cur_state = 0
            self.state_t = 0
            self.t = 0
            self.state = self.states[self.cur_state]
            # print("self.state=",self.state)
