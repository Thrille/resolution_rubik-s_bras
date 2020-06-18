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
        self.socle = self.bullet_client.loadURDF("SocleRubiks.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]], flags=flags)
        #self.socle = self.bullet_client.loadSDF("SocleRubiks.sdf")
        #self.bullet_client.resetBasePositionAndOrientation(self.socle[0], [0, 0, 0], self.bullet_client.getQuaternionFromEuler([0, 0, 0]))
        #self.noyau = self.bullet_client.loadURDF("noyau.urdf", np.array([0, 0.5, -0.7]) + self.offset, flags=flags)
        print("q=",self.bullet_client.getQuaternionFromEuler([-math.pi/4, 0, 0]))
        self.rubiks = self.bullet_client.loadURDF("centres.urdf",
        #                                          [0 + offset[0] + 0.00001, 0.028575 + offset[1] + 0.02 + 0.03, - 0.6 + offset[2]],
                                                  [0 + offset[0], 0 + offset[1], - 0.6 + offset[2]],
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
        #print("state=", self.state)

    def step(self):
        if self.state == 6:
            self.finger_target = 0.01
        if self.state == 5:
            self.finger_target = 0.04
        self.bullet_client.submitProfileTiming("step")
        self.update_state()
        #print("self.state=",self.state)
        #print("self.finger_target=",self.finger_target)
        alpha = 0.9  # 0.99
        if self.state == 1 or self.state == 2 or self.state == 3 or self.state == 4 or self.state == 7:
            # gripper_height = 0.034
            self.gripper_height = alpha * self.gripper_height + (1. - alpha) * 0.03
            if self.state == 2 or self.state == 3 or self.state == 7:
                self.gripper_height = alpha * self.gripper_height + (1. - alpha) * 0.2

            t = self.t
            self.t += self.control_dt
            pos = [self.offset[0] + 0.2 * math.sin(1.5 * t), self.offset[1] + self.gripper_height,
                   self.offset[2] + -0.6 + 0.1 * math.cos(1.5 * t)]
            if self.state == 3 or self.state == 4:
                pos, o = self.bullet_client.getBasePositionAndOrientation(self.cube)
                pos = [pos[0], self.gripper_height, pos[2]]
                self.prev_pos = pos
            if self.state == 7:
                pos = self.prev_pos
                diffX = pos[0] - self.offset[0]
                diffZ = pos[2] - (self.offset[2] - 0.6)
                self.prev_pos = [self.prev_pos[0] - diffX * 0.1, self.prev_pos[1], self.prev_pos[2] - diffZ * 0.1]

            orn = self.bullet_client.getQuaternionFromEuler([math.pi / 2., 0., 0.])
            self.bullet_client.submitProfileTiming("IK")
            jointPoses = self.bullet_client.calculateInverseKinematics(self.panda, pandaEndEffectorIndex, pos, orn, ll,
                                                                       ul,
                                                                       jr, rp, maxNumIterations=20)
            self.bullet_client.submitProfileTiming()
            for i in range(pandaNumDofs):
                self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,
                                                         jointPoses[i], force=5 * 240.)
            # target for fingers
        for i in [9, 10]:
            self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,
                                                     self.finger_target, force=10)
        self.bullet_client.submitProfileTiming()


class PandaSimAuto(PandaSim):
    def __init__(self, bullet_client, offset):
        PandaSim.__init__(self, bullet_client, offset)
        self.state_t = 0
        self.cur_state = 0
        self.states = [0, 3, 5, 4, 6, 3, 7]
        self.state_durations = [1, 1, 1, 2, 1, 1, 2]
'''
    def update_state(self):
        self.state_t += self.control_dt
        if self.state_t > self.state_durations[self.cur_state]:
            self.cur_state += 1
            if self.cur_state >= len(self.states):
                self.cur_state = 0
            self.state_t = 0
            self.state = self.states[self.cur_state]
            # print("self.state=",self.state)
'''
