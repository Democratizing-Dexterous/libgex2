import pybullet as p
import os.path as osp
import numpy as np


abs_path = osp.dirname(osp.abspath(__file__))

class KinEX12:
    def __init__(self, vis=False) -> None:
        self.name = 'EX12'
        if vis:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        self.offset_z = 1

        self.bullet_hand = p.loadURDF(osp.join(abs_path, "urdf/ex12.urdf"), useFixedBase=True, basePosition=[0, 0, self.offset_z])
        for i in range(20):
            p.stepSimulation()

        self.thumb_link_id = 4
        self.thumb_joint_ids = [0, 1, 2, 3]

        self.index_link_id = 9
        self.index_joint_ids = [5, 6, 7, 8]

        self.middle_link_id = 14
        self.middle_joint_ids = [10, 11, 12, 13]


    def fk_finger1(self, q=[0]*4):
        """
        finger1 正运动学，3自由度
        """
        q = [q_*np.pi/180 for q_ in q]


        for i, joint_position in zip(self.thumb_joint_ids, q):
            p.setJointMotorControl2(self.bullet_hand, i, p.POSITION_CONTROL, joint_position)
        
        for i in range(120):
            p.stepSimulation()

        ee_pos = p.getLinkState(self.bullet_hand, self.thumb_link_id, computeForwardKinematics=1)[4]

        ee_pos = list(ee_pos)
        ee_pos[2] -= self.offset_z

        return ee_pos


    def fk_finger2(self, q=[0]*4):
        """
        finger2 正运动学，4自由度
        """
        q = [q_*np.pi/180 for q_ in q]

        for i, joint_position in zip(self.index_joint_ids, q):
            p.setJointMotorControl2(self.bullet_hand, i, p.POSITION_CONTROL, joint_position)
        
        for i in range(120):
            p.stepSimulation()

        ee_pos = p.getLinkState(self.bullet_hand, self.index_link_id, computeForwardKinematics=1)[4]

        ee_pos = list(ee_pos)
        ee_pos[2] -= self.offset_z

        return ee_pos

    
    def fk_finger3(self, q=[0]*4):
        """
        finger3 正运动学，4自由度
        """
        q = [q_*np.pi/180 for q_ in q]


        for i, joint_position in zip(self.middle_joint_ids, q):
            p.setJointMotorControl2(self.bullet_hand, i, p.POSITION_CONTROL, joint_position)
        
        for i in range(120):
            p.stepSimulation()

        ee_pos = p.getLinkState(self.bullet_hand, self.middle_link_id, computeForwardKinematics=1)[4]

        ee_pos = list(ee_pos)
        ee_pos[2] -= self.offset_z

        return ee_pos
    

