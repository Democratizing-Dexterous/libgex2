import time
import yaml
import sys
import os
import numpy as np

from ..dynamixel_sdk import PortHandler, PacketHandler
from ..motor import Motor

# from .gx10 import kinematics

def load_config(config_file):
    with open(config_file, "r") as file:
        return yaml.safe_load(file)

abs_path = os.path.abspath(__file__)

gx10_config_file = os.path.join(
            os.path.dirname(abs_path), "config.yaml"
        )
gx10_configs = load_config(gx10_config_file)

PROTOCOL_VERSION = gx10_configs["BASIC"]["PROTOCOL_VERSION"]
BAUDRATE = gx10_configs["BASIC"]["BAUDRATE"]
NAME = gx10_configs["HAND"]["NAME"]
NUM_MOTORS = gx10_configs["HAND"]["NUM"] # 电机数量
THUMB_IDS = gx10_configs["HAND"]["THUMB_IDS"] # 大拇指ID
INDEX_IDS = gx10_configs["HAND"]["INDEX_IDS"] # 食指ID
MID_IDS = gx10_configs["HAND"]["MID_IDS"] # 中指ID

POSKP = gx10_configs["ExtendedPos"]["Motor1"]['Pos_kp'] 
POSKI = gx10_configs["ExtendedPos"]["Motor1"]['Pos_ki'] 
POSKD = gx10_configs["ExtendedPos"]["Motor1"]['Pos_kd'] 
PROVEL = gx10_configs["ExtendedPos"]["Motor1"]['Profile_vel'] 
PROACC = gx10_configs["ExtendedPos"]["Motor1"]['Profile_acc'] 

class Hand:
    def __init__(self, port) -> None:
        self.is_connected = False
        self.port = port
        self.name = NAME
        self.kin = None 

    def connect(self, curr_limit=1000):
        """
        连接Hand，并且使能每个电机为默认的力控位置模式, 
        curr_limit为电机最大限流(mA，最大不超过1750)，goal_current（要小于curr_limit）限制控制过程中最大力矩，goal_pwm限制运动速度（最大不超过885）
        """

        portHandler = PortHandler(self.port)
        packetHandler = PacketHandler(PROTOCOL_VERSION)

        if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
            print(f'Open {self.port} Success...')
            self.is_connected = True
        else:
            print(f'Failed...')
            self.is_connected = False
            sys.exit(0)
        
        self.portHandler = portHandler
        self.packetHandler = packetHandler

        self.motors = [Motor(i+1, portHandler, packetHandler, curr_limit) for i in range(NUM_MOTORS)]

        for m in self.motors:
            m.init_config(curr_limit=curr_limit)
            m.set_profile(PROACC, PROVEL)
            m.set_pos_pid(POSKP, POSKD, POSKI)
        
        print(f'{self.name} init done...')

    def off(self):
        """
        失能所有电机
        """
        for m in self.motors:
            m.torq_off()
    
    def on(self):
        """
        使能所有电机
        """
        for m in self.motors:
            m.torq_on()

    def set_zero_whole_hand(self):
        """
        GX10手掌设置编码器以当前位置归0，慎用，归0前先off
        """
        for m in self.motors:
            m.set_zero()


    def home(self):
        """
        GX10会到原点
        """
        motors = self.motors
        for m in motors:
            m.set_pos(0)
        time.sleep(1)

    def getj(self):
        """
        获取GX10关节角度，单位度
        """
        js = [m.get_pos() for m in self.motors]
        return js
    
    def setj(self, js):
        """
        设置GX10关节角度，单位度
        """
        for m, j in zip(self.motors, js):
            m.set_pos(j)
    
    # def getj_finger1(self):
    #     """
    #     获取GX10指1关节角度，单位度
    #     """
    #     js = [m.get_pos() for m in self.motors[0:3]]
    #     return js
    
    # def setj_finger1(self, js):
    #     """
    #     设置GX10指1关节角度，单位度
    #     """
    #     for m, j in zip(self.motors[0:3], js):
    #         m.set_pos(j)

    # def getj_finger2(self):
    #     """
    #     获取GX10指2关节角度，单位度
    #     """
    #     js = [m.get_pos() for m in self.motors[3:7]]
    #     return js
    
    # def setj_finger2(self, js):
    #     """
    #     设置GX10指2关节角度，单位度
    #     """
    #     for m, j in zip(self.motors[3:7], js):
    #         m.set_pos(j)

    # def getj_finger3(self):
    #     """
    #     获取GX10指3关节角度，单位度
    #     """
    #     js = [m.get_pos() for m in self.motors[7:11]]
    #     return js
    
    # def setj_finger3(self, js):
    #     """
    #     设置GX10指3关节角度，单位度
    #     """
    #     for m, j in zip(self.motors[7:11], js):
    #         m.set_pos(j)
    
    
    

    # def fk_finger1(self):
    #     """获取finger1的正运动学，单位m"""
    #     xyz = self.kin.fk_finger1(self.getj_finger1())

    #     return xyz
    
    # def ik_finger1(self, xyz):
    #     """获取finger1的逆运动学，单位m"""
    #     q = self.kin.ik_finger1(xyz)

    #     return q


    # def fk_finger2(self):
    #     """获取finger2的正运动学，单位m"""
    #     xyz = self.kin.fk_finger2(self.getj_finger2())

    #     return xyz
    
    # def ik_finger2(self, xyz):
    #     """获取finger2的逆运动学，单位m"""
    #     q = self.kin.ik_finger2(xyz)

    #     return q
    
    # def fk_finger3(self):
    #     """获取finger3的正运动学，单位m"""
    #     xyz = self.kin.fk_finger3(self.getj_finger3())

    #     return xyz
    
    # def ik_finger3(self, xyz):
    #     """获取finger3的逆运动学，单位m"""
    #     q = self.kin.ik_finger3(xyz)

    #     return q