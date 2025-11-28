import time
import yaml
import sys
import os
import numpy as np

from ..dynamixel_sdk import PortHandler, PacketHandler
from ..motor import Motor

def load_config(config_file):
    with open(config_file, "r") as file:
        return yaml.safe_load(file)

abs_path = os.path.abspath(__file__)

ex12_config_file = os.path.join(
            os.path.dirname(abs_path), "config.yaml"
        )
ex12_configs = load_config(ex12_config_file)

PROTOCOL_VERSION = ex12_configs["BASIC"]["PROTOCOL_VERSION"]
BAUDRATE = ex12_configs["BASIC"]["BAUDRATE"]
NAME = ex12_configs["HAND"]["NAME"]
NUM_MOTORS = ex12_configs["HAND"]["NUM"] # 电机数量
THUMB_IDS = ex12_configs["HAND"]["THUMB_IDS"] # 大拇指ID
INDEX_IDS = ex12_configs["HAND"]["INDEX_IDS"] # 食指ID
MID_IDS = ex12_configs["HAND"]["MID_IDS"] # 中指ID


class Glove:
    def __init__(self, port) -> None:
        self.is_connected = False
        self.port = port
        self.name = NAME
        self.kin = None 

    def connect(self):
        """
        连接Glove,目前版本不使能，只获取角度
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

        self.motors = [Motor(i+1, portHandler, packetHandler) for i in range(NUM_MOTORS)]
        
        print(f'{self.name} connect done...')
        
        init_js = [m.get_pos() for m in self.motors] 
        
        self.init_offsets = [0 if j<270 else 360 for j in init_js] # 初始上电会出现大于360度的角度，所以构建offset
        
        print(init_js)
        print(self.init_offsets)

    def off(self):
        """
        失能所有电机
        """
        for m in self.motors:
            m.torq_off()
    
    def getj(self):
        """
        获取EX12关节角度，单位度
        """
        # 固定电机舵盘安装位置，初始角度90，因此减去90，然后减去初始上电的offset（如果有大于360度的情况）
        js = [m.get_pos() - 90 - o for m, o in zip(self.motors, self.init_offsets)] 
        
        return js