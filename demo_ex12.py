import time
import pybullet as p
import pybullet_data
from libgex import Glove
import numpy as np

# 连接到PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 用于查找常用模型

# 加载地面
p.loadURDF("plane.urdf")

# 加载你的机械手 URDF（修改为你的文件路径）
# 假设 URDF 放在当前目录，且 joint 顺序和 glove 对应
hand = p.loadURDF("libgex/ex12/urdf/ex12.urdf", basePosition=[0,0,0.2], baseOrientation=p.getQuaternionFromEuler([np.pi/2, 0, np.pi/2]), useFixedBase=True)

cam_pos = [-0.71, 0.16, -0.13]
pitch_deg = -26.60
yaw_deg = 79.60

target_pos = cam_pos
distance = 1.12

# 计算相机位置（基于目标位置、距离和角度）
cam_distance = distance
cam_yaw = yaw_deg
cam_pitch = pitch_deg

# 设置相机
p.resetDebugVisualizerCamera(
    cameraDistance=cam_distance,
    cameraYaw=cam_yaw,
    cameraPitch=cam_pitch,
    cameraTargetPosition=target_pos
)

# 初始化手套
glove = Glove('COM11')
glove.connect()

# 获取模型中的可动关节（revolute/prismatic）
num_joints = p.getNumJoints(hand)
revolute_joint_indices = []

for i in range(num_joints):
    joint_info = p.getJointInfo(hand, i)
    joint_type = joint_info[2]  # 关节类型
    # 筛选可旋转关节 (p.JOINT_REVOLUTE) 或可移动关节 (p.JOINT_PRISMATIC)
    if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
        revolute_joint_indices.append(i)
        print(f"可动关节 {i}: {joint_info[1]}")  # 打印关节名称

print(f"找到 {len(revolute_joint_indices)} 个可动关节")

# 检查关节数量是否匹配
if len(revolute_joint_indices) != 12:
    print(f"警告: URDF有{len(revolute_joint_indices)}个可动关节，但手套提供12个数据，可能不匹配")

# 设置仿真参数
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)  # 开启实时模式

print("开始实时同步手套数据到 PyBullet... (Ctrl+C 退出)")

def normalize_angle(angle_rad):
    """
    将角度归一化到 [-π, π] 范围内
    """
    if angle_rad > np.pi * 2:
        print('outlier:', angle_rad)
        angle_rad = angle_rad - np.pi*2
        
    return angle_rad

try:
    while True:
        # 从手套获取12个关节角度（单位：度）
        qs = glove.getj()  # 返回长度为12的列表
        if qs and len(qs) >= 12:
            # 将角度转为弧度
            qs_rad = [q * np.pi / 180.0 for q in qs]

        
            
            # 将手套数据映射到模型关节
            # 只控制可旋转关节
            for i in range(min(len(qs_rad), len(revolute_joint_indices))):
                p.setJointMotorControl2(
                    hand,
                    revolute_joint_indices[i],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=qs_rad[i],
                    force=5
                )

        time.sleep(0.01)  # 控制刷新频率

except KeyboardInterrupt:
    print("退出程序")
finally:
    p.disconnect()