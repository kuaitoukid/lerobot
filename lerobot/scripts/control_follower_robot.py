import logging
import time
import math
import numpy as np
from dataclasses import asdict
from pprint import pformat
import torch

from lerobot.common.robot_devices.control_configs import ControlPipelineConfig
from lerobot.common.robot_devices.robots.utils import make_robot_from_config
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot, ensure_safe_goal_position
from lerobot.common.robot_devices.utils import busy_wait, safe_disconnect
from lerobot.common.utils.utils import init_logging
from lerobot.configs import parser
from lerobot.scripts.test_ik import ik_so100


def generate_waypoints(corners, num_points_per_segment=100):
    """
    将3D点数组corners中的相邻点连线，使用linspace生成连续点
    
    参数:
    - corners: 形状为(n, 3)的numpy数组，表示n个3D点
    - num_points_per_segment: 每两个相邻点之间生成的连续点数量
    
    返回:
    - continuous_points: 形状为(m, 3)的numpy数组，表示生成的连续点
    """
    # 检查输入是否有效
    if corners.ndim != 2:
        raise ValueError("输入corners必须是形状为(n, 3)的numpy数组")
    
    num_corners = corners.shape[0]
    if num_corners < 2:
        return corners.copy()
    
    # 预分配结果数组
    total_points = (num_corners - 1) * num_points_per_segment + 1
    continuous_points = np.zeros((total_points, corners.shape[1]))
    
    # 生成连续点
    for i in range(num_corners - 1):
        start_point = corners[i]
        end_point = corners[i + 1]
        
        # 在相邻两点之间使用linspace生成连续点
        segment_points = np.linspace(start_point, end_point, 
                                    num_points_per_segment, 
                                    endpoint=False)
        
        # 将生成的点添加到结果数组中
        continuous_points[i * num_points_per_segment : (i + 1) * num_points_per_segment] = segment_points
    
    # 添加最后一个角点
    continuous_points[-1] = corners[-1]
    
    return continuous_points


safe_range = [[-90, 0, 0, -90, -90, 0], [90, 180, 180, 90, 90, 90]]

@safe_disconnect
def custom_control(robot: ManipulatorRobot):
    fps = 1
    num_frames = 2
    if not robot.is_connected:
        robot.connect()

    # if robot.is_connected:
    #     robot.disconnect()

    last_init = None
    warmup = 2
    goal_pos = None
    init_point = [0, 0.15, 0.15]
    init_joints = ik_so100(pos=init_point, rot=None)
    corners = np.array([
        [0, 0.15, 0.15],
        [-0.05, 0.1, 0.15],
        [0.05, 0.1, 0.15],
        [0.05, 0.2, 0.15],
        [-0.05, 0.2, 0.15],
        [-0.05, 0.1, 0.15],
        [0, 0.15, 0.15]
    ])
    waypoints = generate_waypoints(corners, num_points_per_segment=1)
    wayjoints = np.array([ik_so100(e, rot=None) for e in waypoints])
    wayjoints = generate_waypoints(wayjoints, num_points_per_segment=1)

    # warm up
    for name in robot.follower_arms:
        start_joints = torch.from_numpy(robot.follower_arms[name].read("Present_Position"))
        end_joints = init_joints.copy()
        warmup_joints = generate_waypoints(np.stack((start_joints, end_joints), axis=0), num_points_per_segment=warmup)
        for joints in warmup_joints:
            start_episode_t = time.perf_counter()
            # joints = np.clip(joints, safe_range[0], safe_range[1])
            robot.follower_arms[name].write("Goal_Position", joints.astype(np.float32))
            dt_s = time.perf_counter() - start_episode_t
            busy_wait(1 / fps - dt_s)

    # start moving
    for name in robot.follower_arms:
        for joints in wayjoints:
            print(joints)
            start_episode_t = time.perf_counter()
            # joints = np.clip(joints, safe_range[0], safe_range[1])
            robot.follower_arms[name].write("Goal_Position", joints.astype(np.float32))
            dt_s = time.perf_counter() - start_episode_t
            busy_wait(1 / fps - dt_s)


@parser.wrap()
def control_robot(cfg: ControlPipelineConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.robot.type not in ["so100_follwer", "so101_follower"]:
        return
    robot = make_robot_from_config(cfg.robot)
    if not isinstance(robot, ManipulatorRobot):
        robot.disconnect()

    custom_control(robot)

    if robot.is_connected:
        # Disconnect manually to avoid a "Core dump" during process
        # termination due to camera threads not properly exiting.
        robot.disconnect()


if __name__ == "__main__":
    # init_position = [0.1, 0.1, 0.15]
    # x, y, z = init_position
    # joints = np.zeros(6)
    # joints[0] = np.arctan(y / x)
    # joints[1], joints[2] = inverse_kinematics(y, -z)
    # pitch = 0
    # joints[3] = joints[1] - joints[2] + pitch
    # joints[4] = 0
    # joints[5] = 0
    # joints /= np.pi
    # joints *= 180
    # print(joints)
    # exit()
    control_robot()
