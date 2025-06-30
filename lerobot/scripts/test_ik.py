import time
import math
import numpy as np


def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    """
    Calculate inverse kinematics for a 2-link robotic arm, considering joint offsets

    Parameters:
        x: End effector x coordinate
        y: End effector y coordinate
        l1: Upper arm length (default 0.1159 m)
        l2: Lower arm length (default 0.1350 m)

    Returns:
        joint2, joint3: Joint angles in radians as defined in the URDF file
    """
    # Calculate joint2 and joint3 offsets in theta1 and theta2
    theta1_offset = -math.atan2(0.028, 0.11257)  # theta1 offset when joint2=0
    theta2_offset = -math.atan2(0.0052, 0.1349) + theta1_offset  # theta2 offset when joint3=0

    # Calculate distance from origin to target point
    r = math.sqrt(x**2 + y**2)
    r_max = l1 + l2  # Maximum reachable distance

    # If target point is beyond maximum workspace, scale it to the boundary
    if r > r_max:
        scale_factor = r_max / r
        x *= scale_factor
        y *= scale_factor
        r = r_max

    # If target point is less than minimum workspace (|l1-l2|), scale it
    r_min = abs(l1 - l2)
    if r < r_min and r > 0:
        scale_factor = r_min / r
        x *= scale_factor
        y *= scale_factor
        r = r_min

    # Use law of cosines to calculate theta2
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)

    # Calculate theta2 (elbow angle)
    theta2 = math.pi - math.acos(cos_theta2)

    # Calculate theta1 (shoulder angle)
    beta = math.atan2(y, x)
    gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    theta1 = beta + gamma

    # Convert theta1 and theta2 to joint2 and joint3 angles
    joint2 = theta1 - theta1_offset
    joint3 = theta2 - theta2_offset

    # Ensure angles are within URDF limits
    joint2 = max(-0.1, min(3.45, joint2))
    joint3 = max(-0.2, min(math.pi, joint3))

    return joint2, joint3


def ik_so100(pos, pitch, roll):
    z, y, x = pos
    joints = np.zeros(6)

    joints[0] = np.arctan2(z, x)
    if x < 0:
        print("warning: the ik result may not be reachable")
    flag = 1 if x >= 0 else -1
    joints[1], joints[2] = inverse_kinematics(np.sqrt(x ** 2 + z ** 2) * flag, y)
    joints[3] = joints[1] - joints[2] + pitch
    joints[4] = roll
    joints[5] = np.pi / 4

    joints /= np.pi
    joints *= 180
    # joints[0] = 90 - joints[0]
    # joints[3] = 90 - joints[3]
    # joints[1] = 180 - joints[1]
    # joints[3] += 180
    return joints



if __name__ == "__main__":
    print(ik_so100(pos=[-0, 0.15, 0.15], rot=None).tolist())
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
