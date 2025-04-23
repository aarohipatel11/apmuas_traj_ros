#!/usr/bin/env python3

import rclpy
import math
import numpy as np
import mavros
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from nav_msgs.msg import Odometry
from drone_interfaces.msg import Telem, CtlTraj
from apmuas_traj_ros import rotation_utils as rot_utils
from re import S
from typing import List
from mavros.base import SENSOR_QOS

"""
For this application we will be sending roll, pitch yaw commands to the drone
"""


def yaw_enu_to_ned(yaw_enu:float)-> float:
    """
    Convert yaw angle from ENU to NED.
    
    The conversion is symmetric:
        yaw_ned = (pi/2 - yaw_enu) wrapped to [-pi, pi]

    Parameters:
        yaw_enu (float): Yaw angle in radians in the ENU frame.
        
    Returns:
        float: Yaw angle in radians in the NED frame.
    """
    yaw_ned = np.pi/2 - yaw_enu
    return wrap_to_pi(yaw_ned)

def wrap_to_pi(angle:float) -> float:
    """
    Wrap an angle in radians to the range [-pi, pi].

    Parameters:
        angle (float): Angle in radians.
    
    Returns:
        float: Angle wrapped to [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def get_relative_ned_yaw_cmd(
        current_ned_yaw:float, 
        inert_ned_yaw_cmd:float) -> float:

    yaw_cmd:float = inert_ned_yaw_cmd - current_ned_yaw
        
        # wrap the angle to [-pi, pi]
    return wrap_to_pi(yaw_cmd)

class GuidancePublisher(Node):
    """
    GOAL want to publish a roll,pitch,yaw trajectory to the drone to get
    to the target location
    HINTS Not in order
    - Remember the current coordinate system is in ENU need to convert to NED
    - Might need to add some safety checks
    - Need to calculate something to get the roll, pitch, yaw commands
        - Yaw and roll control the lateral motion
        - Pitch control the vertical motion  
    - Need to subscribe to something else besides the mavros state
    """

    def __init__(self, ns=''):
        super().__init__('pub_example')

        self.trajectory_publisher: Publisher = self.create_publisher(
            CtlTraj, 'trajectory', 10)

        self.state_sub: Subscription = self.create_subscription(
            mavros.local_position.Odometry,
            'mavros/local_position/odom',
            self.mavros_state_callback,
            qos_profile=SENSOR_QOS)

        # subscribe to your target position
        # self.target_sub: Subscription = self.create_subscription(
        #     Odometry,
        #     'target_position',
        #     self.calculate_trajectory,
        #     10)
        
        self.target: List[float] = [
            -170,  # x
            -170,  # y
            50,  # z
        ]
        
        self.current_state: List[float] = [
            None,  # x
            None,  # y
            None,  # z
            None,  # phi
            None,  # theta
            None,  # psi
            None,  # airspeed
        ]

    def mavros_state_callback(self, msg: mavros.local_position.Odometry) -> None:
        """
        Converts NED to ENU and publishes the trajectory
        """
        self.current_state[0] = msg.pose.pose.position.x
        self.current_state[1] = msg.pose.pose.position.y
        self.current_state[2] = msg.pose.pose.position.z

        # quaternion attitudes
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = rot_utils.euler_from_quaternion(
            qx, qy, qz, qw)

        self.current_state[3] = roll
        self.current_state[4] = pitch
        self.current_state[5] = yaw  # (yaw+ (2*np.pi) ) % (2*np.pi);

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        # get magnitude of velocity
        self.current_state[6] = np.sqrt(vx**2 + vy**2 + vz**2)
        self.calculate_trajectory()

        
    def calculate_trajectory(self) -> CtlTraj:
        """
        You need to calculate the trajectory based on the target position
        Remember the yaw command must be RELATIVE 
        """
        if self.current_state[0] is None:
            return
        dx:float = self.target[0] - self.current_state[0]
        dy:float = self.target[1] - self.current_state[1]
        dz:float = self.target[2] - self.current_state[2]
        dist: float = np.sqrt(dx**2 + dy**2)
        print("dist", dist)
        enu_yaw_rad:float = np.arctan2(dy, dx)
        #ned_yaw_cmd_rad:float = yaw_enu_to_ned(enu_yaw_rad)
        ned_yaw_rad = yaw_enu_to_ned(enu_yaw_rad)
        ned_yaw_state = yaw_enu_to_ned(self.current_state[5]) 
        rel_yaw_cmd:float = get_relative_ned_yaw_cmd(
            ned_yaw_state, ned_yaw_rad)
        
        kp:float = 0.5
        roll_cmd:float = kp * rel_yaw_cmd
        roll_cmd = np.clip(roll_cmd, -np.deg2rad(45), np.deg2rad(45))
        
        kp_pitch:float = 0.25
        pitch_cmd:float = kp_pitch * (dz)
        pitch_cmd = np.clip(pitch_cmd, -np.deg2rad(15), np.deg2rad(10))
        thrust_cmd = float(0.5)
        # create a trajectory message
        trajectory: CtlTraj = CtlTraj()
        trajectory.roll = [roll_cmd, roll_cmd]
        trajectory.pitch = [pitch_cmd, pitch_cmd]
        trajectory.yaw = [rel_yaw_cmd, rel_yaw_cmd]
        trajectory.idx = int(0)
        trajectory.thrust = [thrust_cmd, thrust_cmd, thrust_cmd]
#        print(trajectory)
        self.publish_trajectory(trajectory)
        
    def publish_trajectory(self, trajectory: CtlTraj) -> None:
        """
        Publishes the trajectory
        """
        self.trajectory_publisher.publish(trajectory)

def main() -> None:
    rclpy.init()
    guidance_publisher:GuidancePublisher = GuidancePublisher()
    while rclpy.ok():
        try:
            if guidance_publisher.current_state[0] is None:
                rclpy.spin_once(guidance_publisher, timeout_sec=0.05)
                continue
            rclpy.spin_once(guidance_publisher, timeout_sec=0.05)
        
        except KeyboardInterrupt:
            
            guidance_publisher.get_logger().info('Keyboard Interrupt')
            break
    
    guidance_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()