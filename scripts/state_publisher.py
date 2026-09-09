#!/usr/bin/env python3
"""
State and Trajectory Publisher for KUKA YouBot in ROS 2.
Supports both direct joint state publication (/joint_states for RViz)
and trajectory command publication (/youbot/set_joint_trajectory for Gazebo simulation).
"""

from math import pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration, Time


DEFAULT_PATH = [
    {"t": 3.0, "arm": [169, 60, -140, 100, 167]},
    {"t": 3.0, "arm": [-45, 7, -7, 7, 7]},
    {"t": 3.0, "arm": [-45, 60, -140, 145, 100]},
    {"t": 3.0, "arm": [-45, 7, -7, 7, 7]},
    {"t": 3.0, "arm": [-45, 60, -140, 145, 167]},
    {"t": 6.0, "arm": [169, 60, -140, 100, 100]},
    {"t": 4.0, "arm": [169, 60, -100, 100, 167]},
    {"t": 3.0, "arm": [7, 60, -100, 100, 167]},
    {"t": 3.0, "arm": [169, 60, -100, 100, 167]},
    {"t": 3.0, "arm": [7, 60, -100, 100, 167]},
    {"t": 5.0, "arm": [169, 60, -140, 100, 167]},
    {"t": 3.0, "arm": [7, 7, -7, 7, 7]},
    {"t": 5.0, "arm": [0, 0, 0, 0, 0]}
]


class YoubotStatePublisher(Node):

    def __init__(self):
        super().__init__('youbot_state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.traj_pub = self.create_publisher(JointTrajectory, '/youbot/set_joint_trajectory', qos_profile)

        self.get_logger().info("YouBot state publisher initialized")

        self.degree = pi / 180.0
        self.rate_hz = 30.0
        self.dt = 1.0 / self.rate_hz

        # Arm joints for standard ROS (RViz / URDF)
        self.arm_joint_names = [
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'arm_joint_4',
            'arm_joint_5'
        ]

        # Gazebo scoped joint names
        self.gazebo_arm_joint_names = [
            'youbot::arm_joint_1',
            'youbot::arm_joint_2',
            'youbot::arm_joint_3',
            'youbot::arm_joint_4',
            'youbot::arm_joint_5'
        ]

        # All joints for RViz
        self.all_joint_names = [
            'virtual_joint_x',
            'virtual_joint_y',
            'virtual_joint_z',
            'wheel_joint_fl',
            'wheel_joint_fr',
            'wheel_joint_bl',
            'wheel_joint_br',
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'arm_joint_4',
            'arm_joint_5'
        ]

        self.current_arm = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.path = DEFAULT_PATH
        self.waypoint_idx = 0
        self.segment_start_time = self.get_clock().now()
        self.segment_start_arm = list(self.current_arm)

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(f"Loaded {len(self.path)} trajectory waypoints. Starting motion...")

    @staticmethod
    def lerp(alpha: float, x0: float, x1: float) -> float:
        """Linear interpolation between x0 and x1 with parameter alpha in [0, 1]."""
        return float(x0) + float(x1 - x0) * max(0.0, min(1.0, alpha))

    def timer_callback(self):
        if self.waypoint_idx >= len(self.path):
            self.get_logger().info("Trajectory execution completed.")
            self.timer.cancel()
            return

        target_wp = self.path[self.waypoint_idx]
        segment_duration = float(target_wp["t"])
        target_degrees = target_wp["arm"]
        target_radians = [deg * self.degree for deg in target_degrees]

        now = self.get_clock().now()
        elapsed = (now - self.segment_start_time).nanoseconds / 1e9

        if segment_duration <= 0.0:
            alpha = 1.0
        else:
            alpha = elapsed / segment_duration

        # Interpolate current joint positions
        for j in range(5):
            self.current_arm[j] = self.lerp(alpha, self.segment_start_arm[j], target_radians[j])

        # 1. Publish JointState (for RViz visualization)
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = self.all_joint_names
        joint_state.position = [
            0.0, 0.0, 0.0,  # virtual_joint x, y, z
            0.0, 0.0, 0.0, 0.0,  # wheel joints
            self.current_arm[0],
            self.current_arm[1],
            self.current_arm[2],
            self.current_arm[3],
            self.current_arm[4]
        ]
        self.joint_pub.publish(joint_state)

        # 2. Publish JointTrajectory (for Gazebo simulation)
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = Time(sec=0, nanosec=0)  # 0 means start immediately in sim time
        traj_msg.header.frame_id = 'world'
        traj_msg.joint_names = self.gazebo_arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = list(self.current_arm)
        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)

        # Check if segment completed
        if alpha >= 1.0:
            self.get_logger().info(
                f"Reached waypoint {self.waypoint_idx + 1}/{len(self.path)}: {target_degrees}"
            )
            self.waypoint_idx += 1
            self.segment_start_time = now
            self.segment_start_arm = list(self.current_arm)


def main(args=None):
    rclpy.init(args=args)
    node = YoubotStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()