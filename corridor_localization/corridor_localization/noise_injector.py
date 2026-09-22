#!/usr/bin/env python3
"""
Noise injection node — Task 2 of the Shifting Wall & Slipping Base Challenge.

Two independent corruption pipelines, sharing one node:
  1. Odometry: wheel-slip velocity scaling in a "dusty patch" (X=10-15).
  2. IMU: gyro bias random-walk drift + white noise, applied continuously
     to the z-axis (yaw rate) only, since the robot only ever rotates
     about z on a flat corridor floor.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# --- Odometry: slip patch region (meters, along corridor X) ---
PATCH_X_MIN = 10.0
PATCH_X_MAX = 15.0
SLIP_SCALE = 0.70      # s(t) inside the patch

# --- Odometry: velocity measurement noise ---
SIGMA_V = 0.02          # m/s, white noise stddev (both in and out of patch)

# --- IMU: gyro noise models (z-axis / yaw rate only) ---
SIGMA_G = 0.01           # rad/s, instantaneous measurement noise (per-sample, no dt scaling)
SIGMA_B = 0.0005         # rad/s per sqrt(s), bias random-walk rate (dt-scaled)


class NoiseInjector(Node):

    def __init__(self):
        super().__init__('noise_injector')

        # --- Odometry pipeline ---
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.on_ground_truth_odom, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom_slipped', 10)

        self.x_est = None
        self.y_est = None
        self.theta_est = None
        self.last_odom_stamp = None

        # --- IMU pipeline ---
        self.sub_imu = self.create_subscription(
            Imu, '/imu', self.on_ground_truth_imu, 10)
        self.pub_imu = self.create_publisher(Imu, '/imu_drifted', 10)

        self.gyro_bias_z = 0.0
        self.last_imu_stamp = None

        self.get_logger().info(
            'Noise injector started — odometry slip + IMU gyro drift models active')

    # ------------------------------------------------------------------
    # Odometry pipeline (unchanged from before)
    # ------------------------------------------------------------------

    def on_ground_truth_odom(self, msg: Odometry):
        x_true = msg.pose.pose.position.x
        v_true = msg.twist.twist.linear.x
        w_true = msg.twist.twist.angular.z

        stamp = msg.header.stamp
        t_now = stamp.sec + stamp.nanosec * 1e-9

        if self.x_est is None:
            self.x_est = x_true
            self.y_est = msg.pose.pose.position.y
            self.theta_est = self._yaw_from_quat(msg.pose.pose.orientation)
            self.last_odom_stamp = t_now
            return

        dt = t_now - self.last_odom_stamp
        self.last_odom_stamp = t_now
        if dt <= 0.0:
            return

        in_patch = PATCH_X_MIN <= x_true <= PATCH_X_MAX
        s_t = SLIP_SCALE if in_patch else 1.0
        w_v = np.random.normal(0.0, SIGMA_V)
        v_recorded = s_t * v_true + w_v

        self.x_est += v_recorded * math.cos(self.theta_est) * dt
        self.y_est += v_recorded * math.sin(self.theta_est) * dt
        self.theta_est += w_true * dt

        self._publish_corrupted_odom(msg.header.stamp, v_recorded, w_true)

    def _publish_corrupted_odom(self, stamp, v_recorded, w_recorded):
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'

        out.pose.pose.position.x = self.x_est
        out.pose.pose.position.y = self.y_est
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation = self._quat_from_yaw(self.theta_est)

        out.twist.twist.linear.x = v_recorded
        out.twist.twist.angular.z = w_recorded

        self.pub_odom.publish(out)

    # ------------------------------------------------------------------
    # IMU pipeline (new)
    # ------------------------------------------------------------------

    def on_ground_truth_imu(self, msg: Imu):
        w_true_z = msg.angular_velocity.z

        stamp = msg.header.stamp
        t_now = stamp.sec + stamp.nanosec * 1e-9

        if self.last_imu_stamp is None:
            self.last_imu_stamp = t_now
            return  # first message: nothing to integrate yet

        dt = t_now - self.last_imu_stamp
        self.last_imu_stamp = t_now
        if dt <= 0.0:
            return

        # Bias random walk: b_k = b_{k-1} + N(0, sigma_b^2 * dt)
        # (std of the increment scales with sqrt(dt), not dt itself —
        # see discussion: this keeps drift rate independent of tick rate)
        self.gyro_bias_z += np.random.normal(0.0, SIGMA_B * math.sqrt(dt))

        # Instantaneous measurement noise: fresh draw every tick, no dt scaling
        eta_g = np.random.normal(0.0, SIGMA_G)

        w_measured_z = w_true_z + self.gyro_bias_z + eta_g

        self._publish_corrupted_imu(msg, w_measured_z)

    def _publish_corrupted_imu(self, true_msg: Imu, w_measured_z: float):
        out = Imu()
        out.header = true_msg.header

        # x/y gyro and full accelerometer passed through uncorrupted —
        # only z-axis (yaw rate) carries real signal on this flat-floor robot
        out.angular_velocity.x = true_msg.angular_velocity.x
        out.angular_velocity.y = true_msg.angular_velocity.y
        out.angular_velocity.z = w_measured_z

        out.linear_acceleration = true_msg.linear_acceleration
        out.orientation = true_msg.orientation

        self.pub_imu.publish(out)

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quat_from_yaw(yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = NoiseInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
