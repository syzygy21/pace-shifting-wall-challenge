#!/usr/bin/env python3
"""
EKF sensor fusion node — Task 2 (fusion + degeneracy tracking) of the
Shifting Wall & Slipping Base Challenge.

State: [x, y, theta] (3-state — gyro bias NOT estimated in-state for now).

Predict: triggered by /imu_drifted (higher rate), using the most recently
cached v from /odom_slipped.

Correct: triggered by each new /lidar/points/points scan, matched via
point-to-plane ICP against a KEYFRAME scan (refreshed every N scans).
The ICP information matrix becomes R, eigenvalue-clipped on both ends
(floor for near-singular directions, cap against overconfidence).

R_SCALE: diagnostic multiplier on R. Set to 1.0 normally. Set to something
large (e.g. 1e4) to effectively disable correction and check whether
x_fused converges onto x_raw — isolates whether a bug is in predict or
correct.

Publishes: /fused_odom (nav_msgs/Odometry, pose + covariance filled in).
"""

import math

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import Quaternion
import sensor_msgs_py.point_cloud2 as pc2


# --- Input noise levels (must match noise_injector.py — used to build Q) ---
SIGMA_V = 0.02      # m/s, from the odometry slip model
SIGMA_OMEGA = 0.01  # rad/s, from the IMU gyro noise model (SIGMA_G)

# --- Initial covariance: small but nonzero (spawn pose is known reasonably well) ---
P0 = np.diag([0.01, 0.01, 0.01])

# --- Degeneracy detection + R flooring/capping ---
EIGENVALUE_FLOOR = 1e-3        # minimum information eigenvalue (near-singular directions)
EIGENVALUE_CAP = 2500.0        # maximum information eigenvalue (~2cm/2crad realistic floor on R)
DEGENERACY_THRESHOLD = 5e-3    # warn if the RAW (pre-clip) min eigenvalue drops below this

# --- DIAGNOSTIC: R scale multiplier ---
# 1.0 = normal. Set large (e.g. 1e4) to effectively disable correction
# and test whether x_fused converges onto x_raw (isolates predict vs correct).
R_SCALE = 1e4

# --- ICP tuning ---
ICP_MAX_CORRESPONDENCE_DIST = 0.3  # meters
ICP_NORMAL_RADIUS = 0.3
ICP_NORMAL_MAX_NN = 30

# --- Keyframe management ---
KEYFRAME_REFRESH_SCANS = 10   # re-anchor every N scans (~1s at 10Hz)
MIN_CORRESPONDENCES = 20      # below this, skip correction AND force a keyframe refresh


class EkfFusion(Node):

    def __init__(self):
        super().__init__('ekf_fusion')

        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.P = P0.copy()

        # --- Cached inputs ---
        self.cached_v = 0.0
        self.last_predict_stamp = None

        # --- Keyframe ICP state ---
        self.keyframe_cloud = None       # o3d.geometry.PointCloud
        self.keyframe_pose = None        # (x, y, theta) at keyframe capture time
        self.scans_since_keyframe = 0

        # --- Subscriptions ---
        self.sub_odom = self.create_subscription(
            Odometry, '/odom_slipped', self.on_odom_slipped, 10)
        self.sub_imu = self.create_subscription(
            Imu, '/imu_drifted', self.on_imu_drifted, 10)
        self.sub_lidar = self.create_subscription(
            PointCloud2, '/lidar/points/points', self.on_lidar_scan, 10)

        # --- Publisher ---
        self.pub_fused = self.create_publisher(Odometry, '/fused_odom', 10)

        self.get_logger().info(
            f'EKF fusion node started (3-state, scan-to-keyframe ICP, R_SCALE={R_SCALE})')

    # ------------------------------------------------------------------
    # Cache the latest corrupted velocity — no predict logic here at all
    # ------------------------------------------------------------------

    def on_odom_slipped(self, msg: Odometry):
        self.cached_v = msg.twist.twist.linear.x

    # ------------------------------------------------------------------
    # PREDICT — triggered by IMU arrival (higher rate)
    # ------------------------------------------------------------------

    def on_imu_drifted(self, msg: Imu):
        omega = msg.angular_velocity.z

        stamp = msg.header.stamp
        t_now = stamp.sec + stamp.nanosec * 1e-9

        if self.last_predict_stamp is None:
            self.last_predict_stamp = t_now
            return

        dt = t_now - self.last_predict_stamp
        self.last_predict_stamp = t_now
        if dt <= 0.0:
            return

        v = self.cached_v
        theta = self.theta

        self.x += v * math.cos(theta) * dt
        self.y += v * math.sin(theta) * dt
        self.theta += omega * dt

        F = np.array([
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0,  1.0],
        ])

        W = np.array([
            [math.cos(theta) * dt, 0.0],
            [math.sin(theta) * dt, 0.0],
            [0.0,                  dt],
        ])
        M = np.diag([SIGMA_V ** 2, SIGMA_OMEGA ** 2])
        Q = W @ M @ W.T

        self.P = F @ self.P @ F.T + Q

    # ------------------------------------------------------------------
    # CORRECT — triggered by each new LiDAR scan (scan-to-KEYFRAME ICP)
    # ------------------------------------------------------------------

    def on_lidar_scan(self, msg: PointCloud2):
        cloud = self._pointcloud2_to_o3d(msg)
        if cloud is None or len(cloud.points) < 50:
            return

        cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=ICP_NORMAL_RADIUS, max_nn=ICP_NORMAL_MAX_NN))

        if self.keyframe_cloud is None:
            self._set_keyframe(cloud)
            return

        kx, ky, ktheta = self.keyframe_pose
        init_guess = self._relative_transform(kx, ky, ktheta, self.x, self.y, self.theta)

        result = o3d.pipelines.registration.registration_icp(
            cloud, self.keyframe_cloud,
            ICP_MAX_CORRESPONDENCE_DIST,
            init_guess,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )

        if len(result.correspondence_set) < MIN_CORRESPONDENCES:
            self._set_keyframe(cloud)
            return

        info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            cloud, self.keyframe_cloud, ICP_MAX_CORRESPONDENCE_DIST, result.transformation)

        idx = [0, 1, 5]  # x, y, yaw — out of Open3D's full 6x6
        info_2d = info[np.ix_(idx, idx)]

        eigvals, eigvecs = np.linalg.eigh(info_2d)

        if eigvals.min() < DEGENERACY_THRESHOLD:
            self.get_logger().warn(
                f'LOCALIZATION_DEGENERACY_WARNING: min information eigenvalue '
                f'{eigvals.min():.6f} below threshold {DEGENERACY_THRESHOLD} '
                f'at pose x={self.x:.3f}, y={self.y:.3f}'
            )

        eigvals_clipped = np.clip(eigvals, EIGENVALUE_FLOOR, EIGENVALUE_CAP)
        info_clipped = eigvecs @ np.diag(eigvals_clipped) @ eigvecs.T
        R = R_SCALE * np.linalg.inv(info_clipped)

        T = result.transformation
        meas_x, meas_y, meas_theta = self._compose_pose(kx, ky, ktheta, T)
        z = np.array([meas_x, meas_y, meas_theta])
        x_pred = np.array([self.x, self.y, self.theta])

        y_innov = z - x_pred
        y_innov[2] = math.atan2(math.sin(y_innov[2]), math.cos(y_innov[2]))

        S = self.P + R
        K = self.P @ np.linalg.inv(S)

        x_corrected = x_pred + K @ y_innov
        self.x, self.y, self.theta = x_corrected
        self.P = (np.eye(3) - K) @ self.P

        self._publish_fused(msg.header.stamp)

        self.scans_since_keyframe += 1
        if self.scans_since_keyframe >= KEYFRAME_REFRESH_SCANS:
            self._set_keyframe(cloud)

    def _set_keyframe(self, cloud):
        self.keyframe_cloud = cloud
        self.keyframe_pose = (self.x, self.y, self.theta)
        self.scans_since_keyframe = 0

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_fused(self, stamp):
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'

        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.orientation = self._quat_from_yaw(self.theta)

        cov = [0.0] * 36
        cov[0] = self.P[0, 0]
        cov[1] = self.P[0, 1]
        cov[6] = self.P[1, 0]
        cov[7] = self.P[1, 1]
        cov[35] = self.P[2, 2]
        out.pose.covariance = cov

        self.pub_fused.publish(out)

    @staticmethod
    def _pointcloud2_to_o3d(msg: PointCloud2):
        points = np.array(
            [[p[0], p[1], p[2]] for p in pc2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)],
            dtype=np.float64)
        if points.shape[0] == 0:
            return None
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        return cloud

    @staticmethod
    def _relative_transform(x1, y1, th1, x2, y2, th2):
        dtheta = th2 - th1
        dx_world = x2 - x1
        dy_world = y2 - y1
        dx = math.cos(-th1) * dx_world - math.sin(-th1) * dy_world
        dy = math.sin(-th1) * dx_world + math.cos(-th1) * dy_world
        T = np.eye(4)
        T[0, 0] = math.cos(dtheta)
        T[0, 1] = -math.sin(dtheta)
        T[1, 0] = math.sin(dtheta)
        T[1, 1] = math.cos(dtheta)
        T[0, 3] = dx
        T[1, 3] = dy
        return T

    @staticmethod
    def _compose_pose(x1, y1, th1, T):
        dx = T[0, 3]
        dy = T[1, 3]
        dtheta = math.atan2(T[1, 0], T[0, 0])
        world_dx = math.cos(th1) * dx - math.sin(th1) * dy
        world_dy = math.sin(th1) * dx + math.cos(th1) * dy
        return x1 + world_dx, y1 + world_dy, th1 + dtheta

    @staticmethod
    def _quat_from_yaw(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = EkfFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
