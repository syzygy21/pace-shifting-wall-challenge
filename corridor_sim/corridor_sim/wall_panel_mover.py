#!/usr/bin/env python3
"""
Toggles the wall panel between its retracted and extended positions
every PANEL_SHIFT_INTERVAL_SEC seconds, by calling Gazebo's set_pose
service (bridged to ROS 2 via ros_gz_bridge).
"""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose

from corridor_sim.generate_corridor_world import (
    PANEL_X, PANEL_RETRACTED_Y, PANEL_EXTENDED_Y, PANEL_HEIGHT,
)

WORLD_NAME = 'corridor_world'
PANEL_SHIFT_INTERVAL_SEC = 15.0


class WallPanelMover(Node):
    def __init__(self):
        super().__init__('wall_panel_mover')

        self.client = self.create_client(SetEntityPose, f'/world/{WORLD_NAME}/set_pose')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_pose service (is the bridge running?)...')

        self.extended = False  # matches the world file's initial retracted state
        self.timer = self.create_timer(PANEL_SHIFT_INTERVAL_SEC, self.toggle_panel)
        self.get_logger().info(f'Wall panel mover started — toggling every {PANEL_SHIFT_INTERVAL_SEC}s')

    def toggle_panel(self):
        self.extended = not self.extended
        target_y = PANEL_EXTENDED_Y if self.extended else PANEL_RETRACTED_Y

        request = SetEntityPose.Request()
        request.entity.name = 'wall_panel'
        request.pose.position.x = PANEL_X
        request.pose.position.y = target_y
        request.pose.position.z = PANEL_HEIGHT / 2
        request.pose.orientation.w = 1.0  # identity rotation

        future = self.client.call_async(request)
        future.add_done_callback(self.on_response)

        state = 'EXTENDED' if self.extended else 'RETRACTED'
        self.get_logger().info(f'Panel -> {state} (Y={target_y:.3f})')

    def on_response(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn('set_pose service reported failure')
        except Exception as e:
            self.get_logger().error(f'set_pose service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WallPanelMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
