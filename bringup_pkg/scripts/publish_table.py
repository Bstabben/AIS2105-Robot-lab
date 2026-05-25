#!/usr/bin/env python3
"""
Add table collision geometry to MoveIt2 planning scene.
Run this after launching full_system.
"""

import rclpy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from rclpy.node import Node


class TablePublisher(Node):
    def __init__(self):
        super().__init__('table_publisher')

        self.collision_pub = self.create_publisher(
            CollisionObject,
            '/moveit_cpp/planning_scene_monitor/planning_scene',
            10
        )

        self.get_logger().info('Publishing table collision object...')
        self.publish_table()

    def publish_table(self):
        # Create a box collision object for the table
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = 'base_link'
        collision_obj.id = 'table'
        collision_obj.operation = CollisionObject.ADD

        # Table: 2m x 2m x 0.05m, top surface at z=0.047
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, 0.05]

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.047 - 0.025  # Top surface at 0.047, box center offset down
        pose.orientation.w = 1.0

        collision_obj.primitives.append(box)
        collision_obj.primitive_poses.append(pose)

        # Publish multiple times to ensure it gets through
        for _ in range(5):
            self.collision_pub.publish(collision_obj)
            self.get_logger().info('Table collision object published')


def main(args=None):
    rclpy.init(args=args)
    node = TablePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
