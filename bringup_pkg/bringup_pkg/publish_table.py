#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class TablePublisher(Node):
    def __init__(self):
        super().__init__('table_publisher')
        self.declare_parameter('table_z', 0.047)
        self._pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self._msg = self._build_msg()
        # Publish immediately, then every 10s
        self.create_timer(10.0, self._publish)
        self._publish()

    def _build_msg(self):
        table_z = self.get_parameter('table_z').get_parameter_value().double_value

        table = CollisionObject()
        table.header.frame_id = 'base_link'
        table.id = 'table'
        table.operation = CollisionObject.ADD

        table_box = SolidPrimitive()
        table_box.type = SolidPrimitive.BOX
        table_box.dimensions = [2.0, 2.0, 1.0]   # 1m thick, extends below floor

        table_pose = Pose()
        # Top of box sits exactly at table_z: bottom is 1 m below.
        table_pose.position.z = table_z - 0.5
        table_pose.orientation.w = 1.0

        table.primitives.append(table_box)
        table.primitive_poses.append(table_pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(table)
        return scene

    def _publish(self):
        self._pub.publish(self._msg)
        self.get_logger().info('Table collision object published to /planning_scene')


def main(args=None):
    rclpy.init(args=args)
    node = TablePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
