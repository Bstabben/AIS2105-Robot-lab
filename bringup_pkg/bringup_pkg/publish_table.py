#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class TablePublisher(Node):
    def __init__(self):
        super().__init__('table_publisher')
        self._pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self._msg = self._build_msg()
        # Publish immediately, then every 10 s so the table survives a MoveIt2 restart
        self.create_timer(10.0, self._publish)
        self._publish()

    def _build_msg(self):
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'table'
        obj.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, 0.05]

        pose = Pose()
        pose.position.z = -0.05 - 0.025  # top surface at z=-0.05, centre offset down
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
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
