#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from arduino_bot_msgs.srv import EulerToQuat, QuatToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class AngleConverter(Node):
    def __init__(self):
        super().__init__("angle_conversion_server")

        self.euler_to_quaternion_ = self.create_service(
            EulerToQuat, "euler_to_quaternion", self.eulerToQuaternionCallback)
        self.quaternion_to_euler_ = self.create_service(
            QuatToEuler, "quaternion_to_euler", self.quaternionToEulerCallback)
        self.get_logger().info("Angle Conversion Services Ready")

    def eulerToQuaternionCallback(self, req, res):
        self.get_logger().info("Request to convert Euler to Quat angles roll: %f, pitch: %f yaw: %f " %
                               (req.roll, req.pitch, req.yaw))
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch,req.yaw)
        self.get_logger().info("Returning Quat angles x: %f, y: %f, z: %f, w: %f " %
                               (res.x, res.y, res.z, res.w))
        return res
        
    def quaternionToEulerCallback(self, req, res):
        self.get_logger().info("Request to convert Quat to Euler angles x: %f, y: %f z: %f w: %f " %
                               (req.x, req.y, req.z, req.w))
        (res.roll, res.pitch,res.yaw) = euler_from_quaternion([req.x, req.y, req.w, req.z])
        self.get_logger().info("Returning Euler angles roll: %f, pitch: %f, yaw: %f " %
                               (res.roll, res.pitch,res.yaw))
        return res

def main():
    rclpy.init()
    node = AngleConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
