#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")
        self.pub_ = self.create_publisher(String, "serial_receiver", 10)
        self.frequency_ = 0.01
        self.get_logger().info("publishing at %d Hz" % self.frequency_)

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.port_ = self.get_parameter("port").string_value
        self.baud_rate_ = self.get_parameter("baud_rate").value
        self.arduino_ = serial.Serial(
            port=self.port_, baud_rate=self.baud_rate_, timeout=0.1)

        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open():
            data = self.arduino_.readline()
            try:
                data.decode("utf-8")
            except:
                return

            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)


def main():
    rclpy.init()
    node = SimpleSerialReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
