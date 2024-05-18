#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        self.declare_parameter("port","/dev/ttyACM0")
        self.declare_parameter("baud_rate",115200)
        self.port_ = self.get_parameter("port").string_value
        self.baud_rate_ = self.get_parameter("baud_rate").value
        
        self.subscription = self.create_subscription(
            String, "serial_transmitter", self.msgCallback, 10)
        self.arduino_ = serial.Serial(port=self.port_, baud_rate=self.baud_rate_,timeout=0.1)

    def msgCallback(self, msg):
        self.get_logger().info("New Message publishing on serial: {0}".format(self.arduino_.name))
        self.arduino_.write(msg.data.encode("utf-8"))

def main():
    rclpy.init()
    node = SimpleSerialTransmitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
