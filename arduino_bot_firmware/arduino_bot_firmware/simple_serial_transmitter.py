#!/usr/bin/env python3
import rclpy.node import Node
from std_msgs.msg import String
 
class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")
        self.subscription = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)

    def msgCallback(self,msg):
        self.get_logger().info("I heard: {0}".format(msg.data))
        
def main():
    rclpy.init()
    node = SimpleSerialTransmitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":  
    main()