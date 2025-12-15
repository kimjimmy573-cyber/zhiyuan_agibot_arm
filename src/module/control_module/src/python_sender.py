#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSender(Node):
    def __init__(self):
        super().__init__('py_sender_node')
        self.publisher_ = self.create_publisher(String, 'pycpp', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello C++, count: {self.i}'
        self.publisher_.publish(msg)
        print(f'Sending: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SimpleSender())
    rclpy.shutdown()

if __name__ == '__main__':
    main()