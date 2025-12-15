import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

STEP_SIZE = 0.1 

class KeyboardArmController(Node):
    def __init__(self):
        super().__init__('jyf_twist_commander')
        # 复用现有的 cmd_vel_limiter 话题
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_limiter', 10)
        self.left_angle = 0.0
        self.right_angle = 0.0
        print(f"模式：借用速度通道控制手臂")
        print(f"按 [W/S] : 左臂上下")
        print(f"按 [A/D] : 右臂上下")
        print(f"按 [Ctrl+C] : 退出")

    def publish_cmd(self):
        msg = Twist()
        # 把角度伪装成速度发出去
        msg.linear.x = self.left_angle   # 左臂
        msg.angular.z = self.right_angle # 右臂
        
        self.publisher_.publish(msg)
        print(f"发送 -> 左臂: {self.left_angle:.2f}, 右臂: {self.right_angle:.2f}")

    def update(self, key):
        updated = False
        if key == 'w': 
            self.left_angle += STEP_SIZE
            updated = True
        elif key == 's': 
            self.left_angle -= STEP_SIZE
            updated = True
        elif key == 'a': 
            self.right_angle += STEP_SIZE
            updated = True
        elif key == 'd': 
            self.right_angle -= STEP_SIZE
            updated = True
        
        if updated:
            self.publish_cmd()

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardArmController()
    settings = termios.tcgetattr(sys.stdin)
    try:
        node.publish_cmd() # 发送初始位置
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03': break # Ctrl+C
            if key != '': node.update(key)
            rclpy.spin_once(node, timeout_sec=0)
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()