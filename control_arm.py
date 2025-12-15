import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import select
import termios
import tty

# 每次按键改变的角度量
STEP_SIZE = 0.1 

class KeyboardArmController(Node):
    def __init__(self):
        super().__init__('jyf_keyboard_commander')
        # 话题名必须与 C++ 代码中订阅的一致
        self.publisher_ = self.create_publisher(JointState, '/jyf_arm_cmd', 10)
        self.current_angle = 0.0 # 初始角度
        print(f"控制节点已启动！")
        print(f"按 [空格键] : 双臂肩关节角度减少 {STEP_SIZE}")
        print(f"按 [Ctrl+C] : 退出程序")

    def publish_angle(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 定义要控制的关节名字
        msg.name = [
            "left_shoulder_pitch_joint",
            "right_shoulder_pitch_joint"
        ]
        
        # 设置对应的角度 (两个关节都使用 self.current_angle)
        msg.position = [
            self.current_angle,
            self.current_angle
        ]
        
        self.publisher_.publish(msg)
        print(f"指令已发送: 角度 = {self.current_angle:.2f}")

    def update_angle(self):
        # 核心逻辑：减去 0.1
        self.current_angle -= STEP_SIZE
        self.publish_angle()

# === 下面是获取键盘输入的通用函数 (Linux/ROS常用写法) ===
def get_key(settings):
    # 将终端设置为原始模式，读取一个字符后立即返回
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    # 恢复终端设置，否则退出程序后终端会乱码
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardArmController()

    # 保存当前终端设置
    settings = termios.tcgetattr(sys.stdin)

    try:
        # 发送一次初始位置(0.0)
        node.publish_angle()
        
        while rclpy.ok():
            # 读取按键
            key = get_key(settings)
            
            if key == ' ':  # 如果是空格键
                node.update_angle()
            elif key == '\x03': # 如果是 Ctrl+C (ASCII码 0x03)
                break
                
            # 处理ROS的回调（如果有其他订阅需要处理）
            rclpy.spin_once(node, timeout_sec=0)
            
    except Exception as e:
        print(e)
        
    finally:
        # 再次确保恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
