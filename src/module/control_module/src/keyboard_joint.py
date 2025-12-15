#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, select, termios, tty
import os

class FullArmController(Node):
    def __init__(self):
        super().__init__('py_full_control')
        self.publisher_ = self.create_publisher(String, 'py_joint_cmd', 10)
        
        # 默认选择左臂，第一个关节
        self.active_arm = "left" 
        self.active_joint_idx = 0 
        
        # 关节后缀列表 (严格匹配 dcu_x1.yaml)
        self.joint_suffixes = [
            "shoulder_pitch_joint",
            "shoulder_roll_joint",
            "shoulder_yaw_joint",
            "elbow_pitch_joint",
            "elbow_yaw_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
            "claw_joint"   # [修正] 必须叫 claw_joint
        ]
        
        # 用于显示的名称
        self.joint_display_names = [
            "1. 肩部-俯仰 (Shoulder Pitch)",
            "2. 肩部-横滚 (Shoulder Roll)",
            "3. 肩部-偏航 (Shoulder Yaw)",
            "4. 肘部-俯仰 (Elbow Pitch)",
            "5. 肘部-偏航 (Elbow Yaw)",
            "6. 腕部-俯仰 (Wrist Pitch)",
            "7. 腕部-横滚 (Wrist Roll)",
            "8. 末端-夹爪 (Claw) [0.0 - 1.0]" # [新增] 显示夹爪
        ]

        # 本地记录夹爪当前状态 (0.0=松开, 1.0=闭合)
        self.claw_values = {"left": 0.0, "right": 0.0}

        self.print_interface()

    def print_interface(self):
        os.system('clear') # 清屏
        print("==================================================")
        print("      ROS2 双臂 + 夹爪 (Claw) 键盘控制器      ")
        print("==================================================")
        
        # 显示当前选中的手臂
        left_mark = ">>> LEFT (左) <<<" if self.active_arm == 'left' else "    LEFT (左)    "
        right_mark = ">>> RIGHT (右) <<<" if self.active_arm == 'right' else "    RIGHT (右)    "
        print(f"当前选择手臂: [{left_mark}]   [{right_mark}]")
        print("--------------------------------------------------")
        
        # 打印关节列表，高亮选中的行
        for i, name in enumerate(self.joint_display_names):
            prefix = ">>> " if i == self.active_joint_idx else "    "
            
            # 如果是夹爪，额外显示当前数值
            val_info = ""
            if "Claw" in name:
                val = self.claw_values[self.active_arm]
                val_info = f" [当前: {val:.1f}]"
            
            print(f"{prefix} {name}{val_info}")
            
        print("--------------------------------------------------")
        print(" [TAB] : 切换左/右臂")
        print(" [1-8] : 直接选择关节")
        print(" [ W ] : 增加角度/闭合夹爪 (+0.1)")
        print(" [ S ] : 减少角度/松开夹爪 (-0.1)")
        print(" [Ctrl+C] : 退出程序")
        print("==================================================")

    def send_command(self, delta):
        # 拼接完整的关节名称，例如 left_claw_joint
        suffix = self.joint_suffixes[self.active_joint_idx]
        full_joint_name = f"{self.active_arm}_{suffix}"
        
        final_delta = delta

        # --- 针对夹爪 (Claw) 的特殊处理 ---
        if "claw" in suffix:
            current = self.claw_values[self.active_arm]
            new_val = current + delta
            
            # 软件限位：强制限制在 0.0 ~ 1.0 之间
            if new_val > 1.0: new_val = 1.0
            elif new_val < 0.0: new_val = 0.0
            
            # 计算实际需要发送的增量
            # (如果已经到了1.0，再按+0.1，这里算出来的 final_delta 会是 0.0，不会发送无效指令)
            final_delta = new_val - current
            
            # 更新本地记录
            self.claw_values[self.active_arm] = new_val
        
        # 只有增量不为0时才发送，减少通信负载
        if abs(final_delta) > 0.0001:
            msg = String()
            msg.data = f"{full_joint_name}:{final_delta}"
            self.publisher_.publish(msg)
            # 调试用：也可以把这个取消注释
            # self.get_logger().info(f"发送: {msg.data}")

# --- 以下为键盘监听底层逻辑，无需修改 ---
settings = termios.tcgetattr(sys.stdin)

def get_key():
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
    node = FullArmController()

    try:
        while rclpy.ok():
            key = get_key()
            if key == '':
                rclpy.spin_once(node, timeout_sec=0)
                continue

            update_ui = False
            
            # 1. 切换手臂 (TAB键)
            if key == '\t': 
                node.active_arm = "right" if node.active_arm == "left" else "left"
                update_ui = True
            
            # 2. 选择关节 (数字 1-8)
            elif key in ['1', '2', '3', '4', '5', '6', '7', '8']:
                node.active_joint_idx = int(key) - 1
                update_ui = True
            
            # 3. 控制增量 (W/S)
            elif key == 'w' or key == 'W': # 增加
                node.send_command(0.1)
                update_ui = True # 更新UI以显示新的夹爪数值
            elif key == 's' or key == 'S': # 减少
                node.send_command(-0.1)
                update_ui = True
                
            # 4. 退出
            elif key == '\x03': # Ctrl+C
                break

            if update_ui:
                node.print_interface()

            rclpy.spin_once(node, timeout_sec=0)
            
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()