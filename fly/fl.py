#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import State
import threading
import time
import sys

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        
        # 声明控制舵机的服务客户端
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        # 订阅状态信息
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_callback,
            10
        )
        
        self.current_state = State()
        
        # 舵机位置状态 (-1.0 到 1.0)
        self.servo1_position = 0.0  # AUX1
        self.servo2_position = 0.0  # AUX2
        
        # 等待服务可用
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 CommandLong 服务...')
            
        self.get_logger().info('舵机控制节点已启动')
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def servo_change(self, servo1_pos, servo2_pos):
        """控制舵机函数"""
        request = CommandLong.Request()
        request.command = 187  # MAV_CMD_DO_SET_ACTUATOR
        
        # 限制舵机位置在 -1.0 到 1.0 之间
        servo1_pos = max(-1.0, min(1.0, servo1_pos))
        servo2_pos = max(-1.0, min(1.0, servo2_pos))
        
        request.param1 = float(servo1_pos)  # AUX1
        request.param2 = float(servo2_pos)  # AUX2
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = 0.0
        request.param6 = 0.0
        request.param7 = 0.0
        
        self.get_logger().info(
            f"舵机控制 - AUX1: {request.param1:.2f}, AUX2: {request.param2:.2f}"
        )
        
        future = self.cmd_client.call_async(request)
        
        # 非阻塞方式处理结果
        def handle_response():
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.result() is not None:
                pass  # 成功，不打印过多信息
            else:
                self.get_logger().error("调用 CommandLong 服务失败")
        
        # 在新线程中处理响应，避免阻塞
        response_thread = threading.Thread(target=handle_response)
        response_thread.daemon = True
        response_thread.start()
    
    def set_servo_position(self, servo_num, position):
        """设置单个舵机位置"""
        if servo_num == 1:
            self.servo1_position = position
        elif servo_num == 2:
            self.servo2_position = position
        
        self.servo_change(self.servo1_position, self.servo2_position)
    
    def set_both_servos(self, pos1, pos2):
        """同时设置两个舵机位置"""
        self.servo1_position = pos1
        self.servo2_position = pos2
        self.servo_change(self.servo1_position, self.servo2_position)
    
    def reset_servos(self):
        """重置舵机到中位"""
        self.set_both_servos(0.0, 0.0)
    
    def sweep_servo(self, servo_num, duration=2.0):
        """让指定舵机扫描运动"""
        steps = 50
        for i in range(steps + 1):
            # 从-1到1的正弦波运动
            import math
            position = math.sin(2 * math.pi * i / steps)
            self.set_servo_position(servo_num, position)
            time.sleep(duration / steps)

def print_help():
    """打印帮助信息"""
    print("\n=== 舵机控制命令 ===")
    print("1 <位置>     - 设置舵机1位置 (AUX1), 位置范围: -1.0 到 1.0")
    print("2 <位置>     - 设置舵机2位置 (AUX2), 位置范围: -1.0 到 1.0")
    print("both <位置1> <位置2> - 同时设置两个舵机位置")
    print("reset        - 重置两个舵机到中位 (0.0)")
    print("sweep1       - 舵机1扫描运动")
    print("sweep2       - 舵机2扫描运动")
    print("status       - 显示当前舵机位置")
    print("help         - 显示此帮助信息")
    print("quit         - 退出程序")
    print("==================")

def interactive_control(node):
    """交互式控制函数"""
    print("\n欢迎使用舵机控制系统!")
    print_help()
    
    while rclpy.ok():
        try:
            command = input("\n请输入命令: ").strip().split()
            
            if not command:
                continue
                
            cmd = command[0].lower()
            
            if cmd == 'quit' or cmd == 'q':
                print("退出程序...")
                break
            elif cmd == 'help' or cmd == 'h':
                print_help()
            elif cmd == 'reset' or cmd == 'r':
                node.reset_servos()
                print("舵机已重置到中位")
            elif cmd == 'status' or cmd == 's':
                print(f"当前舵机位置 - AUX1: {node.servo1_position:.2f}, AUX2: {node.servo2_position:.2f}")
            elif cmd == 'sweep1':
                print("舵机1开始扫描运动...")
                node.sweep_servo(1)
                print("舵机1扫描运动完成")
            elif cmd == 'sweep2':
                print("舵机2开始扫描运动...")
                node.sweep_servo(2)
                print("舵机2扫描运动完成")
            elif cmd == '1' and len(command) == 2:
                try:
                    position = float(command[1])
                    node.set_servo_position(1, position)
                    print(f"舵机1设置到位置: {position:.2f}")
                except ValueError:
                    print("错误: 位置必须是数字")
            elif cmd == '2' and len(command) == 2:
                try:
                    position = float(command[1])
                    node.set_servo_position(2, position)
                    print(f"舵机2设置到位置: {position:.2f}")
                except ValueError:
                    print("错误: 位置必须是数字")
            elif cmd == 'both' and len(command) == 3:
                try:
                    pos1 = float(command[1])
                    pos2 = float(command[2])
                    node.set_both_servos(pos1, pos2)
                    print(f"舵机位置设置 - AUX1: {pos1:.2f}, AUX2: {pos2:.2f}")
                except ValueError:
                    print("错误: 位置必须是数字")
            else:
                print("未知命令，输入 'help' 查看可用命令")
                
        except KeyboardInterrupt:
            print("\n程序被用户中断")
            break
        except EOFError:
            print("\n程序结束")
            break

def main(args=None):
    rclpy.init(args=args)
    
    node = ServoControlNode()
    
    # 等待连接
    print("等待MAVROS连接...")
    while rclpy.ok() and not node.current_state.connected:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)
    
    if node.current_state.connected:
        print("MAVROS已连接!")
        
        # 在后台线程中运行ROS2自旋
        def spin_thread():
            rclpy.spin(node)
        
        spinner = threading.Thread(target=spin_thread)
        spinner.daemon = True
        spinner.start()
        
        # 重置舵机到初始位置
        node.reset_servos()
        
        # 启动交互式控制
        interactive_control(node)
    else:
        print("无法连接到MAVROS")
    
    # 清理
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()