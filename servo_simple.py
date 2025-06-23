#!/usr/bin/env python3
"""
极简舵机控制程序 - 只支持开关控制
仅支持两个90度舵机的完全打开(90度)和完全关闭(0度)
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
import threading
import time


class SimpleServoController(Node):
    """极简舵机控制器 - 只有开关功能"""
    
    def __init__(self):
        super().__init__('simple_servo_controller')
        
        # 发布器
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10
        )
        
        self.get_logger().info("极简舵机控制器已启动")
    
    def send_vehicle_command(self, servo_id, pwm_value):
        """发送舵机控制命令"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 187  # MAV_CMD_DO_SET_ACTUATOR
        msg.param1 = float(servo_id)  # 舵机编号 (1或2)
        msg.param2 = float(pwm_value)  # PWM值 (1000或2000)
        msg.param3 = float('nan')
        msg.param4 = float('nan')
        msg.param5 = float('nan')
        msg.param6 = float('nan')
        msg.param7 = float('nan')
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info(f"舵机{servo_id} PWM={pwm_value}")
    
    def open_servo1(self):
        """舵机1完全打开(90度)"""
        self.send_vehicle_command(1, 2000)
    
    def close_servo1(self):
        """舵机1完全关闭(0度)"""
        self.send_vehicle_command(1, 1000)
    
    def open_servo2(self):
        """舵机2完全打开(90度)"""
        self.send_vehicle_command(2, 2000)
    
    def close_servo2(self):
        """舵机2完全关闭(0度)"""
        self.send_vehicle_command(2, 1000)
    
    def open_both(self):
        """两个舵机同时打开"""
        self.open_servo1()
        time.sleep(0.1)
        self.open_servo2()
    
    def close_both(self):
        """两个舵机同时关闭"""
        self.close_servo1()
        time.sleep(0.1)
        self.close_servo2()


def main():
    """主函数"""
    rclpy.init()
    
    controller = SimpleServoController()
    
    def interactive_control():
        time.sleep(1)
        print("\n=== 极简舵机控制 ===")
        print("命令: 1o=舵机1开 1c=舵机1关 2o=舵机2开 2c=舵机2关 ao=全开 ac=全关 q=退出")
        
        while rclpy.ok():
            try:
                cmd = input("命令: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == '1o':
                    controller.open_servo1()
                elif cmd == '1c':
                    controller.close_servo1()
                elif cmd == '2o':
                    controller.open_servo2()
                elif cmd == '2c':
                    controller.close_servo2()
                elif cmd == 'ao':
                    controller.open_both()
                elif cmd == 'ac':
                    controller.close_both()
                elif cmd == '':
                    continue
                else:
                    print("错误命令")
                
            except (KeyboardInterrupt, EOFError):
                break
    
    try:
        # 启动交互控制
        control_thread = threading.Thread(target=interactive_control, daemon=True)
        control_thread.start()
        
        # 运行节点
        rclpy.spin(controller)
    
    except KeyboardInterrupt:
        pass
    finally:
        controller.close_both()  # 关闭舵机
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
