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
    
    def publish_vehicle_command(self, command, **params) -> None:
        """发布MAVLink命令 - 通用函数（参考主飞控）"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def send_vehicle_command(self, servo_id, normalized_value):
        """发送舵机控制命令 - 符合MAVLink标准"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 187  # MAV_CMD_DO_SET_ACTUATOR
        
        # 根据MAVLink文档，使用标准化值 [-1.0 到 1.0]
        if servo_id == 1:
            msg.param1 = float(normalized_value)  # Actuator 1 value
            msg.param2 = float('nan')             # Actuator 2 (未使用)
        elif servo_id == 2:
            msg.param1 = float('nan')             # Actuator 1 (未使用)
            msg.param2 = float(normalized_value)  # Actuator 2 value
        
        msg.param3 = float('nan')  # Actuator 3 (未使用)
        msg.param4 = float('nan')  # Actuator 4 (未使用)
        msg.param5 = float('nan')  # Actuator 5 (未使用)
        msg.param6 = float('nan')  # Actuator 6 (未使用)
        msg.param7 = float('nan')  # Index (未使用)
        
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_publisher.publish(msg)
        # 计算对应的PWM值用于日志显示
        pwm_value = 1500 + (normalized_value * 500)  # -1.0→1000us, 1.0→2000us
        self.get_logger().info(f"舵机{servo_id} 标准化值={normalized_value:.1f} (对应PWM≈{pwm_value:.0f}us)")
    
    def drop_payload(self):
        """投放载荷 - 参考主飞控逻辑"""
        self.get_logger().info("---------------Payload dropped.-------------------")
        # 打开投放门
        self.open_both()
    
    def control_servo_standard(self, servo_id, normalized_value):
        """使用标准MAVLink方式控制舵机"""
        if servo_id == 1:
            self.publish_vehicle_command(
                187,  # MAV_CMD_DO_SET_ACTUATOR
                param1=float(normalized_value),
                param2=float('nan'),
                param3=float('nan'),
                param4=float('nan'),
                param5=float('nan'),
                param6=float('nan'),
                param7=float('nan')
            )
        elif servo_id == 2:
            self.publish_vehicle_command(
                187,  # MAV_CMD_DO_SET_ACTUATOR
                param1=float('nan'),
                param2=float(normalized_value),
                param3=float('nan'),
                param4=float('nan'),
                param5=float('nan'),
                param6=float('nan'),
                param7=float('nan')
            )
        
        pwm_value = 1500 + (normalized_value * 500)
        self.get_logger().info(f"舵机{servo_id} 标准控制={normalized_value:.1f} (PWM≈{pwm_value:.0f}us)")
    
    def open_servo1(self):
        """舵机1完全打开(90度)"""
        self.send_vehicle_command(1, 1.0)  # 1.0 = 2000us PWM
    
    def close_servo1(self):
        """舵机1完全关闭(0度)"""
        self.send_vehicle_command(1, -1.0)  # -1.0 = 1000us PWM
    
    def open_servo2(self):
        """舵机2完全打开(90度)"""
        self.send_vehicle_command(2, 1.0)  # 1.0 = 2000us PWM
    
    def close_servo2(self):
        """舵机2完全关闭(0度)"""
        self.send_vehicle_command(2, -1.0)  # -1.0 = 1000us PWM
    
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
