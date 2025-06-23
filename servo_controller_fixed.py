#!/usr/bin/env python3
"""
90度舵机控制程序
专门用于控制0-90度舵机，通过ROS2 MAVLink协议控制PX4飞控

QGC配置要求:
- Channel 1: 设置为 "Peripheral via Actuator Set 1"
- Channel 2: 设置为 "Peripheral via Actuator Set 2" 
- PWM范围: Min=1000us, Max=2000us (对应0-90度)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand
import time
import threading
from std_msgs.msg import Float32


class ServoController(Node):
    """90度舵机控制器类 - 仅使用MAVLink控制"""
    
    def __init__(self):
        super().__init__('servo_controller_90deg')
        
        # 配置QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ROS2发布器 - 用于MAVLink控制
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # ROS2订阅器 - 接收外部控制命令
        self.servo1_angle_subscriber = self.create_subscription(
            Float32, '/servo1_angle', self.servo1_angle_callback, 10)
        self.servo2_angle_subscriber = self.create_subscription(
            Float32, '/servo2_angle', self.servo2_angle_callback, 10)
        self.both_servos_subscriber = self.create_subscription(
            Float32, '/both_servos_angle', self.both_servos_callback, 10)
          # 90度舵机参数 (使用完整PWM范围)
        self.min_pwm = 1000  # 对应0度
        self.max_pwm = 2000  # 对应90度
        
        # 当前角度记录
        self.servo1_current_angle = 0.0
        self.servo2_current_angle = 0.0
        
        self.get_logger().info("90度舵机控制器已启动")
        self.get_logger().info("支持角度范围: 0-90度")
        self.get_logger().info("控制方式: MAVLink (通过飞控)")
        self.get_logger().info("ROS2话题: /servo1_angle, /servo2_angle, /both_servos_angle")
        self.get_logger().info("QGC配置: Channel1/2 -> Peripheral via Actuator Set 1/2")
      def angle_to_pwm(self, angle):
        """将角度(0-90)转换为PWM值"""
        # 限制角度范围
        angle = max(0, min(90, angle))
        # 线性映射: 0度->1000us, 90度->2000us
        pwm_value = self.min_pwm + (angle / 90.0) * (self.max_pwm - self.min_pwm)
        return int(pwm_value)
    
    def publish_vehicle_command(self, command, **params):
        """发布MAVLink命令"""
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
    
    def control_servo_mavlink(self, servo_channel, angle_degrees):
        """使用MAVLink控制90度舵机"""
        # 限制角度范围
        angle_degrees = max(0, min(90, angle_degrees))
        pwm_value = self.angle_to_pwm(angle_degrees)
        
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_SERVO,
            param1=float(servo_channel),  # 舵机通道 (1或2)
            param2=float(pwm_value)       # PWM值
        )
        
        # 更新当前角度记录
        if servo_channel == 1:
            self.servo1_current_angle = angle_degrees
        elif servo_channel == 2:
            self.servo2_current_angle = angle_degrees
            
        self.get_logger().info(f"舵机{servo_channel} 设置为 {angle_degrees}度 (PWM: {pwm_value}us)")
    
    def control_both_servos(self, angle_degrees):
        """同时控制两个舵机到相同角度"""
        self.control_servo_mavlink(1, angle_degrees)
        self.control_servo_mavlink(2, angle_degrees)
        self.get_logger().info(f"两个舵机同时设置为 {angle_degrees}度")
    
    # ROS2回调函数
    def servo1_angle_callback(self, msg):
        """舵机1角度控制回调"""
        angle = msg.data
        self.control_servo_mavlink(1, angle)
    
    def servo2_angle_callback(self, msg):
        """舵机2角度控制回调"""
        angle = msg.data
        self.control_servo_mavlink(2, angle)
    
    def both_servos_callback(self, msg):
        """同时控制两个舵机回调"""
        angle = msg.data
        self.control_both_servos(angle)
    
    # 预设动作函数
    def open_payload_doors(self):
        """打开投放门 (90度)"""
        self.control_both_servos(90)
        self.get_logger().info("投放门已打开")
    
    def close_payload_doors(self):
        """关闭投放门 (0度)"""
        self.control_both_servos(0)
        self.get_logger().info("投放门已关闭")
    
    def half_open_doors(self):
        """半开投放门 (45度)"""
        self.control_both_servos(45)
        self.get_logger().info("投放门半开")
    
    def quarter_open_doors(self):
        """四分之一开启 (22.5度)"""
        self.control_both_servos(22.5)
        self.get_logger().info("投放门四分之一开启")
    
    def three_quarter_open_doors(self):
        """四分之三开启 (67.5度)"""
        self.control_both_servos(67.5)
        self.get_logger().info("投放门四分之三开启")
    
    def get_current_angles(self):
        """获取当前舵机角度"""
        return {
            'servo1': self.servo1_current_angle,
            'servo2': self.servo2_current_angle
        }
    
    def smooth_move_servo(self, servo_number, target_angle, steps=10, delay=0.1):
        """平滑移动舵机到目标角度"""
        current_angle = self.servo1_current_angle if servo_number == 1 else self.servo2_current_angle
        target_angle = max(0, min(90, target_angle))
        
        if current_angle == target_angle:
            return
        
        step_size = (target_angle - current_angle) / steps
        angle = current_angle
        
        self.get_logger().info(f"舵机{servo_number} 平滑移动: {current_angle}° -> {target_angle}°")
        
        for i in range(steps + 1):
            self.control_servo_mavlink(servo_number, angle)
            angle += step_size
            time.sleep(delay)
    
    def test_sequence(self):
        """测试序列 - 展示90度舵机的各种位置"""
        self.get_logger().info("开始90度舵机测试序列...")
        
        # 测试各个角度位置
        test_angles = [0, 22.5, 45, 67.5, 90, 67.5, 45, 22.5, 0]
        
        for angle in test_angles:
            self.get_logger().info(f"测试角度: {angle}度")
            self.control_both_servos(angle)
            time.sleep(1.5)
        
        self.get_logger().info("测试序列完成")
    
    def print_qgc_config_guide(self):
        """打印QGC配置指南"""
        print("\n" + "="*60)
        print("           QGroundControl 舵机配置指南")
        print("="*60)
        print("1. 打开QGroundControl")
        print("2. 进入 Vehicle Setup > Actuators")        print("3. 配置输出通道:")
        print("   📍 Channel 1:")
        print("      - Function: Peripheral via Actuator Set 1")
        print("      - Min PWM: 1000")
        print("      - Max PWM: 2000")
        print("   📍 Channel 2:")
        print("      - Function: Peripheral via Actuator Set 2") 
        print("      - Min PWM: 1000")
        print("      - Max PWM: 2000")
        print("4. 点击 'Apply and Restart'")
        print("5. 重启飞控后配置生效")        print("\n💡 注意事项:")
        print("   - 90度舵机的PWM范围是1000-2000us")
        print("   - 1000us = 0度, 2000us = 90度")
        print("   - 确保舵机电源供应充足(通常5V)")
        print("   - 建议使用外部BEC为舵机供电")
        print("="*60)
    
    def cleanup(self):
        """清理资源"""
        # 将舵机移动到安全位置 (0度)
        self.close_payload_doors()
        self.get_logger().info("舵机已移动到安全位置")


def main():
    """主函数 - 90度舵机交互控制"""
    rclpy.init()
    
    servo_controller = ServoController()
    
    try:
        # 启动一个线程用于交互式控制
        def interactive_control():
            time.sleep(2)  # 等待初始化完成
            print("\n=== 90度舵机控制交互界面 ===")
            print("命令:")
            print("  0 - 关闭投放门 (0度)")
            print("  1 - 四分之一开启 (22.5度)")
            print("  2 - 半开投放门 (45度)")
            print("  3 - 四分之三开启 (67.5度)")
            print("  4 - 完全打开 (90度)")
            print("  t - 执行完整测试序列")
            print("  s1 <角度> - 控制舵机1到指定角度 (0-90)")
            print("  s2 <角度> - 控制舵机2到指定角度 (0-90)")
            print("  sb <角度> - 同时控制两个舵机 (0-90)")
            print("  smooth1 <角度> - 平滑移动舵机1")
            print("  smooth2 <角度> - 平滑移动舵机2")
            print("  status - 显示当前角度")
            print("  config - 显示QGC配置指南")
            print("  q - 退出")
            
            while rclpy.ok():
                try:
                    cmd = input("\n请输入命令: ").strip().lower()
                    
                    if cmd == 'q':
                        break
                    elif cmd == '0':
                        servo_controller.close_payload_doors()
                    elif cmd == '1':
                        servo_controller.quarter_open_doors()
                    elif cmd == '2':
                        servo_controller.half_open_doors()
                    elif cmd == '3':
                        servo_controller.three_quarter_open_doors()
                    elif cmd == '4':
                        servo_controller.open_payload_doors()
                    elif cmd == 't':
                        servo_controller.test_sequence()
                    elif cmd == 'status':
                        angles = servo_controller.get_current_angles()
                        print(f"当前角度 - 舵机1: {angles['servo1']}°, 舵机2: {angles['servo2']}°")
                    elif cmd == 'config':
                        servo_controller.print_qgc_config_guide()
                    elif cmd.startswith('s1 '):
                        try:
                            angle = float(cmd.split()[1])
                            if 0 <= angle <= 90:
                                servo_controller.control_servo_mavlink(1, angle)
                            else:
                                print("角度必须在0-90度范围内")
                        except (IndexError, ValueError):
                            print("格式错误，请使用: s1 <角度> (0-90)")
                    elif cmd.startswith('s2 '):
                        try:
                            angle = float(cmd.split()[1])
                            if 0 <= angle <= 90:
                                servo_controller.control_servo_mavlink(2, angle)
                            else:
                                print("角度必须在0-90度范围内")
                        except (IndexError, ValueError):
                            print("格式错误，请使用: s2 <角度> (0-90)")
                    elif cmd.startswith('sb '):
                        try:
                            angle = float(cmd.split()[1])
                            if 0 <= angle <= 90:
                                servo_controller.control_both_servos(angle)
                            else:
                                print("角度必须在0-90度范围内")
                        except (IndexError, ValueError):
                            print("格式错误，请使用: sb <角度> (0-90)")
                    elif cmd.startswith('smooth1 '):
                        try:
                            angle = float(cmd.split()[1])
                            if 0 <= angle <= 90:
                                servo_controller.smooth_move_servo(1, angle)
                            else:
                                print("角度必须在0-90度范围内")
                        except (IndexError, ValueError):
                            print("格式错误，请使用: smooth1 <角度> (0-90)")
                    elif cmd.startswith('smooth2 '):
                        try:
                            angle = float(cmd.split()[1])
                            if 0 <= angle <= 90:
                                servo_controller.smooth_move_servo(2, angle)
                            else:
                                print("角度必须在0-90度范围内")
                        except (IndexError, ValueError):
                            print("格式错误，请使用: smooth2 <角度> (0-90)")
                    elif cmd == '':
                        continue  # 忽略空输入
                    else:
                        print("未知命令，输入 'config' 查看QGC配置指南")
                
                except KeyboardInterrupt:
                    break
                except EOFError:
                    break
        
        # 启动交互控制线程
        control_thread = threading.Thread(target=interactive_control, daemon=True)
        control_thread.start()
        
        # 运行ROS2节点
        servo_controller.get_logger().info("90度舵机控制器运行中...")
        print("\n💡 QGC配置提示:")        print("📋 Vehicle Setup > Actuators:")
        print("   - Channel 1: 设置为 'Peripheral via Actuator Set 1'")
        print("   - Channel 2: 设置为 'Peripheral via Actuator Set 2'")
        print("   - PWM范围: Min=1000us, Max=2000us")
        print("🔌 确保飞控已连接并配置了舵机输出通道")
        rclpy.spin(servo_controller)
    
    except KeyboardInterrupt:
        pass
    finally:
        servo_controller.cleanup()
        servo_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
