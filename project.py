'''import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleGlobalPosition, MissionResult, VehicleStatus, VehicleCommand, TrajectorySetpoint


class OffboardSwitcher(Node):
    def __init__(self):
        super().__init__('offboard_switcher')

        # 设置QoS配置（匹配PX4）
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 订阅者
        self.global_position_sub = self.create_subscription(

        # 发布者
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        self.trajectory_setpoint_pub= self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        self.offboard_active = False

        # 定时器 - 用于Offboard模式下的持续控制
        self.offboard_timer = self.create_timer(0.1, self.offboard_control_callback)

    def arm(self):
        """Arm the vehicle."""
        self.msg = VehicleCommand()
        self.msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.msg.param1 = 1.0
        self.msg.from_external = True
        self.vehicle_command_pub.publish(self.msg)
        self.get_logger().info("Vehicle armed.")

    def disarm(self):
        """Disarm the vehicle."""
        self.msg = VehicleCommand()
        self.msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.msg.param1 = 0.0   # 1.0 for arm, 0.0 for disarm
        self.msg.from_external = True
        self.vehicle_command_pub.publish(self.msg)
        self.get_logger().info("Vehicle disarmed.")

    def offboard_mode(self):
        """Switch to offboard mode."""
        self.msg = VehicleCommand()
        self.msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        self.msg.param1 = 1.0
        self.msg.param2 = 6.0
        self.msg.from_external = True
        self.vehicle_command_pub.publish(self.msg)
        self.offboard_active = True
        self.get_logger().info("Switched to offboard mode.")

    def mission_mode(self):
        """Switch to mission mode."""
        self.msg = VehicleCommand()
        self.msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        self.msg.param1 = 1.0
        self.msg.param2 = 4.0
        self.msg.from_external = True
        self.vehicle_command_pub.publish(self.msg)
        self.get_logger().info("Switched to mission mode.")
    
    def publish_setpoint(self):
        # Publish a setpoint in offboard mode.
        msg = TrajectorySetpoint()
        msg.position = 
        self.get_logger().info(f"Publishing setpoint: {msg.position}")
        self.trajectory_setpoint_pub.publish(msg)

    def offboard_control_callback(self):
        if self.offboard_active:
            self.publish_setpoint()'''

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum
import time

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import MissionResult


class MissionState(Enum):
    """简单的状态机"""
    IDLE = 0
    STARTING_MISSION = 1
    IN_MISSION = 2
    SWITCHING_TO_OFFBOARD1 = 3
    IN_OFFBOARD = 4
    SWITCHING_TO_MISSION = 5
    MISSION_RESUMED = 6
    DONE = 7

class OffboardMissionNode(Node):

    def __init__(self):
        super().__init__('offboard_mission_node')

        # 配置QoS，确保与PX4的通信稳定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1j
        )

        # 创建发布者和订阅者
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.mission_result_sub = self.create_subscription(
            MissionResult, '/fmu/out/mission_result', self.mission_result_callback, qos_profile)
        
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # 状态机和任务参数
        self.state = MissionState.IDLE
        self.vehicle_status = None
        self.mission_result = None
        
        # 目标航点序号 (从0开始计数)
        self.target_waypoint_index1 = 1  # 假设在第2航点后切换
        self.target_waypoint_index2 = 2  # 假设在第3个航点后切换
        self.offboard_task_start_time = None
        self.offboard_task_duration = 10.0  # Offboard 任务持续10秒

        # 创建一个定时器来驱动主逻辑
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz

    def vehicle_status_callback(self, msg):
        """获取飞控状态"""
        self.vehicle_status = msg

    def mission_result_callback(self, msg):
        """获取航线执行结果"""
        self.mission_result = msg
        self.get_logger().info(f"Mission result updated: sequence reached: {msg.seq_reached}")


    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")
        
    def start_mission(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=4.0, param2=3.0)
        self.get_logger().info("Switching to Mission mode")

    def switch_to_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to Offboard mode")
    
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
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


    def publish_offboard_control_mode(self):
        """发布Offboard控制模式"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocieration = False
        msg.attity = False
        msg.acceltude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        """发布轨迹设定点"""
        msg = TrajectorySetpoint()
        # 这里只是一个简单的例子：在当前位置悬停
        # 你可以根据需要修改这里的x, y, z来执行复杂的轨迹
        msg.position = [float('nan'), float('nan'), float('nan')] # NaN表示保持当前值
        msg.yaw = float('nan')
        # 如果要移动到特定点，可以这样设置：
        # msg.position = [10.0, 5.0, -5.0] # NED坐标系
        # msg.yaw = -3.14  # 航向角
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def timer_callback(self):
        """主逻辑循环，由定时器驱动"""
        if self.vehicle_status is None:
            self.get_logger().warn("Waiting for vehicle status...")
            return

        # 状态机逻辑
        if self.state == MissionState.IDLE:
            # 等待飞控连接并准备就绪
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Vehicle is armed. Starting mission.")
                self.state = MissionState.STARTING_MISSION
                self.start_mission()
            else:
                 # 在实际应用中，你可能需要先发送起飞指令，然后开始mission
                 # 这里为简化，假设飞机已在空中或通过QGC起飞
                 # 如果要代码自动arm，可以在这里调用 self.arm()
                 self.get_logger().info("Please arm the vehicle to start.")
                 self.arm()


        elif self.state == MissionState.STARTING_MISSION:
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:
                self.get_logger().info("Successfully switched to Mission mode.")
                self.state = MissionState.IN_MISSION

        elif self.state == MissionState.IN_MISSION:
            if self.mission_result and self.mission_result.seq_reached == self.target_waypoint_index1:
                self.get_logger().info(f"Reached target waypoint {self.target_waypoint_index1}. Switching to Offboard.")
                self.state = MissionState.SWITCHING_TO_OFFBOARD1
        
        elif self.state == MissionState.SWITCHING_TO_OFFBOARD1:
            # **重要**: 进入Offboard模式前必须持续发送设定点
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint() # 先发送一个保持当前位置的指令
            self.switch_to_offboard()
            
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info("Successfully switched to Offboard mode.")
                self.state = MissionState.IN_OFFBOARD
                self.offboard_task_start_time = time.time()

        elif self.state == MissionState.IN_OFFBOARD:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint() # 持续发送指令执行Offboard任务
            
            # 检查Offboard任务是否完成 (例如，持续10秒)
            if time.time() - self.offboard_task_start_time > self.offboard_task_duration:
                self.get_logger().info("Offboard task finished. Switching back to Mission mode.")
                self.state = MissionState.SWITCHING_TO_MISSION
                
        elif self.state == MissionState.SWITCHING_TO_MISSION:
            self.start_mission() # 发送指令切回Mission模式
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:
                self.get_logger().info("Successfully switched back to Mission mode.")
                self.state = MissionState.MISSION_RESUMED

        elif self.state == MissionState.MISSION_RESUMED:
            # 监控任务是否继续或完成
            if self.mission_result and self.mission_result.finished:
                self.get_logger().info("Mission complete!")
                self.state = MissionState.DONE
                # 你可以在这里添加降落或解除锁定的逻辑
                # self.disarm()
                self.destroy_node()
                rclpy.shutdown()

        elif self.state == MissionState.DONE:
            pass # 什么都不做

def main(args=None):
    rclpy.init(args=args)
    node = OffboardMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
    

    