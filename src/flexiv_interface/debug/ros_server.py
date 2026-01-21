#!/usr/bin/env python3
import json
import time
import redis
import sys
import signal
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import (
    WrenchStamped, Wrench, Vector3,
    PoseStamped, Pose, Point, Quaternion,
    TwistStamped, Twist
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64MultiArray, MultiArrayDimension
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

class RedisToROS2MultiTopic(Node):
    def __init__(self, redis_host='localhost', redis_port=6379, frequency=100):
        """
        初始化Redis到ROS2的多topic发布器
        
        Args:
            redis_host: Redis主机地址
            redis_port: Redis端口
            frequency: 发布频率(Hz)
        """
        super().__init__('redis_robot_state_publisher')
        
        self.frequency = frequency
        self.interval = 1.0 / frequency
        
        # 连接到Redis
        try:
            self.redis_client = redis.Redis(
                host=redis_host,
                port=redis_port,
                db=0,
                decode_responses=True,
                socket_connect_timeout=5,
                socket_timeout=5,
                retry_on_timeout=True
            )
            
            if not self.redis_client.ping():
                raise ConnectionError("无法连接到Redis服务器")
            self.get_logger().info(f"成功连接到Redis: {redis_host}:{redis_port}")
            
        except redis.ConnectionError as e:
            self.get_logger().error(f"Redis连接错误: {e}")
            raise
        
        # 创建多个ROS2发布器
        self.create_publishers()
        
        # 设置定时器
        self.timer = self.create_timer(self.interval, self.publish_all_callback)
        
        # 统计信息
        self.message_counts = {
            'joint_state': 0,
            'tcp_pose': 0,
            'tcp_velocity': 0,
            'wrench_tcp': 0,
            'wrench_world': 0,
            'diagnostic': 0
        }
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        self.running = True
        
        self.get_logger().info(f"Redis到ROS2多topic发布器已启动，频率: {frequency} Hz")
        
    def create_publishers(self):
        """创建所有ROS2发布器"""
        # 关节状态发布器
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # TCP位姿发布器
        self.tcp_pose_pub = self.create_publisher(
            PoseStamped,
            '/tcp_pose',
            10
        )
        
        # TCP速度发布器
        self.tcp_velocity_pub = self.create_publisher(
            TwistStamped,
            '/tcp_velocity',
            10
        )
        
        # TCP受力发布器（TCP坐标系）
        self.wrench_tcp_pub = self.create_publisher(
            WrenchStamped,
            '/external_wrench_tcp',
            10
        )
        
        # 世界坐标系受力发布器
        self.wrench_world_pub = self.create_publisher(
            WrenchStamped,
            '/external_wrench_world',
            10
        )
        
        # 关节位置数组发布器（备用格式）
        self.joint_position_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_positions',
            10
        )
        
        # 关节速度数组发布器（备用格式）
        self.joint_velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_velocities',
            10
        )
        
        # 关节扭矩数组发布器（备用格式）
        self.joint_torque_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_torques',
            10
        )
        
        # 诊断信息发布器
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # 关节外部扭矩发布器
        self.joint_external_torque_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_external_torques',
            10
        )
        
        self.get_logger().info("所有ROS2发布器创建完成")
        
    def signal_handler(self, signum, frame):
        """处理中断信号"""
        self.get_logger().info(f"收到信号 {signum}，正在停止...")
        self.running = False
        rclpy.shutdown()
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        将欧拉角转换为四元数
        
        Args:
            roll: X轴旋转角度 (rad)
            pitch: Y轴旋转角度 (rad)
            yaw: Z轴旋转角度 (rad)
            
        Returns:
            Quaternion对象
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        
        return q
    
    def create_joint_state_msg(self, state_data):
        """创建JointState消息"""
        msg = JointState()
        
        # 设置时间戳
        current_time = self.get_clock().now()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = state_data.get('header', {}).get('frame_id', 'robot_base')
        
        # 设置关节名称（假设为关节1, 关节2, ...）
        num_joints = state_data['joint_state']['num_joints']
        msg.name = [f'joint_{i+1}' for i in range(num_joints)]
        
        # 设置位置、速度、力矩
        msg.position = [float(p) for p in state_data['joint_state']['position']]
        msg.velocity = [float(v) for v in state_data['joint_state']['velocity']]
        msg.effort = [float(t) for t in state_data['joint_state']['torque']]
        
        return msg
    
    def create_tcp_pose_msg(self, state_data):
        """创建TCP位姿消息"""
        msg = PoseStamped()
        
        # 设置时间戳
        current_time = self.get_clock().now()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = state_data.get('header', {}).get('frame_id', 'robot_base')
        
        # 设置位置
        position = state_data['tcp_state']['pose']['position']
        msg.pose.position = Point(x=float(position[0]), 
                                  y=float(position[1]), 
                                  z=float(position[2]))
        
        # 设置四元数（从欧拉角转换）
        orientation_euler = state_data['tcp_state']['pose']['orientation_euler']
        msg.pose.orientation = self.euler_to_quaternion(
            orientation_euler[0],  # roll
            orientation_euler[1],  # pitch
            orientation_euler[2]   # yaw
        )
        
        return msg
    
    def create_tcp_velocity_msg(self, state_data):
        """创建TCP速度消息"""
        msg = TwistStamped()
        
        # 设置时间戳
        current_time = self.get_clock().now()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = state_data.get('header', {}).get('frame_id', 'robot_base')
        
        # 设置线速度
        linear = state_data['tcp_state']['velocity']['linear']
        msg.twist.linear = Vector3(x=float(linear[0]), 
                                    y=float(linear[1]), 
                                    z=float(linear[2]))
        
        # 设置角速度
        angular = state_data['tcp_state']['velocity']['angular']
        msg.twist.angular = Vector3(x=float(angular[0]), 
                                     y=float(angular[1]), 
                                     z=float(angular[2]))
        
        return msg
    
    def create_wrench_msg(self, state_data, frame='tcp'):
        """创建受力消息"""
        msg = WrenchStamped()
        
        # 设置时间戳
        current_time = self.get_clock().now()
        msg.header.stamp = current_time.to_msg()
        
        if frame == 'tcp':
            msg.header.frame_id = 'tcp_frame'
            wrench_data = state_data['force_state']['wrench_tcp']
        else:  # world frame
            msg.header.frame_id = 'world'
            wrench_data = state_data['force_state']['wrench_world']
        
        # 设置力
        force = wrench_data['force']
        msg.wrench.force = Vector3(x=float(force[0]), 
                                    y=float(force[1]), 
                                    z=float(force[2]))
        
        # 设置力矩
        torque = wrench_data['torque']
        msg.wrench.torque = Vector3(x=float(torque[0]), 
                                     y=float(torque[1]), 
                                     z=float(torque[2]))
        
        return msg
    
    def create_float64_multiarray(self, data_list, name='data'):
        msg = Float64MultiArray()
        msg.data = [float(d) for d in data_list]
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = name
        msg.layout.dim[0].size = len(data_list)
        msg.layout.dim[0].stride = len(data_list)
        
        return msg
    
    def create_diagnostic_msg(self, state_data):
        diag_array = DiagnosticArray()
        
        current_time = self.get_clock().now()
        diag_array.header.stamp = current_time.to_msg()
        diag_array.header.frame_id = 'diagnostics'

        robot_status = DiagnosticStatus()
        robot_status.name = 'robot_state'
        robot_status.hardware_id = state_data.get('header', {}).get('robot_sn', 'unknown')

        system_state = state_data['system_state']
        if system_state.get('fault', False):
            robot_status.level = DiagnosticStatus.ERROR
            robot_status.message = 'Robot in fault state'
        elif not system_state.get('operational', False):
            robot_status.level = DiagnosticStatus.WARN
            robot_status.message = 'Robot not operational'
        else:
            robot_status.level = DiagnosticStatus.OK
            robot_status.message = 'Robot operational'
        
        robot_status.values.extend([
            KeyValue(key='mode', value=str(system_state.get('mode', 'unknown'))),
            KeyValue(key='busy', value=str(system_state.get('busy', False))),
            KeyValue(key='update_rate', value=f"{system_state.get('update_rate', 0.0):.1f} Hz"),
            KeyValue(key='num_joints', value=str(state_data['joint_state']['num_joints'])),
            KeyValue(key='timestamp', value=f"{state_data['timestamp']:.6f}")
        ])
        
        diag_array.status.append(robot_status)
        
        return diag_array
    
    def publish_all_callback(self):
        if not self.running:
            return
            
        try:
            try:
                self.redis_client.ping()
            except:
                self.get_logger().error("Redis连接丢失")
                return
            
            data_str = self.redis_client.get('robot_full_state')
            
            if not data_str:
                self.get_logger().warn("未获取到完整状态数据，尝试获取独立数据...")
                return
            
            try:
                state_data = json.loads(data_str)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON解析错误: {e}")
                return
            
            current_time = time.time()
            
            joint_msg = self.create_joint_state_msg(state_data)
            self.joint_state_pub.publish(joint_msg)
            self.message_counts['joint_state'] += 1
            
            tcp_pose_msg = self.create_tcp_pose_msg(state_data)
            self.tcp_pose_pub.publish(tcp_pose_msg)
            self.message_counts['tcp_pose'] += 1
            
            tcp_velocity_msg = self.create_tcp_velocity_msg(state_data)
            self.tcp_velocity_pub.publish(tcp_velocity_msg)
            self.message_counts['tcp_velocity'] += 1
            
            wrench_tcp_msg = self.create_wrench_msg(state_data, frame='tcp')
            self.wrench_tcp_pub.publish(wrench_tcp_msg)
            self.message_counts['wrench_tcp'] += 1

            wrench_world_msg = self.create_wrench_msg(state_data, frame='world')
            self.wrench_world_pub.publish(wrench_world_msg)
            self.message_counts['wrench_world'] += 1
            
            joint_pos_msg = self.create_float64_multiarray(
                state_data['joint_state']['position'], 'joint_positions'
            )
            self.joint_position_pub.publish(joint_pos_msg)
            
            joint_vel_msg = self.create_float64_multiarray(
                state_data['joint_state']['velocity'], 'joint_velocities'
            )
            self.joint_velocity_pub.publish(joint_vel_msg)
            
            joint_torque_msg = self.create_float64_multiarray(
                state_data['joint_state']['torque'], 'joint_torques'
            )
            self.joint_torque_pub.publish(joint_torque_msg)
            
            joint_ext_torque_msg = self.create_float64_multiarray(
                state_data['joint_state']['external_torque'], 'joint_external_torques'
            )
            self.joint_external_torque_pub.publish(joint_ext_torque_msg)
            if self.message_counts['diagnostic'] % self.frequency == 0:
                diag_msg = self.create_diagnostic_msg(state_data)
                self.diagnostic_pub.publish(diag_msg)
            
            self.message_counts['diagnostic'] += 1
            if sum(self.message_counts.values()) % (self.frequency * 10) == 0:
                self.get_logger().info(
                    f"发布统计: "
                    f"关节状态={self.message_counts['joint_state']}, "
                    f"TCP位姿={self.message_counts['tcp_pose']}, "
                    f"TCP速度={self.message_counts['tcp_velocity']}"
                )
                
        except KeyError as e:
            self.get_logger().error(f"数据字段缺失: {e}")
        except Exception as e:
            self.get_logger().error(f"发布错误: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='从Redis读取机器人状态并发布到多个ROS2 topic')
    parser.add_argument('--redis_host', default='localhost', help='Redis主机地址')
    parser.add_argument('--redis_port', default=6379, type=int, help='Redis端口')
    parser.add_argument('--frequency', default=100, type=int, help='发布频率(Hz)')
    # parser.add_argument('--debug', action='store_true', help='启用调试模式')
    
    args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建节点
    publisher = RedisToROS2MultiTopic(
        redis_host=args.redis_host,
        redis_port=args.redis_port,
        frequency=args.frequency
    )
    
    try:
        # 运行节点
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        publisher.destroy_node()
        rclpy.shutdown()
        print("\n程序已停止")
        print("发布统计:")
        for key, value in publisher.message_counts.items():
            print(f"  {key}: {value} 次")

if __name__ == '__main__':
    main()