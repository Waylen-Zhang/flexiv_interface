#!/usr/bin/env python3
import json
import time
import redis
import sys
import signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3

class RedisToROS2(Node):
    def __init__(self, redis_host='localhost', redis_port=6379, frequency=100):
        """
        初始化Redis到ROS2的发布器
        
        Args:
            redis_host: Redis主机地址
            redis_port: Redis端口
            frequency: 发布频率(Hz)
        """
        super().__init__('redis_wrench_publisher')
        
        self.frequency = frequency
        self.interval = 1.0 / frequency
        
        # 连接到Redis
        self.redis_client = redis.Redis(
            host=redis_host,
            port=redis_port,
            db=0,
            decode_responses=True
        )
        
        # 创建ROS2发布器
        self.publisher = self.create_publisher(
            WrenchStamped,
            '/external_wrench_tcp',
            10  # QoS队列深度
        )
        
        # 设置定时器
        self.timer = self.create_timer(self.interval, self.publish_callback)
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        self.running = True
        
        self.get_logger().info(f"Redis到ROS2转换器已启动，频率: {frequency} Hz")
        
    def signal_handler(self, signum, frame):
        """处理中断信号"""
        self.get_logger().info(f"收到信号 {signum}，正在停止...")
        self.running = False
        rclpy.shutdown()
    
    def parse_wrench_data(self, data_str):
        """
        从JSON字符串解析力数据
        
        Args:
            data_str: JSON格式的字符串
            
        Returns:
            解析后的字典或None
        """
        try:
            return json.loads(data_str)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON解析错误: {e}")
            return None
    
    def create_wrench_stamped(self, wrench_data):
        """
        创建WrenchStamped消息
        
        Args:
            wrench_data: 包含力/力矩数据的字典
            
        Returns:
            WrenchStamped消息
        """
        msg = WrenchStamped()
        
        # 设置时间戳
        current_time = self.get_clock().now()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'tcp_frame'  # 根据实际情况修改
        
        # 设置力数据
        if 'force' in wrench_data:
            msg.wrench.force = Vector3()
            msg.wrench.force.x = float(wrench_data['force'].get('x', 0.0))
            msg.wrench.force.y = float(wrench_data['force'].get('y', 0.0))
            msg.wrench.force.z = float(wrench_data['force'].get('z', 0.0))
        
        # 设置力矩数据
        if 'torque' in wrench_data:
            msg.wrench.torque = Vector3()
            msg.wrench.torque.x = float(wrench_data['torque'].get('x', 0.0))
            msg.wrench.torque.y = float(wrench_data['torque'].get('y', 0.0))
            msg.wrench.torque.z = float(wrench_data['torque'].get('z', 0.0))
        
        return msg
    
    def publish_callback(self):
        if not self.running:
            return
            
        try:
            # 从Redis获取数据
            data_str = self.redis_client.get('wrench_tcp_data')
            
            if data_str:
                # 解析数据
                wrench_data = self.parse_wrench_data(data_str)
                
                if wrench_data:
                    # 创建并发布ROS2消息
                    msg = self.create_wrench_stamped(wrench_data)
                    self.publisher.publish(msg)
                    
                    # 可选：记录日志
                    self.get_logger().debug(
                        f"发布力数据: F=[{msg.wrench.force.x:.3f}, "
                        f"{msg.wrench.force.y:.3f}, {msg.wrench.force.z:.3f}], "
                        f"T=[{msg.wrench.torque.x:.3f}, "
                        f"{msg.wrench.torque.y:.3f}, {msg.wrench.torque.z:.3f}]"
                    )
            else:
                self.get_logger().warn("未从Redis获取到数据")
                
        except redis.ConnectionError as e:
            self.get_logger().error(f"Redis连接错误: {e}")
        except Exception as e:
            self.get_logger().error(f"发布错误: {e}")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='从Redis读取数据并发布到ROS2')
    parser.add_argument('--redis_host', default='localhost', help='Redis主机地址')
    parser.add_argument('--redis_port', default=6379, type=int, help='Redis端口')
    parser.add_argument('--frequency', default=50, type=int, help='发布频率(Hz)')
    
    args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建节点
    publisher = RedisToROS2(
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
        print("程序已停止")

if __name__ == '__main__':
    main()