#!/usr/bin/env python3
import json
import time
import redis
import sys
import signal
from robot_interface.flexiv_interface import FlexivInterface  # 假设这是你的Flexiv接口

class ForceToRedis:
    def __init__(self, robot_sn, redis_host='localhost', redis_port=6379, frequency=100):
        self.robot_sn = robot_sn
        self.frequency = frequency
        self.interval = 1.0 / frequency
        
        self.redis_client = redis.Redis(
            host=redis_host,
            port=redis_port,
            db=0,
            decode_responses=True
        )
        
        self.robot = FlexivInterface(robot_sn)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.running = True
        
    def signal_handler(self, signum, frame):
        print(f"\n收到信号 {signum}，正在停止...")
        self.running = False
    
    def format_wrench_data(self, wrench_data):
        if len(wrench_data) >= 6:
            return {
                'timestamp': time.time(),
                'force': {
                    'x': float(wrench_data[0]),
                    'y': float(wrench_data[1]),
                    'z': float(wrench_data[2])
                },
                'torque': {
                    'x': float(wrench_data[3]),
                    'y': float(wrench_data[4]),
                    'z': float(wrench_data[5])
                }
            }
        else:
            # 如果数据格式不匹配，返回默认结构
            return {
                'timestamp': time.time(),
                'force': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'torque': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'error': 'Invalid data format'
            }
    
    def run(self):    
        while self.running:
            try:
                start_time = time.time()
                wrench_data = self.robot.get_external_wrench_tcp()
                
                formatted_data = self.format_wrench_data(wrench_data)
                self.redis_client.set(
                    'wrench_tcp_data',
                    json.dumps(formatted_data),
                    ex=5  # 设置5秒过期，防止数据过期
                )
                
                # 可选：发布到Redis频道，供订阅者使用
                self.redis_client.publish(
                    'wrench_tcp_channel',
                    json.dumps(formatted_data)
                )
                
                # 打印数据（可选）
                # print(f"力数据: {formatted_data['force']}, 力矩: {formatted_data['torque']}")
                
                # 控制频率
                elapsed = time.time() - start_time
                sleep_time = max(0, self.interval - elapsed)
                time.sleep(sleep_time)
                
            except Exception as e:
                print(f"错误: {e}")
                time.sleep(0.1)
        
        print("程序已停止")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='读取力数据并存入Redis')
    parser.add_argument('--robot_sn', default="Rizon4s-063274", help='机器人序列号')
    parser.add_argument('--redis_host', default='localhost', help='Redis主机地址')
    parser.add_argument('--redis_port', default=6379, type=int, help='Redis端口')
    parser.add_argument('--frequency', default=50, type=int, help='采集频率(Hz)')
    
    args = parser.parse_args()
    
    collector = ForceToRedis(
        robot_sn=args.robot_sn,
        redis_host=args.redis_host,
        redis_port=args.redis_port,
        frequency=args.frequency
    )
    
    collector.run()

if __name__ == '__main__':
    main()