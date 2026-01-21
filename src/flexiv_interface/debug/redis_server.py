#!/usr/bin/env python3
import json
import time
import redis
import sys
import signal
import threading
from  flexiv_interface.robot_interface.flexiv_interface import FlexivInterface  # 假设这是你的Flexiv接口

class StateToRedis:
    def __init__(self, robot_sn, redis_host='localhost', redis_port=6379, frequency=100):
        self.robot_sn = robot_sn
        self.frequency = frequency
        self.interval = 1.0 / frequency
        self.doF = None  # 将在初始化机器人后设置
        
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
            print(f"成功连接到Redis: {redis_host}:{redis_port}")
            
        except redis.ConnectionError as e:
            print(f"Redis连接错误: {e}")
            raise
        
        # 初始化机器人接口
        try:
            self.robot = FlexivInterface(robot_sn)
            self.doF = self.robot.DoF
            print(f"成功连接到机器人: {robot_sn}, DoF: {self.doF}")
        except Exception as e:
            print(f"机器人连接错误: {e}")
            raise
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.running = True
        self.last_update_time = time.time()
        
    def signal_handler(self, signum, frame):
        print(f"\n收到信号 {signum}，正在停止...")
        self.running = False
    
    def format_state_data(self):
        timestamp = time.time()
        states = self.robot.update_states()
        
        joint_position = self.robot.get_joint_position()
        joint_velocity = self.robot.get_joint_velocity()
        joint_torque = self.robot.get_joint_torque()
        joint_external_torque = self.robot.get_joint_external_torque()
        tcp_pose = self.robot.get_tcp_pose()
        tcp_velocity = self.robot.get_tcp_velocity()
        external_wrench_tcp = self.robot.get_external_wrench_tcp()
        external_wrench_world = self.robot.get_external_wrench_world()
        
        state_data = {
            'timestamp': timestamp,
            'header': {
                'seq': int(timestamp * 1000),  
                'frame_id': 'robot_base',
                'robot_sn': self.robot_sn
            },
            'joint_state': {
                'position': [float(p) for p in joint_position],
                'velocity': [float(v) for v in joint_velocity],
                'torque': [float(t) for t in joint_torque],
                'external_torque': [float(et) for et in joint_external_torque],
                'num_joints': self.doF
            },
            'tcp_state': {
                'pose': {
                    'position': [float(p) for p in tcp_pose[:3]],
                    'orientation_euler': [float(o) for o in tcp_pose[3:6]]
                },
                'velocity': {
                    'linear': [float(v) for v in tcp_velocity[:3]],
                    'angular': [float(w) for w in tcp_velocity[3:6]]
                }
            },
            'force_state': {
                'wrench_tcp': {
                    'force': [float(f) for f in external_wrench_tcp[:3]],
                    'torque': [float(t) for t in external_wrench_tcp[3:6]]
                },
                'wrench_world': {
                    'force': [float(f) for f in external_wrench_world[:3]],
                    'torque': [float(t) for t in external_wrench_world[3:6]]
                }
            },
            'system_state': {
                'operational': self.robot.robot.operational(),
                'fault': self.robot.robot.fault(),
                'mode': str(self.robot.robot.mode()),
                'busy': self.robot.robot.busy(),
                'update_rate': 1.0 / (timestamp - self.last_update_time)
            }
        }
        
        self.last_update_time = timestamp
        return state_data
    
    def save_to_redis(self, state_data):
        try:
            self.redis_client.set(
                'robot_full_state',
                json.dumps(state_data),
                ex=5 
            )
            
            self.redis_client.set(
                'robot_joint_state',
                json.dumps(state_data['joint_state']),
                ex=5
            )
            
            self.redis_client.set(
                'robot_tcp_state',
                json.dumps(state_data['tcp_state']),
                ex=5
            )
            
            self.redis_client.set(
                'robot_force_state',
                json.dumps(state_data['force_state']),
                ex=5
            )
            
            self.redis_client.publish(
                'robot_state_channel',
                json.dumps(state_data)
            )
            
            self.redis_client.set(
                'robot_last_update',
                state_data['timestamp'],
                ex=5
            )
            
        except Exception as e:
            print(f"Redis保存错误: {e}")
    
    def print_debug_info(self, state_data):
        print("\n=== 机器人状态数据 ===")
        print(f"时间戳: {state_data['timestamp']:.6f}")
        print(f"关节数: {state_data['joint_state']['num_joints']}")
        
        if state_data['joint_state']['num_joints'] > 0:
            print(f"关节1 - 位置: {state_data['joint_state']['position'][0]:.4f} rad, "
                  f"速度: {state_data['joint_state']['velocity'][0]:.4f} rad/s, "
                  f"扭矩: {state_data['joint_state']['torque'][0]:.4f} Nm")
        
        print(f"TCP位置: [{state_data['tcp_state']['pose']['position'][0]:.3f}, "
              f"{state_data['tcp_state']['pose']['position'][1]:.3f}, "
              f"{state_data['tcp_state']['pose']['position'][2]:.3f}] m")
        
        force_tcp = state_data['force_state']['wrench_tcp']['force']
        print(f"TCP受力: [{force_tcp[0]:.3f}, {force_tcp[1]:.3f}, {force_tcp[2]:.3f}] N")

        print(f"更新率: {state_data['system_state']['update_rate']:.1f} Hz")
        print("=" * 40)
    
    def run(self):
        """主循环，读取机器人状态并存入Redis"""
        print(f"开始以 {self.frequency} Hz 的频率读取机器人状态...")
        print("按 Ctrl+C 停止\n")
        
        cycle_count = 0
        print_interval = 50  
        
        while self.running:
            try:
                start_time = time.time()
                state_data = self.format_state_data()
                self.save_to_redis(state_data)
                cycle_count += 1
                if cycle_count % print_interval == 0:
                    self.print_debug_info(state_data)
                elapsed = time.time() - start_time
                sleep_time = max(0, self.interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    print(f"警告: 循环超时 {abs(sleep_time):.6f} 秒")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"错误: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
        
        print("\n程序已停止")
        
        if hasattr(self, 'robot'):
            self.robot.stop_control()
            print("机器人控制已停止")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='读取机器人状态数据并存入Redis')
    parser.add_argument('--robot_sn', default="Rizon4s-063274", help='机器人序列号')
    parser.add_argument('--redis_host', default='localhost', help='Redis主机地址')
    parser.add_argument('--redis_port', default=6379, type=int, help='Redis端口')
    parser.add_argument('--frequency', default=100, type=int, help='采集频率(Hz)')
    parser.add_argument('--debug', action='store_true', help='启用详细调试输出')
    
    args = parser.parse_args()
    
    collector = StateToRedis(
        robot_sn=args.robot_sn,
        redis_host=args.redis_host,
        redis_port=args.redis_port,
        frequency=args.frequency
    )
    
    collector.run()

if __name__ == '__main__':
    main()