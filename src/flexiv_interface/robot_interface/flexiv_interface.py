#!/usr/bin/env python
"""
Flexiv interface:
ee control: use xyz + quaternion(wxyz)
joint control: use rad not degree
"""

import time
import threading
import numpy as np
import flexivrdk
from pyrobotiqgripper import RobotiqGripper
from flexiv_interface.robot_interface.robotiq_interface import SafeGripperController


class FlexivInterface:
    def __init__(self, robot_sn, enable_robot=True, enable_gripper=False):

        self.robot_sn = robot_sn
        self.mode = flexivrdk.Mode
        
        self.robot = flexivrdk.Robot(robot_sn)
        
        if self.robot.fault():
            self.robot.ClearFault()
        
        if enable_robot:
            self.robot.Enable()
            while not self.robot.operational():
                time.sleep(0.1)
                
        if enable_gripper:
            self.gripper = SafeGripperController()
        
        self.info = self.robot.info()
        self.DoF = self.info.DoF
        
        self.control_thread = None
        self.stop_event = threading.Event()
        self.target_joint_positions = None
        self.target_cartesian_pose = None
        self.zero_ft_sensor()
    
    def update_states(self):
        return self.robot.states()
    
    def get_joint_position(self):
        return self.robot.states().q.copy()
    
    def get_joint_velocity(self):
        return self.robot.states().dq.copy()
    
    def get_joint_torque(self):
        return self.robot.states().tau.copy()
    
    def get_joint_external_torque(self):
        return self.robot.states().tau_ext.copy()
    
    def get_tcp_pose(self):
        return self.robot.states().tcp_pose.copy()
    
    def get_tcp_velocity(self):
        return self.robot.states().tcp_vel.copy()
    
    def get_external_wrench_tcp(self):
        return self.robot.states().ext_wrench_in_tcp.copy()
    
    def get_external_wrench_world(self):
        return self.robot.states().ext_wrench_in_world.copy()
    
    def get_gripper_distance(self):
        # open: maximum 40mm, close: minimum 0mm
        if hasattr(self, 'gripper'):
            return self.gripper.get_gripper_distance()
        else:
            return None
        
    def get_gripper_state(self):
        # return 1 for closed, 0 for opened
        if hasattr(self, 'gripper'):
            return self.gripper.get_gripper_distance() 
        else:
            return None
    
    def start_joint_position_control(self, frequency=100, max_vel=2.0, max_acc=3.0):
        self.stop_control()
        self.robot.SwitchMode(self.mode.NRT_JOINT_POSITION)
        
        self.control_frequency = frequency
        self.control_period = 1.0 / frequency
        self.max_vel = max_vel
        self.max_acc = max_acc
        
        self.target_joint_positions = self.get_joint_position()
        
        self.stop_event.clear()
        self.control_thread = threading.Thread(target=self._joint_control_loop)
        self.control_thread.start()
    
    def set_joint_target_positions(self, positions):
        if self.target_joint_positions is not None and len(positions) == self.DoF:
            self.target_joint_positions = positions.copy()
    
    def _joint_control_loop(self):
        period = self.control_period
        target_vel = [0.0] * self.DoF
        target_acc = [0.0] * self.DoF
        MAX_VEL = [self.max_vel] * self.DoF
        MAX_ACC = [self.max_acc] * self.DoF
        
        while not self.stop_event.is_set():
            start_time = time.time()
            
            if self.target_joint_positions is not None:
                self.robot.SendJointPosition(
                    self.target_joint_positions, 
                    target_vel, 
                    target_acc,
                    MAX_VEL, 
                    MAX_ACC
                )
            
            elapsed = time.time() - start_time
            if elapsed < period:
                time.sleep(period - elapsed)
    
    def start_cartesian_control(self, frequency=100):
        self.stop_control()
        self.robot.SwitchMode(self.mode.NRT_CARTESIAN_MOTION_FORCE)
        self.robot.SetForceControlAxis([False, False, False, False, False, False])
        
        self.control_frequency = frequency
        self.control_period = 1.0 / frequency
        self.target_cartesian_pose = self.get_tcp_pose()
        
        self.stop_event.clear()
        self.control_thread = threading.Thread(target=self._cartesian_control_loop)
        self.control_thread.start()
    
    def set_cartesian_target_pose(self, pose):
        if self.target_cartesian_pose is not None and len(pose) == 7:
            self.target_cartesian_pose = pose.copy()
    
    def _cartesian_control_loop(self):
        period = self.control_period
        
        while not self.stop_event.is_set():
            start_time = time.time()
            # print(self.target_cartesian_pose)
            if self.target_cartesian_pose is not None:
                self.robot.SendCartesianMotionForce(self.target_cartesian_pose)
            
            elapsed = time.time() - start_time
            if elapsed < period:
                time.sleep(period - elapsed)
    
    def open_gripper(self):
        self.gripper.open_gripper()
        
    def close_gripper(self):
        self.gripper.close_gripper()
        
        
    def gripper_goto(self, position_mm):
        self.gripper.move_gripper(position_mm)
    
    def stop_control(self):
        if self.control_thread is not None and self.control_thread.is_alive():
            self.stop_event.set()
            self.control_thread.join()
    
    def stop_robot(self):
        self.robot.Stop()
    
    def move_to_home(self, plan_name="PLAN-Home"):
        current_mode = self.robot.mode()
        self.robot.SwitchMode(self.mode.NRT_PLAN_EXECUTION)
        self.robot.ExecutePlan(plan_name)
        while self.robot.busy():
            time.sleep(0.1)
        self.robot.SwitchMode(current_mode)
        
    def move_to_home_center(self,target_joint_position=None):
        if target_joint_position is None:
            target_joint_position = np.radians([40.0,-40.0, 0.0, 90.0, 0.0, 40.0, 40.0])
        target_joint_position = np.array(target_joint_position)
        current_mode = self.robot.mode()
        self.start_joint_position_control()
        current_joint_position = np.array(self.get_joint_position())
        delta_position = max(np.abs(current_joint_position-target_joint_position))
        time.sleep(0.5)
        delta_time = delta_position*5 #max 0.2 rad/s
        time_start = time.time()
        while time.time()-time_start < delta_time:
            step_position = (time.time()-time_start) / delta_time * (target_joint_position-current_joint_position) +current_joint_position
            self.set_joint_target_positions(step_position)
            time.sleep(0.01)
    
    def zero_ft_sensor(self):
        current_mode = self.robot.mode()
        self.robot.SwitchMode(self.mode.NRT_PRIMITIVE_EXECUTION)
        self.robot.ExecutePrimitive("ZeroFTSensor", dict())
        while not self.robot.primitive_states()["terminated"]:
            time.sleep(0.1)
        self.robot.SwitchMode(current_mode)
        
    def set_impedance(self):
        new_Kq = np.multiply(self.robot.info().K_q_nom, 0.5)
        self.robot.SetJointImpedance(new_Kq)
        # print(self.robot.info().K_q)
        
    def start_joint_impedance_control(self,frequency=100, max_vel=2.0, max_acc=3.0):
        self.stop_control()
        self.robot.SwitchMode(self.mode.NRT_JOINT_IMPEDANCE)   
        self.control_frequency = frequency
        self.control_period = 1.0 / frequency
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.target_joint_positions = self.get_joint_position()
        
        self.stop_event.clear()
        self.control_thread = threading.Thread(target=self._joint_control_loop)
        self.control_thread.start()
        
    
    def __del__(self):
        self.stop_control()
        