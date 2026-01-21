#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Zhang Jingwei
# Description: Load local URDF, visualize with MeshCat, and solve IK using Pink

import numpy as np
import pinocchio as pin
from pinocchio.romeo_wrapper import RomeoWrapper
from pinocchio.robot_wrapper import RobotWrapper
import qpsolvers
import pink
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask
from pink.visualization import start_meshcat_visualizer
from scipy.spatial.transform import Rotation as R
import time
import time
import threading
import socket
import json
from typing import Dict, Iterable, Optional, Union

class PinkIKSolver:
    def __init__(self, urdf_path, ee_frame="ee_link", visualize=False, mesh_dir=None, init_q=None):
        """
        Initialize the arm IK solver.
        Args:
            urdf_path (str): local URDF file path
            ee_frame (str): end-effector frame name
            visualize (bool): whether to open MeshCat viewer
        """
        if mesh_dir:
            self.robot = RobotWrapper.BuildFromURDF(
                urdf_path, 
                package_dirs=[mesh_dir]
            )
        else:
            self.robot = RobotWrapper.BuildFromURDF(urdf_path)
        self.model = self.robot.model
        self.data = self.robot.data

        if init_q is None:
            q0 = np.radians([-1.48, 33.7, 79, 1, 0, 60, -158])
        else:
            q0 = init_q
        self.configuration = pink.Configuration(self.model, self.data, q0)

        if visualize:
            self.viz = start_meshcat_visualizer(
                self.robot) 
            self.viz.display(q0)
        self.ee_frame = ee_frame

        self.ee_task = FrameTask(
            self.ee_frame,
            position_cost=2.0,
            orientation_cost=1.0,
        )
        self.posture_task = PostureTask(cost=1e-3)
        self.posture_task.set_target(self.configuration.q)

        self.tasks = [self.ee_task, self.posture_task]
        self.ee_task.set_target_from_configuration(self.configuration)

        self.solver = "daqp" if "daqp" in qpsolvers.available_solvers else qpsolvers.available_solvers[
            0]
        print(f"[Pink IK] Using solver: {self.solver}")

        self.dt = 1/100
        self.calculate_threshold = 1e-2
        self.max_iter = 100

    def set_target(self, pos, quat_xyzw):
        """
        Set EE target pose.
        Args:
            pos (np.array): 目标位置 [x, y, z]
            quat_xyzw (np.array): 目标四元数, 格式为 [x, y, z, w]
        """
        R = pin.Quaternion(quat_xyzw[3],  # w
                           quat_xyzw[0],  # x
                           quat_xyzw[1],  # y
                           quat_xyzw[2]   # z
                           ).toRotationMatrix()
        
        target = self.ee_task.transform_target_to_world
        target.translation = np.array(pos)
        target.rotation = R
        self.ee_task.set_target(target)

    def step(self):
        """Perform one IK iteration"""
        dq = solve_ik(
            self.configuration,
            self.tasks,
            self.dt,
            solver=self.solver,
            damping=0.05
        )
        self.configuration.integrate_inplace(dq, self.dt)
        if hasattr(self, "viz"):
            self.viz.display(self.configuration.q)
            
    def get_q(self):
        return self.configuration.q
    
    def solve_ik_pose(self, target_pos, target_quat, 
                                    pos_threshold=1e-4,  
                                    ori_threshold=1e-3,  
                                    debug_print=False):   
        # quat defined as [x,y,z,w]
        self.set_target(target_pos, target_quat)
        iter = 0

        while True:
            iter += 1
            self.step() 
            error_vector = self.ee_task.compute_error(self.configuration)
            
            pos_err_norm = np.linalg.norm(error_vector[:3]) 
            ori_err_norm = np.linalg.norm(error_vector[3:]) 
            if pos_err_norm < pos_threshold and ori_err_norm < ori_threshold:
                if debug_print:
                    print(f"Converged in {iter} iterations.")
                    print(f"Final Position Error: {pos_err_norm:.6f} m")
                    print(f"Final Orientation Error: {ori_err_norm:.6f}")
                return self.get_q()
            
            if iter > self.max_iter:
                print(f"Failed to converge after {self.max_iter} iterations.")
                print(f"Current Position Error: {pos_err_norm:.6f} m (Target: {pos_threshold})")
                print(f"Current Orientation Error: {ori_err_norm:.6f} (Target: {ori_threshold})")
                return self.get_q()
    
    def solve_ik(self, 
                 target_transform: np.ndarray,
                 pos_threshold: float = 1e-4,  
                 ori_threshold: float = 1e-3,  
                 debug_print: bool = False,
                 max_iter: int = None) -> np.ndarray:

        if target_transform.shape != (4, 4):
            raise ValueError(f"Transform matrix must be 4x4, got shape {target_transform.shape}")
        pos = target_transform[:3, 3]
        rot_matrix = target_transform[:3, :3]
        quat = pin.Quaternion(rot_matrix)
        target_quat = np.array([quat.x, quat.y, quat.z, quat.w])
        return self.solve_ik_pose(pos, target_quat)

    def run_demo(self):
        """Circular trajectory demo"""
        t = 0.0
        t_start = time.time()

        while True:
            radius = 0.2 
            center = np.array([-0.3, 0.391, 0.429])
            omega = 2 * np.pi / 3.0
            pos = center + \
                np.array([0.0, radius * np.cos(omega * t),
                         radius * np.sin(omega * t)])
            quat = R.from_euler('zyx',[ 3.05727925, -0.02256491,  0.66418082]).as_quat("xyzw")
            self.set_target(pos, quat)
            self.step()
            elapse_time = time.time() - t_start
            time.sleep(max(0.0, self.dt - elapse_time))
            t_start = time.time()
            t += self.dt
            error_vector = self.ee_task.compute_error(self.configuration)
            # print(np.linalg.norm(error_vector[3:]))
    def run_demo2(self):
        t = 0.0
        t_start = time.time()
        radius = 0.2
        center = np.array([-0.3, 0.391, 0.429])
        omega = 2 * np.pi / 3.0  
        target_quat = R.from_euler('zyx',[ 3.05727925, -0.02256491,  0.66418082]).as_quat("xyzw")
        
        try:
            while True:
                pos = center + np.array([
                    0.0, radius * np.cos(omega * t),
                    radius * np.sin(omega * t)
                ])
                
                R_mat = R.from_quat(target_quat).as_matrix()
                T = np.eye(4)
                T[:3, :3] = R_mat
                T[:3, 3] = pos

                self.solve_ik(T)
                elapsed = time.time() - t_start
                sleep_time = max(0.0, self.dt - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                t_start = time.time()
                t += self.dt
                    
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    solver = PinkIKSolver(
        "/home/waylen/teleai/flexiv_interface/resources/flexiv_Rizon10_kinematics.urdf",
        ee_frame="flange",
        visualize=True,
        mesh_dir="/home/waylen/teleai/flexiv_interface/resources"
    )
    udp_ip = "127.0.0.1"
    udp_port = 12345
    run_thread = threading.Thread(target=solver.run_demo2)
    run_thread.start()

