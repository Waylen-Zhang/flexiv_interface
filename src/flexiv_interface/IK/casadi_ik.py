#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Zhang Jingwei
# Description: Load local URDF, visualize with MeshCat, and solve IK using Pink

import os
import time
from typing import Optional
import numpy as np
import pinocchio as pin
import casadi
import pinocchio.casadi as cpin
from scipy.spatial.transform import Rotation as R

class CasadiIKSolver:
    def __init__(self, urdf_path, ee_frame="joint7", visualize=False, mesh_dir=None, init_q=None):
        if mesh_dir:
            self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)
        else:
            self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path)
        mesh = self.robot.visual_model.geometryObjects[0].geometry
        mesh.buildConvexRepresentation(True)
        
        if self.robot.model.existFrame(ee_frame):
            self.ee_id = self.robot.model.getFrameId(ee_frame)
            print(f"Using existing frame '{ee_frame}' with id {self.ee_id}")
        else:
            try:
                joint_id = self.robot.model.getJointId(ee_frame)
                frame = pin.Frame(
                    ee_frame,
                    joint_id,
                    pin.SE3.Identity(),  
                    pin.FrameType.OP_FRAME
                )
                self.ee_id = self.robot.model.addFrame(frame)
                print(f"Created new OP_FRAME '{ee_frame}' for joint '{ee_frame}' with id {self.ee_id}")
            except Exception as e:
                print(f"Error creating frame for '{ee_frame}': {e}")
                print("Using last frame as end-effector.")
                self.ee_id = self.robot.model.nframes - 1
        
        self.cmodel = cpin.Model(self.robot.model)
        self.cdata = self.cmodel.createData()
        if init_q is None:
            self.init_data = np.radians([-1.48, 33.7, 79, 1, 0, 60, -158])
        else:
            self.init_data = init_q
        self.q = self.init_data.copy()

        self.cq = casadi.SX.sym("q", self.robot.model.nq, 1) 
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf],
            [casadi.vertcat(self.cdata.oMf[self.ee_id].translation - self.cTf[:3,3])],
        )
        
        rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf],
            [casadi.vertcat(cpin.log3(self.cdata.oMf[self.ee_id].rotation @ self.cTf[:3,:3].T))],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.robot.model.nq)
        self.var_q_last = self.opti.parameter(self.robot.model.nq)   # for smooth
        self.param_tf = self.opti.parameter(4, 4)
        
        translational_cost = casadi.sumsqr(translational_error(self.var_q, self.param_tf))
        rotation_cost = casadi.sumsqr(rotational_error(self.var_q, self.param_tf))
        regularization_cost = casadi.sumsqr(self.var_q - self.init_data)
        smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)
        
        self.opti.subject_to(self.opti.bounded(
            self.robot.model.lowerPositionLimit,
            self.var_q,
            self.robot.model.upperPositionLimit)
        )
        
        self.opti.minimize(50 * translational_cost + rotation_cost + 0.02 * regularization_cost + 0.1 * smooth_cost)

        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 1000,
                'tol': 1e-6
            },
            'print_time': False,
            'calc_lam_p': False
        }
        self.opti.solver("ipopt", opts)
        
        self.visualize = visualize
        if visualize:
            self.viz = None
            self.init_visualizer()
        
        self.dt = 1/100
        
    def init_visualizer(self):
        from pinocchio.visualize import MeshcatVisualizer
        self.viz = MeshcatVisualizer(self.robot.model, 
                                    self.robot.visual_model, 
                                    self.robot.visual_model)
        self.viz.initViewer(open=True)
        self.viz.loadViewerModel()
        self.viz.display(self.q)
            
    def solve_ik(self, target_transform_matrix, current_q=None):
        """Solve IK for given target transformation"""
        if current_q is None:
            current_q = self.q
        
        self.opti.set_initial(self.var_q, current_q)
        self.opti.set_value(self.param_tf, target_transform_matrix)
        self.opti.set_value(self.var_q_last, current_q)  
        
        try:
            sol = self.opti.solve()
            sol_q = sol.value(self.var_q)
            self.q = sol_q  
            
            if self.visualize:
                self.viz.display(self.q)
                
            return sol_q
        except Exception as e:
            print(f"IK求解失败: {e}")
            return current_q
        
    def solve_ik_pose(self, 
                      target_pos: np.ndarray, 
                      target_quat: np.ndarray, 
                      current_q: Optional[np.ndarray] = None,
                      debug_print: bool = False) -> np.ndarray:
        # quat defined as [x,y,z,w]
        transform_matrix = np.eye(4)
        transform_matrix[:3, 3] = target_pos
        r = R.from_quat([target_quat[0], target_quat[1], target_quat[2], target_quat[3]])
        transform_matrix[:3, :3] = r.as_matrix()
        return self.solve_ik(transform_matrix, current_q, debug_print)
    

    def run_demo(self):
        """Circular trajectory demo"""
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
            
    def forward_kinematics(self, q):
        q = np.array(q)
        pin.framesForwardKinematics(self.robot.model, self.robot.data, q)
        ee_transform = self.robot.data.oMf[-1]
        transform_matrix = ee_transform.homogeneous
        
        return transform_matrix
    
    def compute_center_of_mass(self, q):
        self.forward_kinematics(q)
        total_mass = 0
        com = np.zeros(3)
        for i in range(self.robot.model.njoints):
            inertia = self.robot.model.inertias[i]
            mass = inertia.mass
            com_link = self.robot.data.oMi[i].translation 
            total_mass += mass
            com += mass * com_link
            print(i,"com_link",com_link,"mass",mass)

        com /= total_mass
        return com
    
if __name__ == "__main__":
    ik_solver = CasadiIKSolver("/home/waylen/teleai/flexiv_interface/resources/flexiv_Rizon10_kinematics.urdf",
        ee_frame="flange",
        visualize=True,
        mesh_dir="/home/waylen/teleai/flexiv_interface/resources")
    ik_solver.run_demo()