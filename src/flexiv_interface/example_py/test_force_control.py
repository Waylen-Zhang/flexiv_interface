from flexiv_interface.IK.pink_ik import PinkIKSolver
from flexiv_interface.robot_interface.flexiv_interface import FlexivInterface
import time
import numpy as np
from flexiv_interface.admittance.admittance import AdmittanceController
from flexiv_interface.admittance.force_keeping import ForceKeeping
from flexiv_interface.admittance.x_admittance import XAdmittanceController
from flexiv_interface.common.utils import lpf

def test_admittance(robot_sn):
    dt = 0.01
    freq = 1/ dt
    myrobot = FlexivInterface(robot_sn)
    current_pose = np.array(myrobot.get_tcp_pose())
    current_q = np.array(myrobot.get_joint_position())
    target_pose = np.array(current_pose.copy())
    M = np.diag([2, 2, 2, 2, 2, 2])
    D = np.diag([40, 40, 40, 10, 10, 10])
    K = np.diag([400, 400, 400, 5, 5, 5])
    
    admit = AdmittanceController(M=M,D=D,K=K,dt=0.01,
                                 init_pos=current_pose[:3],
                                 init_quat=current_pose[3:][[1,2,3,0]])
    
    solver = PinkIKSolver(
        "/home/waylen/teleai/flexiv_interface/resources/flexiv_Rizon4s_kinematics.urdf",
        ee_frame="flange", visualize=False,
        mesh_dir="/home/waylen/teleai/flexiv_interface/resources",
        init_q=current_q
    )
    jointstate_lpf = lpf(0.1, current_q)
    
    myrobot.zero_ft_sensor()
    time.sleep(0.5)
    myrobot.start_joint_position_control(frequency=freq)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 <30:
        time_start = time.time()
        F_ext = myrobot.get_external_wrench_world()
        pos_new,quat_new = admit.update(target_pose[:3],
                                        target_pose[3:][[1,2,3,0]],
                                        F_ext)

        target_q = solver.solve_ik_pose(pos_new, quat_new)
        send_q = jointstate_lpf.update(target_q)
        myrobot.set_joint_target_positions(send_q)
        time.sleep(max(dt - (time.time()-time_start),0))
    print("DONE")
    myrobot.stop_control()
        
    
def test_x_admittance(robot_sn):
    """
    Do not set robot activate in this case, just check the output
    """
    dt = 0.02
    freq = 1/ dt
    myrobot = FlexivInterface(robot_sn)
    current_pose = np.array(myrobot.get_tcp_pose())
    current_q = np.array(myrobot.get_joint_position())
    target_pose = np.array(current_pose.copy())
    solver = PinkIKSolver(
        "/home/waylen/teleai/flexiv_interface/resources/flexiv_Rizon4s_kinematics.urdf",
        ee_frame="flange", visualize=False,
        mesh_dir="/home/waylen/teleai/flexiv_interface/resources",
        init_q=current_q
    )
    jointstate_lpf = lpf(0.1, current_q)
    
    M = np.diag([2, 1e6, 1e6])
    D = np.diag([80, 0, 0])
    x_admittance = XAdmittanceController(M=M,D=D,dt=dt,init_x=current_pose[:3])
    
    myrobot.zero_ft_sensor()
    time.sleep(0.5)
    myrobot.start_joint_position_control(frequency=freq)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 <300:
        time_start = time.time()
        F_ext = myrobot.get_external_wrench_world()
        new_pos = x_admittance.update(target_pose[:3], F_ext, F_des=10)
        # print(new_pos[0]-current_pose[0],F_ext[0])
        target_q = solver.solve_ik_pose(new_pos, target_pose[3:][[1,2,3,0]])
        # print(target_q-current_q)
        time.sleep(max(dt - (time.time()-time_start),0))
    print("DONE")
    myrobot.stop_control()
    
def test_force_keep(robot_sn):
    dt = 0.01
    freq = 1/ dt
    myrobot = FlexivInterface(robot_sn)
    current_pose = np.array(myrobot.get_tcp_pose())
    current_q = np.array(myrobot.get_joint_position())
    target_pose = np.array(current_pose.copy())
    solver = PinkIKSolver(
        "/home/waylen/teleai/flexiv_interface/resources/flexiv_Rizon4s_kinematics.urdf",
        ee_frame="flange", visualize=False,
        mesh_dir="/home/waylen/teleai/flexiv_interface/resources",
        init_q=current_q
    )
    jointstate_lpf = lpf(0.1, current_q)    
    force_keeper = ForceKeeping(dt, 3, 5, 0.001, 3)
    
    myrobot.zero_ft_sensor()
    time.sleep(0.5)
    myrobot.start_joint_position_control(frequency=freq)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 <300:
        time_start = time.time()
        F_ext = myrobot.get_external_wrench_world()
        new_pos, new_quat = force_keeper.step(target_pose[:3],target_pose[3:][[1,2,3,0]], F_ext[:3])
        # print(new_pos-current_pose[:3])
        target_q = solver.solve_ik_pose(new_pos, new_quat)
        send_q = jointstate_lpf.update(target_q)
        myrobot.set_joint_target_positions(send_q)
        time.sleep(max(dt - (time.time()-time_start),0))
        # print(target_q-current_q)
    print("DONE")
    myrobot.stop_control()
    
   
def test_exit(robot_sn):
    dt = 0.01
    freq = 1/ dt
    myrobot = FlexivInterface(robot_sn)
    current_pose = np.array(myrobot.get_tcp_pose())
    current_q = np.array(myrobot.get_joint_position())
    target_pose = np.array(current_pose.copy())
    solver = PinkIKSolver(
        "/home/waylen/teleai/flexiv_interface/resources/flexiv_Rizon4s_kinematics.urdf",
        ee_frame="flange", visualize=False,
        mesh_dir="/home/waylen/teleai/flexiv_interface/resources",
        init_q=current_q
    )
    jointstate_lpf = lpf(0.1, current_q)    
    force_keeper = ForceKeeping(dt, 3, 5, 0.001, 3)
    
    myrobot.zero_ft_sensor()
    time.sleep(0.5)
    myrobot.start_joint_position_control(frequency=freq)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 <300:
        time_start = time.time()
        F_ext = myrobot.get_external_wrench_world()
        if time.time() - time0 < 20 and time.time() - time0 > 10:
            target_pose[2] = current_pose[2] +0.03 * (time.time() - time0)
        new_pos, new_quat = force_keeper.step(target_pose[:3],target_pose[3:][[1,2,3,0]], F_ext[:3])
        # print(new_pos-current_pose[:3])
        target_q = solver.solve_ik_pose(new_pos, new_quat)
        send_q = jointstate_lpf.update(target_q)
        myrobot.set_joint_target_positions(send_q)
        time.sleep(max(dt - (time.time()-time_start),0))
        # print(target_q-current_q)
    print("DONE")
    myrobot.stop_control()
    
if __name__ == "__main__":
    robot_sn = "Rizon4s-063274"
    # test_admittance(robot_sn)
    # test_x_admittance(robot_sn)
    # test_force_keep(robot_sn)
    test_exit(robot_sn)