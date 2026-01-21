from flexiv_interface.IK.pink_ik import PinkIKSolver
from flexiv_interface.robot_interface.flexiv_interface import FlexivInterface
import time
import numpy as np
from flexiv_interface.admittance.force_keeping import ForceKeepingWithVelocityCompensation
from flexiv_interface.common.utils import lpf, smooth_make_z_parallel_to_normal


def trajectory(t,initial_pose,update_orientation=None):
    refer_pose = initial_pose.copy()
    if update_orientation is not None:
        refer_pose[3:] = update_orientation
    # target_pose = refer_pose.copy()
    # return target_pose
    if t <10:
        target_pose = refer_pose.copy()
        target_pose[2] -= 0.2 * (t/10)
        return target_pose
    elif t < 300:
        target_pose = refer_pose.copy()
        target_pose[0] = refer_pose[0] - 0.12 * np.sin((t-10)/100*2*np.pi)
        target_pose[1] = refer_pose[1] + 0.22 * np.sin((t-10)/20*2*np.pi)
        target_pose[2] = refer_pose[2] -0.2 
        return target_pose

def clear_board(robot_sn):
    dt = 0.01
    freq = 1/ dt
    myrobot = FlexivInterface(robot_sn)
    current_pose = np.array(myrobot.get_tcp_pose())
    current_q = np.array(myrobot.get_joint_position())
    target_pose = np.array(current_pose.copy())
    solver = PinkIKSolver(
        "flexiv_interface/resources/flexiv_Rizon4s_kinematics.urdf",
        ee_frame="flange", visualize=False,
        mesh_dir="flexiv_interface/resources",
        init_q=current_q
    )
    jointstate_lpf = lpf(0.1, current_q)    
    force_keeper = ForceKeepingWithVelocityCompensation(dt, 3, 10, 0.001, 3,velocity_compensation_gain=1.0)
    
    myrobot.zero_ft_sensor()
    time.sleep(0.5)
    myrobot.start_joint_position_control(frequency=freq)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 <300:
        time_start = time.time()
        F_ext = myrobot.get_external_wrench_world()
        # target_pose[2] = current_pose[2] - 0.2 * np.sin((time.time() - time0)/40*2*np.pi) 
        target_pose = trajectory(time.time() - time0, current_pose)
        curr_vel = myrobot.get_tcp_velocity()
        
        new_pos, new_quat = force_keeper.step(target_pose[:3],target_pose[3:][[1,2,3,0]], F_ext[:3],velocity=curr_vel[:3])
        target_q = solver.solve_ik_pose(new_pos, new_quat)
        send_q = jointstate_lpf.update(target_q)
        myrobot.set_joint_target_positions(send_q)
        time.sleep(max(dt - (time.time()-time_start),0))
        # print(target_q-current_q)
    print("DONE")
    myrobot.stop_control()
    
def clear_board_with_orientation(robot_sn):
    dt = 0.01
    freq = 1/ dt
    myrobot = FlexivInterface(robot_sn)
    current_pose = np.array(myrobot.get_tcp_pose())
    current_q = np.array(myrobot.get_joint_position())
    target_pose = np.array(current_pose.copy())
    solver = PinkIKSolver(
        "flexiv_interface/resources/flexiv_Rizon4s_kinematics.urdf",
        ee_frame="flange", visualize=False,
        mesh_dir="flexiv_interface/resources",
        init_q=current_q
    )
    jointstate_lpf = lpf(0.1, current_q)    
    force_keeper = ForceKeepingWithVelocityCompensation(dt, 3, 10, 0.001, 3,velocity_compensation_gain=1.0)
    
    myrobot.zero_ft_sensor()
    time.sleep(0.5)
    myrobot.start_joint_position_control(frequency=freq)
    time.sleep(0.5)
    
    time0 = time.time()
    last_direction = target_pose[3:][[1,2,3,0]]
    while time.time() - time0 <300:
        time_start = time.time()
        F_ext = myrobot.get_external_wrench_world()
        target_pose = trajectory(time.time() - time0, current_pose)
        curr_vel = myrobot.get_tcp_velocity()
        
        new_pos, new_quat = force_keeper.step(target_pose[:3],target_pose[3:][[1,2,3,0]], F_ext[:3],velocity=curr_vel[:3])
        if force_keeper.in_contact == True and np.linalg.norm(force_keeper.normal_estimator.integral_vector)>1000:
            # normal_direction = force_keeper.normal_estimator.normal
            normal_direction = [0,0,1]
            normal_direction = normal_direction/np.linalg.norm(normal_direction)
            new_quat = smooth_make_z_parallel_to_normal(last_direction,normal_direction,0.01)
            last_direction = new_quat
            
        target_q = solver.solve_ik_pose(new_pos, new_quat)
        send_q = jointstate_lpf.update(target_q)
        myrobot.set_joint_target_positions(send_q)
        time.sleep(max(dt - (time.time()-time_start),0))
    print("DONE")
    myrobot.stop_control()
    
if __name__ == "__main__":
    robot_sn = "Rizon4s-063274"
    clear_board(robot_sn)
    # clear_board_with_orientation(robot_sn)