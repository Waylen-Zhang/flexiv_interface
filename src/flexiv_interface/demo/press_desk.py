from flexiv_interface.IK.pink_ik import PinkIKSolver
from flexiv_interface.robot_interface.flexiv_interface import FlexivInterface
import time
import numpy as np
from flexiv_interface.admittance.force_keeping import ForceKeeping
from flexiv_interface.common.utils import lpf

def press_desk(robot_sn):
    dt = 0.01
    freq = 1/ dt
    myrobot = FlexivInterface(robot_sn)
    current_pose = np.array(myrobot.get_tcp_pose())
    current_q = np.array(myrobot.get_joint_position())
    target_pose = np.array(current_pose.copy())
    solver = PinkIKSolver(
        "resources/flexiv_Rizon4s_kinematics.urdf",
        ee_frame="flange", visualize=False,
        mesh_dir="resources",
        init_q=current_q
    )
    jointstate_lpf = lpf(0.1, current_q)    
    force_keeper = ForceKeeping(dt, 3, 10, 0.001, 3)
    
    myrobot.zero_ft_sensor()
    time.sleep(0.5)
    myrobot.start_joint_position_control(frequency=freq)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 <300:
        time_start = time.time()
        F_ext = myrobot.get_external_wrench_world()
        target_pose[2] = current_pose[2] - 0.2 * np.sin((time.time() - time0)/40*2*np.pi) 
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
    press_desk(robot_sn)