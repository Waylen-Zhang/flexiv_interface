from flexiv_interface.IK.pink_ik import PinkIKSolver
from flexiv_interface.robot_interface.flexiv_interface import FlexivInterface
import time
import numpy as np


def test_ik(robot_sn):
    myrobot = FlexivInterface(robot_sn)
    current_pose = np.array(myrobot.get_tcp_pose())
    current_q = np.array(myrobot.get_joint_position())
    target_pose = np.array(current_pose.copy())
    
    solver = PinkIKSolver(
        "/home/waylen/teleai/flexiv_interface/resources/flexiv_Rizon4s_kinematics.urdf",
        ee_frame="flange",
        visualize=False,
        mesh_dir="/home/waylen/teleai/flexiv_interface/resources",
        init_q=current_q
    )
    
    myrobot.start_joint_position_control(frequency=100)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 < 30:
        target_pose[1] = current_pose[1] + 0.05*np.sin(time.time()-time0)
        target_q = solver.solve_ik_pose(target_pose[:3],target_pose[3:][[1,2,3,0]])
        myrobot.set_joint_target_positions(target_q)
    print("DONE")
    myrobot.stop_control()
    
if __name__ == "__main__":
    robot_sn = "Rizon4s-063274"
    test_ik(robot_sn)