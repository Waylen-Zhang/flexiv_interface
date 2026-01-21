from flexiv_interface.robot_interface.flexiv_interface import FlexivInterface
import time
import numpy as np

# before running the tests, give gripper 777 permission:
# sudo chmod 777 /dev/ttyUSB0
def test_read(robot_sn):
    myrobot = FlexivInterface(robot_sn)
    external_force = myrobot.get_external_wrench_tcp()
    print(external_force)
    myrobot.stop_control()
    
def test_write_ee(robot_sn):
    myrobot = FlexivInterface(robot_sn)
    current_pose = myrobot.get_tcp_pose()
    target_pose = current_pose.copy()
    target_pose[0] += 0.03
    
    myrobot.start_cartesian_control(frequency=100)
    time.sleep(0.5)
    myrobot.set_cartesian_target_pose(target_pose)
    print("DONE")
    myrobot.stop_control()
    
def test_write_joint(robot_sn):
    myrobot = FlexivInterface(robot_sn)
    current_q = myrobot.get_joint_position()
    target_q = current_q.copy()
    target_q[-1] += 0.1
    
    myrobot.start_joint_position_control(frequency=100)
    time.sleep(0.5)
    myrobot.set_joint_target_positions(target_q)
    print("DONE")
    myrobot.stop_control()
    
def test_follow_ee_trajectory(robot_sn):
    myrobot = FlexivInterface(robot_sn)
    current_pose = myrobot.get_tcp_pose()
    target_pose = current_pose.copy()
    myrobot.start_cartesian_control(frequency=100)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 < 30:
        target_pose[1] = current_pose[1] + 0.05*np.sin(time.time()-time0)
        myrobot.set_cartesian_target_pose(target_pose)
    print("DONE")
    myrobot.stop_control()


def test_follow_joint_trajectory(robot_sn):
    myrobot = FlexivInterface(robot_sn)
    current_q = myrobot.get_joint_position()
    target_q = current_q.copy()
    myrobot.start_joint_position_control(frequency=100)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 < 30:
        target_q[-1] = current_q[-1] + 0.3*np.sin(time.time()-time0)
        myrobot.set_joint_target_positions(target_q)
    print("DONE")
    myrobot.stop_control()
    
def test_impedance_joint_trajectory(robot_sn):
    myrobot = FlexivInterface(robot_sn)
    current_q = myrobot.get_joint_position()
    target_q = current_q.copy()
    myrobot.start_joint_impedance_control(frequency=100)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 < 30:
        target_q[-1] = current_q[-1] + 0.3*np.sin(time.time()-time0)
        myrobot.set_joint_target_positions(target_q)
    print("DONE")
    myrobot.stop_control()
    
def test_gripper(robot_sn):
    myrobot = FlexivInterface(robot_sn, enable_gripper=False)
    # time.sleep(2)
    # myrobot.close_gripper()
    # time.sleep(2)
    # myrobot.open_gripper()
    # time.sleep(2)
    # myrobot.gripper_goto(100)
    time.sleep(2)
    print("DONE")
    myrobot.stop_control()
    
def test_async_gripper(robot_sn):
    dt = 0.02
    myrobot = FlexivInterface(robot_sn, enable_gripper=True)
    current_q = myrobot.get_joint_position()
    target_q = current_q.copy()
    myrobot.start_joint_position_control(frequency=100)
    time.sleep(0.5)
    
    time0 = time.time()
    while time.time() - time0 < 30:
        time_start = time.time()
        target_q[-1] = current_q[-1] + 0.3*np.sin(time.time()-time0)
        myrobot.set_joint_target_positions(target_q)
        if time.time()-time0 <10:
            myrobot.close_gripper()
        else:
            myrobot.open_gripper()
        time.sleep(max(dt - (time.time()-time_start),0))
    print("DONE")
    myrobot.stop_control()
    
def test_movehome(robot_sn):
    myrobot = FlexivInterface(robot_sn)
    myrobot.move_to_home()
    print("DONE")
    # myrobot.stop_control()
    
def test_movehomecenter(robot_sn):
    myrobot = FlexivInterface(robot_sn)
    myrobot.move_to_home_center()
    print("DONE")
    myrobot.stop_control()
    
def test_impedance(robot_sn):
    myrobot = FlexivInterface(robot_sn)
    print(myrobot.update_states())
    time0 = time.time()
    myrobot.start_joint_impedance_control()
    while time.time() - time0 < 100:
        myrobot.set_impedance()
        time.sleep(5)
        print("set")
    print("DONE")
    myrobot.stop_control()
    
    
if __name__ == "__main__":
    robot_sn = "Rizon4s-063274"
    # test_read(robot_sn=robot_sn)
    # test_write_ee(robot_sn)
    # test_write_joint(robot_sn)
    # test_follow_ee_trajectory(robot_sn)
    # test_gripper(robot_sn)
    # test_movehome(robot_sn)
    test_movehomecenter(robot_sn)
    # test_async_gripper(robot_sn)
    # test_impedance_joint_trajectory(robot_sn=robot_sn)
    # test_impedance(robot_sn)