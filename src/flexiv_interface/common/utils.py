import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

class lpf:
    def __init__(self, alpha, init_value):
        self.alpha = alpha
        self.state = init_value
        
    def update(self, new_value):
        self.state = self.alpha * new_value + (1 - self.alpha) * self.state
        return self.state

class safety_barrier:
    def __init__(self, velocity_limit, dt, max_position_step=0.1, max_orientation_step=0.2):
        self.dt = dt
        self.max_position_step = max_position_step
        self.max_orientation_step = max_orientation_step
        
        if isinstance(velocity_limit, (int, float)):
            self.velocity_limit = {
                'linear': np.array([velocity_limit, velocity_limit, velocity_limit]),
                'angular': np.array([velocity_limit, velocity_limit, velocity_limit])
            }
        elif len(velocity_limit) == 2:
            self.velocity_limit = {
                'linear': np.array([velocity_limit[0], velocity_limit[0], velocity_limit[0]]),
                'angular': np.array([velocity_limit[1], velocity_limit[1], velocity_limit[1]])
            }
        elif len(velocity_limit) == 6:
            self.velocity_limit = {
                'linear': np.array(velocity_limit[:3]),
                'angular': np.array(velocity_limit[3:])
            }
        else:
            raise ValueError("velocity_limit must be scalar, 2D [v, w], or 6D [vx, vy, vz, wx, wy, wz]")
    
    def _quaternion_to_euler(self, quaternion):
        rotation = R.from_quat(quaternion)
        return rotation.as_euler('xyz')
    
    def _euler_to_quaternion(self, euler):
        rotation = R.from_euler('xyz', euler)
        return rotation.as_quat()
    
    def _angle_difference(self, angle1, angle2):
        diff = angle1 - angle2
        return np.arctan2(np.sin(diff), np.cos(diff))
    
    def _constrain_velocity(self, desired_velocity, velocity_limit):
        velocity_norm = np.linalg.norm(desired_velocity)
        if velocity_norm > velocity_limit:
            return desired_velocity * velocity_limit / velocity_norm
        return desired_velocity
    
    def _apply_position_constraints(self, current_position, desired_position):
        position_diff = desired_position - current_position
        position_norm = np.linalg.norm(position_diff)
        if position_norm > self.max_position_step:
            position_diff = position_diff * self.max_position_step / position_norm

        desired_velocity = position_diff / self.dt
        constrained_velocity = self._constrain_velocity(desired_velocity, np.linalg.norm(self.velocity_limit['linear']))
        safe_position = current_position + constrained_velocity * self.dt
        
        return safe_position, constrained_velocity
    
    def _apply_orientation_constraints(self, current_orientation, desired_orientation):
        angle_diff = np.zeros(3)
        for i in range(3):
            angle_diff[i] = self._angle_difference(desired_orientation[i], current_orientation[i])
        
        angle_norm = np.linalg.norm(angle_diff)
        if angle_norm > self.max_orientation_step:
            angle_diff = angle_diff * self.max_orientation_step / angle_norm
        
        desired_angular_velocity = angle_diff / self.dt
        constrained_angular_velocity = self._constrain_velocity(
            desired_angular_velocity, 
            np.linalg.norm(self.velocity_limit['angular'])
        )
        
        safe_orientation = current_orientation + constrained_angular_velocity * self.dt
        safe_orientation = np.arctan2(np.sin(safe_orientation), np.cos(safe_orientation))
        
        return safe_orientation, constrained_angular_velocity
    
    def update(self, current_pose, target_pose):
        if len(current_pose) == 7 and len(target_pose) == 7:
            current_pos = np.array(current_pose[:3])
            current_quat = np.array(current_pose[3:])
            current_euler = self._quaternion_to_euler(current_quat)
            
            target_pos = np.array(target_pose[:3])
            target_quat = np.array(target_pose[3:])
            target_euler = self._quaternion_to_euler(target_quat)
        else:
            current_pos = np.array(current_pose[:3])
            current_euler = np.array(current_pose[3:])
            
            target_pos = np.array(target_pose[:3])
            target_euler = np.array(target_pose[3:])
        
        safe_position, constrained_linear_velocity = self._apply_position_constraints(current_pos, target_pos)
        safe_orientation, constrained_angular_velocity = self._apply_orientation_constraints(current_euler, target_euler)
        safe_pose = np.concatenate([safe_position, safe_orientation])
        
        self.current_velocity = {
            'linear': constrained_linear_velocity,
            'angular': constrained_angular_velocity
        }
        
        return safe_pose
    
    def update_with_velocity(self, current_pose, desired_velocity):
        linear_velocity = np.array(desired_velocity[:3])
        linear_norm = np.linalg.norm(linear_velocity)
        if linear_norm > np.linalg.norm(self.velocity_limit['linear']):
            linear_velocity = linear_velocity * np.linalg.norm(self.velocity_limit['linear']) / linear_norm
        
        angular_velocity = np.array(desired_velocity[3:])
        angular_norm = np.linalg.norm(angular_velocity)
        if angular_norm > np.linalg.norm(self.velocity_limit['angular']):
            angular_velocity = angular_velocity * np.linalg.norm(self.velocity_limit['angular']) / angular_norm
        
        current_array = np.array(current_pose)
        desired_pose = current_array.copy()
        desired_pose[:3] += linear_velocity * self.dt

        angle_change = angular_velocity * self.dt
        for i in range(3, 6):
            desired_pose[i] = current_array[i] + angle_change[i-3]
            desired_pose[i] = np.arctan2(np.sin(desired_pose[i]), np.cos(desired_pose[i]))
        
        safe_pose = self.update(current_pose, desired_pose)
        
        return safe_pose
    
def concat_pose(pos:np.ndarray,quat:np.ndarray):
    return np.array(pos.tolist()+quat.tolist())

def split_pose(pose:np.ndarray):
    return pose[:3], pose[3:]
    
def smooth_make_z_parallel_to_normal(current_quat, target_normal, t):
    target_normal = np.asarray(target_normal, dtype=float)
    target_normal /= np.linalg.norm(target_normal)

    r_curr = R.from_quat(current_quat)
    z = r_curr.apply([0, 0, 1])

    normals = [target_normal, -target_normal]

    best_angle = np.inf
    best_axis = None

    for n in normals:
        dot = np.clip(np.dot(z, n), -1.0, 1.0)
        angle = np.arccos(dot)

        if angle < best_angle:
            if angle < 1e-6:
                best_angle = 0.0
                best_axis = None
            elif np.pi - angle < 1e-6:
                # 180 deg: choose deterministic perpendicular axis
                perp = np.array([1, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1, 0])
                axis = np.cross(z, perp)
                axis /= np.linalg.norm(axis)
                best_angle = np.pi
                best_axis = axis
            else:
                axis = np.cross(z, n)
                axis /= np.linalg.norm(axis)
                best_angle = angle
                best_axis = axis
    if best_angle < 1e-6:
        r_target = r_curr
    else:
        r_delta = R.from_rotvec(best_axis * best_angle)
        r_target = r_delta * r_curr

    slerp = Slerp(
        [0.0, 1.0],
        R.from_quat([r_curr.as_quat(), r_target.as_quat()])
    )

    return slerp([t]).as_quat()[0]