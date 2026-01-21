import numpy as np
from flexiv_interface.common.utils import safety_barrier, concat_pose, split_pose


class ScalarAdmittance1D:
    """
    1D admittance:
        M * x_ddot + D * x_dot = F_env - F_des
    """
    def __init__(self, M, D, dt, x_init=0.0):
        self.M = float(M)
        self.D = float(D)
        self.dt = dt

        self.x = float(x_init)
        self.dx = 0.0

    def reset(self, x_init):
        self.x = float(x_init)
        self.dx = 0.0

    def step(self, F_env, F_des):
        ddx = (F_env - F_des - self.D * self.dx) / self.M
        self.dx += ddx * self.dt
        self.x  += self.dx * self.dt
        return self.x
    
    
class ContactNormalEstimator:
    def __init__(self):
        self.normal = None
        self.integral_vector = np.zeros(3) 

    @staticmethod
    def _normalize(v):
        n = np.linalg.norm(v)
        return v / n

    def reset(self):
        self.normal = None
        self.integral_vector = np.zeros(3)

    def update(self, measured_force):
        self.integral_vector += measured_force
        self.normal = self._normalize(self.integral_vector)
        return self.normal.copy()      
    
class ForceKeeping:
    def __init__(self, dt,
         contact_force_threshold,
         desired_contact_force,
         exit_pos_threshold,
         release_force_threshold = 5):
        
        self.dt = dt
        self.in_contact = False
        self.reenter_lpf = None
        self.last_send_pos = None
        self.transition_error = None

        self.contact_force_threshold = contact_force_threshold
        self.desired_contact_force = desired_contact_force
        self.exit_pos_threshold = exit_pos_threshold
        self.allow_reenter = True
        self.release_force_threshold = release_force_threshold
        self.last_pos_n = 0
        self.safety_barrier = safety_barrier([0.05, 0.05], dt)

        self.normal_estimator = ContactNormalEstimator(
        )
        self.adm = ScalarAdmittance1D(
            M=2.0,
            D=160.0,
            dt=dt,
            x_init=0.0,
        )

        self.contact_pos_ref = None
        
    @staticmethod
    def _decompose(vec, normal):
        n = np.dot(vec, normal)
        t = vec - n * normal
        return n, t
    
    def step(self, target_pos, target_quat, measured_force):
        force_norm = np.linalg.norm(measured_force)

        if force_norm < self.release_force_threshold:
            self.allow_reenter = True
        if force_norm < self.contact_force_threshold:
            self.in_contact = False
                
        if not self.in_contact:
            if force_norm < self.contact_force_threshold or (not self.allow_reenter): 
                if self.last_send_pos is not None:
                    transition_error = np.linalg.norm(target_pos-self.last_send_pos)
                    current_pose = concat_pose(self.last_send_pos,target_quat)
                    target_pose = concat_pose(target_pos,target_quat)
                    new_pose = self.safety_barrier.update(current_pose,target_pose)
                    new_pos, new_quat = split_pose(new_pose)
                    self.last_send_pos = new_pos
                    return new_pos, target_quat
                return target_pos, target_quat

            
            self.in_contact = True
            self.contact_pos_ref = target_pos.copy()
            self.normal_estimator.reset()
            self.adm.reset(x_init=0.0)

        normal = self.normal_estimator.update(measured_force)

        pos_error = target_pos - self.contact_pos_ref
        pos_n, pos_t = self._decompose(pos_error, normal)
        self.last_pos_n = np.abs(pos_n)

        if pos_n > self.exit_pos_threshold:
            self.in_contact = False
            self.contact_pos_ref = None
            self.allow_reenter = False
            current_pose = concat_pose(self.last_send_pos,target_quat)
            target_pose = concat_pose(target_pos,target_quat)
            new_pose = self.safety_barrier.update(current_pose,target_pose)
            new_pos, new_quat = split_pose(new_pose)
            self.last_send_pos = new_pos
            return new_pos, target_quat
            
        force_n = np.dot(measured_force, normal)

        adm_disp = self.adm.step(
            F_env=force_n,
            F_des=self.desired_contact_force,
        )

        new_pos = (
            self.contact_pos_ref
            + pos_t
            + adm_disp * normal
        )

        self.last_send_pos = new_pos

        return new_pos, target_quat
    

class ForceKeepingWithVelocityCompensation:
    def __init__(self, dt,
         contact_force_threshold,
         desired_contact_force,
         exit_pos_threshold,
         release_force_threshold = 5,
         velocity_compensation_gain = 0.5):
        
        self.dt = dt
        self.in_contact = False
        self.reenter_lpf = None
        self.last_send_pos = None
        self.transition_error = None

        self.contact_force_threshold = contact_force_threshold
        self.desired_contact_force = desired_contact_force
        self.exit_pos_threshold = exit_pos_threshold
        self.allow_reenter = True
        self.release_force_threshold = release_force_threshold
        self.velocity_compensation_gain = velocity_compensation_gain
        self.last_pos_n = 0
        self.safety_barrier = safety_barrier([0.05, 0.05], dt)

        self.normal_estimator = ContactNormalEstimator()
        self.adm = ScalarAdmittance1D(
            M=2.0,
            D=160.0,
            dt=dt,
            x_init=0.0,
        )

        self.contact_pos_ref = None
        
    @staticmethod
    def _decompose(vec, normal):
        n = np.dot(vec, normal)
        t = vec - n * normal
        return n, t
    
    @staticmethod
    def _get_velocity_parallel_component(velocity, normal, force):
        if np.linalg.norm(velocity) < 1e-3: 
            return np.zeros(3)
        
        velocity_mag = np.linalg.norm(velocity)
        velocity_dir = velocity / velocity_mag
        force_in_velocity_dir = np.dot(force, velocity_dir) * velocity_dir
        if normal is not None:
            force_parallel_velocity = force_in_velocity_dir - np.dot(force_in_velocity_dir, normal) * normal
        else:
            force_parallel_velocity = np.zeros(3)
            
        return force_parallel_velocity
    
    def _compensate_friction_for_normal_estimation(self, measured_force, velocity):
        if self.normal_estimator.normal is None:
            return measured_force.copy()
        
        friction_component = self._get_velocity_parallel_component(velocity, 
                                                                  self.normal_estimator.normal,
                                                                  measured_force)
        compensated_force = measured_force - self.velocity_compensation_gain * friction_component
        original_normal_component = np.dot(measured_force, self.normal_estimator.normal)
        compensated_normal_component = np.dot(compensated_force, self.normal_estimator.normal)
        if np.sign(original_normal_component) != np.sign(compensated_normal_component) and abs(original_normal_component) > 1e-3:
            compensated_force = (original_normal_component * self.normal_estimator.normal + 
                                0.1 * (compensated_force - np.dot(compensated_force, self.normal_estimator.normal) * self.normal_estimator.normal))
        return compensated_force

    def step(self, target_pos, target_quat, measured_force, velocity=None):
        force_norm = np.linalg.norm(measured_force)

        if force_norm < self.release_force_threshold:
            self.allow_reenter = True
        if force_norm < self.contact_force_threshold:
            self.in_contact = False
                
        if not self.in_contact:
            if force_norm < self.contact_force_threshold or (not self.allow_reenter): 
                if self.last_send_pos is not None:
                    transition_error = np.linalg.norm(target_pos-self.last_send_pos)
                    current_pose = concat_pose(self.last_send_pos,target_quat)
                    target_pose = concat_pose(target_pos,target_quat)
                    new_pose = self.safety_barrier.update(current_pose,target_pose)
                    new_pos, new_quat = split_pose(new_pose)
                    self.last_send_pos = new_pos
                    return new_pos, target_quat
                return target_pos, target_quat

            self.in_contact = True
            self.contact_pos_ref = target_pos.copy()
            self.normal_estimator.reset()
            self.adm.reset(x_init=0.0)

        if velocity is not None and np.linalg.norm(velocity) > 0.01:  
            compensated_force = self._compensate_friction_for_normal_estimation(measured_force, velocity)
        else:
            compensated_force = measured_force.copy()

        normal = self.normal_estimator.update(compensated_force)

        pos_error = target_pos - self.contact_pos_ref
        pos_n, pos_t = self._decompose(pos_error, normal)
        self.last_pos_n = np.abs(pos_n)

        if pos_n > self.exit_pos_threshold:
            self.in_contact = False
            self.contact_pos_ref = None
            self.allow_reenter = False
            current_pose = concat_pose(self.last_send_pos,target_quat)
            target_pose = concat_pose(target_pos,target_quat)
            new_pose = self.safety_barrier.update(current_pose,target_pose)
            new_pos, new_quat = split_pose(new_pose)
            self.last_send_pos = new_pos
            return new_pos, target_quat
            
        force_n = np.dot(measured_force, normal)

        adm_disp = self.adm.step(
            F_env=force_n,
            F_des=self.desired_contact_force,
        )

        new_pos = (
            self.contact_pos_ref
            + pos_t
            + adm_disp * normal
        )

        self.last_send_pos = new_pos

        return new_pos, target_quat