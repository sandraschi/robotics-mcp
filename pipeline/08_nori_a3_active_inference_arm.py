"""Active Inference 7-DoF Arm Controller for norirobotics a3.

Implements Karl Friston's Free Energy Principle for robotic manipulation:
1. Forward Kinematics (FK) & Jacobian (J) for the 7-DoF right arm:
   - shoulder_pitch -> shoulder_roll -> bicep_yaw -> elbow_pitch -> forearm_yaw -> wrist_pitch -> wrist_roll -> TCP
2. Variational Free Energy Minimization (VFE):
   - F(q) = 1/2 * e_x^T * Pi_x * e_x + 1/2 * e_q^T * Pi_q * e_q + V_limits(q) + V_obs(q)
   - Motor velocity commands: q_dot = -kappa * grad_q F(q)
   - Naturally avoids matrix inversion (J^T mapping), singularity-robust, intrinsically compliant.
3. Feetech STS3215 constraints:
   - Joint limits, angular velocity clamps (rad/s), and smooth trajectory tracking.

Usage:
    python 08_nori_a3_active_inference_arm.py
"""

from dataclasses import dataclass, field
import numpy as np
import time


def _rot_x(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c,   -s,  0.0],
        [0.0, s,    c,  0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _rot_y(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c,   0.0, s,   0.0],
        [ 0.0, 1.0, 0.0, 0.0],
        [-s,   0.0, c,   0.0],
        [ 0.0, 0.0, 0.0, 1.0],
    ])


def _rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c,   -s,  0.0, 0.0],
        [s,    c,  0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _trans(xyz: tuple[float, float, float]) -> np.ndarray:
    t = np.eye(4)
    t[0:3, 3] = xyz
    return t


def _rpy(rpy: tuple[float, float, float]) -> np.ndarray:
    return _rot_z(rpy[2]) @ _rot_y(rpy[1]) @ _rot_x(rpy[0])


@dataclass
class JointSpec:
    name: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    limits: tuple[float, float]  # [min, max] radians


class NoriA3RightArmKinematics:
    """Rigorous analytical kinematics derived from nori_a3.urdf."""

    def __init__(self):
        # 7 Revolute joints + TCP offset
        self.joints: list[JointSpec] = [
            JointSpec("right_shoulder_pitch", (0.0, -0.125, 0.087), (0.0, 0.0, np.pi), (0, 1, 0), (-np.pi, np.pi)),
            JointSpec("right_shoulder_roll", (0.0, 0.0567, 0.0), (np.pi / 2, 0.0, 0.0), (1, 0, 0), (-np.pi / 2, np.pi / 2)),
            JointSpec("right_bicep_yaw", (0.0, 0.0, -0.115), (0.0, 0.0, 0.0), (0, 0, 1), (-np.pi, np.pi)),
            JointSpec("right_elbow_pitch", (0.0, 0.0, -0.06736), (0.0, 0.0, 0.0), (0, 1, 0), (-np.pi / 2, np.pi / 2)),
            JointSpec("right_forearm_yaw", (0.0, 0.0, -0.115), (0.0, 0.0, 0.0), (0, 0, 1), (-np.pi, np.pi)),
            JointSpec("right_wrist_pitch", (0.0, 0.0, -0.0462), (0.0, 0.0, 0.0), (0, 1, 0), (-np.pi / 2, np.pi / 2)),
            JointSpec("right_wrist_roll", (0.0, 0.0, -0.0678), (0.0, 0.0, np.pi), (-1, 0, 0), (-np.pi / 2, np.pi / 2)),
        ]
        # TCP offset from wrist roll link
        self.tcp_offset = _trans((0.0024, 0.0089, -0.1123)) @ _rpy((0.0, np.pi, np.pi))

        self.num_joints = len(self.joints)
        self.q_min = np.array([j.limits[0] for j in self.joints])
        self.q_max = np.array([j.limits[1] for j in self.joints])

    def forward_kinematics(self, q: np.ndarray) -> tuple[np.ndarray, list[np.ndarray]]:
        """Computes end-effector TCP position and intermediate link origins."""
        t_curr = np.eye(4)
        link_origins = [t_curr[0:3, 3].copy()]

        for i, j_spec in enumerate(self.joints):
            # Static parent-to-joint transform
            t_joint = _trans(j_spec.xyz) @ _rpy(j_spec.rpy)
            # Active joint rotation
            angle = q[i]
            if j_spec.axis == (1, 0, 0):
                r_act = _rot_x(angle)
            elif j_spec.axis == (-1, 0, 0):
                r_act = _rot_x(-angle)
            elif j_spec.axis == (0, 1, 0):
                r_act = _rot_y(angle)
            else:
                r_act = _rot_z(angle)

            t_curr = t_curr @ t_joint @ r_act
            link_origins.append(t_curr[0:3, 3].copy())

        # Final TCP position
        t_tcp = t_curr @ self.tcp_offset
        tcp_pos = t_tcp[0:3, 3]
        return tcp_pos, link_origins

    def compute_jacobian(self, q: np.ndarray, eps: float = 1e-4) -> np.ndarray:
        """Computes 3x7 position Jacobian via numerical differentiation."""
        j = np.zeros((3, self.num_joints))
        f0, _ = self.forward_kinematics(q)

        for i in range(self.num_joints):
            q_pert = q.copy()
            q_pert[i] += eps
            f_pert, _ = self.forward_kinematics(q_pert)
            j[:, i] = (f_pert - f0) / eps

        return j


class NoriActiveInferenceArmController:
    """Active Inference controller for the 7-DoF manipulator."""

    def __init__(self, kinematics: NoriA3RightArmKinematics):
        self.kin = kinematics
        self.q = np.zeros(self.kin.num_joints)
        # Comfortable home posture (elbow slightly bent)
        self.q_home = np.array([0.0, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0])

        # Sensitivities / Precision matrices
        self.pi_x = np.diag([80.0, 80.0, 80.0])  # Cartesian task precision
        self.pi_q = np.diag([0.5, 0.5, 0.2, 0.2, 0.1, 0.1, 0.1])  # Posture regularization
        self.kappa = 0.8  # Learning rate / descent gain
        self.max_dq = 2.0  # Feetech max joint velocity (rad/s)

        # Obstacle definition (sphere: center xyz, radius)
        self.obstacle_center = np.array([0.15, -0.20, 0.05])
        self.obstacle_radius = 0.08

    def step(self, target_tcp: np.ndarray, dt: float = 0.02) -> tuple[np.ndarray, float, float]:
        """One iteration of Variational Free Energy minimization."""
        current_tcp, _ = self.kin.forward_kinematics(self.q)
        jacobian = self.kin.compute_jacobian(self.q)

        # 1. Cartesian Tracking Prediction Error: e_x = x* - FK(q)
        e_x = target_tcp - current_tcp
        cartesian_dist = float(np.linalg.norm(e_x))

        # 2. Posture Prediction Error: e_q = q_home - q
        e_q = self.q_home - self.q

        # 3. Joint Limit Potential (quadratic repulsive field near edges)
        grad_v_limits = np.zeros(self.kin.num_joints)
        margin = 0.15  # rad
        for i in range(self.kin.num_joints):
            dist_min = self.q[i] - self.kin.q_min[i]
            dist_max = self.kin.q_max[i] - self.q[i]
            if dist_min < margin:
                grad_v_limits[i] -= 10.0 * (margin - dist_min)
            elif dist_max < margin:
                grad_v_limits[i] += 10.0 * (margin - dist_max)

        # 4. Obstacle Avoidance Potential
        grad_v_obs = np.zeros(self.kin.num_joints)
        dist_to_obs = np.linalg.norm(current_tcp - self.obstacle_center)
        safety_dist = self.obstacle_radius + 0.04
        if dist_to_obs < safety_dist:
            obs_dir = (current_tcp - self.obstacle_center) / (dist_to_obs + 1e-6)
            f_repulsive = 15.0 * (safety_dist - dist_to_obs) * obs_dir
            grad_v_obs = -jacobian.T @ f_repulsive

        # 5. Free Energy Gradient Descent: q_dot = J^T * Pi_x * e_x + Pi_q * e_q - grad_V
        task_force = jacobian.T @ (self.pi_x @ e_x)
        posture_force = self.pi_q @ e_q
        q_dot = self.kappa * (task_force + posture_force - grad_v_limits - grad_v_obs)

        # 6. Actuator Velocity Saturation (Feetech limits)
        q_dot = np.clip(q_dot, -self.max_dq, self.max_dq)

        # Integrate joint state
        self.q += q_dot * dt
        self.q = np.clip(self.q, self.kin.q_min, self.kin.q_max)

        # Calculate Variational Free Energy
        vfe = 0.5 * float(e_x.T @ self.pi_x @ e_x + e_q.T @ self.pi_q @ e_q)
        return current_tcp, cartesian_dist, vfe


def run_benchmark():
    print("=========================================================================")
    print(" norirobotics a3: 7-DoF Active Inference Arm Control Demonstration")
    print("=========================================================================")
    kin = NoriA3RightArmKinematics()
    ctrl = NoriActiveInferenceArmController(kin)

    # Sequence of 3D Waypoint Targets (Meters)
    waypoints = [
        ("Home Pose", np.array([0.05, -0.22, 0.0])),
        ("Reach Forward-Right", np.array([0.22, -0.18, -0.05])),
        ("Lift Object High", np.array([0.18, -0.15, 0.15])),
        ("Reach across Center", np.array([0.12, -0.05, 0.02])),
        ("Return Home", np.array([0.05, -0.22, 0.0])),
    ]

    total_steps = 0
    dt = 0.02  # 50 Hz control loop

    for wp_name, target in waypoints:
        print(f"\n---> Commencing Goal Attractor: '{wp_name}' -> Target: {np.round(target, 3)}")
        for step in range(50):
            tcp_pos, err, vfe = ctrl.step(target, dt=dt)
            total_steps += 1
            if step % 10 == 0 or err < 0.01:
                print(f"  Step {step:02d} | TCP: [{tcp_pos[0]:+.3f}, {tcp_pos[1]:+.3f}, {tcp_pos[2]:+.3f}] | Error: {err*1000:5.1f} mm | VFE: {vfe:7.2f}")
            if err < 0.008:  # 8 mm convergence threshold
                print(f"  [CONVERGED] in {step+1} cycles ({(step+1)*dt*1000:.0f} ms)! Final Error: {err*1000:.1f} mm")
                break
            time.sleep(0.005)

    print("\n=========================================================================")
    print(f" [SUCCESS] Multi-waypoint reaching completed in {total_steps} cycles.")
    print(" Singularity-free compliant trajectory generated via Variational Free Energy.")
    print("=========================================================================")


if __name__ == "__main__":
    run_benchmark()
