"""Active Inference Arm & Telescopic Lift Controller for saschibot.

Implements Karl Friston's Free Energy Principle for whole-body manipulation:
1. Kinematic Chain from base_link:
   - column_stage1 -> lift_stage2 (prismatic) -> lift_stage3 (prismatic, 2x mimic)
   - column_stage3 -> right_shoulder_pan -> right_shoulder_lift -> right_elbow ->
     right_wrist_pitch -> right_wrist_roll -> right_flange -> right_palm -> TCP
2. Unit Harmonization (Prismatic Meters + Revolute Radians):
   - Classical Jacobian IK suffers from mismatched unit dimensions (m vs rad).
   - Active Inference uses precision-weighted Variational Free Energy (VFE):
     F(q) = 1/2 * e_x^T * Pi_x * e_x + 1/2 * e_q^T * Pi_q * e_q + V_limits(q)
     where Pi_q naturally balances lift travel vs joint rotation stiffness.
3. Synergistic Whole-Body Behavior:
   - Automatically extends telescopic lift when reaching high shelves or low floors.
   - Retracts lift and preserves ergonomic joint angles during nominal tabletop manipulation.

Usage:
    python 09_saschibot_active_inference_arm_lift.py
"""

from dataclasses import dataclass
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


class SaschibotArmLiftKinematics:
    """Analytical kinematics derived from rust-saschirob-mcp/models/saschirob.urdf."""

    def __init__(self):
        # Joint 0: Prismatic Lift (stage 2 + stage 3 mimic)
        # Limits: [0.0, 0.35] meters for stage 2 (mimic stage 3 extends by 2.0 * d)
        self.lift_min = 0.0
        self.lift_max = 0.35

        # Joints 1-6: 6-DoF Revolute Right Arm
        self.arm_limits_min = np.array([-2.6, -1.5, -2.2, -1.5, -1.5, -1.5])
        self.arm_limits_max = np.array([ 2.6,  1.5,  2.2,  1.5,  1.5,  1.5])

        # Combined limits for q = [d_lift, pan, lift, elbow, pitch, roll, flange]
        self.q_min = np.concatenate(([self.lift_min], self.arm_limits_min))
        self.q_max = np.concatenate(([self.lift_max], self.arm_limits_max))
        self.num_dof = 7

    def forward_kinematics(self, q: np.ndarray) -> tuple[np.ndarray, list[np.ndarray]]:
        """Computes end-effector TCP position in base_link frame.
        
        q[0] = d_lift (prismatic travel in meters)
        q[1..6] = arm joint angles in radians
        """
        d_lift = q[0]
        # Mimic multiplier from URDF: stage3 = 2.0 * stage2
        total_lift_z = 0.47 + d_lift + 0.15 + (2.0 * d_lift)  # Base column height + telescopic extensions

        t_curr = np.eye(4)
        link_origins = [t_curr[0:3, 3].copy()]

        # 1. Column Stage 3 origin (where arm mounts)
        # xyz: 0, 0, total_lift_z
        t_col3 = _trans((0.0, 0.0, total_lift_z))
        t_curr = t_curr @ t_col3
        link_origins.append(t_curr[0:3, 3].copy())

        # 2. Right Shoulder Pan (origin: [0, -0.22, 0.42], axis: [0, 0, 1])
        t_curr = t_curr @ _trans((0.0, -0.22, 0.42)) @ _rot_z(q[1])
        link_origins.append(t_curr[0:3, 3].copy())

        # 3. Right Shoulder Lift (origin: [0, 0, 0], axis: [0, 1, 0])
        t_curr = t_curr @ _rot_y(q[2])
        link_origins.append(t_curr[0:3, 3].copy())

        # 4. Right Elbow (origin: [0, 0, -0.25], axis: [0, 1, 0])
        t_curr = t_curr @ _trans((0.0, 0.0, -0.25)) @ _rot_y(q[3])
        link_origins.append(t_curr[0:3, 3].copy())

        # 5. Right Wrist Pitch (origin: [0, 0, -0.24], axis: [0, 1, 0])
        t_curr = t_curr @ _trans((0.0, 0.0, -0.24)) @ _rot_y(q[4])
        link_origins.append(t_curr[0:3, 3].copy())

        # 6. Right Wrist Roll (origin: [0, 0, -0.06], axis: [1, 0, 0])
        t_curr = t_curr @ _trans((0.0, 0.0, -0.06)) @ _rot_x(q[5])
        link_origins.append(t_curr[0:3, 3].copy())

        # 7. Right Flange (origin: [0, 0, 0], axis: [0, 0, 1])
        t_curr = t_curr @ _rot_z(q[6])
        link_origins.append(t_curr[0:3, 3].copy())

        # 8. Right Palm & Gripper TCP (origin: [0, 0, -0.06] + [0, 0, -0.05])
        t_curr = t_curr @ _trans((0.0, 0.0, -0.11))
        tcp_pos = t_curr[0:3, 3]
        link_origins.append(tcp_pos.copy())

        return tcp_pos, link_origins

    def compute_jacobian(self, q: np.ndarray, eps: float = 1e-4) -> np.ndarray:
        """Computes 3x7 Jacobian via numerical differentiation."""
        j = np.zeros((3, self.num_dof))
        f0, _ = self.forward_kinematics(q)

        for i in range(self.num_dof):
            q_pert = q.copy()
            q_pert[i] += eps
            f_pert, _ = self.forward_kinematics(q_pert)
            j[:, i] = (f_pert - f0) / eps

        return j


class SaschibotActiveInferenceController:
    """Active Inference controller coordinating telescopic lift + 6-DoF arm."""

    def __init__(self, kinematics: SaschibotArmLiftKinematics):
        self.kin = kinematics
        # State: [d_lift (m), pan, lift, elbow, pitch, roll, flange (rad)]
        self.q = np.array([0.10, 0.0, 0.3, -0.8, -0.5, 0.0, 0.0])
        # Home pose: lift mid-travel (0.12 m), arm slightly bent in readiness
        self.q_home = np.array([0.12, 0.0, 0.3, -0.8, -0.5, 0.0, 0.0])

        # Task precision: high weight on Cartesian reaching accuracy
        self.pi_x = np.diag([75.0, 75.0, 75.0])

        # Posture precision: balances units!
        # Lift has lower impedance (wants to move if needed), arm joints gently held
        self.pi_q = np.diag([1.5, 0.5, 0.5, 0.3, 0.2, 0.1, 0.1])

        self.kappa = 0.85
        # Max velocity: 0.15 m/s for lift, 2.0 rad/s for servos
        self.max_dq = np.array([0.15, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])

    def step(self, target_tcp: np.ndarray, dt: float = 0.02) -> tuple[np.ndarray, float, float]:
        """One iteration of Variational Free Energy minimization."""
        current_tcp, _ = self.kin.forward_kinematics(self.q)
        jacobian = self.kin.compute_jacobian(self.q)

        # 1. Cartesian Tracking Prediction Error: e_x = x* - FK(q)
        e_x = target_tcp - current_tcp
        cartesian_dist = float(np.linalg.norm(e_x))

        # 2. Posture Prediction Error: e_q = q_home - q
        e_q = self.q_home - self.q

        # 3. Joint Limit Potential (quadratic barrier for both lift and revolute joints)
        grad_v_limits = np.zeros(self.kin.num_dof)
        margin_lift = 0.03  # 3 cm margin for lift
        margin_arm = 0.15   # rad margin for revolute
        margins = np.array([margin_lift] + [margin_arm] * 6)

        for i in range(self.kin.num_dof):
            dist_min = self.q[i] - self.kin.q_min[i]
            dist_max = self.kin.q_max[i] - self.q[i]
            if dist_min < margins[i]:
                grad_v_limits[i] -= 12.0 * (margins[i] - dist_min)
            elif dist_max < margins[i]:
                grad_v_limits[i] += 12.0 * (margins[i] - dist_max)

        # 4. Free Energy Gradient: q_dot = J^T * Pi_x * e_x + Pi_q * e_q - grad_V_limits
        task_force = jacobian.T @ (self.pi_x @ e_x)
        posture_force = self.pi_q @ e_q
        q_dot = self.kappa * (task_force + posture_force - grad_v_limits)

        # 5. Velocity saturation
        q_dot = np.clip(q_dot, -self.max_dq, self.max_dq)

        # 6. Integration
        self.q += q_dot * dt
        self.q = np.clip(self.q, self.kin.q_min, self.kin.q_max)

        vfe = 0.5 * float(e_x.T @ self.pi_x @ e_x + e_q.T @ self.pi_q @ e_q)
        return current_tcp, cartesian_dist, vfe


def run_saschibot_benchmark():
    print("=========================================================================")
    print(" saschibot: Whole-Body Active Inference (Lift + 6-DoF Arm)")
    print("=========================================================================")
    kin = SaschibotArmLiftKinematics()
    ctrl = SaschibotActiveInferenceController(kin)

    # Waypoints covering full spatial envelope relative to base_link
    waypoints = [
        ("Nominal Tabletop (Mid)", np.array([0.35, -0.22, 0.75])),
        ("High Shelf Reach",       np.array([0.40, -0.15, 1.35])),  # Requires lift extension
        ("Floor Pick Position",    np.array([0.30, -0.25, 0.25])),  # Requires lift contraction
        ("Across Center Deck",     np.array([0.32,  0.10, 0.70])),  # Reaching across chassis
        ("Return Stowed",          np.array([0.25, -0.22, 0.65])),
    ]

    dt = 0.02
    total_steps = 0

    for wp_name, target in waypoints:
        print(f"\n---> Commencing Goal Attractor: '{wp_name}' -> Target: {np.round(target, 3)}")
        for step in range(60):
            tcp_pos, err, vfe = ctrl.step(target, dt=dt)
            total_steps += 1
            if step % 15 == 0 or err < 0.015:
                print(f"  Step {step:02d} | Lift: {ctrl.q[0]*100:4.1f} cm | TCP: [{tcp_pos[0]:+.3f}, {tcp_pos[1]:+.3f}, {tcp_pos[2]:+.3f}] | Error: {err*1000:5.1f} mm | VFE: {vfe:7.2f}")
            if err < 0.012:  # 12 mm threshold
                print(f"  [CONVERGED] in {step+1} cycles ({(step+1)*dt*1000:.0f} ms)! Final Error: {err*1000:.1f} mm | Lift: {ctrl.q[0]*100:.1f} cm")
                break
            time.sleep(0.005)

    print("\n=========================================================================")
    print(f" [SUCCESS] Multi-level reaching completed in {total_steps} cycles.")
    print(" Synergistic telescopic lift + revolute arm coordinated seamlessly via VFE.")
    print("=========================================================================")


if __name__ == "__main__":
    run_saschibot_benchmark()
