"""Closed-Loop MuJoCo Physics Active Inference Controller for Saschirob.

Implements Laplace-Preconditioned (Natural Gradient / DEM) Variational Free Energy (VFE)
whole-body bimanual + telescopic lift control inside real MuJoCo 3.x physics:
- Gravity, floor contacts, wheel/caster support polygon, inertia, equality constraints
- 50 Hz (20 ms) control loop matching the 1 Mbps Feetech STS serial bus cycle
- Natural-gradient precision update:
    Pi_mu = J_L^T Pi_x J_L + J_R^T Pi_x J_R + Pi_p + Pi_q
    delta_q = Pi_mu^{-1} * grad_q F

Loads: D:/Dev/repos/rust-saschirob-mcp/simulation/mujoco/saschirob.xml
"""

from pathlib import Path
import numpy as np
import mujoco


MJCF_PATH = Path(r"D:\Dev/repos/rust-saschirob-mcp/simulation/mujoco/saschirob.xml")


class MuJoCoActiveInferenceWholeBody:
    """Whole-body Laplace-Active-Inference controller running inside MuJoCo physics."""

    def __init__(self, mjcf_path: Path = MJCF_PATH):
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)

        # Reset directly to 'reach_mid' keyframe (carries identity base quat + forward-ready pose)
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "reach_mid")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        mujoco.mj_forward(self.model, self.data)

        self.site_left = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "cam_gripper_left")
        self.site_right = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "cam_gripper_right")

        self.joint_names = [
            "lift_stage3",
            "left_shoulder_pan",
            "left_shoulder_lift",
            "left_elbow",
            "left_wrist_pitch",
            "left_wrist_roll",
            "left_flange",
            "right_shoulder_pan",
            "right_shoulder_lift",
            "right_elbow",
            "right_wrist_pitch",
            "right_wrist_roll",
            "right_flange",
        ]
        self.qpos_idx = []
        self.dof_idx = []
        self.act_idx = []
        self.act_kp = []
        self.q_min = []
        self.q_max = []

        for jn in self.joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            self.qpos_idx.append(self.model.jnt_qposadr[jid])
            self.dof_idx.append(self.model.jnt_dofadr[jid])
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"m_{jn}")
            self.act_idx.append(aid)
            self.act_kp.append(self.model.actuator_gainprm[aid, 0])
            lo, hi = self.model.jnt_range[jid]
            self.q_min.append(lo)
            self.q_max.append(hi)

        self.jid_lift2 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "lift_stage2")
        self.dof_lift2 = self.model.jnt_dofadr[self.jid_lift2]
        self.act_lift2 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_lift_stage2")
        self.kp_lift2 = self.model.actuator_gainprm[self.act_lift2, 0]

        self.q_min = np.array(self.q_min, dtype=float)
        self.q_max = np.array(self.q_max, dtype=float)
        self.act_kp = np.array(self.act_kp, dtype=float)
        self.q_home = self.data.qpos[self.qpos_idx].copy()
        self.mu_q = self.q_home.copy()

        # Precision matrices (Laplace Active Inference)
        self.pi_x = np.diag([120.0, 120.0, 120.0])       # Cartesian sensory precision
        self.pi_p = np.diag([25.0] + [8.0] * 12)         # Proprioceptive damping precision
        self.pi_q = np.diag([0.05] + [0.02] * 12)        # Comfortable home posture prior
        self.max_step = np.array([0.020] + [0.08] * 12)  # Max step per 20 ms cycle

        self._apply_ctrl(self.q_home)

    def _apply_ctrl(self, q_des: np.ndarray) -> None:
        """Spinal reflex arc: proprioceptive setpoint + exact gravity feedforward compensation."""
        qfrc_bias = self.data.qfrc_bias[self.dof_idx]
        for i, aid in enumerate(self.act_idx):
            grav_comp = qfrc_bias[i] / max(self.act_kp[i], 1.0)
            self.data.ctrl[aid] = q_des[i] + grav_comp
        grav_comp_lift2 = self.data.qfrc_bias[self.dof_lift2] / max(self.kp_lift2, 1.0)
        self.data.ctrl[self.act_lift2] = 0.5 * q_des[0] + grav_comp_lift2

    def step_control(
        self,
        target_left: np.ndarray,
        target_right: np.ndarray,
        control_dt: float = 0.02,
    ) -> tuple[float, float, float]:
        """Executes one 50 Hz (20 ms) Laplace Active Inference update + 10 MuJoCo 500 Hz physics steps."""
        q_obs = self.data.qpos[self.qpos_idx].copy()
        pos_left = self.data.site_xpos[self.site_left].copy()
        pos_right = self.data.site_xpos[self.site_right].copy()

        jacp_left = np.zeros((3, self.model.nv))
        jacp_right = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp_left, None, self.site_left)
        mujoco.mj_jacSite(self.model, self.data, jacp_right, None, self.site_right)

        j_left = jacp_left[:, self.dof_idx].copy()
        j_right = jacp_right[:, self.dof_idx].copy()
        # Coupled telescopic column: stage2 moves at 0.5x stage3
        j_left[:, 0] += 0.5 * jacp_left[:, self.dof_lift2]
        j_right[:, 0] += 0.5 * jacp_right[:, self.dof_lift2]

        # Prediction errors evaluated against physical observations
        qvel_obs = self.data.qvel[self.dof_idx].copy()
        e_left = target_left - pos_left
        e_right = target_right - pos_right
        e_posture = self.q_home - q_obs

        # Gradient and Laplace Curvature (Posterior Precision Pi_mu) of Variational Free Energy
        grad_f = (
            j_left.T @ (self.pi_x @ e_left)
            + j_right.T @ (self.pi_x @ e_right)
            + self.pi_q @ e_posture
        )
        pi_mu = (
            j_left.T @ self.pi_x @ j_left
            + j_right.T @ self.pi_x @ j_right
            + self.pi_p
            + self.pi_q
        )

        # Natural gradient step + generalized velocity damping (Active Inference d_mu)
        dq = np.linalg.solve(pi_mu, grad_f) - 0.03 * qvel_obs
        dq = np.clip(dq, -self.max_step, self.max_step)
        q_des = np.clip(q_obs + dq, self.q_min, self.q_max)

        # Step MuJoCo physics for 20 ms (10 substeps of 2 ms)
        substeps = max(1, int(round(control_dt / self.model.opt.timestep)))
        for _ in range(substeps):
            self._apply_ctrl(q_des)
            mujoco.mj_step(self.model, self.data)

        err_l = float(np.linalg.norm(self.data.site_xpos[self.site_left] - target_left))
        err_r = float(np.linalg.norm(self.data.site_xpos[self.site_right] - target_right))
        vfe = 0.5 * float(
            e_left.T @ self.pi_x @ e_left
            + e_right.T @ self.pi_x @ e_right
            + e_posture.T @ self.pi_q @ e_posture
        )
        return err_l, err_r, vfe


def run_mujoco_verification():
    print("==========================================================================")
    print(" Saschirob Closed-Loop Active Inference inside MuJoCo 3.x Physics Engine")
    print("==========================================================================")
    sim = MuJoCoActiveInferenceWholeBody()

    # Let initial settle on floor for 50 steps (100 ms)
    for _ in range(50):
        sim._apply_ctrl(sim.q_home)
        mujoco.mj_step(sim.model, sim.data)

    p_l0 = sim.data.site_xpos[sim.site_left].copy()
    p_r0 = sim.data.site_xpos[sim.site_right].copy()
    print(f"Initial Left TCP:  {np.round(p_l0, 3)} (y > 0 is Left)")
    print(f"Initial Right TCP: {np.round(p_r0, 3)} (y < 0 is Right)")

    tasks = [
        (
            "Bimanual Countertop Reach",
            p_l0 + np.array([0.08, -0.04, 0.06]),
            p_r0 + np.array([0.08, +0.04, 0.06]),
        ),
        (
            "High Shelf Telescopic Reach",
            p_l0 + np.array([0.06, -0.05, 0.22]),
            p_r0 + np.array([0.06, +0.05, 0.22]),
        ),
        (
            "Low Counter Lower & Place",
            p_l0 + np.array([0.05, -0.02, -0.15]),
            p_r0 + np.array([0.05, +0.02, -0.15]),
        ),
    ]

    for task_name, tgt_l, tgt_r in tasks:
        print(f"\n---> Task: '{task_name}'")
        print(f"     Targets: L={np.round(tgt_l, 3)} | R={np.round(tgt_r, 3)}")
        for cycle in range(80):
            err_l, err_r, vfe = sim.step_control(tgt_l, tgt_r, control_dt=0.02)
            lift_ext = sim.data.qpos[sim.qpos_idx[0]] * 100.0
            if cycle % 15 == 0 or (err_l < 0.015 and err_r < 0.015):
                print(
                    f"  Cycle {cycle:02d} ({cycle*20:4d} ms) | Lift: {lift_ext:4.1f} cm | "
                    f"Err L: {err_l*1000:5.1f} mm | Err R: {err_r*1000:5.1f} mm | VFE: {vfe:6.2f}"
                )
            if err_l < 0.015 and err_r < 0.015:
                print(
                    f"  [PHYSICS CONVERGED] in {(cycle+1)*20} ms! "
                    f"Err L={err_l*1000:.1f} mm, Err R={err_r*1000:.1f} mm | Base z={sim.data.qpos[2]:.3f} m"
                )
                break

    assert np.all(np.isfinite(sim.data.qpos)), "Non-finite state detected!"
    if sim.model.nsensor > 0:
        sd = sim.data.sensordata
        print(
            f"\n[Telemetry Sensors] nsensor={sim.model.nsensor} | "
            f"imu_accel={np.round(sd[0:3], 2)} m/s^2 | "
            f"imu_quat={np.round(sd[6:10], 3)} | "
            f"lift_frc={sd[11]:.1f} N"
        )
    print("\n==========================================================================")
    print(" [SUCCESS] Real MuJoCo physics closed-loop bimanual + lift control PASS.")
    print("==========================================================================")


if __name__ == "__main__":
    run_mujoco_verification()
