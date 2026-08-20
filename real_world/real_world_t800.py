"""EngineAI base-T800 ROS 2 backend for the MCC right-arm experiment.

The backend intentionally exposes only the five right-arm joints to MCC.  The
factory ``lower_body_balance`` task remains responsible for the other joints.
Read-only and shadow modes do not create a command publisher.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Any

import mujoco
import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header

from interface_protocol.msg import JointOverrideCommand, JointState, MotionState
from sim.base_sim import Obs


class RealWorldT800:
    """Map T800 ROS 2 joint state/override topics to the MCC ``BaseSim`` API."""

    def __init__(
        self,
        control_dt: float,
        xml_path: str,
        mode: str = "shadow",
    ) -> None:
        self.name = "real"
        self.control_dt = float(control_dt)
        self.mode = str(mode).strip().lower()
        if self.mode not in {"readonly", "shadow", "zero_replay", "command"}:
            raise ValueError(f"Unsupported T800 real mode: {mode}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        if int(self.model.nu) != 5 or int(self.model.nq) != 5:
            raise ValueError(
                "T800 MCC right-arm model must contain exactly five actuators "
                f"and five qpos entries, got nu={self.model.nu}, nq={self.model.nq}."
            )

        config_path = os.path.join(os.path.dirname(xml_path), "real.yaml")
        with open(config_path, "r", encoding="utf-8") as f:
            self.config: dict[str, Any] = yaml.safe_load(f) or {}

        self.joint_indices = np.asarray(
            self.config["joint_indices"], dtype=np.int32
        ).reshape(-1)
        if self.joint_indices.shape != (5,):
            raise ValueError("T800 real joint_indices must contain five entries.")

        self.stiffness = np.asarray(
            self.config["stiffness"], dtype=np.float64
        ).reshape(5)
        self.damping = np.asarray(
            self.config["damping"], dtype=np.float64
        ).reshape(5)
        self.required_motion_state = str(self.config["required_motion_state"])
        self.publish_frequency = float(self.config["publish_frequency"])
        self.max_state_age = float(self.config["max_state_age"])
        self.max_joint_delta = float(self.config["max_joint_delta_per_publish"])
        self.max_joint_velocity = float(self.config["max_joint_velocity"])
        self.output_arm_duration = float(
            self.config.get("output_arm_duration", 1.0)
        )
        self.baseline_duration = float(self.config["baseline_duration"])
        self.baseline_velocity_threshold = float(
            self.config["baseline_velocity_threshold"]
        )
        self.prep_duration = float(self.config["duration"]) + float(
            self.config.get("settle_duration", 0.0)
        )
        self.command_weight = float(self.config["weight"])

        self._lock = threading.Lock()
        self._full_position: np.ndarray | None = None
        self._full_velocity: np.ndarray | None = None
        self._full_torque: np.ndarray | None = None
        self._rx_monotonic = 0.0
        self._first_rx_monotonic = 0.0
        self._motion_state = ""
        self._desired_target: np.ndarray | None = None
        self._last_published_target: np.ndarray | None = None
        self._baseline_samples: list[np.ndarray] = []
        self._baseline_started_at = 0.0
        self._torque_offset: np.ndarray | None = None
        self._fault_reason = ""
        self._output_safe_since = 0.0
        self._output_armed = False
        self._closed = False

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True
        else:
            self._owns_rclpy = False

        self.node = rclpy.create_node("mcc_t800_right_arm")
        joint_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        motion_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/hardware/joint_state",
            self._joint_state_callback,
            joint_state_qos,
        )
        self.motion_state_sub = self.node.create_subscription(
            MotionState,
            "/motion/motion_state",
            self._motion_state_callback,
            motion_state_qos,
        )

        self.command_pub = None
        self.publish_timer = None
        if self.mode in {"zero_replay", "command"}:
            command_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.command_pub = self.node.create_publisher(
                JointOverrideCommand,
                "/motion/joint_override_command",
                command_qos,
            )
            self.publish_timer = self.node.create_timer(
                1.0 / self.publish_frequency,
                self._publish_timer_callback,
            )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            name="mcc-t800-ros2",
            daemon=True,
        )
        self.executor_thread.start()

    def _joint_state_callback(self, msg: JointState) -> None:
        pos = np.asarray(msg.position, dtype=np.float64).reshape(-1)
        vel = np.asarray(msg.velocity, dtype=np.float64).reshape(-1)
        tor = np.asarray(msg.torque, dtype=np.float64).reshape(-1)
        required_width = int(np.max(self.joint_indices)) + 1
        if min(pos.size, vel.size, tor.size) < required_width:
            self._fault_reason = (
                "joint_state arrays are shorter than required T800 joint index "
                f"{required_width - 1}: pos={pos.size}, vel={vel.size}, tor={tor.size}"
            )
            return
        now = time.monotonic()
        with self._lock:
            self._full_position = pos
            self._full_velocity = vel
            self._full_torque = tor
            self._rx_monotonic = now
            if self._first_rx_monotonic <= 0.0:
                self._first_rx_monotonic = now

    def _motion_state_callback(self, msg: MotionState) -> None:
        with self._lock:
            self._motion_state = str(msg.current_motion_task)

    def _arm_snapshot(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, str]:
        with self._lock:
            if (
                self._full_position is None
                or self._full_velocity is None
                or self._full_torque is None
            ):
                raise RuntimeError("No T800 /hardware/joint_state sample received yet.")
            return (
                self._full_position[self.joint_indices].astype(np.float32),
                self._full_velocity[self.joint_indices].astype(np.float32),
                self._full_torque[self.joint_indices].astype(np.float32),
                float(self._rx_monotonic),
                str(self._motion_state),
            )

    def _model_bias(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        self.data.qpos[:] = np.asarray(q, dtype=np.float64)
        self.data.qvel[:] = np.asarray(dq, dtype=np.float64)
        mujoco.mj_forward(self.model, self.data)
        return np.asarray(self.data.qfrc_bias, dtype=np.float32).copy()

    def _update_baseline(
        self,
        now: float,
        q: np.ndarray,
        dq: np.ndarray,
        tau: np.ndarray,
        bias: np.ndarray,
    ) -> None:
        if self._torque_offset is not None:
            return
        if self._first_rx_monotonic <= 0.0:
            return
        if now - self._first_rx_monotonic < self.prep_duration:
            return
        if float(np.max(np.abs(dq))) > self.baseline_velocity_threshold:
            self._baseline_samples.clear()
            self._baseline_started_at = 0.0
            return
        if self._baseline_started_at <= 0.0:
            self._baseline_started_at = now
        self._baseline_samples.append((tau - bias).astype(np.float32))
        if now - self._baseline_started_at >= self.baseline_duration:
            self._torque_offset = np.mean(
                np.stack(self._baseline_samples), axis=0
            ).astype(np.float32)
            self.node.get_logger().info(
                "T800 right-arm torque baseline ready: "
                + np.array2string(self._torque_offset, precision=4)
            )

    def get_observation(self, retries: int = 0) -> Obs:
        deadline = None if int(retries) < 0 else time.monotonic() + 3.0
        while True:
            if self._closed or not rclpy.ok():
                raise RuntimeError("T800 ROS 2 backend is shutting down.")
            try:
                q, dq, tau, rx_time, _ = self._arm_snapshot()
                break
            except RuntimeError:
                if deadline is not None and time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)

        now = time.monotonic()
        bias = self._model_bias(q, dq)
        self._update_baseline(now, q, dq, tau, bias)
        # Until the static baseline is ready, make the MCC residual exactly zero.
        # This lets the existing preparation trajectory run without interpreting
        # its PD/inertial reaction as human contact.
        motor_tor = (
            bias
            if self._torque_offset is None
            else tau - self._torque_offset
        )
        return Obs(
            ang_vel=np.zeros(3, dtype=np.float32),
            time=float(rx_time),
            motor_pos=q.copy(),
            motor_vel=dq.copy(),
            motor_tor=np.asarray(motor_tor, dtype=np.float32).copy(),
            qpos=q.copy(),
            qvel=dq.copy(),
            rot=R.identity(),
        )

    def set_motor_target(self, motor_target: np.ndarray) -> None:
        target = np.asarray(motor_target, dtype=np.float64).reshape(-1)
        if target.shape != (5,):
            raise ValueError(f"T800 MCC target must have shape (5,), got {target.shape}.")
        if not np.all(np.isfinite(target)):
            self._fault_reason = "non-finite MCC motor target"
            return
        with self._lock:
            self._desired_target = target.copy()

    def _publish_timer_callback(self) -> None:
        if self.command_pub is None or self._closed:
            return
        try:
            q, dq, _, rx_time, motion_state = self._arm_snapshot()
        except RuntimeError:
            return
        # Joint state commonly arrives before the first motion-state sample.
        # Stay silent until the safety state is initialized instead of turning
        # this normal startup race into a sticky fault.
        if not motion_state:
            self._output_safe_since = 0.0
            return
        now = time.monotonic()
        if now - rx_time > self.max_state_age:
            self._fault_reason = "stale T800 joint state"
            return
        if motion_state != self.required_motion_state:
            self._fault_reason = (
                f"motion state {motion_state!r} != {self.required_motion_state!r}"
            )
            return
        abs_velocity = np.abs(dq)
        max_velocity_index = int(np.argmax(abs_velocity))
        max_velocity = float(abs_velocity[max_velocity_index])
        if max_velocity > self.max_joint_velocity:
            if not self._output_armed:
                self._output_safe_since = 0.0
                return
            joint_index = int(self.joint_indices[max_velocity_index])
            self._fault_reason = (
                "T800 right-arm velocity safety limit exceeded: "
                f"J{joint_index:02d}={float(dq[max_velocity_index]):.4f} rad/s, "
                f"limit={self.max_joint_velocity:.4f}, "
                f"arm_velocity={np.array2string(dq, precision=4)}"
            )
            return

        if not self._output_armed:
            if self._output_safe_since <= 0.0:
                self._output_safe_since = now
                return
            if now - self._output_safe_since < self.output_arm_duration:
                return
            self._output_armed = True
            self.node.get_logger().info(
                "T800 output armed after "
                f"{self.output_arm_duration:.2f}s of stable right-arm velocity"
            )

        if self.mode == "zero_replay":
            requested = q.astype(np.float64)
        else:
            with self._lock:
                if self._desired_target is None:
                    return
                requested = self._desired_target.copy()

        joint_range = np.asarray(self.model.jnt_range, dtype=np.float64)
        requested = np.clip(requested, joint_range[:, 0], joint_range[:, 1])
        if self._last_published_target is None:
            self._last_published_target = q.astype(np.float64)
        delta = np.clip(
            requested - self._last_published_target,
            -self.max_joint_delta,
            self.max_joint_delta,
        )
        bounded = self._last_published_target + delta
        self._last_published_target = bounded.copy()

        msg = JointOverrideCommand()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.weight = float(self.command_weight)
        msg.joint_indices = self.joint_indices.tolist()
        msg.position = bounded.tolist()
        msg.velocity = np.zeros(5, dtype=np.float64).tolist()
        msg.feed_forward_torque = np.zeros(5, dtype=np.float64).tolist()
        msg.torque = np.zeros(5, dtype=np.float64).tolist()
        msg.stiffness = self.stiffness.tolist()
        msg.damping = self.damping.tolist()
        self.command_pub.publish(msg)

    def step(self) -> None:
        return None

    def sync(self) -> bool:
        if self._fault_reason:
            self.node.get_logger().error(self._fault_reason)
            return False
        return rclpy.ok() and not self._closed

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.command_pub is not None:
            try:
                q, _, _, _, _ = self._arm_snapshot()
                msg = JointOverrideCommand()
                msg.header = Header()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.weight = 0.0
                msg.joint_indices = self.joint_indices.tolist()
                msg.position = q.astype(np.float64).tolist()
                msg.velocity = np.zeros(5).tolist()
                msg.feed_forward_torque = np.zeros(5).tolist()
                msg.torque = np.zeros(5).tolist()
                msg.stiffness = self.stiffness.tolist()
                msg.damping = self.damping.tolist()
                for _ in range(5):
                    self.command_pub.publish(msg)
                    time.sleep(0.01)
            except RuntimeError:
                pass
        self.executor.shutdown(timeout_sec=1.0)
        if self.executor_thread.is_alive():
            self.executor_thread.join(timeout=1.0)
        self.node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
