"""Minimal, headless MCC runtime for an EngineAI T800 Orin."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import replace
from pathlib import Path
from typing import Sequence

import gin
import numpy as np

from minimalist_compliance_control.controller import (
    ComplianceController,
    ControllerConfig,
    RefConfig,
)
from minimalist_compliance_control.wrench_estimation import WrenchEstimateConfig
from policy.compliance import CompliancePolicy
from real_world.real_world_t800 import RealWorldT800


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "config" / "t800_runtime.gin"


def _resolve(path: str | None) -> str | None:
    if path is None:
        return None
    candidate = Path(path)
    if not candidate.is_absolute():
        candidate = REPO_ROOT / candidate
    return str(candidate.resolve())


def build_controller(config_path: str) -> tuple[ComplianceController, str]:
    gin.clear_config()
    gin.parse_config_file(str(Path(config_path).resolve()))

    controller_config = ControllerConfig()
    ref_config = RefConfig()
    xml_path = _resolve(controller_config.xml_path)
    if xml_path is None:
        raise ValueError("ControllerConfig.xml_path is required")

    controller_config = replace(controller_config, xml_path=xml_path)
    ref_config = replace(
        ref_config,
        fixed_model_xml_path=_resolve(ref_config.fixed_model_xml_path),
    )
    controller = ComplianceController(
        config=controller_config,
        estimate_config=WrenchEstimateConfig(),
        ref_config=ref_config,
    )
    return controller, xml_path


def run(args: argparse.Namespace) -> None:
    controller, xml_path = build_controller(args.config)
    if args.smoke_test:
        model = controller.wrench_sim.model
        print(
            "T800 MCC smoke test OK: "
            f"nq={model.nq}, nv={model.nv}, nu={model.nu}, xml={xml_path}"
        )
        controller.close()
        return

    if args.mode in {"zero_replay", "command"} and not args.allow_output:
        controller.close()
        raise RuntimeError(
            f"mode {args.mode!r} creates a command publisher; "
            "pass --allow-output after completing shadow validation"
        )

    sim = RealWorldT800(
        control_dt=float(controller.ref_config.dt),
        xml_path=xml_path,
        mode=args.mode,
    )
    policy: CompliancePolicy | None = None
    try:
        initial_observation = sim.get_observation(retries=-1)
        policy = CompliancePolicy(
            name="compliance",
            robot="t800",
            init_motor_pos=initial_observation.motor_pos,
            controller=controller,
            show_help=False,
            start_keyboard_listener=False,
            enable_plotter=False,
            enable_force_perturbation=False,
        )

        control_dt = float(policy.control_dt)
        started = time.monotonic()
        first_sample_time: float | None = None
        next_tick = started
        steps = 0
        print(
            f"T800 MCC runtime started: mode={args.mode}, "
            f"dt={control_dt:.4f}s, duration={args.duration or 'unlimited'}"
        )

        while True:
            observation = sim.get_observation(retries=-1)
            if first_sample_time is None:
                first_sample_time = float(observation.time)
            observation.time -= first_sample_time

            target = np.asarray(policy.step(observation, sim), dtype=np.float32)
            sim.set_motor_target(target)
            sim.step()
            if not sim.sync():
                raise RuntimeError("T800 backend reported a safety or DDS fault")

            steps += 1
            elapsed = time.monotonic() - started
            if args.duration > 0.0 and elapsed >= args.duration:
                break

            next_tick += control_dt
            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()

        elapsed = max(time.monotonic() - started, 1e-9)
        print(
            f"T800 MCC runtime completed: steps={steps}, "
            f"average_rate={steps / elapsed:.1f}Hz"
        )
    except KeyboardInterrupt:
        print("T800 MCC runtime stopped by operator")
    finally:
        if policy is not None:
            policy.close()
        else:
            controller.close()
        sim.close()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode",
        nargs="?",
        default="shadow",
        choices=["readonly", "shadow", "zero_replay", "command"],
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; zero runs until Ctrl-C",
    )
    parser.add_argument(
        "--allow-output",
        action="store_true",
        help="required for zero_replay and command modes",
    )
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="load configuration and model without creating a ROS node",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    args = parse_args(argv)
    if args.duration < 0.0:
        raise ValueError("--duration must be non-negative")
    try:
        run(args)
    except RuntimeError as exc:
        print(f"T800 MCC runtime error: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
