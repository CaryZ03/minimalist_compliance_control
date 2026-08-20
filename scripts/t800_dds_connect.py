#!/usr/bin/env python3
"""Select a T800 robot and prepare direct CycloneDDS routing from the server.

The script deliberately never accepts or stores passwords. SSH and sudo inherit
the controlling terminal so that credentials can be entered interactively.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import socket
import subprocess
import sys
from pathlib import Path
from typing import Any


ROBOTS = {
    "t800a": "10.249.86.80",
    "t800b": "10.249.86.82",
    "t800c": "10.249.86.84",
    "t800d": "10.249.86.103",
    "t800e": "10.249.86.88",
}
ORIN_INTERNAL = "192.168.0.162"
NEZHA_INTERNAL = "192.168.0.163"
REQUIRED_TOPICS = {
    "/hardware/joint_state",
    "/motion/motion_state",
    "/motion/joint_override_command",
}


class ConnectError(RuntimeError):
    pass


class T800DDS:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.config_dir = Path(args.config_dir).expanduser().resolve()
        self.state_path = self.config_dir / "active.json"
        self.env_path = self.config_dir / "active.env"
        self.xml_path = self.config_dir / "cyclonedds.xml"

    def run(
        self,
        command: list[str],
        *,
        check: bool = True,
        capture: bool = False,
        mutating: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        print(f"+ {shlex.join(command)}")
        if self.args.dry_run:
            return subprocess.CompletedProcess(command, 0, "", "")
        return subprocess.run(command, check=check, text=True, capture_output=capture)

    def server_ssh(
        self,
        remote: list[str],
        *,
        check: bool = True,
        mutating: bool = False,
        destination: str | None = None,
        port: int | None = None,
    ):
        command = [
            "ssh",
            "-p",
            str(port or self.args.server_ssh_port),
            "-tt" if mutating else "-T",
            destination or self.args.server_ssh,
            *remote,
        ]
        return self.run(command, check=check, capture=not mutating, mutating=mutating)

    def robot_ssh(
        self,
        alias: str,
        remote: list[str],
        *,
        check: bool = True,
        mutating: bool = False,
    ):
        command = ["ssh", "-tt" if mutating else "-T", alias, *remote]
        return self.run(command, check=check, capture=not mutating, mutating=mutating)

    def load_state(self) -> dict[str, Any] | None:
        if not self.state_path.exists():
            return None
        try:
            return json.loads(self.state_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ConnectError(f"cannot read {self.state_path}: {exc}") from exc

    def confirm(self, prompt: str) -> None:
        if self.args.yes or self.args.dry_run:
            return
        if input(f"{prompt} [y/N] ").strip().lower() not in {"y", "yes"}:
            raise ConnectError("cancelled")

    def host_route(
        self,
        destination: str,
        gateway: str,
        action: str,
        *,
        check: bool = True,
        interface: str | None = None,
        server_ssh: str | None = None,
        server_port: int | None = None,
    ):
        remote = ["sudo", "--", "ip", "route", action, f"{destination}/32"]
        remote += ["via", gateway, "dev", interface or self.args.server_interface]
        return self.server_ssh(
            remote,
            check=check,
            mutating=True,
            destination=server_ssh,
            port=server_port,
        )

    def nezha_route(
        self,
        robot: str,
        action: str,
        *,
        check: bool = True,
        server_ip: str | None = None,
    ):
        remote = [
            "sudo", "--", "ip", "route", action,
            f"{server_ip or self.args.server_ip}/32", "via", ORIN_INTERNAL,
        ]
        return self.robot_ssh(f"{robot}-nezha", remote, check=check, mutating=True)

    def check_forwarding(self, robot: str) -> None:
        if self.args.configure_forwarding:
            self.robot_ssh(
                robot,
                ["sudo", "--", "sysctl", "-w", "net.ipv4.ip_forward=1"],
                mutating=True,
            )
            return
        result = self.robot_ssh(
            robot,
            ["cat", "/proc/sys/net/ipv4/ip_forward"],
        )
        if not self.args.dry_run and result.stdout.strip() != "1":
            raise ConnectError(
                f"{robot} has IPv4 forwarding disabled; rerun with --configure-forwarding"
            )

    def write_runtime_files(self, robot: str, external_ip: str) -> None:
        if self.args.dry_run:
            print(f"+ write runtime files under {self.config_dir}")
            return
        self.config_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        xml = f"""<CycloneDDS>
  <Domain id=\"any\">
    <General>
      <Interfaces><NetworkInterface name=\"{self.args.server_interface}\"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address=\"{ORIN_INTERNAL}\"/>
        <Peer address=\"{NEZHA_INTERNAL}\"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""
        state = {
            "robot": robot,
            "external_ip": external_ip,
            "server_ip": self.args.server_ip,
            "server_interface": self.args.server_interface,
            "server_ssh": self.args.server_ssh,
            "server_ssh_port": self.args.server_ssh_port,
        }
        env = {
            "ROS_DOMAIN_ID": "69",
            "ROS_LOCALHOST_ONLY": "0",
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "CYCLONEDDS_URI": self.xml_path.as_uri(),
            "T800_DDS_ROBOT": robot,
        }
        self.xml_path.write_text(xml)
        self.state_path.write_text(json.dumps(state, indent=2) + "\n")
        self.env_path.write_text(
            "\n".join(f"export {key}={shlex.quote(value)}" for key, value in env.items()) + "\n"
        )
        for path in (self.xml_path, self.state_path, self.env_path):
            path.chmod(0o600)

    def connect(self, robot: str) -> None:
        old = self.load_state()
        if old and old.get("robot") != robot:
            self.confirm(f"switch DDS routing from {old['robot']} to {robot}?")
            self.disconnect(old)

        external_ip = ROBOTS[robot]
        self.check_forwarding(robot)
        routes_added: list[str] = []
        try:
            for destination in (ORIN_INTERNAL, NEZHA_INTERNAL):
                self.host_route(destination, external_ip, "replace")
                routes_added.append(destination)
            self.nezha_route(robot, "replace")
        except subprocess.CalledProcessError as exc:
            for destination in reversed(routes_added):
                self.host_route(destination, external_ip, "del", check=False)
            raise ConnectError(f"route configuration failed (exit {exc.returncode})") from exc

        self.write_runtime_files(robot, external_ip)
        print(f"connected routing to {robot} ({external_ip})")
        print(f"run: source {shlex.quote(str(self.env_path))}")
        if not self.args.dry_run and not self.tcp_probe():
            raise ConnectError(
                "routing is installed, but the Nezha TCP probe failed; "
                "check forwarding/firewall, then run status or disconnect"
            )

    def disconnect(self, state: dict[str, Any] | None = None) -> None:
        state = state or self.load_state()
        if not state:
            print("no active T800 DDS selection")
            return
        robot = str(state["robot"])
        gateway = str(state["external_ip"])
        self.nezha_route(robot, "del", check=False, server_ip=str(state["server_ip"]))
        for destination in (ORIN_INTERNAL, NEZHA_INTERNAL):
            self.host_route(
                destination,
                gateway,
                "del",
                check=False,
                interface=str(state["server_interface"]),
                server_ssh=str(state["server_ssh"]),
                server_port=int(state["server_ssh_port"]),
            )
        if not self.args.dry_run:
            for path in (self.state_path, self.env_path, self.xml_path):
                path.unlink(missing_ok=True)
        print(f"disconnected routing from {robot}")

    def tcp_probe(self) -> bool:
        try:
            with socket.create_connection((NEZHA_INTERNAL, 22), timeout=2):
                print(f"TCP probe OK: {NEZHA_INTERNAL}:22")
                return True
        except OSError as exc:
            print(f"TCP probe FAILED: {NEZHA_INTERNAL}:22 ({exc})", file=sys.stderr)
            return False

    def status(self) -> int:
        state = self.load_state()
        if not state:
            print("no active T800 DDS selection")
            return 1
        print(json.dumps(state, indent=2))
        return 0 if self.tcp_probe() else 2

    def probe(self) -> int:
        if not self.load_state():
            raise ConnectError("no active selection; run connect first")
        env = os.environ.copy()
        env.update(
            ROS_DOMAIN_ID="69",
            ROS_LOCALHOST_ONLY="0",
            RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
            CYCLONEDDS_URI=self.xml_path.as_uri(),
        )
        command = [
            "ros2", "topic", "list", "-t", "--include-hidden-topics",
            "--no-daemon", "--spin-time", "5",
        ]
        print(f"+ {shlex.join(command)}")
        result = subprocess.run(command, text=True, capture_output=True, env=env)
        if result.stdout:
            print(result.stdout, end="")
        if result.returncode:
            if result.stderr:
                print(result.stderr, end="", file=sys.stderr)
            return result.returncode
        found = {line.split()[0] for line in result.stdout.splitlines() if line.startswith("/")}
        missing = sorted(REQUIRED_TOPICS - found)
        if missing:
            print("missing required topics: " + ", ".join(missing), file=sys.stderr)
            return 2
        print("required T800 topics discovered")
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="print commands without running them")
    parser.add_argument("--yes", action="store_true", help="do not prompt when switching robots")
    parser.add_argument("--server-ip", default=os.getenv("T800_DDS_SERVER_IP", "10.249.86.100"))
    parser.add_argument("--server-interface", default=os.getenv("T800_DDS_SERVER_INTERFACE", "enp4s0"))
    parser.add_argument("--server-ssh", default=os.getenv("T800_DDS_SERVER_SSH", "zhangkairui@10.249.86.100"))
    parser.add_argument("--server-ssh-port", type=int, default=int(os.getenv("T800_DDS_SERVER_SSH_PORT", "2222")))
    parser.add_argument("--config-dir", default=os.getenv("T800_DDS_CONFIG_DIR", "~/.config/t800-dds"))
    parser.add_argument(
        "--configure-forwarding",
        action="store_true",
        help="enable IPv4 forwarding on the selected Orin using sudo",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    connect = subparsers.add_parser("connect", help="select and route to a robot")
    connect.add_argument("robot", choices=sorted(ROBOTS))
    subparsers.add_parser("disconnect", help="remove routes for the active robot")
    subparsers.add_parser("status", help="show selection and test Nezha TCP reachability")
    subparsers.add_parser("probe", help="discover and validate required ROS 2 topics")
    subparsers.add_parser("env", help="print the shell command for loading DDS settings")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manager = T800DDS(args)
    try:
        if args.command == "connect":
            manager.connect(args.robot)
            return 0
        if args.command == "disconnect":
            manager.disconnect()
            return 0
        if args.command == "status":
            return manager.status()
        if args.command == "probe":
            return manager.probe()
        if not manager.load_state():
            raise ConnectError("no active selection; run connect first")
        print(f"source {shlex.quote(str(manager.env_path))}")
        return 0
    except (ConnectError, subprocess.CalledProcessError, FileNotFoundError) as exc:
        print(f"t800-dds: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
