#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
arm_gripper_collector.py

A small, reusable collector that logs:
- UR5e RTDE TCP pose
- Gripper commands (serial) as a state + discrete events
- Optional freedrive enable/disable (URScript socket)

Designed to be called by a higher-level script that may also record cameras.

Notes
- RTDE connection logic is delegated to your existing rtde_tcp_logger.py (same as collect_arm_gripper.py).
- Gripper serial protocol is delegated to your existing gripper_serial.py.
- Freedrive URScript socket is delegated to your existing freedrive_urscript.py.
"""

from __future__ import annotations

import csv
import json
import os
import sys
import select
from dataclasses import asdict, dataclass
from typing import Optional, Tuple, Any


# External deps already used in your repo (see collect_arm_gripper.py)
from rtde_tcp_logger import RtdeConfig, RtdeTcpClient
from gripper_serial import GripperSerial
from freedrive_urscript import start_freedrive, stop_freedrive


@dataclass
class ArmGripperPaths:
    rtde_csv: str
    gripper_events_csv: str
    config_json: str


def nonblocking_readline() -> Optional[str]:
    """
    Returns a line (str, without trailing newline) if user typed something; otherwise returns None.
    Works in a normal terminal (stdin is a TTY).
    """
    rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not rlist:
        return None
    line = sys.stdin.readline()
    if not line:
        return None
    return line.strip()


class ArmGripperCollector:
    """
    Minimal stateful collector:
    - rtde_client.receive() -> (controller_ts, tcp_pose)
    - maintains last commanded gripper_state (0=open, 1=closed) and an event counter
    """

    def __init__(
        self,
        *,
        data_dir: str,
        run_id: str,
        robot_host: str,
        robot_port: int,
        frequency_hz: float,
        serial_port: str,
        serial_baudrate: int,
        serial_timeout_s: float,
        enable_freedrive_on_start: bool,
        freedrive_host: str,
        freedrive_port: int,
        freedrive_timeout_s: float,
        rtde_prefix: str = "rtde_tcp_gripper",
    ):
        os.makedirs(data_dir, exist_ok=True)

        self._run_id = run_id
        self._data_dir = data_dir

        self.paths = ArmGripperPaths(
            rtde_csv=os.path.join(data_dir, f"{rtde_prefix}_{run_id}.csv"),
            gripper_events_csv=os.path.join(data_dir, f"gripper_events_{run_id}.csv"),
            config_json=os.path.join(data_dir, f"arm_gripper_config_{run_id}.json"),
        )

        # RTDE
        self._cfg = RtdeConfig(robot_host=robot_host, robot_port=robot_port, frequency_hz=frequency_hz)
        self._rtde = RtdeTcpClient(self._cfg)

        # Gripper
        self._gripper = GripperSerial(port=serial_port, baudrate=serial_baudrate, timeout_s=serial_timeout_s)

        # State
        self.gripper_state = 0  # 0=open, 1=closed (last commanded)
        self.gripper_event_counter = 0

        # Freedrive
        self._enable_freedrive = enable_freedrive_on_start
        self._freedrive_host = freedrive_host
        self._freedrive_port = freedrive_port
        self._freedrive_timeout_s = freedrive_timeout_s

        # Writers
        self._rtde_f = open(self.paths.rtde_csv, "w", newline="")
        self._rtde_w = csv.writer(self._rtde_f)
        self._rtde_w.writerow([
            "controller_time_s",
            "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
            "gripper_state",           # 0=open, 1=closed (last commanded)
            "gripper_event_counter",   # increments only when a command is sent
        ])
        self._rtde_f.flush()

        self._grip_f = open(self.paths.gripper_events_csv, "w", newline="")
        self._grip_w = csv.writer(self._grip_f)
        self._grip_w.writerow(["controller_time_s", "event", "gripper_state"])
        self._grip_f.flush()

        # Save config JSON (one place to inspect later)
        cfg_dump = {
            "run_id": run_id,
            "rtde": asdict(self._cfg) if hasattr(self._cfg, "__dict__") else str(self._cfg),
            "paths": asdict(self.paths),
            "gripper": {
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "serial_timeout_s": serial_timeout_s,
                "state_encoding": {"open": 0, "closed": 1},
                "control_keys": {"close": "c", "open": "o", "quit": "q"},
            },
            "freedrive": {
                "enabled_on_start": enable_freedrive_on_start,
                "host": freedrive_host,
                "port": freedrive_port,
                "timeout_s": freedrive_timeout_s,
            },
        }
        with open(self.paths.config_json, "w") as f:
            json.dump(cfg_dump, f, indent=2)

        # Connect RTDE
        self._rtde.connect()

        # Enable freedrive (optional)
        if self._enable_freedrive:
            try:
                start_freedrive(host=self._freedrive_host, port=self._freedrive_port, timeout_s=self._freedrive_timeout_s)
                print("[FREEDRIVE] enabled.")
            except Exception as e:
                print("[FREEDRIVE] enable failed:", repr(e))
                print("Most common cause: robot is in LOCAL mode or URScript ports are blocked.")

    def receive(self) -> Optional[Tuple[float, Any]]:
        """
        Returns (controller_ts, tcp_pose[6]) or None if RTDE closed.
        """
        out = self._rtde.receive()
        return out

    def handle_cmd(self, cmd: str, controller_ts: Optional[float] = None) -> str:
        """
        Handle a user command.
        Returns: "ok" | "quit" | "unknown"
        """
        if cmd == "c":
            self._gripper.close()
            self.gripper_state = 1
            self.gripper_event_counter += 1
            ts = float(controller_ts) if controller_ts is not None else -1.0
            self._grip_w.writerow([ts, "close", self.gripper_state])
            self._grip_f.flush()
            print(f"[GRIP] close @ {ts:.3f}s")
            return "ok"

        if cmd == "o":
            self._gripper.open()
            self.gripper_state = 0
            self.gripper_event_counter += 1
            ts = float(controller_ts) if controller_ts is not None else -1.0
            self._grip_w.writerow([ts, "open", self.gripper_state])
            self._grip_f.flush()
            print(f"[GRIP] open  @ {ts:.3f}s")
            return "ok"

        if cmd == "q":
            print("[EXIT] user requested quit.")
            return "quit"

        print(f"[WARN] unknown cmd: {cmd!r} (use c/o/q)")
        return "unknown"

    def write_rtde_row(self, controller_ts: float, tcp_pose: Any) -> None:
        self._rtde_w.writerow([controller_ts] + list(tcp_pose) + [self.gripper_state, self.gripper_event_counter])
        self._rtde_f.flush()

    def close(self) -> None:
        # Gripper
        try:
            self._gripper.shutdown()
        except Exception:
            pass

        # Freedrive
        if self._enable_freedrive:
            try:
                stop_freedrive(host=self._freedrive_host, port=self._freedrive_port, timeout_s=self._freedrive_timeout_s)
                print("[FREEDRIVE] disabled.")
            except Exception as e:
                print("[FREEDRIVE] disable failed:", repr(e))

        # RTDE
        try:
            self._rtde.close()
        except Exception:
            pass

        # Files
        try:
            self._rtde_f.close()
        except Exception:
            pass
        try:
            self._grip_f.close()
        except Exception:
            pass
