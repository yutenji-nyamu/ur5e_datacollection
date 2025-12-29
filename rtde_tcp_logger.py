#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RTDE TCP pose logging - refactored into a small class + helpers.

This is based on your rtde_collect_2_csv_func.py, but:
- Exposes a RtdeTcpClient and a RtdeTcpCsvLogger class
- Designed to be called by an orchestrator (e.g., arm+gripper collection)

Notes:
- RTDE output fields: timestamp, actual_TCP_pose
- timestamp is controller time (seconds)
"""

# =========================
# CONFIG
# =========================
ROBOT_HOST = "192.168.0.3"
ROBOT_PORT = 30004

# IMPORTANT: Point this to where your RTDE_Python_Client_Library package is located.
# (You previously used a local extracted zip path.)
RTDE_PYTHON_PATH = "/home/zhangw/UR5e_DataCollection/rtde-2.7.12-release/rtde-2.7.12"

# Default logging frequency. Must match how you set output setup on the controller.
FREQUENCY_HZ = 10

import sys
import os
import csv
import json
from dataclasses import dataclass
from datetime import datetime

sys.path.append(RTDE_PYTHON_PATH)
import rtde.rtde as rtde  # type: ignore


def make_run_paths(data_dir: str, prefix: str):
    os.makedirs(data_dir, exist_ok=True)
    run_time = datetime.now()
    ts_str = run_time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(data_dir, f"{prefix}_{ts_str}.csv")
    cfg_path = os.path.join(data_dir, f"{prefix}_{ts_str}.json")
    return ts_str, csv_path, cfg_path


@dataclass
class RtdeConfig:
    robot_host: str = ROBOT_HOST
    robot_port: int = ROBOT_PORT
    frequency_hz: int = FREQUENCY_HZ


class RtdeTcpClient:
    """Small wrapper around rtde.RTDE for timestamp + actual_TCP_pose streaming."""
    def __init__(self, cfg: RtdeConfig):
        self.cfg = cfg
        self.con = None

    def connect(self):
        con = rtde.RTDE(self.cfg.robot_host, self.cfg.robot_port)
        con.connect()
        con.get_controller_version()

        ok = con.send_output_setup(["timestamp", "actual_TCP_pose"], frequency=self.cfg.frequency_hz)
        if not ok:
            con.disconnect()
            raise RuntimeError("Failed to setup RTDE output (timestamp, actual_TCP_pose).")

        ok = con.send_start()
        if not ok:
            con.disconnect()
            raise RuntimeError("Failed to start RTDE data synchronization.")
        self.con = con

    def receive(self):
        if self.con is None:
            raise RuntimeError("RTDE not connected. Call connect() first.")
        state = self.con.receive()
        if state is None:
            return None
        ts = float(state.timestamp)
        tcp = list(state.actual_TCP_pose)
        return ts, tcp

    def close(self):
        if self.con is None:
            return
        try:
            self.con.send_pause()
        except Exception:
            pass
        try:
            self.con.disconnect()
        except Exception:
            pass
        self.con = None


class RtdeTcpCsvLogger:
    """
    Writes RTDE TCP stream into a CSV file.
    The orchestrator owns the loop; this logger only provides write_row().
    """
    def __init__(self, csv_path: str):
        self.csv_path = csv_path
        self._f = open(self.csv_path, "w", newline="")
        self._writer = csv.writer(self._f)
        self._writer.writerow([
            "controller_time_s",
            "tcp_x", "tcp_y", "tcp_z",
            "tcp_rx", "tcp_ry", "tcp_rz",
        ])
        self._f.flush()

    def write_row(self, controller_ts_s: float, tcp_pose_6):
        self._writer.writerow([controller_ts_s] + list(tcp_pose_6))

    def flush(self):
        self._f.flush()

    def close(self):
        try:
            self._f.flush()
        except Exception:
            pass
        try:
            self._f.close()
        except Exception:
            pass


def write_config_json(cfg_path: str, cfg: RtdeConfig, extra: dict):
    payload = {
        "run_start_time": datetime.now().isoformat(),
        "robot_host": cfg.robot_host,
        "robot_port": cfg.robot_port,
        "frequency_hz": cfg.frequency_hz,
        "rtde_fields": ["timestamp", "actual_TCP_pose"],
        "csv_columns": [
            "controller_time_s",
            "tcp_x", "tcp_y", "tcp_z",
            "tcp_rx", "tcp_ry", "tcp_rz",
        ],
        **(extra or {}),
    }
    with open(cfg_path, "w") as f:
        json.dump(payload, f, indent=2)


if __name__ == "__main__":
    # Minimal standalone test (Ctrl+C to stop)
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    DATA_DIR = os.path.join(SCRIPT_DIR, "action_data")

    cfg = RtdeConfig()
    ts_str, csv_path, cfg_path = make_run_paths(DATA_DIR, "rtde_tcp")

    client = RtdeTcpClient(cfg)
    client.connect()

    logger = RtdeTcpCsvLogger(csv_path)
    write_config_json(cfg_path, cfg, extra={"notes": "Standalone RTDE TCP logger test."})

    print(f"[RTDE] logging to: {csv_path}")
    try:
        while True:
            out = client.receive()
            if out is None:
                print("[RTDE] connection closed by robot.")
                break
            t, tcp = out
            logger.write_row(t, tcp)
            logger.flush()
            print(f"{t:8.3f} | {tcp}")
    except KeyboardInterrupt:
        pass
    finally:
        logger.close()
        client.close()
        print(f"[RTDE] saved: {csv_path}")
