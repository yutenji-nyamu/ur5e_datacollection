#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Collect UR5e RTDE TCP pose + gripper (serial) commands into the same run.

Interaction model (minimal, no extra deps):
- Runs a continuous RTDE receive loop at configured frequency
- You type commands in the terminal:
    c + Enter : gripper close
    o + Enter : gripper open
    q + Enter : quit
- Logs:
    1) rtde_tcp_gripper_*.csv : RTDE stream + last_gripper_state
    2) gripper_events_*.csv   : discrete gripper events

This is intended for demonstration / teleop data collection where arm motion is manual,
and gripper open/close is user-triggered.

Requirements:
- Polyscope: RTDE enabled / robot reachable (ROBOT_HOST)
- Serial permission: user in 'dialout' (no sudo)
"""

# =========================
# CONFIG
# =========================
ROBOT_HOST = "192.168.0.3"
ROBOT_PORT = 30004
FREQUENCY_HZ = 10

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 9600
SERIAL_TIMEOUT_S = 1.0

# Freedrive automation
ENABLE_FREEDRIVE_ON_START = True
# (Optional) use same HOST/PORT as URScript channel; if different, set here
FREEDRIVE_HOST = ROBOT_HOST
FREEDRIVE_PORT = 30001
FREEDRIVE_TIMEOUT_S = 10.0

# Where to write logs (default: alongside this script)
# You can set this to /home/zhangw/UR5e_DataCollection/action_data if you prefer.
import os
from pathlib import Path
SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = str(SCRIPT_DIR / "action_data")

# =========================
# Implementation
# =========================
import csv
import json
import time
import sys
import select
from datetime import datetime

from rtde_tcp_logger import RtdeConfig, RtdeTcpClient, make_run_paths, write_config_json
from gripper_serial import GripperSerial
from freedrive_urscript import start_freedrive, stop_freedrive


def _nonblocking_readline():
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


def main():
    os.makedirs(DATA_DIR, exist_ok=True)

    # Prepare file names
    ts_str, rtde_csv_path, rtde_cfg_path = make_run_paths(DATA_DIR, "rtde_tcp_gripper")
    _, gripper_csv_path, _ = make_run_paths(DATA_DIR, "gripper_events")

    # RTDE setup
    cfg = RtdeConfig(robot_host=ROBOT_HOST, robot_port=ROBOT_PORT, frequency_hz=FREQUENCY_HZ)
    rtde_client = RtdeTcpClient(cfg)
    rtde_client.connect()

    # CSV writers
    rtde_f = open(rtde_csv_path, "w", newline="")
    rtde_w = csv.writer(rtde_f)
    rtde_w.writerow([
        "controller_time_s",
        "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
        "gripper_state",           # 0=open, 1=closed (last commanded)
        "gripper_event_counter",   # increments only when a command is sent
    ])
    rtde_f.flush()

    grip_f = open(gripper_csv_path, "w", newline="")
    grip_w = csv.writer(grip_f)
    grip_w.writerow(["controller_time_s", "event", "gripper_state"])
    grip_f.flush()

    # Gripper
    gripper = GripperSerial(port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE, timeout_s=SERIAL_TIMEOUT_S)

    # State
    gripper_state = 0  # assume open
    gripper_event_counter = 0
    last_controller_ts = None

    # Save config
    write_config_json(
        rtde_cfg_path,
        cfg,
        extra={
            "serial_port": SERIAL_PORT,
            "serial_baudrate": SERIAL_BAUDRATE,
            "rtde_csv": rtde_csv_path,
            "gripper_events_csv": gripper_csv_path,
            "gripper_state_encoding": {"open": 0, "closed": 1},
            "control_keys": {"close": "c", "open": "o", "quit": "q"},
        },
    )

    if ENABLE_FREEDRIVE_ON_START:
        try:
            start_freedrive(host=FREEDRIVE_HOST, port=FREEDRIVE_PORT, timeout_s=FREEDRIVE_TIMEOUT_S)
            print("[FREEDRIVE] enabled (hand-guiding without pendant hold).")
        except Exception as e:
            print("[FREEDRIVE] enable failed:", repr(e))
            print("Most common cause: robot is in LOCAL mode or URScript ports are blocked.")

    print("=== Collecting RTDE TCP + Gripper Commands ===")
    print(f"[RTDE CSV] {rtde_csv_path}")
    print(f"[GRIP CSV] {gripper_csv_path}")
    print("Type: 'c' (close), 'o' (open), 'q' (quit) then press Enter.\n")

    try:
        while True:
            out = rtde_client.receive()
            if out is None:
                print("[RTDE] connection closed by robot.")
                break
            controller_ts, tcp = out
            last_controller_ts = controller_ts

            # Handle user input (non-blocking)
            cmd = _nonblocking_readline()
            if cmd:
                if cmd == "c":
                    gripper.close()
                    gripper_state = 1
                    gripper_event_counter += 1
                    grip_w.writerow([controller_ts, "close", gripper_state])
                    grip_f.flush()
                    print(f"[GRIP] close @ {controller_ts:.3f}s")
                elif cmd == "o":
                    gripper.open()
                    gripper_state = 0
                    gripper_event_counter += 1
                    grip_w.writerow([controller_ts, "open", gripper_state])
                    grip_f.flush()
                    print(f"[GRIP] open  @ {controller_ts:.3f}s")
                elif cmd == "q":
                    print("[EXIT] user requested quit.")
                    break
                else:
                    print(f"[WARN] unknown cmd: {cmd!r} (use c/o/q)")

            # Write RTDE row
            rtde_w.writerow([controller_ts] + list(tcp) + [gripper_state, gripper_event_counter])
            rtde_f.flush()

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C")
    finally:
        try:
            gripper.shutdown()
        except Exception:
            pass

        try:
            rtde_f.close()
        except Exception:
            pass

        try:
            grip_f.close()
        except Exception:
            pass

        if ENABLE_FREEDRIVE_ON_START:
            try:
                stop_freedrive(host=FREEDRIVE_HOST, port=FREEDRIVE_PORT, timeout_s=FREEDRIVE_TIMEOUT_S)
                print('[FREEDRIVE] disabled.')
            except Exception as e:
                print('[FREEDRIVE] disable failed:', repr(e))

        try:
            rtde_client.close()
        except Exception:
            pass

        print("\nSaved:")
        print(f"  - {rtde_csv_path}")
        print(f"  - {gripper_csv_path}")
        print(f"  - {rtde_cfg_path}")


if __name__ == "__main__":
    main()
