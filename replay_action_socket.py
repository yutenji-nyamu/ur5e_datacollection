#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
replay_action_socket.py

Replay collected UR5e TCP pose + gripper events with simple socket control.

Expected input format:
- rtde_tcp_gripper_*.csv from arm_gripper_collector.py
- gripper_events_*.csv from arm_gripper_collector.py

Robot mode:
- Switch PolyScope to Remote Control.
- Make sure the robot is in a safe state and not in freedrive.
"""

import csv
import os
import socket
import time

try:
    import serial
except ImportError:
    serial = None


# =========================
# CONFIG (all tunables here)
# =========================

# Input data
RTDE_CSV_PATH = "/home/zhangw/UR5e_DataCollection/action_data/rtde_tcp_gripper_20260603_140832.csv"
GRIPPER_EVENTS_CSV_PATH = "/home/zhangw/UR5e_DataCollection/action_data/gripper_events_20260603_140832.csv"

# Robot socket control
ROBOT_HOST = "192.168.0.4"
ROBOT_PORT = 30001
SOCKET_TIMEOUT_S = 10.0
SOCKET_CONNECT_WAIT_S = 1.0

# Gripper serial control
ENABLE_GRIPPER = True
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 9600
SERIAL_TIMEOUT_S = 1.0

# Replay pacing
# Your original data is 10 Hz. ROW_STRIDE=5 gives about 2 Hz.
REPLAY_HZ = 2.0
ROW_STRIDE = 5

# Debug / safety during bring-up
DRY_RUN = False          # First run should be True. Set False to control robot.
ENABLE_ARM = True
START_ROW = 0
MAX_STEPS = None          # First real run: 10~20. Full replay: set to None.

# Move command parameters
MOVE_A = 0.20
MOVE_V = 0.04
GO_TO_START_FIRST = True
GO_TO_START_WAIT_S = 3.0

# Gripper command bytes
GRIPPER_OPEN_BYTES = bytes([0x02, 0x00, 0x20, 0x2F, 0x00, 0x00, 0xA4])
GRIPPER_CLOSE_BYTES = bytes([0x02, 0x01, 0x20, 0x2F, 0x00, 0x00, 0xA4])


# =========================
# Implementation
# =========================

POSE_COLS = ["tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz"]


def load_arm_rows(csv_path):
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = ["controller_time_s"] + POSE_COLS
        missing = [c for c in required if c not in reader.fieldnames]
        if missing:
            raise ValueError(f"Missing columns in RTDE CSV: {missing}. Found: {reader.fieldnames}")

        for i, row in enumerate(reader):
            pose = [float(row[c]) for c in POSE_COLS]
            rows.append({
                "row_index": i,
                "controller_time_s": float(row["controller_time_s"]),
                "pose": pose,
            })
    return rows


def load_gripper_events(csv_path):
    if not csv_path or not os.path.exists(csv_path):
        return []

    events = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = ["controller_time_s", "event"]
        missing = [c for c in required if c not in reader.fieldnames]
        if missing:
            raise ValueError(f"Missing columns in gripper CSV: {missing}. Found: {reader.fieldnames}")

        for row in reader:
            event = row["event"].strip().lower()
            if event not in ("open", "close"):
                continue
            events.append({
                "controller_time_s": float(row["controller_time_s"]),
                "event": event,
            })

    events.sort(key=lambda x: x["controller_time_s"])
    return events


def pose_to_urscript(pose):
    return "p[" + ", ".join(f"{x:.6f}" for x in pose) + "]"


def build_movel_cmd(pose):
    return f"movel({pose_to_urscript(pose)}, a={MOVE_A:.3f}, v={MOVE_V:.3f})\n"


def send_arm_movel(sock, pose):
    cmd = build_movel_cmd(pose)
    if DRY_RUN or not ENABLE_ARM:
        print("[ARM DRY]", cmd.strip())
        return
    sock.sendall(cmd.encode("utf-8"))


def send_gripper_cmd(ser, event):
    if event == "close":
        data = GRIPPER_CLOSE_BYTES
    elif event == "open":
        data = GRIPPER_OPEN_BYTES
    else:
        return

    if DRY_RUN or not ENABLE_GRIPPER:
        print(f"[GRIP DRY] {event}")
        return

    ser.write(data)
    ser.flush()
    print(f"[GRIP] {event}")


def open_robot_socket():
    if DRY_RUN or not ENABLE_ARM:
        return None
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(SOCKET_TIMEOUT_S)
    sock.connect((ROBOT_HOST, ROBOT_PORT))
    time.sleep(SOCKET_CONNECT_WAIT_S)
    return sock


def open_gripper_serial():
    if DRY_RUN or not ENABLE_GRIPPER:
        return None
    if serial is None:
        raise ImportError("pyserial is not installed. Run: pip install pyserial")
    return serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT_S)


def main():
    arm_rows = load_arm_rows(RTDE_CSV_PATH)
    gripper_events = load_gripper_events(GRIPPER_EVENTS_CSV_PATH)

    if not arm_rows:
        raise RuntimeError("No arm rows loaded.")

    selected = arm_rows[START_ROW::ROW_STRIDE]
    if MAX_STEPS is not None:
        selected = selected[:MAX_STEPS]

    t0 = arm_rows[0]["controller_time_s"]
    t1 = arm_rows[-1]["controller_time_s"]

    print("=== Replay Action Socket ===")
    print(f"RTDE CSV: {RTDE_CSV_PATH}")
    print(f"Gripper CSV: {GRIPPER_EVENTS_CSV_PATH}")
    print(f"Loaded arm rows: {len(arm_rows)}")
    print(f"Selected replay rows: {len(selected)}")
    print(f"Original duration: {t1 - t0:.3f}s")
    print(f"Replay Hz: {REPLAY_HZ:.3f}")
    print(f"ROW_STRIDE: {ROW_STRIDE}")
    print(f"DRY_RUN: {DRY_RUN}")
    print(f"ENABLE_ARM: {ENABLE_ARM}")
    print(f"ENABLE_GRIPPER: {ENABLE_GRIPPER}")
    print(f"Gripper events: {len(gripper_events)}")
    print("")

    sock = None
    ser = None

    try:
        sock = open_robot_socket()
        ser = open_gripper_serial()

        event_i = 0

        if GO_TO_START_FIRST and selected:
            first_pose = selected[0]["pose"]
            print("[ARM] go to first replay pose")
            send_arm_movel(sock, first_pose)
            if not DRY_RUN:
                time.sleep(GO_TO_START_WAIT_S)

        interval_s = 1.0 / REPLAY_HZ

        for step_i, row in enumerate(selected):
            ts = row["controller_time_s"]
            rel_t = ts - t0

            while event_i < len(gripper_events) and gripper_events[event_i]["controller_time_s"] <= ts:
                ev = gripper_events[event_i]
                print(f"[STEP {step_i:04d}] gripper event @ {ev['controller_time_s'] - t0:.3f}s: {ev['event']}")
                send_gripper_cmd(ser, ev["event"])
                event_i += 1

            print(f"[STEP {step_i:04d}] row={row['row_index']} t={rel_t:.3f}s")
            send_arm_movel(sock, row["pose"])

            time.sleep(interval_s)

        print("\n[DONE] replay finished.")

    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
