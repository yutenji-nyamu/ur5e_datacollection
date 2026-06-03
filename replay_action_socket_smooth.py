#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
replay_action_socket.py

Replay collected UR5e TCP pose + gripper events with simple socket control.

This version adds light smoothing:
1. unwrap rotation vectors to avoid equivalent axis-angle branch jumps;
2. compute each segment's movel speed from waypoint distance;
3. add a small movel blend radius when neighboring segments are long enough.

Robot mode:
- Switch PolyScope to Remote Control.
- Make sure the robot is in a safe state and not in freedrive.
"""

import csv
import math
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
DRY_RUN = False
ENABLE_ARM = True
START_ROW = 0
MAX_STEPS = None

# Move command parameters
MOVE_A = 0.4
MOVE_V = 0.04
GO_TO_START_FIRST = True
GO_TO_START_WAIT_S = 3.0

# Light smoothing
# 1. Rotation vectors from RTDE may jump between equivalent axis-angle branches.
SMOOTH_ROTATION_VECTOR = True

# 2. Use per-segment speed instead of one fixed MOVE_V.
#    For 2 Hz replay, segment time is about 0.5 s.
USE_SEGMENT_SPEED = True
MOVE_V_MIN = 0.010
MOVE_V_MAX = 0.120
MOVE_V_SCALE = 1.10

# 3. Small blend radius for movel.
#    Keep this small. If the robot reports blend overlap, reduce to 0.001 or set 0.0.
MOVE_BLEND_R = 0.008
MIN_BLEND_SEGMENT_M = 0.002

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


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def trans_dist(pose_a, pose_b):
    dx = pose_a[0] - pose_b[0]
    dy = pose_a[1] - pose_b[1]
    dz = pose_a[2] - pose_b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def vec_dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(len(a))))


def rotvec_candidates(rot):
    norm = math.sqrt(rot[0] ** 2 + rot[1] ** 2 + rot[2] ** 2)
    candidates = [rot[:]]
    if norm > 1e-9:
        unit = [x / norm for x in rot]
        candidates.append([rot[i] - 2.0 * math.pi * unit[i] for i in range(3)])
        candidates.append([rot[i] + 2.0 * math.pi * unit[i] for i in range(3)])
    return candidates


def smooth_rotation_vectors(rows):
    if len(rows) < 2:
        return 0

    corrected = 0
    prev_rot = rows[0]["pose"][3:6]

    for row in rows[1:]:
        rot = row["pose"][3:6]
        candidates = rotvec_candidates(rot)
        best = min(candidates, key=lambda c: vec_dist(c, prev_rot))

        if vec_dist(best, rot) > 1e-6:
            row["pose"][3:6] = best
            corrected += 1

        prev_rot = row["pose"][3:6]

    return corrected


def build_motion_profile(rows, interval_s):
    profile = []
    speed_values = []
    blend_values = []

    for i, row in enumerate(rows):
        if i == 0:
            move_v = MOVE_V
            blend_r = 0.0
        else:
            prev_pose = rows[i - 1]["pose"]
            pose = row["pose"]
            d_prev = trans_dist(prev_pose, pose)

            if USE_SEGMENT_SPEED:
                move_v = clamp((d_prev / interval_s) * MOVE_V_SCALE, MOVE_V_MIN, MOVE_V_MAX)
            else:
                move_v = MOVE_V

            blend_r = 0.0
            if MOVE_BLEND_R > 0.0 and i < len(rows) - 1:
                next_pose = rows[i + 1]["pose"]
                d_next = trans_dist(pose, next_pose)
                if d_prev >= MIN_BLEND_SEGMENT_M and d_next >= MIN_BLEND_SEGMENT_M:
                    blend_r = min(MOVE_BLEND_R, 0.30 * d_prev, 0.30 * d_next)

        profile.append({
            "move_v": move_v,
            "blend_r": blend_r,
        })
        speed_values.append(move_v)
        blend_values.append(blend_r)

    return profile, speed_values, blend_values


def build_movel_cmd(pose, move_v=None, blend_r=0.0):
    v = MOVE_V if move_v is None else move_v
    if blend_r > 0.0:
        return f"movel({pose_to_urscript(pose)}, a={MOVE_A:.3f}, v={v:.3f}, r={blend_r:.4f})\n"
    return f"movel({pose_to_urscript(pose)}, a={MOVE_A:.3f}, v={v:.3f})\n"


def send_arm_movel(sock, pose, move_v=None, blend_r=0.0):
    cmd = build_movel_cmd(pose, move_v=move_v, blend_r=blend_r)
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


def process_events_until(event_i, gripper_events, ts, t0, step_i, ser):
    while event_i < len(gripper_events) and gripper_events[event_i]["controller_time_s"] <= ts:
        ev = gripper_events[event_i]
        print(f"[STEP {step_i:04d}] gripper event @ {ev['controller_time_s'] - t0:.3f}s: {ev['event']}")
        send_gripper_cmd(ser, ev["event"])
        event_i += 1
    return event_i


def main():
    arm_rows = load_arm_rows(RTDE_CSV_PATH)
    gripper_events = load_gripper_events(GRIPPER_EVENTS_CSV_PATH)

    if not arm_rows:
        raise RuntimeError("No arm rows loaded.")

    selected = arm_rows[START_ROW::ROW_STRIDE]
    if MAX_STEPS is not None:
        selected = selected[:MAX_STEPS]

    if SMOOTH_ROTATION_VECTOR:
        corrected_rot = smooth_rotation_vectors(selected)
    else:
        corrected_rot = 0

    interval_s = 1.0 / REPLAY_HZ
    motion_profile, speed_values, blend_values = build_motion_profile(selected, interval_s)

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
    print(f"Rotation-vector corrections: {corrected_rot}")
    if speed_values:
        print(f"Segment speed v: min={min(speed_values):.3f}, max={max(speed_values):.3f}")
    if blend_values:
        print(f"Blend r: max={max(blend_values):.4f}")
    print("")

    sock = None
    ser = None

    try:
        sock = open_robot_socket()
        ser = open_gripper_serial()

        event_i = 0
        start_i = 0

        if GO_TO_START_FIRST and selected:
            first_pose = selected[0]["pose"]
            print("[ARM] go to first replay pose")
            send_arm_movel(sock, first_pose, move_v=MOVE_V, blend_r=0.0)
            if not DRY_RUN:
                time.sleep(GO_TO_START_WAIT_S)
            start_i = 1

        for step_i in range(start_i, len(selected)):
            row = selected[step_i]
            prof = motion_profile[step_i]
            ts = row["controller_time_s"]
            rel_t = ts - t0

            event_i = process_events_until(event_i, gripper_events, ts, t0, step_i, ser)

            print(
                f"[STEP {step_i:04d}] "
                f"row={row['row_index']} "
                f"t={rel_t:.3f}s "
                f"v={prof['move_v']:.3f} "
                f"r={prof['blend_r']:.4f}"
            )
            send_arm_movel(sock, row["pose"], move_v=prof["move_v"], blend_r=prof["blend_r"])

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
