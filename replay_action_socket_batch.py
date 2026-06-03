#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
replay_action_socket_batch.py

Replay collected UR5e TCP pose + gripper events with simple socket control.

Compared with the previous step-by-step socket implementation, this version:
1. down-samples the RTDE pose stream;
2. unwraps rotation vectors to avoid equivalent axis-angle branch jumps;
3. removes near-static duplicate waypoints;
4. splits the path at gripper events;
5. sends each motion segment as one URScript program containing multiple movel commands.

The goal is to reduce the stop-and-go behavior caused by sending one movel command
from Python, sleeping, and then sending the next command.

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
GRIPPER_ACTION_WAIT_S = 0.8

# Basic replay sampling
# Original collection is usually 10 Hz. ROW_STRIDE=5 gives roughly 2 Hz waypoints before filtering.
ROW_STRIDE = 5
START_ROW = 0
MAX_STEPS = None

# Debug / safety
DRY_RUN = False
ENABLE_ARM = True
MAX_SEGMENTS = None       # Use 1 for first real test. None means run all segments.
PRINT_URSCRIPT = False    # True prints the full generated URScript programs.

# Go-to-start motion
GO_TO_START_FIRST = True
GO_TO_START_A = 0.4
GO_TO_START_V = 0.04
GO_TO_START_WAIT_S = 3.0

# Path motion parameters
MOVE_A = 0.4
MOVE_V_DEFAULT = 0.04
MOVE_V_MIN = 0.020
MOVE_V_MAX = 0.120
MOVE_V_SCALE = 1.00
ASSUMED_POINT_DT_S = 0.5  # Used to estimate speed from distance after filtering.

# movel blend radius
MOVE_BLEND_R = 0.008
MIN_BLEND_SEGMENT_M = 0.002
BLEND_DIST_RATIO = 0.30

# Waypoint filtering
ENABLE_STATIC_FILTER = True
MIN_KEEP_TRANS_M = 0.003     # Drop near-duplicate TCP positions below 3 mm.
MIN_KEEP_ROT_RAD = 0.030     # Keep if rotation vector changed enough.
FORCE_KEEP_EVENT_NEIGHBORS = True

# Rotation-vector branch smoothing
SMOOTH_ROTATION_VECTOR = True

# Segment execution wait. Because the PC cannot directly know when a URScript segment finishes,
# wait for an estimated duration before sending the next gripper event or segment.
SEGMENT_WAIT_SCALE = 1.35
SEGMENT_WAIT_EXTRA_S = 0.25
MIN_SEGMENT_WAIT_S = 0.50

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


def rot_dist(pose_a, pose_b):
    dx = pose_a[3] - pose_b[3]
    dy = pose_a[4] - pose_b[4]
    dz = pose_a[5] - pose_b[5]
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


def find_nearest_event_neighbor_indices(rows, events):
    keep = set()
    if not rows:
        return keep

    keep.add(0)
    keep.add(len(rows) - 1)

    for ev in events:
        ts = ev["controller_time_s"]
        before = None
        after = None
        for i, row in enumerate(rows):
            if row["controller_time_s"] <= ts:
                before = i
            if row["controller_time_s"] >= ts and after is None:
                after = i
                break

        if before is not None:
            keep.add(before)
        if after is not None:
            keep.add(after)

    return keep


def filter_events_to_range(events, t_start, t_end):
    return [ev for ev in events if t_start <= ev["controller_time_s"] <= t_end]


def filter_static_waypoints(rows, events):
    if not ENABLE_STATIC_FILTER or len(rows) < 2:
        return rows[:], 0

    force_keep = find_nearest_event_neighbor_indices(rows, events) if FORCE_KEEP_EVENT_NEIGHBORS else {0, len(rows) - 1}

    filtered = []
    skipped = 0
    last_kept = None

    for i, row in enumerate(rows):
        keep = False
        if i in force_keep:
            keep = True
        elif last_kept is None:
            keep = True
        else:
            dp = trans_dist(last_kept["pose"], row["pose"])
            dr = rot_dist(last_kept["pose"], row["pose"])
            keep = (dp >= MIN_KEEP_TRANS_M) or (dr >= MIN_KEEP_ROT_RAD)

        if keep:
            filtered.append(row)
            last_kept = row
        else:
            skipped += 1

    if filtered[-1] is not rows[-1]:
        filtered.append(rows[-1])

    return filtered, skipped


def last_index_leq_ts(rows, ts):
    idx = -1
    for i, row in enumerate(rows):
        if row["controller_time_s"] <= ts:
            idx = i
        else:
            break
    return idx


def build_segments(rows, events, first_target_idx):
    segments = []
    current_start = first_target_idx

    for ev in events:
        end_idx = last_index_leq_ts(rows, ev["controller_time_s"])
        if end_idx >= current_start:
            segments.append({
                "start": current_start,
                "end": end_idx,
                "event_after": ev,
            })
            current_start = end_idx + 1
        else:
            segments.append({
                "start": current_start,
                "end": current_start - 1,
                "event_after": ev,
            })

    if current_start < len(rows):
        segments.append({
            "start": current_start,
            "end": len(rows) - 1,
            "event_after": None,
        })

    if MAX_SEGMENTS is not None:
        segments = segments[:MAX_SEGMENTS]

    return segments


def compute_target_profile(rows, target_i, segment_end_i):
    if target_i <= 0:
        move_v = MOVE_V_DEFAULT
        blend_r = 0.0
        prev_dist = 0.0
    else:
        prev_pose = rows[target_i - 1]["pose"]
        pose = rows[target_i]["pose"]
        prev_dist = trans_dist(prev_pose, pose)
        move_v = clamp((prev_dist / ASSUMED_POINT_DT_S) * MOVE_V_SCALE, MOVE_V_MIN, MOVE_V_MAX)

        blend_r = 0.0
        if MOVE_BLEND_R > 0.0 and target_i < segment_end_i:
            next_pose = rows[target_i + 1]["pose"]
            next_dist = trans_dist(pose, next_pose)
            if prev_dist >= MIN_BLEND_SEGMENT_M and next_dist >= MIN_BLEND_SEGMENT_M:
                blend_r = min(MOVE_BLEND_R, BLEND_DIST_RATIO * prev_dist, BLEND_DIST_RATIO * next_dist)

    return move_v, blend_r, prev_dist


def build_movel_line(pose, move_v, blend_r):
    if blend_r > 0.0:
        return f"  movel({pose_to_urscript(pose)}, a={MOVE_A:.3f}, v={move_v:.3f}, r={blend_r:.4f})"
    return f"  movel({pose_to_urscript(pose)}, a={MOVE_A:.3f}, v={move_v:.3f})"


def build_segment_program(rows, segment, seg_i):
    start = segment["start"]
    end = segment["end"]
    if end < start:
        return "", 0.0, []

    lines = [f"def replay_segment_{seg_i:03d}():"]
    profiles = []
    estimated_motion_s = 0.0

    for target_i in range(start, end + 1):
        move_v, blend_r, prev_dist = compute_target_profile(rows, target_i, end)
        lines.append(build_movel_line(rows[target_i]["pose"], move_v, blend_r))
        if target_i > 0:
            estimated_motion_s += prev_dist / max(move_v, 1e-6)
        profiles.append({
            "target_i": target_i,
            "row_index": rows[target_i]["row_index"],
            "controller_time_s": rows[target_i]["controller_time_s"],
            "move_v": move_v,
            "blend_r": blend_r,
            "prev_dist": prev_dist,
        })

    lines.append("end")
    program = "\n".join(lines) + "\n"
    wait_s = max(MIN_SEGMENT_WAIT_S, estimated_motion_s * SEGMENT_WAIT_SCALE + SEGMENT_WAIT_EXTRA_S)
    return program, wait_s, profiles


def build_go_to_start_cmd(pose):
    return f"movel({pose_to_urscript(pose)}, a={GO_TO_START_A:.3f}, v={GO_TO_START_V:.3f})\n"


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


def send_robot_text(sock, text, label):
    if DRY_RUN or not ENABLE_ARM:
        print(f"[ARM DRY] {label}: {len(text.splitlines())} lines")
        if PRINT_URSCRIPT:
            print(text.rstrip())
        return
    sock.sendall(text.encode("utf-8"))


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


def print_segment_summary(rows, segments):
    for i, seg in enumerate(segments):
        start = seg["start"]
        end = seg["end"]
        ev = seg["event_after"]
        if end >= start:
            target_num = end - start + 1
            row_start = rows[start]["row_index"]
            row_end = rows[end]["row_index"]
            t_start = rows[start]["controller_time_s"] - rows[0]["controller_time_s"]
            t_end = rows[end]["controller_time_s"] - rows[0]["controller_time_s"]
            motion_desc = f"targets={target_num}, rows={row_start}->{row_end}, t={t_start:.3f}->{t_end:.3f}s"
        else:
            motion_desc = "targets=0"

        if ev is None:
            event_desc = "event_after=None"
        else:
            event_desc = f"event_after={ev['event']} @ {ev['controller_time_s'] - rows[0]['controller_time_s']:.3f}s"
        print(f"[SEG {i:02d}] {motion_desc}, {event_desc}")


def main():
    arm_rows = load_arm_rows(RTDE_CSV_PATH)
    all_gripper_events = load_gripper_events(GRIPPER_EVENTS_CSV_PATH)

    if not arm_rows:
        raise RuntimeError("No arm rows loaded.")

    selected = arm_rows[START_ROW::ROW_STRIDE]
    if MAX_STEPS is not None:
        selected = selected[:MAX_STEPS]

    if not selected:
        raise RuntimeError("No selected rows after START_ROW / ROW_STRIDE / MAX_STEPS.")

    selected_t0 = selected[0]["controller_time_s"]
    selected_t1 = selected[-1]["controller_time_s"]
    gripper_events = filter_events_to_range(all_gripper_events, selected_t0, selected_t1)

    if SMOOTH_ROTATION_VECTOR:
        corrected_rot = smooth_rotation_vectors(selected)
    else:
        corrected_rot = 0

    filtered, skipped_static = filter_static_waypoints(selected, gripper_events)

    if GO_TO_START_FIRST:
        first_target_idx = 1
    else:
        first_target_idx = 0

    segments = build_segments(filtered, gripper_events, first_target_idx)

    print("=== Replay Action Socket Batch ===")
    print(f"RTDE CSV: {RTDE_CSV_PATH}")
    print(f"Gripper CSV: {GRIPPER_EVENTS_CSV_PATH}")
    print(f"Loaded arm rows: {len(arm_rows)}")
    print(f"Selected rows before filtering: {len(selected)}")
    print(f"Filtered rows: {len(filtered)}")
    print(f"Skipped near-static rows: {skipped_static}")
    print(f"Original selected duration: {selected_t1 - selected_t0:.3f}s")
    print(f"ROW_STRIDE: {ROW_STRIDE}")
    print(f"DRY_RUN: {DRY_RUN}")
    print(f"ENABLE_ARM: {ENABLE_ARM}")
    print(f"ENABLE_GRIPPER: {ENABLE_GRIPPER}")
    print(f"Gripper events in range: {len(gripper_events)}")
    print(f"Rotation-vector corrections: {corrected_rot}")
    print(f"Segments: {len(segments)}")
    print_segment_summary(filtered, segments)
    print("")

    sock = None
    ser = None

    try:
        sock = open_robot_socket()
        ser = open_gripper_serial()

        if GO_TO_START_FIRST and filtered:
            print("[ARM] go to first replay pose")
            cmd = build_go_to_start_cmd(filtered[0]["pose"])
            send_robot_text(sock, cmd, "go_to_start")
            if not DRY_RUN:
                time.sleep(GO_TO_START_WAIT_S)

        for seg_i, seg in enumerate(segments):
            program, wait_s, profiles = build_segment_program(filtered, seg, seg_i)

            if profiles:
                first = profiles[0]
                last = profiles[-1]
                max_r = max(p["blend_r"] for p in profiles)
                min_v = min(p["move_v"] for p in profiles)
                max_v = max(p["move_v"] for p in profiles)
                print(
                    f"[SEG {seg_i:02d}] send path: "
                    f"targets={len(profiles)}, "
                    f"rows={first['row_index']}->{last['row_index']}, "
                    f"v={min_v:.3f}->{max_v:.3f}, "
                    f"max_r={max_r:.4f}, "
                    f"wait={wait_s:.2f}s"
                )
                send_robot_text(sock, program, f"segment_{seg_i:03d}")
                if not DRY_RUN:
                    time.sleep(wait_s)
            else:
                print(f"[SEG {seg_i:02d}] no arm motion before event")

            ev = seg["event_after"]
            if ev is not None:
                print(f"[SEG {seg_i:02d}] gripper event: {ev['event']}")
                send_gripper_cmd(ser, ev["event"])
                if not DRY_RUN:
                    time.sleep(GRIPPER_ACTION_WAIT_S)

        print("\n[DONE] batch replay finished.")

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
