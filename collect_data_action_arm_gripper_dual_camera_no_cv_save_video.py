#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""collect_data_action_arm_gripper_dual_camera.py

Simultaneously collect:
- Action data: UR5e RTDE TCP pose + gripper commands (state + events)
- Camera data: dual RealSense color streams (Head + Wrist)

Controls:
- Terminal input (recommended):
    c : gripper close
    o : gripper open
    q : quit

Preview:
- Default: no OpenCV windows (SHOW_PREVIEW=False) so terminal keeps focus.
- If SHOW_PREVIEW=True, windows will appear but exit is still only via terminal 'q' / Ctrl+C.

Expected mode:
- RTDE works in Local mode.
- Freedrive via URScript usually requires Remote mode; if it fails you can set ENABLE_FREEDRIVE_ON_START=False.
"""

import os
import time
import csv
import json
from datetime import datetime
import sys
import select
import termios
import tty

import numpy as np
import cv2

from realsense_dual_collect_2_folder_func import start_realsense_pipeline, list_serials
from arm_gripper_collector import ArmGripperCollector


# =========================
# CONFIG (all tunables here)
# =========================

# Robot / RTDE
ROBOT_HOST = "192.168.0.4"
# ROBOT_HOST = "192.168.0.3"
ROBOT_PORT = 30004
RTDE_HZ = 10.0

# Gripper (serial)
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 9600
SERIAL_TIMEOUT_S = 1.0

# Freedrive automation
ENABLE_FREEDRIVE_ON_START = True
FREEDRIVE_HOST = ROBOT_HOST
FREEDRIVE_PORT = 30001
FREEDRIVE_TIMEOUT_S = 10.0

# Paths (raw data)
ACTION_DATA_DIR = "/home/zhangw/UR5e_DataCollection/action_data"
CAMERA_BASE_DIR = "/home/zhangw/UR5e_DataCollection/camera_data"

# Camera (dual RealSense)
CAM_SAVE_HZ = 10.0
COLOR_WIDTH = 640
COLOR_HEIGHT = 480
COLOR_FPS = 30

# Serial numbers (fill with your actual devices)
HEAD_SERIAL = "243522072333"
WRIST_SERIAL = "233522079334"

# Show live preview windows
SHOW_PREVIEW = False

# Video preview saving
SAVE_VIDEO = True
VIDEO_FPS = CAM_SAVE_HZ
VIDEO_HEAD_NAME = "head.mp4"
VIDEO_WRIST_NAME = "wrist.mp4"
VIDEO_CODEC = "mp4v"


class TerminalKeyPoller:
    """Non-blocking single-key input (Linux TTY).

    - Uses tty.setcbreak() so keys take effect immediately without Enter.
    - Restores terminal settings on exit.
    - If stdin is not a TTY (e.g., nohup), polling is disabled.
    """

    def __init__(self):
        self._enabled = False
        self._fd = None
        self._old = None

    def __enter__(self):
        if not sys.stdin.isatty():
            return self
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)  # keep Ctrl+C signals
        # Disable local echo so single-key controls don't spam the terminal.
        try:
            attrs = termios.tcgetattr(self._fd)
            attrs[3] = attrs[3] & ~termios.ECHO
            termios.tcsetattr(self._fd, termios.TCSADRAIN, attrs)
        except Exception:
            pass
        self._enabled = True
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._enabled and self._old is not None and self._fd is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            except Exception:
                pass

    @property
    def enabled(self) -> bool:
        return self._enabled

    def poll(self) -> str | None:
        if not self._enabled:
            return None
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not rlist:
            return None
        ch = sys.stdin.read(1)
        return ch if ch else None


def nonblocking_readline_line_mode() -> str | None:
    """Fallback: non-blocking line input (requires Enter)."""
    rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not rlist:
        return None
    line = sys.stdin.readline()
    return line.strip() if line else None


# =========================
# Implementation
# =========================
def _make_run_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _prepare_dual_camera_run_with_id(
    *,
    base_dir: str,
    run_id: str,
    save_hz: float,
    width: int,
    height: int,
    fps: int,
    head_serial: str,
    wrist_serial: str,
):
    """Prepare camera folders but force a shared run_id for action/camera."""
    os.makedirs(base_dir, exist_ok=True)
    run_dir = os.path.join(base_dir, f"cam_dual_{run_id}")
    head_dir = os.path.join(run_dir, "head")
    wrist_dir = os.path.join(run_dir, "wrist")
    os.makedirs(head_dir, exist_ok=True)
    os.makedirs(wrist_dir, exist_ok=True)

    cfg_path = os.path.join(run_dir, "config.json")
    cfg = {
        "run_id": run_id,
        "run_start_time": datetime.now().isoformat(),
        "folder_name": f"cam_dual_{run_id}",
        "save_hz": save_hz,
        "color_resolution": [width, height],
        "color_fps": fps,
        "head_serial": head_serial,
        "wrist_serial": wrist_serial,
        "notes": "Dual RealSense color streams; images saved at SAVE_HZ into head/ and wrist/.",
    }
    with open(cfg_path, "w") as f:
        json.dump(cfg, f, indent=2)

    return run_dir, head_dir, wrist_dir, cfg_path


def main():
    # ===== camera presence check (optional) =====
    try:
        serials = list_serials()
        print("Detected RealSense serials:", serials)
        if HEAD_SERIAL not in serials:
            print(f"[ERR] HEAD_SERIAL not found: {HEAD_SERIAL}")
            return
        if WRIST_SERIAL not in serials:
            print(f"[ERR] WRIST_SERIAL not found: {WRIST_SERIAL}")
            return
    except Exception:
        # If list_serials isn't available / fails, skip.
        pass

    run_id = _make_run_id()
    print(f"[RUN_ID] {run_id}")

    # ===== prepare outputs =====
    os.makedirs(ACTION_DATA_DIR, exist_ok=True)
    os.makedirs(CAMERA_BASE_DIR, exist_ok=True)

    cam_run_dir, head_dir, wrist_dir, cam_cfg_path = _prepare_dual_camera_run_with_id(
        base_dir=CAMERA_BASE_DIR,
        run_id=run_id,
        save_hz=CAM_SAVE_HZ,
        width=COLOR_WIDTH,
        height=COLOR_HEIGHT,
        fps=COLOR_FPS,
        head_serial=HEAD_SERIAL,
        wrist_serial=WRIST_SERIAL,
    )
    
    # ===== prepare preview videos =====
    video_head_path = os.path.join(cam_run_dir, VIDEO_HEAD_NAME)
    video_wrist_path = os.path.join(cam_run_dir, VIDEO_WRIST_NAME)

    video_head_writer = None
    video_wrist_writer = None

    if SAVE_VIDEO:
        fourcc = cv2.VideoWriter_fourcc(*VIDEO_CODEC)
        video_head_writer = cv2.VideoWriter(
            video_head_path,
            fourcc,
            float(VIDEO_FPS),
            (COLOR_WIDTH, COLOR_HEIGHT),
        )
        video_wrist_writer = cv2.VideoWriter(
            video_wrist_path,
            fourcc,
            float(VIDEO_FPS),
            (COLOR_WIDTH, COLOR_HEIGHT),
        )

        if not video_head_writer.isOpened():
            raise RuntimeError(f"Failed to open video writer: {video_head_path}")
        if not video_wrist_writer.isOpened():
            raise RuntimeError(f"Failed to open video writer: {video_wrist_path}")

    # A small sync file to align action rows <-> saved frames
    sync_csv_path = os.path.join(ACTION_DATA_DIR, f"sync_action_cam_{run_id}.csv")
    sync_f = open(sync_csv_path, "w", newline="")
    sync_w = csv.writer(sync_f)
    sync_w.writerow(["controller_time_s", "frame_idx", "head_image", "wrist_image"])
    sync_f.flush()

    # ===== start camera pipelines =====
    pipe_head = start_realsense_pipeline(serial=HEAD_SERIAL, width=COLOR_WIDTH, height=COLOR_HEIGHT, fps=COLOR_FPS)
    pipe_wrist = start_realsense_pipeline(serial=WRIST_SERIAL, width=COLOR_WIDTH, height=COLOR_HEIGHT, fps=COLOR_FPS)

    # ===== start arm+gripper collector (RTDE + serial + optional freedrive) =====
    collector = ArmGripperCollector(
        data_dir=ACTION_DATA_DIR,
        run_id=run_id,
        robot_host=ROBOT_HOST,
        robot_port=ROBOT_PORT,
        frequency_hz=RTDE_HZ,
        serial_port=SERIAL_PORT,
        serial_baudrate=SERIAL_BAUDRATE,
        serial_timeout_s=SERIAL_TIMEOUT_S,
        enable_freedrive_on_start=ENABLE_FREEDRIVE_ON_START,
        freedrive_host=FREEDRIVE_HOST,
        freedrive_port=FREEDRIVE_PORT,
        freedrive_timeout_s=FREEDRIVE_TIMEOUT_S,
    )

    # ===== main loop =====
    frame_idx = 0
    next_cam_save = time.time()
    print("=== Collecting: RTDE TCP + Gripper + Dual Camera ===")
    print(f"[ACTION] {collector.paths.rtde_csv}")
    print(f"[GRIP ] {collector.paths.gripper_events_csv}")
    print(f"[SYNC ] {sync_csv_path}")
    print(f"[CAM  ] {cam_run_dir}")

    with TerminalKeyPoller() as key_poller:
        if key_poller.enabled:
            print("Keys: c(close), o(open), q(quit). No Enter needed. (Ctrl+C also works.)\n")
        else:
            print("[WARN] stdin is not a TTY (e.g., nohup/pipe). Falling back to line mode: c/o/q + Enter.\n")

        try:
            while True:
                out = collector.receive()
                if out is None:
                    print("[RTDE] connection closed by robot.")
                    break

                controller_ts, tcp = out

                # user input (terminal)
                if key_poller.enabled:
                    ch = key_poller.poll()
                    if ch:
                        ch = ch.lower()
                        if ch in ("c", "o", "q"):
                            ret = collector.handle_cmd(ch, controller_ts=controller_ts)
                            if ret == "quit":
                                break
                else:
                    cmd = nonblocking_readline_line_mode()
                    if cmd:
                        ret = collector.handle_cmd(cmd, controller_ts=controller_ts)
                        if ret == "quit":
                            break

                # camera frames
                frames_head = pipe_head.wait_for_frames()
                frames_wrist = pipe_wrist.wait_for_frames()
                color_head = frames_head.get_color_frame()
                color_wrist = frames_wrist.get_color_frame()
                if (not color_head) or (not color_wrist):
                    continue

                img_head = np.asanyarray(color_head.get_data())
                img_wrist = np.asanyarray(color_wrist.get_data())

                if SHOW_PREVIEW:
                    cv2.imshow("RealSense HEAD Color", img_head)
                    cv2.imshow("RealSense WRIST Color", img_wrist)
                    # Keep window responsive; do not use it for quit controls.
                    cv2.waitKey(1)

                # write RTDE row (paced by RTDE_HZ)
                collector.write_rtde_row(controller_ts, tcp)

                # save camera frames (paced by CAM_SAVE_HZ)
                now = time.time()
                if now >= next_cam_save:
                    frame_idx += 1
                    head_name = f"frame_{frame_idx:05d}.png"
                    wrist_name = f"frame_{frame_idx:05d}.png"
                    head_path = os.path.join(head_dir, head_name)
                    wrist_path = os.path.join(wrist_dir, wrist_name)

                    cv2.imwrite(head_path, img_head)
                    cv2.imwrite(wrist_path, img_wrist)

                    if SAVE_VIDEO:
                        video_head_writer.write(img_head)
                        video_wrist_writer.write(img_wrist)
                    
                    sync_w.writerow([controller_ts, frame_idx, head_name, wrist_name])
                    sync_f.flush()

                    print(f"[CAM ] [{frame_idx:05d}] saved: {head_path} | {wrist_path}")
                    next_cam_save += 1.0 / CAM_SAVE_HZ

        except KeyboardInterrupt:
            print("\n[EXIT] Ctrl+C")
        finally:
            # cleanup
            try:
                sync_f.close()
            except Exception:
                pass

            try:
                collector.close()
            except Exception:
                pass

            try:
                pipe_head.stop()
            except Exception:
                pass
            try:
                pipe_wrist.stop()
            except Exception:
                pass

            try:
                if video_head_writer is not None:
                    video_head_writer.release()
            except Exception:
                pass

            try:
                if video_wrist_writer is not None:
                    video_wrist_writer.release()
            except Exception:
                pass

            if SHOW_PREVIEW:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass

            print("\nSaved:")
            print(f"  - {collector.paths.rtde_csv}")
            print(f"  - {collector.paths.gripper_events_csv}")
            print(f"  - {collector.paths.config_json}")
            print(f"  - {sync_csv_path}")
            print(f"  - {cam_cfg_path}")
            if SAVE_VIDEO:
                print(f"  - {video_head_path}")
                print(f"  - {video_wrist_path}")
            print(f"  - {cam_run_dir}")


if __name__ == "__main__":
    main()
