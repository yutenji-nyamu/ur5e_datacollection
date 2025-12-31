#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert real-robot raw data to RoboTwin-style episode*.hdf5 (dual camera + gripper).

Inputs (new pipeline):
- action_data/rtde_tcp_gripper_<RUN_ID>.csv
- action_data/sync_action_cam_<RUN_ID>.csv
- camera_data/cam_dual_<RUN_ID>/head/frame_*.png
- camera_data/cam_dual_<RUN_ID>/wrist/frame_*.png

Backwards-compatible (older pipeline):
- action_data/rtde_tcp_<RUN_ID>.csv
- camera_data/cam_<RUN_ID>/frame_*.png   (single cam)

Output:
- RoboTwin_like_data/run_<RUN_TAG>/<TASK_NAME>/<TASK_CONFIG>/data/episode<i>.hdf5
"""

import os
import csv
import numpy as np
import cv2
import h5py
from datetime import datetime


# =========================
# CONFIG (edit here)
# =========================
ACTION_DIR = "/home/zhangw/UR5e_DataCollection/action_data"
CAMERA_DIR = "/home/zhangw/UR5e_DataCollection/camera_data"

OUT_ROOT = "/home/zhangw/UR5e_DataCollection/RoboTwin_like_data"

# TASK_NAME = "torch_cube"
TASK_NAME = "pick_block_bowl"

TASK_CONFIG = "simple"

# camera run dirs
CAM_DUAL_PREFIX = "cam_dual_"
CAM_SINGLE_PREFIX = "cam_"
HEAD_SUBDIR = "head"
WRIST_SUBDIR = "wrist"

# action csv prefixes
RTDE_TCP_GRIPPER_PREFIX = "rtde_tcp_gripper_"
RTDE_TCP_PREFIX = "rtde_tcp_"
SYNC_PREFIX = "sync_action_cam_"

# If True and sync exists: align frames by timestamp -> nearest action row
USE_SYNC_ALIGN = True

# RoboTwin observation camera group mapping (keep downstream stable)
# - head_camera uses head image
# - right_camera uses wrist image
# - left_camera duplicates wrist image (placeholder)
CAMERA_GROUP_MAP = {
    "head_camera": "head",
    "right_camera": "wrist",
    "left_camera": "wrist",
}

# debug artifacts
WRITE_DEBUG = True


# =========================
# helpers
# =========================
def _safe_mkdir(p: str):
    os.makedirs(p, exist_ok=True)


def find_episode_triples():
    """
    Returns a list of episode records:
      (run_id, action_csv_path, cam_mode, cam_dir, sync_csv_or_None)
    cam_mode in {"dual", "single"}
    """
    recs = []

    for name in os.listdir(ACTION_DIR):
        if not name.endswith(".csv"):
            continue

        run_id = None
        action_csv = None

        if name.startswith(RTDE_TCP_GRIPPER_PREFIX):
            run_id = name[len(RTDE_TCP_GRIPPER_PREFIX):-4]
            action_csv = os.path.join(ACTION_DIR, name)
        elif name.startswith(RTDE_TCP_PREFIX) and not name.startswith(RTDE_TCP_GRIPPER_PREFIX):
            # legacy: rtde_tcp_<run_id>.csv
            run_id = name[len(RTDE_TCP_PREFIX):-4]
            action_csv = os.path.join(ACTION_DIR, name)
        else:
            continue

        cam_dual = os.path.join(CAMERA_DIR, f"{CAM_DUAL_PREFIX}{run_id}")
        cam_single = os.path.join(CAMERA_DIR, f"{CAM_SINGLE_PREFIX}{run_id}")

        if os.path.isdir(cam_dual):
            cam_mode = "dual"
            cam_dir = cam_dual
        elif os.path.isdir(cam_single):
            cam_mode = "single"
            cam_dir = cam_single
        else:
            # no matching camera dir
            continue

        sync_csv = os.path.join(ACTION_DIR, f"{SYNC_PREFIX}{run_id}.csv")
        sync_csv = sync_csv if os.path.isfile(sync_csv) else None

        recs.append((run_id, action_csv, cam_mode, cam_dir, sync_csv))

    recs.sort(key=lambda x: x[0])
    return recs


def load_action_csv(action_csv_path: str):
    """
    Load action CSV.
    Returns:
      times: (N,)
      tcp:   (N,6)
      grip:  (N,) float32 (0/1), or zeros if not present.
    """
    with open(action_csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)

    # normalize header
    header_l = [h.strip() for h in header]

    data = np.loadtxt(action_csv_path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data[None, :]

    # common fields
    # controller_time_s, tcp_x..tcp_rz
    times = data[:, 0].astype(np.float64)
    tcp = data[:, 1:7].astype(np.float32)

    # optional gripper_state
    grip = np.zeros((len(times),), dtype=np.float32)
    if "gripper_state" in header_l:
        g_idx = header_l.index("gripper_state")
        grip = data[:, g_idx].astype(np.float32)

    return times, tcp, grip


def load_sync_csv(sync_csv_path: str):
    """
    Load sync CSV:
      controller_time_s, frame_idx, head_image, wrist_image
    Returns:
      sync_times (M,), head_names (M,), wrist_names (M,)
    """
    rows = []
    with open(sync_csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)

    sync_times = np.array([float(r["controller_time_s"]) for r in rows], dtype=np.float64)
    head_names = [r["head_image"] for r in rows]
    wrist_names = [r["wrist_image"] for r in rows]
    return sync_times, head_names, wrist_names


def align_nearest(action_times: np.ndarray, query_times: np.ndarray):
    """
    For each query time, pick the nearest index in action_times.
    action_times must be sorted (it is, because it is logged sequentially).
    """
    n = len(action_times)
    idx1 = np.searchsorted(action_times, query_times, side="left")
    idx1 = np.clip(idx1, 0, n - 1)
    idx0 = np.clip(idx1 - 1, 0, n - 1)

    t0 = action_times[idx0]
    t1 = action_times[idx1]
    choose0 = np.abs(query_times - t0) <= np.abs(query_times - t1)
    idx = np.where(choose0, idx0, idx1)
    return idx.astype(np.int64)


def encode_jpg(img_bgr: np.ndarray):
    ok, buf = cv2.imencode(".jpg", img_bgr)
    if not ok:
        return None
    return np.frombuffer(buf.tobytes(), dtype=np.uint8)


def save_episode_hdf5(
    out_path: str,
    right_arm: np.ndarray,
    right_gripper: np.ndarray,
    imgs_head_paths: list,
    imgs_wrist_paths: list,
    debug_dir: str | None = None,
):
    """
    Writes a single episode.
    right_arm: (T,6)
    right_gripper: (T,)
    imgs_*_paths: length T, absolute paths
    """
    T = len(imgs_head_paths)
    if T == 0:
        print("  -> skip: T=0")
        return

    left_arm = right_arm  # placeholder
    left_gripper = np.zeros((T,), dtype=np.float32)

    _safe_mkdir(os.path.dirname(out_path))

    if debug_dir:
        _safe_mkdir(debug_dir)
        np.save(os.path.join(debug_dir, "right_arm.npy"), right_arm)
        np.save(os.path.join(debug_dir, "right_gripper.npy"), right_gripper)
        with open(os.path.join(debug_dir, "info.txt"), "w") as f:
            f.write(f"T = {T}\n")
            f.write(f"out_path = {out_path}\n")
            if T > 0:
                f.write(f"head0 = {imgs_head_paths[0]}\n")
                f.write(f"wrist0 = {imgs_wrist_paths[0]}\n")

    with h5py.File(out_path, "w") as f:
        # joint_action
        ja = f.create_group("joint_action")
        ja.create_dataset("left_arm", data=left_arm)
        ja.create_dataset("left_gripper", data=left_gripper)
        ja.create_dataset("right_arm", data=right_arm)
        ja.create_dataset("right_gripper", data=right_gripper)

        # observation / cameras (vlen uint8, jpg bitstream)
        vlen_uint8 = h5py.vlen_dtype(np.dtype("uint8"))
        obs = f.create_group("observation")

        cam_dsets = {}
        for cam_name in CAMERA_GROUP_MAP.keys():
            g = obs.create_group(cam_name)
            dset = g.create_dataset("rgb", (T,), dtype=vlen_uint8)
            cam_dsets[cam_name] = dset

        # stream-write images
        for t in range(T):
            img_head = cv2.imread(imgs_head_paths[t])
            img_wrist = cv2.imread(imgs_wrist_paths[t])
            if img_head is None or img_wrist is None:
                # minimal robustness: skip this timestep by writing empty bytes
                for cam_name in cam_dsets.keys():
                    cam_dsets[cam_name][t] = np.array([], dtype=np.uint8)
                continue

            for cam_name, src in CAMERA_GROUP_MAP.items():
                img = img_head if src == "head" else img_wrist
                arr = encode_jpg(img)
                if arr is None:
                    arr = np.array([], dtype=np.uint8)
                cam_dsets[cam_name][t] = arr


def main():
    run_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_root = os.path.join(OUT_ROOT, f"run_{run_tag}")
    out_data_dir = os.path.join(run_root, TASK_NAME, TASK_CONFIG, "data")
    debug_root = os.path.join(run_root, "_debug")

    recs = find_episode_triples()
    print(f"Found {len(recs)} episode records.")
    print(f"[OUT] {run_root}")

    for epi_idx, (run_id, action_csv, cam_mode, cam_dir, sync_csv) in enumerate(recs):
        print(f"\n[{epi_idx}] run_id = {run_id}")
        print(f"    action_csv = {action_csv}")
        print(f"    cam_mode   = {cam_mode}")
        print(f"    cam_dir    = {cam_dir}")
        print(f"    sync_csv   = {sync_csv}")

        action_times, tcp, grip = load_action_csv(action_csv)
        if len(action_times) == 0:
            print("  -> skip: empty action csv")
            continue

        # build frame lists
        if cam_mode == "dual":
            head_dir = os.path.join(cam_dir, HEAD_SUBDIR)
            wrist_dir = os.path.join(cam_dir, WRIST_SUBDIR)

            if sync_csv and USE_SYNC_ALIGN:
                sync_times, head_names, wrist_names = load_sync_csv(sync_csv)

                head_paths = [os.path.join(head_dir, n) for n in head_names]
                wrist_paths = [os.path.join(wrist_dir, n) for n in wrist_names]

                # keep only entries whose files exist
                valid = [i for i in range(len(sync_times)) if os.path.isfile(head_paths[i]) and os.path.isfile(wrist_paths[i])]
                sync_times = sync_times[valid]
                head_paths = [head_paths[i] for i in valid]
                wrist_paths = [wrist_paths[i] for i in valid]

                idx = align_nearest(action_times, sync_times)
                right_arm = tcp[idx].astype(np.float32)
                right_gripper = grip[idx].astype(np.float32)

                if WRITE_DEBUG:
                    epi_debug = os.path.join(debug_root, f"episode{epi_idx}")
                    _safe_mkdir(epi_debug)
                    np.save(os.path.join(epi_debug, "action_times.npy"), action_times)
                    np.save(os.path.join(epi_debug, "sync_times.npy"), sync_times)
                    np.save(os.path.join(epi_debug, "aligned_action_idx.npy"), idx)

            else:
                # fallback: scan frame_*.png and min-length align
                head_files = sorted([f for f in os.listdir(head_dir) if f.startswith("frame_") and f.endswith(".png")])
                wrist_files = sorted([f for f in os.listdir(wrist_dir) if f.startswith("frame_") and f.endswith(".png")])
                T = min(len(tcp), len(head_files), len(wrist_files))
                head_paths = [os.path.join(head_dir, f) for f in head_files[:T]]
                wrist_paths = [os.path.join(wrist_dir, f) for f in wrist_files[:T]]
                right_arm = tcp[:T].astype(np.float32)
                right_gripper = grip[:T].astype(np.float32)

        else:
            # single camera legacy: use same frames for head+wrist
            files = sorted([f for f in os.listdir(cam_dir) if f.startswith("frame_") and f.endswith(".png")])
            T = min(len(tcp), len(files))
            paths = [os.path.join(cam_dir, f) for f in files[:T]]
            head_paths = paths
            wrist_paths = paths
            right_arm = tcp[:T].astype(np.float32)
            right_gripper = grip[:T].astype(np.float32)

        out_path = os.path.join(out_data_dir, f"episode{epi_idx}.hdf5")
        epi_debug = os.path.join(debug_root, f"episode{epi_idx}") if WRITE_DEBUG else None
        save_episode_hdf5(out_path, right_arm, right_gripper, head_paths, wrist_paths, debug_dir=epi_debug)

        print(f"  -> saved: {out_path}")

    print("\nAll done.")


if __name__ == "__main__":
    main()
