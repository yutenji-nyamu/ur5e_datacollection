#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
go_home_rtde_local.py

Local-mode "go home" using RTDE + a running URP control loop (servoj).

This is aligned with your working servoj RTDE demo (servoj_rtde_min_urp.py):
- Assumes you keep this script under: ~/UR5e_DataCollection/
- Assumes the servoj resources live under: ~/UR5e_DataCollection/Servoj_RTDE_UR5/
  and include control_loop_configuration.xml

Prerequisite:
1) Robot is in Polyscope Local mode
2) The URP servoj control loop is RUNNING on the robot (e.g., translation_sample_servoj.urp)
3) This script ONLY streams target pose via RTDE; it does NOT start the URP.
"""

# =======================
# CONFIG (only edit here)
# =======================
from pathlib import Path
import time
from math import sqrt
from typing import Sequence

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

ROBOT_HOST = "192.168.0.3"
ROBOT_PORT = 30004
RTDE_HZ = 500

# Keep this script under ~/UR5e_DataCollection/ and you don't need to edit ROOT_DIR.
# If you move this script elsewhere, set ROOT_DIR explicitly to your UR5e_DataCollection root.
ROOT_DIR = Path(__file__).resolve().parent
SERVOJ_DIR = ROOT_DIR / "Servoj_RTDE_UR5"
RTDE_CONFIG_XML = SERVOJ_DIR / "control_loop_configuration.xml"

# "home" TCP pose: [x, y, z, rx, ry, rz] (meters, radians)
HOME_TCP_POSE = [-0.4479512414011685, -0.47795323591463873, 0.3397913564298358,
                 1.135105428797068, 2.8938840645319757, -0.02678754322562883]

# Stop conditions
POS_TOL_M = 0.005      # 5 mm
ROT_TOL_RAD = 0.05     # ~3 deg
STABLE_CYCLES = 200    # within tol for this many RTDE cycles (~0.4s at 500Hz)
TIMEOUT_S = 12.0

# Mode convention used by your URP
SERVOJ_MODE = 2
IDLE_MODE = 0


def _set_pose_to_setp(setp, pose6: Sequence[float]) -> None:
    for i in range(6):
        setattr(setp, f"input_double_register_{i}", float(pose6[i]))
    if hasattr(setp, "input_bit_registers0_to_31"):
        setp.input_bit_registers0_to_31 = 0


def main() -> None:
    if HOME_TCP_POSE is None or len(HOME_TCP_POSE) != 6:
        raise ValueError("HOME_TCP_POSE must be a length-6 list in CONFIG.")

    if not RTDE_CONFIG_XML.exists():
        raise FileNotFoundError(
            f"RTDE XML not found: {RTDE_CONFIG_XML}\n"
            f"Expected layout (same as servoj_rtde_min_urp.py):\n"
            f"  {ROOT_DIR}/Servoj_RTDE_UR5/control_loop_configuration.xml\n"
            f"If you moved this script, set ROOT_DIR in CONFIG to your UR5e_DataCollection root."
        )

    conf = rtde_config.ConfigFile(str(RTDE_CONFIG_XML))
    state_names, state_types = conf.get_recipe("state")
    setp_names, setp_types = conf.get_recipe("setp")
    watchdog_names, watchdog_types = conf.get_recipe("watchdog")

    con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)

    connection_state = con.connect()
    while connection_state != 0:
        time.sleep(0.5)
        connection_state = con.connect()

    con.get_controller_version()
    con.send_output_setup(state_names, state_types, RTDE_HZ)

    setp = con.send_input_setup(setp_names, setp_types)
    watchdog = con.send_input_setup(watchdog_names, watchdog_types)

    # init registers to avoid uninitialized warnings
    _set_pose_to_setp(setp, HOME_TCP_POSE)
    watchdog.input_int_register_0 = int(IDLE_MODE)

    if not con.send_start():
        raise RuntimeError("RTDE send_start() failed")

    # switch to servoj mode
    watchdog.input_int_register_0 = int(SERVOJ_MODE)
    con.send(watchdog)

    t0 = time.time()
    stable = 0

    while True:
        state = con.receive()
        if state is None:
            continue

        # Only stream setpoint while UR program is running
        if int(state.runtime_state) > 1:
            _set_pose_to_setp(setp, HOME_TCP_POSE)
            con.send(setp)

        tcp = list(state.actual_TCP_pose)
        dp = sqrt((tcp[0] - HOME_TCP_POSE[0]) ** 2 + (tcp[1] - HOME_TCP_POSE[1]) ** 2 + (tcp[2] - HOME_TCP_POSE[2]) ** 2)
        dr = sqrt((tcp[3] - HOME_TCP_POSE[3]) ** 2 + (tcp[4] - HOME_TCP_POSE[4]) ** 2 + (tcp[5] - HOME_TCP_POSE[5]) ** 2)

        if dp < POS_TOL_M and dr < ROT_TOL_RAD:
            stable += 1
        else:
            stable = 0

        if stable >= STABLE_CYCLES:
            break
        if (time.time() - t0) > TIMEOUT_S:
            break

    # idle (stop motion but keep URP alive)
    watchdog.input_int_register_0 = int(IDLE_MODE)
    try:
        con.send(watchdog)
    except Exception:
        pass

    try:
        con.send_pause()
    except Exception:
        pass
    con.disconnect()


if __name__ == "__main__":
    main()
