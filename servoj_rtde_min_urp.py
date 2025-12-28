#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =====================
# CONFIG（只改这里）
# =====================
from pathlib import Path
import sys, time, logging

ROBOT_HOST = "192.168.0.3"
ROBOT_PORT = 30004
FREQUENCY_HZ = 500
TRAJ_TIME_S = 3.0

# 你把脚本放在 ~/UR5e_DataCollection/ 根目录时，这样写就对
ROOT_DIR = Path(__file__).resolve().parent
SERVOJ_DIR = ROOT_DIR / "Servoj_RTDE_UR5"   # 里面有 min_jerk_planner_translation.py 和 control_loop_configuration.xml

POSE_A = [-0.503, -0.2088, 0.31397, 1.266, -2.572, -0.049]
POSE_B = [-0.403, -0.2088, 0.31397, 1.266, -2.572, -0.049]
POSE_C = [-0.403, -0.1088, 0.31397, 1.266, -2.572, -0.049]
POSE_D = [-0.503, -0.1088, 0.31397, 1.266, -2.572, -0.049]
# =====================

# 关键：让 import 能找到 Servoj_RTDE_UR5 里的模块（也让 rtde 优先用仓库自带版本）
sys.path.insert(0, str(SERVOJ_DIR))

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from min_jerk_planner_translation import PathPlanTranslation


def list_to_setp(setp, pose6):
    for i in range(6):
        setattr(setp, f"input_double_register_{i}", pose6[i])
    return setp


def main():
    logging.getLogger().setLevel(logging.INFO)

    config_filename = str(SERVOJ_DIR / "control_loop_configuration.xml")

    conf = rtde_config.ConfigFile(config_filename)
    state_names, state_types = conf.get_recipe("state")
    setp_names, setp_types = conf.get_recipe("setp")
    watchdog_names, watchdog_types = conf.get_recipe("watchdog")

    con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
    connection_state = con.connect()
    while connection_state != 0:
        time.sleep(0.5)
        connection_state = con.connect()
    print("---------------Successfully connected to the robot-------------\n")

    con.get_controller_version()

    con.send_output_setup(state_names, state_types, FREQUENCY_HZ)
    setp = con.send_input_setup(setp_names, setp_types)
    watchdog = con.send_input_setup(watchdog_names, watchdog_types)

    # 初始化 input（避免 Uninitialized）
    list_to_setp(setp, [0, 0, 0, 0, 0, 0])
    setp.input_bit_registers0_to_31 = 0
    watchdog.input_int_register_0 = 0

    if not con.send_start():
        return

    state = con.receive()
    tcp = state.actual_TCP_pose
    print("Current TCP pose:", tcp)

    # 保持姿态为“启动时实际姿态”（你原脚本就是这样）
    orientation_const = tcp[3:]

    # 切到 mode=2（servoj）
    watchdog.input_int_register_0 = 2
    con.send(watchdog)

    # planner：平移用 min-jerk，姿态固定
    planners = [
        PathPlanTranslation(tcp,   POSE_A, TRAJ_TIME_S),
        PathPlanTranslation(POSE_A, POSE_B, TRAJ_TIME_S),
        PathPlanTranslation(POSE_B, POSE_C, TRAJ_TIME_S),
        PathPlanTranslation(POSE_C, POSE_D, TRAJ_TIME_S),
    ]

    for idx, planner in enumerate(planners, start=1):
        print(f"-------Executing servoJ to point {idx} -----------\n")

        t_start = time.time()
        while True:
            state = con.receive()
            t = time.time() - t_start
            if t >= TRAJ_TIME_S:
                break

            if state.runtime_state > 1:
                pos_ref, _, _ = planner.trajectory_planning(t)
                pose_cmd = pos_ref.tolist() + orientation_const
                list_to_setp(setp, pose_cmd)
                con.send(setp)

        print(f"It took {time.time() - t_start:.3f}s to execute point {idx}")
        print("Final TCP pose:", con.receive().actual_TCP_pose)

    con.send_pause()
    con.disconnect()


if __name__ == "__main__":
    main()
