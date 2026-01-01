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

HOME_TCP_POSE = [
    -0.40,
    -0.50,
    0.2597913564298358,
    1.135105428797068,
    2.8938840645319757,
    -0.02678754322562883,
]

# HOME_TCP_POSE = [
#     -0.4479512414011685,
#     -0.47795323591463873,
#     0.3397913564298358,
#     1.135105428797068,
#     2.8938840645319757,
#     -0.02678754322562883,
# ]

# 是否把姿态也平滑插值到 HOME（推荐 True，避免姿态瞬间跳变）
INTERP_ORIENTATION = True

# =====================
# 关键：让 import 能找到 Servoj_RTDE_UR5 里的模块（也让 rtde 优先用仓库自带版本）
# =====================
sys.path.insert(0, str(SERVOJ_DIR))

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from min_jerk_planner_translation import PathPlanTranslation


def list_to_setp(setp, pose6):
    for i in range(6):
        setattr(setp, f"input_double_register_{i}", float(pose6[i]))
    # optional bit registers in the recipe
    if hasattr(setp, "input_bit_registers0_to_31"):
        setp.input_bit_registers0_to_31 = 0
    return setp


def main():
    logging.getLogger().setLevel(logging.INFO)

    config_filename = SERVOJ_DIR / "control_loop_configuration.xml"
    if not config_filename.exists():
        raise FileNotFoundError(f"Cannot find RTDE config xml: {config_filename}")

    conf = rtde_config.ConfigFile(str(config_filename))
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
    watchdog.input_int_register_0 = 0

    if not con.send_start():
        raise RuntimeError("RTDE send_start() failed")

    # 读取当前 TCP
    state = con.receive()
    tcp0 = list(state.actual_TCP_pose)
    print("Current TCP pose:", tcp0)
    print("Home TCP pose   :", HOME_TCP_POSE)

    # planner：平移用 min-jerk
    planner = PathPlanTranslation(tcp0, HOME_TCP_POSE, TRAJ_TIME_S)

    # 姿态：要么固定当前姿态，要么线性插值到 HOME 姿态（rotvec 空间）
    ori0 = tcp0[3:]
    ori1 = HOME_TCP_POSE[3:]

    # 切到 mode=2（servoj）
    watchdog.input_int_register_0 = 2
    con.send(watchdog)

    print("-------Executing servoJ go_home -----------\n")

    t_start = time.time()
    while True:
        state = con.receive()
        if state is None:
            continue

        t = time.time() - t_start
        if t >= TRAJ_TIME_S:
            break

        if int(state.runtime_state) > 1:
            pos_ref, _, _ = planner.trajectory_planning(t)  # pos_ref: (3,)
            if INTERP_ORIENTATION:
                alpha = max(0.0, min(1.0, t / float(TRAJ_TIME_S)))
                ori = [(1 - alpha) * ori0[i] + alpha * ori1[i] for i in range(3)]
            else:
                # 最保守：不动姿态，保持启动时姿态
                ori = ori0

            pose_cmd = pos_ref.tolist() + ori
            list_to_setp(setp, pose_cmd)
            con.send(setp)

    # 末尾再发几次 HOME，确保落稳（很小的保险，不改变总体结构）
    settle_start = time.time()
    while time.time() - settle_start < 0.3:
        state = con.receive()
        if state is None:
            continue
        if int(state.runtime_state) > 1:
            list_to_setp(setp, HOME_TCP_POSE)
            con.send(setp)

    final_tcp = con.receive().actual_TCP_pose
    print("Final TCP pose:", final_tcp)

    con.send_pause()
    con.disconnect()


if __name__ == "__main__":
    main()
