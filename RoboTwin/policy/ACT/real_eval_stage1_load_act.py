#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Stage 1: 只做一件事 —— 加载 ACT 模型 + ckpt，然后用假数据前向一次，
确认模型结构和权重都是 OK 的。

使用方法：
    cd /home/zhangw/UR5e_DataCollection/RoboTwin/policy/ACT
    python real_eval_stage1_load_act.py
"""

import os
import sys
import numpy as np
import torch

# 关键：给 detr.main 里的 argparse 塞一份“假命令行参数”，避免报缺少必选项
if len(sys.argv) == 1:
    sys.argv += [
        "--ckpt_dir", "dummy_ckpt_dir",
        "--policy_class", "ACT",
        "--task_name", "dummy_task",
        "--seed", "0",
        "--num_epochs", "1",
        "--state_dim", "14",
    ]

from act_policy import ACTPolicy
from constants import SIM_TASK_CONFIGS



# ================== 需要你根据实际情况改的三个参数 ==================

# 训练时的三个参数（和 train.sh 保持一致）
TASK_NAME = "pick_block_bowl"   # 比如 torch_cube
TASK_CONFIG = "simple"     # 比如 simple
EXPERT_DATA_NUM = 15        # 比如 3，对应你采集了 3 条演示

# 使用哪个 ckpt 文件
CKPT_NAME = "policy_epoch_2000_seed_0.ckpt"  # 
# CKPT_NAME = "policy_best.ckpt"  # 如果你只有 policy_last.ckpt，就改成那个


# ====================================================================


def load_act_from_ckpt():
    """
    仿照 imitate_episodes.py 里的写法，构造 policy_config，然后
    实例化 ACTPolicy 并加载 ckpt。
    """
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] Using device: {device}")

    # 1) 从 SIM_TASK_CONFIGS 里拿 camera_names（和训练保持一致）
    task_key = f"sim-{TASK_NAME}-{TASK_CONFIG}-{EXPERT_DATA_NUM}"
    camera_names = SIM_TASK_CONFIGS[task_key]["camera_names"]
    print(f"[INFO] Task key = {task_key}")
    print(f"[INFO] Camera names = {camera_names}")

    # 2) 这些超参和 imitate_episodes.py + train.sh 保持一致
    state_dim = 14          # yiheng，你在训练脚本里就是这么写的
    lr = 1e-5               # train.sh 里 --lr 1e-5
    kl_weight = 10          # train.sh 里 --kl_weight 10
    chunk_size = 50         # train.sh 里 --chunk_size 50
    hidden_dim = 512        # train.sh 里 --hidden_dim 512
    dim_feedforward = 3200  # train.sh 里 --dim_feedforward 3200
    lr_backbone = 1e-5
    backbone = "resnet18"
    enc_layers = 4
    dec_layers = 7
    nheads = 8

    policy_config = {
        "lr": lr,
        "num_queries": chunk_size,
        "chunk_size": chunk_size,      # ★ 新增这一行
        "kl_weight": kl_weight,
        "hidden_dim": hidden_dim,
        "dim_feedforward": dim_feedforward,
        "lr_backbone": lr_backbone,
        "backbone": backbone,
        "enc_layers": enc_layers,
        "dec_layers": dec_layers,
        "nheads": nheads,
        "camera_names": camera_names,
    }

    # 3) 实例化 ACTPolicy
    policy = ACTPolicy(policy_config)
    policy.to(device)

    # 4) 加载 ckpt（相对于当前脚本所在目录）
    base_dir = os.path.dirname(os.path.abspath(__file__))  # /home/zhangw/UR5e_DataCollection/RoboTwin/policy/ACT
    ckpt_dir = os.path.join(
        base_dir,
        "act_ckpt",
        f"act-{TASK_NAME}",
        f"{TASK_CONFIG}-{EXPERT_DATA_NUM}",
    )
    ckpt_path = os.path.join(ckpt_dir, CKPT_NAME)
    print(f"[INFO] Loading ckpt from: {ckpt_path}")

    state_dict = torch.load(ckpt_path, map_location=device)
    loading_status = policy.load_state_dict(state_dict)
    print(f"[INFO] load_state_dict status: {loading_status}")

    policy.eval()
    print("[INFO] ACTPolicy is ready (eval mode).")

    return policy, camera_names, device


def run_dummy_forward():
    """
    用全 0 的 qpos + 全 0 的假图像前向一次，只检查：
    - 代码不报错
    - 输出形状是 (1, 14)
    """
    policy, camera_names, device = load_act_from_ckpt()

    # 假设图像大小 480x640（你真实的 HDF5 里就是这个尺寸）
    H, W = 480, 640
    B = 1
    N_cam = len(camera_names)

    # 构造假输入
    qpos = torch.zeros(B, 14, dtype=torch.float32, device=device)
    # 注意：这里 image 还没有 /255，也没有做 normalize，
    # 只是测试网络结构是否能跑通
    dummy_image = torch.zeros(B, N_cam, 3, H, W, dtype=torch.float32, device=device)

    with torch.no_grad():
        a_hat = policy(qpos, dummy_image)  # (1, num_queries, state_dim) 之类

    print(f"[RESULT] a_hat type: {type(a_hat)}")
    if isinstance(a_hat, torch.Tensor):
        print(f"[RESULT] a_hat.shape = {a_hat.shape}")
    else:
        try:
            print(f"[RESULT] a_hat[0].shape = {a_hat[0].shape}")
        except Exception:
            pass

    print("[DONE] Stage 1 dummy forward finished.")


if __name__ == "__main__":
    run_dummy_forward()
