#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UR5e freedrive minimal demo (timed)
- Send URScript via socket (same style as your movel demo)
- Enter freedrive for SECONDS, then exit automatically
"""

import socket
import time

# =========================
# CONFIG (edit here only)
# =========================
# HOST = "192.168.0.3"
HOST = "192.168.0.4"
PORT = 30001          # keep same as your working script
TIMEOUT_S = 10

SECONDS = 30          # freedrive duration
# =========================


def send_urscript(cmd: str):
    if not cmd.endswith("\n"):
        cmd += "\n"
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(TIMEOUT_S)
    s.connect((HOST, PORT))
    s.sendall(cmd.encode("utf-8"))
    s.close()


def main():
    # 最小 URScript：进入 -> sleep 持续运行 -> 退出
    program = f"""def _fd():
  freedrive_mode()
  sleep({int(SECONDS)})
  end_freedrive_mode()
end
_fd()
"""
    try:
        print(f"[INFO] connecting {HOST}:{PORT} ...")
        send_urscript(program)
        print(f"[OK] freedrive requested for {SECONDS}s. Hand-guide now.")
    except Exception as e:
        print("[FAIL] socket/urscript send failed:", repr(e))
        print("Most common cause on e-Series: robot is in LOCAL mode (30001/2/3 will drop).")
        raise


if __name__ == "__main__":
    main()
