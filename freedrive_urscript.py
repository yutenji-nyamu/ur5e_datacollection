#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Freedrive control via URScript over TCP (Primary/Secondary interface).

Design goal: minimal, robust enough for data-collection teleoperation.

Key idea:
- To keep freedrive active on many e-Series setups, you usually need a running program that
  stays alive after calling freedrive_mode().
- So start_freedrive() sends a URScript program that:
    freedrive_mode()
    while True: sync()
  This program keeps running until you send another URScript program, which will interrupt it.
- stop_freedrive() sends a small URScript program that calls end_freedrive_mode().

Notes / common failure causes:
- Robot must be in Remote Control mode; in Local mode ports 30001/30002/30003 may be blocked/dropped.
- Your own demo used the same socket approach (timed freedrive). This is a slightly modified version
  that keeps freedrive on until you stop it.  (See freedrive_socket.py)
"""

# =========================
# CONFIG (defaults)
# =========================
HOST = "192.168.0.3"
PORT = 30001
TIMEOUT_S = 10.0
# =========================

import socket


def send_urscript(program: str, host: str = HOST, port: int = PORT, timeout_s: float = TIMEOUT_S):
    if not program.endswith("\n"):
        program += "\n"
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout_s)
    s.connect((host, port))
    s.sendall(program.encode("utf-8"))
    s.close()


def start_freedrive(host: str = HOST, port: int = PORT, timeout_s: float = TIMEOUT_S):
    """
    Enter freedrive and keep it active by running an infinite loop.
    To exit, call stop_freedrive() (which will typically interrupt this program).
    """
    program = r"""def _fd_hold():
  freedrive_mode()
  while (True):
    sync()
  end
end
_fd_hold()
"""
    send_urscript(program, host=host, port=port, timeout_s=timeout_s)


def stop_freedrive(host: str = HOST, port: int = PORT, timeout_s: float = TIMEOUT_S):
    """
    Exit freedrive. This is sent as a new URScript program.
    """
    program = r"""def _fd_stop():
  end_freedrive_mode()
end
_fd_stop()
"""
    send_urscript(program, host=host, port=port, timeout_s=timeout_s)


def _demo():
    print("[INFO] start freedrive (will keep running until you send stop)")
    start_freedrive()
    print("[INFO] sent start_freedrive(). Now send stop from another terminal or Ctrl+C and run stop_freedrive().")


if __name__ == "__main__":
    _demo()
