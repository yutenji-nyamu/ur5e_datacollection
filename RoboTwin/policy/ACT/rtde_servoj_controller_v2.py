#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""rtde_servoj_controller_v2.py

A minimal RTDE servoJ "setpoint streamer" for UR e-Series.

This is a drop-in replacement for rtde_servoj_controller.py, but it fixes
one practical failure mode you just hit:

- Your known-good servoj_rtde_min_urp.py prepends
  UR5e_DataCollection/Servoj_RTDE_UR5 to sys.path before importing rtde.
  If you don't do that, Python may import a different 'rtde' package from
  the environment, where RTDE.connect() can have different return semantics.
  Your controller's loop `while connection_state != 0:` can then loop forever
  even though the socket is already connected.

Fixes in v2:
- Prefer the bundled rtde client by inserting Servoj_RTDE_UR5 into sys.path
  before importing rtde.
- Make connect robust: attempt connect(), then confirm by get_controller_version().
  This avoids relying on connect() return codes.

Robot-side prerequisite (unchanged):
- Polyscope Local mode
- URP program running that reads:
    input_int_register_0 == 2  -> servoj mode
    input_double_register_0..5 -> target TCP pose [x,y,z,rx,ry,rz]
"""

from __future__ import annotations

import sys
import time
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

# ----------------------------------------------------------------------
# Path bootstrap (match your working demo): prefer Servoj_RTDE_UR5 rtde
# ----------------------------------------------------------------------
_THIS_DIR = Path(__file__).resolve().parent
try:
    # .../UR5e_DataCollection/RoboTwin/policy/ACT -> parents[2] == .../UR5e_DataCollection
    _UR5E_ROOT = _THIS_DIR.parents[2]
    _SERVOJ_DIR = _UR5E_ROOT / "Servoj_RTDE_UR5"
    if _SERVOJ_DIR.exists():
        sys.path.insert(0, str(_SERVOJ_DIR))
except Exception:
    # keep minimal: do not fail import because of path math
    pass

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


@dataclass
class RTDEServoJConfig:
    robot_host: str = "192.168.0.3"
    robot_port: int = 30004
    frequency_hz: int = 500
    config_xml_path: str = "control_loop_configuration.xml"

    # mode=2 -> servoj in your URP convention
    servoj_mode: int = 2

    connect_retry_s: float = 0.5
    connect_timeout_s: float = 15.0


class RTDEServoJController:
    """Threaded RTDE setpoint streamer."""

    def __init__(self, cfg: RTDEServoJConfig):
        self.cfg = cfg

        self._con: Optional[rtde.RTDE] = None
        self._setp = None
        self._watchdog = None

        self._latest_tcp = None
        self._latest_runtime_state = None

        self._target_tcp = None
        self._lock = threading.Lock()

        self._running = False
        self._th: Optional[threading.Thread] = None

    def connect_and_start(self) -> None:
        cfg = self.cfg

        conf = rtde_config.ConfigFile(str(cfg.config_xml_path))
        state_names, state_types = conf.get_recipe("state")
        setp_names, setp_types = conf.get_recipe("setp")
        watchdog_names, watchdog_types = conf.get_recipe("watchdog")

        con = rtde.RTDE(cfg.robot_host, cfg.robot_port)

        # Robust connect: don't depend on return code; confirm by get_controller_version().
        t0 = time.time()
        last_err: Optional[Exception] = None
        while True:
            try:
                con.connect()
                con.get_controller_version()
                break
            except Exception as e:
                last_err = e
                if (time.time() - t0) >= float(cfg.connect_timeout_s):
                    raise RuntimeError(
                        f"RTDE connect timeout after {cfg.connect_timeout_s}s to {cfg.robot_host}:{cfg.robot_port}. "
                        f"(Common causes: robot not reachable, another RTDE client connected, wrong mode/port.)"
                    ) from e
                time.sleep(float(cfg.connect_retry_s))

        con.send_output_setup(state_names, state_types, int(cfg.frequency_hz))
        setp = con.send_input_setup(setp_names, setp_types)
        watchdog = con.send_input_setup(watchdog_names, watchdog_types)

        # Avoid "uninitialized" registers on robot side
        for i in range(6):
            setattr(setp, f"input_double_register_{i}", 0.0)
        try:
            setp.input_bit_registers0_to_31 = 0
        except Exception:
            pass

        watchdog.input_int_register_0 = 0

        if not con.send_start():
            raise RuntimeError("RTDE send_start() failed")

        # Read one state to initialize latest_tcp and default setpoint
        state = con.receive()
        if state is None:
            raise RuntimeError("RTDE receive() returned None right after send_start()")
        self._latest_tcp = list(state.actual_TCP_pose)
        self._latest_runtime_state = int(state.runtime_state)

        # Set initial target to current TCP
        with self._lock:
            self._target_tcp = list(self._latest_tcp)

        # Switch to servoj mode (2) once
        watchdog.input_int_register_0 = int(cfg.servoj_mode)
        con.send(watchdog)

        self._con = con
        self._setp = setp
        self._watchdog = watchdog

        # Start background thread
        self._running = True
        self._th = threading.Thread(target=self._loop, name="rtde_servoj_loop", daemon=True)
        self._th.start()

    def _loop(self) -> None:
        con = self._con
        setp = self._setp
        if con is None or setp is None:
            return

        while self._running:
            state = con.receive()
            if state is None:
                continue

            self._latest_tcp = list(state.actual_TCP_pose)
            self._latest_runtime_state = int(state.runtime_state)

            # Only send setpoints when program is running
            if self._latest_runtime_state > 1:
                with self._lock:
                    tgt = self._target_tcp

                if tgt is not None:
                    for i in range(6):
                        setattr(setp, f"input_double_register_{i}", float(tgt[i]))
                    con.send(setp)

    def set_target_tcp(self, pose6: Sequence[float]) -> None:
        if len(pose6) != 6:
            raise ValueError("pose6 must be length-6")
        with self._lock:
            self._target_tcp = [float(x) for x in pose6]

    def get_latest_tcp(self) -> Optional[list]:
        return None if self._latest_tcp is None else list(self._latest_tcp)

    def stop(self) -> None:
        self._running = False
        if self._th is not None:
            self._th.join(timeout=1.0)

        try:
            if self._con is not None:
                self._con.send_pause()
        except Exception:
            pass

        try:
            if self._con is not None:
                self._con.disconnect()
        except Exception:
            pass
