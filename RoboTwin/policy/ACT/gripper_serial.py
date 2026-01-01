#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Gripper (serial) control - clean functional wrapper.

Assumptions:
- You already fixed permission: add your user to 'dialout' group so you can open /dev/ttyUSB* without sudo.
- The gripper accepts raw 7-byte commands (see your vendor manual).

Usage:
    from gripper_serial import GripperSerial
    g = GripperSerial(port="/dev/ttyUSB0", baudrate=9600)
    g.open()
    g.close()
    g.shutdown()
"""

# =========================
# CONFIG (default values)
# =========================
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 9600
DEFAULT_TIMEOUT_S = 1.0

# Raw command bytes (from your minimal demo)
MOTOR_OPEN_CMD  = bytes([0x02, 0x00, 0x20, 0x2F, 0x00, 0x00, 0xA4])
MOTOR_CLOSE_CMD = bytes([0x02, 0x01, 0x20, 0x2F, 0x00, 0x00, 0xA4])

import time
import serial


class GripperSerial:
    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE, timeout_s: float = DEFAULT_TIMEOUT_S):
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)
        # Some USB-serial adapters need a short delay after open
        time.sleep(0.2)

    def open(self):
        """Command gripper to open."""
        self.ser.write(MOTOR_OPEN_CMD)
        self.ser.flush()

    def close(self):
        """Command gripper to close."""
        self.ser.write(MOTOR_CLOSE_CMD)
        self.ser.flush()

    def shutdown(self):
        try:
            self.ser.close()
        except Exception:
            pass


def _demo():
    g = GripperSerial()
    print("Gripper demo: close -> wait -> open")
    g.close()
    time.sleep(2)
    g.open()
    g.shutdown()


if __name__ == "__main__":
    _demo()
