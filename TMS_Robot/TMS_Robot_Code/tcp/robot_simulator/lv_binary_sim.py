#!/usr/bin/env python3
"""
lv_binary_sim.py -- stand-in for the LabVIEW Main VI's binary TCP interface (the one codes/TMS_MATLAB_Controller
talks to), for testing tms_bridge.py --robot-proto binary without the robot.

    client -> VI : 72 bytes = 18 big-endian float32
                   [x_speed x_pos 0 y_speed y_pos 0 z_speed z_pos 0 psi_speed psi_pos 0
                    tab_mode curve_speed theta_pos 0 phi_pos 0]
    VI -> client : 44 bytes = 11 big-endian float32 every 100 ms
                   [x_moving x_real y_moving y_real z_moving z_real psi_moving psi_real
                    curve_moving theta_real phi_real]          moving: 1 = idle, 0 = moving

Motion: steps at the packet's speed (steps/s), 26 steps/mm for X/Y/Z/PSI, 0.15778 deg/step for THETA/PHI.
'real' is the position reached so far. tab_mode 0 takes theta_pos, 1 takes phi_pos (like the curve VI).

    python3 lv_binary_sim.py [--port 6341] [--theta -85] [--phi 53.1916]
"""
import argparse
import socket
import struct
import time

TX = struct.Struct(">18f")
RX = struct.Struct(">11f")
UNIT = dict(x=1 / 26, y=1 / 26, z=1 / 26, psi=1 / 26, theta=0.15778, phi=0.15778)


def serve(c, a):
    pos = dict(x=0.0, y=0.0, z=0.0, psi=0.0, theta=a.theta, phi=a.phi)
    tgt = dict(pos)
    spd = dict(x=4.0, y=4.0, z=4.0, psi=4.0, theta=4.0, phi=4.0)
    c.settimeout(0.1)
    buf = b""
    last = time.monotonic()
    t_start = 0.0
    n = 0
    while True:
        try:
            d = c.recv(4096)
            if not d:
                return
            buf += d
        except socket.timeout:
            pass
        while len(buf) >= TX.size:
            v = TX.unpack(buf[:TX.size])
            buf = buf[TX.size:]
            n += 1
            for k, i in (("x", 0), ("y", 3), ("z", 6), ("psi", 9)):
                spd[k], tgt[k] = v[i], v[i + 1]
            spd["theta"] = spd["phi"] = v[13]
            if round(v[12]) == 0:
                tgt["theta"] = v[14]
            else:
                tgt["phi"] = v[16]
            t_start = time.monotonic() + a.latency
            print(f"RX 72B #{n}: " + " ".join(f"{k}={tgt[k]:.4g}" for k in tgt) + f" tab={v[12]:g}", flush=True)
        now = time.monotonic()
        dt, last = now - last, now
        moving = {}
        for k in pos:
            if now < t_start:
                moving[k] = False
                continue
            e = tgt[k] - pos[k]
            if abs(e) < UNIT[k] / 2:
                pos[k] = tgt[k]
                moving[k] = False
            else:
                step = min(abs(e), spd[k] * UNIT[k] * dt)
                pos[k] += step if e > 0 else -step
                moving[k] = True
        idle = lambda *ks: 0.0 if any(moving[k] for k in ks) else 1.0
        r = tgt if a.report == "target" else pos
        c.sendall(RX.pack(idle("x"), r["x"], idle("y"), r["y"], idle("z"), r["z"], idle("psi"), r["psi"],
                          idle("theta", "phi"), r["theta"], r["phi"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=6341)
    ap.add_argument("--theta", type=float, default=-85.0)
    ap.add_argument("--phi", type=float, default=53.1916)
    ap.add_argument("--report", choices=("target", "position"), default="target",
                    help="value sent back per axis: target (as the real VI, 2026-11-24) or position")
    ap.add_argument("--latency", type=float, default=1.0,
                    help="seconds between a packet and the start of motion (real VI: ~1 s)")
    a = ap.parse_args()
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((a.host, a.port))
    s.listen(1)
    print(f"LabVIEW binary VI simulator on {a.host}:{a.port}", flush=True)
    while True:
        c, addr = s.accept()
        print(f"client {addr}", flush=True)
        try:
            serve(c, a)
        except OSError as e:
            print(f"client gone: {e}", flush=True)
        c.close()


if __name__ == "__main__":
    main()
