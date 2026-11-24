#!/usr/bin/env python3
"""
tms_robot_sim.py -- stand-in for the TMS-robot TCP server (protocol v1)

Lets nav_camera_terminal (nav_camera_tool_comm.m) be developed without the
Windows/LabVIEW PC. Follows the contract in TMS_Robot/doc/PROTOCOL.md (fetched
from the robot with GET DOC PROTOCOL on Nov 17 2026):

  on connect      -> HELLO TMS-ROBOT 1
  ASCII lines ending CRLF (bare LF accepted), one reply line per request,
  max 512 bytes; commands are case-insensitive
  HELP            -> OK HELP <summary>
  HELLO ...       -> OK HELLO TMS-ROBOT 1
  PING            -> OK PONG
  GET DOC <BRIEF|PROTOCOL|GUIDE> -> DOC <name> <nbytes>\r\n + <nbytes> raw bytes
  GET STATUS      -> STATUS t=..;x=..;y=..;z=..;psi=..;theta=..;phi=..;
                     mx=..;my=..;mz=..;mpsi=..;ml=..;mr=..;ox=..;oy=..;oz=..;opsi=..;ol=..;or=..
  SET <X|Y|Z|PSI> <mm>      absolute target, -25..25, quantized to 1/26 mm
  SET THETA <v>             absolute azimuth target, 0.15778/step, start -85, moves L and R
  SET PHI <v>               absolute elevation target, start 53.1916, moves L and R
  SET <L|R> <steps>         relative steps on one motor
                  -> OK SET <AX> <value>
  DELTA <dx> <dy> <dz> <d> (mm)  store tracker offset, no motion -> OK DELTA <the 4 tokens as received>
  GET DELTA       -> DELTA <dx> <dy> <dz> <d> <age_ms>   or   DELTA NONE
  BYE             -> OK BYE, connection closed (robot keeps running)
  STOP            -> OK STOP, ends the robot program (here: the simulator exits)
  errors          -> ERR <UNKNOWN_CMD|BAD_ARGS|BAD_AXIS|OUT_OF_RANGE|BUSY> <text>
  one client at a time (others get ERR BUSY and are closed); 10 s idle -> disconnect
  numbers in STATUS use the short %g form (x=0, x=0.5)

Motion is step-based at --speed steps/s (4 = panel default, so SET X 0.5 =
13 steps keeps mx=1 for ~3.25 s). As on the real VI, the reported position
jumps to the (step-quantized) target at once; only the m* flag shows when
the move is done. THETA/PHI moves set ml/mr.

Operator 'Remote' switch: start with --operator (Remote off) or toggle it
while running with  kill -USR1 <pid>. While the operator has control, SET
gets 'ERR BUSY operator has control'; STATUS, DELTA, PING always work.

Usage:
  python3 tms_robot_sim.py                    # listen on 127.0.0.1:6340
  python3 tms_robot_sim.py --host 0.0.0.0 --port 6340 --speed 4 --operator -v
Then in MATLAB, before pressing 'Comm.':
  setenv('TMS_ROBOT_HOST','127.0.0.1')        % setenv('TMS_ROBOT_HOST','') -> real robot PC

fhlin@Nov 17, 2026
"""
import argparse
import math
import os
import random
import signal
import socketserver
import threading
import time

LINEAR = ("X", "Y", "Z", "PSI")
ANGULAR = ("THETA", "PHI")
STEPPERS = ("L", "R")
MM_PER_STEP = 2 / 52                      # 26 steps/mm
THETA_PER_STEP = 10800 / (21788 * math.pi)  # ~0.15778
PHI_PER_STEP = THETA_PER_STEP             # PHI units not verified on the robot side; assume the same
LIMIT_MM = 25.0
IDLE_TIMEOUT_S = 10.0
HERE = os.path.dirname(os.path.abspath(__file__))
# GET DOC names (protocol v1) -> files, as on the bridge: BRIEF = TMS_Robot/README.md, PROTOCOL, GUIDE = bridge guide
DOC_FILES = dict(BRIEF=os.path.join(HERE, "..", "README.md"),
                 PROTOCOL=os.path.join(HERE, "..", "doc", "PROTOCOL.md"),
                 GUIDE=os.path.join(HERE, "..", "doc", "WINDOWS_BRIDGE.md"))

HELP = ("OK HELP protocol v1, ASCII lines ending CRLF, one reply per request | PING -> OK PONG | "
        "GET STATUS -> STATUS k=v;k=v... | SET <X|Y|Z|PSI> <mm -25..25> | SET <THETA|PHI> <value> (absolute targets) | "
        "SET <L|R> <steps> (relative) | DELTA <dx> <dy> <dz> <d> (store tracker offset in mm, no motion) -> OK DELTA echo | "
        "GET DELTA -> DELTA <dx> <dy> <dz> <d> <age_ms> or DELTA NONE | STOP (ends robot program) | "
        "BYE (close connection) | HELP | GET DOC <BRIEF|PROTOCOL|GUIDE> -> 'DOC <name> <nbytes>' then nbytes raw text")


def g(v):
    """short number form used in STATUS: 0, 0.5, -84.0533"""
    return f"{v:.6g}"


class Robot:
    """Shared robot state. Motors are kept in steps and move at `speed` steps/s;
    the reported value is the commanded target (like the VI's Real Output)."""

    def __init__(self, speed, operator):
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.speed = speed
        self.operator = operator             # True: panel Remote switch off, SET refused
        self.unit = dict(X=MM_PER_STEP, Y=MM_PER_STEP, Z=MM_PER_STEP, PSI=MM_PER_STEP,
                         THETA=THETA_PER_STEP, PHI=PHI_PER_STEP)
        self.value = dict(X=0.0, Y=0.0, Z=0.0, PSI=0.0, THETA=-85.0, PHI=53.1916)   # reported targets
        self.pos = {ax: 0 for ax in self.value}   # motor steps, counted from the start position
        self.target = dict(self.pos)
        self.motor = dict(L=0, R=0)        # remaining relative steps on L/R
        self.last_update = time.monotonic()
        self.last_delta = None
        self.last_delta_t = None

    def _advance(self):
        # whole steps earned since last_update; the unused fraction carries over to the next call
        # (set() calls this before changing a target, so idle time never turns into instant motion)
        n = int(self.speed * (time.monotonic() - self.last_update))
        if n <= 0:
            return
        self.last_update += n / self.speed
        for ax in self.pos:
            err = self.target[ax] - self.pos[ax]
            self.pos[ax] += max(-n, min(n, err))
        for m in self.motor:
            r = self.motor[m]
            self.motor[m] = r - max(-n, min(n, r))

    def status(self):
        with self.lock:
            self._advance()
            t = int((time.monotonic() - self.t0) * 1000)
            mv = {ax: int(self.pos[ax] != self.target[ax]) for ax in self.pos}
            angle_moving = mv["THETA"] or mv["PHI"]   # THETA and PHI drive L and R together
            f = [f"t={t}"]
            f += [f"{ax.lower()}={g(self.value[ax])}" for ax in LINEAR + ANGULAR]
            f += [f"m{ax.lower()}={mv[ax]}" for ax in LINEAR]
            f += [f"ml={int(angle_moving or self.motor['L'] != 0)}", f"mr={int(angle_moving or self.motor['R'] != 0)}"]
            f += [f"o{k}={g(round(random.uniform(0.08, 0.14), 2))}" for k in ("x", "y", "z", "psi", "l", "r")]
            return "STATUS " + ";".join(f)

    def set(self, ax, value):
        with self.lock:
            self._advance()
            if ax in STEPPERS:
                self.motor[ax] += int(value)
            else:
                # like the VI: steps = round((target - position) / step), position += steps * step,
                # so the grid is anchored at the start value (THETA -85 -> -84 gives -84.0533)
                steps = round((value - self.value[ax]) / self.unit[ax])
                self.target[ax] += steps
                self.value[ax] = round(self.value[ax] + steps * self.unit[ax], 6)

    def get_delta(self):
        if self.last_delta is None:
            return "DELTA NONE"
        age = int((time.monotonic() - self.last_delta_t) * 1000)
        return "DELTA " + " ".join(self.last_delta) + f" {age}"


def read_doc(name):
    path = DOC_FILES[name]
    try:
        with open(path, "rb") as fh:
            return fh.read()
    except OSError:
        return f"# {name}\n\n(not available in the simulator: {path} missing)\n".encode()


def handle_line(robot, line, allow_delta, log):
    """Return (reply_bytes, action) where action is None, 'bye' or 'stop'."""
    words = line.split()
    if not words:
        return b"ERR UNKNOWN_CMD empty line", None
    cmd = words[0].upper()
    if cmd == "HELP":
        return HELP.encode(), None
    if cmd == "HELLO":
        return b"OK HELLO TMS-ROBOT 1 (send HELP for commands)", None
    if cmd == "PING":
        return b"OK PONG", None
    if cmd == "GET":
        what = words[1].upper() if len(words) > 1 else ""
        if what == "STATUS" and len(words) == 2:
            return robot.status().encode(), None
        if what == "DELTA" and len(words) == 2 and allow_delta:
            return robot.get_delta().encode(), None
        if what == "DOC" and len(words) == 3 and words[2].upper() in ("BRIEF", "PROTOCOL", "GUIDE"):
            name = words[2].upper()
            body = read_doc(name)
            return f"DOC {name} {len(body)}\r\n".encode() + body, "raw"
        return b"ERR BAD_ARGS use GET STATUS, GET DELTA or GET DOC <BRIEF|PROTOCOL|GUIDE>", None
    if cmd == "SET":
        if len(words) != 3:
            return b"ERR BAD_ARGS use SET <axis> <value>", None
        ax = words[1].upper()
        if ax not in LINEAR + ANGULAR + STEPPERS:
            return f"ERR BAD_AXIS {words[1]}".encode(), None
        try:
            value = int(words[2]) if ax in STEPPERS else float(words[2])
        except ValueError:
            return f"ERR BAD_ARGS {words[2]} is not a number".encode(), None
        if ax in LINEAR and not -LIMIT_MM <= value <= LIMIT_MM:
            return f"ERR OUT_OF_RANGE {ax} must be -{LIMIT_MM}..{LIMIT_MM}".encode(), None
        if robot.operator:
            return b"ERR BUSY operator has control", None
        robot.set(ax, value)
        return f"OK SET {ax} {words[2]}".encode(), None
    if cmd == "DELTA" and allow_delta:
        try:
            if len(words) != 5:
                raise ValueError
            dx, dy, dz, d = (float(w) for w in words[1:5])
            if not all(math.isfinite(v) for v in (dx, dy, dz, d)) or d < 0:
                raise ValueError
        except ValueError:
            return b"ERR BAD_ARGS use DELTA <dx> <dy> <dz> <d> (mm, d >= 0)", None
        robot.last_delta = tuple(words[1:5])
        robot.last_delta_t = time.monotonic()
        if log:
            print(f"  delta dx={dx:+.1f} dy={dy:+.1f} dz={dz:+.1f} |d|={d:.1f} mm", flush=True)
        return ("OK DELTA " + " ".join(words[1:5])).encode(), None   # ack echoes the values as received
    if cmd == "BYE":
        return b"OK BYE", "bye"
    if cmd == "STOP":
        return b"OK STOP", "stop"
    return f"ERR UNKNOWN_CMD {cmd} (send HELP for the command list)".encode(), None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=6340)
    ap.add_argument("--speed", type=float, default=4.0, help="motor speed (steps/s); 4 = panel default")
    ap.add_argument("--operator", action="store_true", help="start with the panel Remote switch off (SET -> ERR BUSY)")
    ap.add_argument("--no-delta", action="store_true", help="reject DELTA / GET DELTA (server before Nov 17 2026)")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every request/reply")
    args = ap.parse_args()
    robot = Robot(args.speed, args.operator)
    client_lock = threading.Lock()

    def toggle_remote(*_):
        robot.operator = not robot.operator
        print(f"Remote switch {'OFF: operator has control' if robot.operator else 'ON: TCP may SET'}", flush=True)
    signal.signal(signal.SIGUSR1, toggle_remote)

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            peer = "%s:%d" % self.client_address
            if not client_lock.acquire(blocking=False):
                self.wfile.write(b"ERR BUSY another client connected\r\n")
                print(f"[{peer}] refused: busy", flush=True)
                return
            try:
                self.serve(peer)
            finally:
                client_lock.release()

        def serve(self, peer):
            print(f"[{peer}] connected", flush=True)
            self.request.settimeout(IDLE_TIMEOUT_S)
            self.wfile.write(b"HELLO TMS-ROBOT 1\r\n")
            buf = b""
            while True:
                try:
                    chunk = self.request.recv(4096)
                except TimeoutError:
                    print(f"[{peer}] idle {IDLE_TIMEOUT_S:.0f} s: disconnected", flush=True)
                    return
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.rstrip(b"\r")[:512].decode("ascii", "replace")
                    reply, action = handle_line(robot, line, not args.no_delta, True)
                    if args.verbose and not line.upper().startswith(("DELTA", "PING", "GET STATUS")):
                        head = reply.split(b"\r\n", 1)[0].decode()
                        print(f"[{peer}] > {line}\n[{peer}] < {head}", flush=True)
                    self.wfile.write(reply if action == "raw" else reply + b"\r\n")
                    if action == "bye":
                        print(f"[{peer}] bye", flush=True)
                        return
                    if action == "stop":
                        print(f"[{peer}] STOP received: robot program ends", flush=True)
                        threading.Thread(target=self.server.shutdown, daemon=True).start()
                        return
            print(f"[{peer}] disconnected", flush=True)

    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    with Server((args.host, args.port), Handler) as srv:
        print(f"TMS-robot simulator (protocol v1{'' if args.no_delta else ' + DELTA'}) "
              f"listening on {args.host}:{args.port}, pid {os.getpid()}, "
              f"Remote {'OFF' if robot.operator else 'ON'} (kill -USR1 {os.getpid()} toggles; Ctrl-C quits)", flush=True)
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
