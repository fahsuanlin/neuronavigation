#!/usr/bin/env python3
"""
tms_bridge.py -- TMS Robot Bridge: standalone Windows program between the Mac (Polaris navigation
terminal, nav_camera_terminal) and the LabVIEW robot program (TMS Robot_adjust.vi on the myRIO).

    Mac (MATLAB, client) --TCP 6340--> [ tms_bridge on the Windows PC ] --TCP--> LabVIEW VI TCP loop (myRIO)
                                         monitor window, Remote switch,           or the built-in simulator
                                         STOP blocked, DELTA store

Mac side: listens on --listen (default 0.0.0.0:6340) and speaks TMS-robot protocol v1
(doc/tms_robot/PROTOCOL.md) exactly, so the Mac code needs no change:
    greeting HELLO TMS-ROBOT 1 bridge ...; one client at a time (others: ERR BUSY); 10 s idle -> disconnect
    answered by the bridge itself : PING, HELLO, HELP, BYE, GET DOC <name>, GET DELTA,
                                    DELTA <dx> <dy> <dz> <d> (validated, acked with the echo, stored, and the
                                    newest one forwarded to LabVIEW in the background)
    forwarded to LabVIEW          : GET STATUS, SET <axis> <value>, and any other command
                                    (robot not connected -> ERR BUSY robot not connected)
    refused                       : SET while 'Allow motion from Mac' is off -> ERR BUSY operator has control
                                    STOP -> ERR BUSY (it would end the robot program); --allow-stop to permit

LabVIEW side (--robot HOST:PORT, default 172.22.11.2:6340 = myRIO over USB), one persistent client connection:
  --robot-proto binary (default): the Main VI's binary interface, the same one codes/TMS_MATLAB_Controller
      uses: 72-byte command packets (18 big-endian float32) out, 44-byte frames (11 float32) in every ~100 ms.
      GET STATUS is answered from the newest frame; SET <X|Y|Z|PSI|THETA|PHI> updates one target and sends a
      full packet (targets start from the first frame, so connecting sends nothing and moves nothing).
      No L/R steps, no DELTA forwarding (DELTA is kept on the bridge for GET DELTA).
  --robot-proto text: protocol v1 text lines to a VI TCP loop as in doc/tms_robot/GUIDE.md.
Reconnects every 2 s when the link is down.
--robot sim : built-in simulated robot (4 steps/s, 26 steps/mm, THETA -85, PHI 53.1916).
Test the binary path without the robot: tcp/robot_simulator/lv_binary_sim.py --port 6341, then
    py tms_bridge.py --robot 127.0.0.1:6341

Usage (Windows, Python 3.8+, standard library only):
    py tms_bridge.py                                  # GUI; Mac on 0.0.0.0:6340; LabVIEW at 172.22.11.2:6340
    py tms_bridge.py --robot sim                      # GUI with the built-in simulated robot
    py tms_bridge.py --robot 127.0.0.1:6341 --nogui   # console only
    py tms_bridge.py --remote                         # start with 'Allow motion from Mac' on
Firewall (admin PowerShell, once):
    New-NetFirewallRule -DisplayName "TMS Robot TCP 6340" -Direction Inbound -Protocol TCP -LocalPort 6340 -Action Allow -Profile Any
Single .exe (optional): build_exe.bat (PyInstaller).

fhlin@Nov 19, 2026
"""
import argparse
import collections
import math
import os
import queue
import random
import socket
import struct
import sys
import threading
import time

VERSION = "1"
PROTO_GREETING = f"HELLO TMS-ROBOT {VERSION}"
IDLE_TIMEOUT_S = 10.0
MAX_LINE = 512
ROBOT_TIMEOUT_S = 2.0
RECONNECT_S = 2.0
LIMIT_MM = 25.0
LINEAR = ("X", "Y", "Z", "PSI")
ANGULAR = ("THETA", "PHI")
STEPPERS = ("L", "R")
AXES = LINEAR + ANGULAR + STEPPERS

HELP = ("OK HELP protocol v1 via tms_bridge, ASCII lines ending CRLF, one reply per request | PING -> OK PONG | "
        "GET STATUS -> STATUS k=v;k=v... | SET <X|Y|Z|PSI> <mm -25..25> | SET <THETA|PHI> <value> (absolute targets) | "
        "SET <L|R> <steps> (relative) | DELTA <dx> <dy> <dz> <d> (store tracker offset in mm, no motion) -> OK DELTA echo | "
        "GET DELTA -> DELTA <dx> <dy> <dz> <d> <age_ms> or DELTA NONE | STOP (blocked by the bridge) | "
        "BYE (close connection) | HELP | GET DOC <BRIEF|PROTOCOL|GUIDE> -> 'DOC <name> <nbytes>' then nbytes raw text")


def g(v):
    return f"{v:.6g}"


def hostport(s):
    h, p = s.rsplit(":", 1)
    return h, int(p)


def parse_status(line):
    out = {}
    for kv in line[len("STATUS "):].split(";"):
        k, _, v = kv.partition("=")
        try:
            out[k.strip()] = float(v)
        except ValueError:
            pass
    return out


# ======================================================================================
# LabVIEW side
# ======================================================================================
class SimRobot:
    """Built-in robot model (same behaviour as codes/tms_robot_sim.py): step-based motion, the reported
    value is the quantized target, the m* flags show motion."""
    MM = 2 / 52
    ANG = 10800 / (21788 * math.pi)

    def __init__(self, speed=4.0):
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.speed = speed
        self.unit = dict(X=self.MM, Y=self.MM, Z=self.MM, PSI=self.MM, THETA=self.ANG, PHI=self.ANG)
        self.value = dict(X=0.0, Y=0.0, Z=0.0, PSI=0.0, THETA=-85.0, PHI=53.1916)
        self.pos = {a: 0 for a in self.value}
        self.target = dict(self.pos)
        self.motor = dict(L=0, R=0)
        self.last = time.monotonic()

    def _advance(self):
        n = int(self.speed * (time.monotonic() - self.last))
        if n <= 0:
            return
        self.last += n / self.speed
        for a in self.pos:
            e = self.target[a] - self.pos[a]
            self.pos[a] += max(-n, min(n, e))
        for m in self.motor:
            r = self.motor[m]
            self.motor[m] = r - max(-n, min(n, r))

    def request(self, line):
        w = line.split()
        cmd = w[0].upper() if w else ""
        with self.lock:
            self._advance()
            if cmd == "GET" and len(w) == 2 and w[1].upper() == "STATUS":
                mv = {a: int(self.pos[a] != self.target[a]) for a in self.pos}
                ang = mv["THETA"] or mv["PHI"]
                f = [f"t={int((time.monotonic() - self.t0) * 1000)}"]
                f += [f"{a.lower()}={g(self.value[a])}" for a in LINEAR + ANGULAR]
                f += [f"m{a.lower()}={mv[a]}" for a in LINEAR]
                f += [f"ml={int(ang or self.motor['L'] != 0)}", f"mr={int(ang or self.motor['R'] != 0)}"]
                f += [f"o{k}={g(round(random.uniform(0.08, 0.14), 2))}" for k in ("x", "y", "z", "psi", "l", "r")]
                return "STATUS " + ";".join(f)
            if cmd == "SET" and len(w) == 3:
                a = w[1].upper()
                if a not in AXES:
                    return f"ERR BAD_AXIS {w[1]}"
                try:
                    v = int(w[2]) if a in STEPPERS else float(w[2])
                except ValueError:
                    return f"ERR BAD_ARGS {w[2]} is not a number"
                if a in LINEAR and not -LIMIT_MM <= v <= LIMIT_MM:
                    return f"ERR OUT_OF_RANGE {a} must be -{LIMIT_MM}..{LIMIT_MM}"
                if a in STEPPERS:
                    self.motor[a] += v
                else:
                    steps = round((v - self.value[a]) / self.unit[a])
                    self.target[a] += steps
                    self.value[a] = round(self.value[a] + steps * self.unit[a], 6)
                return f"OK SET {a} {w[2]}"
            if cmd == "DELTA":
                return "OK DELTA " + " ".join(w[1:])
            if cmd == "PING":
                return "OK PONG"
            return f"ERR UNKNOWN_CMD {cmd}"


class RobotLink:
    """One persistent connection to the LabVIEW TCP loop (or the built-in simulator)."""

    def __init__(self, core, target, poll_hz):
        self.core = core
        self.target = target              # "sim" or "host:port"
        self.poll_s = 1.0 / poll_hz if poll_hz > 0 else None
        self.lock = threading.Lock()      # one request at a time: replies come back in order
        self.sock = None
        self.rfile = None
        self.sim = SimRobot(core.args.sim_speed) if target == "sim" else None
        self.connected = self.sim is not None
        self.greeting = "(built-in simulator)" if self.sim else ""
        self.state = "connected (simulator)" if self.sim else "not connected"
        self.rtt_ms = None
        self.status = {}
        self.status_t = None
        self.n_req = 0
        self.n_err = 0
        self.delta_unsupported = False
        self.delta_q = queue.Queue(maxsize=1)
        self.stop_ev = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True, name="robot-link").start()
        threading.Thread(target=self._delta_worker, daemon=True, name="robot-delta").start()

    # -- connection ---------------------------------------------------------------------
    def _connect(self):
        h, p = hostport(self.target)
        self.state = f"connecting to {h}:{p}"
        try:
            s = socket.create_connection((h, p), timeout=3)
            s.settimeout(ROBOT_TIMEOUT_S)
            rf = s.makefile("rb")
            greet = rf.readline(MAX_LINE + 2).decode("utf-8", "replace").strip()
        except OSError as e:
            self.state = f"not connected ({h}:{p}: {e})"
            return False
        tok = greet.split()
        if len(tok) < 3 or tok[0] != "HELLO" or tok[1] != "TMS-ROBOT" or tok[2] != VERSION:
            self.core.log("!", f"LabVIEW {h}:{p}: unexpected greeting [{greet}]; expected '{PROTO_GREETING}'")
            self.state = f"bad greeting from {h}:{p}"
            s.close()
            return False
        with self.lock:
            self.sock, self.rfile = s, rf
            self.connected = True
            self.greeting = greet
            self.delta_unsupported = False
        self.state = f"connected to {h}:{p}"
        self.core.log("*", f"LabVIEW connected: {h}:{p}  [{greet}]")
        return True

    def drop(self, why):
        with self.lock:
            self._drop_locked(why)

    def _drop_locked(self, why):
        if self.sim or not self.connected:
            return
        self.connected = False
        try:
            self.sock.close()
        except OSError:
            pass
        self.sock = self.rfile = None
        self.state = f"not connected ({why})"
        self.core.log("!", f"LabVIEW link lost: {why}")

    def reconnect(self):
        if self.sim:
            return
        self.drop("reconnect requested")

    # -- requests -----------------------------------------------------------------------
    def request(self, line):
        """Send one line, return its reply line, or None if the robot is not reachable."""
        with self.lock:
            if self.sim:
                t = time.perf_counter()
                r = self.sim.request(line)
                self.rtt_ms = (time.perf_counter() - t) * 1000
            else:
                if not self.connected:
                    return None
                try:
                    t = time.perf_counter()
                    self.sock.sendall((line + "\r\n").encode())
                    raw = self.rfile.readline(MAX_LINE + 2)
                    if not raw:
                        raise OSError("closed by LabVIEW")
                    r = raw.decode("utf-8", "replace").strip()
                    self.rtt_ms = (time.perf_counter() - t) * 1000
                except (OSError, socket.timeout) as e:
                    self._drop_locked(f"{line.split()[0] if line else ''}: {e or 'timeout'}")
                    return None
            self.n_req += 1
            if r.startswith("ERR"):
                self.n_err += 1
            if r.startswith("STATUS "):
                self.status = parse_status(r)
                self.status_t = time.monotonic()
            return r

    def send_delta(self, line):
        """newest DELTA wins: forwarded in the background, never delays the Mac"""
        try:
            self.delta_q.get_nowait()
        except queue.Empty:
            pass
        try:
            self.delta_q.put_nowait(line)
        except queue.Full:
            pass

    def _delta_worker(self):
        while not self.stop_ev.is_set():
            line = self.delta_q.get()
            if self.delta_unsupported or not self.connected:
                continue
            r = self.request(line)
            if r is None:
                continue
            if r.startswith("ERR UNKNOWN_CMD"):
                self.delta_unsupported = True
                self.core.log("!", "LabVIEW does not know DELTA; offsets are kept on the bridge only (GET DELTA).")
            elif r != "OK " + line:
                self.core.log("!", f"LabVIEW: sent [{line}], got [{r}]")

    def _run(self):
        while not self.stop_ev.is_set():
            if not self.connected:
                if not self._connect():
                    self.stop_ev.wait(RECONNECT_S)
                    continue
            if self.poll_s:
                self.request("GET STATUS")   # monitor display; also keeps the link alive
                self.stop_ev.wait(self.poll_s)
            else:
                if time.monotonic() - (self.status_t or 0) > 4:
                    self.request("PING")
                self.stop_ev.wait(0.5)


class BinaryRobotLink:
    """LabVIEW Main VI binary interface (codes/TMS_MATLAB_Controller, README.txt):
        bridge -> VI : 72 bytes = 18 big-endian float32
                       [x_speed x_pos 0  y_speed y_pos 0  z_speed z_pos 0  psi_speed psi_pos 0
                        tab_mode curve_speed theta_pos 0  phi_pos 0]        (the 0s are Jog, never used)
        VI -> bridge : 44 bytes = 11 big-endian float32, sent by the VI every ~100 ms
                       [x_moving x_real y_moving y_real z_moving z_real psi_moving psi_real
                        curve_moving theta_real phi_real]                    (moving: 1 = idle, 0 = moving)
    Text requests from the Mac are translated: GET STATUS is answered from the newest frame, SET changes one
    target and sends a full packet. Targets start from the first frame's values, and nothing is sent before a
    SET, so connecting never moves the robot. THETA uses tab_mode 0, PHI tab_mode 1 (the curve VI takes one
    of them per packet). A SET axis is reported as moving until the frames show the move (the next frame
    may still say idle), so the Mac's Goto waits correctly."""
    TX = struct.Struct(">18f")
    RX = struct.Struct(">11f")
    FRAME_TIMEOUT_S = 2.0
    PENDING_MAX_S = 1.5             # a SET that changes nothing never shows motion: stop waiting after this
    STEP = dict(X=1 / 26, Y=1 / 26, Z=1 / 26, PSI=1 / 26, THETA=0.15778, PHI=0.15778)

    def __init__(self, core, target, speed, curve_speed):
        self.core = core
        self.target = target
        self.lock = threading.Lock()
        self.sock = None
        self.connected = False
        self.greeting = ""
        self.state = "not connected"
        self.rtt_ms = None                # here: interval between VI frames
        self.status = {}
        self.status_t = None
        self.n_req = 0
        self.n_err = 0
        self.n_rx = 0
        self.n_tx = 0
        self.delta_unsupported = True     # the binary interface has no DELTA; offsets stay on the bridge
        self.speed = float(speed)
        self.curve_speed = float(curve_speed)
        self.cmd = None                   # current targets, seeded from the first frame after each connect
        self.tab_mode = 0
        self.pending = {}                 # axis -> (t_set, saw_motion)
        self.stop_ev = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True, name="robot-link").start()

    def send_delta(self, line):
        pass

    def drop(self, why):
        with self.lock:
            self._drop_locked(why)

    def _drop_locked(self, why):
        if not self.connected and self.sock is None:
            return
        self.connected = False
        try:
            self.sock.close()
        except (OSError, AttributeError):
            pass
        self.sock = None
        self.cmd = None
        self.pending.clear()
        self.state = f"not connected ({why})"
        self.core.log("!", f"LabVIEW link lost: {why}")

    def reconnect(self):
        self.drop("reconnect requested")

    # -- frames from the VI -------------------------------------------------------------
    def _run(self):
        while not self.stop_ev.is_set():
            h, p = hostport(self.target)
            self.state = f"connecting to {h}:{p}"
            try:
                s = socket.create_connection((h, p), timeout=3)
                s.settimeout(self.FRAME_TIMEOUT_S)
            except OSError as e:
                self.state = f"not connected ({h}:{p}: {e})"
                self.stop_ev.wait(RECONNECT_S)
                continue
            with self.lock:
                self.sock = s
                self.connected = True
                self.greeting = "binary 72B/44B"
                self.state = f"connected to {h}:{p}, waiting for the first 44-byte frame"
            self.core.log("*", f"LabVIEW connected: {h}:{p} (binary 72B/44B interface)")
            buf = b""
            t_last = None
            t_conn = time.monotonic()
            warned = False
            try:
                while not self.stop_ev.is_set():
                    try:
                        chunk = s.recv(4096)
                    except socket.timeout:
                        # silence is not a broken link: stay connected (reconnecting churns the VI's listener)
                        quiet = time.monotonic() - (self.status_t if t_last else t_conn)
                        self.state = (f"connected to {h}:{p}, no 44-byte frame for {quiet:.0f} s"
                                      + ("" if t_last else " (the VI may wait for a first 72-byte packet)"))
                        if not warned:
                            self.core.log("!", f"LabVIEW: connected but no 44-byte frame for {quiet:.0f} s"
                                          + ("" if t_last else "; the VI may send frames only after it gets a 72-byte packet"))
                            warned = True
                        continue
                    warned = False
                    if not chunk:
                        raise OSError("closed by LabVIEW")
                    buf += chunk
                    while len(buf) >= self.RX.size:
                        f = self.RX.unpack(buf[:self.RX.size])
                        buf = buf[self.RX.size:]
                        if not all(math.isfinite(v) for v in f) or any(f[i] not in (0.0, 1.0) for i in (0, 2, 4, 6, 8)):
                            raise OSError("bad 44-byte frame (moving flags not 0/1): stream out of step")
                        now = time.monotonic()
                        if t_last is not None:
                            self.rtt_ms = (now - t_last) * 1000
                        t_last = now
                        self._frame(f, now)
            except (OSError, socket.timeout) as e:
                self.drop(str(e) if str(e) else f"no frame from LabVIEW for {self.FRAME_TIMEOUT_S:g} s")
            self.stop_ev.wait(RECONNECT_S)

    def _frame(self, f, now):
        real = dict(X=f[1], Y=f[3], Z=f[5], PSI=f[7], THETA=f[9], PHI=f[10])
        mov = dict(X=f[0] == 0, Y=f[2] == 0, Z=f[4] == 0, PSI=f[6] == 0, THETA=f[8] == 0, PHI=f[8] == 0)
        with self.lock:
            if self.sock is None:
                return
            self.n_rx += 1
            if self.cmd is None:
                self.cmd = dict(real)
                h, p = hostport(self.target)
                self.state = f"connected to {h}:{p}"
                self.core.log("*", "LabVIEW first frame: " + "  ".join(f"{a.lower()}={g(v)}" for a, v in real.items()))
            for a, (t_set, saw) in list(self.pending.items()):
                if mov[a]:
                    self.pending[a] = (t_set, True)
                elif saw or abs(real[a] - self.cmd[a]) < self.STEP[a] / 2 or now - t_set > self.PENDING_MAX_S:
                    del self.pending[a]
            m = {a: int(mov[a] or a in self.pending) for a in real}
            st = dict(t=int(now * 1000) % 10**9)
            st.update({a.lower(): real[a] for a in LINEAR + ANGULAR})
            st.update({f"m{a.lower()}": m[a] for a in LINEAR})
            st["ml"] = st["mr"] = int(m["THETA"] or m["PHI"])
            self.status, self.status_t = st, now

    # -- requests from the Mac ----------------------------------------------------------
    def request(self, line):
        w = line.split()
        cmd = w[0].upper() if w else ""
        with self.lock:
            if not self.connected:
                return None
            self.n_req += 1
            r = self._request_locked(w, cmd)
            if r is None:
                return None
            if r.startswith("ERR"):
                self.n_err += 1
            return r

    def _request_locked(self, w, cmd):
        fresh = self.status_t is not None and time.monotonic() - self.status_t < 1.0
        if cmd == "PING":
            return "OK PONG"
        if cmd == "GET" and len(w) == 2 and w[1].upper() == "STATUS":
            if not fresh or self.cmd is None:
                return "ERR BUSY no recent 44-byte frame from LabVIEW"
            st = dict(self.status)
            for a in self.pending:        # SET since the last frame: report it as moving already
                for k in ((f"m{a.lower()}",) if a in LINEAR else ("ml", "mr")):
                    st[k] = 1
            out = [f"t={st['t']}"] + [f"{k}={g(st[k])}" for k in ("x", "y", "z", "psi", "theta", "phi")]
            out += [f"{k}={st[k]}" for k in ("mx", "my", "mz", "mpsi", "ml", "mr")]
            return "STATUS " + ";".join(out)
        if cmd == "SET":
            if len(w) != 3:
                return "ERR BAD_ARGS use SET <axis> <value>"
            a = w[1].upper()
            if a in STEPPERS or a not in AXES:
                return f"ERR BAD_AXIS {w[1]} (the LabVIEW binary interface has X Y Z PSI THETA PHI)"
            try:
                v = float(w[2])
                if not math.isfinite(v):
                    raise ValueError
            except ValueError:
                return f"ERR BAD_ARGS {w[2]} is not a number"
            if a in LINEAR and not -LIMIT_MM <= v <= LIMIT_MM:
                return f"ERR OUT_OF_RANGE {a} must be -{LIMIT_MM:g}..{LIMIT_MM:g}"
            if not fresh or self.cmd is None:
                return "ERR BUSY no recent 44-byte frame from LabVIEW"
            self.cmd[a] = v
            if a == "THETA":
                self.tab_mode = 0
            elif a == "PHI":
                self.tab_mode = 1
            c, sp = self.cmd, self.speed
            pkt = self.TX.pack(sp, c["X"], 0, sp, c["Y"], 0, sp, c["Z"], 0, sp, c["PSI"], 0,
                               self.tab_mode, self.curve_speed, c["THETA"], 0, c["PHI"], 0)
            try:
                self.sock.sendall(pkt)
            except OSError as e:
                self._drop_locked(f"write: {e}")
                return None
            self.n_tx += 1
            self.pending[a] = (time.monotonic(), False)
            if a in ANGULAR:   # the curve moving flag is shared by THETA and PHI
                self.pending["THETA" if a == "PHI" else "PHI"] = (time.monotonic(), False)
            self.core.log("*", f"72B #{self.n_tx}: " + "  ".join(f"{k.lower()}={g(x)}" for k, x in c.items())
                          + f"  tab={self.tab_mode}")
            return f"OK SET {a} {w[2]}"
        return f"ERR UNKNOWN_CMD {cmd} (not available through the LabVIEW binary interface)"


# ======================================================================================
# Mac side
# ======================================================================================
class MacServer:
    def __init__(self, core, listen):
        self.core = core
        self.listen = listen
        self.client = None                # (socket, addr) of the connected Mac
        self.client_lock = threading.Lock()
        self.since = None
        self.last_rx = None
        self.counts = collections.Counter()
        self.delta_times = collections.deque(maxlen=50)
        self.last_delta = None            # tuple of the 4 tokens as received
        self.last_delta_t = None
        self.last_cmd = ""
        self.state = "starting"

    def start(self):
        h, p = hostport(self.listen)
        try:
            self.srv = socket.create_server((h, p), reuse_port=False)
        except OSError as e:
            self.state = f"cannot listen on {h}:{p}: {e}"
            self.core.log("!", self.state + "  (is the old simulator or tcp_bridge.py still running?)")
            return False
        self.state = f"listening on {h}:{p}"
        self.core.log("*", f"Mac side {self.state}")
        threading.Thread(target=self._accept, daemon=True, name="mac-accept").start()
        return True

    def _accept(self):
        while True:
            try:
                c, addr = self.srv.accept()
            except OSError:
                return
            with self.client_lock:
                busy = self.client is not None
                if not busy:
                    self.client = (c, addr)
            if busy:
                try:
                    c.sendall(b"ERR BUSY another client connected\r\n")
                    c.close()
                except OSError:
                    pass
                self.core.log("!", f"refused a second client from {addr[0]}:{addr[1]} (one client at a time)")
                continue
            threading.Thread(target=self._serve, args=(c, addr), daemon=True, name="mac-client").start()

    def kick(self):
        with self.client_lock:
            if self.client:
                try:
                    self.client[0].shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

    def delta_rate(self):
        now = time.monotonic()
        ts = [t for t in self.delta_times if now - t < 2.0]
        return len(ts) / 2.0

    def _serve(self, c, addr):
        who = f"{addr[0]}:{addr[1]}"
        self.since = self.last_rx = time.monotonic()
        self.counts.clear()
        self.state = f"connected: {who}"
        self.core.log("*", f"Mac connected from {who}")
        why = "closed by the Mac"
        try:
            robot = "robot connected" if self.core.robot.connected else "robot NOT connected"
            c.sendall(f"{PROTO_GREETING} bridge ({robot})\r\n".encode())
            c.settimeout(0.5)
            buf = b""
            while True:
                try:
                    data = c.recv(4096)
                except socket.timeout:
                    if time.monotonic() - self.last_rx > IDLE_TIMEOUT_S:
                        why = f"idle for {IDLE_TIMEOUT_S:g} s"
                        break
                    continue
                if not data:
                    break
                buf += data
                self.last_rx = time.monotonic()
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("utf-8", "replace").strip()
                    if len(raw) > MAX_LINE:
                        reply, action = b"ERR BAD_ARGS line too long", None
                    else:
                        reply, action = self.core.handle(line)
                    c.sendall(reply if action == "raw" else reply + b"\r\n")   # GET DOC body: exactly nbytes
                    if action == "bye":
                        why = "BYE"
                        raise StopIteration
                if len(buf) > MAX_LINE:
                    c.sendall(b"ERR BAD_ARGS line too long\r\n")
                    buf = b""
        except StopIteration:
            pass
        except OSError as e:
            why = str(e)
        finally:
            try:
                c.close()
            except OSError:
                pass
            with self.client_lock:
                self.client = None
            self.state = f"listening on {self.listen} (last client {who} left: {why})"
            self.core.log("*", f"Mac {who} disconnected ({why}); DELTA received {self.counts['DELTA']}")


# ======================================================================================
# Core: request handling shared by both sides
# ======================================================================================
class Core:
    def __init__(self, args):
        self.args = args
        self.allow_motion = args.remote
        self.allow_stop = args.allow_stop
        self.log_q = queue.Queue()
        self.console = args.nogui
        self.verbose = args.verbose
        if args.robot != "sim" and args.robot_proto == "binary":
            self.robot = BinaryRobotLink(self, args.robot, args.speed, args.curve_speed)
        else:
            self.robot = RobotLink(self, args.robot, args.poll)
        self.mac = MacServer(self, args.listen)
        self.doc_dirs = [d for d in (args.docs,
                                     os.path.join(SCRIPT_DIR, "docs"),
                                     os.path.join(SCRIPT_DIR, "..", "..", "..", "..", "doc", "tms_robot"),
                                     os.path.join(SCRIPT_DIR, "..")) if d]

    def log(self, kind, text):
        line = f"{time.strftime('%H:%M:%S')} {kind} {text}"
        self.log_q.put(line)
        if self.console:
            print(line, flush=True)

    def set_allow_motion(self, on):
        self.allow_motion = bool(on)
        self.log("*", "motion from the Mac " + ("ALLOWED (SET is forwarded to LabVIEW)" if on else
                                                  "BLOCKED (SET -> ERR BUSY operator has control)"))

    def read_doc(self, name):
        for d in self.doc_dirs:
            p = os.path.join(d, f"{name}.md")
            if os.path.isfile(p):
                with open(p, "rb") as fh:
                    return fh.read()
        return f"# {name}\n\n(not found on the bridge; looked in: {', '.join(self.doc_dirs)})\n".encode()

    def handle(self, line):
        """one request line from the Mac -> (reply bytes, action)"""
        w = line.split()
        if not w:
            return b"ERR UNKNOWN_CMD empty line", None
        cmd = w[0].upper()
        m = self.mac
        m.counts[cmd] += 1
        if cmd != "DELTA":
            m.last_cmd = line
        if self.verbose and cmd not in ("DELTA", "PING"):
            self.log(">", line)
        r, action = self._handle(line, w, cmd)
        if self.verbose and cmd not in ("DELTA", "PING") and not r.startswith(b"DOC"):
            self.log("<", r.decode(errors="replace"))
        return r, action

    def _handle(self, line, w, cmd):
        m = self.mac
        if cmd == "PING":
            return b"OK PONG", None
        if cmd == "HELLO":
            return f"OK {PROTO_GREETING}".encode(), None
        if cmd == "HELP":
            return HELP.encode(), None
        if cmd == "BYE":
            return b"OK BYE", "bye"
        if cmd == "STOP":
            if not self.allow_stop:
                self.log("!", "Mac sent STOP: refused (it would end the robot program; start with --allow-stop to permit)")
                return b"ERR BUSY STOP is disabled on the bridge (use the robot panel)", None
        if cmd == "GET" and len(w) >= 2:
            what = w[1].upper()
            if what == "DOC":
                if len(w) != 3 or w[2].upper() not in ("BRIEF", "PROTOCOL", "GUIDE"):
                    return b"ERR BAD_ARGS use GET DOC <BRIEF|PROTOCOL|GUIDE>", None
                body = self.read_doc(w[2].upper())
                return f"DOC {w[2].upper()} {len(body)}\r\n".encode() + body, "raw"
            if what == "DELTA" and len(w) == 2:
                if m.last_delta is None:
                    return b"DELTA NONE", None
                age = int((time.monotonic() - m.last_delta_t) * 1000)
                return ("DELTA " + " ".join(m.last_delta) + f" {age}").encode(), None
        if cmd == "DELTA":
            try:
                if len(w) != 5:
                    raise ValueError
                v = [float(x) for x in w[1:5]]
                if not all(math.isfinite(x) for x in v) or v[3] < 0:
                    raise ValueError
            except ValueError:
                return b"ERR BAD_ARGS use DELTA <dx> <dy> <dz> <d> (mm, d >= 0)", None
            m.last_delta = tuple(w[1:5])
            m.last_delta_t = time.monotonic()
            m.delta_times.append(m.last_delta_t)
            norm = "DELTA " + " ".join(w[1:5])
            self.robot.send_delta(norm)
            return ("OK " + norm).encode(), None
        if cmd == "SET":
            if not self.allow_motion:
                self.log("!", f"Mac: [{line}] refused, motion from the Mac is not allowed (switch in the bridge window)")
                return b"ERR BUSY operator has control", None
            self.log(">", f"Mac -> LabVIEW: {line}")
        r = self.robot.request(line)
        if r is None:
            return f"ERR BUSY robot not connected ({self.robot.state})".encode()[:MAX_LINE], None
        if cmd == "SET" or (r.startswith("ERR") and cmd != "GET"):
            self.log("<", f"LabVIEW: {r}")
        return r.encode(), None


# ======================================================================================
# GUI
# ======================================================================================
def run_gui(core):
    import tkinter as tk
    from tkinter import ttk, scrolledtext

    root = tk.Tk()
    root.title("TMS Robot Bridge")
    root.geometry("760x620")
    pad = dict(padx=6, pady=3)

    def light(parent):
        cv = tk.Canvas(parent, width=18, height=18, highlightthickness=0)
        dot = cv.create_oval(2, 2, 16, 16, fill="grey", outline="")
        return cv, dot

    # --- Mac link
    fm = ttk.LabelFrame(root, text="Mac (Polaris navigation terminal)")
    fm.pack(fill="x", **pad)
    mac_cv, mac_dot = light(fm); mac_cv.grid(row=0, column=0, **pad)
    mac_state = ttk.Label(fm, text=""); mac_state.grid(row=0, column=1, columnspan=3, sticky="w", **pad)
    mac_info = ttk.Label(fm, text="", font=("Consolas", 10)); mac_info.grid(row=1, column=1, columnspan=3, sticky="w", **pad)
    mac_delta = ttk.Label(fm, text="", font=("Consolas", 10)); mac_delta.grid(row=2, column=1, columnspan=3, sticky="w", **pad)
    ttk.Button(fm, text="Disconnect Mac", command=core.mac.kick).grid(row=0, column=4, sticky="e", **pad)
    fm.columnconfigure(3, weight=1)

    # --- LabVIEW link
    fr = ttk.LabelFrame(root, text=f"LabVIEW robot ({core.args.robot})")
    fr.pack(fill="x", **pad)
    rob_cv, rob_dot = light(fr); rob_cv.grid(row=0, column=0, **pad)
    rob_state = ttk.Label(fr, text=""); rob_state.grid(row=0, column=1, columnspan=3, sticky="w", **pad)
    ttk.Button(fr, text="Reconnect", command=core.robot.reconnect).grid(row=0, column=4, sticky="e", **pad)
    rob_info = ttk.Label(fr, text="", font=("Consolas", 10)); rob_info.grid(row=1, column=1, columnspan=4, sticky="w", **pad)
    cols = ("x", "y", "z", "psi", "theta", "phi")
    tbl = ttk.Frame(fr); tbl.grid(row=2, column=1, columnspan=4, sticky="w", **pad)
    pos_lbl = {}
    for j, k in enumerate(cols):
        ttk.Label(tbl, text=k, width=9, anchor="e").grid(row=0, column=j)
        pos_lbl[k] = tk.Label(tbl, text="-", width=9, anchor="e", font=("Consolas", 11))
        pos_lbl[k].grid(row=1, column=j)
    opt_lbl = ttk.Label(fr, text="", font=("Consolas", 9)); opt_lbl.grid(row=3, column=1, columnspan=4, sticky="w", **pad)
    fr.columnconfigure(3, weight=1)

    # --- control
    fc = ttk.Frame(root); fc.pack(fill="x", **pad)
    allow = tk.BooleanVar(value=core.allow_motion)
    ttk.Checkbutton(fc, text="Allow motion from Mac (SET / Goto / CALIBRATE)", variable=allow,
                    command=lambda: core.set_allow_motion(allow.get())).pack(side="left")
    ttk.Label(fc, text="STOP from the Mac is " + ("allowed" if core.allow_stop else "blocked")).pack(side="right")

    # --- log
    logw = scrolledtext.ScrolledText(root, height=14, font=("Consolas", 9), state="disabled")
    logw.pack(fill="both", expand=True, **pad)

    def refresh():
        m, r = core.mac, core.robot
        now = time.monotonic()
        # Mac
        if m.client:
            idle = now - m.last_rx
            mac_cv.itemconfig(mac_dot, fill="green" if idle < 5 else "orange")
            mac_info.config(text=f"connected {now - m.since:6.0f} s   idle {idle:4.1f} s (drop at {IDLE_TIMEOUT_S:g})   "
                                 f"requests {sum(m.counts.values())}   last: {m.last_cmd[:40]}")
        else:
            mac_cv.itemconfig(mac_dot, fill="grey" if "listening" in m.state else "red")
            mac_info.config(text="")
        mac_state.config(text=m.state)
        if m.last_delta:
            age = now - m.last_delta_t
            dx, dy, dz, d = m.last_delta
            mac_delta.config(text=f"DELTA (marker - target, mm)  dx {dx:>7}  dy {dy:>7}  dz {dz:>7}  |d| {d:>7}   "
                                  f"{m.delta_rate():.1f} Hz   age {age:5.1f} s")
        # robot
        rob_cv.itemconfig(rob_dot, fill="green" if r.connected else "red")
        rob_state.config(text=r.state + (f"   [{r.greeting}]" if r.connected and r.greeting else ""))
        st = r.status if (r.status_t and now - r.status_t < 3) else {}
        rtt = f"{r.rtt_ms:.1f} ms" if r.rtt_ms is not None else "-"
        rob_info.config(text=f"round trip {rtt}   requests {r.n_req}   errors {r.n_err}" +
                             ("   DELTA not supported by LabVIEW" if r.delta_unsupported else ""))
        mflag = dict(x="mx", y="my", z="mz", psi="mpsi", theta="ml", phi="mr")
        for k in cols:
            if k in st:
                moving = st.get(mflag[k], 0) >= 1
                pos_lbl[k].config(text=g(st[k]), fg="#c06000" if moving else "black")
            else:
                pos_lbl[k].config(text="-", fg="grey")
        if st:
            opt_lbl.config(text="optical  " + "  ".join(f"{k}={g(st[k])}" for k in
                                                       ("ox", "oy", "oz", "opsi", "ol", "or") if k in st)
                                + "      orange = moving")
        # log
        lines = []
        while True:
            try:
                lines.append(core.log_q.get_nowait())
            except queue.Empty:
                break
        if lines:
            logw.config(state="normal")
            logw.insert("1.0", "\n".join(reversed(lines)) + "\n")   # newest on top
            if int(logw.index("end-1c").split(".")[0]) > 2000:
                logw.delete("1500.0", "end")
            logw.config(state="disabled")
        root.after(200, refresh)

    refresh()
    root.mainloop()


SCRIPT_DIR = os.path.dirname(os.path.abspath(sys.executable if getattr(sys, "frozen", False) else __file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen", default="0.0.0.0:6340", help="Mac side (default 0.0.0.0:6340)")
    ap.add_argument("--robot", default="172.22.11.2:6340",
                    help="LabVIEW VI TCP loop host:port (default myRIO 172.22.11.2:6340), or 'sim'")
    ap.add_argument("--robot-proto", choices=("binary", "text"), default="binary",
                    help="LabVIEW interface: binary = Main VI 72B/44B (TMS_MATLAB_Controller), text = protocol v1 lines")
    ap.add_argument("--speed", type=float, default=4.0, help="binary: X/Y/Z/PSI speed sent with every packet (steps/s)")
    ap.add_argument("--curve-speed", type=float, default=4.0, help="binary: THETA/PHI curve speed (steps/s)")
    ap.add_argument("--poll", type=float, default=4.0, help="text: GET STATUS rate to LabVIEW for the monitor (Hz, 0 = off)")
    ap.add_argument("--sim-speed", type=float, default=4.0, help="motor speed of --robot sim (steps/s; 4 = panel default)")
    ap.add_argument("--remote", action="store_true", help="start with 'Allow motion from Mac' on")
    ap.add_argument("--allow-stop", action="store_true", help="forward STOP from the Mac (ends the robot program!)")
    ap.add_argument("--docs", default=None, help="folder with BRIEF.md / PROTOCOL.md / GUIDE.md for GET DOC")
    ap.add_argument("--nogui", action="store_true", help="console only")
    ap.add_argument("-v", "--verbose", action="store_true", help="log every Mac request except DELTA/PING")
    args = ap.parse_args()
    if args.poll > 20:
        ap.error("--poll must be <= 20 Hz (protocol rule)")

    core = Core(args)
    core.log("*", f"TMS Robot Bridge: Mac {args.listen} <-> LabVIEW {args.robot}; motion from Mac "
                  + ("allowed" if core.allow_motion else "blocked"))
    if not core.mac.start():
        if args.nogui:
            sys.exit(1)
    core.robot.start()
    if args.nogui:
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    else:
        run_gui(core)


if __name__ == "__main__":
    main()
