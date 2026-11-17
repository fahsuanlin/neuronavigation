"""TMS Robot simulator: a stand-in for the LabVIEW/myRIO TCP server (PROTOCOL.md v1).

Run it on the Windows PC so the Mac client can be developed and tested before the LabVIEW
TCP loop exists:
    python robot_sim_server.py --host 0.0.0.0 --port 6340
"""
import argparse
import socket
import threading
import time

PROTO_VERSION = 1
AXES = {  # protocol name -> (initial value, simulated speed in units/s)
    "X": (0.0, 50.0), "Y": (0.0, 50.0), "Z": (0.0, 50.0), "PSI": (0.0, 50.0),
    "THETA": (-85.0, 20.0), "PHI": (0.0, 20.0),
    "L": (0.0, 100.0), "R": (0.0, 100.0),
}
RELATIVE = {"L", "R"}  # Motor Input L/R are relative steps
HELP_LINE = (
    "OK HELP protocol v1, ASCII lines ending CRLF, one reply per request | "
    "PING -> OK PONG | GET STATUS -> STATUS k=v;k=v... | "
    "SET <X|Y|Z|PSI|THETA|PHI> <value> (absolute target) | SET <L|R> <steps> (relative) | "
    "STOP (ends robot program) | BYE (close connection) | HELP"
)


class Robot:
    def __init__(self):
        self.lock = threading.Lock()
        self.pos = {a: v for a, (v, _) in AXES.items()}
        self.target = dict(self.pos)
        self.t0 = time.monotonic()
        self.running = True
        self.last = time.monotonic()

    def set(self, axis, value):
        with self.lock:
            self.target[axis] = self.pos[axis] + value if axis in RELATIVE else value

    def step(self):
        now = time.monotonic()
        with self.lock:
            dt, self.last = now - self.last, now
            for a, (_, speed) in AXES.items():
                d = self.target[a] - self.pos[a]
                lim = speed * dt
                self.pos[a] += max(-lim, min(lim, d))

    def status(self):
        self.step()
        with self.lock:
            p, t = self.pos, self.target
            mv = {a: int(abs(t[a] - p[a]) > 1e-6) for a in AXES}
            f = [
                ("t", int((time.monotonic() - self.t0) * 1000)),
                ("x", round(p["X"], 3)), ("y", round(p["Y"], 3)), ("z", round(p["Z"], 3)),
                ("psi", round(p["PSI"], 3)), ("theta", round(p["THETA"], 3)), ("phi", round(p["PHI"], 3)),
                ("mx", mv["X"]), ("my", mv["Y"]), ("mz", mv["Z"]), ("mpsi", mv["PSI"]),
                ("ml", mv["L"]), ("mr", mv["R"]),
                ("ox", 0.12), ("oy", 0.10), ("oz", 0.11), ("opsi", 0.09), ("ol", 0.13), ("or", 0.12),
            ]
        return "STATUS " + ";".join(f"{k}={v}" for k, v in f)


def handle(line, robot):
    """Return (response, close_connection)."""
    parts = line.strip().split()
    if not parts:
        return "ERR BAD_ARGS empty line", False
    cmd = parts[0].upper()
    if cmd in ("HELP", "PROTOCOL", "?"):
        return HELP_LINE, False
    if cmd in ("HELLO", "HELLOW", "HI"):
        return f"OK HELLO TMS-ROBOT {PROTO_VERSION} (send HELP for commands)", False
    if cmd == "PING":
        return "OK PONG", False
    if cmd == "GET" and len(parts) == 2 and parts[1].upper() == "STATUS":
        return robot.status(), False
    if cmd == "SET":
        if len(parts) != 3:
            return "ERR BAD_ARGS usage: SET <AXIS> <value>", False
        axis = parts[1].upper()
        if axis not in AXES:
            return f"ERR BAD_AXIS {axis}", False
        try:
            val = float(parts[2])
        except ValueError:
            return f"ERR BAD_ARGS not a number: {parts[2]}", False
        robot.set(axis, val)
        return f"OK SET {axis} {parts[2]}", False
    if cmd == "STOP":
        robot.running = False
        return "OK STOP", True
    if cmd == "BYE":
        return "OK BYE", True
    return f"ERR UNKNOWN_CMD {cmd} (send HELP for the command list)", False


def serve_client(conn, addr, robot, busy):
    print(f"client connected: {addr}")
    conn.settimeout(10.0)  # idle timeout from PROTOCOL.md
    f = conn.makefile("rwb", buffering=0)
    try:
        f.write(f"HELLO TMS-ROBOT {PROTO_VERSION}\r\n".encode())
        while robot.running:
            raw = f.readline(513)
            if not raw:
                break
            req = raw.decode("ascii", "replace")
            resp, close = handle(req, robot)
            print(f"{addr[0]} >> {req.strip()!r}   << {resp}", flush=True)
            f.write((resp + "\r\n").encode())
            if close:
                break
    except (socket.timeout, ConnectionError) as e:
        print(f"client {addr} dropped: {e!r}")
    finally:
        conn.close()
        busy.release()
        print(f"client disconnected: {addr}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=6340)
    a = ap.parse_args()
    robot, busy = Robot(), threading.Semaphore(1)
    srv = socket.create_server((a.host, a.port))
    srv.settimeout(0.5)
    print(f"TMS robot simulator listening on {a.host}:{a.port}")
    while robot.running:
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        if not busy.acquire(blocking=False):
            conn.sendall(b"ERR BUSY another client connected\r\n")
            conn.close()
            continue
        threading.Thread(target=serve_client, args=(conn, addr, robot, busy), daemon=True).start()
    print("STOP received - simulator exiting")


if __name__ == "__main__":
    main()
