"""TMS Robot TCP client (PROTOCOL.md v1). Python 3.8+, standard library only.

Library use:
    from tms_client import TMSClient
    with TMSClient("192.168.50.149") as r:
        r.set("THETA", -60)
        print(r.status())

CLI:
    python tms_client.py --host 192.168.50.149 ping
    python tms_client.py --host 192.168.50.149 status
    python tms_client.py --host 192.168.50.149 set THETA -60 --wait
    python tms_client.py --host 192.168.50.149 shell        # interactive raw commands
"""
import argparse
import socket
import time

PROTO_VERSION = 1


class TMSError(Exception):
    pass


class TMSClient:
    def __init__(self, host, port=6340, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.f = self.sock.makefile("rwb", buffering=0)
        hello = self._readline()
        if hello.startswith("ERR"):
            raise TMSError(hello)
        parts = hello.split()
        if parts[:2] != ["HELLO", "TMS-ROBOT"] or int(parts[2]) != PROTO_VERSION:
            raise TMSError(f"unexpected greeting: {hello!r}")

    def _readline(self):
        raw = self.f.readline(513)
        if not raw:
            raise TMSError("connection closed by robot")
        return raw.decode("ascii", "replace").rstrip("\r\n")

    def cmd(self, line):
        self.f.write((line + "\r\n").encode("ascii"))
        resp = self._readline()
        if resp.startswith("ERR"):
            raise TMSError(resp)
        return resp

    def ping(self):
        return self.cmd("PING")

    def set(self, axis, value):
        return self.cmd(f"SET {axis.upper()} {value:g}")

    def status(self):
        resp = self.cmd("GET STATUS")
        body = resp.split(" ", 1)[1]
        out = {}
        for kv in body.split(";"):
            k, v = kv.split("=", 1)
            out[k] = float(v) if any(c in v for c in ".eE") else int(v)
        return out

    def wait_idle(self, timeout=60.0, poll=0.1):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            s = self.status()
            if not any(s[k] for k in ("mx", "my", "mz", "mpsi", "ml", "mr")):
                return s
            time.sleep(poll)
        raise TMSError("timeout waiting for motion to finish")

    def close(self):
        try:
            self.cmd("BYE")
        except (TMSError, OSError):
            pass
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=6340)
    sub = ap.add_subparsers(dest="op", required=True)
    sub.add_parser("ping")
    sub.add_parser("status")
    s = sub.add_parser("set")
    s.add_argument("axis")
    s.add_argument("value", type=float)
    s.add_argument("--wait", action="store_true")
    sub.add_parser("shell")
    a = ap.parse_args()

    with TMSClient(a.host, a.port) as r:
        if a.op == "ping":
            print(r.ping())
        elif a.op == "status":
            for k, v in r.status().items():
                print(f"{k:>6} = {v}")
        elif a.op == "set":
            print(r.set(a.axis, a.value))
            if a.wait:
                print(r.wait_idle())
        elif a.op == "shell":
            print("raw protocol shell; Ctrl-D to quit")
            while True:
                try:
                    line = input("> ")
                except EOFError:
                    break
                try:
                    print(r.cmd(line))
                except TMSError as e:
                    print(e)


if __name__ == "__main__":
    main()
