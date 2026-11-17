"""Forward a TCP port on this Windows PC to the myRIO.

Use this when the myRIO is reachable only over its USB link (172.22.11.2), which the Mac cannot
see. The Mac connects to the PC's LAN IP, and this script relays the bytes to the myRIO.

    python tcp_bridge.py --listen 0.0.0.0:6340 --target 172.22.11.2:6340

You do not need it if the myRIO is on the same LAN or Wi-Fi as the Mac. In that case, connect
straight to the myRIO's LAN IP.
"""
import argparse
import socket
import threading


def pipe(src, dst):
    try:
        while True:
            data = src.recv(4096)
            if not data:
                break
            dst.sendall(data)
    except OSError:
        pass
    finally:
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def hostport(s):
    h, p = s.rsplit(":", 1)
    return h, int(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="0.0.0.0:6340")
    ap.add_argument("--target", default="172.22.11.2:6340")
    a = ap.parse_args()
    srv = socket.create_server(hostport(a.listen))
    print(f"bridge {a.listen} -> {a.target}")
    while True:
        c, addr = srv.accept()
        try:
            t = socket.create_connection(hostport(a.target), timeout=5)
            t.settimeout(None)
        except OSError as e:
            print(f"{addr}: cannot reach target: {e}")
            c.sendall(b"ERR BUSY robot unreachable from bridge\r\n")
            c.close()
            continue
        print(f"{addr} connected")
        threading.Thread(target=pipe, args=(c, t), daemon=True).start()
        threading.Thread(target=pipe, args=(t, c), daemon=True).start()


if __name__ == "__main__":
    main()
