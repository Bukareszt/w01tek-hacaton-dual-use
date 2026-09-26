#!/usr/bin/env python3
"""Viewer-side relay for camera_stream.py --push: receives frames over TCP, serves them locally.

Run on the laptop:
    stream_relay.py --listen 0.0.0.0:8096 --http 127.0.0.1:8097
then open http://127.0.0.1:8097/  (browser <img> MJPEG, newest frame only).

Framing from camera_stream.py: 4-byte big-endian length, 8-byte float64 capture time, JPEG.
/status reports approximate capture->arrival delay (depends on clock sync between machines).
"""

import argparse
import json
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HDR = struct.Struct(">Id")


class Store:
    def __init__(self):
        self.jpg = None
        self.seq = 0
        self.cap_ts = 0.0
        self.rx_ts = 0.0
        self.fps = 0.0
        self.source = None
        self.cond = threading.Condition()

    def put(self, jpg, cap_ts):
        with self.cond:
            self.jpg, self.cap_ts, self.rx_ts = jpg, cap_ts, time.time()
            self.seq += 1
            self.cond.notify_all()

    def wait(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout=timeout)
            return self.jpg, self.seq


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(1 << 20, n - len(buf)))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return bytes(buf)


def receiver(store, host, port):
    """Accept pushers; the newest connection wins (older one is closed); dead ones time out."""
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(4)
    print(f"waiting for pusher on {host}:{port}", flush=True)
    current = {"conn": None}

    def serve(conn, addr):
        name = f"{addr[0]}:{addr[1]}"
        store.source = name
        n, t0 = 0, time.time()
        try:
            while True:
                length, cap_ts = HDR.unpack(recv_exact(conn, HDR.size))
                store.put(recv_exact(conn, length), cap_ts)
                n += 1
                if n % 60 == 0:
                    store.fps = 60 / (time.time() - t0)
                    t0 = time.time()
        except (OSError, ConnectionError) as e:
            print(f"pusher {name} disconnected: {e}", flush=True)
            if store.source == name:
                store.source = None
        finally:
            conn.close()

    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(10)                      # a silent pusher is a dead pusher
        old = current["conn"]
        if old is not None:
            try:
                old.close()                      # unblocks the old thread with an error
            except OSError:
                pass
        current["conn"] = conn
        print(f"pusher connected from {addr[0]}:{addr[1]}", flush=True)
        threading.Thread(target=serve, args=(conn, addr), daemon=True).start()


PAGE = b"""<!doctype html><html><head><title>fast cam</title><style>body{margin:0;background:#000;
display:flex;justify-content:center;align-items:center;height:100vh}img{max-width:100%;max-height:100%}
</style></head><body><img src="/stream"></body></html>"""


def make_handler(store):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def setup(self):
            super().setup()
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/":
                self._send(200, "text/html", PAGE)
            elif p == "/status":
                self._send(200, "application/json", json.dumps({
                    "source": store.source, "fps": round(store.fps, 1), "frames": store.seq,
                    "age_s": round(time.time() - store.rx_ts, 3) if store.rx_ts else None,
                    "net_delay_s_approx": round(store.rx_ts - store.cap_ts, 3) if store.rx_ts else None,
                }).encode())
            elif p == "/snap":
                jpg, _ = store.wait(-1)
                self._send(200 if jpg else 503, "image/jpeg", jpg or b"")
            elif p == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache, no-store")
                self.send_header("Connection", "close")
                self.end_headers()
                seq = store.seq
                try:
                    while True:
                        jpg, seq = store.wait(seq)
                        if jpg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._send(404, "text/plain", b"")

    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen", default="0.0.0.0:8096", help="where camera_stream.py --push connects")
    ap.add_argument("--http", default="127.0.0.1:8097", help="local HTTP endpoint for the browser")
    a = ap.parse_args()
    store = Store()
    lh, lp = a.listen.rsplit(":", 1)
    threading.Thread(target=receiver, args=(store, lh, int(lp)), daemon=True).start()
    hh, hp = a.http.rsplit(":", 1)
    srv = ThreadingHTTPServer((hh, int(hp)), make_handler(store))
    srv.daemon_threads = True
    print(f"open http://{hh}:{hp}/", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
