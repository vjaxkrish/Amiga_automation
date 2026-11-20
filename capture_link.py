"""
capture_link.py - the mission asks the camera app to capture, over a local
Unix socket. ONE file, imported by both sides, so the protocol cannot drift
(the antenna-offset bug of 2026-11-11 was two copies of one constant).

    amiga_nav.py mission ... --capture      client: asks for captures
    main_app_final.py (Autonomous run)      server: does them

Newline-delimited JSON. Every message has a "type".

  client -> server
    hello    {protocol, run_id, teach, mode, labels:[...]}
    status   {state, label?, index?, total?, detail?}        no reply
    capture  {id, label, row, col, mode, nav:{...}}
    bye      {result}

  server -> client
    hello_ok {ok, invalid:[...], map, warmup_remaining_s, detail}
    result   {id, status, detail, folder}
             status: ok | quarantined | failed | skipped

The navigation process never blocks on this: it polls the socket from its
20 Hz loop, so the Amiga keeps receiving commands while the camera works -
a blocked read would starve the Dashboard watchdog (~550 ms) and drop it
out of auto.
"""

import json
import os
import queue
import re
import socket
import threading

PROTOCOL = 1
SOCKET_ENV = "TURF_CAPTURE_SOCKET"
DEFAULT_SOCKET = "/tmp/turf_capture.sock"

_PLOT_RE = re.compile(r"^R(\d+)_C(\d+)$")


def socket_path():
    return os.environ.get(SOCKET_ENV) or DEFAULT_SOCKET


def parse_plot_label(label):
    """'R3_C7' -> (3, 7); anything else (headland 'mark_4', 'plot_001') -> None."""
    m = _PLOT_RE.match(str(label or ""))
    return (int(m.group(1)), int(m.group(2))) if m else None


def _encode(msg):
    return (json.dumps(msg, separators=(",", ":"), default=str) + "\n").encode()


class LinkClosed(Exception):
    pass


class CaptureClient:
    """Mission side. connect() blocks briefly; everything after is polled."""

    def __init__(self, path=None):
        self.path = path or socket_path()
        self.sock = None
        self.buf = b""
        self.next_id = 1
        self.closed_why = None

    def connect(self, timeout=2.0):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(timeout)
        try:
            s.connect(self.path)
        except OSError:
            s.close()
            raise
        s.setblocking(False)
        self.sock = s

    def send(self, msg):
        if self.sock is None:
            raise LinkClosed(self.closed_why or "not connected")
        data = _encode(msg)
        try:
            self.sock.setblocking(True)
            self.sock.settimeout(1.0)
            self.sock.sendall(data)
        except OSError as e:
            raise LinkClosed(f"send failed: {e}")
        finally:
            try:
                self.sock.setblocking(False)
            except OSError:
                pass

    def status(self, **kw):
        """Fire-and-forget progress for the app's screen. Never raises."""
        try:
            self.send(dict(type="status", **kw))
        except LinkClosed:
            pass

    def request_capture(self, label, row, col, mode, nav):
        rid = self.next_id
        self.next_id += 1
        self.send({"type": "capture", "id": rid, "label": label, "row": row,
                   "col": col, "mode": mode, "nav": nav})
        return rid

    def poll(self):
        """All complete messages waiting now. Raises LinkClosed if the app
        has gone away."""
        if self.sock is None:
            raise LinkClosed(self.closed_why or "not connected")
        closed = None
        while True:
            try:
                chunk = self.sock.recv(65536)
            except BlockingIOError:
                break
            except OSError as e:
                closed = f"recv failed: {e}"
                break
            if not chunk:
                closed = "camera app closed the connection"
                break
            self.buf += chunk
        if closed:
            # Hand over whatever arrived before the close (a refusal, a last
            # result) - the NEXT poll raises.
            self.close()
            self.closed_why = closed
            if b"\n" not in self.buf:
                raise LinkClosed(closed)
        out = []
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            if line.strip():
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
        return out

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


class CaptureServer:
    """App side. A background thread accepts ONE mission at a time and puts
    every incoming message on `inbox` as (conn_no, msg). The GUI thread
    drains inbox and answers with reply(conn_no, msg). A reply to a mission
    that has since disconnected is dropped silently."""

    def __init__(self, path=None):
        self.path = path or socket_path()
        self.inbox = queue.Queue()
        self._lsock = None
        self._conn = None
        self._conn_no = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass
        ls = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        ls.bind(self.path)
        ls.listen(1)
        ls.settimeout(0.5)
        self._lsock = ls
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        # Accepting continues while a mission is being served, so a second
        # mission is told "another run is active" at once instead of hanging
        # in the backlog.
        while not self._stop.is_set():
            try:
                conn, _ = self._lsock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._lock:
                busy = self._conn is not None
                if not busy:
                    self._conn_no += 1
                    self._conn = conn
                    no = self._conn_no
            if busy:
                try:
                    conn.sendall(_encode({"type": "hello_ok", "ok": False,
                                          "detail": "another run is active"}))
                except OSError:
                    pass
                finally:
                    conn.close()
                continue
            threading.Thread(target=self._session, args=(conn, no),
                             daemon=True).start()

    def _session(self, conn, no):
        self.inbox.put((no, {"type": "_connected"}))
        self._serve(conn, no)
        with self._lock:
            if self._conn is conn:
                self._conn = None
        self.inbox.put((no, {"type": "_disconnected"}))

    def _serve(self, conn, no):
        conn.settimeout(0.5)
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = conn.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.strip():
                    try:
                        self.inbox.put((no, json.loads(line)))
                    except ValueError:
                        pass
        try:
            conn.close()
        except OSError:
            pass

    def reply(self, conn_no, msg):
        with self._lock:
            conn = self._conn if conn_no == self._conn_no else None
        if conn is None:
            return False
        try:
            conn.sendall(_encode(msg))
            return True
        except OSError:
            return False

    def stop(self):
        self._stop.set()
        with self._lock:
            conn, self._conn = self._conn, None
        for s in (conn, self._lsock):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
        try:
            os.unlink(self.path)
        except OSError:
            pass
