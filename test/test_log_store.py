"""Tests for persistent managed-launch log storage."""

import os
import time
import urllib.error
import urllib.request
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest

from web_control import core, web_control_node
from web_control.core import TerminalLogBuffer
from web_control.log_store import (
    LogStorageError,
    ManagedLogStore,
    default_managed_log_dir,
    resolve_managed_log_dir,
)


def test_default_log_dir_honours_xdg_state_home(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    assert default_managed_log_dir() == str(tmp_path / "state" / "web_control" / "logs")
    monkeypatch.setenv("XDG_STATE_HOME", "relative")
    assert default_managed_log_dir().endswith(".local/state/web_control/logs")
    monkeypatch.delenv("XDG_STATE_HOME", raising=False)
    assert resolve_managed_log_dir("~/custom/logs") == Path.home() / "custom" / "logs"


def test_session_file_records_command_and_streams_lines(tmp_path):
    store = ManagedLogStore(tmp_path / "logs")
    session = store.start_session("chassis", ["ros2", "launch", "hunter_base", "x.py"])
    assert session.path.parent.name == "chassis"
    session.write_line("hello live")

    # Data is flushed synchronously and readable while the session is open.
    content = session.path.read_text(encoding="utf-8")
    assert "$ ros2 launch hunter_base x.py" in content
    assert "hello live" in content

    sessions = store.list_sessions("chassis")
    assert [row["session"] for row in sessions] == [session.session_id]
    assert sessions[0]["active"] is True
    store.end_session(session)
    assert store.list_sessions("chassis")[0]["active"] is False


def test_new_service_process_can_list_and_read_old_sessions(tmp_path):
    root = tmp_path / "logs"
    first = ManagedLogStore(root)
    session = first.start_session("imu", ["ros2", "launch", "imu", "x.py"])
    session.write_line("persisted line")
    first.end_session(session)
    first.close()

    # A brand new store instance (simulating a restarted web node) sees it.
    second = ManagedLogStore(root)
    sessions = second.list_sessions("imu")
    assert len(sessions) == 1
    text, truncated = second.read_tail("imu", sessions[0]["session"])
    assert "persisted line" in text
    assert truncated is False


def test_restart_creates_distinct_files_and_old_reader_stays_isolated(tmp_path):
    store = ManagedLogStore(tmp_path / "logs")
    old = store.start_session("navigation", ["ros2", "launch", "nav", "x.py"])
    new = store.start_session("navigation", ["ros2", "launch", "nav", "x.py"])
    assert old.path != new.path
    assert old.session_id != new.session_id

    # A stale reader writes through its own session, never the new file.
    store._sessions["navigation"]  # noqa: SLF001 - intentional state probe
    old.write_line("old reader line")
    new.write_line("new launch line")
    assert "old reader line" in old.path.read_text(encoding="utf-8")
    assert "old reader line" not in new.path.read_text(encoding="utf-8")


def test_read_tail_is_bounded_and_drops_partial_first_line(tmp_path):
    store = ManagedLogStore(tmp_path / "logs")
    session = store.start_session("lidar", ["ros2", "launch", "lidar", "x.py"])
    for index in range(5000):
        session.write_line(f"line-{index:05d}-{'x' * 40}")
    store.end_session(session)

    text, truncated = store.read_tail("lidar", session.session_id, max_bytes=4096)
    assert truncated is True
    assert len(text.encode("utf-8")) <= 4096
    assert text.splitlines()[0].startswith("line-")
    assert text.splitlines()[-1].startswith("line-04999")


def test_resolve_rejects_traversal_bad_ids_and_symlinks(tmp_path):
    store = ManagedLogStore(tmp_path / "logs")
    with pytest.raises(ValueError):
        store.resolve_session_file("chassis", "../../etc/passwd")
    with pytest.raises(ValueError):
        store.resolve_session_file("chassis", "not-a-session")
    with pytest.raises(ValueError):
        store.list_sessions("../etc")

    outside = tmp_path / "outside.log"
    outside.write_text("secret", encoding="utf-8")
    target_dir = tmp_path / "logs" / "chassis"
    target_dir.mkdir(parents=True, exist_ok=True)
    link_name = "20260101-000000-abcdef123456"
    (target_dir / f"{link_name}.log").symlink_to(outside)
    with pytest.raises(ValueError):
        store.resolve_session_file("chassis", link_name)


def test_unavailable_root_reports_error_without_crashing(tmp_path):
    root_file = tmp_path / "not-a-dir"
    root_file.write_text("x", encoding="utf-8")
    store = ManagedLogStore(root_file)
    with pytest.raises(LogStorageError):
        store.start_session("chassis", ["ros2", "launch", "x", "y"])


def test_terminal_log_buffer_persists_and_surfaces_write_failure(tmp_path):
    store = ManagedLogStore(tmp_path / "logs")
    logs = TerminalLogBuffer("mapping", max_lines=3)
    session = store.start_session("mapping", ["ros2", "launch", "map", "x.py"])
    logs.attach_session(session)
    logs.append("\x1b[31mred line\x1b[0m")
    assert "red line" in session.path.read_text(encoding="utf-8")
    assert "\x1b" not in session.path.read_text(encoding="utf-8")

    # A failure is visible and does not stop the in-memory buffer.
    class _BrokenSession:
        closed = False
        write_error = None

        def write_line(self, text):
            raise LogStorageError("disk full")

    logs.attach_session(_BrokenSession())
    logs.append("still in memory")
    assert logs.persist_error == "disk full"
    assert logs.snapshot()["lines"][-1] == "still in memory"


def test_managed_process_begin_session_and_failure_surface(tmp_path):
    from web_control.web_control_node import ManagedLaunchProcess

    store = ManagedLogStore(tmp_path / "logs")
    logs = TerminalLogBuffer("chassis", max_lines=10)
    process = ManagedLaunchProcess(
        "chassis",
        ["ros2", "launch", "hunter_base", "x.py"],
        logs,
        SimpleNamespace(error=lambda *_a, **_k: None),
        log_store=store,
    )
    with process.lock:
        process._begin_session_locked()
    assert logs.session_info()["session"]
    process.log_buffer.append("live")
    assert "live" in process.session.path.read_text(encoding="utf-8")

    broken_root = tmp_path / "file-root"
    broken_root.write_text("x", encoding="utf-8")
    broken = ManagedLogStore(broken_root)
    logs2 = TerminalLogBuffer("imu", max_lines=10)
    process2 = ManagedLaunchProcess(
        "imu",
        ["ros2", "launch", "imu", "x.py"],
        logs2,
        SimpleNamespace(error=lambda *_a, **_k: None),
        log_store=broken,
    )
    with process2.lock:
        process2._begin_session_locked()
    assert process2.session is None
    assert "persistent log unavailable" in logs2.persist_error


def _log_node(tmp_path):
    node = SimpleNamespace()
    node.managed_logs = {"chassis": TerminalLogBuffer("chassis", max_lines=5)}
    node.log_store = ManagedLogStore(tmp_path / "logs")
    node.now_sec = lambda: 123.0
    node.get_logger = lambda: SimpleNamespace(
        debug=lambda *_a, **_k: None, error=lambda *_a, **_k: None
    )
    node.static_dir = tmp_path / "static"
    node.static_dir.mkdir()
    for name in (
        "_require_log_target",
        "saved_logs_payload",
        "saved_log_tail",
        "saved_log_download_path",
    ):
        setattr(
            node, name, MethodType(getattr(web_control_node.WebControlNode, name), node)
        )
    return node


def _serve(node):
    from http.server import ThreadingHTTPServer
    import threading

    handler = web_control_node.WebControlNode.make_handler(node)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def test_saved_log_endpoints_serve_tail_and_download(tmp_path):
    node = _log_node(tmp_path)
    session = node.log_store.start_session("chassis", ["ros2", "launch", "x", "y"])
    session.write_line("downloaded payload")
    node.log_store.end_session(session)
    node.managed_logs["chassis"].set_persist_error("disk full")

    server, thread = _serve(node)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urllib.request.urlopen(
            f"{base}/api/logs/sessions?target=chassis", timeout=10
        ) as response:
            body = response.read().decode("utf-8")
        assert session.session_id in body
        assert '"persist_error":"disk full"' in body

        with urllib.request.urlopen(
            f"{base}/api/logs/tail?target=chassis&session={session.session_id}",
            timeout=10,
        ) as response:
            assert "downloaded payload" in response.read().decode("utf-8")

        with urllib.request.urlopen(
            f"{base}/api/logs/download?target=chassis&session={session.session_id}",
            timeout=10,
        ) as response:
            assert "downloaded payload" in response.read().decode("utf-8")
            assert "attachment" in response.headers.get("Content-Disposition", "")

        with pytest.raises(urllib.error.HTTPError) as excinfo:
            urllib.request.urlopen(
                f"{base}/api/logs/tail?target=chassis&session=../../etc/passwd",
                timeout=10,
            )
        assert excinfo.value.code == 400

        with pytest.raises(urllib.error.HTTPError) as excinfo:
            urllib.request.urlopen(
                f"{base}/api/logs/sessions?target=reactor", timeout=10
            )
        assert excinfo.value.code == 400
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def test_read_tail_is_bounded_when_file_grows_during_read(tmp_path, monkeypatch):
    store = ManagedLogStore(tmp_path / "logs")
    session = store.start_session("lidar", ["ros2", "launch", "lidar", "x.py"])
    for _ in range(200):
        session.write_line("seed-line")
    store.end_session(session)
    real_open = open
    target = str(session.path)

    def growing_open(path, mode="r", *args, **kwargs):
        if str(path) == target and "r" in mode:
            with real_open(path, "ab") as handle:
                handle.write(b"grown-after-snapshot\n" * 20000)
        return real_open(path, mode, *args, **kwargs)

    monkeypatch.setattr("web_control.log_store.open", growing_open, raising=False)
    text, truncated = store.read_tail("lidar", session.session_id, max_bytes=4096)

    assert len(text.encode("utf-8")) <= 4096
    assert "grown-after-snapshot" not in text


def test_list_sessions_rejects_target_symlink_escaping_root(tmp_path):
    store = ManagedLogStore(tmp_path / "logs")
    outside = tmp_path / "outside-logs"
    outside.mkdir()
    (outside / "20260101-000000-abcdef123456.log").write_text(
        "secret", encoding="utf-8"
    )
    (store.root / "chassis").symlink_to(outside, target_is_directory=True)

    with pytest.raises(ValueError):
        store.list_sessions("chassis")


def test_log_download_streams_exactly_snapshot_size_when_file_grows(
    tmp_path, monkeypatch
):
    import socket

    node = _log_node(tmp_path)
    session = node.log_store.start_session("chassis", ["ros2", "launch", "x", "y"])
    session.write_line("initial payload line")
    snapshot_size = session.path.stat().st_size
    real_open = open
    target = str(session.path)

    class _GrowingHandle:
        def __init__(self, handle):
            self._handle = handle
            self._grew = False

        def fileno(self):
            return self._handle.fileno()

        def read(self, size=-1):
            if not self._grew:
                self._grew = True
                with real_open(target, "ab") as extra:
                    extra.write(b"appended-after-open\n" * 20000)
            return self._handle.read(size)

        def close(self):
            self._handle.close()

        def __enter__(self):
            return self

        def __exit__(self, *_exc):
            self.close()

    def growing_open(path, mode="r", *args, **kwargs):
        handle = real_open(path, mode, *args, **kwargs)
        if str(path) == target and "b" in str(mode):
            return _GrowingHandle(handle)
        return handle

    monkeypatch.setattr(web_control_node, "open", growing_open, raising=False)

    server, thread = _serve(node)
    try:
        sock = socket.create_connection(server.server_address, timeout=10)
        request = (
            f"GET /api/logs/download?target=chassis&session={session.session_id} "
            "HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n"
        )
        sock.sendall(request.encode("ascii"))
        chunks = []
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
        sock.close()
        raw = b"".join(chunks)
        head, _sep, body = raw.partition(b"\r\n\r\n")
        assert b"Content-Length: %d" % snapshot_size in head
        assert len(body) == snapshot_size
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
