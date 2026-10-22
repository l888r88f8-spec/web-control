"""Persistent storage for web-launched managed process logs.

The live :class:`~web_control.core.TerminalLogBuffer` only keeps the last few
hundred lines in memory so it can serve the incremental realtime API cheaply.
This module writes the same, already ANSI-sanitised lines to a per-target,
per-launch file so the browser can still read a session after a page refresh,
after the process exited, or after the web node itself restarted.

Layout (root defaults to ``~/.local/state/web_control/logs``)::

    <root>/<target>/<YYYYmmdd-HHMMSS>-<session>.log

Only validated target names and session ids can be resolved back to a path, so
a browser can never request an arbitrary file, escape the log root with ``..``
or read through a symlink that points outside it.
"""

from __future__ import annotations

import os
import re
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

DEFAULT_LOG_SUBDIR = ("web_control", "logs")
SESSION_ID_RE = re.compile(r"^[0-9]{8}-[0-9]{6}-[0-9a-f]{8,32}$")
TARGET_RE = re.compile(r"^[A-Za-z0-9_]+$")


class LogStorageError(RuntimeError):
    """Raised when a managed log file cannot be created, written or read."""


def default_managed_log_dir() -> str:
    """Return the default log root, honouring ``XDG_STATE_HOME`` if absolute."""
    xdg_state = (os.environ.get("XDG_STATE_HOME") or "").strip()
    if xdg_state and os.path.isabs(xdg_state):
        return os.path.join(xdg_state, *DEFAULT_LOG_SUBDIR)
    return os.path.join(os.path.expanduser("~"), ".local", "state", *DEFAULT_LOG_SUBDIR)


def resolve_managed_log_dir(value: Any) -> Path:
    """Expand ``~`` in the configured root, falling back to the default."""
    text = str(value or "").strip()
    if not text:
        text = default_managed_log_dir()
    return Path(os.path.expanduser(text))


@dataclass
class LogSession:
    """A single open persistent log file owned by one managed launch."""

    target: str
    session_id: str
    path: Path
    started_at: float
    command: tuple[str, ...]
    _handle: Any = None
    bytes_written: int = 0
    write_error: Optional[str] = None
    _closed: bool = False
    _lock: threading.RLock = field(default_factory=threading.RLock)

    @property
    def closed(self) -> bool:
        return self._closed

    def write_line(self, text: str) -> None:
        """Append one line and flush it, or raise :class:`LogStorageError`."""
        line = text if text.endswith("\n") else text + "\n"
        encoded = line.encode("utf-8", "replace")
        with self._lock:
            if self._closed or self._handle is None:
                return
            try:
                self._handle.write(line)
                self._handle.flush()
            except Exception as exc:  # noqa: BLE001 - surfaced to the browser
                self.write_error = f"log write failed: {exc}"
                self._closed = True
                handle, self._handle = self._handle, None
                try:
                    handle.close()
                except Exception:
                    pass
                raise LogStorageError(self.write_error) from exc
            self.bytes_written += len(encoded)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            handle, self._handle = self._handle, None
            if handle is None:
                return
            try:
                handle.flush()
            except Exception:
                pass
            try:
                handle.close()
            except Exception:
                pass

    def metadata(self) -> dict[str, Any]:
        return {
            "session": self.session_id,
            "started_at": self.started_at,
            "bytes": self.bytes_written,
            "closed": self._closed,
            "error": self.write_error,
            "path": str(self.path),
        }


class ManagedLogStore:
    """Thread-safe, bounded-by-filesystem catalog of managed launch logs."""

    def __init__(self, root: Any, logger: Any = None) -> None:
        self.root = Path(root)
        self.logger = logger
        self._lock = threading.RLock()
        self._sessions: dict[str, list[LogSession]] = {}
        self._root_error: Optional[str] = None
        try:
            self.root.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            # A broken log root must never stop the node from starting; the
            # error is re-raised by start_session so the browser sees it.
            self._root_error = f"log directory unavailable ({self.root}): {exc}"

    # -- session lifecycle -------------------------------------------------
    def start_session(self, target: str, command: list[str]) -> LogSession:
        """Open a new session file for ``target``.

        Every genuinely new launch gets its own file; the process layer already
        refuses to launch while a target is running, so this never creates an
        empty duplicate for an in-flight process. Any still-open previous
        session (for example a slow reader draining an exited process) keeps its
        own handle and cannot be written into the new file.
        """
        with self._lock:
            if self._root_error:
                raise LogStorageError(self._root_error)
            self._validate_target(target)
            started_at = time.time()
            session_id = time.strftime(
                "%Y%m%d-%H%M%S", time.localtime(started_at)
            ) + "-" + uuid.uuid4().hex[:12]
            target_dir = self.root / target
            try:
                target_dir.mkdir(parents=True, exist_ok=True)
                path = target_dir / f"{session_id}.log"
                handle = open(path, "x", encoding="utf-8", errors="replace")
            except OSError as exc:
                raise LogStorageError(
                    f"cannot create log file for {target}: {exc}"
                ) from exc
            session = LogSession(
                target=target,
                session_id=session_id,
                path=path,
                started_at=started_at,
                command=tuple(str(part) for part in command),
                _handle=handle,
            )
            self._sessions.setdefault(target, []).append(session)
            try:
                session.write_line(
                    f"# managed log target={target} session={session_id} "
                    f"started={time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(started_at))}"
                )
                session.write_line("$ " + " ".join(session.command))
            except LogStorageError:
                # The header write failed (for example a full disk); keep the
                # error on the session so attach_session can report it.
                pass
            return session

    def end_session(self, session: Optional[LogSession]) -> None:
        if session is None:
            return
        session.close()
        with self._lock:
            sessions = self._sessions.get(session.target)
            if sessions is not None and session in sessions:
                sessions.remove(session)
                if not sessions:
                    self._sessions.pop(session.target, None)

    # -- enumeration / reading --------------------------------------------
    def list_sessions(self, target: str) -> list[dict[str, Any]]:
        self._validate_target(target)
        target_dir = self.root / target
        root_resolved = self.root.resolve(strict=False)
        try:
            target_resolved = target_dir.resolve(strict=False)
        except OSError as exc:
            raise LogStorageError(f"cannot resolve log target: {exc}") from exc
        if (
            target_resolved != root_resolved
            and root_resolved not in target_resolved.parents
        ):
            # A symlinked target directory must not enumerate files outside root.
            raise ValueError("log target escapes the managed log directory")
        rows: list[dict[str, Any]] = []
        try:
            entries = list(target_dir.iterdir())
        except OSError:
            return rows
        for entry in entries:
            session_id = entry.name[:-4] if entry.name.endswith(".log") else ""
            if not SESSION_ID_RE.fullmatch(session_id):
                continue
            if entry.is_symlink() or not entry.is_file():
                continue
            try:
                stat = entry.stat()
            except OSError:
                continue
            active = self._sessions.get(target, [])
            rows.append(
                {
                    "session": session_id,
                    "started_at": stat.st_mtime,
                    "size": stat.st_size,
                    "modified": stat.st_mtime,
                    "active": any(item.session_id == session_id for item in active),
                }
            )
        rows.sort(key=lambda row: row["started_at"], reverse=True)
        return rows

    def resolve_session_file(self, target: str, session_id: str) -> Path:
        """Resolve a validated (target, session) pair to a file inside root."""
        self._validate_target(target)
        session_id = str(session_id or "")
        if not SESSION_ID_RE.fullmatch(session_id):
            raise ValueError("invalid log session id")
        path = self.root / target / f"{session_id}.log"
        if path.is_symlink():
            raise ValueError("refusing to read a symlinked log file")
        try:
            resolved = path.resolve(strict=False)
        except OSError as exc:
            raise LogStorageError(f"cannot resolve log file: {exc}") from exc
        root_resolved = self.root.resolve(strict=False)
        if resolved != root_resolved and root_resolved not in resolved.parents:
            raise ValueError("log path escapes the managed log directory")
        if not path.is_file():
            raise FileNotFoundError(f"log session {session_id} not found")
        return path

    def read_tail(
        self, target: str, session_id: str, max_bytes: int = 262144
    ) -> tuple[str, bool]:
        """Read at most ``max_bytes`` from the end of a session file."""
        max_bytes = max(1024, int(max_bytes))
        path = self.resolve_session_file(target, session_id)
        try:
            size = path.stat().st_size
            with open(path, "rb") as handle:
                if size > max_bytes:
                    handle.seek(size - max_bytes)
                    # Strictly bounded: read no more than this many bytes even
                    # if the live log keeps growing while we read.
                    data = handle.read(max_bytes)
                    newline = data.find(b"\n")
                    if newline != -1:
                        data = data[newline + 1 :]
                    truncated = True
                else:
                    data = handle.read(size)
                    truncated = False
        except OSError as exc:
            raise LogStorageError(f"cannot read log file: {exc}") from exc
        return data.decode("utf-8", "replace"), truncated

    def display_root(self) -> str:
        return str(self.root)

    # -- helpers -----------------------------------------------------------
    def _validate_target(self, target: str) -> None:
        target = str(target or "")
        if not TARGET_RE.fullmatch(target):
            raise ValueError(f"invalid managed target: {target!r}")

    def close(self) -> None:
        with self._lock:
            sessions = [
                session
                for target_sessions in self._sessions.values()
                for session in target_sessions
            ]
            self._sessions.clear()
        for session in sessions:
            session.close()
