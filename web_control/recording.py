"""ROS-free helpers for authorized rosbag recording from the web UI."""

from __future__ import annotations

import os
import re
import shutil
import signal
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Iterable, Sequence

NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
MAX_TOPICS = 64
LOG_TAIL_LINES = 80
SIGINT_TIMEOUT = 8.0
SIGTERM_TIMEOUT = 2.0
SIGKILL_TIMEOUT = 1.0
READER_JOIN_TIMEOUT = 1.0


class RecordingError(RuntimeError):
    """Base error for recording requests."""


class RecordingBusyError(RecordingError):
    """A recording is already running or finalizing."""


class RecordingConflictError(RecordingError):
    """The requested output already exists."""


class RecordingUnavailableError(RecordingError):
    """rosbag2 or the recorder process is unavailable."""


def _root_path(root: str | os.PathLike[str]) -> Path:
    value = str(root or "").strip()
    if not value or not os.path.isabs(value):
        raise ValueError("recording root must be an absolute directory")
    resolved = Path(value).resolve()
    if not resolved.is_dir():
        raise ValueError("recording root does not exist")
    return resolved


def resolve_recording_output(
    directory: str, name: str, root: str | os.PathLike[str]
) -> Path:
    """Validate an absolute robot directory and return its new bag path.

    The directory must already exist, contain no ``..`` components, resolve
    (through symlinks) inside the configured root, and be writable. The bag
    folder name is created by rosbag2, so the target must not exist yet.
    """
    if not isinstance(directory, str):
        raise ValueError("save directory must be a string path")
    directory = directory.strip()
    if not directory:
        raise ValueError("save directory is required")
    if not os.path.isabs(directory):
        raise ValueError("save directory must be an absolute path")
    if ".." in Path(directory).parts:
        raise ValueError("save directory must not contain '..' path components")
    if not isinstance(name, str):
        raise ValueError("bag name must be a string")
    name = name.strip()
    if not NAME_RE.fullmatch(name):
        raise ValueError(
            "bag name must start with a letter or number and use only "
            "letters, numbers, dot, underscore or dash"
        )
    root_path = _root_path(root)
    requested = Path(directory)
    if not requested.exists():
        raise ValueError("save directory does not exist")
    if not requested.is_dir():
        raise ValueError("save path is not a directory")
    resolved = requested.resolve()
    try:
        resolved.relative_to(root_path)
    except ValueError as exc:
        raise ValueError("save directory is outside the allowed recording root") from exc
    if not os.access(resolved, os.W_OK):
        raise ValueError("save directory is not writable")
    target = resolved / name
    if os.path.lexists(target):
        raise RecordingConflictError(f"a bag already exists at {target}")
    return target


def validate_topic_selection(
    payload: dict[str, Any], available: dict[str, Sequence[str]]
) -> list[str]:
    """Validate a user-selected topic list against the live ROS graph."""
    raw = payload.get("topics") if isinstance(payload, dict) else None
    if not isinstance(raw, list):
        raise ValueError("select at least one topic to record")
    if not raw:
        raise ValueError("select at least one topic to record")
    if len(raw) > MAX_TOPICS:
        raise ValueError(f"select no more than {MAX_TOPICS} topics")
    topics: list[str] = []
    seen: set[str] = set()
    graph = set(available)
    for value in raw:
        if not isinstance(value, str) or not value.strip():
            raise ValueError("topic names must be non-empty strings")
        topic = value.strip()
        if topic in seen:
            raise ValueError(f"duplicate topic: {topic}")
        if topic not in graph:
            raise ValueError(f"topic is not available: {topic}")
        seen.add(topic)
        topics.append(topic)
    return topics


def browse_recording_directories(
    path: str, root: str | os.PathLike[str]
) -> dict[str, Any]:
    """Return immediate subdirectories for the backend directory browser."""
    root_path = _root_path(root)
    if not isinstance(path, str):
        raise ValueError("browse path must be a string")
    path = path.strip()
    requested = root_path if not path else Path(path)
    if not os.path.isabs(requested):
        raise ValueError("browse path must be absolute")
    if ".." in requested.parts:
        raise ValueError("browse path must not contain '..' path components")
    if not requested.exists() or not requested.is_dir():
        raise ValueError("browse path is not a directory")
    current = requested.resolve()
    try:
        current.relative_to(root_path)
    except ValueError as exc:
        raise ValueError("browse path is outside the allowed recording root") from exc
    try:
        children = list(current.iterdir())
    except PermissionError as exc:
        raise ValueError("directory is not readable") from exc
    entries = []
    for child in children:
        try:
            if not child.is_dir():
                continue
        except OSError:
            continue
        entries.append(
            {
                "name": child.name,
                "path": str(child),
                "writable": os.access(child, os.W_OK),
            }
        )
    entries.sort(key=lambda item: item["name"].casefold())
    parent = None if current == root_path else str(current.parent)
    return {
        "path": str(current),
        "parent": parent,
        "writable": os.access(current, os.W_OK),
        "entries": entries,
    }


class RosbagRecorder:
    """Own one ``ros2 bag record`` child process and its lifecycle state."""

    def __init__(
        self,
        root: str | os.PathLike[str] = "/",
        *,
        command: Sequence[str] | None = None,
    ) -> None:
        self.root = _root_path(root)
        self.command = list(command or ("ros2", "bag", "record"))
        self._lock = threading.RLock()
        self._stop_lock = threading.Lock()
        self._process: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None
        self._pgid: int | None = None
        self._log: deque[str] = deque(maxlen=LOG_TAIL_LINES)
        self._state = "idle"
        self._topics: list[str] = []
        self._output: Path | None = None
        self._message = "No recording in progress."
        self._returncode: int | None = None
        self._started_at: float | None = None
        self._finished_at: float | None = None
        self._stop_requested = False
        self._escalated = False
        self._finalized = False

    def available(self) -> bool:
        if not self.command:
            return False
        return shutil.which(self.command[0]) is not None

    def start(
        self, topics: Iterable[str], output_dir: Path, now: float | None = None
    ) -> dict[str, Any]:
        topics = list(topics)
        output = Path(output_dir)
        with self._lock:
            if self._state in ("recording", "stopping"):
                raise RecordingBusyError("a recording is already in progress")
            if os.path.lexists(output):
                raise RecordingConflictError(f"a bag already exists at {output}")
            if not self.available():
                message = "ros2 bag command is unavailable"
                self._state = "failed"
                self._message = message
                self._topics = topics
                self._output = output
                self._returncode = None
                self._started_at = now
                self._finished_at = time.time()
                raise RecordingUnavailableError(message)
            argv = [*self.command, "-o", str(output), *topics]
            try:
                process = subprocess.Popen(
                    argv,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    errors="replace",
                    bufsize=1,
                    start_new_session=True,
                )
            except (OSError, ValueError) as exc:
                message = f"failed to start rosbag recording: {exc}"
                self._state = "failed"
                self._message = message
                self._topics = topics
                self._output = output
                self._returncode = None
                self._started_at = now
                self._finished_at = time.time()
                raise RecordingUnavailableError(message) from exc
            self._process = process
            # ``start_new_session`` makes the child a session and process-group
            # leader, so its pid is the only group this recorder may signal.
            self._pgid = process.pid
            self._reader = threading.Thread(
                target=self._drain,
                args=(process,),
                name="rosbag-recorder-reader",
                daemon=True,
            )
            self._log.clear()
            self._state = "recording"
            self._topics = topics
            self._output = output
            self._message = f"Recording {len(topics)} topics."
            self._returncode = None
            self._started_at = now
            self._finished_at = None
            self._stop_requested = False
            self._escalated = False
            self._finalized = False
            self._reader.start()
        return self.status(now)

    def _drain(self, process: subprocess.Popen[str]) -> None:
        stream = process.stdout
        try:
            if stream is not None:
                try:
                    for line in iter(stream.readline, ""):
                        text = line.rstrip()
                        if text:
                            with self._lock:
                                self._log.append(text)
                except (OSError, ValueError, UnicodeError):
                    # A decoding/read failure must not strand the recorder in
                    # recording/stopping; wait for the child and finalize below.
                    pass
            try:
                returncode = process.wait()
            except Exception:
                returncode = process.poll()
            if returncode is not None:
                self._finalize(process, returncode)
        finally:
            if stream is not None:
                try:
                    stream.close()
                except (OSError, ValueError):
                    pass

    def _finalize(
        self, process: subprocess.Popen[str], returncode: int | None
    ) -> None:
        # Never publish a terminal state before the child has actually exited.
        # The reader thread calls this after its drain loop, and stop() calls it
        # only after both the process exit and the reader drain are complete.
        if process.poll() is None:
            return
        with self._lock:
            if self._process is not process or self._finalized:
                return
            self._finalized = True
            output = self._output
            metadata_ok = False
            if output is not None:
                try:
                    metadata = output / "metadata.yaml"
                    metadata_ok = metadata.is_file() and metadata.stat().st_size > 0
                except OSError:
                    metadata_ok = False
            self._returncode = returncode
            self._finished_at = time.time()
            stopped = self._stop_requested
            failed = False
            if stopped and not self._escalated and metadata_ok and returncode in (0, -2, 130):
                self._state = "stopped"
                self._message = f"Recording stopped; bag finalized at {output}."
            elif stopped:
                self._state = "failed"
                failed = True
                if not metadata_ok:
                    self._message = "Recording stopped but metadata.yaml was not finalized."
                else:
                    self._message = "Recording required forced termination."
            elif returncode != 0:
                self._state = "failed"
                failed = True
                self._message = f"rosbag recording exited with code {returncode}."
            elif not metadata_ok:
                self._state = "failed"
                failed = True
                self._message = "rosbag recording exited without metadata.yaml."
            else:
                self._state = "stopped"
                self._message = f"Recording finished; bag saved at {output}."
            if failed:
                tail = list(self._log)[-5:]
                if tail:
                    self._message += " " + " | ".join(tail)

    @staticmethod
    def _signal_group(pgid: int | None, sig: int) -> bool:
        if pgid is None:
            return False
        try:
            os.killpg(pgid, sig)
            return True
        except (ProcessLookupError, PermissionError):
            return False

    @staticmethod
    def _wait_exit(process: subprocess.Popen[str], timeout: float) -> bool:
        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False
        except Exception:
            return process.poll() is not None

    def _terminate(self, process: subprocess.Popen[str], pgid: int | None) -> None:
        if process.poll() is not None:
            return
        self._signal_group(pgid, signal.SIGINT)
        if self._wait_exit(process, SIGINT_TIMEOUT):
            return
        if process.poll() is not None:
            return
        with self._lock:
            self._escalated = True
        self._signal_group(pgid, signal.SIGTERM)
        if self._wait_exit(process, SIGTERM_TIMEOUT):
            return
        if process.poll() is not None:
            return
        self._signal_group(pgid, signal.SIGKILL)
        self._wait_exit(process, SIGKILL_TIMEOUT)

    def stop(self, now: float | None = None) -> dict[str, Any]:
        # A dedicated lock serializes stop/cleanup calls, including callers that
        # observe an already-stopping recording: they wait for the same work
        # instead of returning a half-finished snapshot.
        with self._stop_lock:
            with self._lock:
                process = self._process
                if process is None or self._state not in ("recording", "stopping"):
                    return self.status(now)
                if self._state == "recording":
                    self._state = "stopping"
                    self._message = (
                        "Stopping recording; waiting for rosbag to finalize."
                    )
                    self._stop_requested = True
                pgid = self._pgid
            self._terminate(process, pgid)
            reader = self._reader
            if reader is not None and reader is not threading.current_thread():
                # The reader owns closing process.stdout. Never close it from
                # this thread while the reader may be blocked in readline on
                # the same IO lock: that close can itself block unbounded.
                # Stay in "stopping" until the reader drains and finalizes.
                reader.join(timeout=READER_JOIN_TIMEOUT)
            reader_alive = (
                reader is not None
                and reader is not threading.current_thread()
                and reader.is_alive()
            )
            returncode = process.poll()
            if returncode is not None and not reader_alive:
                self._finalize(process, returncode)
            return self.status(now)

    def cleanup(self, now: float | None = None) -> None:
        # Always go through stop() so concurrent cleanup waits on the same stop
        # lock and can finish a stop that previously timed out.
        self.stop(now)

    def status(self, now: float | None = None) -> dict[str, Any]:
        with self._lock:
            process = self._process
            running = process is not None and process.poll() is None
            output = self._output
            directory = str(output.parent) if output is not None else ""
            return {
                "available": self.available(),
                "state": self._state,
                "running": running,
                "topics": list(self._topics),
                "directory": directory,
                "output_dir": str(output) if output is not None else "",
                "message": self._message,
                "returncode": self._returncode,
                "started_at": self._started_at,
                "finished_at": self._finished_at,
                "log_tail": list(self._log),
                "stamp": time.time() if now is None else now,
            }
