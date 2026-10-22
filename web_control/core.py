import math
import os
import re
import shlex
import threading
from dataclasses import dataclass
from collections import deque
from pathlib import PurePosixPath
from typing import Any, Iterable, Optional

from web_control.log_store import LogSession, LogStorageError

CpuTicks = tuple[int, ...]
CpuSnapshot = dict[str, Any]


class ViewerRegistry:
    def __init__(self, timeout_sec: float = 10.0) -> None:
        self.timeout_sec = timeout_sec
        self.viewers: dict[str, float] = {}
        self.lock = threading.RLock()

    def heartbeat(self, viewer_id: str, now: float) -> int:
        if not viewer_id:
            raise ValueError("viewer_id is required")
        with self.lock:
            self.viewers[viewer_id] = now
            self._prune(now)
            return len(self.viewers)

    def count(self, now: float) -> int:
        with self.lock:
            self._prune(now)
            return len(self.viewers)

    def _prune(self, now: float) -> None:
        cutoff = now - self.timeout_sec
        expired = [viewer_id for viewer_id, stamp in self.viewers.items() if stamp < cutoff]
        for viewer_id in expired:
            del self.viewers[viewer_id]


def quaternion_from_yaw(yaw: float) -> dict[str, float]:
    half = yaw * 0.5
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def yaw_from_quaternion(q: dict[str, float]) -> float:
    x = float(q.get("x", 0.0))
    y = float(q.get("y", 0.0))
    z = float(q.get("z", 0.0))
    w = float(q.get("w", 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _finite_number(payload: dict[str, Any], key: str) -> float:
    if key not in payload:
        raise ValueError(f"missing {key}")
    try:
        value = float(payload[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"{key} must be finite")
    return value


def validate_pose_payload(payload: dict[str, Any]) -> dict[str, float]:
    x = _finite_number(payload, "x")
    y = _finite_number(payload, "y")
    yaw = _finite_number(payload, "yaw")
    z = float(payload.get("z", 0.0))
    if not math.isfinite(z):
        raise ValueError("z must be finite")
    return {"x": x, "y": y, "z": z, "yaw": yaw}


class SaveMapUnavailableError(RuntimeError):
    """Raised when the save map service cannot accept a request."""


class SaveMapBusyError(RuntimeError):
    """Raised when a save map job is already running."""


class SaveMapJob:
    """Thread-safe state for the single in-flight save map service call.

    The job owns a token that changes on every run. A late or cancelled
    service callback whose token no longer matches is ignored, so a stale
    result can never overwrite the outcome of a newer run.
    """

    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state = self.IDLE
        self._token = 0
        self._destination = ""
        self._message = ""
        self._started_at: float | None = None
        self._finished_at: float | None = None

    def begin(self, destination: str, now: float) -> int | None:
        """Claim the job for a new run, or return None if one is running."""
        with self._lock:
            if self._state == self.RUNNING:
                return None
            self._token += 1
            self._state = self.RUNNING
            self._destination = destination
            self._message = ""
            self._started_at = now
            self._finished_at = None
            return self._token

    def finish(self, token: int, success: bool, message: str, now: float) -> bool:
        """Record a service result; stale or duplicate tokens are ignored."""
        with self._lock:
            if self._state != self.RUNNING or token != self._token:
                return False
            self._state = self.SUCCEEDED if success else self.FAILED
            self._message = message
            self._finished_at = now
            return True

    def abandon_running(self, message: str, now: float) -> bool:
        """Fail a running job whose service call can no longer complete.

        Bumping the token invalidates any callback still in flight for the
        abandoned run.
        """
        with self._lock:
            if self._state != self.RUNNING:
                return False
            self._state = self.FAILED
            self._message = message
            self._finished_at = now
            self._token += 1
            return True

    def is_running(self) -> bool:
        with self._lock:
            return self._state == self.RUNNING

    def state(self) -> str:
        with self._lock:
            return self._state

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            finished = self._state in (self.SUCCEEDED, self.FAILED)
            return {
                "state": self._state,
                "running": self._state == self.RUNNING,
                "success": (self._state == self.SUCCEEDED) if finished else None,
                "destination": self._destination,
                "message": self._message,
                "started_at": self._started_at,
                "finished_at": self._finished_at,
            }


def _optional_bool(payload: dict[str, Any], key: str, default: bool) -> bool:
    if key not in payload:
        return default
    value = payload[key]
    if not isinstance(value, bool):
        raise ValueError(f"{key} must be a boolean")
    return value


def _optional_finite_number(
    payload: dict[str, Any], key: str, default: float
) -> float:
    if key not in payload or payload[key] is None:
        return default
    try:
        value = float(payload[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"{key} must be finite")
    return value


def runtime_home_dir() -> str:
    """Return the home directory of the user running the web node."""
    return os.path.expanduser("~")


def resolve_save_map_destination(destination: str, home: str) -> str:
    """Resolve a web save-map destination to an absolute path under ``home``.

    The web dialog sends a path starting with ``/`` that is relative to the web
    node user's home directory (``/map`` or ``/map/site``), so the browser can
    never pick an arbitrary robot directory. ``..`` components are rejected
    instead of normalised, keeping the resolved path inside ``home``, and an
    already home-prefixed destination (``/home/robot/map`` when the home is
    ``/home/robot``) is rejected so the prefix is not applied twice.
    """
    if not isinstance(destination, str):
        raise ValueError("destination must be a string path")
    destination = destination.strip()
    if not destination:
        raise ValueError("destination is required")
    if not destination.startswith("/"):
        raise ValueError(
            "destination must start with / and name a path under the home directory"
        )

    parts = PurePosixPath(destination).parts
    if ".." in parts:
        raise ValueError("destination must not contain '..' path components")
    relative_parts = parts[1:]
    if not relative_parts:
        raise ValueError("destination must not be the filesystem root /")

    if not isinstance(home, str) or not home.strip():
        raise ValueError("home directory is required")
    home = home.strip()
    if not os.path.isabs(home):
        raise ValueError("home directory must be an absolute path")

    home_path = PurePosixPath(home)
    destination_path = PurePosixPath(destination)
    if home_path != PurePosixPath("/") and (
        destination_path == home_path or home_path in destination_path.parents
    ):
        raise ValueError(
            "destination must be relative to the home directory; "
            f"remove the {home} prefix (for example /map instead of {home}/map)"
        )

    return os.path.join(home, *relative_parts)


def validate_save_map_payload(
    payload: dict[str, Any], *, home: str | None = None
) -> dict[str, Any]:
    """Validate a web save-map payload against the SaveMapOptions service.

    The web destination is relative to the web node user's home directory
    (``/map``, ``/map/site``); it is resolved to an absolute path under ``home``
    (default: the runtime user's home) before the service sees it. Omitted
    toggles use the web dialog defaults (both artifacts enabled), and explicit
    resolution values are forwarded unchanged.
    """
    if not isinstance(payload, dict):
        raise ValueError("payload must be an object")
    destination = resolve_save_map_destination(
        payload.get("destination"),
        runtime_home_dir() if home is None else home,
    )
    return {
        "destination": destination,
        "resolution": _optional_finite_number(payload, "resolution", -1.0),
        "ground_resolution": _optional_finite_number(
            payload, "ground_resolution", -1.0
        ),
        "save_ground_map": _optional_bool(payload, "save_ground_map", True),
        "save_tiled_map": _optional_bool(payload, "save_tiled_map", True),
    }


def downsample_points(
    points: list[tuple[float, float, float]], max_points: int
) -> list[tuple[float, float, float]]:
    if max_points <= 0 or len(points) <= max_points:
        return list(points)
    step = max(1, math.ceil(len(points) / max_points))
    sampled = points[::step]
    if sampled[-1] != points[-1] and len(sampled) < max_points:
        sampled.append(points[-1])
    return sampled[:max_points]


def project_points_2d(points: Iterable[tuple[float, float, float]]) -> list[list[float]]:
    return [[float(x), float(y)] for x, y, _z in points]


def parse_proc_stat_cpu_ticks(text: str) -> CpuSnapshot:
    total: CpuTicks | None = None
    cores: list[CpuTicks] = []
    for line in text.splitlines():
        parts = line.split()
        if not parts:
            continue
        name = parts[0]
        if name != "cpu" and not re.fullmatch(r"cpu\d+", name):
            continue
        try:
            ticks = tuple(int(value) for value in parts[1:])
        except ValueError:
            continue
        if name == "cpu":
            total = ticks
        else:
            cores.append(ticks)
    if total is None:
        raise ValueError("missing aggregate cpu line")
    return {"total": total, "cores": cores}


def _cpu_idle_ticks(ticks: CpuTicks) -> int:
    idle = ticks[3] if len(ticks) > 3 else 0
    iowait = ticks[4] if len(ticks) > 4 else 0
    return idle + iowait


def _cpu_percent(previous: CpuTicks | None, current: CpuTicks | None) -> float | None:
    if previous is None or current is None:
        return None
    total_delta = sum(current) - sum(previous)
    idle_delta = _cpu_idle_ticks(current) - _cpu_idle_ticks(previous)
    if total_delta <= 0:
        return None
    busy_delta = max(0, total_delta - idle_delta)
    return round((busy_delta / total_delta) * 100.0, 2)


def calculate_cpu_usage(
    previous: CpuSnapshot | None, current: CpuSnapshot
) -> dict[str, Any]:
    previous_cores = previous.get("cores", []) if previous else []
    current_cores = current.get("cores", [])
    cores = []
    for index, current_ticks in enumerate(current_cores):
        previous_ticks = previous_cores[index] if index < len(previous_cores) else None
        cores.append({"id": index, "percent": _cpu_percent(previous_ticks, current_ticks)})
    return {
        "total_percent": _cpu_percent(
            previous.get("total") if previous else None,
            current.get("total"),
        ),
        "cores": cores,
    }


def parse_proc_meminfo(text: str) -> dict[str, Any]:
    values: dict[str, int] = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, raw_value = line.split(":", 1)
        parts = raw_value.strip().split()
        if not parts:
            continue
        try:
            values[key] = int(parts[0])
        except ValueError:
            continue

    mem_total = max(0, values.get("MemTotal", 0))
    mem_available = max(0, values.get("MemAvailable", values.get("MemFree", 0)))
    mem_used = max(0, mem_total - mem_available)
    swap_total = max(0, values.get("SwapTotal", 0))
    swap_free = max(0, values.get("SwapFree", 0))
    swap_used = max(0, swap_total - swap_free)

    return {
        "memory": {
            "total_kb": mem_total,
            "used_kb": mem_used,
            "percent": round((mem_used / mem_total) * 100.0, 2) if mem_total else 0.0,
        },
        "swap": {
            "total_kb": swap_total,
            "used_kb": swap_used,
            "percent": round((swap_used / swap_total) * 100.0, 2) if swap_total else 0.0,
        },
    }


def build_system_usage_payload(
    proc_stat_text: str,
    proc_meminfo_text: str,
    previous_ticks: CpuSnapshot | None,
    stamp: float,
) -> tuple[dict[str, Any], CpuSnapshot]:
    current_ticks = parse_proc_stat_cpu_ticks(proc_stat_text)
    payload = {
        "stamp": stamp,
        "cpu": calculate_cpu_usage(previous_ticks, current_ticks),
        **parse_proc_meminfo(proc_meminfo_text),
    }
    return payload, current_ticks


@dataclass
class SourceCache:
    name: str
    stale_timeout_sec: float
    static_source: bool = False
    data: Optional[Any] = None
    stamp: Optional[float] = None
    frame_id: str = ""
    count: int = 0
    # Monotonic content generation. Point-cloud HTTP serialization keys on it
    # so a cached body can never outlive an update or clear, without keeping a
    # separate revision map in the node.
    version: int = 0

    def update(
        self, data: Any, stamp: float, frame_id: str = "", count: int = 0
    ) -> None:
        self.data = data
        self.stamp = stamp
        self.frame_id = frame_id
        self.count = count
        self.version += 1

    def clear(self) -> None:
        self.data = None
        self.stamp = None
        self.frame_id = ""
        self.count = 0
        self.version += 1

    def status(self, now: float) -> str:
        if self.stamp is None:
            return "waiting"
        if self.static_source:
            return "loaded"
        if now - self.stamp > self.stale_timeout_sec:
            return "stale"
        return "active"

    def summary(self, now: float) -> dict[str, Any]:
        return {
            "status": self.status(now),
            "stamp": self.stamp,
            "age": None if self.stamp is None else max(0.0, now - self.stamp),
            "frame_id": self.frame_id,
            "count": self.count,
        }


# Distinguishes "caller omitted the session" (fall back to the buffer's current
# session) from an explicit ``None`` (this round has no persistent session, so
# never spill into whatever session the buffer currently holds).
_SESSION_UNSPECIFIED = object()


class TerminalLogBuffer:
    def __init__(self, target: str, max_lines: int = 400) -> None:
        self.target = target
        # Each entry is (seq, text) where seq is a monotonically increasing id
        # assigned at append time. seq is never reused, so dropping old lines
        # from the front (rotation) cannot confuse a delta consumer.
        self.lines: deque[tuple[int, str]] = deque(maxlen=max_lines)
        self._seq = 0
        self.enabled = False
        self.running = False
        self.returncode: int | None = None
        self.lock = threading.RLock()
        # Optional persistent session writing every sanitised line to disk.
        self.session: LogSession | None = None
        self.persist_error: str | None = None

    def attach_session(self, session: LogSession | None) -> None:
        with self.lock:
            self.session = session
            self.persist_error = None if session is None else session.write_error

    def set_persist_error(self, message: str) -> None:
        with self.lock:
            self.persist_error = message

    def _write_session(self, text: str, session: Any = _SESSION_UNSPECIFIED) -> None:
        target = self.session if session is _SESSION_UNSPECIFIED else session
        if target is None or target.closed:
            return
        try:
            target.write_line(text)
        except LogStorageError as exc:
            # Keep the live buffer and the running process working; only the
            # persistent copy is degraded and the error is surfaced via the API.
            self.persist_error = str(exc)

    def append(self, line: str, session: Any = _SESSION_UNSPECIFIED) -> None:
        clean_lines = line.splitlines() or [line]
        with self.lock:
            for clean_line in clean_lines:
                text = ANSI_ESCAPE_RE.sub("", clean_line).rstrip("\r\n")
                self._seq += 1
                self.lines.append((self._seq, text))
                self._write_session(text, session)

    def finish_session(
        self, session: Any = _SESSION_UNSPECIFIED, message: str | None = None
    ) -> None:
        """Record a final status line and close the session's file handle."""
        with self.lock:
            target = self.session if session is _SESSION_UNSPECIFIED else session
            if message is not None:
                self._seq += 1
                self.lines.append((self._seq, message))
                self._write_session(message, target)
            if target is not None:
                target.close()
            if self.session is target:
                self.session = None
            if target is not None and target.write_error:
                self.persist_error = target.write_error

    def session_info(self) -> dict[str, Any]:
        with self.lock:
            info: dict[str, Any] = {"persist_error": self.persist_error}
            if self.session is not None:
                info.update(self.session.metadata())
            return info

    def set_process_state(
        self, enabled: bool, running: bool, returncode: int | None
    ) -> None:
        with self.lock:
            self.enabled = enabled
            self.running = running
            self.returncode = returncode

    def lines_since(self, since: int) -> tuple[list[str], int]:
        """Return (texts_with_seq_gt_since, current_max_seq) under the lock."""
        with self.lock:
            texts = [text for seq, text in self.lines if seq > since]
            return texts, self._seq

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "target": self.target,
                "enabled": self.enabled,
                "running": self.running,
                "returncode": self.returncode,
                "lines": [text for _seq, text in self.lines],
            }

    def delta(self, since: int) -> dict[str, Any]:
        """Like snapshot() but returns only lines with seq > since plus the
        current max seq, so clients can poll incrementally."""
        with self.lock:
            texts = [text for seq, text in self.lines if seq > since]
            return {
                "target": self.target,
                "enabled": self.enabled,
                "running": self.running,
                "returncode": self.returncode,
                "lines": texts,
                "seq": self._seq,
            }


def build_managed_launch_command(
    package: str, launch_file: str, launch_args: str = ""
) -> list[str]:
    return ["ros2", "launch", package, launch_file, *shlex.split(launch_args)]


ANSI_ESCAPE_RE = re.compile(r"\x1b(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")
