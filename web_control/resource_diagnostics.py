"""ROS-free, bounded resource sampling for periodic diagnostics logs.

The sampler is intentionally independent from the HTTP ``system_payload()``
baseline and from ROS callbacks.  It reads ``/proc`` once per sample (plus
bounded per-selected-PID status/cgroup files), never launches ``ps`` or any
other subprocess, and never reads command lines or environments.

Scope is deliberately narrow: the web process itself plus descendants of the
launch roots that the web node manages.  Unmanaged ROS processes are not
guessed from names or command lines.  First observations, PID identity changes,
counter resets and disappeared/denied processes use ``null`` rather than a
fabricated zero.
"""

from __future__ import annotations

import math
import os
import re
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Optional

DEFAULT_SAMPLE_PERIOD_SEC = 1.0
DEFAULT_CGROUP_ROOT = "/sys/fs/cgroup"
MAX_CGROUP_DEPTH = 16
MAX_CONTROL_FILE_BYTES = 64 * 1024

CPU_PERCENT_BASIS = "process_delta_ticks"
SYSTEM_CPU_BASIS = "proc_stat_busy_delta"

_MEASUREMENT_LIMITATIONS = (
    "Only the PID namespace visible to web_control is sampled; processes in a "
    "different/root PID namespace are not observable.",
    "Unmanaged or externally launched ROS processes are not included and are "
    "never inferred from process names or command lines.",
    "Process CPU is the /proc/<pid>/stat utime+stime delta over the actual "
    "monotonic interval; 100% is one CPU core and a process may exceed 100%.",
    "Cpus_allowed_list is the main thread's scheduler affinity from "
    "/proc/<pid>/status; other threads may have different affinity, and it is "
    "not the last CPU executed. cgroup quota/cpuset values are read-only "
    "observations.",
    "RSS is /proc/<pid>/stat rss pages multiplied by the proc page size; this "
    "is not a complete measure of all memory used by the process.",
    "First sample, new PID, PID identity change, counter reset, disappearance "
    "or denied access produces null CPU, never a fabricated zero.",
)


def _finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _read_text(path: Path) -> tuple[Optional[str], Optional[str]]:
    try:
        with path.open("r", encoding="utf-8", errors="replace") as handle:
            return handle.read(MAX_CONTROL_FILE_BYTES), None
    except OSError as exc:
        return None, str(exc)


def parse_pid_stat(text: str) -> dict[str, Any]:
    """Parse one ``/proc/<pid>/stat`` line with robust ``comm`` handling."""
    lparen = text.find("(")
    rparen = text.rfind(")")
    if lparen < 0 or rparen < lparen:
        raise ValueError("missing process name delimiters")
    try:
        pid = int(text[:lparen].strip())
    except ValueError as exc:
        raise ValueError("invalid pid") from exc
    fields = text[rparen + 2 :].split()
    if len(fields) < 22:
        raise ValueError("truncated process stat")

    def field(index: int) -> int:
        try:
            return int(fields[index])
        except (IndexError, ValueError) as exc:
            raise ValueError(f"invalid stat field {index + 3}") from exc

    return {
        "pid": pid,
        "name": text[lparen + 1 : rparen],
        "state": fields[0],
        "ppid": field(1),
        "pgrp": field(2),
        "utime": field(11),
        "stime": field(12),
        "threads": field(17),
        "starttime": field(19),
        "rss_pages": field(21),
        "last_cpu_executed": field(36) if len(fields) > 36 else None,
    }


def parse_proc_stat_cpu(text: str) -> dict[str, Any]:
    total: Optional[tuple[int, ...]] = None
    cores: list[dict[str, Any]] = []
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
            cores.append({"id": int(name[3:]), "ticks": ticks})
    if total is None:
        raise ValueError("missing aggregate cpu line")
    return {"total": total, "cores": cores}


def _idle_ticks(ticks: tuple[int, ...]) -> int:
    idle = ticks[3] if len(ticks) > 3 else 0
    iowait = ticks[4] if len(ticks) > 4 else 0
    return idle + iowait


def _cpu_percent(
    previous: Optional[tuple[int, ...]], current: Optional[tuple[int, ...]]
) -> Optional[float]:
    if previous is None or current is None:
        return None
    # Linux appends guest and guest_nice after steal, but those ticks are
    # already included in user/nice.  Count only the first eight columns
    # (through steal) so guest time is not double-counted.
    previous = previous[:8]
    current = current[:8]
    total_delta = sum(current) - sum(previous)
    idle_delta = _idle_ticks(current) - _idle_ticks(previous)
    if total_delta <= 0 or idle_delta < 0 or idle_delta > total_delta:
        return None
    busy_delta = total_delta - idle_delta
    return round((busy_delta / total_delta) * 100.0, 2)


def parse_meminfo_bytes(text: str) -> dict[str, Any]:
    values: dict[str, int] = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, raw_value = line.split(":", 1)
        parts = raw_value.strip().split()
        if not parts:
            continue
        try:
            values[key] = int(parts[0]) * 1024
        except ValueError:
            continue
    total = max(0, values.get("MemTotal", 0))
    available = max(0, values.get("MemAvailable", values.get("MemFree", 0)))
    free = max(0, values.get("MemFree", 0))
    used = max(0, total - available)
    swap_total = max(0, values.get("SwapTotal", 0))
    swap_free = max(0, values.get("SwapFree", 0))
    swap_used = max(0, swap_total - swap_free)
    return {
        "total_bytes": total,
        "used_bytes": used,
        "available_bytes": available,
        "free_bytes": free,
        "percent": round((used / total) * 100.0, 2) if total else None,
        "swap": {
            "total_bytes": swap_total,
            "used_bytes": swap_used,
            "free_bytes": swap_free,
            "percent": round((swap_used / swap_total) * 100.0) if swap_total else None,
        },
    }


def parse_loadavg(text: str) -> dict[str, Any]:
    parts = text.split()
    result: dict[str, Any] = {
        "load1": None,
        "load5": None,
        "load15": None,
        "running": None,
        "total_tasks": None,
        "last_pid": None,
    }
    for index, key in enumerate(("load1", "load5", "load15")):
        if index < len(parts):
            result[key] = _finite(parts[index])
    if len(parts) > 3 and "/" in parts[3]:
        running, _, total = parts[3].partition("/")
        try:
            result["running"] = int(running)
            result["total_tasks"] = int(total)
        except ValueError:
            pass
    if len(parts) > 4:
        try:
            result["last_pid"] = int(parts[4])
        except ValueError:
            pass
    return result


def parse_cpu_list(text: Any) -> dict[str, Any]:
    raw = "" if text is None else str(text).strip()
    if not raw:
        return {"list": None, "count": None, "valid": False}
    count = 0
    try:
        for part in raw.split(","):
            part = part.strip()
            if not part:
                raise ValueError("empty cpu range")
            if "-" in part:
                start_text, end_text = part.split("-", 1)
                start, end = int(start_text), int(end_text)
            else:
                start = end = int(part)
            if start < 0 or end < start:
                raise ValueError("invalid cpu range")
            count += end - start + 1
    except (TypeError, ValueError):
        return {"list": raw, "count": None, "valid": False}
    return {"list": raw, "count": count, "valid": True}


def _parse_cgroup_membership(text: str) -> dict[str, Any]:
    membership: dict[str, Any] = {
        "v2": None,
        "v1_cpu": None,
        "v1_cpuset": None,
    }
    for line in text.splitlines():
        parts = line.split(":", 2)
        if len(parts) != 3:
            continue
        hierarchy, controllers, path = parts
        if hierarchy.strip() == "0" and not controllers.strip():
            membership["v2"] = path or "/"
            continue
        names = {name.strip() for name in controllers.split(",") if name.strip()}
        if "cpu" in names:
            membership["v1_cpu"] = path or "/"
        if "cpuset" in names:
            membership["v1_cpuset"] = path or "/"
    return membership


def _safe_cgroup_path(root: Path, relative: str) -> Optional[Path]:
    parts = Path(relative).parts
    if ".." in parts:
        return None
    candidate = root / relative.lstrip("/")
    try:
        root_resolved = root.resolve(strict=False)
        candidate_resolved = candidate.resolve(strict=False)
        if os.path.commonpath(
            [str(root_resolved), str(candidate_resolved)]
        ) != str(root_resolved):
            return None
    except (OSError, ValueError):
        return None
    return candidate


def _ancestor_paths(path: str) -> tuple[list[str], bool]:
    parts = [part for part in path.split("/") if part]
    truncated = len(parts) > MAX_CGROUP_DEPTH
    if truncated:
        parts = parts[:MAX_CGROUP_DEPTH]
    return [""] + [
        "/".join(parts[:index]) for index in range(1, len(parts) + 1)
    ], truncated


def _parse_v2_limit(raw: Optional[str]) -> tuple[Optional[int], Optional[int], Optional[str]]:
    if raw is None:
        return None, None, "unreadable"
    parts = raw.split()
    if len(parts) != 2:
        return None, None, "malformed cpu.max"
    try:
        period = int(parts[1])
        quota = None if parts[0] == "max" else int(parts[0])
    except ValueError:
        return None, None, "malformed cpu.max"
    if period <= 0:
        return None, None, "invalid cpu.max period"
    return quota, period, None


def _parse_v1_limit(
    quota_raw: Optional[str], period_raw: Optional[str]
) -> tuple[Optional[int], Optional[int], Optional[str]]:
    if quota_raw is None or period_raw is None:
        return None, None, "unreadable"
    try:
        quota = int(quota_raw.strip())
        period = int(period_raw.strip())
    except ValueError:
        return None, None, "malformed cpu quota"
    if period <= 0:
        return None, None, "invalid cpu period"
    return (None if quota < 0 else quota), period, None


def _quota_record(
    quota: Optional[int], period: Optional[int]
) -> Optional[float]:
    if quota is None or period is None or period <= 0:
        return None
    return round((quota / period) * 1000.0, 3)


def _parse_cpu_stat(text: str, *, v1: bool) -> dict[str, Any]:
    raw: dict[str, int] = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) != 2:
            continue
        try:
            raw[parts[0]] = int(parts[1])
        except ValueError:
            continue
    throttled_usec = raw.get("throttled_usec")
    if throttled_usec is None and "throttled_time" in raw:
        throttled_usec = raw["throttled_time"] // 1000
    periods = raw.get("nr_periods")
    throttled = raw.get("nr_throttled")
    ratio = None
    if periods is not None and periods > 0 and throttled is not None:
        ratio = round(throttled / periods, 6)
    return {
        "available": True,
        "source": "cpu.stat",
        "controller_version": 1 if v1 else 2,
        "raw": raw,
        "usage_usec": raw.get("usage_usec"),
        "nr_periods": periods,
        "nr_throttled": throttled,
        "throttled_usec": throttled_usec,
        "throttled_period_ratio": ratio,
    }


class ResourceSampler:
    """Sample system and managed-process resources without ROS or subprocesses."""

    def __init__(
        self,
        *,
        proc_root: str | os.PathLike[str] = "/proc",
        cgroup_root: str | os.PathLike[str] = DEFAULT_CGROUP_ROOT,
        page_size: Optional[int] = None,
        clock_ticks: Optional[int] = None,
    ) -> None:
        self.proc_root = Path(proc_root)
        self.cgroup_root = Path(cgroup_root)
        try:
            self.page_size = int(page_size or os.sysconf("SC_PAGE_SIZE"))
        except (AttributeError, OSError, ValueError):
            self.page_size = 4096
        try:
            self.clock_ticks = int(clock_ticks or os.sysconf("SC_CLK_TCK"))
        except (AttributeError, OSError, ValueError):
            self.clock_ticks = 100
        if self.page_size <= 0:
            self.page_size = 4096
        if self.clock_ticks <= 0:
            self.clock_ticks = 100
        self._lock = threading.RLock()
        self._previous_system: Optional[dict[str, Any]] = None
        self._previous_processes: dict[int, dict[str, Any]] = {}
        self._previous_groups: dict[str, dict[int, dict[str, Any]]] = {}

    # -- public ---------------------------------------------------------
    def sample(
        self,
        scope: Iterable[dict[str, Any]],
        now_mono: float,
        now_wall: float,
    ) -> dict[str, Any]:
        with self._lock:
            return self._sample_locked(list(scope or []), now_mono, now_wall)

    # -- system ---------------------------------------------------------
    def _system_payload(
        self, now_mono: float, errors: list[str]
    ) -> tuple[dict[str, Any], Optional[float]]:
        previous = self._previous_system
        interval = None
        if previous is not None:
            interval = now_mono - float(previous["monotonic"])
            if not math.isfinite(interval) or interval <= 0:
                interval = None

        cpu: dict[str, Any] = {
            "total_percent": None,
            "per_core": [],
            "basis": SYSTEM_CPU_BASIS,
            "normalization": "0..100 across all CPUs",
            "available": False,
            "unknown_reason": None,
            "error": None,
        }
        stat_text, stat_error = _read_text(self.proc_root / "stat")
        if stat_text is None:
            cpu["error"] = f"/proc/stat: {stat_error}"
            cpu["unknown_reason"] = "unreadable"
            errors.append(cpu["error"])
        else:
            try:
                current = parse_proc_stat_cpu(stat_text)
            except ValueError as exc:
                cpu["error"] = f"/proc/stat: {exc}"
                cpu["unknown_reason"] = "unreadable"
                errors.append(cpu["error"])
            else:
                previous_total = previous.get("total") if previous else None
                previous_cores = previous.get("cores", {}) if previous else {}
                cpu["total_percent"] = _cpu_percent(
                    previous_total, current["total"]
                )
                if previous is None:
                    cpu["unknown_reason"] = "first_sample"
                elif interval is None:
                    cpu["unknown_reason"] = "time_reset"
                elif cpu["total_percent"] is None:
                    cpu["unknown_reason"] = "counter_reset_or_no_delta"
                cpu["per_core"] = [
                    {
                        "id": core["id"],
                        "percent": _cpu_percent(
                            previous_cores.get(core["id"]), core["ticks"]
                        ),
                    }
                    for core in current["cores"]
                ]
                cpu["available"] = True
                self._previous_system = {
                    "monotonic": now_mono,
                    "total": current["total"],
                    "cores": {core["id"]: core["ticks"] for core in current["cores"]},
                }

        memory: dict[str, Any] = {
            "available": False,
            "total_bytes": None,
            "used_bytes": None,
            "available_bytes": None,
            "free_bytes": None,
            "percent": None,
            "error": None,
        }
        mem_text, mem_error = _read_text(self.proc_root / "meminfo")
        if mem_text is None:
            memory["error"] = f"/proc/meminfo: {mem_error}"
            errors.append(memory["error"])
        else:
            parsed = parse_meminfo_bytes(mem_text)
            memory.update(parsed)
            memory["swap"] = parsed["swap"]
            memory["available"] = True
        swap = memory.get("swap") or {
            "total_bytes": None,
            "used_bytes": None,
            "free_bytes": None,
            "percent": None,
        }
        memory.pop("swap", None)

        loadavg: dict[str, Any] = {"available": False, "error": None}
        load_text, load_error = _read_text(self.proc_root / "loadavg")
        if load_text is None:
            loadavg["error"] = f"/proc/loadavg: {load_error}"
            errors.append(loadavg["error"])
        else:
            loadavg.update(parse_loadavg(load_text))
            loadavg["available"] = True

        return (
            {
                "cpu": cpu,
                "memory": memory,
                "swap": swap,
                "loadavg": loadavg,
            },
            interval,
        )

    # -- process scope --------------------------------------------------
    def _scan_pid_stats(self, errors: list[str]) -> dict[int, dict[str, Any]]:
        stats: dict[int, dict[str, Any]] = {}
        try:
            entries = list(self.proc_root.iterdir())
        except OSError as exc:
            errors.append(f"{self.proc_root}: {exc}")
            return stats
        for entry in entries:
            if not entry.name.isdigit():
                continue
            text, _error = _read_text(entry / "stat")
            if text is None:
                continue
            try:
                parsed = parse_pid_stat(text)
            except ValueError:
                continue
            stats[parsed["pid"]] = parsed
        return stats

    def _read_affinity(self, pid: int) -> dict[str, Any]:
        source = str(self.proc_root / str(pid) / "status")
        text, error = _read_text(Path(source))
        if text is None:
            return {
                "readable": False,
                "source": source,
                "scope": "main_thread_pid_status",
                "cpus_allowed_list": None,
                "cpu_count": None,
                "error": error,
            }
        allowed_list = None
        for line in text.splitlines():
            if line.startswith("Cpus_allowed_list:"):
                allowed_list = line.split(":", 1)[1].strip()
                break
        parsed = parse_cpu_list(allowed_list)
        return {
            "readable": parsed["valid"],
            "source": source,
            "scope": "main_thread_pid_status",
            "cpus_allowed_list": parsed["list"],
            "cpu_count": parsed["count"],
            "error": None if parsed["valid"] else "Cpus_allowed_list unavailable",
        }

    def _read_cgroup(
        self, pid: int, cache: dict[tuple[Any, ...], tuple[Optional[str], Optional[str]]]
    ) -> dict[str, Any]:
        membership_path = self.proc_root / str(pid) / "cgroup"
        text, error = _read_text(membership_path)
        unknown_quota = {
            "state": "unknown",
            "raw_current": None,
            "current_quota_us": None,
            "current_period_us": None,
            "current_quota_millicores": None,
            "effective_quota_millicores": None,
            "effective_source": None,
            "inherited": None,
            "ancestors_complete": False,
            "source": None,
            "error": "cgroup membership unreadable",
        }
        if text is None:
            return {
                "version": None,
                "path": None,
                "membership": None,
                "cpu_quota": dict(unknown_quota),
                "cpu_stat": {
                    "available": False,
                    "raw": {},
                    "nr_periods": None,
                    "nr_throttled": None,
                    "throttled_usec": None,
                    "throttled_period_ratio": None,
                    "error": error,
                },
                "cpuset": {
                    "available": False,
                    "current_list": None,
                    "effective_list": None,
                    "effective_cpu_count": None,
                    "error": error,
                },
                "error": error,
            }
        membership = _parse_cgroup_membership(text)
        if membership["v2"] is not None:
            return self._cgroup_v2(membership, cache)
        if membership["v1_cpu"] is not None or membership["v1_cpuset"] is not None:
            return self._cgroup_v1(membership, cache)
        return {
            "version": None,
            "path": None,
            "membership": membership,
            "cpu_quota": dict(unknown_quota),
            "cpu_stat": {"available": False, "raw": {}, "error": "cpu controller not found"},
            "cpuset": {
                "available": False,
                "current_list": None,
                "effective_list": None,
                "effective_cpu_count": None,
                "error": "cpuset controller not found",
            },
            "error": "cpu/cpuset cgroup membership not found",
        }

    def _cached_text(
        self,
        path: Path,
        cache: dict[tuple[Any, ...], tuple[Optional[str], Optional[str]]],
    ) -> tuple[Optional[str], Optional[str]]:
        key = ("text", str(path))
        if key not in cache:
            cache[key] = _read_text(path)
        return cache[key]

    def _cgroup_v2(
        self,
        membership: dict[str, Any],
        cache: dict[tuple[Any, ...], tuple[Optional[str], Optional[str]]],
    ) -> dict[str, Any]:
        relative = str(membership["v2"] or "/")
        base = _safe_cgroup_path(self.cgroup_root, relative)
        if base is None:
            return self._unknown_cgroup(2, relative, "unsafe cgroup path")
        ancestors: list[dict[str, Any]] = []
        ancestor_paths, truncated = _ancestor_paths(relative)
        for cpath in ancestor_paths:
            directory = _safe_cgroup_path(self.cgroup_root, cpath)
            if directory is None:
                ancestors.append({"path": cpath, "error": "unsafe cgroup path"})
                continue
            raw, error = self._cached_text(directory / "cpu.max", cache)
            quota, period, parse_error = _parse_v2_limit(raw)
            if parse_error is not None and raw is not None:
                error = parse_error
            ancestors.append(
                {
                    "path": cpath,
                    "source": str(directory / "cpu.max"),
                    "raw": None if raw is None else raw.strip(),
                    "quota_us": quota,
                    "period_us": period,
                    "error": error,
                }
            )
        if truncated:
            ancestors.append(
                {
                    "path": relative,
                    "source": None,
                    "raw": None,
                    "quota_us": None,
                    "period_us": None,
                    "error": "cgroup ancestry exceeds bounded read depth",
                }
            )
        quota = self._quota_summary(ancestors, "v2", relative)
        stat_path = base / "cpu.stat"
        stat_text, stat_error = self._cached_text(stat_path, cache)
        if stat_text is None:
            cpu_stat = {
                "available": False,
                "source": str(stat_path),
                "raw": {},
                "nr_periods": None,
                "nr_throttled": None,
                "throttled_usec": None,
                "throttled_period_ratio": None,
                "error": stat_error,
            }
        else:
            cpu_stat = _parse_cpu_stat(stat_text, v1=False)
            cpu_stat["source"] = str(stat_path)
        cpuset = self._v2_cpuset(base, cache)
        return {
            "version": 2,
            "path": relative,
            "membership": membership,
            "cpu_quota": quota,
            "cpu_stat": cpu_stat,
            "cpuset": cpuset,
            "error": None,
        }

    def _quota_summary(
        self, ancestors: list[dict[str, Any]], version: str, relative: str
    ) -> dict[str, Any]:
        finite = []
        for entry in ancestors:
            quota = entry.get("quota_us")
            period = entry.get("period_us")
            ratio = _quota_record(quota, period)
            if ratio is not None:
                finite.append((ratio, entry))
        errors = [entry for entry in ancestors if entry.get("error")]
        current = ancestors[-1] if ancestors else {}
        current_ratio = _quota_record(
            current.get("quota_us"), current.get("period_us")
        )
        if finite:
            ratio, winner = min(finite, key=lambda item: item[0])
            state = "limited"
            effective = ratio
            source = winner.get("source")
            inherited = not (
                current_ratio is not None
                and abs(current_ratio - ratio) < 1e-9
                and winner is current
            )
        elif not errors and ancestors:
            state = "unlimited"
            effective = None
            source = current.get("source")
            inherited = False
        else:
            state = "unknown"
            effective = None
            source = None
            inherited = None
        return {
            "state": state,
            "raw_current": current.get("raw"),
            "current_quota_us": current.get("quota_us"),
            "current_period_us": current.get("period_us"),
            "current_quota_millicores": current_ratio,
            "effective_quota_millicores": effective,
            "effective_source": source,
            "inherited": inherited,
            "ancestors_complete": not errors,
            "source": current.get("source"),
            "error": errors[0].get("error") if errors else None,
            "version": version,
            "cgroup_path": relative,
        }

    def _v2_cpuset(
        self,
        base: Path,
        cache: dict[tuple[Any, ...], tuple[Optional[str], Optional[str]]],
    ) -> dict[str, Any]:
        current_path = base / "cpuset.cpus"
        effective_path = base / "cpuset.cpus.effective"
        current_text, current_error = self._cached_text(current_path, cache)
        effective_text, effective_error = self._cached_text(effective_path, cache)
        current = parse_cpu_list(current_text)
        effective = parse_cpu_list(effective_text)
        error = None
        if current_error or effective_error:
            error = current_error or effective_error
        return {
            "available": current["valid"] or effective["valid"],
            "source": str(effective_path),
            "current_list": current["list"],
            "current_cpu_count": current["count"],
            "effective_list": effective["list"],
            "effective_cpu_count": effective["count"],
            "error": error,
        }

    def _cgroup_v1(
        self,
        membership: dict[str, Any],
        cache: dict[tuple[Any, ...], tuple[Optional[str], Optional[str]]],
    ) -> dict[str, Any]:
        relative = str(membership["v1_cpu"] or membership["v1_cpuset"] or "/")
        cpu_relative = str(membership["v1_cpu"] or relative)
        cpuset_relative = str(membership["v1_cpuset"] or relative)
        ancestors: list[dict[str, Any]] = []
        ancestor_paths, truncated = _ancestor_paths(cpu_relative)
        for cpath in ancestor_paths:
            quota_path = self._first_v1_file(
                cpath, ("cpu", "cpu,cpuacct"), "cpu.cfs_quota_us", cache
            )
            period_path = self._first_v1_file(
                cpath, ("cpu", "cpu,cpuacct"), "cpu.cfs_period_us", cache
            )
            quota_raw = quota_path[1]
            period_raw = period_path[1]
            quota, period, parse_error = _parse_v1_limit(quota_raw, period_raw)
            error = None
            if quota_path[0] is None:
                error = quota_path[2]
            elif period_path[0] is None:
                error = period_path[2]
            elif parse_error is not None:
                error = parse_error
            ancestors.append(
                {
                    "path": cpath,
                    "source": quota_path[0],
                    "raw": None if quota_raw is None else quota_raw.strip(),
                    "quota_us": quota,
                    "period_us": period,
                    "error": error,
                }
            )
        if truncated:
            ancestors.append(
                {
                    "path": cpu_relative,
                    "source": None,
                    "raw": None,
                    "quota_us": None,
                    "period_us": None,
                    "error": "cgroup ancestry exceeds bounded read depth",
                }
            )
        quota_summary = self._quota_summary(ancestors, "v1", cpu_relative)
        stat_path = self._first_v1_file(
            cpu_relative, ("cpu", "cpu,cpuacct"), "cpu.stat", cache
        )
        if stat_path[0] is None:
            cpu_stat = {
                "available": False,
                "source": stat_path[0],
                "raw": {},
                "nr_periods": None,
                "nr_throttled": None,
                "throttled_usec": None,
                "throttled_period_ratio": None,
                "error": stat_path[2],
            }
        else:
            cpu_stat = _parse_cpu_stat(stat_path[1] or "", v1=True)
            cpu_stat["source"] = stat_path[0]
        cpuset = self._v1_cpuset(cpuset_relative, cache)
        return {
            "version": 1,
            "path": cpu_relative,
            "membership": membership,
            "cpu_quota": quota_summary,
            "cpu_stat": cpu_stat,
            "cpuset": cpuset,
            "error": None,
        }

    def _first_v1_file(
        self,
        cgroup_path: str,
        controllers: tuple[str, ...],
        filename: str,
        cache: dict[tuple[Any, ...], tuple[Optional[str], Optional[str]]],
    ) -> tuple[Optional[str], Optional[str], Optional[str]]:
        last_error = "unreadable"
        for controller in controllers:
            base = _safe_cgroup_path(self.cgroup_root, controller)
            if base is None:
                continue
            directory = _safe_cgroup_path(base, cgroup_path)
            if directory is None:
                continue
            path = directory / filename
            text, error = self._cached_text(path, cache)
            if text is not None:
                return str(path), text, None
            last_error = error
        return None, None, last_error

    def _v1_cpuset(
        self,
        relative: str,
        cache: dict[tuple[Any, ...], tuple[Optional[str], Optional[str]]],
    ) -> dict[str, Any]:
        for filename in ("cpuset.effective_cpus", "cpuset.cpus"):
            path = self._first_v1_file(relative, ("cpuset",), filename, cache)
            if path[0] is not None:
                parsed = parse_cpu_list(path[1])
                return {
                    "available": parsed["valid"],
                    "source": path[0],
                    "current_list": parsed["list"],
                    "current_cpu_count": parsed["count"],
                    "effective_list": parsed["list"],
                    "effective_cpu_count": parsed["count"],
                    "error": None if parsed["valid"] else "cpuset CPU list invalid",
                }
        return {
            "available": False,
            "source": None,
            "current_list": None,
            "current_cpu_count": None,
            "effective_list": None,
            "effective_cpu_count": None,
            "error": "cpuset files unreadable",
        }

    def _unknown_cgroup(
        self, version: int, relative: str, reason: str
    ) -> dict[str, Any]:
        return {
            "version": version,
            "path": relative,
            "membership": None,
            "cpu_quota": {
                "state": "unknown",
                "raw_current": None,
                "current_quota_us": None,
                "current_period_us": None,
                "current_quota_millicores": None,
                "effective_quota_millicores": None,
                "effective_source": None,
                "inherited": None,
                "ancestors_complete": False,
                "source": None,
                "error": reason,
                "version": version,
                "cgroup_path": relative,
            },
            "cpu_stat": {"available": False, "raw": {}, "error": reason},
            "cpuset": {
                "available": False,
                "current_list": None,
                "effective_list": None,
                "effective_cpu_count": None,
                "error": reason,
            },
            "error": reason,
        }

    # -- full sample ----------------------------------------------------
    def _sample_locked(
        self, scope: list[dict[str, Any]], now_mono: float, now_wall: float
    ) -> dict[str, Any]:
        now_mono = float(now_mono)
        now_wall = float(now_wall)
        if not math.isfinite(now_mono) or not math.isfinite(now_wall):
            raise ValueError("sample clocks must be finite")
        errors: list[str] = []
        system, interval = self._system_payload(now_mono, errors)
        stats = self._scan_pid_stats(errors)
        children: dict[int, list[int]] = {}
        for pid, row in stats.items():
            children.setdefault(int(row["ppid"]), []).append(pid)
        for values in children.values():
            values.sort()

        cgroup_cache: dict[tuple[Any, ...], tuple[Optional[str], Optional[str]]] = {}
        assigned: set[int] = set()
        groups: list[dict[str, Any]] = []
        current_group_pids: dict[str, set[int]] = {}
        present_baselines: dict[int, dict[str, Any]] = {}
        next_group_cache: dict[str, dict[int, dict[str, Any]]] = {}
        all_targets: list[str] = []
        managed_targets: list[str] = []

        for raw_spec in scope:
            if not isinstance(raw_spec, dict):
                continue
            target = str(raw_spec.get("target") or "unknown")
            all_targets.append(target)
            kind = str(raw_spec.get("kind") or "managed_ros_launch")
            if kind == "managed_ros_launch":
                managed_targets.append(target)
            roots = self._int_list(raw_spec.get("root_pids"))
            extras = self._int_list(raw_spec.get("extra_pids"))
            include_descendants = bool(
                raw_spec.get("include_descendants", True)
            )
            pgid = None
            try:
                if raw_spec.get("pgid") is not None:
                    pgid = int(raw_spec["pgid"])
            except (TypeError, ValueError):
                pgid = None
            visible_roots = [pid for pid in roots if pid in stats]
            missing_roots = [pid for pid in roots if pid not in stats]
            # ``extra_pids`` comes from the managed stop-path tracker, which
            # retains PIDs after a descendant exits.  Only keep an extra when
            # its current /proc process group still matches the managed pgid;
            # this prevents a recycled PID from being mislabelled as managed.
            seed = list(roots)
            for pid in extras:
                if pid not in stats:
                    continue
                if pgid is not None and int(stats[pid].get("pgrp", -1)) != pgid:
                    continue
                seed.append(pid)
            if include_descendants:
                selected = self._descendants(seed, stats, children)
            else:
                selected = [pid for pid in seed if pid in stats]
            selected = [pid for pid in selected if pid not in assigned]
            assigned.update(selected)
            current_group_pids[target] = set(selected)
            rows: list[dict[str, Any]] = []
            for pid in selected:
                row = self._process_row(
                    stats[pid],
                    target,
                    kind,
                    roots,
                    extras,
                    now_mono,
                    cgroup_cache,
                    errors,
                )
                rows.append(row)
                present_baselines[pid] = {
                    "starttime": row["starttime_ticks"],
                    "ticks": row["_ticks"],
                    "sampled_mono": now_mono,
                }
            next_group_cache[target] = {
                row["pid"]: {
                    "pid": row["pid"],
                    "ppid": row["ppid"],
                    "name": row["name"],
                    "starttime_ticks": row["starttime_ticks"],
                    "threads": row["threads"],
                    "rss_bytes": row["rss_bytes"],
                }
                for row in rows
            }
            if selected:
                status = "running"
            elif roots or extras:
                status = "missing"
            else:
                status = "not_running"
            groups.append(
                {
                    "target": target,
                    "kind": kind,
                    "status": status,
                    "scope": (
                        (
                            "web_process_and_descendants"
                            if include_descendants
                            else "web_process"
                        )
                        if kind == "web_process"
                        else (
                            "rosbag_process_and_descendants"
                            if kind == "rosbag_process"
                            else "managed_ros_launch_descendants"
                        )
                    ),
                    "root_pids": roots,
                    "extra_pids": extras,
                    "root_pids_visible": visible_roots,
                    "root_pids_missing": missing_roots,
                    "pid_rows": rows,
                    "summary": self._process_summary(rows),
                    "error": None,
                }
            )

        # Emit one clearly-marked gone row for processes that left a still-scoped
        # target, instead of silently keeping their last CPU value.
        for group in groups:
            target = group["target"]
            previous = self._previous_groups.get(target, {})
            for pid in sorted(previous):
                if pid in current_group_pids.get(target, set()) or pid in assigned:
                    continue
                cached = previous[pid]
                group["pid_rows"].append(
                    {
                        "target": target,
                        "group_kind": group["kind"],
                        "pid": pid,
                        "ppid": cached.get("ppid"),
                        "name": cached.get("name"),
                        "state": "gone",
                        "available": False,
                        "role": "descendant",
                        "starttime_ticks": cached.get("starttime_ticks"),
                        "identity": (
                            f"pid:{pid}:starttime:{cached.get('starttime_ticks')}"
                        ),
                        "threads": None,
                        "rss_pages": None,
                        "rss_bytes": None,
                        "cpu_percent": None,
                        "cpu_percent_basis": CPU_PERCENT_BASIS,
                        "cpu_unknown_reason": "process_gone",
                        "cpu_ticks_delta": None,
                        "cpu_time_sec": None,
                        "last_cpu_executed": None,
                        "allocation": None,
                    }
                )
                group["summary"] = self._process_summary(group["pid_rows"])

        for group in groups:
            for row in group["pid_rows"]:
                row.pop("_ticks", None)
        self._previous_processes = present_baselines
        self._previous_groups = next_group_cache
        summary = self._overall_summary(groups)
        status = "ok" if not errors else "partial"
        if system["cpu"].get("error") and not any(
            group["pid_rows"] for group in groups
        ):
            status = "error"
        scope_payload = {
            "description": (
                "web_control itself plus the process tree rooted at each "
                "web_control-managed launch; no unmanaged ROS discovery."
            ),
            "included_targets": all_targets,
            "managed_ros_targets": managed_targets,
            "excluded_unmanaged_ros_processes": True,
            "pid_source": "/proc/<pid>/stat scanned once per sample",
            "process_metadata": "pid, ppid, comm/name, state, starttime, threads, RSS",
            "excluded_data": "command lines, environment variables, private arguments",
        }
        return {
            "kind": "resources",
            "source": "resource_sampler",
            "ok": status != "error",
            "status": status,
            "timestamp_unix": now_wall,
            "monotonic_sec": now_mono,
            "interval_sec": None if interval is None else round(interval, 6),
            "sampling": {
                "clock_ticks_per_sec": self.clock_ticks,
                "page_size_bytes": self.page_size,
                "proc_root": str(self.proc_root),
                "cgroup_root": str(self.cgroup_root),
            },
            "cpu_percent_basis": {
                "system": SYSTEM_CPU_BASIS,
                "process": CPU_PERCENT_BASIS,
                "system_normalization": "0..100 across all CPUs",
                "process_normalization": "100%=one CPU core; can exceed 100%",
                "first_sample": None,
            },
            "system": system,
            "groups": groups,
            "summary": summary,
            "scope": scope_payload,
            "measurement_limitations": list(_MEASUREMENT_LIMITATIONS),
            "errors": list(dict.fromkeys(errors)),
        }

    @staticmethod
    def _int_list(value: Any) -> list[int]:
        result = []
        for item in value or []:
            try:
                pid = int(item)
            except (TypeError, ValueError):
                continue
            if pid > 0 and pid not in result:
                result.append(pid)
        return result

    @staticmethod
    def _descendants(
        roots: list[int],
        stats: dict[int, dict[str, Any]],
        children: dict[int, list[int]],
    ) -> list[int]:
        selected: list[int] = []
        seen: set[int] = set()
        queue: deque[int] = deque()
        for pid in roots:
            if pid in stats and pid not in seen:
                seen.add(pid)
                selected.append(pid)
                queue.append(pid)
        while queue:
            parent = queue.popleft()
            for child in children.get(parent, []):
                if child not in seen:
                    seen.add(child)
                    selected.append(child)
                    queue.append(child)
        return selected

    def _process_row(
        self,
        parsed: dict[str, Any],
        target: str,
        kind: str,
        roots: list[int],
        extras: list[int],
        now_mono: float,
        cgroup_cache: dict[tuple[Any, ...], tuple[Optional[str], Optional[str]]],
        errors: list[str],
    ) -> dict[str, Any]:
        pid = int(parsed["pid"])
        starttime = int(parsed["starttime"])
        ticks = int(parsed["utime"]) + int(parsed["stime"])
        previous = self._previous_processes.get(pid)
        cpu_percent = None
        reason = None
        ticks_delta = None
        if previous is None:
            reason = "pid_new"
        elif int(previous.get("starttime", -1)) != starttime:
            reason = "identity_changed"
        else:
            previous_mono = _finite(previous.get("sampled_mono"))
            process_interval = (
                None if previous_mono is None else now_mono - previous_mono
            )
            if (
                process_interval is None
                or not math.isfinite(process_interval)
                or process_interval <= 0
            ):
                reason = "invalid_interval"
            else:
                ticks_delta = ticks - int(previous.get("ticks", 0))
                if ticks_delta < 0:
                    reason = "counter_reset"
                else:
                    cpu_percent = round(
                        (ticks_delta / self.clock_ticks)
                        / process_interval
                        * 100.0,
                        3,
                    )
        if pid in roots:
            if kind == "web_process":
                role = "web_process"
            elif kind == "rosbag_process":
                role = "rosbag_process"
            else:
                role = "launch_root"
        else:
            role = "descendant"
        rss_pages = max(0, int(parsed["rss_pages"]))
        affinity = self._read_affinity(pid)
        cgroup = self._read_cgroup(pid, cgroup_cache)
        if not affinity["readable"] and affinity.get("error"):
            errors.append(f"pid {pid} affinity: {affinity['error']}")
        if cgroup.get("error"):
            errors.append(f"pid {pid} cgroup: {cgroup['error']}")
        elif cgroup.get("cpu_quota", {}).get("error"):
            errors.append(
                f"pid {pid} cgroup quota: {cgroup['cpu_quota']['error']}"
            )
        return {
            "target": target,
            "group_kind": kind,
            "pid": pid,
            "ppid": int(parsed["ppid"]),
            "name": str(parsed["name"]),
            "state": str(parsed["state"]),
            "available": True,
            "role": role,
            "starttime_ticks": starttime,
            "identity": f"pid:{pid}:starttime:{starttime}",
            "threads": int(parsed["threads"]),
            "rss_pages": rss_pages,
            "rss_bytes": rss_pages * self.page_size,
            "cpu_percent": cpu_percent,
            "cpu_percent_basis": CPU_PERCENT_BASIS,
            "cpu_unknown_reason": reason,
            "cpu_ticks_delta": ticks_delta,
            "cpu_time_sec": round(ticks / self.clock_ticks, 3),
            "last_cpu_executed": parsed.get("last_cpu_executed"),
            "allocation": {
                "affinity": affinity,
                "cgroup": cgroup,
                "note": (
                    "Cpus_allowed_list is the main thread's affinity from "
                    "/proc/<pid>/status; other threads may have different "
                    "affinity. last_cpu_executed is not a binding. cgroup "
                    "quota state unknown means the readable files did not "
                    "establish a limit, not that it is unlimited."
                ),
            },
            "_ticks": ticks,
        }

    def _process_summary(self, rows: list[dict[str, Any]]) -> dict[str, Any]:
        present = [row for row in rows if row.get("available")]
        known = [
            float(row["cpu_percent"])
            for row in present
            if row.get("cpu_percent") is not None
        ]
        rss_values = [
            int(row["rss_bytes"])
            for row in present
            if row.get("rss_bytes") is not None
        ]
        thread_values = [
            int(row["threads"])
            for row in present
            if row.get("threads") is not None
        ]
        return {
            "process_count": len(rows),
            "present_count": len(present),
            "gone_count": len(rows) - len(present),
            "cpu_percent_sum": round(sum(known), 3) if known else None,
            "cpu_percent_known_count": len(known),
            "cpu_percent_complete": bool(present) and len(known) == len(present),
            "cpu_percent_sum_basis": (
                "sum of per-PID deltas; 100%=one CPU core; null if no known row"
            ),
            "rss_bytes_sum": sum(rss_values),
            "threads_sum": sum(thread_values),
        }

    def _overall_summary(self, groups: list[dict[str, Any]]) -> dict[str, Any]:
        rows = [row for group in groups for row in group["pid_rows"]]
        present = [row for row in rows if row.get("available")]
        known = [
            float(row["cpu_percent"])
            for row in present
            if row.get("cpu_percent") is not None
        ]
        return {
            "target_count": len(groups),
            "running_target_count": sum(
                1 for group in groups if group["status"] == "running"
            ),
            "process_count": len(present),
            "gone_count": len(rows) - len(present),
            "cpu_percent_sum": round(sum(known), 3) if known else None,
            "cpu_percent_complete": bool(present) and len(known) == len(present),
            "cpu_percent_sum_basis": (
                "sum of monitored per-PID deltas; 100%=one CPU core; null if "
                "no visible process CPU is known"
            ),
            "rss_bytes_sum": sum(
                int(row["rss_bytes"])
                for row in present
                if row.get("rss_bytes") is not None
            ),
            "threads_sum": sum(
                int(row["threads"])
                for row in present
                if row.get("threads") is not None
            ),
            "processes_by_target": {
                group["target"]: group["summary"]["present_count"]
                for group in groups
            },
        }


class ResourceLoggingWorker:
    """One daemon thread that samples and writes at a fixed interval."""

    def __init__(
        self,
        sampler: Any,
        scope_provider: Callable[[], Iterable[dict[str, Any]]],
        write_entry: Callable[[dict[str, Any]], bool],
        *,
        period_sec: float = DEFAULT_SAMPLE_PERIOD_SEC,
        logger: Any = None,
        clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        self.sampler = sampler
        self.scope_provider = scope_provider
        self.write_entry = write_entry
        self.period_sec = max(0.01, float(period_sec))
        self.logger = logger
        self.clock = clock
        self.wall_clock = wall_clock
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
        self.error: Optional[str] = None
        self.samples = 0
        self._last_logged_error: Optional[str] = None

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._run,
                name="web_control_resource_logging",
                daemon=True,
            )
            self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        thread = self._thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=max(0.0, float(timeout)))

    def is_alive(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def _run(self) -> None:
        while not self._stop.is_set():
            started = self.clock()
            now_mono = started
            now_wall = self.wall_clock()
            try:
                scope = list(self.scope_provider() or [])
                entry = self.sampler.sample(scope, now_mono, now_wall)
                if self.write_entry(entry) is False:
                    self.error = "resource diagnostics write rejected"
                else:
                    self.samples += 1
                    self.error = None
            except Exception as exc:  # noqa: BLE001 - logging must not crash control
                self.error = f"resource diagnostics sample failed: {exc}"
                self._log_error(self.error)
                try:
                    self.write_entry(
                        {
                            "kind": "resources",
                            "source": "resource_sampler",
                            "ok": False,
                            "status": "error",
                            "timestamp_unix": now_wall,
                            "monotonic_sec": now_mono,
                            "error": str(exc),
                            "system": None,
                            "groups": [],
                            "summary": None,
                            "scope": {},
                            "measurement_limitations": list(
                                _MEASUREMENT_LIMITATIONS
                            ),
                        }
                    )
                except Exception as write_exc:  # noqa: BLE001 - bounded recovery
                    self._log_error(
                        f"resource diagnostics error entry failed: {write_exc}"
                    )
            elapsed = self.clock() - started
            self._stop.wait(max(0.0, self.period_sec - elapsed))

    def _log_error(self, message: str) -> None:
        if self.logger is None or message == self._last_logged_error:
            return
        self._last_logged_error = message
        try:
            self.logger.error(message)
        except Exception:
            pass
