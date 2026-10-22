"""Deterministic tests for ROS-free periodic resource diagnostics logging."""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest

from web_control.diagnostics import DiagnosticsLogWriter
from web_control.log_store import ManagedLogStore
from web_control.resource_diagnostics import (
    ResourceLoggingWorker,
    ResourceSampler,
    parse_pid_stat,
)
from web_control import web_control_node


STAT_1 = """cpu  100 0 100 800 0 0 0 0 0 0
cpu0 50 0 50 400 0 0 0 0 0 0
cpu1 50 0 50 400 0 0 0 0 0 0
intr 1
"""
STAT_2 = """cpu  150 0 150 900 0 0 0 0 0 0
cpu0 75 0 75 450 0 0 0 0 0 0
cpu1 75 0 75 450 0 0 0 0 0 0
intr 2
"""
STAT_GUEST_1 = """cpu  100 0 0 100 0 0 0 0 50 0
cpu0 50 0 0 50 0 0 0 0 25 0
cpu1 50 0 0 50 0 0 0 0 25 0
intr 1
"""
STAT_GUEST_2 = """cpu  200 0 0 200 0 0 0 0 150 0
cpu0 100 0 0 100 0 0 0 0 75 0
cpu1 100 0 0 100 0 0 0 0 75 0
intr 2
"""
MEMINFO = """MemTotal:       1024 kB
MemFree:         256 kB
MemAvailable:    512 kB
SwapTotal:       256 kB
SwapFree:        128 kB
"""
LOADAVG = "1.50 0.50 0.25 2/100 1234\n"


def _stat_text(
    pid: int,
    comm: str,
    *,
    ppid: int = 1,
    utime: int = 0,
    stime: int = 0,
    threads: int = 1,
    starttime: int = 100,
    rss_pages: int = 12,
    state: str = "S",
    processor: int = 0,
    pgrp: int = 0,
) -> str:
    after = ["S", str(ppid), str(pgrp)]
    after += ["0"] * 8
    after += [str(utime), str(stime)]
    after += ["0", "0"]
    after += ["20", "0"]
    after += [str(threads), "0"]
    after += [str(starttime), "0", str(rss_pages)]
    after += ["0"] * 14
    after += [str(processor)]
    after += ["0"] * 12
    after[0] = state
    assert len(after) == 49
    return f"{pid} ({comm}) " + " ".join(after) + "\n"


def _write_pid(
    proc: Path,
    pid: int,
    comm: str,
    *,
    ppid: int = 1,
    utime: int = 0,
    stime: int = 0,
    threads: int = 1,
    starttime: int = 100,
    rss_pages: int = 12,
    cgroup: str | None = None,
    cpus_allowed: str = "0-3",
    status: bool = True,
    pgrp: int = 0,
) -> None:
    directory = proc / str(pid)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "stat").write_text(
        _stat_text(
            pid,
            comm,
            ppid=ppid,
            utime=utime,
            stime=stime,
            threads=threads,
            starttime=starttime,
            rss_pages=rss_pages,
            pgrp=pgrp,
        ),
        encoding="utf-8",
    )
    if status:
        (directory / "status").write_text(
            f"Name:\t{comm}\nCpus_allowed_list:\t{cpus_allowed}\n"
            "Cpus_allowed:\t0000000f\n",
            encoding="utf-8",
        )
    if cgroup is not None:
        (directory / "cgroup").write_text(cgroup, encoding="utf-8")


def _base_fixture(tmp_path: Path) -> tuple[Path, Path]:
    proc = tmp_path / "proc"
    sys_root = tmp_path / "sys"
    proc.mkdir()
    sys_root.mkdir()
    (proc / "stat").write_text(STAT_1, encoding="utf-8")
    (proc / "meminfo").write_text(MEMINFO, encoding="utf-8")
    (proc / "loadavg").write_text(LOADAVG, encoding="utf-8")
    _write_pid(proc, 100, "web_control_node", ppid=1, starttime=10, cpus_allowed="0-7")
    return proc, sys_root


def _sampler(proc: Path, sys_root: Path) -> ResourceSampler:
    return ResourceSampler(
        proc_root=proc, cgroup_root=sys_root, page_size=4096, clock_ticks=100
    )


def _scope(*groups):
    return list(groups)


def _group(target="localization", roots=(400,), kind="managed_ros_launch", extra=()):
    return {
        "target": target,
        "kind": kind,
        "root_pids": list(roots),
        "extra_pids": list(extra),
    }


def _row(entry, target, pid):
    group = next(item for item in entry["groups"] if item["target"] == target)
    return next(row for row in group["pid_rows"] if row["pid"] == pid), group


def test_system_cpu_delta_per_core_memory_bytes_and_loadavg(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    sampler = _sampler(proc, sys_root)
    first = sampler.sample(_scope(_group()), 10.0, 1000.0)
    assert first["kind"] == "resources"
    assert first["timestamp_unix"] == 1000.0
    assert first["interval_sec"] is None
    assert first["system"]["cpu"]["total_percent"] is None
    assert first["system"]["cpu"]["unknown_reason"] == "first_sample"
    assert [core["percent"] for core in first["system"]["cpu"]["per_core"]] == [
        None,
        None,
    ]
    assert first["system"]["memory"]["total_bytes"] == 1024 * 1024
    assert first["system"]["memory"]["used_bytes"] == 512 * 1024
    assert first["system"]["memory"]["available_bytes"] == 512 * 1024
    assert first["system"]["memory"]["percent"] == pytest.approx(50.0)
    assert first["system"]["swap"]["total_bytes"] == 256 * 1024
    assert first["system"]["swap"]["used_bytes"] == 128 * 1024
    assert first["system"]["loadavg"]["load1"] == pytest.approx(1.5)
    assert first["system"]["loadavg"]["running"] == 2
    assert first["system"]["loadavg"]["total_tasks"] == 100
    assert first["sampling"]["clock_ticks_per_sec"] == 100
    assert first["sampling"]["page_size_bytes"] == 4096

    (proc / "stat").write_text(STAT_2, encoding="utf-8")
    second = sampler.sample(_scope(_group()), 11.0, 1001.0)
    assert second["interval_sec"] == pytest.approx(1.0)
    assert second["system"]["cpu"]["total_percent"] == pytest.approx(50.0)
    assert second["system"]["cpu"]["unknown_reason"] is None
    assert [core["percent"] for core in second["system"]["cpu"]["per_core"]] == [
        50.0,
        50.0,
    ]
    assert second["system"]["cpu"]["per_core"][0]["id"] == 0


def test_system_cpu_excludes_guest_ticks_already_in_user_and_nice(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    # guest is already counted in user, and guest_nice in nice.  Summing all
    # /proc/stat columns double-counts them (66.67% instead of 50% here).
    (proc / "stat").write_text(STAT_GUEST_1, encoding="utf-8")
    sampler = _sampler(proc, sys_root)
    first = sampler.sample(_scope(_group()), 10.0, 1000.0)
    assert first["system"]["cpu"]["total_percent"] is None

    (proc / "stat").write_text(STAT_GUEST_2, encoding="utf-8")
    second = sampler.sample(_scope(_group()), 11.0, 1001.0)
    assert second["system"]["cpu"]["total_percent"] == pytest.approx(50.0)
    assert [core["id"] for core in second["system"]["cpu"]["per_core"]] == [0, 1]
    assert [core["percent"] for core in second["system"]["cpu"]["per_core"]] == [
        50.0,
        50.0,
    ]


def test_process_cpu_is_normalized_to_one_core_not_machine_total(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    _write_pid(proc, 400, "ros2 launch", utime=100, stime=0, threads=2)
    sampler = _sampler(proc, sys_root)
    first = sampler.sample(_scope(_group()), 10.0, 1000.0)
    row, _ = _row(first, "localization", 400)
    assert row["cpu_percent"] is None
    assert row["cpu_unknown_reason"] == "pid_new"

    # 250 ticks in one second at 100 Hz is 2.5 CPU cores, i.e. 250%.
    _write_pid(proc, 400, "ros2 launch", utime=350, stime=0, threads=2)
    second = sampler.sample(_scope(_group()), 11.0, 1001.0)
    row, group = _row(second, "localization", 400)
    assert row["cpu_percent"] == pytest.approx(250.0)
    assert row["cpu_percent_basis"] == "process_delta_ticks"
    assert group["summary"]["cpu_percent_sum"] == pytest.approx(250.0)


def test_process_cpu_interval_survives_repeated_system_stat_failure(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    _write_pid(proc, 400, "localization", utime=10, starttime=100)
    sampler = _sampler(proc, sys_root)
    sampler.sample(_scope(_group()), 10.0, 1000.0)
    (proc / "stat").unlink()
    _write_pid(proc, 400, "localization", utime=110, starttime=100)
    first_failure = sampler.sample(_scope(_group()), 11.0, 1001.0)
    row, _ = _row(first_failure, "localization", 400)
    assert row["cpu_percent"] == pytest.approx(100.0)
    # The system CPU baseline is still the last readable sample, but process
    # CPU must use the actual one-second interval since its own last sample.
    _write_pid(proc, 400, "localization", utime=210, starttime=100)
    second_failure = sampler.sample(_scope(_group()), 12.0, 1002.0)
    row, _ = _row(second_failure, "localization", 400)
    assert row["cpu_percent"] == pytest.approx(100.0)


def test_process_cpu_uses_pid_delta_when_system_stat_is_missing_then_restored(
    tmp_path,
):
    proc, sys_root = _base_fixture(tmp_path)
    _write_pid(proc, 400, "localization", utime=10, starttime=100)
    sampler = _sampler(proc, sys_root)
    sampler.sample(_scope(_group()), 10.0, 1000.0)

    (proc / "stat").unlink()
    _write_pid(proc, 400, "localization", utime=110, starttime=100)
    missing = sampler.sample(_scope(_group()), 11.0, 1001.0)
    row, _ = _row(missing, "localization", 400)
    assert row["cpu_percent"] == pytest.approx(100.0)
    assert missing["system"]["cpu"]["total_percent"] is None
    assert missing["system"]["cpu"]["unknown_reason"] == "unreadable"

    # Restoring /proc/stat continues the *system* baseline across the missing
    # sample (2 s), while this PID's CPU still uses its own 1 s interval.
    (proc / "stat").write_text(STAT_2, encoding="utf-8")
    _write_pid(proc, 400, "localization", utime=210, starttime=100)
    restored = sampler.sample(_scope(_group()), 12.0, 1002.0)
    row, _ = _row(restored, "localization", 400)
    assert row["cpu_percent"] == pytest.approx(100.0)
    assert restored["interval_sec"] == pytest.approx(2.0)
    assert restored["system"]["cpu"]["total_percent"] == pytest.approx(50.0)
    assert [core["percent"] for core in restored["system"]["cpu"]["per_core"]] == [
        50.0,
        50.0,
    ]


def test_pid_stat_comm_with_spaces_parentheses_and_raw_fields(tmp_path):
    text = _stat_text(
        777,
        "lio (planner) node",
        ppid=55,
        utime=12,
        stime=3,
        threads=9,
        starttime=4242,
        rss_pages=7,
        processor=5,
    )
    parsed = parse_pid_stat(text)
    assert parsed["pid"] == 777
    assert parsed["name"] == "lio (planner) node"
    assert parsed["ppid"] == 55
    assert parsed["utime"] == 12
    assert parsed["stime"] == 3
    assert parsed["threads"] == 9
    assert parsed["starttime"] == 4242
    assert parsed["rss_pages"] == 7
    assert parsed["last_cpu_executed"] == 5


def test_pid_reuse_counter_reset_and_gone_use_null_not_zero(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    _write_pid(proc, 400, "localization", utime=10, starttime=100)
    sampler = _sampler(proc, sys_root)
    sampler.sample(_scope(_group()), 10.0, 1000.0)
    _write_pid(proc, 400, "localization", utime=30, starttime=100)
    second = sampler.sample(_scope(_group()), 11.0, 1001.0)
    row, _ = _row(second, "localization", 400)
    assert row["cpu_percent"] == pytest.approx(20.0)

    # Same PID, different starttime: the old baseline must not be reused.
    _write_pid(proc, 400, "localization", utime=50, starttime=200)
    reused = sampler.sample(_scope(_group()), 12.0, 1002.0)
    row, _ = _row(reused, "localization", 400)
    assert row["cpu_percent"] is None
    assert row["cpu_unknown_reason"] == "identity_changed"

    # Counters going backwards are a reset, not a negative or zero sample.
    _write_pid(proc, 400, "localization", utime=10, starttime=200)
    reset = sampler.sample(_scope(_group()), 13.0, 1003.0)
    row, _ = _row(reset, "localization", 400)
    assert row["cpu_percent"] is None
    assert row["cpu_unknown_reason"] == "counter_reset"

    (proc / "400" / "stat").unlink()
    gone = sampler.sample(_scope(_group()), 14.0, 1004.0)
    row, group = _row(gone, "localization", 400)
    assert row["state"] == "gone"
    assert row["available"] is False
    assert row["cpu_percent"] is None
    assert row["cpu_unknown_reason"] == "process_gone"
    assert row["rss_bytes"] is None
    assert group["summary"]["gone_count"] == 1
    assert group["status"] == "missing"


def test_managed_scope_includes_actual_descendants_only(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    _write_pid(proc, 400, "ros2 launch localization", ppid=1, utime=10, starttime=1)
    _write_pid(proc, 401, "lio_sam_node", ppid=400, utime=20, starttime=2, threads=8)
    _write_pid(proc, 402, "lio_sam_rviz", ppid=401, utime=30, starttime=3)
    _write_pid(proc, 403, "unrelated_service", ppid=1, utime=40, starttime=4)
    _write_pid(proc, 500, "external_ros_node", ppid=1, utime=50, starttime=5)
    sampler = _sampler(proc, sys_root)
    scope = _scope(
        _group("web_control", (100,), kind="web_process"),
        _group("localization", (400,)),
    )
    first = sampler.sample(scope, 10.0, 1000.0)
    localization = next(
        group for group in first["groups"] if group["target"] == "localization"
    )
    pids = {row["pid"] for row in localization["pid_rows"]}
    assert pids == {400, 401, 402}
    assert 403 not in pids and 500 not in pids
    node_row, _ = _row(first, "localization", 401)
    assert node_row["name"] == "lio_sam_node"
    assert node_row["ppid"] == 400
    assert node_row["threads"] == 8
    assert node_row["role"] == "descendant"
    root_row, _ = _row(first, "localization", 400)
    assert root_row["role"] == "launch_root"
    web_row, _ = _row(first, "web_control", 100)
    assert web_row["role"] == "web_process"
    assert first["scope"]["excluded_unmanaged_ros_processes"] is True


def test_tracked_extra_pids_must_stay_in_the_managed_process_group(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    _write_pid(proc, 400, "ros2 launch", ppid=1, pgrp=400, starttime=1)
    _write_pid(proc, 404, "reused_external_pid", ppid=1, pgrp=999, starttime=2)
    _write_pid(proc, 405, "reparented_managed_node", ppid=1, pgrp=400, starttime=3)
    sampler = _sampler(proc, sys_root)
    first = sampler.sample(
        _scope(
            {
                "target": "localization",
                "kind": "managed_ros_launch",
                "root_pids": [400],
                "extra_pids": [404, 405],
                "pgid": 400,
            }
        ),
        10.0,
        1000.0,
    )
    pids = {row["pid"] for row in first["groups"][0]["pid_rows"]}
    assert 405 in pids
    assert 404 not in pids


def test_v2_cgroup_quota_throttling_cpuset_and_inherited_effective_limit(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    _write_pid(
        proc,
        400,
        "lio_sam_node",
        cgroup="0::/test.slice\n",
        cpus_allowed="0-1",
    )
    (sys_root / "cpu.max").write_text("max 100000\n", encoding="utf-8")
    (sys_root / "test.slice").mkdir()
    (sys_root / "test.slice" / "cpu.max").write_text(
        "200000 100000\n", encoding="utf-8"
    )
    (sys_root / "test.slice" / "cpu.stat").write_text(
        "usage_usec 1000000\nnr_periods 100\nnr_throttled 10\nthrottled_usec 5000\n",
        encoding="utf-8",
    )
    (sys_root / "test.slice" / "cpuset.cpus").write_text("0-1\n", encoding="utf-8")
    (sys_root / "test.slice" / "cpuset.cpus.effective").write_text(
        "0-3\n", encoding="utf-8"
    )
    sampler = _sampler(proc, sys_root)
    first = sampler.sample(_scope(_group()), 10.0, 1000.0)
    row, _ = _row(first, "localization", 400)
    allocation = row["allocation"]
    assert allocation["affinity"]["cpus_allowed_list"] == "0-1"
    assert allocation["affinity"]["cpu_count"] == 2
    cgroup = allocation["cgroup"]
    assert cgroup["version"] == 2
    assert cgroup["path"] == "/test.slice"
    quota = cgroup["cpu_quota"]
    assert quota["state"] == "limited"
    assert quota["raw_current"] == "200000 100000"
    assert quota["current_quota_millicores"] == pytest.approx(2000.0)
    assert quota["effective_quota_millicores"] == pytest.approx(2000.0)
    assert quota["inherited"] is False
    assert cgroup["cpu_stat"]["nr_periods"] == 100
    assert cgroup["cpu_stat"]["nr_throttled"] == 10
    assert cgroup["cpu_stat"]["throttled_usec"] == 5000
    assert cgroup["cpu_stat"]["throttled_period_ratio"] == pytest.approx(0.1)
    assert cgroup["cpuset"]["current_list"] == "0-1"
    assert cgroup["cpuset"]["effective_list"] == "0-3"
    assert cgroup["cpuset"]["effective_cpu_count"] == 4

    # A parent quota is detected as the effective inherited limit.
    (sys_root / "test.slice" / "cpu.max").write_text(
        "max 100000\n", encoding="utf-8"
    )
    (sys_root / "cpu.max").write_text("50000 100000\n", encoding="utf-8")
    second = sampler.sample(_scope(_group()), 11.0, 1001.0)
    row, _ = _row(second, "localization", 400)
    quota = row["allocation"]["cgroup"]["cpu_quota"]
    assert quota["state"] == "limited"
    assert quota["raw_current"] == "max 100000"
    assert quota["effective_quota_millicores"] == pytest.approx(500.0)
    assert quota["inherited"] is True
    assert quota["effective_source"].endswith("/cpu.max")


def test_deep_cgroup_ancestry_is_unknown_not_claimed_unlimited(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    relative = "/" + "/".join(f"level{index}" for index in range(20))
    _write_pid(proc, 400, "localization", cgroup=f"0::{relative}\n")
    # The first 16 readable levels say "max"; the unread deeper levels could
    # still contain a limit, so the bounded walk must not report unlimited.
    (sys_root / "cpu.max").write_text("max 100000\n", encoding="utf-8")
    current = sys_root
    for index in range(16):
        current = current / f"level{index}"
        current.mkdir()
        (current / "cpu.max").write_text("max 100000\n", encoding="utf-8")
    sampler = _sampler(proc, sys_root)
    entry = sampler.sample(_scope(_group()), 10.0, 1000.0)
    row, _ = _row(entry, "localization", 400)
    quota = row["allocation"]["cgroup"]["cpu_quota"]
    assert quota["state"] == "unknown"
    assert quota["ancestors_complete"] is False


def test_v1_cpu_quota_cpuset_and_unknown_quota_is_not_unlimited(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    _write_pid(
        proc,
        400,
        "navigation_node",
        cgroup="5:cpu,cpuacct:/test.service\n4:cpuset:/test.service\n",
    )
    cpu_dir = sys_root / "cpu" / "test.service"
    cpu_dir.mkdir(parents=True)
    (cpu_dir / "cpu.cfs_quota_us").write_text("150000\n", encoding="utf-8")
    (cpu_dir / "cpu.cfs_period_us").write_text("100000\n", encoding="utf-8")
    (cpu_dir / "cpu.stat").write_text(
        "nr_periods 50\nnr_throttled 5\nthrottled_time 1000000000\n",
        encoding="utf-8",
    )
    cpuset_dir = sys_root / "cpuset" / "test.service"
    cpuset_dir.mkdir(parents=True)
    (cpuset_dir / "cpuset.effective_cpus").write_text("0-1\n", encoding="utf-8")
    sampler = _sampler(proc, sys_root)
    first = sampler.sample(_scope(_group()), 10.0, 1000.0)
    row, _ = _row(first, "localization", 400)
    cgroup = row["allocation"]["cgroup"]
    assert cgroup["version"] == 1
    assert cgroup["cpu_quota"]["state"] == "limited"
    assert cgroup["cpu_quota"]["current_quota_millicores"] == pytest.approx(1500.0)
    assert cgroup["cpu_stat"]["throttled_usec"] == 1000000
    assert cgroup["cpuset"]["effective_list"] == "0-1"

    # Missing quota files must be unknown, never "unlimited".
    (cpu_dir / "cpu.cfs_quota_us").unlink()
    (cpu_dir / "cpu.cfs_period_us").unlink()
    (sys_root / "cpu" / "cpu.cfs_quota_us").unlink(missing_ok=True)
    (sys_root / "cpu" / "cpu.cfs_period_us").unlink(missing_ok=True)
    second = sampler.sample(_scope(_group()), 11.0, 1001.0)
    row, _ = _row(second, "localization", 400)
    quota = row["allocation"]["cgroup"]["cpu_quota"]
    assert quota["state"] == "unknown"
    assert quota["raw_current"] is None
    assert quota["effective_quota_millicores"] is None


def test_unreadable_status_degrades_allocation_without_losing_cpu_row(tmp_path):
    proc, sys_root = _base_fixture(tmp_path)
    _write_pid(proc, 400, "localization", utime=10, status=False)
    (proc / "400" / "status").mkdir()
    sampler = _sampler(proc, sys_root)
    entry = sampler.sample(_scope(_group()), 10.0, 1000.0)
    row, _ = _row(entry, "localization", 400)
    assert row["available"] is True
    assert row["allocation"]["affinity"]["readable"] is False
    assert row["allocation"]["affinity"]["error"]
    assert entry["status"] == "partial"


def test_resource_sampler_has_independent_cpu_baseline_from_http_payload(tmp_path):
    from web_control.core import build_system_usage_payload

    proc, sys_root = _base_fixture(tmp_path)
    sampler = _sampler(proc, sys_root)
    sampler.sample(_scope(_group()), 10.0, 1000.0)
    # HTTP/UI-style polling repeatedly advances a separate previous-tick value.
    http_previous = None
    for _index in range(4):
        _payload, http_previous = build_system_usage_payload(
            STAT_2, MEMINFO, http_previous, 1001.0
        )
    (proc / "stat").write_text(STAT_2, encoding="utf-8")
    entry = sampler.sample(_scope(_group()), 11.0, 1001.0)
    assert entry["system"]["cpu"]["total_percent"] == pytest.approx(50.0)
    # The sampler's first sample is still unknown even though HTTP polled first.
    fresh = _sampler(proc, sys_root).sample(_scope(_group()), 20.0, 2000.0)
    assert fresh["system"]["cpu"]["total_percent"] is None


class _Logger:
    def __init__(self):
        self.errors = []

    def info(self, *_a, **_k):
        pass

    def warning(self, *_a, **_k):
        pass

    def error(self, message, *_a, **_k):
        self.errors.append(str(message))

    def debug(self, *_a, **_k):
        pass


class _SamplerStub:
    def __init__(self):
        self.calls = 0

    def sample(self, scope, now_mono, now_wall):
        self.calls += 1
        return {
            "kind": "resources",
            "timestamp_unix": now_wall,
            "monotonic_sec": now_mono,
            "sample_number": self.calls,
        }


def test_background_worker_lifecycle_and_write_failure_does_not_crash():
    entries = []
    sampler = _SamplerStub()
    worker = ResourceLoggingWorker(
        sampler,
        lambda: _scope(_group()),
        lambda entry: entries.append(entry) or True,
        period_sec=0.02,
        logger=_Logger(),
    )
    worker.start()
    deadline = time.monotonic() + 2.0
    while len(entries) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    worker.stop()
    count = len(entries)
    assert count >= 2
    assert not worker.is_alive()
    time.sleep(0.08)
    assert len(entries) == count

    failures = _Logger()
    bad = ResourceLoggingWorker(
        sampler,
        lambda: _scope(_group()),
        lambda entry: (_ for _ in ()).throw(OSError("disk full")),
        period_sec=0.02,
        logger=failures,
    )
    bad.start()
    deadline = time.monotonic() + 2.0
    while bad.error is None and time.monotonic() < deadline:
        time.sleep(0.01)
    bad.stop()
    assert bad.error is not None
    assert failures.errors


class _RecordingWriter:
    def __init__(self):
        self.entries = []
        self.closed = False
        self.error = None
        self.session_id = "resource-session"

    def write(self, entry, now=None):
        if self.closed:
            raise AssertionError("write after diagnostics log close")
        self.entries.append(dict(entry))
        return True

    def status(self):
        return {"error": self.error, "active_session": self.session_id}

    def close(self):
        self.closed = True


def test_resources_persist_when_topic_logging_is_off_and_bypass_topic_filter(tmp_path):
    store = ManagedLogStore(tmp_path / "logs")
    writer = DiagnosticsLogWriter(store)
    node = SimpleNamespace(
        diagnostics_log=writer,
        topic_logging_enabled=False,
        _diagnostics_lock=threading.RLock(),
        _diagnostics_log_error=None,
        _resource_logging_active=True,
        _diagnostics_closed=False,
        get_logger=lambda: _Logger(),
    )
    for name in ("_write_diagnostics", "_write_resources"):
        setattr(
            node,
            name,
            MethodType(getattr(web_control_node.WebControlNode, name), node),
        )
    node._write_diagnostics(
        {"kind": "snapshot", "topics": [{"topic": "/imu"}], "motion": {"state": "idle"}},
        time.monotonic(),
    )
    resource_now = time.time()
    resource_row = {
        "kind": "resources",
        "source": "resource_sampler",
        "timestamp_unix": resource_now,
        "system": {"cpu": {"total_percent": 12.5}},
        "groups": [{"target": "localization", "pid_rows": [{"pid": 401}]}],
    }
    assert node._write_resources(resource_row) is True
    session_id = writer.status()["active_session"]
    writer.close()
    records = [
        json.loads(line)
        for line in store.resolve_session_file("diagnostics", session_id)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.startswith("{")
    ]
    resources = [record for record in records if record.get("kind") == "resources"]
    assert len(resources) == 1
    assert resources[0]["timestamp_unix"] == pytest.approx(resource_now)
    assert resources[0]["at"] == pytest.approx(resource_now)
    assert resources[0]["at_iso"].endswith("+00:00")
    assert resources[0]["groups"][0]["target"] == "localization"
    snapshots = [record for record in records if record.get("kind") == "snapshot"]
    assert snapshots and "topics" not in snapshots[0]


def test_shutdown_stops_worker_before_closing_log_and_rejects_late_writes():
    calls = []
    writer = _RecordingWriter()
    node = SimpleNamespace(
        diagnostics_log=writer,
        topic_logging_enabled=False,
        _diagnostics_lock=threading.RLock(),
        _diagnostics_log_error=None,
        _resource_logging_active=True,
        _diagnostics_closed=False,
        diagnostics_service=SimpleNamespace(
            tick=lambda now: ({"motion": {"state": "idle"}, "topics": []}, [])
        ),
        get_logger=lambda: _Logger(),
    )
    for name in ("_write_diagnostics", "_write_resources", "shutdown_diagnostics"):
        setattr(
            node,
            name,
            MethodType(getattr(web_control_node.WebControlNode, name), node),
        )
    node.stop_resource_logging = lambda: calls.append("stop_resources")

    original_close = writer.close

    def close_log():
        calls.append("close_log")
        original_close()

    writer.close = close_log
    node.shutdown_diagnostics()
    assert calls.index("stop_resources") < calls.index("close_log")
    assert writer.closed is True
    assert node._write_resources({"kind": "resources"}) is False


def test_worker_failure_is_persisted_as_error_status_entry(tmp_path):
    store = ManagedLogStore(tmp_path / "logs")
    writer = DiagnosticsLogWriter(store)
    node = SimpleNamespace(
        diagnostics_log=writer,
        topic_logging_enabled=False,
        _diagnostics_lock=threading.RLock(),
        _diagnostics_log_error=None,
        _resource_logging_active=True,
        _diagnostics_closed=False,
        get_logger=lambda: _Logger(),
    )
    node._write_resources = MethodType(
        web_control_node.WebControlNode._write_resources, node
    )

    def broken_sampler(scope, now_mono, now_wall):
        raise OSError("proc unavailable")

    worker = ResourceLoggingWorker(
        SimpleNamespace(sample=broken_sampler),
        lambda: _scope(_group()),
        lambda entry: node._write_resources(entry),
        period_sec=0.02,
        logger=_Logger(),
    )
    worker.start()
    deadline = time.monotonic() + 2.0
    while writer.last_entry_at is None and time.monotonic() < deadline:
        time.sleep(0.01)
    worker.stop()
    session_id = writer.status()["active_session"]
    writer.close()
    records = [
        json.loads(line)
        for line in store.resolve_session_file("diagnostics", session_id)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.startswith("{")
    ]
    errors = [
        record
        for record in records
        if record.get("kind") == "resources" and record.get("ok") is False
    ]
    assert errors
    assert "proc unavailable" in errors[0]["error"]
    assert errors[0]["status"] == "error"
    assert errors[0]["at_iso"].endswith("+00:00")


@pytest.mark.skipif(not Path("/proc/stat").is_file(), reason="/proc is unavailable")
def test_current_proc_smoke_never_asserts_board_values():
    sampler = ResourceSampler()
    entry = sampler.sample(
        _scope(_group("web_control", (os.getpid(),), kind="web_process")),
        time.monotonic(),
        time.time(),
    )
    assert entry["kind"] == "resources"
    assert entry["system"]["cpu"]["total_percent"] is None or 0.0 <= entry[
        "system"
    ]["cpu"]["total_percent"] <= 100.0
    group = entry["groups"][0]
    assert group["target"] == "web_control"
    assert any(row["pid"] == os.getpid() for row in group["pid_rows"])
