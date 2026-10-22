#!/usr/bin/env python3
"""Synthetic, isolated PointCloud2 decode benchmark.

Compares the historical per-point Python path (read_points-equivalent
``struct`` unpack + isfinite + list downsampling) with the vectorised NumPy
decoder used by the visualization worker.

This is a CPU/method sanity check on synthetic data, not real-robot CPU data.
It performs a few short iterations with a warmup and changes no ROS state.

Usage:
    python3 benchmarks/pointcloud_benchmark.py --points 200000 --iterations 5
"""

from __future__ import annotations

import argparse
import gc
import json
import math
import os
import platform
import struct
import sys
import time
import tracemalloc
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from web_control.core import downsample_points  # noqa: E402
from web_control.pointcloud_runtime import parse_pointcloud2  # noqa: E402

FLOAT32 = 7


def _field(name: str, offset: int) -> SimpleNamespace:
    return SimpleNamespace(name=name, offset=offset, datatype=FLOAT32, count=1)


def _make_cloud(point_count: int) -> SimpleNamespace:
    """Build one 200k-scale XYZ F32 cloud with ~1% non-finite samples."""
    rows = []
    for index in range(point_count):
        if index % 97 == 0:
            x = float("nan")
        elif index % 193 == 0:
            x = float("inf")
        else:
            x = index * 0.001
        y = math.sin(index * 0.013) * 10.0
        z = math.cos(index * 0.017) * 2.0
        rows.append(struct.pack("<fff", x, y, z))
    data = b"".join(rows)
    return SimpleNamespace(
        fields=[_field("x", 0), _field("y", 4), _field("z", 8)],
        point_step=12,
        row_step=point_count * 12,
        width=point_count,
        height=1,
        is_bigendian=False,
        data=data,
        header=SimpleNamespace(frame_id="map"),
    )


def _old_decode(msg: SimpleNamespace, max_points: int):
    """Equivalent to the historical read_points + isfinite + downsample loop."""
    points: list[tuple[float, float, float]] = []
    unpack = struct.Struct("<fff").unpack_from
    data = msg.data
    stride = msg.point_step
    for index in range(msg.width * msg.height):
        x, y, z = unpack(data, index * stride)
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    sampled = downsample_points(points, max_points)
    return [[x, y, z] for x, y, z in sampled], len(points)


def _measure(fn, msg, max_points, iterations):
    gc.collect()
    wall = 0.0
    cpu = 0.0
    result = None
    for _ in range(iterations):
        wall_start = time.perf_counter()
        cpu_start = time.process_time()
        result = fn(msg, max_points)
        wall += time.perf_counter() - wall_start
        cpu += time.process_time() - cpu_start
    return result, wall / iterations, cpu / iterations


def _peak_memory(fn, msg, max_points):
    gc.collect()
    tracemalloc.start()
    try:
        fn(msg, max_points)
        _current, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    return peak


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--points", type=int, default=200_000)
    parser.add_argument("--iterations", type=int, default=5)
    parser.add_argument("--max-points", type=int, default=60_000)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if args.points < 1 or args.iterations < 1:
        parser.error("--points and --iterations must be positive")

    msg = _make_cloud(args.points)
    # Warm up imports, NumPy views and allocator paths before measurement.
    old_warm, _, _ = _measure(_old_decode, msg, args.max_points, 1)
    new_warm, _, _ = _measure(parse_pointcloud2, msg, args.max_points, 1)
    assert old_warm == new_warm, "decoder output differs from the old algorithm"

    old_result, old_wall, old_cpu = _measure(
        _old_decode, msg, args.max_points, args.iterations
    )
    new_result, new_wall, new_cpu = _measure(
        parse_pointcloud2, msg, args.max_points, args.iterations
    )
    assert old_result == new_result
    old_peak = _peak_memory(_old_decode, msg, args.max_points)
    new_peak = _peak_memory(parse_pointcloud2, msg, args.max_points)

    result = {
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "cpu": platform.processor() or "unknown",
            "numpy": __import__("numpy").__version__,
            "pid": os.getpid(),
        },
        "method": {
            "kind": "isolated synthetic PointCloud2, not robot CPU",
            "points": args.points,
            "iterations": args.iterations,
            "warmup_iterations": 1,
            "max_points": args.max_points,
            "old_algorithm": (
                "pure-Python struct.unpack per point + math.isfinite + "
                "downsample_points (equivalent to read_points path; "
                "sensor_msgs_py is unavailable in this harness)"
            ),
            "new_algorithm": "web_control.pointcloud_runtime.parse_pointcloud2",
        },
        "output": {
            "finite_count": old_result[1],
            "output_points": len(old_result[0]),
            "identical": old_result == new_result,
            "last_output_point": old_result[0][-1] if old_result[0] else None,
        },
        "old": {
            "wall_sec_per_iteration": round(old_wall, 6),
            "process_cpu_sec_per_iteration": round(old_cpu, 6),
            "tracemalloc_peak_bytes": old_peak,
        },
        "new": {
            "wall_sec_per_iteration": round(new_wall, 6),
            "process_cpu_sec_per_iteration": round(new_cpu, 6),
            "tracemalloc_peak_bytes": new_peak,
        },
        "ratios": {
            "wall_speedup": round(old_wall / new_wall, 2) if new_wall else None,
            "cpu_speedup": round(old_cpu / new_cpu, 2) if new_cpu else None,
            "peak_memory_reduction": (
                round(old_peak / new_peak, 2) if new_peak else None
            ),
        },
    }
    text = json.dumps(result, indent=2)
    print(text)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
