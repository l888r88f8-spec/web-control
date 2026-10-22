"""Focused contracts for the bounded point-cloud visualization pipeline.

These tests are ROS-free: binary PointCloud2 fixtures are built directly so the
NumPy decoder, the single-slot worker, lifecycle rejection and response cache
can be exercised without a ROS runtime or sockets.
"""

import gc
import json
import struct
import threading
import time
import urllib.request
import weakref
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest

from web_control import web_control_node
from web_control.core import SourceCache, downsample_points
from web_control.pointcloud_runtime import (
    CloudResponseCache,
    PointCloudIngestWorker,
    UnsupportedPointCloudFormat,
    parse_pointcloud2,
)


# ---------------------------------------------------------------------------
# Binary PointCloud2 fixtures
# ---------------------------------------------------------------------------

FLOAT32 = 7
FLOAT64 = 8
INT32 = 5


def _field(name, offset, datatype):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=1)


def _f32_cloud(points, *, big_endian=False, point_step=12):
    endian = ">" if big_endian else "<"
    pack = struct.Struct(endian + "fff").pack
    data = b"".join(pack(*point) for point in points)
    return SimpleNamespace(
        fields=[
            _field("x", 0, FLOAT32),
            _field("y", 4, FLOAT32),
            _field("z", 8, FLOAT32),
        ],
        point_step=point_step,
        row_step=len(data),
        width=len(points),
        height=1,
        is_bigendian=big_endian,
        data=data,
        header=SimpleNamespace(frame_id="map"),
    )


def _f64_big_endian_cloud(points):
    data = b"".join(struct.pack(">ddd", *point) for point in points)
    return SimpleNamespace(
        fields=[
            _field("x", 0, FLOAT64),
            _field("y", 8, FLOAT64),
            _field("z", 16, FLOAT64),
        ],
        point_step=24,
        row_step=len(data),
        width=len(points),
        height=1,
        is_bigendian=True,
        data=data,
        header=SimpleNamespace(frame_id="odom"),
    )


def _organized_mixed_cloud():
    """2x2 organized cloud: mixed F32/F64 offsets plus row padding."""
    width, height, point_step, row_step = 2, 2, 20, 44
    rows = []
    values = [
        [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)],
        [(7.0, 8.0, 9.0), (10.0, 11.0, 12.0)],
    ]
    for row_values in values:
        row = bytearray(b"\xff" * row_step)
        for column, (x, y, z) in enumerate(row_values):
            base = column * point_step
            struct.pack_into("<f", row, base + 4, x)
            struct.pack_into("<d", row, base + 8, y)
            struct.pack_into("<f", row, base + 16, z)
        rows.append(bytes(row))
    return SimpleNamespace(
        fields=[
            _field("intensity", 0, FLOAT32),
            _field("x", 4, FLOAT32),
            _field("y", 8, FLOAT64),
            _field("z", 16, FLOAT32),
        ],
        point_step=point_step,
        row_step=row_step,
        width=width,
        height=height,
        is_bigendian=False,
        data=b"".join(rows),
        header=SimpleNamespace(frame_id="map"),
    )


# ---------------------------------------------------------------------------
# Decoder
# ---------------------------------------------------------------------------

def test_binary_parser_filters_nan_inf_and_keeps_finite_count():
    points = [
        (1.0, 2.0, 3.0),
        (float("nan"), 4.0, 5.0),
        (6.0, 7.0, float("inf")),
        (8.0, 9.0, 10.0),
    ]
    decoded, count = parse_pointcloud2(_f32_cloud(points), 10)
    assert count == 2
    assert decoded == [[1.0, 2.0, 3.0], [8.0, 9.0, 10.0]]


def test_binary_parser_supports_f64_big_endian():
    decoded, count = parse_pointcloud2(
        _f64_big_endian_cloud([(1.5, -2.5, 3.5), (4.5, 5.5, 6.5)]), 10
    )
    assert count == 2
    assert decoded == [[1.5, -2.5, 3.5], [4.5, 5.5, 6.5]]


def test_binary_parser_handles_mixed_offsets_f64_and_organized_row_padding():
    decoded, count = parse_pointcloud2(_organized_mixed_cloud(), 10)
    assert count == 4
    assert decoded == [
        [1.0, 2.0, 3.0],
        [4.0, 5.0, 6.0],
        [7.0, 8.0, 9.0],
        [10.0, 11.0, 12.0],
    ]


def test_binary_parser_matches_old_downsample_coverage_and_max_bounds():
    points = [(float(index), 2.0, 3.0) for index in range(10)]
    decoded, count = parse_pointcloud2(_f32_cloud(points), 3)
    assert count == 10
    assert len(decoded) == 3
    # Old downsample_points: step = ceil(10/3) = 4 -> indices 0, 4, 8, then
    # the final point is appended because there is room.
    assert [int(point[0]) for point in decoded] == [0, 4, 8]

    all_points, _ = parse_pointcloud2(_f32_cloud(points), 0)
    assert len(all_points) == 10


def test_binary_parser_matches_old_downsample_for_duplicate_final_xyz():
    points = [(float(index), 2.0, 3.0) for index in range(10)]
    points[8] = (8.0, 4.0, 5.0)
    points[9] = points[8]
    expected = [list(point) for point in downsample_points(points, 6)]
    decoded, count = parse_pointcloud2(_f32_cloud(points), 6)
    assert count == 10
    assert decoded == expected
    # step=ceil(10/6)=2 samples indices 0,2,4,6,8; the duplicate final XYZ
    # means the old algorithm does not append index 9.
    assert decoded[-1] == [8.0, 4.0, 5.0]
    assert len(decoded) == 5


def test_binary_parser_rejects_truncated_and_inconsistent_layouts():
    msg = _f32_cloud([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
    msg.data = msg.data[:-4]
    with pytest.raises(ValueError):
        parse_pointcloud2(msg, 10)

    msg = _f32_cloud([(1.0, 2.0, 3.0)])
    msg.point_step = 8
    msg.row_step = 8
    with pytest.raises(ValueError):
        parse_pointcloud2(msg, 10)


def test_default_empty_cloud_is_valid_while_nonempty_bad_layout_still_raises():
    empty = SimpleNamespace(
        point_step=0, width=0, height=0, fields=[], data=b""
    )
    assert parse_pointcloud2(empty, 10) == ([], 0)

    malformed = SimpleNamespace(
        point_step=0, width=1, height=1, fields=[], data=b""
    )
    with pytest.raises(ValueError):
        parse_pointcloud2(malformed, 10)


def test_unsupported_datatype_uses_legacy_only_and_never_misdecodes():
    msg = _f32_cloud([(1.0, 2.0, 3.0)])
    msg.fields[0] = _field("x", 0, INT32)
    calls = []

    def legacy(_msg):
        calls.append(1)
        return iter([(9.0, 8.0, 7.0)])

    # Without a fallback the unsupported layout is an explicit error.
    with pytest.raises(UnsupportedPointCloudFormat):
        parse_pointcloud2(msg, 10)
    assert calls == []

    decoded, count = parse_pointcloud2(msg, 10, legacy_reader=legacy)
    assert calls == [1]
    assert (decoded, count) == ([[9.0, 8.0, 7.0]], 1)

    # A supported but truncated layout must not silently fall back.
    good = _f32_cloud([(1.0, 2.0, 3.0)])
    good.data = b""
    with pytest.raises(ValueError):
        parse_pointcloud2(good, 10, legacy_reader=legacy)
    assert calls == [1]


# ---------------------------------------------------------------------------
# Worker: one slot, overwrite, demand, rates and lifecycle
# ---------------------------------------------------------------------------

def _frame(tag, frame_id="map"):
    return SimpleNamespace(tag=tag, header=SimpleNamespace(frame_id=frame_id))


class _ManualClock:
    def __init__(self, now=0.0):
        self.now = float(now)

    def __call__(self):
        return self.now


def _worker(
    applied,
    *,
    rates=None,
    static_layers=None,
    parser=None,
    logger=None,
    clock=None,
):
    return PointCloudIngestWorker(
        lambda *args: applied.append(args),
        max_points=10,
        parser=parser or (lambda msg: ([[1.0, 2.0, 3.0]], 1)),
        logger=logger,
        rates=rates,
        static_layers=static_layers,
        clock=clock or time.monotonic,
    )


def _wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_callbacks_only_replace_one_pending_frame_and_decode_latest():
    applied = []
    parsed = []

    def parser(msg):
        parsed.append(msg.tag)
        return [[1.0, 2.0, 3.0]], 1

    worker = _worker(applied, parser=parser)
    for tag in range(50):
        worker.submit(
            ("nav", "global_map"),
            _frame(tag),
            arrival_stamp=float(tag),
            clear_on_empty=False,
            generation=0,
        )
    assert worker.pending_count() == 1
    assert worker.pending_keys() == [("nav", "global_map")]
    worker.start()
    try:
        assert _wait_until(lambda: applied)
    finally:
        worker.stop()

    # Only the newest pending reference was decoded; no queue existed.
    assert parsed == [49]
    assert len(applied) == 1
    assert applied[0][7] == 0
    assert worker.pending_count() == 0


def test_pending_frame_is_released_after_static_apply():
    applied = []
    worker = _worker(applied)
    worker.submit(
        ("nav", "global_map"),
        _frame(1),
        arrival_stamp=1.0,
        clear_on_empty=False,
        generation=0,
    )
    worker.start()
    try:
        assert _wait_until(lambda: applied)
        assert worker.pending_count() == 0
    finally:
        worker.stop()


def test_raw_frame_is_released_while_worker_remains_running_idle():
    class Frame:
        def __init__(self, tag):
            self.tag = tag
            self.header = SimpleNamespace(frame_id="map")

    applied = []
    worker = _worker(applied)
    frame = Frame(1)
    reference = weakref.ref(frame)
    worker.submit(
        ("nav", "global_map"),
        frame,
        arrival_stamp=1.0,
        clear_on_empty=False,
        generation=0,
    )
    del frame
    worker.start()
    try:
        assert _wait_until(lambda: applied)
        assert worker.pending_count() == 0
        assert worker.is_alive()
        for _ in range(50):
            gc.collect()
            if reference() is None:
                break
            time.sleep(0.01)
        # The worker is still running and idle: the raw frame must already be
        # collectable, not merely gone from the pending queue.
        assert reference() is None
        assert worker.is_alive()
    finally:
        worker.stop()


def test_dynamic_decoding_pauses_without_recent_http_demand():
    applied = []
    worker = _worker(applied, rates={("nav", "local_map"): 1.0})
    key = ("nav", "local_map")
    worker.submit(
        key, _frame(1), arrival_stamp=0.0, clear_on_empty=False, generation=0
    )
    assert worker._select_ready(0.0) is None
    worker.note_demand("nav", "local_map", now=0.0)
    assert worker._select_ready(0.0) == key
    # Demand TTL default is 2 s; after it expires the retained latest frame is
    # not decoded until a new request arrives.
    assert worker._select_ready(2.1) is None
    worker.note_demand("nav", "local_map", now=2.2)
    assert worker._select_ready(2.2) == key
    worker.stop()


def test_static_layers_decode_once_per_new_input_without_demand():
    applied = []
    worker = _worker(applied)
    key = ("nav", "global_map")
    worker.submit(
        key, _frame(1), arrival_stamp=0.0, clear_on_empty=False, generation=0
    )
    # Static latched map is ready for a late browser with no HTTP request.
    assert worker._select_ready(0.0) == key
    worker._pending.pop(key)
    worker._last_output[key] = 0.0
    worker.submit(
        key, _frame(2), arrival_stamp=0.5, clear_on_empty=False, generation=0
    )
    assert worker._select_ready(0.5) is None  # 1 Hz output ceiling
    assert worker._select_ready(1.0) == key
    worker.stop()


def test_mapping_layers_pause_without_demand_then_decode_latest_on_resume():
    clock = _ManualClock()
    applied = []
    parsed = []

    def parser(msg):
        parsed.append(msg.tag)
        return [[1.0, 2.0, 3.0]], 1

    worker = _worker(
        applied,
        rates={("mapping", "global_map"): 1.0},
        parser=parser,
        clock=clock,
    )
    key = ("mapping", "global_map")
    worker.submit(
        key, _frame(1), arrival_stamp=0.0, clear_on_empty=False, generation=0
    )
    worker.submit(
        key, _frame(2), arrival_stamp=0.0, clear_on_empty=False, generation=0
    )
    # No view request: only the newest raw frame is retained, nothing decoded.
    assert worker.pending_count() == 1
    assert worker._select_ready(0.0) is None
    assert parsed == []

    # A late Mapping view activates the existing latest frame.
    worker.note_demand("mapping", "global_map", now=0.0)
    worker.start()
    try:
        assert _wait_until(lambda: applied)
        assert parsed == [2]

        # After the TTL a new frame stays pending without decoding.
        clock.now = 3.0
        worker.submit(
            key, _frame(3), arrival_stamp=3.0, clear_on_empty=False, generation=0
        )
        assert worker._select_ready(3.0) is None
        assert parsed == [2]

        # Resuming demand decodes the latest retained frame, not an old one.
        worker.note_demand("mapping", "global_map", now=3.0)
        assert _wait_until(lambda: len(applied) == 2)
        assert parsed == [2, 3]
    finally:
        worker.stop()
    assert worker.pending_count() == 0


def test_output_period_is_measured_from_admission_not_completion():
    clock = _ManualClock()
    applied = []

    def parser(msg):
        if msg.tag == 1:
            # Simulate a 50 ms decode inside a 200 ms (5 Hz) period.
            clock.now += 0.05
        return [[1.0, 2.0, 3.0]], 1

    worker = _worker(
        applied,
        rates={("nav", "obstacles"): 5.0},
        static_layers=(),
        parser=parser,
        clock=clock,
    )
    key = ("nav", "obstacles")
    worker.note_demand("nav", "obstacles", now=0.0)
    worker.submit(
        key, _frame(1), arrival_stamp=0.0, clear_on_empty=False, generation=0
    )
    worker.start()
    try:
        assert _wait_until(lambda: applied)
        # Admission happened at 0.0; completion at 0.05 must not postpone the
        # next output to 0.25.
        assert clock.now == pytest.approx(0.05)
        assert worker._last_output[key] == pytest.approx(0.0)
        worker.submit(
            key, _frame(2), arrival_stamp=0.05, clear_on_empty=False, generation=0
        )
        assert worker._select_ready(0.05) is None
        assert worker._select_ready(0.199) is None
        assert worker._select_ready(0.2) == key
    finally:
        worker.stop()


def test_dynamic_output_rate_is_independent_per_layer():
    applied = []
    worker = _worker(
        applied,
        rates={("nav", "local_map"): 1.0, ("nav", "obstacles"): 5.0},
        static_layers=(),
    )
    now = 10.0
    worker.note_demand("nav", "local_map", now=now)
    worker.note_demand("nav", "obstacles", now=now)
    worker.submit(
        ("nav", "local_map"),
        _frame(1),
        arrival_stamp=now,
        clear_on_empty=False,
        generation=0,
    )
    worker.submit(
        ("nav", "obstacles"),
        _frame(2),
        arrival_stamp=now,
        clear_on_empty=False,
        generation=0,
    )
    local = ("nav", "local_map")
    obstacles = ("nav", "obstacles")
    worker._pending.pop(local)
    worker._pending.pop(obstacles)
    worker._last_output[local] = now
    worker._last_output[obstacles] = now
    worker.submit(
        local, _frame(3), arrival_stamp=now, clear_on_empty=False, generation=0
    )
    worker.submit(
        obstacles, _frame(4), arrival_stamp=now, clear_on_empty=False, generation=0
    )
    assert worker._select_ready(now + 0.19) is None
    assert worker._select_ready(now + 0.2) == obstacles
    worker._last_output[obstacles] = now + 0.2
    assert worker._select_ready(now + 0.39) is None
    assert worker._select_ready(now + 0.41) == obstacles
    worker._pending.pop(obstacles)
    worker._last_output[obstacles] = now + 0.41
    assert worker._select_ready(now + 1.01) == local
    worker.stop()


def test_invalidate_drops_inflight_result_after_clear():
    started = threading.Event()
    release = threading.Event()
    applied = []

    def parser(_msg):
        started.set()
        assert release.wait(2.0)
        return [[1.0, 2.0, 3.0]], 1

    worker = _worker(applied, parser=parser)
    key = ("nav", "obstacles")
    worker.note_demand("nav", "obstacles")
    worker.submit(
        key, _frame(1), arrival_stamp=1.0, clear_on_empty=False, generation=0
    )
    worker.start()
    try:
        assert started.wait(2.0)
        worker.invalidate([key])
        release.set()
        time.sleep(0.15)
        assert applied == []
    finally:
        release.set()
        worker.stop()


def test_bad_cloud_is_logged_and_does_not_stop_worker():
    class Logger:
        def __init__(self):
            self.messages = []

        def warning(self, message):
            self.messages.append(message)

    calls = []

    def parser(msg):
        calls.append(msg.tag)
        if msg.tag == 1:
            raise ValueError("bad layout")
        return [[2.0, 3.0, 4.0]], 1

    logger = Logger()
    applied = []
    worker = _worker(applied, parser=parser, logger=logger)
    worker.submit(
        ("nav", "global_map"),
        _frame(1),
        arrival_stamp=1.0,
        clear_on_empty=False,
        generation=0,
    )
    worker.start()
    try:
        assert _wait_until(lambda: logger.messages)
        worker._last_output.clear()
        worker.submit(
            ("nav", "global_map"),
            _frame(2),
            arrival_stamp=2.0,
            clear_on_empty=False,
            generation=0,
        )
        assert _wait_until(lambda: applied)
        assert calls == [1, 2]
        assert "bad layout" in logger.messages[0]
    finally:
        worker.stop()


def test_stop_is_bounded_and_rejects_new_work():
    applied = []
    worker = _worker(applied)
    worker.start()
    worker.stop(timeout=0.5)
    assert not worker.is_alive()
    assert worker.submit(
        ("nav", "obstacles"),
        _frame(1),
        arrival_stamp=1.0,
        clear_on_empty=True,
        generation=0,
    ) is False
    worker.stop()


# ---------------------------------------------------------------------------
# Node-side generation and cache application
# ---------------------------------------------------------------------------

def _cache_node():
    node = SimpleNamespace(
        lock=threading.RLock(),
        layers={
            "global_map": SourceCache("global_map", 2.0, static_source=True),
            "local_map": SourceCache("local_map", 2.0, static_source=True),
            "mapcloud": SourceCache("mapcloud", 2.0, static_source=True),
            "mapground": SourceCache("mapground", 2.0, static_source=True),
            "obstacles": SourceCache("obstacles", 2.0),
        },
        mapping_layers={
            "global_map": SourceCache("mapping_global_map", 2.0, static_source=True),
            "ground": SourceCache("mapping_ground", 2.0, static_source=True),
        },
        _cloud_generations={},
        cloud_responses=CloudResponseCache(),
        max_2d_points=60,
        now=10.0,
    )
    node.now_sec = lambda: node.now
    node._apply_pointcloud_result = MethodType(
        web_control_node.WebControlNode._apply_pointcloud_result, node
    )
    node.advance = lambda keys=None: web_control_node.advance_cloud_generation(
        node, keys
    )
    return node


def test_binary_empty_cloud_clears_obstacles_through_real_worker():
    node = _cache_node()
    node.layers["obstacles"].update(
        {"points3d": [[1.0, 2.0, 3.0]]}, stamp=10.0, frame_id="map", count=1
    )
    applied = []
    worker = PointCloudIngestWorker(
        lambda *args: applied.append(node._apply_pointcloud_result(*args)),
        max_points=10,
        parser=lambda msg: parse_pointcloud2(msg, 10),
        rates={("nav", "obstacles"): 20.0},
        static_layers=(),
    )
    worker.note_demand("nav", "obstacles")
    worker.submit(
        ("nav", "obstacles"),
        SimpleNamespace(
            point_step=0, width=0, height=0, fields=[], data=b""
        ),
        arrival_stamp=10.1,
        clear_on_empty=True,
        generation=0,
    )
    worker.start()
    try:
        assert _wait_until(lambda: applied)
    finally:
        worker.stop()
    assert applied == [True]
    assert node.layers["obstacles"].data is None
    assert node.layers["obstacles"].status(10.1) == "waiting"


def test_generation_rejects_inflight_result_after_clear():
    node = _cache_node()
    nav_global = ("nav", "global_map")
    node._cloud_generations[nav_global] = 1
    applied = node._apply_pointcloud_result(
        "nav",
        "global_map",
        [[1.0, 2.0, 3.0]],
        "map",
        1,
        10.0,
        False,
        0,
    )
    assert applied is False
    assert node.layers["global_map"].data is None

    applied = node._apply_pointcloud_result(
        "nav",
        "global_map",
        [[1.0, 2.0, 3.0]],
        "map",
        1,
        10.0,
        False,
        1,
    )
    assert applied is True
    assert node.layers["global_map"].data == {"points3d": [[1.0, 2.0, 3.0]]}


def test_empty_obstacle_result_clears_and_late_frame_after_restart_applies():
    node = _cache_node()
    node.layers["obstacles"].update(
        {"points3d": [[1.0, 2.0, 3.0]]}, stamp=10.0, frame_id="map", count=1
    )
    assert node._apply_pointcloud_result(
        "nav", "obstacles", [], "map", 0, 10.1, True, 0
    )
    assert node.layers["obstacles"].data is None
    assert node.layers["obstacles"].status(10.1) == "waiting"

    # A clear advances only its own key generation; a frame from the new
    # lifecycle (new generation) is accepted.
    node.advance([("nav", "obstacles")])
    stale_generation = node._cloud_generations[("nav", "obstacles")] - 1
    assert node._apply_pointcloud_result(
        "nav", "obstacles", [[9.0, 9.0, 9.0]], "map", 1, 11.0, True,
        stale_generation,
    ) is False
    accepted_generation = node._cloud_generations[("nav", "obstacles")]
    assert node._apply_pointcloud_result(
        "nav", "obstacles", [[9.0, 9.0, 9.0]], "map", 1, 11.0, True,
        accepted_generation,
    )
    assert node.layers["obstacles"].data == {"points3d": [[9.0, 9.0, 9.0]]}


def test_nav_clear_does_not_drop_latched_localization_map_generation():
    node = _cache_node()
    node.advance(
        [("nav", "mapcloud"), ("nav", "mapground"), ("nav", "obstacles")]
    )
    assert node._cloud_generations[("nav", "mapcloud")] == 1
    assert node._cloud_generations.get(("nav", "global_map"), 0) == 0
    assert node._cloud_generations.get(("nav", "local_map"), 0) == 0


# ---------------------------------------------------------------------------
# Serialized response cache and node payload versions
# ---------------------------------------------------------------------------

def test_json_response_is_encoded_once_per_version_status_and_concurrently():
    cache = CloudResponseCache(max_entries=6, versions_per_key=2)
    calls = []

    def encoder(value):
        def serialize():
            calls.append(value)
            return ('{"value":%d}' % value).encode()
        return serialize

    logical = ("nav", "local_map", "3d")
    first = cache.encode(logical, (1, "active", "map", 1), encoder(1))
    again = cache.encode(logical, (1, "active", "map", 1), encoder(2))
    assert first is again
    assert calls == [1]

    # A status transition invalidates the body even with the same version.
    stale = cache.encode(logical, (1, "stale", "map", 1), encoder(3))
    assert stale == b'{"value":3}'
    assert calls == [1, 3]

    # Concurrent identical misses serialize once under the per-key lock.
    cache.clear()
    calls.clear()
    barrier = threading.Barrier(8)
    results = []

    def request():
        barrier.wait()
        results.append(
            cache.encode(("nav", "obstacles", "3d"), (5, "active", "map", 1), encoder(9))
        )

    threads = [threading.Thread(target=request) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert calls == [9]
    assert len(set(results)) == 1


def test_response_cache_default_keeps_one_current_version_per_endpoint():
    cache = CloudResponseCache(max_entries=10)
    logical = ("nav", "local_map", "3d")
    for version in range(4):
        cache.encode(
            logical,
            (version, "active", "map", 1),
            lambda version=version: str(version).encode(),
        )
    assert cache.size() == 1
    assert cache.encode(
        logical, (3, "active", "map", 1), lambda: b"unexpected"
    ) == b"3"

    # Explicit retention remains supported for callers that request it.
    explicit = CloudResponseCache(max_entries=10, versions_per_key=2)
    for version in range(4):
        explicit.encode(
            logical,
            (version, "active", "map", 1),
            lambda version=version: str(version).encode(),
        )
    assert explicit.size() == 2


def test_response_cache_is_bounded_and_clear_releases_bodies():
    cache = CloudResponseCache(max_entries=4, versions_per_key=2)
    logical = ("nav", "local_map", "3d")
    for version in range(10):
        cache.encode(
            logical,
            (version, "active", "map", 1),
            lambda version=version: str(version).encode(),
        )
    assert cache.size() <= 4
    cache.clear()
    assert cache.size() == 0


def test_clear_during_serialization_does_not_retain_old_body():
    cache = CloudResponseCache(max_entries=4)
    logical = ("nav", "local_map", "3d")
    entered = threading.Event()
    release = threading.Event()
    old_body = b'{"points":[[1,2,3]]}'
    results = []

    def blocked_serializer():
        entered.set()
        assert release.wait(5)
        return old_body

    thread = threading.Thread(
        target=lambda: results.append(
            cache.encode(logical, (1, "active", "map", 1), blocked_serializer)
        )
    )
    thread.start()
    try:
        assert entered.wait(5)
        cache.clear()
        release.set()
        thread.join(5)
        assert not thread.is_alive()
    finally:
        release.set()
        thread.join(5)

    # The blocked HTTP request may still return its body, but the cleared
    # cache must not retain those old bytes.
    assert results == [old_body]
    assert cache.size() == 0

    # The next request for the current version serializes and caches once.
    current_body = b'{"points":[[9,9,9]]}'
    current_calls = []
    signature = (2, "active", "map", 1)

    def current_serializer():
        current_calls.append("encode")
        return current_body

    current = cache.encode(logical, signature, current_serializer)
    assert current == current_body
    assert cache.encode(
        logical,
        signature,
        lambda: (_ for _ in ()).throw(AssertionError("cached body was re-encoded")),
    ) is current
    assert current_calls == ["encode"]


def _response_node():
    node = _cache_node()
    node.layers["local_map"].update(
        {"points3d": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]},
        stamp=10.0,
        frame_id="map",
        count=2,
    )
    node.mapping_layers["global_map"].update(
        {"points3d": [[7.0, 8.0, 9.0]]}, stamp=10.0, frame_id="map", count=1
    )
    node.layers["local_map"].static_source = False
    node.layer_payload = MethodType(
        web_control_node.WebControlNode.layer_payload, node
    )
    node.layer_response = MethodType(
        web_control_node.WebControlNode.layer_response, node
    )
    node.mapping_layer_payload = MethodType(
        web_control_node.WebControlNode.mapping_layer_payload, node
    )
    node.mapping_layer_response = MethodType(
        web_control_node.WebControlNode.mapping_layer_response, node
    )
    return node


def test_node_cloud_response_reuses_bytes_until_version_or_status_changes():
    node = _response_node()
    body = node.layer_response("local_map", "3d")
    assert body is node.layer_response("local_map", "3d")
    assert json.loads(body)["points"] == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]

    node.layers["local_map"].update(
        {"points3d": [[9.0, 9.0, 9.0]]}, stamp=10.0, frame_id="map", count=1
    )
    updated = node.layer_response("local_map", "3d")
    assert updated != body
    assert json.loads(updated)["points"] == [[9.0, 9.0, 9.0]]

    node.now = 12.1
    stale = node.layer_response("local_map", "3d")
    assert stale != updated
    assert json.loads(stale)["status"] == "stale"
    # Once cached, the stale-but-unchanged body is reused.
    assert stale is node.layer_response("local_map", "3d")

    node.layers["local_map"].clear()
    cleared = node.layer_response("local_map", "3d")
    assert json.loads(cleared)["status"] == "waiting"
    assert json.loads(cleared)["points"] == []


def test_mapping_response_cache_and_legacy_2d_projection_stay_separate():
    node = _response_node()
    body_3d = node.mapping_layer_response("global_map")
    assert body_3d is node.mapping_layer_response("global_map")
    assert json.loads(body_3d)["points"] == [[7.0, 8.0, 9.0]]

    legacy = json.loads(node.layer_response("local_map", "2d"))
    assert legacy["points"] == [[1.0, 2.0], [4.0, 5.0]]
    # The 2D body never mutates or replaces the 3D cache.
    assert node.layers["local_map"].data == {
        "points3d": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    }
    assert node.layer_response("local_map", "2d") is node.layer_response(
        "local_map", "2d"
    )


def test_node_layer_response_does_not_cache_snapshot_taken_before_clear(monkeypatch):
    node = _response_node()
    original = web_control_node._layer_payload_snapshot
    clears = []

    def snapshot_then_clear(owner, layer_name, mode):
        payload, version = original(owner, layer_name, mode)
        if not clears:
            clears.append((layer_name, mode))
            node.cloud_responses.clear()
        return payload, version

    monkeypatch.setattr(
        web_control_node, "_layer_payload_snapshot", snapshot_then_clear
    )
    body = node.layer_response("local_map", "3d")
    assert json.loads(body)["points"] == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    # The clear landed after the values snapshot but before encode; the old
    # body must not be inserted into the cleared cache.
    assert node.cloud_responses.size() == 0

    cached = node.layer_response("local_map", "3d")
    assert node.cloud_responses.size() == 1
    assert node.layer_response("local_map", "3d") is cached


def test_node_mapping_response_does_not_cache_snapshot_taken_before_clear(monkeypatch):
    node = _response_node()
    original = web_control_node._mapping_layer_payload_snapshot
    clears = []

    def snapshot_then_clear(owner, layer_name):
        payload, version = original(owner, layer_name)
        if not clears:
            clears.append(layer_name)
            node.cloud_responses.clear()
        return payload, version

    monkeypatch.setattr(
        web_control_node,
        "_mapping_layer_payload_snapshot",
        snapshot_then_clear,
    )
    body = node.mapping_layer_response("global_map")
    assert json.loads(body)["points"] == [[7.0, 8.0, 9.0]]
    assert node.cloud_responses.size() == 0

    cached = node.mapping_layer_response("global_map")
    assert node.cloud_responses.size() == 1
    assert node.mapping_layer_response("global_map") is cached


def test_node_callback_does_not_parse_or_touch_cache_when_worker_exists():
    class Worker:
        def __init__(self):
            self.submitted = []

        def submit(self, *args, **kwargs):
            self.submitted.append((args, kwargs))

    node = SimpleNamespace(
        lock=threading.RLock(),
        layers={"global_map": SourceCache("global_map", 2.0, static_source=True)},
        mapping_layers={},
        cloud_worker=Worker(),
        _cloud_generations={("nav", "global_map"): 3},
        now=10.0,
    )
    node.now_sec = lambda: node.now
    node.update_pointcloud_cache = lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError("synchronous parse must not run")
    )
    node._enqueue_pointcloud = MethodType(
        web_control_node.WebControlNode._enqueue_pointcloud, node
    )
    node.pointcloud_callback = MethodType(
        web_control_node.WebControlNode.pointcloud_callback, node
    )
    msg = _frame(1)
    node.pointcloud_callback("global_map", msg)

    assert len(node.cloud_worker.submitted) == 1
    args, kwargs = node.cloud_worker.submitted[0]
    assert args == (("nav", "global_map"), msg)
    assert kwargs["generation"] == 3
    assert kwargs["arrival_stamp"] == 10.0


# ---------------------------------------------------------------------------
# HTTP routing: demand activation without blocking on the worker
# ---------------------------------------------------------------------------

def test_cloud_routes_activate_demand_and_send_cached_bytes(monkeypatch):
    class Logger:
        def debug(self, *_args, **_kwargs):
            pass

        def error(self, *_args, **_kwargs):
            pass

    class RouteNode:
        def __init__(self):
            self.static_dir = Path(__file__).resolve().parents[1] / "static"
            self.demand = []

        def note_cloud_demand(self, group, layer):
            self.demand.append((group, layer))

        def layer_response(self, layer, mode):
            return ('{"route":"%s","layer":"%s","mode":"%s"}' % (
                "nav", layer, mode
            )).encode()

        def mapping_layer_response(self, layer):
            return ('{"route":"mapping","layer":"%s"}' % layer).encode()

        def now_sec(self):
            return 1.0

        def get_logger(self):
            return Logger()

    node = RouteNode()
    handler = web_control_node.WebControlNode.make_handler(node)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urllib.request.urlopen(f"{base}/api/map3d?layer=local_map", timeout=5) as response:
            assert json.loads(response.read()) == {
                "route": "nav",
                "layer": "local_map",
                "mode": "3d",
            }
        with urllib.request.urlopen(f"{base}/api/map2d?layer=obstacles", timeout=5) as response:
            assert json.loads(response.read())["mode"] == "2d"
        with urllib.request.urlopen(
            f"{base}/api/mapping/map3d?layer=ground", timeout=5
        ) as response:
            assert json.loads(response.read()) == {
                "route": "mapping",
                "layer": "ground",
            }
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert node.demand == [
        ("nav", "local_map"),
        ("nav", "obstacles"),
        ("mapping", "ground"),
    ]
