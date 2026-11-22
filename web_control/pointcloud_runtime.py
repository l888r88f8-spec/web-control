"""Bounded, demand-aware PointCloud2 visualization pipeline.

ROS subscription callbacks only replace one pending frame reference per layer.
A single daemon worker performs the vectorised decode away from the executor and
from ``WebControlNode.lock``.  Every layer is demand-gated: it decodes only
while an HTTP/SSE viewer has requested it within the demand TTL.  The three
latched navigation maps keep a 1 Hz output ceiling so a re-subscribed map is
not decoded twice per publisher sample.  Every layer releases its raw ROS
buffer as soon as its decoded 3D cache exists.  A layer that is never demanded
keeps at most one raw frame (its single pending slot) until demand,
invalidation or shutdown.  The web display calls the worker with
``max_points=0``: every finite received point is retained.  The
bounded sampling helper remains available for callers that explicitly
request a limit.

Dynamic layers follow incoming publisher frames.  There is no fixed output
ceiling: while the worker is busy, newer frames replace the single pending slot
so a slow decode degrades to the latest frame instead of growing a backlog.
The three latched navigation maps keep their one-new-input-per-second output
ceiling because they are static until a real lifecycle reset.

``CloudRevisionHub`` publishes metadata-only revisions to the HTTP layer.  It
never receives or serializes point arrays, so notification can happen outside
the ROS executor and without rebuilding cloud JSON.

The module deliberately owns no ROS imports.  Callers pass the message object,
the result callback and, optionally, a legacy reader used only for formats
NumPy cannot decode.
"""

from __future__ import annotations

import json
import math
import threading
import time
import uuid
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Optional, Sequence, Tuple

try:  # NumPy is a declared runtime dependency; keep the import guard so a
    # partial install can fall back to the legacy reader.
    import numpy as np
except Exception:  # pragma: no cover - only on a broken installation
    np = None


class UnsupportedPointCloudFormat(ValueError):
    """The NumPy decoder cannot safely interpret this layout.

    The caller may use the legacy ``sensor_msgs_py`` reader for this case.  A
    layout that is supported but inconsistent (for example a truncated buffer)
    raises plain :class:`ValueError` instead, so malformed frames are never
    silently misdecoded by the fallback.
    """


_DATATYPE_SIZES = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
_DATATYPE_NUMPY = {
    1: "i1",
    2: "u1",
    3: "i2",
    4: "u2",
    5: "i4",
    6: "u4",
    7: "f4",
    8: "f8",
}
# PointField FLOAT32 / FLOAT64.  XYZ as integers is not a real PointCloud2
# convention; rejecting it keeps the decoder from guessing units or scaling.
_FLOAT_DATATYPES = frozenset({7, 8})


def _sample_indices(
    finite_count: int,
    max_points: int,
    coordinate_at: Optional[Callable[[int], Sequence[float]]] = None,
) -> Any:
    """Return the same evenly-spaced sample indices as ``downsample_points``.

    The historical implementation used ``points[::step]`` and appended the
    final point only when its XYZ differed from the last sampled point.  That
    coordinate check matters for clouds whose final duplicate point has a
    different index, so ``coordinate_at`` lets the NumPy path compare just the
    two endpoint coordinates without materialising a Python input list.
    """
    if max_points <= 0 or finite_count <= max_points:
        return np.arange(finite_count, dtype=np.intp)
    step = max(1, int(math.ceil(finite_count / max_points)))
    indices = np.arange(0, finite_count, step, dtype=np.intp)
    if int(indices[-1]) != finite_count - 1 and len(indices) < max_points:
        append = True
        if coordinate_at is not None:
            last = tuple(float(value) for value in coordinate_at(int(indices[-1])))
            final = tuple(
                float(value) for value in coordinate_at(finite_count - 1)
            )
            append = last != final
        if append:
            indices = np.append(indices, finite_count - 1)
    return indices[:max_points]


def _downsample_python(
    points: list[tuple[float, float, float]], max_points: int
) -> list[tuple[float, float, float]]:
    if max_points <= 0 or len(points) <= max_points:
        return list(points)
    step = max(1, int(math.ceil(len(points) / max_points)))
    sampled = points[::step]
    if sampled[-1] != points[-1] and len(sampled) < max_points:
        sampled.append(points[-1])
    return sampled[:max_points]


def _parse_legacy(
    msg: Any,
    max_points: int,
    reader: Callable[[Any], Iterable[Sequence[float]]],
) -> tuple[list[list[float]], int]:
    """Small compatibility path for unsupported field layouts only."""
    points: list[tuple[float, float, float]] = []
    for point in reader(msg):
        try:
            x, y, z = float(point[0]), float(point[1]), float(point[2])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    sampled = _downsample_python(points, max_points)
    return [[x, y, z] for x, y, z in sampled], len(points)


def _numpy_fields(msg: Any) -> tuple[list[tuple[str, int, int, int]], int]:
    point_step = int(getattr(msg, "point_step", 0) or 0)
    if point_step <= 0:
        raise ValueError("PointCloud2 point_step must be positive")
    by_name: dict[str, Any] = {}
    for field in getattr(msg, "fields", ()) or ():
        name = str(getattr(field, "name", ""))
        if name in ("x", "y", "z"):
            by_name[name] = field
    if set(("x", "y", "z")) - set(by_name):
        raise UnsupportedPointCloudFormat("PointCloud2 is missing x/y/z fields")
    arrays = []
    for name in ("x", "y", "z"):
        field = by_name[name]
        datatype = int(getattr(field, "datatype", 0))
        offset = int(getattr(field, "offset", 0))
        size = _DATATYPE_SIZES.get(datatype)
        if datatype not in _FLOAT_DATATYPES or size is None:
            raise UnsupportedPointCloudFormat(
                f"PointCloud2 field {name} has unsupported datatype {datatype}"
            )
        if offset < 0 or offset + size > point_step:
            raise ValueError(
                f"PointCloud2 field {name} offset {offset} exceeds point_step "
                f"{point_step}"
            )
        arrays.append((name, datatype, offset, size))
    return arrays, point_step


def _parse_with_numpy(
    msg: Any, max_points: int
) -> tuple[list[list[float]], int]:
    if np is None:  # pragma: no cover - guarded by parse_pointcloud2
        raise UnsupportedPointCloudFormat("NumPy is unavailable")

    # A default-constructed PointCloud2 (width=height=0, point_step=0,
    # fields=[]) is a legitimate empty sample: it must reach the worker as an
    # empty result so a clear-on-empty layer can be cleared.  Check this before
    # layout validation; a non-empty malformed cloud still raises below.
    width = int(getattr(msg, "width", 0) or 0)
    height = int(getattr(msg, "height", 0) or 0)
    if width == 0 or height == 0:
        return [], 0
    if width < 0 or height < 0:
        raise ValueError("PointCloud2 width/height must not be negative")

    layouts, point_step = _numpy_fields(msg)
    row_step = int(getattr(msg, "row_step", 0) or 0)
    if row_step <= 0:
        row_step = width * point_step
    if row_step < width * point_step:
        raise ValueError(
            f"PointCloud2 row_step {row_step} is smaller than one row of "
            f"{width} points"
        )

    data = getattr(msg, "data", None)
    if data is None:
        raise ValueError("PointCloud2 data is missing")
    try:
        raw = np.frombuffer(data, dtype=np.uint8)
    except TypeError:
        raw = np.asarray(data, dtype=np.uint8)
    required = row_step * (height - 1) + width * point_step
    if raw.size < required:
        raise ValueError(
            f"PointCloud2 data is truncated: need {required} bytes, got {raw.size}"
        )

    big_endian = bool(getattr(msg, "is_bigendian", False))
    strides = (row_step, point_step)
    shape = (height, width)
    endian = ">" if big_endian else "<"
    arrays = {}
    for name, datatype, offset, size in layouts:
        code = _DATATYPE_NUMPY[datatype]
        if size > 1:
            code = endian + code
        arrays[name] = np.ndarray(
            shape=shape,
            dtype=np.dtype(code),
            buffer=raw,
            offset=offset,
            strides=strides,
        )

    finite = np.isfinite(arrays["x"])
    finite &= np.isfinite(arrays["y"])
    finite &= np.isfinite(arrays["z"])
    finite_count = int(np.count_nonzero(finite))
    if finite_count == 0:
        return [], 0

    # Map the sampled position among finite points back to its flat point
    # index, then gather only the selected coordinates.  No full Python tuple
    # list and no full float64 copy of the input cloud is created.
    finite_flat_indices = np.flatnonzero(finite.reshape(-1))

    def coordinate_at(position: int) -> tuple[Any, Any, Any]:
        flat = int(finite_flat_indices[position])
        row = flat // width
        column = flat - row * width
        return (
            arrays["x"][row, column],
            arrays["y"][row, column],
            arrays["z"][row, column],
        )

    sample_positions = _sample_indices(finite_count, max_points, coordinate_at)
    chosen = finite_flat_indices[sample_positions]
    rows = chosen // width
    cols = chosen - rows * width
    sampled = np.column_stack(
        (
            arrays["x"][rows, cols],
            arrays["y"][rows, cols],
            arrays["z"][rows, cols],
        )
    )
    return sampled.tolist(), finite_count


def parse_pointcloud2(
    msg: Any,
    max_points: int,
    legacy_reader: Optional[Callable[[Any], Iterable[Sequence[float]]]] = None,
) -> tuple[list[list[float]], int]:
    """Decode a PointCloud2 into ``(xyz, finite_count)``.

    ``max_points <= 0`` returns every finite point; a positive value retains
    the historical bounded sampling for explicit unit/benchmark callers.
    ``legacy_reader`` is only consulted for unsupported field types.  Invalid
    layouts always raise so a bad cloud produces a logged error, not a silent
    partial or misaligned scene.
    """
    try:
        return _parse_with_numpy(msg, max_points)
    except UnsupportedPointCloudFormat:
        if legacy_reader is None:
            raise
    return _parse_legacy(msg, max_points, legacy_reader)


# Only the static latched navigation maps retain an output ceiling.  Dynamic
# layers are intentionally absent: they follow publisher frames and rely on the
# single latest-frame slot, not a rate limiter, for bounded load.
LAYER_RATES = {
    ("nav", "global_map"): 1.0,
    ("nav", "mapcloud"): 1.0,
    ("nav", "mapground"): 1.0,
}
STATIC_LAYERS = frozenset(
    {
        # The three latched navigation maps keep a 1 Hz output ceiling so a
        # re-subscribed map is not decoded twice per publisher sample.  They
        # are demand-gated like every other layer.
        ("nav", "global_map"),
        ("nav", "mapcloud"),
        ("nav", "mapground"),
    }
)
STATIC_RATE = 1.0
# A request is considered active for a short window so an SSE viewer can renew
# demand about once per second without keeping every layer permanently active.
DEFAULT_DEMAND_TTL_SEC = 2.0


@dataclass
class _PendingFrame:
    msg: Any
    arrival_stamp: float
    clear_on_empty: bool
    generation: int
    epoch: int
    key_epoch: int


class PointCloudIngestWorker:
    """One background worker with a latest-frame slot per layer.

    ``apply_result`` is called outside every worker lock with the decoded
    points, so the node can briefly take its own cache lock without blocking
    other ROS callbacks while a cloud is decoded.  ``submit`` is O(1) and never
    calls the parser.
    """

    def __init__(
        self,
        apply_result: Callable[
            [str, str, list[list[float]], str, int, float, bool, int], Any
        ],
        *,
        max_points: int,
        legacy_reader: Optional[Callable[[Any], Iterable[Sequence[float]]]] = None,
        parser: Optional[Callable[[Any], tuple[list[list[float]], int]]] = None,
        logger: Any = None,
        clock: Callable[[], float] = time.monotonic,
        demand_ttl_sec: float = DEFAULT_DEMAND_TTL_SEC,
        rates: Optional[dict[tuple[str, str], float]] = None,
        static_layers: Optional[Iterable[tuple[str, str]]] = None,
        stop_timeout_sec: float = 1.0,
    ) -> None:
        self._apply_result = apply_result
        self._max_points = int(max_points)
        self._legacy_reader = legacy_reader
        self._parser = parser
        self._logger = logger
        self._clock = clock
        self._demand_ttl = max(0.0, float(demand_ttl_sec))
        self._rates = dict(LAYER_RATES if rates is None else rates)
        self._static_layers = frozenset(
            STATIC_LAYERS if static_layers is None else static_layers
        )
        self._stop_timeout = max(0.0, float(stop_timeout_sec))

        self._cond = threading.Condition(threading.RLock())
        self._pending: dict[tuple[str, str], _PendingFrame] = {}
        self._demand: dict[tuple[str, str], float] = {}
        # Admission time, not completion time: a slow decode must not add its
        # own duration on top of the per-layer output period.
        self._last_output: dict[tuple[str, str], float] = {}
        self._last_error_log: dict[tuple[str, str], float] = {}
        self._epoch = 0
        self._key_epochs: dict[tuple[str, str], int] = {}
        self._stopped = False
        self._thread: Optional[threading.Thread] = None

    # -- producer side -------------------------------------------------
    def start(self) -> None:
        with self._cond:
            if self._thread is not None or self._stopped:
                return
            self._thread = threading.Thread(
                target=self._run,
                name="web_control_pointcloud",
                daemon=True,
            )
            self._thread.start()

    def submit(
        self,
        key: tuple[str, str],
        msg: Any,
        *,
        arrival_stamp: float,
        clear_on_empty: bool,
        generation: int,
    ) -> bool:
        """Replace the pending frame for ``key``; never grows a queue."""
        with self._cond:
            if self._stopped:
                return False
            self._pending[key] = _PendingFrame(
                msg=msg,
                arrival_stamp=float(arrival_stamp),
                clear_on_empty=bool(clear_on_empty),
                generation=int(generation),
                epoch=self._epoch,
                key_epoch=self._key_epochs.get(key, 0),
            )
            self._cond.notify()
            return True

    def note_demand(
        self, group: str, layer: str, *, now: Optional[float] = None
    ) -> None:
        """Mark a visualization layer as requested by an HTTP client."""
        key = (str(group), str(layer))
        with self._cond:
            self._demand[key] = self._clock() if now is None else float(now)
            self._cond.notify()

    def demand_active(
        self, key: tuple[str, str], *, now: Optional[float] = None
    ) -> bool:
        """Return whether ``key`` was demanded within the TTL."""
        with self._cond:
            current = self._clock() if now is None else float(now)
            return self._demand_active((str(key[0]), str(key[1])), current)

    def demand_stamps(self) -> dict[tuple[str, str], float]:
        """Copy of the last demand stamp per layer.

        The reconciler reads this to decide which subscriptions to keep; the
        worker owns the stamps so a released layer cannot keep decoding.
        Stamps use this worker's clock (``time.monotonic`` by default), so the
        reconciler must compare them with ``now()``, never a wall clock.
        """
        with self._cond:
            return dict(self._demand)

    def now(self) -> float:
        """Current worker-clock time; the only clock demand stamps use."""
        return self._clock()

    def invalidate(
        self, keys: Optional[Iterable[tuple[str, str]]] = None
    ) -> None:
        """Drop pending raw frames and reject in-flight results.

        ``keys`` invalidates only those layers (for example a navigation stop
        must not reject an in-flight latched localization map).  Omitting it
        invalidates every layer, demand window and pending buffer.
        """
        with self._cond:
            if keys is None:
                self._epoch += 1
                self._pending.clear()
                self._demand.clear()
            else:
                for key in keys:
                    self._key_epochs[key] = self._key_epochs.get(key, 0) + 1
                    self._pending.pop(key, None)
                    self._demand.pop(key, None)
            self._cond.notify_all()

    def pending_count(self) -> int:
        with self._cond:
            return len(self._pending)

    def pending_keys(self) -> list[tuple[str, str]]:
        with self._cond:
            return list(self._pending)

    def is_alive(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def stop(self, timeout: Optional[float] = None) -> None:
        """Stop within a bounded time and release every pending raw buffer."""
        with self._cond:
            if self._stopped:
                thread = self._thread
            else:
                self._stopped = True
                self._epoch += 1
                self._pending.clear()
                self._demand.clear()
                self._cond.notify_all()
                thread = self._thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(self._stop_timeout if timeout is None else timeout)

    # -- worker side ---------------------------------------------------
    def _error(self, key: tuple[str, str], exc: BaseException) -> None:
        logger = self._logger
        if logger is None:
            return
        now = self._clock()
        with self._cond:
            last = self._last_error_log.get(key)
            if last is not None and now - last < 5.0:
                return
            self._last_error_log[key] = now
        try:
            logger.warning(
                f"point cloud decode failed for {key[0]}/{key[1]}: {exc}"
            )
        except Exception:
            pass

    def _demand_active(self, key: tuple[str, str], now: float) -> bool:
        stamp = self._demand.get(key)
        return stamp is not None and (now - stamp) <= self._demand_ttl

    def _ready_at(self, key: tuple[str, str]) -> float:
        # Dynamic layers have no output ceiling.  Their single pending slot is
        # the backpressure mechanism, so any retained frame is immediately
        # eligible once demand is active.
        if key not in self._static_layers:
            return float("-inf")
        last = self._last_output.get(key)
        if last is None:
            return float("-inf")
        rate = self._rates.get(key, STATIC_RATE)
        period = 0.0 if rate <= 0 else 1.0 / rate
        return last + period

    def _select_ready(self, now: float) -> Optional[tuple[str, str]]:
        for key in self._pending:
            if not self._demand_active(key, now):
                continue
            if now >= self._ready_at(key):
                return key
        return None

    def _next_wait(self, now: float) -> float:
        wait = 0.5
        for key in self._pending:
            if not self._demand_active(key, now):
                continue
            ready_in = self._ready_at(key) - now
            if ready_in > 0:
                wait = min(wait, ready_in)
        return max(0.01, wait)

    def _decode(self, msg: Any) -> tuple[list[list[float]], int]:
        if self._parser is not None:
            return self._parser(msg)
        return parse_pointcloud2(
            msg,
            self._max_points,
            legacy_reader=self._legacy_reader,
        )

    def _run(self) -> None:
        # Keep these locals in a try/finally that clears them after every
        # success, parse error, rejected generation and invalidated result.
        # Otherwise the loop's last ``frame`` keeps a potentially huge raw ROS
        # buffer strongly referenced while the worker sits idle with an empty
        # pending queue, and ``points`` keeps the decoded temporary list alive.
        frame = None
        points = None
        try:
            while True:
                try:
                    with self._cond:
                        while not self._stopped:
                            now = self._clock()
                            key = self._select_ready(now)
                            if key is not None:
                                frame = self._pending.pop(key)
                                # The rate period starts at admission, so the
                                # wait below is ``period - decode_duration``
                                # rather than a fixed period after completion.
                                self._last_output[key] = now
                                break
                            self._cond.wait(self._next_wait(now))
                        if self._stopped:
                            return
                        epoch = self._epoch
                    if frame.epoch != epoch:
                        continue

                    try:
                        decoded = self._decode(frame.msg)
                    except Exception as exc:
                        self._error(key, exc)
                        continue
                    # A decoder for raw subscription bytes returns the frame id
                    # with the points because there is no message object; the
                    # default decoder keeps reading it from the header.
                    if len(decoded) == 3:
                        points, finite_count, frame_id = decoded
                    else:
                        points, finite_count = decoded
                        frame_id = str(
                            getattr(
                                getattr(frame.msg, "header", None),
                                "frame_id",
                                "",
                            )
                        )
                    # Static and dynamic frames share the same lifecycle rule:
                    # a clear/stop that happened while decoding wins over the
                    # result.
                    with self._cond:
                        if self._stopped or frame.epoch != self._epoch:
                            continue
                        if frame.key_epoch != self._key_epochs.get(key, 0):
                            continue
                    group, layer = key
                    try:
                        self._apply_result(
                            group,
                            layer,
                            points,
                            frame_id,
                            int(finite_count),
                            float(frame.arrival_stamp),
                            bool(frame.clear_on_empty),
                            int(frame.generation),
                        )
                    except Exception as exc:
                        self._error(key, exc)
                finally:
                    frame = None
                    points = None
        finally:
            frame = None
            points = None


class CloudRevisionHub:
    """Metadata-only revision fan-out for browser cloud subscriptions.

    Entries contain versions and status, never point arrays.  A subscriber
    keeps its own ``seen`` sequence map, so one slow or disconnected client
    cannot consume an event on behalf of another.  Duplicate metadata is
    suppressed; a version or status change increments only that layer's
    sequence.

    ``metadata_reader`` is optional.  When supplied, a small monitor thread
    periodically refreshes all metadata so time-based transitions (for example
    ``active`` -> ``stale``) still reach subscribers without a new frame.  The
    reader must return a mapping of ``(group, layer)`` to metadata dicts and
    must not serialize geometry.
    """

    def __init__(
        self,
        metadata_reader: Optional[Callable[[], dict[Any, dict[str, Any]]]] = None,
        *,
        epoch: Optional[str] = None,
        clock: Callable[[], float] = time.monotonic,
        status_interval_sec: float = 1.0,
    ) -> None:
        self._reader = metadata_reader
        self._clock = clock
        # Immutable per-server-session identity.  A browser treats a change as
        # a new web-service process and resets its version cursors.
        self.epoch = str(epoch) if epoch is not None else uuid.uuid4().hex
        self._status_interval = max(0.1, float(status_interval_sec))
        self._cond = threading.Condition(threading.RLock())
        self._metadata: dict[Any, dict[str, Any]] = {}
        self._seq: dict[Any, int] = {}
        # Highest snapshot order accepted per layer.  Snapshot orders are
        # assigned while the node lock is held, so an older read that returns
        # after a newer one can never roll metadata (or status) backwards.
        self._order: dict[Any, int] = {}
        self._stopped = False
        self._thread: Optional[threading.Thread] = None
        self._monitor_stop = threading.Event()

    @property
    def stopped(self) -> bool:
        with self._cond:
            return self._stopped

    @staticmethod
    def _comparable_entry(entry: dict[str, Any]) -> dict[str, Any]:
        """Return an entry with refresh-only pose age removed.

        ``cloud_metadata_snapshot`` deliberately attaches a wall-time-derived
        ``age`` to every pose entry so a late subscriber sees an accurate HUD.
        That field changes on every monitor refresh, but it is not a new
        revision: comparing it would emit a same-version event once per
        status interval and make the browser re-apply an unchanged pose.
        """
        comparable = dict(entry)
        pose = comparable.get("pose")
        if isinstance(pose, dict) and "age" in pose:
            pose = dict(pose)
            pose.pop("age", None)
            comparable["pose"] = pose
        return comparable

    @classmethod
    def _revision_equal(
        cls, current: Optional[dict[str, Any]], entry: dict[str, Any]
    ) -> bool:
        if current is None:
            return False
        return cls._comparable_entry(current) == cls._comparable_entry(entry)

    def _entry(self, key: Any, payload: dict[str, Any]) -> dict[str, Any]:
        group, layer = key
        entry = {
            "group": str(payload.get("group", group)),
            "layer": str(payload.get("layer", layer)),
            "version": int(payload.get("version", 0)),
            "generation": int(payload.get("generation", 0)),
            "status": str(payload.get("status", "waiting")),
            "frame_id": str(payload.get("frame_id", "")),
            "count": int(payload.get("count", 0)),
        }
        # Pose is tiny and deliberately travels inline with its revision so the
        # browser does not issue one HTTP request per odometry frame.  Large
        # path/cloud arrays are never accepted here.
        pose = payload.get("pose")
        if isinstance(pose, dict):
            entry["pose"] = dict(pose)
        return entry

    def publish_many(
        self,
        metadata: dict[Any, dict[str, Any]],
        *,
        snapshot_order: Optional[int] = None,
    ) -> list[dict[str, Any]]:
        """Publish metadata and return only entries whose tuple changed.

        ``snapshot_order`` (or a per-entry ``snapshot_order``) is assigned
        under the node lock.  A lower order is an older read and is discarded
        even when it arrives later, so concurrent refresh threads cannot roll
        a newer version or a fresher same-version status back.
        """
        changed: list[dict[str, Any]] = []
        with self._cond:
            if self._stopped:
                return []
            for key, payload in metadata.items():
                entry = self._entry(key, payload)
                raw_order = payload.get("snapshot_order", snapshot_order)
                order: Optional[int] = None
                if raw_order is not None:
                    try:
                        order = int(raw_order)
                    except (TypeError, ValueError):
                        order = None
                if order is not None:
                    previous_order = self._order.get(key)
                    if previous_order is not None and order < previous_order:
                        continue
                current = self._metadata.get(key)
                if current is not None:
                    if entry["version"] < current["version"]:
                        continue
                    if entry["generation"] < current["generation"]:
                        continue
                if order is not None:
                    self._order[key] = order
                if self._revision_equal(current, entry):
                    # Refresh the stored HUD age without creating a revision;
                    # status/frame/generation/version/pose changes still emit.
                    self._metadata[key] = entry
                    continue
                self._metadata[key] = entry
                self._seq[key] = self._seq.get(key, 0) + 1
                changed.append({**entry, "seq": self._seq[key]})
            if changed:
                self._cond.notify_all()
        changed.sort(key=lambda item: (item["group"], item["layer"]))
        return changed

    def snapshot(
        self, keys: Optional[Iterable[Any]] = None
    ) -> list[dict[str, Any]]:
        """Return current metadata plus each layer's durable sequence."""
        allowed = None if keys is None else set(keys)
        with self._cond:
            entries = []
            for key, entry in self._metadata.items():
                if allowed is not None and key not in allowed:
                    continue
                entries.append({**entry, "seq": self._seq.get(key, 0)})
        entries.sort(key=lambda item: (item["group"], item["layer"]))
        return entries

    def wait_for_changes(
        self,
        seen: dict[Any, int],
        *,
        timeout: float,
        keys: Optional[Iterable[Any]] = None,
    ) -> Optional[list[dict[str, Any]]]:
        """Wait for metadata newer than ``seen``.

        Returns changed entries, ``[]`` when the hub stops, and ``None`` on a
        timeout with no change.  ``seen`` is caller-owned, so each browser
        keeps its own event cursor.
        """
        allowed = None if keys is None else set(keys)
        deadline = self._clock() + max(0.0, float(timeout))
        with self._cond:
            while True:
                if self._stopped:
                    return []
                changed = []
                for key, entry in self._metadata.items():
                    if allowed is not None and key not in allowed:
                        continue
                    seq = self._seq.get(key, 0)
                    if seen.get(key) != seq:
                        changed.append({**entry, "seq": seq})
                if changed:
                    changed.sort(key=lambda item: (item["group"], item["layer"]))
                    return changed
                remaining = deadline - self._clock()
                if remaining <= 0:
                    return None
                self._cond.wait(remaining)

    def refresh(self) -> list[dict[str, Any]]:
        """Read current metadata from the node and publish any changes."""
        reader = self._reader
        if reader is None:
            return []
        try:
            metadata = reader()
        except Exception:
            return []
        return self.publish_many(metadata)

    def start(self) -> None:
        reader = self._reader
        with self._cond:
            if self._stopped or reader is None or self._thread is not None:
                return
            self._monitor_stop.clear()
            self._thread = threading.Thread(
                target=self._monitor,
                name="web_control_cloud_revisions",
                daemon=True,
            )
            self._thread.start()

    def _monitor(self) -> None:
        while not self._monitor_stop.wait(self._status_interval):
            if self.stopped:
                return
            self.refresh()

    def stop(self, timeout: float = 1.0) -> None:
        with self._cond:
            if not self._stopped:
                self._stopped = True
                self._cond.notify_all()
            thread = self._thread
        self._monitor_stop.set()
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(max(0.0, float(timeout)))
        # The monitor may have published once more after ``_stopped`` was set;
        # publish_many rejects that path, so no post-stop events are queued.


class CloudResponseCache:
    """Bounded serialized-response cache for the cloud JSON endpoints.

    A body is keyed by a logical endpoint plus a signature containing the
    source version and freshness status.  The default retains only the one
    current body per logical endpoint; callers may explicitly keep more
    versions, and ``max_entries`` still bounds the whole cache.  Time-varying
    payloads are never served across a signature change.
    """

    def __init__(
        self, max_entries: int = 12, versions_per_key: int = 1
    ) -> None:
        self._max_entries = max(1, int(max_entries))
        self._versions_per_key = max(1, int(versions_per_key))
        self._lock = threading.RLock()
        self._entries: "OrderedDict[tuple[Any, Any], bytes]" = OrderedDict()
        self._key_locks: dict[Any, threading.Lock] = {}
        self._epoch = 0

    def _key_lock(self, logical: Any) -> threading.Lock:
        with self._lock:
            lock = self._key_locks.get(logical)
            if lock is None:
                lock = threading.Lock()
                self._key_locks[logical] = lock
            return lock

    def epoch(self) -> int:
        """Return the invalidation epoch, advanced by every ``clear``."""
        with self._lock:
            return self._epoch

    def encode(
        self,
        logical: Any,
        signature: Any,
        serializer: Callable[[], bytes],
        expected_epoch: int | None = None,
    ) -> bytes:
        full_key = (logical, signature)
        lock = self._key_lock(logical)
        with lock:
            with self._lock:
                epoch = self._epoch
                stale_snapshot = (
                    expected_epoch is not None and expected_epoch != epoch
                )
                if not stale_snapshot:
                    body = self._entries.get(full_key)
                    if body is not None:
                        self._entries.move_to_end(full_key)
                        return body
            # The body may still be returned to the request that is already
            # serializing, but a clear (including one between a caller's
            # payload snapshot and this call) must never be undone by
            # reinserting those bytes into the cache.
            body = serializer()
            if stale_snapshot:
                return body
            with self._lock:
                if self._epoch != epoch:
                    return body
                self._entries[full_key] = body
                self._entries.move_to_end(full_key)
                same = [key for key in self._entries if key[0] == logical]
                while len(same) > self._versions_per_key:
                    self._entries.pop(same.pop(0), None)
                while len(self._entries) > self._max_entries:
                    self._entries.popitem(last=False)
            return body

    def clear(self) -> None:
        with self._lock:
            self._epoch += 1
            self._entries.clear()

    def size(self) -> int:
        with self._lock:
            return len(self._entries)


def utf8_json_bytes(payload: Any) -> bytes:
    """Encode one cloud response with the same compact JSON format as HTTP."""
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")
