"""Convert grid global-planner terrain MarkerArray data for the web viewer.

The grid planner publishes one latched ``visualization_msgs/MarkerArray`` per
terrain snapshot.  Every snapshot starts with DELETEALL and then appends
``terrain_cells`` triangle batches whose vertices are already in the map frame
(identity marker pose).  The browser renders those triangles directly, so this
module keeps every complete finite triangle; it never samples or caps vertex
count.  It owns no ROS imports and is unit-testable with plain objects.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Iterable, Optional

# visualization_msgs/Marker type and action constants.
MARKER_ADD = 0
MARKER_MODIFY = 1
MARKER_DELETEALL = 3
MARKER_TRIANGLE_LIST = 11

TERRAIN_CELL_NAMESPACE = "terrain_cells"


def _finite_point(point: Any) -> tuple[float, float, float] | None:
    try:
        x = float(point.x)
        y = float(point.y)
        z = float(point.z)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
    if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
        return x, y, z
    return None


def _marker_color(marker: Any, index: int) -> tuple[float, float, float]:
    """Return one marker vertex colour, falling back to the marker's flat colour."""
    colors = getattr(marker, "colors", None)
    try:
        color = colors[index]
        return float(color.r), float(color.g), float(color.b)
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        color = getattr(marker, "color", None)
        try:
            return float(color.r), float(color.g), float(color.b)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return 1.0, 1.0, 1.0


def terrain_markers_to_triangles(
    markers: Iterable[Any],
    *,
    yield_control: Optional[Callable[[], bool]] = None,
    yield_interval: int = 4096,
) -> Optional[dict[str, Any]]:
    """Return all triangle vertices and colours from one terrain MarkerArray.

    The returned mapping is always a full replacement: an empty ``points`` list
    means the terrain snapshot has no renderable cells (including a DELETEALL
    only message).  There is deliberately no vertex/point budget: every
    complete finite triangle in every accepted batch is returned.

    ``yield_control`` is called every ``yield_interval`` triangles.  It should
    release the interpreter lock briefly and return ``False`` when the
    snapshot is no longer wanted (a newer one arrived, or shutdown); this
    function then returns ``None`` and the caller discards the partial result.
    """
    points: list[list[float]] = []
    colors: list[list[float]] = []
    frame_id = ""
    interval = max(1, int(yield_interval))
    triangles = 0

    for marker in markers:
        try:
            action = int(getattr(marker, "action", MARKER_ADD))
            marker_type = int(getattr(marker, "type", 0))
        except (TypeError, ValueError, OverflowError):
            continue
        namespace = str(getattr(marker, "ns", "") or "")
        header = getattr(marker, "header", None)
        candidate_frame = str(getattr(header, "frame_id", "") or "")
        if candidate_frame:
            frame_id = candidate_frame
        if action == MARKER_DELETEALL:
            # A snapshot begins by clearing the previous display.  Keep reading
            # the following ADD batches in this same array.
            points.clear()
            colors.clear()
            continue
        if marker_type != MARKER_TRIANGLE_LIST or namespace != TERRAIN_CELL_NAMESPACE:
            continue
        if action not in (MARKER_ADD, MARKER_MODIFY):
            continue
        marker_points = getattr(marker, "points", None) or ()
        # The terrain publisher uses a world-coordinate marker with identity
        # pose, so points are already in ``frame_id``.
        for start in range(0, len(marker_points) - 2, 3):
            triangle = [
                _finite_point(marker_points[start + offset])
                for offset in range(3)
            ]
            if any(point is None for point in triangle):
                continue
            for offset in range(3):
                x, y, z = triangle[offset]  # type: ignore[misc]
                points.append([x, y, z])
                red, green, blue = _marker_color(marker, start + offset)
                colors.append([red, green, blue])
            triangles += 1
            if yield_control is not None and triangles % interval == 0:
                if not yield_control():
                    return None
    return {"points": points, "colors": colors, "frame_id": frame_id}


class TerrainIngestWorker:
    """Convert the newest terrain snapshot off the ROS executor thread.

    The grid planner's MarkerArray can hold hundreds of thousands of vertices.
    Converting it inside the subscription callback blocks the single-threaded
    executor (IMU, odometry and diagnostics timers) for seconds, so the
    callback only stores the latest message here.  The worker converts one
    snapshot at a time, releases the interpreter lock between chunks and drops
    a snapshot that a newer submission has superseded.
    """

    def __init__(
        self,
        apply: Callable[[dict[str, Any], int, float], Any],
        *,
        yield_interval: int = 4096,
        logger: Any = None,
        stop_timeout_sec: float = 1.0,
    ) -> None:
        self._apply = apply
        self._yield_interval = max(1, int(yield_interval))
        self._logger = logger
        self._stop_timeout = max(0.0, float(stop_timeout_sec))
        self._cond = threading.Condition(threading.RLock())
        self._pending: Optional[tuple[Any, int, float]] = None
        self._stopped = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        with self._cond:
            if self._thread is not None or self._stopped:
                return
            self._thread = threading.Thread(
                target=self._run, name="web_control_terrain", daemon=True
            )
            self._thread.start()

    def submit(self, msg: Any, *, generation: int, arrival_stamp: float) -> bool:
        """Replace the pending snapshot; never grows a queue."""
        with self._cond:
            if self._stopped:
                return False
            self._pending = (msg, int(generation), float(arrival_stamp))
            self._cond.notify()
            return True

    def is_alive(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def stop(self, timeout: Optional[float] = None) -> None:
        with self._cond:
            if not self._stopped:
                self._stopped = True
                self._pending = None
                self._cond.notify_all()
            thread = self._thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(self._stop_timeout if timeout is None else timeout)

    def _superseded(self) -> bool:
        with self._cond:
            return self._stopped or self._pending is not None

    def _convert(self, msg: Any, proceed: Callable[[], bool]) -> Optional[dict[str, Any]]:
        return terrain_markers_to_triangles(
            getattr(msg, "markers", ()) or (),
            yield_control=proceed,
            yield_interval=self._yield_interval,
        )

    def _run(self) -> None:
        while True:
            with self._cond:
                while not self._stopped and self._pending is None:
                    self._cond.wait(0.5)
                if self._stopped:
                    return
                msg, generation, arrival_stamp = self._pending
                self._pending = None

            def proceed() -> bool:
                # Yield the interpreter lock so ROS callbacks and HTTP threads
                # keep running, and abandon work a newer snapshot replaced.
                time.sleep(0)
                return not self._superseded()

            convert_started = time.monotonic()
            try:
                terrain = self._convert(msg, proceed)
            except Exception as exc:  # pragma: no cover - defensive
                if self._logger is not None:
                    try:
                        self._logger.warning(f"terrain conversion failed: {exc}")
                    except Exception:
                        pass
                continue
            convert_elapsed = time.monotonic() - convert_started
            if convert_elapsed >= 0.5 and self._logger is not None:
                try:
                    self._logger.warning(
                        f"terrain conversion took {convert_elapsed:.2f}s"
                    )
                except Exception:
                    pass
            if terrain is None:
                continue
            try:
                self._apply(terrain, generation, arrival_stamp)
            except Exception as exc:  # pragma: no cover - defensive
                if self._logger is not None:
                    try:
                        self._logger.warning(f"terrain apply failed: {exc}")
                    except Exception:
                        pass
