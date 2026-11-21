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
from typing import Any, Iterable

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


def terrain_markers_to_triangles(markers: Iterable[Any]) -> dict[str, Any]:
    """Return all triangle vertices and colours from one terrain MarkerArray.

    The returned mapping is always a full replacement: an empty ``points`` list
    means the terrain snapshot has no renderable cells (including a DELETEALL
    only message).  There is deliberately no vertex/point budget: every
    complete finite triangle in every accepted batch is returned.
    """
    points: list[list[float]] = []
    colors: list[list[float]] = []
    frame_id = ""

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
    return {"points": points, "colors": colors, "frame_id": frame_id}
