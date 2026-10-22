"""Thread-safe cache and forest builder for the ROS ``/tf`` tree.

The web node receives ``tf2_msgs/TFMessage`` on ``/tf`` (dynamic) and
``/tf_static`` (latched). This module keeps the latest parent for every child
frame and turns the edge set into a display forest that tolerates the messy
real world:

* several roots,
* frames with no parent,
* a child that changes parent,
* malformed or empty frame ids,
* self loops and longer cycles.

No frame is ever dropped because a well-known root (such as ``map``) is
missing. Edges that cannot be placed in the forest are returned in a separate
``detached`` list so the browser can still show them with a reason. Everything
here is pure Python so it can be imported and tested without ROS.
"""

from __future__ import annotations

import math
import threading
from typing import Any, Iterable, Optional

# Deep enough for real robot TF chains, small enough that the nested
# dict/list forest stays well under the JSON encoder's recursion limit
# (512 levels previously raised RecursionError on a 600-deep chain).
MAX_TREE_DEPTH = 128


def _finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _edge_view(edge: dict[str, Any], now: float, stale_timeout_sec: float) -> dict[str, Any]:
    static = bool(edge.get("static"))
    last_seen = _finite(edge.get("last_seen"))
    age = None if last_seen is None else max(0.0, float(now) - last_seen)
    stale = (not static) and age is not None and age > stale_timeout_sec
    return {
        "parent": edge.get("parent", ""),
        "child": edge.get("child", ""),
        "static": static,
        "stamp": _finite(edge.get("stamp")),
        "last_seen": last_seen,
        "age": age,
        "stale": stale,
    }


def build_tf_forest(
    edges: dict[str, dict[str, Any]],
    now: float,
    stale_timeout_sec: float = 2.0,
) -> dict[str, Any]:
    """Build a JSON-serialisable forest from ``child -> edge`` records."""
    parent_of: dict[str, dict[str, Any]] = {}
    frames: set[str] = set()
    detached: list[dict[str, Any]] = []

    for edge in edges.values():
        parent = str(edge.get("parent", "") or "")
        child = str(edge.get("child", "") or "")
        # Keep every real frame name, even when the edge itself is malformed or
        # a self loop, so a frame is never hidden just because its parent is
        # missing or invalid.
        if parent:
            frames.add(parent)
        if child:
            frames.add(child)
        if not parent or not child:
            detached.append(
                {**_edge_view(edge, now, stale_timeout_sec), "reason": "malformed frame"}
            )
            continue
        if parent == child:
            detached.append(
                {**_edge_view(edge, now, stale_timeout_sec), "reason": "self cycle"}
            )
            continue
        parent_of[child] = edge

    # Nodes that take part in a parent cycle (a -> b -> a). They are treated as
    # roots so the DFS terminates; their back edge is reported as detached.
    cycle_nodes: set[str] = set()
    color: dict[str, int] = {}
    for start in frames:
        if color.get(start):
            continue
        path: list[str] = []
        index: dict[str, int] = {}
        node: Optional[str] = start
        while node is not None and color.get(node) != 1 and color.get(node) != 2:
            color[node] = 1
            index[node] = len(path)
            path.append(node)
            edge = parent_of.get(node)
            node = str(edge["parent"]) if edge else None
        if node is not None and color.get(node) == 1:
            cycle_nodes.update(path[index[node] :])
        for member in path:
            color[member] = 2

    children: dict[str, list[str]] = {}
    for child, edge in parent_of.items():
        children.setdefault(str(edge["parent"]), []).append(child)
    for siblings in children.values():
        siblings.sort()

    roots = sorted(frame for frame in frames if frame not in parent_of or frame in cycle_nodes)
    visited: set[str] = set()

    def build(frame: str, stack: set[str], depth: int) -> dict[str, Any]:
        node: dict[str, Any] = {"frame": frame, "children": []}
        visited.add(frame)
        for child in children.get(frame, []):
            edge = parent_of.get(child)
            view = _edge_view(edge or {}, now, stale_timeout_sec)
            if child in stack:
                detached.append({**view, "reason": "cycle"})
                continue
            if child in visited:
                detached.append({**view, "reason": "extra parent"})
                continue
            if depth >= MAX_TREE_DEPTH:
                detached.append({**view, "reason": "tree too deep"})
                continue
            child_node = build(child, stack | {child}, depth + 1)
            child_node["parent"] = frame
            child_node["edge"] = view
            node["children"].append(child_node)
        return node

    trees: list[dict[str, Any]] = []
    for root in roots:
        if root in visited:
            continue
        tree = build(root, {root}, 1)
        tree["parent"] = None
        tree["edge"] = None
        trees.append(tree)
    # Defensive: any frame the loop above could not reach still gets shown.
    for frame in sorted(frames):
        if frame not in visited:
            tree = build(frame, {frame}, 1)
            tree["parent"] = None
            tree["edge"] = None
            trees.append(tree)

    return {
        "ok": True,
        "status": "waiting" if not frames else "active",
        "message": "waiting for /tf" if not frames else None,
        "stamp": _finite(now),
        "frame_count": len(frames),
        "edge_count": len(edges),
        "stale_timeout_sec": float(stale_timeout_sec),
        "trees": trees,
        "detached": detached,
    }


class TfTreeCache:
    """Bounded, thread-safe latest-edge cache for ``/tf`` and ``/tf_static``."""

    def __init__(self, max_edges: int = 4096, stale_timeout_sec: float = 2.0) -> None:
        self.max_edges = max(1, int(max_edges))
        self.stale_timeout_sec = float(stale_timeout_sec)
        self.lock = threading.RLock()
        # Dict insertion order doubles as the dynamic eviction order.
        self._edges: dict[str, dict[str, Any]] = {}
        self._malformed_seq = 0

    def clear(self) -> None:
        with self.lock:
            self._edges.clear()
            self._malformed_seq = 0

    def update_static(self, transforms: Iterable[dict[str, Any]], now: float) -> None:
        self._update(transforms, now, is_static=True)

    def update_dynamic(self, transforms: Iterable[dict[str, Any]], now: float) -> None:
        self._update(transforms, now, is_static=False)

    def _update(
        self, transforms: Iterable[dict[str, Any]], now: float, is_static: bool
    ) -> None:
        with self.lock:
            for item in transforms:
                parent = str(item.get("parent", "") or "")
                child = str(item.get("child", "") or "")
                if not child:
                    # Still remember malformed edges so the browser can report
                    # them; key them uniquely.
                    parent = parent or "?"
                if not parent and not child:
                    continue
                edge = {
                    "parent": parent,
                    "child": child,
                    "static": is_static,
                    "stamp": _finite(item.get("stamp")),
                    "last_seen": _finite(now) or 0.0,
                }
                if child:
                    key = child
                else:
                    self._malformed_seq += 1
                    key = f"__malformed_{self._malformed_seq}"
                if key not in self._edges and len(self._edges) >= self.max_edges:
                    self._evict_oldest_dynamic()
                self._edges[key] = edge

    def _evict_oldest_dynamic(self) -> None:
        for key in list(self._edges):
            if not self._edges[key].get("static"):
                self._edges.pop(key, None)
                return
        if self._edges:
            self._edges.pop(next(iter(self._edges)), None)

    def snapshot(self, now: float) -> dict[str, Any]:
        with self.lock:
            copied = {key: dict(value) for key, value in self._edges.items()}
        return build_tf_forest(copied, now, self.stale_timeout_sec)
