"""Focused contracts for the navigation global-map and obstacle layers.

These tests stay ROS-free.  They bind the real node methods to small stubs so
the subscription topics, QoS profiles, layer routing and lifecycle clearing are
executed rather than only checked as source text.
"""

import os
import shutil
import subprocess
import threading
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest

from web_control import web_control_node
from web_control.core import SourceCache
from web_control.map_catalog import MapCatalog
from web_control.map_selection import SelectionStore


class _QosProfile:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class _SubscriptionNode:
    def __init__(self, **overrides):
        self.parameters = {
            "global_map_topic": "/lio_sam/localization/global_map",
            "local_map_topic": "/lio_sam/localization/local_map",
            "mapcloud_topic": "mapcloud",
            "mapground_topic": "mapground",
            "obstacle_topic": "/aggregated_pc",
            "mapping_global_map_topic": "/lio_sam/mapping/map_global",
            "mapping_ground_topic": "/lio_sam/mapping/ground_cloud_global",
        }
        self.parameters.update(overrides)
        self.subscriptions = []

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_subscription(self, msg_type, topic, callback, qos, **kwargs):
        self.subscriptions.append(
            {
                "msg_type": msg_type,
                "topic": topic,
                "callback": callback,
                "qos": qos,
                "kwargs": kwargs,
            }
        )
        return object()

    def pointcloud_callback(self, layer_name, _msg):
        return layer_name

    def obstacle_callback(self, _msg):
        return "obstacles"

    def mapping_pointcloud_callback(self, layer_name, _msg):
        return layer_name


def _configure_subscriptions(monkeypatch, node):
    monkeypatch.setattr(web_control_node, "QoSProfile", _QosProfile)
    monkeypatch.setattr(
        web_control_node,
        "QoSHistoryPolicy",
        SimpleNamespace(KEEP_LAST="keep_last"),
    )
    monkeypatch.setattr(
        web_control_node,
        "QoSReliabilityPolicy",
        SimpleNamespace(RELIABLE="reliable", BEST_EFFORT="best_effort"),
    )
    monkeypatch.setattr(
        web_control_node,
        "QoSDurabilityPolicy",
        SimpleNamespace(TRANSIENT_LOCAL="transient_local", VOLATILE="volatile"),
    )
    monkeypatch.setattr(web_control_node, "PointCloud2", object)
    node.configure_pointcloud_subscriptions = MethodType(
        web_control_node.WebControlNode.configure_pointcloud_subscriptions, node
    )
    node.configure_pointcloud_subscriptions()
    return {entry["topic"]: entry for entry in node.subscriptions}


def test_actual_subscriptions_use_required_topics_and_qos(monkeypatch):
    node = _SubscriptionNode()
    by_topic = _configure_subscriptions(monkeypatch, node)

    assert set(by_topic) == {
        "/lio_sam/localization/global_map",
        "/lio_sam/localization/local_map",
        "mapcloud",
        "mapground",
        "/aggregated_pc",
        "/lio_sam/mapping/map_global",
        "/lio_sam/mapping/ground_cloud_global",
    }

    global_map = by_topic["/lio_sam/localization/global_map"]
    assert global_map["qos"].kwargs == {
        "history": "keep_last",
        "depth": 1,
        "reliability": "reliable",
        "durability": "transient_local",
    }
    assert global_map["callback"].func.__self__ is node
    assert global_map["callback"].func.__func__ is _SubscriptionNode.pointcloud_callback
    assert global_map["callback"].args == ("global_map",)

    obstacles = by_topic["/aggregated_pc"]
    assert obstacles["qos"].kwargs == {
        "history": "keep_last",
        "depth": 1,
        "reliability": "best_effort",
        "durability": "volatile",
    }
    assert obstacles["callback"] == node.obstacle_callback

    # Existing static map subscriptions and the independent mapping topic keep
    # their previous QoS contracts.
    for topic in ("/lio_sam/localization/local_map", "mapcloud", "mapground"):
        assert by_topic[topic]["qos"].kwargs["reliability"] == "reliable"
        assert by_topic[topic]["qos"].kwargs["durability"] == "transient_local"
    assert (
        by_topic["/lio_sam/mapping/map_global"]["qos"].kwargs["reliability"]
        == "best_effort"
    )


def _layer_node(now=10.0):
    class _Node:
        def __init__(self):
            self.lock = threading.RLock()
            self.now = now
            self.layers = {
                "global_map": SourceCache("global_map", 2.0, static_source=True),
                "local_map": SourceCache("local_map", 2.0, static_source=True),
                "mapcloud": SourceCache("mapcloud", 2.0, static_source=True),
                "mapground": SourceCache("mapground", 2.0, static_source=True),
                "obstacles": SourceCache("obstacles", 2.0, static_source=False),
            }

        def now_sec(self):
            return self.now

    node = _Node()
    node.layer_payload = MethodType(web_control_node.WebControlNode.layer_payload, node)
    return node


def test_global_map_and_obstacles_route_through_layer_api_with_freshness():
    node = _layer_node()
    node.layers["global_map"].update(
        {"points3d": [[1.0, 2.0, 3.0]]},
        stamp=10.0,
        frame_id="map",
        count=1,
    )
    node.layers["obstacles"].update(
        {"points3d": [[4.0, 5.0, 0.0]]},
        stamp=10.0,
        frame_id="map",
        count=1,
    )

    global_payload = node.layer_payload("global_map", "3d")
    assert global_payload == {
        "layer": "global_map",
        "status": "loaded",
        "frame_id": "map",
        "count": 1,
        "points": [[1.0, 2.0, 3.0]],
    }
    obstacle_payload = node.layer_payload("obstacles", "3d")
    assert obstacle_payload["status"] == "active"
    assert obstacle_payload["points"] == [[4.0, 5.0, 0.0]]

    node.now = 12.1
    assert node.layer_payload("obstacles", "3d")["status"] == "stale"
    # A latched localization global map remains loaded even when no new sample
    # arrives, matching the other static map caches.
    assert node.layer_payload("global_map", "3d")["status"] == "loaded"


class _PointCloudMsg:
    def __init__(self, frame_id="map"):
        self.header = SimpleNamespace(frame_id=frame_id)


def _point_cache_node(monkeypatch, rows):
    monkeypatch.setattr(
        web_control_node,
        "point_cloud2",
        SimpleNamespace(
            read_points=lambda _msg, field_names, skip_nans: iter(rows)
        ),
    )
    node = _layer_node()
    node.mapping_layers = {
        "global_map": SourceCache(
            "mapping_global_map", 2.0, static_source=True
        ),
        "ground": SourceCache("mapping_ground", 2.0, static_source=True),
    }
    node.max_2d_points = 60
    node.max_3d_points = 40
    node.update_pointcloud_cache = MethodType(
        web_control_node.WebControlNode.update_pointcloud_cache, node
    )
    node.obstacle_callback = MethodType(
        web_control_node.WebControlNode.obstacle_callback, node
    )
    node.pointcloud_callback = MethodType(
        web_control_node.WebControlNode.pointcloud_callback, node
    )
    node.mapping_pointcloud_callback = MethodType(
        web_control_node.WebControlNode.mapping_pointcloud_callback, node
    )
    node.layer_payload = MethodType(
        web_control_node.WebControlNode.layer_payload, node
    )
    node.mapping_layer_payload = MethodType(
        web_control_node.WebControlNode.mapping_layer_payload, node
    )
    return node


def test_empty_obstacle_update_clears_the_display(monkeypatch):
    node = _point_cache_node(monkeypatch, [])
    node.layers["obstacles"].update(
        {"points3d": [[1.0, 2.0, 3.0]]},
        stamp=10.0,
        frame_id="map",
        count=1,
    )

    node.obstacle_callback(_PointCloudMsg())

    assert node.layers["obstacles"].data is None
    assert node.layers["obstacles"].count == 0
    assert node.layers["obstacles"].status(10.0) == "waiting"

    # Existing map layers keep their historical "ignore empty sample" behavior;
    # only the live obstacle layer clears on an empty sample.
    node.layers["global_map"].update(
        {"points3d": [[1.0, 2.0, 3.0]]},
        stamp=10.0,
        frame_id="map",
        count=1,
    )
    node.pointcloud_callback("global_map", _PointCloudMsg())
    assert node.layers["global_map"].data is not None


def test_obstacle_update_caches_points_and_preserves_source_frame(monkeypatch):
    rows = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (float("nan"), 0.0, 0.0)]
    node = _point_cache_node(monkeypatch, rows)

    node.obstacle_callback(_PointCloudMsg(frame_id="base_link"))

    cache = node.layers["obstacles"]
    assert cache.data == {"points3d": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]}
    assert "points2d" not in cache.data
    assert cache.frame_id == "base_link"
    assert cache.count == 2
    assert cache.status(10.0) == "active"


def test_pointcloud_hot_path_downsamples_3d_once_and_never_projects_2d(monkeypatch):
    downsample_calls = []

    def fake_downsample(points, max_points):
        downsample_calls.append((len(points), max_points))
        return list(points)

    def fail_projection(_points):
        raise AssertionError("2D projection must not run during point-cloud ingestion")

    rows = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    node = _point_cache_node(monkeypatch, rows)
    monkeypatch.setattr(web_control_node, "downsample_points", fake_downsample)
    monkeypatch.setattr(web_control_node, "project_points_2d", fail_projection)
    expected = {"points3d": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]}

    node.pointcloud_callback("local_map", _PointCloudMsg())
    node.obstacle_callback(_PointCloudMsg())
    node.mapping_pointcloud_callback("global_map", _PointCloudMsg())

    assert downsample_calls == [(2, 40), (2, 40), (2, 40)]
    assert node.layers["local_map"].data == expected
    assert node.layers["obstacles"].data == expected
    assert node.mapping_layers["global_map"].data == expected


def test_mapping_cache_keeps_3d_payload_metadata_without_2d_copy(monkeypatch):
    rows = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    node = _point_cache_node(monkeypatch, rows)

    node.mapping_pointcloud_callback("global_map", _PointCloudMsg(frame_id="map"))

    cache = node.mapping_layers["global_map"]
    assert cache.data == {"points3d": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]}
    assert cache.frame_id == "map"
    assert cache.count == 2
    assert cache.status(10.0) == "loaded"
    assert node.mapping_layer_payload("global_map") == {
        "layer": "global_map",
        "status": "loaded",
        "frame_id": "map",
        "count": 2,
        "points": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
    }


def test_legacy_map2d_projects_cached_3d_on_request_without_mutating_cache(
    monkeypatch,
):
    rows = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    node = _point_cache_node(monkeypatch, rows)
    node.obstacle_callback(_PointCloudMsg(frame_id="map"))
    cached = node.layers["obstacles"].data

    payload = node.layer_payload("obstacles", "2d")

    assert payload == {
        "layer": "obstacles",
        "status": "active",
        "frame_id": "map",
        "count": 2,
        "points": [[1.0, 2.0], [4.0, 5.0]],
    }
    assert node.layers["obstacles"].data is cached
    assert cached == {"points3d": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]}

    # A later cloud update is reflected by the next legacy request.
    monkeypatch.setattr(
        web_control_node,
        "point_cloud2",
        SimpleNamespace(
            read_points=lambda _msg, field_names, skip_nans: iter([(7.0, 8.0, 9.0)])
        ),
    )
    node.obstacle_callback(_PointCloudMsg())
    assert node.layer_payload("obstacles", "2d")["points"] == [[7.0, 8.0]]

    # Stale live data keeps reporting its last projected points.
    node.now = 12.1
    stale = node.layer_payload("obstacles", "2d")
    assert stale["status"] == "stale"
    assert stale["points"] == [[7.0, 8.0]]

    # An empty live sample clears the obstacle cache; the legacy view follows.
    node.now = 10.0
    monkeypatch.setattr(
        web_control_node,
        "point_cloud2",
        SimpleNamespace(read_points=lambda _msg, field_names, skip_nans: iter([])),
    )
    node.obstacle_callback(_PointCloudMsg())
    empty = node.layer_payload("obstacles", "2d")
    assert empty["status"] == "waiting"
    assert empty["count"] == 0
    assert empty["points"] == []


def test_legacy_map2d_cap_limits_output_without_expanding_cached_3d(monkeypatch):
    rows = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)]
    node = _point_cache_node(monkeypatch, rows)
    node.max_2d_points = 2
    node.max_3d_points = 40
    node.obstacle_callback(_PointCloudMsg())

    payload_3d = node.layer_payload("obstacles", "3d")
    payload_2d = node.layer_payload("obstacles", "2d")

    assert payload_3d["points"] == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]
    assert len(payload_2d["points"]) <= 2
    assert all(len(point) == 2 for point in payload_2d["points"])
    assert node.layers["obstacles"].data == {
        "points3d": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]
    }


def test_legacy_map2d_projects_after_releasing_the_cache_lock(monkeypatch):
    node = _layer_node()
    node.lock = threading.Lock()
    node.max_2d_points = 60
    node.layers["obstacles"].update(
        {"points3d": [[1.0, 2.0, 3.0]]}, stamp=10.0, frame_id="map", count=1
    )
    lock_states = []

    def project(points):
        acquired = node.lock.acquire(blocking=False)
        lock_states.append(acquired)
        if acquired:
            node.lock.release()
        return [[float(x), float(y)] for x, y, _z in points]

    monkeypatch.setattr(web_control_node, "project_points_2d", project)

    payload = node.layer_payload("obstacles", "2d")

    assert payload["points"] == [[1.0, 2.0]]
    assert lock_states == [True]


class _NavigationControlStub:
    def __init__(self):
        self.stops = []

    def on_navigation_stopped(self, now):
        self.stops.append(now)


def test_navigation_stop_clears_nav_clouds_but_keeps_localization_map():
    node = _layer_node()
    for name in node.layers:
        node.layers[name].update(
            {"points3d": [[1.0, 2.0, 3.0]]},
            stamp=10.0,
            frame_id="map",
            count=1,
        )
    node.action_status = SourceCache("action_status", 2.0)
    node.paths = {}
    node.published_goal = {"x": 1.0}
    node.navigation_control = _NavigationControlStub()
    node.clear_navigation_runtime_state = MethodType(
        web_control_node.WebControlNode.clear_navigation_runtime_state, node
    )

    node.clear_navigation_runtime_state()

    assert node.layers["global_map"].data is not None
    assert node.layers["local_map"].data is not None
    assert node.layers["mapcloud"].data is None
    assert node.layers["mapground"].data is None
    assert node.layers["obstacles"].data is None
    assert node.published_goal is None
    assert node.navigation_control.stops == [10.0]


def test_localization_restart_clears_global_map_and_all_runtime_clouds():
    node = _layer_node()
    for name in node.layers:
        node.layers[name].update(
            {"points3d": [[1.0, 2.0, 3.0]]},
            stamp=10.0,
            frame_id="map",
            count=1,
        )
    node.odom = SourceCache("odom", 2.0)
    node.action_status = SourceCache("action_status", 2.0)
    node.paths = {}
    node.clear_map_runtime_state = MethodType(
        web_control_node.WebControlNode.clear_map_runtime_state, node
    )

    node.clear_map_runtime_state()

    assert all(cache.data is None for cache in node.layers.values())
    assert node.layers["global_map"].data is None
    assert node.odom.data is None
    assert node.action_status.data is None


def _make_map(root, name):
    directory = root / name
    directory.mkdir(parents=True)
    (directory / "GlobalMap.pcd").write_text("map\n", encoding="ascii")
    (directory / "GroundMap.pcd").write_text("ground\n", encoding="ascii")
    return directory


def test_map_change_clears_previous_global_map(tmp_path):
    root = tmp_path / "map"
    _make_map(root, "site_a")
    _make_map(root, "site_b")

    class _Node:
        def __init__(self):
            self.lock = threading.RLock()
            self.layers = _layer_node().layers
            self.map_catalog = MapCatalog(root)
            self.map_selection = SelectionStore(tmp_path / "selected_map.json")
            self.managed_processes = {}
            self.navigation_control = SimpleNamespace(
                note_map_selected=lambda name, now: None
            )

        def now_sec(self):
            return 10.0

    node = _Node()
    node.layers["global_map"].update(
        {"points3d": [[1.0, 2.0, 3.0]]},
        stamp=10.0,
        frame_id="map",
        count=1,
    )
    node.layers["local_map"].update(
        {"points3d": [[9.0, 9.0, 9.0]]},
        stamp=10.0,
        frame_id="map",
        count=1,
    )
    node.select_map = MethodType(web_control_node.WebControlNode.select_map, node)

    response = node.select_map({"name": "site_a"})

    assert response["selection"]["name"] == "site_a"
    assert node.layers["global_map"].data is None
    assert node.layers["local_map"].data is not None


_FRONTEND_MATERIAL_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("function makePointCloud");
if (start < 0) throw new Error("makePointCloud not found");
const brace = source.indexOf(") {", start) + 2;
if (brace < 2) throw new Error("makePointCloud signature not found");
let depth = 0;
let end = -1;
for (let i = brace; i < source.length; i += 1) {
  if (source[i] === "{") depth += 1;
  else if (source[i] === "}") {
    depth -= 1;
    if (depth === 0) { end = i + 1; break; }
  }
}
if (end < 0) throw new Error("unterminated makePointCloud");

const THREE = {
  BufferGeometry: class { setAttribute() {} },
  BufferAttribute: class { constructor(array, itemSize) { this.array = array; this.itemSize = itemSize; } },
  PointsMaterial: class { constructor(options) { Object.assign(this, options); } },
  Points: class { constructor(geometry, material) { this.geometry = geometry; this.material = material; } },
};
function rosToThree(x, y, z) { return { x, y, z }; }
const api = new Function(
  "THREE", "rosToThree",
  source.slice(start, end) + "\nreturn { makePointCloud };",
)(THREE, rosToThree);
const { makePointCloud } = api;

function expect(condition, message) { if (!condition) throw new Error(message); }

const normal = makePointCloud([], 0x6f8fb0, { opacity: 0.72 });
expect(normal.material.color === 0x6f8fb0, "global map keeps its gray colour");
expect(normal.material.size === 2, "normal map dots use 2 px");
expect(normal.material.sizeAttenuation === false, "map dot size is stable in screen pixels");
expect(normal.material.vertexColors === false, "map dots use flat material colour");

const obstacle = makePointCloud([[1, 2, 3]], 0xff0000, {
  pointSize: 5, opacity: 1.0, flatColor: true,
});
expect(obstacle.material.color === 0xff0000, "obstacles are flat red");
expect(obstacle.material.size === 5, "obstacle dots are visibly larger (5 px)");
expect(obstacle.material.sizeAttenuation === false, "obstacle dot size is stable in screen pixels");
expect(obstacle.material.vertexColors === false, "height/intensity tinting cannot override red");
expect(obstacle.material.opacity === 1.0, "obstacles are fully opaque");
console.log("layer material ok");
"""


def test_point_material_keeps_obstacles_flat_red_and_larger():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _FRONTEND_MATERIAL_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "layer material ok" in completed.stdout


def test_frontend_enables_both_layers_and_clears_obstacles_on_navigation_stop():
    root = Path(__file__).resolve().parents[1]
    index_html = (root / "static" / "index.html").read_text(encoding="utf-8")
    app_js = (root / "static" / "app.js").read_text(encoding="utf-8")
    navigation_layers = app_js[
        app_js.index("const LAYERS = {") : app_js.index("const MAPPING_LAYERS = {")
    ]

    assert 'data-layer="global_map" checked> global map' in index_html
    assert 'data-layer="obstacles" checked> obstacles' in index_html
    assert "global_map: { color: 0x6f8fb0, points3d: [], enabled: true" in navigation_layers
    assert "obstacles: { color: 0xff0000, points3d: [], enabled: true" in navigation_layers
    assert "pointSize: 5" in navigation_layers
    assert "flatColor: true" in navigation_layers
    assert '["mapcloud", "mapground", "obstacles"].forEach' in app_js
    assert "LAYERS[input.dataset.layer].enabled = input.checked;" in app_js
    assert "fullmap" not in app_js
