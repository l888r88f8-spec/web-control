import ast
import json
import math
import os
import pytest
import shutil
import signal
import subprocess
import threading
import urllib.error
import urllib.request
import yaml
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import MethodType, SimpleNamespace

from web_control import core, web_control_node
from web_control.core import (
    SourceCache,
    TerminalLogBuffer,
    ViewerRegistry,
    build_managed_launch_command,
    build_system_usage_payload,
    calculate_cpu_usage,
    downsample_points,
    parse_proc_meminfo,
    parse_proc_stat_cpu_ticks,
    project_points_2d,
    quaternion_from_yaw,
    validate_pose_payload,
    yaw_from_quaternion,
)
from web_control.navigation_control import PENDING_CLEAR
from web_control.teleop import TeleopController


def test_yaw_quaternion_round_trip():
    q = quaternion_from_yaw(math.pi / 2.0)
    assert q["x"] == 0.0
    assert q["y"] == 0.0
    assert abs(q["z"] - math.sqrt(0.5)) < 1e-9
    assert abs(q["w"] - math.sqrt(0.5)) < 1e-9
    assert abs(yaw_from_quaternion(q) - math.pi / 2.0) < 1e-9


def test_validate_pose_payload_accepts_numbers_and_defaults_z():
    pose = validate_pose_payload({"x": "1.5", "y": 2, "yaw": "-0.25"})
    assert pose == {"x": 1.5, "y": 2.0, "z": 0.0, "yaw": -0.25}


def test_validate_pose_payload_rejects_missing_and_non_finite_values():
    try:
        validate_pose_payload({"x": 1.0, "yaw": 0.0})
    except ValueError as exc:
        assert "missing y" in str(exc)
    else:
        raise AssertionError("missing y was accepted")

    try:
        validate_pose_payload({"x": 1.0, "y": 2.0, "yaw": float("nan")})
    except ValueError as exc:
        assert "finite" in str(exc)
    else:
        raise AssertionError("NaN yaw was accepted")


def test_project_points_2d_and_downsample_points_are_deterministic():
    points = [(float(i), float(i + 1), float(i + 2)) for i in range(10)]
    assert project_points_2d(points)[:3] == [[0.0, 1.0], [1.0, 2.0], [2.0, 3.0]]
    assert downsample_points(points, 4) == [
        (0.0, 1.0, 2.0),
        (3.0, 4.0, 5.0),
        (6.0, 7.0, 8.0),
        (9.0, 10.0, 11.0),
    ]


def test_source_cache_status_transitions():
    cache = SourceCache("map", stale_timeout_sec=1.0)
    assert cache.status(now=10.0) == "waiting"

    cache.update({"points": []}, stamp=10.0)
    assert cache.status(now=10.5) == "active"
    assert cache.status(now=11.5) == "stale"


def test_static_source_cache_stays_loaded_after_first_message():
    cache = SourceCache("mapground", stale_timeout_sec=1.0, static_source=True)
    assert cache.status(now=10.0) == "waiting"

    cache.update({"points": []}, stamp=10.0)
    assert cache.status(now=10.5) == "loaded"
    assert cache.status(now=30.0) == "loaded"


def test_source_cache_clear_resets_static_source_to_waiting():
    cache = SourceCache("mapground", stale_timeout_sec=1.0, static_source=True)
    cache.update({"points3d": [[1.0, 2.0, 3.0]]}, stamp=10.0, frame_id="map", count=1)

    cache.clear()

    assert cache.status(now=30.0) == "waiting"
    assert cache.data is None
    assert cache.stamp is None
    assert cache.frame_id == ""
    assert cache.count == 0


def test_viewer_registry_counts_recent_heartbeats_and_expires_old_sessions():
    viewers = ViewerRegistry(timeout_sec=10.0)

    assert viewers.count(now=100.0) == 0
    assert viewers.heartbeat("tab-a", now=100.0) == 1
    assert viewers.heartbeat("tab-b", now=104.0) == 2
    assert viewers.count(now=109.9) == 2
    assert viewers.count(now=111.0) == 1


def test_navigation_readiness_waits_for_relocalization_then_navigation():
    from web_control.web_control_node import navigation_readiness

    assert navigation_readiness("waiting", "waiting") == {
        "ready": False,
        "status": "waiting_for_relocalization",
        "message": "waiting for relocalization",
    }
    assert navigation_readiness("active", "waiting") == {
        "ready": False,
        "status": "waiting_for_navigation",
        "message": "waiting for navigation",
    }
    assert navigation_readiness("active", "loaded") == {
        "ready": True,
        "status": "ready",
        "message": "ready",
    }


def test_normalize_layer_name_rejects_unknown_layer():
    from web_control.web_control_node import normalize_layer_name

    assert normalize_layer_name("local_map") == "local_map"
    try:
        normalize_layer_name("global_map")
    except ValueError as exc:
        assert "unknown layer" in str(exc)
    else:
        raise AssertionError("navigation global_map layer was accepted")
    try:
        normalize_layer_name("../bad")
    except ValueError as exc:
        assert "unknown layer" in str(exc)
    else:
        raise AssertionError("unknown layer was accepted")


def _layer_payload_node():
    """ROS-free WebControlNode stand-in for the navigation layer endpoint."""

    class _LayerPayloadNode:
        def __init__(self):
            self.lock = threading.RLock()
            self.layers = {
                name: SourceCache(name, stale_timeout_sec=2.0, static_source=True)
                for name in ("local_map", "mapcloud", "mapground")
            }

        def now_sec(self):
            return 10.0

    node = _LayerPayloadNode()
    node.layer_payload = MethodType(
        web_control_node.WebControlNode.layer_payload, node
    )
    return node


def test_navigation_layer_payload_serves_local_map_cloud():
    node = _layer_payload_node()
    node.layers["local_map"].update(
        {"points3d": [[1.0, 2.0, 3.0]], "points2d": [[1.0, 2.0]]},
        stamp=10.0,
        frame_id="map",
        count=1,
    )

    assert node.layer_payload("local_map", "3d") == {
        "layer": "local_map",
        "status": "loaded",
        "frame_id": "map",
        "count": 1,
        "points": [[1.0, 2.0, 3.0]],
    }
    with pytest.raises(ValueError, match="unknown layer"):
        node.layer_payload("global_map", "3d")


def test_navigation_page_replaces_localization_global_map_with_local_map():
    package_root = Path(__file__).resolve().parents[1]
    static_dir = package_root / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    config_yaml = (package_root / "config" / "web_control.yaml").read_text(
        encoding="utf-8"
    )
    node_source = (
        package_root / "web_control" / "web_control_node.py"
    ).read_text(encoding="utf-8")

    # The navigation layer set is localization local_map + navigation clouds.
    assert web_control_node.LAYER_NAMES == {"local_map", "mapcloud", "mapground"}

    # Navigation sidebar checkbox and JavaScript layer declaration.
    assert 'data-layer="local_map"' in index_html
    assert 'data-layer="local_map" checked> local map' in index_html
    assert 'data-layer="mapcloud" checked> global map' in index_html
    assert 'data-layer="mapground" checked> ground map' in index_html
    assert 'data-layer="global_map"' not in index_html
    navigation_layers = app_js[
        app_js.index("const LAYERS = {") : app_js.index("const MAPPING_LAYERS = {")
    ]
    assert "local_map: { color: 0x8ea4b8" in navigation_layers
    assert "mapcloud: { color: 0x49b59f, points3d: [], enabled: true" in navigation_layers
    assert "global_map" not in navigation_layers

    # ROS parameter, cache, subscription, and API default layer.
    assert (
        'self.declare_parameter("local_map_topic", "/lio_sam/localization/local_map")'
        in node_source
    )
    assert '"local_map": SourceCache("local_map"' in node_source
    assert '"local_map": str(self.get_parameter("local_map_topic").value)' in node_source
    assert 'layer = query.get("layer", ["local_map"])[0]' in node_source
    assert 'local_map_topic: "/lio_sam/localization/local_map"' in config_yaml


def test_navigation_page_no_longer_declares_localization_global_map():
    package_root = Path(__file__).resolve().parents[1]
    node_source = (
        package_root / "web_control" / "web_control_node.py"
    ).read_text(encoding="utf-8")
    config_yaml = (package_root / "config" / "web_control.yaml").read_text(
        encoding="utf-8"
    )

    assert (
        'self.declare_parameter("global_map_topic", "/lio_sam/localization/global_map")'
        not in node_source
    )
    assert (
        '"global_map": SourceCache("global_map"' not in node_source
    )
    assert "    global_map_topic:" not in config_yaml


def test_mapping_page_keeps_its_global_map_layer_and_topics():
    package_root = Path(__file__).resolve().parents[1]
    static_dir = package_root / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    config_yaml = (package_root / "config" / "web_control.yaml").read_text(
        encoding="utf-8"
    )
    node_source = (
        package_root / "web_control" / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert web_control_node.MAPPING_LAYER_NAMES == {"global_map", "ground"}
    assert 'data-mapping-layer="global_map"' in index_html
    assert 'data-mapping-layer="ground"' in index_html
    mapping_layers = app_js[
        app_js.index("const MAPPING_LAYERS = {") : app_js.index(
            "const TARGET_MARKER_COLOR"
        )
    ]
    assert "global_map: { color: 0x8ea4b8" in mapping_layers
    assert (
        'self.declare_parameter("mapping_global_map_topic", "/lio_sam/mapping/map_global")'
        in node_source
    )
    assert '"global_map": SourceCache("mapping_global_map"' in node_source
    assert 'mapping_global_map_topic: "/lio_sam/mapping/map_global"' in config_yaml
    # The mapping map3d endpoint keeps its own global_map default.
    assert 'layer = query.get("layer", ["global_map"])[0]' in node_source


class _RecordingLayerWebNode:
    """ROS-free node recording which layer the GET router requests."""

    def __init__(self):
        self.calls = []
        self.static_dir = Path(__file__).resolve().parents[1] / "static"

    def layer_payload(self, layer, mode):
        self.calls.append((layer, mode))
        return {"layer": layer, "mode": mode, "points": []}

    def now_sec(self):
        return 1.0

    def get_logger(self):
        return _LoggerStub()


def test_navigation_map_layer_api_defaults_to_local_map():
    node = _RecordingLayerWebNode()
    server, thread = _serve_fake_web_node(node)
    try:
        base = f"http://127.0.0.1:{server.server_address[1]}"
        status_3d, payload_3d = _json_request(f"{base}/api/map3d")
        status_2d, payload_2d = _json_request(f"{base}/api/map2d")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status_3d == 200
    assert payload_3d["layer"] == "local_map"
    assert status_2d == 200
    assert payload_2d["layer"] == "local_map"
    assert node.calls == [("local_map", "3d"), ("local_map", "2d")]


def test_success_response_has_expected_shape():
    from web_control.web_control_node import success_response

    response = success_response("published", stamp=12.5)
    assert response == {"ok": True, "message": "published", "stamp": 12.5}


def test_action_status_name_uses_latest_status():
    from web_control.web_control_node import action_status_name

    assert action_status_name([]) == "idle"
    assert action_status_name([1]) == "accepted"
    assert action_status_name([2]) == "executing"
    assert action_status_name([4]) == "succeeded"
    assert action_status_name([6]) == "aborted"
    assert action_status_name([4, 2]) == "executing"


def test_view_mode_clears_transient_selection_marker():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert 'if (mode === "view")' in app_js
    assert "state.selection = null;" in app_js


def test_frontend_uses_threejs_3d_map_without_2d_tab():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    setup_py = (Path(__file__).resolve().parents[1] / "setup.py").read_text(encoding="utf-8")

    assert 'src="/app.js"' in index_html
    assert 'type="module"' in index_html
    assert 'id="tab2d"' not in index_html
    assert 'id="view2d"' not in index_html
    assert 'id="tab3d"' not in index_html
    assert 'id="tab3dMap"' in index_html
    assert ">Navigation</button>" in index_html
    assert 'data-active="3d"' in index_html
    assert 'id="mapCanvas3d"' in index_html
    assert 'import * as THREE from "./vendor/three.module.js";' in app_js
    assert 'view: "3d"' in app_js
    assert 'static/vendor' in setup_py


def test_frontend_exposes_mapping_tab_and_mapping_sidebar():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert 'id="tabMapping"' in index_html
    assert "Mapping" in index_html
    assert 'id="mappingSidebar"' in index_html
    assert 'id="navigationSidebar"' in index_html
    assert 'data-panel="navigation"' in index_html
    assert 'data-panel="mapping"' in index_html
    assert 'id="mappingPoseX"' in index_html
    assert 'data-mapping-layer="global_map"' in index_html
    assert 'data-mapping-layer="ground"' in index_html
    assert 'id="mappingPathLayer"' in index_html
    assert 'id="mappingVehicleLayer"' in index_html
    assert 'id="mappingSources"' in index_html
    assert 'document.getElementById("tabMapping").classList.toggle("active", tab === "mapping");' in app_js
    assert 'document.querySelectorAll("[data-panel]")' in app_js
    assert 'panel.classList.toggle("active", panel.dataset.panel === panelName)' in app_js
    assert ".panel-view" in styles_css
    assert ".panel-view.active" in styles_css


def test_frontend_polls_mapping_3d_sources_only_for_mapping_tab():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "const MAPPING_LAYERS = {" in app_js
    assert 'global_map: { color: 0x8ea4b8' in app_js
    assert 'ground: { color: 0xd9a441' in app_js
    assert "const mappingLayerGroup = new THREE.Group();" in app_js
    assert "const mappingPathGroup = new THREE.Group();" in app_js
    assert "const mappingVehicleGroup = new THREE.Group();" in app_js
    assert 'getJson(`/api/mapping/map3d?layer=${encodeURIComponent(name)}`)' in app_js
    assert 'getJson("/api/mapping/path")' in app_js
    assert 'if (state.view !== "mapping") return;' in app_js
    assert "setInterval(() => pollMapping3d(false), 5000)" in app_js


def test_web_control_subscribes_mapping_topics_and_exposes_mapping_api():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert 'self.declare_parameter("mapping_global_map_topic", "/lio_sam/mapping/map_global")' in web_control_node
    assert 'self.declare_parameter("mapping_ground_topic", "/lio_sam/mapping/ground_cloud_global")' in web_control_node
    assert 'self.declare_parameter("mapping_path_topic", "/lio_sam/mapping/path")' in web_control_node
    assert 'self.declare_parameter("mapping_odom_topic", "/lio_sam/mapping/odometry")' in web_control_node
    assert "self.mapping_layers = {" in web_control_node
    assert "self.mapping_path = SourceCache" in web_control_node
    assert "self.mapping_odom = SourceCache" in web_control_node
    assert "self.mapping_pointcloud_callback" in web_control_node
    assert "self.mapping_path_callback" in web_control_node
    assert "self.mapping_odom_callback" in web_control_node
    assert "def mapping_state_payload(self)" in web_control_node
    assert "def mapping_layer_payload(self, layer_name: str)" in web_control_node
    assert "def mapping_path_payload(self)" in web_control_node
    assert 'elif parsed.path == "/api/mapping/state":' in web_control_node
    assert 'elif parsed.path == "/api/mapping/map3d":' in web_control_node
    assert 'elif parsed.path == "/api/mapping/path":' in web_control_node


def test_web_control_exposes_lightweight_pose_endpoint_for_realtime_vehicle_updates():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "def pose_payload(self)" in web_control_node
    assert '"mapping_odom": {' in web_control_node
    assert 'elif parsed.path == "/api/pose":' in web_control_node
    assert "node.pose_payload()" in web_control_node


def test_frontend_polls_lightweight_pose_endpoint_for_realtime_vehicle_markers():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert 'getJson("/api/pose")' in app_js
    assert "function pollPose()" in app_js
    assert "function updateLivePose(data)" in app_js
    assert "function updateNavigationPose(odom)" in app_js
    assert "function updateMappingPose(odom)" in app_js
    assert "setInterval(pollPose, 100)" in app_js

    navigation_start = app_js.index("function updateNavigationPose(odom)")
    navigation_block = app_js[
        navigation_start : app_js.index("function updateMappingPose", navigation_start)
    ]
    mapping_start = app_js.index("function updateMappingPose(odom)")
    mapping_block = app_js[
        mapping_start : app_js.index("function updateLivePose", mapping_start)
    ]
    assert "updateVehicleMarker();" in navigation_block
    assert "updateMappingVehicleMarker();" in mapping_block


def test_web_control_uses_compatible_qos_profiles_for_visualization_topics():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "static_pointcloud_qos = QoSProfile(" in web_control_node
    assert "durability=QoSDurabilityPolicy.TRANSIENT_LOCAL" in web_control_node
    assert "pointcloud_qos = QoSProfile(" in web_control_node
    assert "reliability=QoSReliabilityPolicy.BEST_EFFORT" in web_control_node
    assert "durability=QoSDurabilityPolicy.VOLATILE" in web_control_node
    assert "state_qos = QoSProfile(" in web_control_node
    assert "reliability=QoSReliabilityPolicy.RELIABLE" in web_control_node
    assert "PointCloud2," in web_control_node
    assert "static_pointcloud_qos," in web_control_node
    assert "pointcloud_qos," in web_control_node
    assert "Odometry," in web_control_node
    assert "PathMsg," in web_control_node
    assert "GoalStatusArray," in web_control_node
    assert "state_qos," in web_control_node


def test_3d_target_selection_uses_shared_map_coordinate_frame():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")

    assert 'id="targetLayer"' not in index_html
    assert 'id="selLayer"' not in index_html
    assert 'id="goalLayer"' not in index_html
    assert "Target layer" not in index_html
    assert "targetLayer" not in app_js
    assert "layer: state.targetLayer" not in app_js
    assert "LAYER_TARGET_STYLES" not in app_js
    assert "updateTargetMarkers()" in app_js


def test_mode_specific_3d_selection_controls_goal_z_handling():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")

    assert 'id="selZ"' in index_html
    assert "function pointCloudIntersection(event)" in app_js
    assert "LAYERS.mapground.object" in app_js
    assert "raycaster.intersectObject(LAYERS.mapground.object, false)" in app_js
    assert 'if (state.mode === "initial") return groundIntersection(event);' in app_js
    assert 'if (state.mode === "goal") return pointCloudIntersection(event);' in app_js
    assert "state.selection = { x: pose.x, y: pose.y, z: pose.z, yaw };" in app_js
    assert "function selectedPosePayload(includeZ)" in app_js
    assert "z: includeZ ? state.selection.z : 0" in app_js
    assert 'postJson("/api/initialpose", selectedPosePayload(false))' in app_js
    assert "const payload = selectedPosePayload(true);" in app_js
    assert "group.position.copy(rosToThree(pose.x, pose.y, (pose.z || 0) + 0.1));" in app_js


def test_selection_publish_button_follows_mode():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")

    assert 'id="publishPose"' in index_html
    assert "Publish Pose" in index_html
    assert "publishInitial" not in index_html
    assert "publishGoal" not in index_html
    assert "function updatePublishPoseButton()" in app_js
    assert 'button.disabled = state.mode === "view";' in app_js
    assert 'button.textContent = "Publish Pose";' in app_js
    assert "async function publishPose()" in app_js
    assert "const previousMode = state.mode;" in app_js
    assert "if (previousMode !== mode) {" in app_js
    assert 'if (state.mode === "initial") {' in app_js
    assert 'postJson("/api/initialpose", selectedPosePayload(false))' in app_js
    assert 'if (state.mode === "goal") {' in app_js
    assert 'postJson("/api/goal", payload)' in app_js
    assert 'document.getElementById("publishPose").addEventListener("click", publishPose);' in app_js
    assert "publishInitialPose" not in app_js
    assert "publishGoal()" not in app_js
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")
    assert "grid-template-columns: 1fr;" in styles_css
    assert ".actions button:disabled" in styles_css
    assert "cursor: not-allowed;" in styles_css


def test_3d_pose_markers_do_not_render_text_labels():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "function makeArrowMarker(pose, color)" in app_js
    assert "makeTextSprite" not in app_js
    assert "CanvasTexture" not in app_js
    assert "makeArrowMarker(state.selection, TARGET_MARKER_COLOR)" in app_js
    assert "makeArrowMarker(goal, TARGET_MARKER_COLOR)" in app_js
    assert 'makeArrowMarker({ ...state.odom.pose, layer: "mapcloud" }, 0x32d74b)' in app_js


def test_goal_arrival_clears_goal_marker_and_paths_until_new_goal():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")
    clear_start = app_js.index("function maybeClearGoalFromActionStatus()")
    clear_block = app_js[clear_start : app_js.index("function setMode", clear_start)]
    poll_start = app_js.index("async function pollPaths")
    poll_block = app_js[poll_start : app_js.index("function updateStatus", poll_start)]
    publish_start = app_js.index("async function publishPose()")
    publish_block = app_js[publish_start : app_js.index("async function stopManagedTarget", publish_start)]

    assert "hideCompletedPaths: false" in app_js
    assert "state.activeGoal = null;" in clear_block
    assert "state.lastGoal = null;" in clear_block
    assert "state.paths = {};" in clear_block
    assert "state.hideCompletedPaths = true;" in clear_block
    assert "updatePaths();" in clear_block
    assert "if (state.hideCompletedPaths) return;" in poll_block
    assert "state.hideCompletedPaths = false;" in publish_block


def test_3d_paths_use_tube_geometry_for_thicker_visible_tracks():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")
    update_start = app_js.index("function updatePaths()")
    update_block = app_js[update_start : app_js.index("async function pollState", update_start)]

    assert "new THREE.CatmullRomCurve3(points)" in update_block
    assert "new THREE.TubeGeometry(pathCurve, Math.max(2, points.length * 2), 0.16, 8, false)" in update_block
    assert "new THREE.MeshBasicMaterial" in update_block
    assert "new THREE.LineBasicMaterial" not in update_block


def test_navigation_readiness_owns_managed_process_toggles():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")

    readiness_start = index_html.index("<h2>Navigation Readiness</h2>")
    readiness_end = index_html.index("</section>", readiness_start)
    readiness = index_html[readiness_start:readiness_end]
    terminal_start = index_html.index("<h2>Terminal Logs</h2>")
    terminal_end = index_html.index("<h2>Layers</h2>", terminal_start)
    terminals = index_html[terminal_start:terminal_end]

    assert readiness.index('data-managed-target="localization"') < readiness.index(
        'data-managed-target="navigation"'
    )
    assert readiness.index('data-managed-target="navigation"') < readiness.index(
        'id="navigationReady"'
    )
    assert "Start Localization" in readiness
    assert "Start Navigation" in readiness
    assert 'id="localizationProcessToggle"' in readiness
    assert 'id="navigationProcessToggle"' in readiness
    assert 'data-start-target="localization"' not in terminals
    assert 'data-stop-target="localization"' not in terminals
    assert 'data-start-target="navigation"' not in terminals
    assert 'data-stop-target="navigation"' not in terminals
    assert 'data-start-target="mapping"' in index_html
    assert 'data-stop-target="mapping"' in index_html


def test_frontend_preserves_managed_endpoints_and_mapping_logs():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")

    assert "/api/managed/start" in app_js
    assert "/api/managed/stop" in app_js
    assert 'id="mappingLogState"' in index_html
    assert 'id="mappingLogs"' in index_html
    assert 'id="localizationLogState"' in index_html
    assert 'id="localizationLogs"' in index_html
    assert 'id="navigationLogState"' in index_html
    assert 'id="navigationLogs"' in index_html
    assert 'updateTerminalLog("mapping", data.logs && data.logs.mapping);' in app_js

    terminal_start = app_js.index("function updateTerminalLog(")
    terminal_end = app_js.index("function managedProcessRunning(", terminal_start)
    update_terminal = app_js[terminal_start:terminal_end]
    assert "updateManagedProcessButton(target);" in update_terminal


_MANAGED_PROCESS_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");

function jsFunctionBody(signature) {
  const start = source.indexOf(signature);
  if (start < 0) throw new Error("missing function: " + signature);
  const brace = source.indexOf("{", start);
  let depth = 0;
  for (let index = brace; index < source.length; index += 1) {
    if (source[index] === "{") depth += 1;
    else if (source[index] === "}") {
      depth -= 1;
      if (depth === 0) return source.slice(start, index + 1);
    }
  }
  throw new Error("unterminated function: " + signature);
}

function makeElement() {
  const classes = new Set();
  return {
    textContent: "",
    disabled: false,
    attributes: {},
    classList: {
      toggle(name, on) {
        if (on) classes.add(name);
        else classes.delete(name);
      },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
  };
}

const localizationButton = makeElement();
const navigationButton = makeElement();
const elements = {
  localizationProcessToggle: localizationButton,
  navigationProcessToggle: navigationButton,
};
global.document = {
  getElementById(id) { return elements[id] || null; },
};

const state = {
  localizationRunning: false,
  navigationRunning: false,
  managedProcessPending: { localization: null, navigation: null },
};

let mapSelectionLockCalls = 0;
let mapSelectionLockSawPending = null;
function updateMapSelectionLock() {
  mapSelectionLockCalls += 1;
  mapSelectionLockSawPending = state.managedProcessPending.navigation;
}

const startCalls = {};
const stopCalls = {};
let resolveNavigationStart = null;
let navigationStopResult = true;

function startManagedTarget(target) {
  startCalls[target] = (startCalls[target] || 0) + 1;
  if (target === "navigation") {
    return new Promise((resolve) => { resolveNavigationStart = resolve; });
  }
  return Promise.resolve(true);
}

function stopManagedTarget(target) {
  stopCalls[target] = (stopCalls[target] || 0) + 1;
  if (target === "navigation") return Promise.resolve(navigationStopResult);
  return Promise.resolve(true);
}

eval(jsFunctionBody("function managedProcessRunning")
  + "\n" + jsFunctionBody("function updateManagedProcessButton")
  + "\n" + jsFunctionBody("async function toggleManagedTarget"));

function expect(condition, message) { if (!condition) throw new Error(message); }

expect(source.includes("[data-managed-target]"), "toggle event selector missing");
expect(source.includes("[data-start-target]"), "mapping start selector missing");
expect(source.includes("[data-stop-target]"), "mapping stop selector missing");

(async () => {
  updateManagedProcessButton("localization");
  expect(localizationButton.textContent === "Start Localization", "initial localization label");
  expect(localizationButton.disabled === false, "initial localization enabled");
  expect(localizationButton.getAttribute("aria-pressed") === "false", "initial aria state");
  expect(localizationButton.getAttribute("aria-busy") === "false", "initial aria-busy");

  state.localizationRunning = true;
  updateManagedProcessButton("localization");
  expect(localizationButton.textContent === "Stop Localization", "running localization label");
  expect(localizationButton.classList.contains("stop-log"), "running stop style");
  state.localizationRunning = false;

  const pending = toggleManagedTarget("navigation");
  expect(navigationButton.disabled === true, "pending navigation disabled");
  expect(navigationButton.textContent === "Starting Navigation...", "pending navigation label");
  expect(navigationButton.getAttribute("aria-busy") === "true", "pending navigation aria-busy");
  toggleManagedTarget("navigation");
  expect(startCalls.navigation === 1, "duplicate navigation start blocked");
  expect(localizationButton.disabled === false, "localization remains independent");
  resolveNavigationStart(true);
  await pending;
  expect(state.navigationRunning === true, "navigation running after start success");
  expect(navigationButton.textContent === "Stop Navigation", "navigation stop label after start success");
  expect(navigationButton.getAttribute("aria-pressed") === "true", "navigation aria-pressed after start success");
  expect(navigationButton.getAttribute("aria-busy") === "false", "navigation aria-busy cleared after start success");
  expect(navigationButton.disabled === false, "navigation re-enabled after request");
  expect(mapSelectionLockCalls === 1, "map selection lock resumes on start success");
  expect(mapSelectionLockSawPending === "start", "map selection lock runs before pending clears");

  await toggleManagedTarget("navigation");
  expect(stopCalls.navigation === 1, "running navigation stops");
  expect(state.navigationRunning === false, "navigation stopped after stop success");
  expect(navigationButton.textContent === "Start Navigation", "navigation start label after stop success");
  expect(navigationButton.getAttribute("aria-pressed") === "false", "navigation aria-pressed after stop success");

  state.navigationRunning = true;
  navigationStopResult = false;
  await toggleManagedTarget("navigation");
  expect(state.navigationRunning === true, "failed stop retains running state");
  expect(navigationButton.textContent === "Stop Navigation", "failed stop retains stop label");
  expect(navigationButton.getAttribute("aria-pressed") === "true", "failed stop retains aria-pressed");
  console.log("managed process toggles ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""


def test_frontend_managed_process_toggles_follow_running_and_pending_state():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _MANAGED_PROCESS_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "managed process toggles ok" in completed.stdout


_FETCH_TIMEOUT_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");

function jsFunctionBody(signature) {
  const start = source.indexOf(signature);
  if (start < 0) throw new Error("missing function: " + signature);
  const brace = source.indexOf("{", start);
  let depth = 0;
  for (let index = brace; index < source.length; index += 1) {
    if (source[index] === "{") depth += 1;
    else if (source[index] === "}") {
      depth -= 1;
      if (depth === 0) return source.slice(start, index + 1);
    }
  }
  throw new Error("unterminated function: " + signature);
}

const FETCH_TIMEOUT_MS = 25;
let fetchCalls = 0;
global.fetch = (url, options) => {
  fetchCalls += 1;
  return new Promise((resolve, reject) => {
    const signal = options && options.signal;
    const fail = () => {
      const error = new Error("The operation was aborted.");
      error.name = "AbortError";
      reject(error);
    };
    if (signal && signal.aborted) { fail(); return; }
    if (signal && signal.addEventListener) signal.addEventListener("abort", fail, { once: true });
  });
};

eval(jsFunctionBody("async function fetchWithTimeout"));

function expect(condition, message) { if (!condition) throw new Error(message); }

expect(source.includes("AbortController"), "AbortController missing from source");
expect(source.includes('new Error("Request timed out.")'), "timeout error message missing");
expect(source.includes("fetchWithTimeout"), "shared timeout helper missing");

(async () => {
  const started = Date.now();
  let message = null;
  try {
    await fetchWithTimeout("/slow", {});
  } catch (error) {
    message = error.message;
  }
  expect(message === "Request timed out.", "timeout message was " + message);
  expect(fetchCalls === 1, "timeout helper issued one fetch");
  expect(Date.now() - started < 5000, "timeout helper did not resolve promptly");
  console.log("fetch timeout ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""


def test_frontend_fetch_helpers_share_a_finite_timeout():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")

    assert "const FETCH_TIMEOUT_MS = 15000;" in app_js
    assert "fetchWithTimeout" in app_js

    get_json_start = app_js.index("async function getJson(")
    get_json_end = app_js.index("async function postJson(", get_json_start)
    assert "fetchWithTimeout(" in app_js[get_json_start:get_json_end]

    post_json_start = get_json_end
    post_json_end = app_js.index("function viewerSessionId(", post_json_start)
    assert "fetchWithTimeout(" in app_js[post_json_start:post_json_end]


def test_frontend_fetch_timeout_rejects_hanging_request():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _FETCH_TIMEOUT_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "fetch timeout ok" in completed.stdout


def test_terminal_logs_section_is_above_layers_section():
    index_html = (
        Path(__file__).resolve().parents[1] / "static" / "index.html"
    ).read_text(encoding="utf-8")

    assert index_html.index("<h2>Terminal Logs</h2>") < index_html.index("<h2>Layers</h2>")


def test_terminal_logs_are_text_selectable():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert ".terminal pre" in styles_css
    assert "user-select: text;" in styles_css
    assert "-webkit-user-select: text;" in styles_css


def test_web_launch_does_not_expose_auto_start_arguments():
    launch_py = (
        Path(__file__).resolve().parents[1] / "launch" / "web_control.launch.py"
    ).read_text(encoding="utf-8")

    assert "start_localization" not in launch_py
    assert "start_navigation" not in launch_py


def test_web_launch_does_not_expose_or_inject_use_sim():
    launch_py = (
        Path(__file__).resolve().parents[1] / "launch" / "web_control.launch.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(launch_py)

    declared_arguments = set()
    configured_parameters = set()
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        function_name = getattr(call.func, "id", None) or getattr(
            call.func, "attr", ""
        )
        if function_name == "DeclareLaunchArgument" and call.args:
            argument = call.args[0]
            if isinstance(argument, ast.Constant) and isinstance(argument.value, str):
                declared_arguments.add(argument.value)
        if function_name == "Node":
            for keyword in call.keywords:
                if keyword.arg != "parameters":
                    continue
                for element in ast.walk(keyword.value):
                    if not isinstance(element, ast.Dict):
                        continue
                    for key in element.keys:
                        if isinstance(key, ast.Constant) and isinstance(key.value, str):
                            configured_parameters.add(key.value)

    # The config file argument must remain; use_sim must not be declared or
    # injected, so managed child launches keep their own default/USE_SIM_TIME.
    assert "config_file" in declared_arguments
    assert "use_sim" not in declared_arguments
    assert "use_sim" not in configured_parameters


def test_web_launch_loads_default_parameters_from_config_yaml():
    package_root = Path(__file__).resolve().parents[1]
    launch_py = (package_root / "launch" / "web_control.launch.py").read_text(
        encoding="utf-8"
    )
    setup_py = (package_root / "setup.py").read_text(encoding="utf-8")
    config_yaml = (package_root / "config" / "web_control.yaml").read_text(
        encoding="utf-8"
    )

    assert "FindPackageShare" in launch_py
    assert "PathJoinSubstitution" in launch_py
    assert "DeclareLaunchArgument(" in launch_py
    assert '"config_file"' in launch_py
    assert '"web_control.yaml"' in launch_py
    assert "config_file" in launch_py
    assert "glob(\"config/*.yaml\")" in setup_py
    assert "share/{package_name}/config" in setup_py

    assert "web_control:" in config_yaml
    assert "ros__parameters:" in config_yaml
    assert 'host: "0.0.0.0"                      # HTTP listen address' in config_yaml
    assert 'port: 8080                           # HTTP listen port' in config_yaml
    assert 'mapping_odom_topic: "/lio_sam/mapping/odometry"' in config_yaml
    assert 'localization_launch_package: "lio_sam_hesai"' in config_yaml
    assert 'localization_launch_file: "localization_with_nonground.launch.py"' in config_yaml
    assert 'navigation_launch_package: "p2p_move_base"' in config_yaml
    assert 'navigation_launch_file: "p2p_move_base_localization.launch.py"' in config_yaml
    assert 'mapping_launch_package: "lio_sam_hesai"' in config_yaml
    assert 'mapping_launch_file: "mapping.launch.py"' in config_yaml
    assert 'managed_log_lines: 400               # Terminal log lines kept per process' in config_yaml


def test_terminal_log_buffer_keeps_recent_lines_and_metadata():
    logs = TerminalLogBuffer("localization", max_lines=2)
    logs.append("first")
    logs.append("second\n")
    logs.append("third")
    logs.set_process_state(enabled=True, running=False, returncode=3)

    assert logs.snapshot() == {
        "target": "localization",
        "enabled": True,
        "running": False,
        "returncode": 3,
        "lines": ["second", "third"],
    }


def test_terminal_log_buffer_strips_ansi_escape_sequences():
    logs = TerminalLogBuffer("navigation", max_lines=3)
    logs.append("\x1b[0m[INFO] ready\x1b[0m")
    logs.append("\x1b[37;1mcolored\x1b[31;1m error\x1b[m")

    assert logs.snapshot()["lines"] == ["[INFO] ready", "colored error"]


def test_managed_launch_commands_are_fixed_ros_launch_invocations():
    assert build_managed_launch_command(
        "lio_sam_hesai",
        "localization_with_nonground.launch.py",
        "use_rviz:=false debug_relocalization:=false",
    ) == [
        "ros2",
        "launch",
        "lio_sam_hesai",
        "localization_with_nonground.launch.py",
        "use_rviz:=false",
        "debug_relocalization:=false",
    ]
    assert build_managed_launch_command(
        "lio_sam_hesai",
        "mapping.launch.py",
        "use_rviz:=false",
    ) == [
        "ros2",
        "launch",
        "lio_sam_hesai",
        "mapping.launch.py",
        "use_rviz:=false",
    ]


class _ParameterStub:
    def __init__(self, value):
        self.value = value


def _managed_launch_node(**overrides):
    """ROS-free WebControlNode stand-in that builds the managed commands."""

    class _LaunchNodeStub:
        def __init__(self):
            self.parameters = {
                "localization_launch_package": "lio_sam_hesai",
                "localization_launch_file": "localization_with_nonground.launch.py",
                "localization_launch_args": "",
                "navigation_launch_package": "p2p_move_base",
                "navigation_launch_file": "p2p_move_base_localization.launch.py",
                "navigation_launch_args": "",
                "mapping_launch_package": "lio_sam_hesai",
                "mapping_launch_file": "mapping.launch.py",
                "mapping_launch_args": "",
            }
            self.parameters.update(overrides)
            self.managed_logs = {
                target: TerminalLogBuffer(target, max_lines=5)
                for target in ("localization", "navigation", "mapping")
            }
            self.managed_processes = {}
            self.logger = _LoggerStub()
            self.configure_managed_processes = MethodType(
                web_control_node.WebControlNode.configure_managed_processes, self
            )
            self.handle_managed_exit = MethodType(
                web_control_node.WebControlNode.handle_managed_exit, self
            )

        def get_parameter(self, name):
            return _ParameterStub(self.parameters[name])

        def get_logger(self):
            return self.logger

    node = _LaunchNodeStub()
    node.configure_managed_processes()
    return node


def test_managed_launch_commands_add_no_implicit_use_sim():
    node = _managed_launch_node()
    commands = {
        target: process.command for target, process in node.managed_processes.items()
    }

    assert commands == {
        "localization": [
            "ros2",
            "launch",
            "lio_sam_hesai",
            "localization_with_nonground.launch.py",
        ],
        "navigation": [
            "ros2",
            "launch",
            "p2p_move_base",
            "p2p_move_base_localization.launch.py",
        ],
        "mapping": ["ros2", "launch", "lio_sam_hesai", "mapping.launch.py"],
    }
    assert not any(
        "use_sim" in argument
        for command in commands.values()
        for argument in command
    )


def test_managed_launch_commands_preserve_configured_args_verbatim():
    node = _managed_launch_node(
        localization_launch_args="use_rviz:=false debug_relocalization:=false",
        navigation_launch_args="use_sim:=true",
        mapping_launch_args="use_rviz:=false",
    )

    assert node.managed_processes["localization"].command[-2:] == [
        "use_rviz:=false",
        "debug_relocalization:=false",
    ]
    # An explicit use_sim in the configured args is user intent: keep it as-is.
    assert node.managed_processes["navigation"].command[-1] == "use_sim:=true"
    assert (
        node.managed_processes["navigation"].command.count("use_sim:=true") == 1
    )
    assert node.managed_processes["mapping"].command[-1] == "use_rviz:=false"


def test_web_control_manages_mapping_launch_and_clears_mapping_runtime_state():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert 'self.declare_parameter("mapping_launch_package", "lio_sam_hesai")' in web_control_node
    assert 'self.declare_parameter("mapping_launch_file", "mapping.launch.py")' in web_control_node
    assert 'self.declare_parameter("mapping_launch_args", "use_rviz:=false")' in web_control_node
    assert '"mapping": TerminalLogBuffer(' in web_control_node
    assert "mapping_command = build_managed_launch_command(" in web_control_node
    assert 'str(self.get_parameter("mapping_launch_package").value)' in web_control_node
    assert 'str(self.get_parameter("mapping_launch_file").value)' in web_control_node
    assert 'self.managed_processes["mapping"] = ManagedLaunchProcess(' in web_control_node
    assert "def clear_mapping_runtime_state(self)" in web_control_node
    assert "for cache in self.mapping_layers.values()" in web_control_node
    assert "self.mapping_odom.clear()" in web_control_node
    assert "self.mapping_path.clear()" in web_control_node
    assert 'elif target == "mapping":' in web_control_node
    assert "self.clear_mapping_runtime_state()" in web_control_node


def test_managed_launch_stop_signals_saved_group_after_leader_exits(monkeypatch):
    from web_control.web_control_node import ManagedLaunchProcess

    class Logger:
        def error(self, _message):
            pass

    class ExitedProcess:
        pid = 123
        returncode = 0

        def poll(self):
            return 0

        def wait(self, timeout=None):
            return 0

    calls = []
    monkeypatch.setattr("os.killpg", lambda pgid, sig: calls.append((pgid, sig)))

    managed = ManagedLaunchProcess(
        "navigation",
        ["ros2", "launch", "p2p_move_base", "p2p_move_base_localization.launch.py"],
        TerminalLogBuffer("navigation"),
        Logger(),
    )
    managed.process = ExitedProcess()
    managed.pgid = 456

    managed.stop()

    assert calls == [(456, signal.SIGINT)]


def test_managed_launch_stop_uses_short_shutdown_timeouts(monkeypatch):
    from web_control.web_control_node import ManagedLaunchProcess

    class Logger:
        def error(self, _message):
            pass

    class StubbornProcess:
        pid = 123
        returncode = None

        def __init__(self):
            self.wait_timeouts = []
            self.killed = False

        def poll(self):
            return None

        def wait(self, timeout=None):
            self.wait_timeouts.append(timeout)
            raise subprocess.TimeoutExpired("managed", timeout)

        def kill(self):
            self.killed = True

        def send_signal(self, _sig):
            pass

    monkeypatch.setattr("os.killpg", lambda _pgid, _sig: None)
    process = StubbornProcess()
    managed = ManagedLaunchProcess(
        "navigation",
        ["ros2", "launch", "p2p_move_base", "p2p_move_base_localization.launch.py"],
        TerminalLogBuffer("navigation"),
        Logger(),
    )
    managed.process = process
    managed.pgid = 456

    managed.stop()

    assert process.wait_timeouts == [1.5, 0.75, 0.25]
    assert process.killed is True


def test_managed_launch_stop_signals_descendant_processes(monkeypatch):
    from web_control.web_control_node import ManagedLaunchProcess

    class Logger:
        def error(self, _message):
            pass

    class ExitedProcess:
        pid = 123
        returncode = 0

        def poll(self):
            return 0

        def wait(self, timeout=None):
            return 0

    group_calls = []
    pid_calls = []
    monkeypatch.setattr("os.killpg", lambda pgid, sig: group_calls.append((pgid, sig)))
    monkeypatch.setattr("os.kill", lambda pid, sig: pid_calls.append((pid, sig)))

    managed = ManagedLaunchProcess(
        "navigation",
        ["ros2", "launch", "p2p_move_base", "p2p_move_base_localization.launch.py"],
        TerminalLogBuffer("navigation"),
        Logger(),
    )
    managed.process = ExitedProcess()
    managed.pgid = 456
    managed.descendant_pids = {777, 888}

    managed.stop()

    assert group_calls == [(456, signal.SIGINT)]
    assert pid_calls == [(777, signal.SIGINT), (888, signal.SIGINT)]


def test_web_control_shutdown_uses_destroy_node_for_managed_launch_cleanup():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "signal.signal(signal.SIGINT" not in web_control_node
    assert "signal.signal(signal.SIGTERM" not in web_control_node
    assert "def destroy_node(self) -> bool:" in web_control_node
    assert "self.stop_managed_processes()" in web_control_node
    assert "self.stop_http_server()" in web_control_node


def test_web_control_stops_managed_launches_concurrently_on_shutdown():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert 'name=f"managed_launch_stop_' in web_control_node
    assert "threading.Thread(" in web_control_node
    assert "thread.join(timeout=" in web_control_node


def test_managed_launch_exit_clears_runtime_map_sources():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "def clear_map_runtime_state(self)" in web_control_node
    assert "for cache in self.layers.values()" in web_control_node
    assert "self.odom.clear()" in web_control_node
    assert "self.action_status.clear()" in web_control_node
    assert "def handle_managed_exit(self, target: str)" in web_control_node
    assert 'if target == "localization":' in web_control_node
    assert "self.clear_map_runtime_state()" in web_control_node
    assert "on_exit=self.handle_managed_exit" in web_control_node


def test_parse_proc_stat_cpu_ticks_reads_total_and_cores():
    stat_text = "\n".join(
        [
            "cpu  100 0 50 850 0 0 0 0 0 0",
            "cpu0 40 0 10 450 0 0 0 0 0 0",
            "cpu1 60 0 40 400 0 0 0 0 0 0",
            "intr 1 2 3",
        ]
    )

    ticks = parse_proc_stat_cpu_ticks(stat_text)

    assert ticks["total"] == (100, 0, 50, 850, 0, 0, 0, 0, 0, 0)
    assert ticks["cores"] == [
        (40, 0, 10, 450, 0, 0, 0, 0, 0, 0),
        (60, 0, 40, 400, 0, 0, 0, 0, 0, 0),
    ]


def test_calculate_cpu_usage_returns_percent_from_tick_delta():
    previous = {
        "total": (100, 0, 50, 850, 0, 0, 0, 0, 0, 0),
        "cores": [
            (40, 0, 10, 450, 0, 0, 0, 0, 0, 0),
            (60, 0, 40, 400, 0, 0, 0, 0, 0, 0),
        ],
    }
    current = {
        "total": (130, 0, 70, 900, 0, 0, 0, 0, 0, 0),
        "cores": [
            (50, 0, 15, 485, 0, 0, 0, 0, 0, 0),
            (80, 0, 55, 415, 0, 0, 0, 0, 0, 0),
        ],
    }

    usage = calculate_cpu_usage(previous, current)

    assert usage == {
        "total_percent": 50.0,
        "cores": [
            {"id": 0, "percent": 30.0},
            {"id": 1, "percent": 70.0},
        ],
    }


def test_parse_proc_meminfo_reports_memory_and_swap_usage():
    meminfo = "\n".join(
        [
            "MemTotal:        8000000 kB",
            "MemFree:         1000000 kB",
            "MemAvailable:    2500000 kB",
            "SwapTotal:       2000000 kB",
            "SwapFree:        1500000 kB",
        ]
    )

    usage = parse_proc_meminfo(meminfo)

    assert usage["memory"] == {
        "total_kb": 8000000,
        "used_kb": 5500000,
        "percent": 68.75,
    }
    assert usage["swap"] == {
        "total_kb": 2000000,
        "used_kb": 500000,
        "percent": 25.0,
    }


def test_build_system_usage_payload_is_lightweight_and_handles_first_cpu_sample():
    previous = parse_proc_stat_cpu_ticks("cpu  100 0 0 900\ncpu0 100 0 0 900")
    current = parse_proc_stat_cpu_ticks("cpu  150 0 0 950\ncpu0 150 0 0 950")
    meminfo = "MemTotal: 1000 kB\nMemAvailable: 400 kB\nSwapTotal: 0 kB\nSwapFree: 0 kB\n"

    first_payload, first_ticks = build_system_usage_payload(
        "cpu  100 0 0 900\ncpu0 100 0 0 900",
        meminfo,
        previous_ticks=None,
        stamp=12.0,
    )
    second_payload, second_ticks = build_system_usage_payload(
        "cpu  150 0 0 950\ncpu0 150 0 0 950",
        meminfo,
        previous_ticks=previous,
        stamp=13.0,
    )

    assert first_payload["cpu"] == {
        "total_percent": None,
        "cores": [{"id": 0, "percent": None}],
    }
    assert first_ticks == previous
    assert second_payload["cpu"] == {
        "total_percent": 50.0,
        "cores": [{"id": 0, "percent": 50.0}],
    }
    assert second_ticks == current


def test_web_control_node_imports_system_usage_payload_builder():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "build_system_usage_payload" in web_control_node
    assert "self.system_cpu_ticks" in web_control_node


def test_web_control_get_router_exposes_system_endpoint():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert 'parsed.path == "/api/system"' in web_control_node
    assert "node.system_payload()" in web_control_node


def test_http_handler_ignores_client_disconnects_without_writing_error_response():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "CLIENT_DISCONNECT_ERRORS" in web_control_node
    assert "except CLIENT_DISCONNECT_ERRORS:" in web_control_node
    assert "return" in web_control_node
    assert "BrokenPipeError" in web_control_node
    assert "ConnectionResetError" in web_control_node


def test_frontend_exposes_system_tab_and_metric_targets():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")

    assert 'id="tabSystem"' in index_html
    assert 'id="viewSystem"' in index_html
    assert 'id="systemCpuTotal"' in index_html
    assert 'id="systemMemory"' in index_html
    assert 'id="systemSwap"' in index_html
    assert 'id="systemCpuCores"' in index_html


def test_frontend_exposes_current_viewer_count():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert 'id="viewerCount"' in index_html
    assert "function viewerSessionId()" in app_js
    assert 'postJson("/api/viewer", { viewer_id: viewerSessionId() })' in app_js
    assert "function updateViewerCount(count)" in app_js
    assert "updateViewerCount(data.viewers)" in app_js
    assert "setInterval(heartbeatViewer, 3000)" in app_js
    assert ".header-status" in styles_css


def test_web_control_exposes_viewer_heartbeat_endpoint_and_state_count():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "self.viewers = ViewerRegistry" in web_control_node
    assert '"viewers": self.viewers.count(now)' in web_control_node
    assert "def viewer_heartbeat(self, payload: dict[str, Any])" in web_control_node
    assert 'elif self.path == "/api/viewer":' in web_control_node


def test_left_drag_rotates_3d_view_in_natural_follow_direction():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "state.camera.yaw += dx * 0.006;" in app_js
    assert "state.camera.pitch = Math.max(0.18, Math.min(1.38, state.camera.pitch + dy * 0.006));" in app_js
    assert "state.camera.yaw -= dx * 0.006;" not in app_js
    assert "state.camera.pitch - dy * 0.006" not in app_js


def test_3d_map_supports_touch_pointer_interaction():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "touches: new Map()" in app_js
    assert "function touchGesture()" in app_js
    assert "function handleTouchPanZoom(event)" in app_js
    assert "state.pointer.touches.set(event.pointerId, canvasPoint(event));" in app_js
    assert "state.pointer.touches.delete(event.pointerId);" in app_js
    assert "if (event.pointerType === \"touch\" && state.pointer.touches.size >= 2)" in app_js
    assert "state.camera.distance = Math.max(5, Math.min(280, state.pointer.touchStart.cameraDistance * zoomRatio));" in app_js
    assert "state.camera.target.addScaledVector(right, -dx * scale);" in app_js
    assert "mapCanvas3d.addEventListener(\"pointerleave\", endPointer);" in app_js


def test_frontend_polls_system_api_only_for_system_tab():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert 'getJson("/api/system")' in app_js
    assert 'if (state.view !== "system") return;' in app_js
    assert 'setInterval(pollSystem, 1000)' in app_js
    assert 'setTab("system")' in app_js


def test_system_tab_hides_sidebar_and_expands_stage():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    set_tab_start = app_js.index("function setTab(tab)")
    set_tab_block = app_js[set_tab_start : app_js.index("function selectedPosePayload", set_tab_start)]

    assert 'document.querySelector(".app").classList.toggle("system-mode", tab === "system");' in set_tab_block
    assert 'const panelName = tab === "system" ? "" : (tab === "mapping" ? "mapping" : "navigation");' in set_tab_block
    assert "panel.dataset.panel === panelName" in set_tab_block
    assert ".app.system-mode" in styles_css
    assert ".app.system-mode .panel" in styles_css
    system_panel_start = styles_css.index(".app.system-mode .panel")
    system_panel_block = styles_css[system_panel_start : styles_css.index("}", system_panel_start)]
    assert "display: none;" in system_panel_block


def test_cpu_core_rows_are_reused_so_bars_slide_to_targets():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "function updateCpuCores(cores)" in app_js
    assert 'list.querySelector(`[data-core-id="${core.id}"]`)' in app_js
    assert "row.dataset.coreId = String(core.id);" in app_js
    assert 'list.innerHTML = "";' not in app_js


def test_frontend_clears_3d_scene_when_map_sources_reset():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "function clearMapSceneState()" in app_js
    assert "function mapSourcesAreWaiting(layers)" in app_js
    assert "if (mapSourcesAreWaiting(data.layers || {}))" in app_js
    assert "clearMapSceneState();" in app_js
    assert "Object.keys(LAYERS).forEach((name) =>" in app_js
    assert "state.paths = {};" in app_js
    assert "state.odom = null;" in app_js
    assert 'if (target === "all" || target === "localization")' in app_js


def test_frontend_styles_cpu_core_horizontal_bars():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert ".cpu-core-row" in styles_css
    assert ".cpu-core-fill" in styles_css
    assert "width: var(--cpu-percent" in styles_css


def test_system_tab_uses_refined_dashboard_elements():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert "system-card cpu-card" in index_html
    assert "system-card memory-card" in index_html
    assert "system-card swap-card" in index_html
    assert "system-meter-fill" in index_html
    assert "system-card-status" in index_html
    assert "usageLevel(" in app_js
    assert 'row.className = `cpu-core-row load-${usageLevel(core.percent)}`;' in app_js
    assert ".system-card::before" in styles_css
    assert "transition: width 420ms ease" in styles_css
    assert "@keyframes meterGlow" in styles_css


def test_frontend_uses_ios_inspired_visual_theme():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--ios-blue: #0a84ff" in styles_css
    assert "--glass:" in styles_css
    assert "--radius-lg: 22px" in styles_css
    assert "backdrop-filter: blur(22px) saturate(1.35)" in styles_css
    assert ".panel section" in styles_css
    assert "border-radius: var(--radius-md)" in styles_css
    assert ".tab.active" in styles_css
    assert ".hud" in styles_css


def test_mode_control_uses_ios_sliding_segmented_selector():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert 'id="modeSegment"' in index_html
    assert 'data-active="view"' in index_html
    assert 'document.getElementById("modeSegment").dataset.active = mode;' in app_js
    assert ".segmented::before" in styles_css
    assert '.segmented[data-active="initial"] {' in styles_css
    assert '.segmented[data-active="goal"] {' in styles_css
    assert "--mode-index: 2;" in styles_css


def test_frontend_uses_immersive_glass_light_environment():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "body::before" in styles_css
    assert "body::after" in styles_css
    assert "@keyframes ambientDrift" in styles_css
    assert "@keyframes sheenSweep" in styles_css
    assert "mix-blend-mode: screen" in styles_css
    assert "background-attachment: fixed" in styles_css
    assert ".panel section::before" in styles_css
    assert ".system-card:hover" in styles_css
    assert "transform: translateY(-2px)" in styles_css


def test_frontend_applies_apple_liquid_glass_guidance():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--liquid-glass-tint" in styles_css
    assert "--liquid-glass-highlight" in styles_css
    assert ".tabs::before" in styles_css
    assert ".panel section::after" in styles_css
    assert ".terminal::before" in styles_css
    assert ".source::before" in styles_css
    assert "transition: opacity 260ms ease, transform 420ms ease" in styles_css
    assert "cubic-bezier(0.22, 1, 0.36, 1)" in styles_css


def test_top_tabs_use_sliding_segmented_selector():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert 'id="viewTabs"' in index_html
    assert 'class="tabs sliding-tabs"' in index_html
    assert 'data-active="3d"' in index_html
    assert 'document.getElementById("viewTabs").dataset.active = tab;' in app_js
    assert ".sliding-tabs::before" in styles_css
    assert '.sliding-tabs[data-active="system"] {' in styles_css
    assert "--tabs-index: 1;" in styles_css


def test_sliding_selectors_use_transform_based_ios_spring_motion():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--ios-spring: cubic-bezier(0.2, 0.85, 0.25, 1.08)" in styles_css
    assert "will-change: transform, box-shadow, filter;" in styles_css
    assert "transform: translate3d(calc(var(--tabs-index, 0) * 100%), 0, 0)" in styles_css
    assert "transform: translate3d(calc(var(--mode-index, 0) * 100%), 0, 0)" in styles_css
    assert "transition: transform 430ms var(--ios-spring)" in styles_css
    assert '.sliding-tabs[data-active="system"] {' in styles_css
    assert '.segmented[data-active="goal"] {' in styles_css
    assert "transition: left 280ms" not in styles_css
    assert "transform: translateX(100%)" not in styles_css
    assert "transform: translateX(200%)" not in styles_css


def test_sliding_selectors_use_ios_26_liquid_glass_material():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--liquid-slider-fill" in styles_css
    assert "--liquid-slider-rim" in styles_css
    assert "--liquid-slider-caustic" in styles_css
    assert "--liquid-slider-noise" in styles_css
    assert "data:image/svg+xml" in styles_css
    assert "feTurbulence" in styles_css
    assert "backdrop-filter: blur(18px) saturate(1.9) brightness(1.08);" in styles_css
    assert "-webkit-backdrop-filter: blur(18px) saturate(1.9) brightness(1.08);" in styles_css
    assert "var(--liquid-slider-noise)" in styles_css
    assert "background-blend-mode: soft-light" in styles_css
    assert "background-size: 80px 80px" in styles_css
    assert "radial-gradient(circle at 18% 0%, var(--liquid-slider-caustic), transparent 42%)" in styles_css
    assert "inset 0 1px 0 rgba(255, 255, 255, 0.72)" in styles_css
    assert "filter: drop-shadow(0 10px 24px rgba(10, 132, 255, 0.18));" in styles_css


def test_top_tab_slider_uses_equal_width_segments():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "grid-template-columns: repeat(3, minmax(96px, 1fr))" in styles_css
    assert "grid-template-columns: repeat(3, max-content)" not in styles_css
    assert ".sliding-tabs .tab" in styles_css
    assert "justify-content: center" in styles_css


def test_top_tab_hover_does_not_move_or_draw_extra_blue_button():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert ".sliding-tabs .tab:hover" in styles_css
    assert "transform: none;" in styles_css
    assert ".sliding-tabs .tab.active" in styles_css
    assert "min-width: 0;" in styles_css
    assert ".tab::after" not in styles_css
    assert ".tab:hover::after" not in styles_css


def test_mode_segment_hover_does_not_draw_offset_blue_button():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert ".segmented .mode:hover," in styles_css
    assert ".segmented .mode.active" in styles_css
    assert "background: transparent;" in styles_css[
        styles_css.index(".segmented .mode:hover,") : styles_css.index("label {")
    ]
    assert "transform: none;" in styles_css[
        styles_css.index(".segmented .mode:hover,") : styles_css.index("label {")
    ]


def test_mode_track_is_visible_in_light_theme():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--mode-track-bg: rgba(10, 132, 255, 0.1);" in styles_css
    assert "--mode-track-border: rgba(10, 132, 255, 0.22);" in styles_css
    assert "--mode-track-glow: rgba(10, 132, 255, 0.16);" in styles_css
    assert "border: 1px solid var(--mode-track-border);" in styles_css
    assert "background: linear-gradient(180deg, rgba(255, 255, 255, 0.48), var(--mode-track-bg));" in styles_css
    assert "0 0 0 1px var(--mode-track-glow)" in styles_css
    assert "--mode-track-bg: rgba(255, 255, 255, 0.06);" in styles_css
    assert "--mode-slider-fill" not in styles_css
    assert "border: 1px solid var(--liquid-slider-rim);" in styles_css


def test_mode_slider_uses_contained_spring_to_stay_inside_track():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--ios-contained-spring: cubic-bezier(0.2, 0.85, 0.25, 1);" in styles_css
    segmented_before = styles_css[
        styles_css.index(".segmented::before") : styles_css.index(".segmented[data-active=\"initial\"]")
    ]
    assert "transition: transform 430ms var(--ios-contained-spring)" in segmented_before
    assert "var(--ios-spring)" not in segmented_before


def test_mode_slider_is_clipped_and_mode_text_is_centered():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    segmented_block = styles_css[
        styles_css.index(".segmented {") : styles_css.index(".segmented::before")
    ]
    mode_start = styles_css.index(".mode {")
    mode_block = styles_css[mode_start : styles_css.index(".mode.active", mode_start)]

    assert "overflow: hidden;" in segmented_block
    assert "display: flex;" in mode_block
    assert "align-items: center;" in mode_block
    assert "justify-content: center;" in mode_block
    assert "text-align: center;" in mode_block
    assert "line-height: 1.1;" in mode_block


def test_frontend_follows_system_light_and_dark_color_scheme():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "color-scheme: light dark" in styles_css
    assert "@media (prefers-color-scheme: dark)" in styles_css
    assert "--bg: #f5f7fb" in styles_css
    assert "--text: #111827" in styles_css
    assert "--glass: rgba(255, 255, 255, 0.62)" in styles_css
    assert "--bg: #07080d" in styles_css
    assert "body::before" in styles_css
    assert ".system-viewport" in styles_css


def test_mobile_layout_keeps_control_panel_as_side_panel():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "min-width: 760px;" in styles_css
    assert "@media (max-width: 900px)" in styles_css
    assert "grid-template-columns: minmax(420px, 1fr) 340px;" in styles_css
    assert "grid-template-rows: 1fr;" in styles_css
    assert "overflow-x: auto;" in styles_css
    mobile_app_block_start = styles_css.index("@media (max-width: 900px)")
    mobile_app_block_end = styles_css.index(".panel", mobile_app_block_start)
    assert "grid-template-columns: 1fr;" not in styles_css[mobile_app_block_start:mobile_app_block_end]
    assert "grid-template-rows: 52vh 48vh;" not in styles_css


class _LoggerStub:
    def debug(self, *_args, **_kwargs):
        pass

    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class _FakeFuture:
    def __init__(self, response=None, error=None):
        self.response = response
        self.error = error
        self.done_flag = False
        self.callbacks = []
        self.cancelled = False

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def done(self):
        return self.done_flag

    def cancel(self):
        self.cancelled = True
        self.done_flag = True

    def resolve(self):
        self.done_flag = True
        for callback in list(self.callbacks):
            callback(self)

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response


class _FakeSaveMapRequest:
    def __init__(self):
        self.destination = ""
        self.resolution = 0.0
        self.ground_resolution = 0.0
        self.save_ground_map = False
        self.save_tiled_map = False


class _FakeSaveMapOptions:
    Request = _FakeSaveMapRequest


class _FakeSaveMapResponse:
    def __init__(self, success, message):
        self.success = success
        self.message = message


class _FakeSaveMapClient:
    def __init__(self, ready=True, future=None):
        self.ready = ready
        self.future = future if future is not None else _FakeFuture()
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return self.future


class _FakeWebNode:
    """ROS-free node stand-in for the save map backend methods."""

    def __init__(self, client):
        self.save_map_client = client
        self.save_map_job = core.SaveMapJob()
        self.save_map_future = None
        self.static_dir = Path(__file__).resolve().parents[1] / "static"
        self.logger = _LoggerStub()
        self.destroyed_clients = []
        self.clock = 10.0
        for name in (
            "start_save_map",
            "save_map_status_payload",
            "save_map_available",
            "handle_save_map_result",
            "cancel_save_map_future",
            "cleanup_save_map",
        ):
            setattr(
                self,
                name,
                MethodType(getattr(web_control_node.WebControlNode, name), self),
            )

    def now_sec(self):
        self.clock += 0.25
        return self.clock

    def get_logger(self):
        return self.logger

    def destroy_client(self, client):
        self.destroyed_clients.append(client)


def _serve_fake_web_node(node):
    handler = web_control_node.WebControlNode.make_handler(node)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def _json_request(url, payload=None, method="GET"):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        request.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def test_resolve_save_map_destination_maps_web_paths_under_home():
    assert core.resolve_save_map_destination("/map", "/home/robot") == "/home/robot/map"
    assert (
        core.resolve_save_map_destination("/map/site", "/home/robot")
        == "/home/robot/map/site"
    )
    assert (
        core.resolve_save_map_destination(" /map/site/run_01/ ", "/home/robot")
        == "/home/robot/map/site/run_01"
    )


def test_resolve_save_map_destination_rejects_empty_root_and_relative_paths():
    for destination in ("", "   ", "/", "map", "map/site", 7, None):
        with pytest.raises(ValueError, match="destination"):
            core.resolve_save_map_destination(destination, "/home/robot")


def test_resolve_save_map_destination_rejects_parent_directory_escape():
    for destination in (
        "/..",
        "/../etc",
        "/map/..",
        "/map/../etc",
        "/map/site/../../etc",
    ):
        with pytest.raises(ValueError, match=r"\.\."):
            core.resolve_save_map_destination(destination, "/home/robot")


def test_resolve_save_map_destination_rejects_home_prefixed_paths():
    for destination in (
        "/home/robot",
        "/home/robot/",
        "/home/robot/map",
        "/home/robot/map/site",
    ):
        with pytest.raises(ValueError, match="home"):
            core.resolve_save_map_destination(destination, "/home/robot")

    # A sibling directory merely sharing the home prefix is not home-prefixed.
    assert (
        core.resolve_save_map_destination("/home/robot2/map", "/home/robot")
        == "/home/robot/home/robot2/map"
    )


def test_validate_save_map_payload_resolves_destination_under_runtime_home(monkeypatch):
    monkeypatch.setenv("HOME", "/tmp/web-control-home")

    payload = core.validate_save_map_payload({"destination": "/map/site"})

    assert payload["destination"] == "/tmp/web-control-home/map/site"


def test_validate_save_map_payload_fills_service_defaults():
    payload = core.validate_save_map_payload(
        {"destination": " /map/run_01 "}, home="/home/robot"
    )

    assert payload == {
        "destination": "/home/robot/map/run_01",
        "resolution": -1.0,
        "ground_resolution": -1.0,
        "save_ground_map": True,
        "save_tiled_map": True,
    }


def test_validate_save_map_payload_accepts_explicit_options():
    payload = core.validate_save_map_payload(
        {
            "destination": "/map/run_02",
            "resolution": 0.05,
            "ground_resolution": "0.2",
            "save_ground_map": False,
            "save_tiled_map": True,
        },
        home="/home/robot",
    )

    assert payload == {
        "destination": "/home/robot/map/run_02",
        "resolution": 0.05,
        "ground_resolution": 0.2,
        "save_ground_map": False,
        "save_tiled_map": True,
    }


def test_validate_save_map_payload_requires_home_relative_destination():
    for destination in (None, "", "   ", 7, "map/run", "/"):
        with pytest.raises(ValueError, match="destination"):
            core.validate_save_map_payload(
                {"destination": destination}, home="/home/robot"
            )

    with pytest.raises(ValueError, match=r"\.\."):
        core.validate_save_map_payload(
            {"destination": "/map/../etc"}, home="/home/robot"
        )

    with pytest.raises(ValueError, match="home"):
        core.validate_save_map_payload(
            {"destination": "/home/robot/map"}, home="/home/robot"
        )

    with pytest.raises(ValueError, match="object"):
        core.validate_save_map_payload([])


def test_validate_save_map_payload_rejects_non_boolean_toggles():
    for key, value in (
        ("save_ground_map", 1),
        ("save_ground_map", None),
        ("save_tiled_map", "true"),
    ):
        payload = {"destination": "/map/run"}
        payload[key] = value
        try:
            core.validate_save_map_payload(payload)
        except ValueError as exc:
            assert key in str(exc)
            assert "boolean" in str(exc)
        else:
            raise AssertionError(f"{key}={value!r} was accepted")


def test_validate_save_map_payload_rejects_non_numeric_resolutions():
    for key, value in (("resolution", "wide"), ("ground_resolution", float("nan"))):
        payload = {"destination": "/map/run", key: value}
        try:
            core.validate_save_map_payload(payload)
        except ValueError as exc:
            assert key in str(exc)
        else:
            raise AssertionError(f"{key}={value!r} was accepted")


def test_save_map_job_guards_duplicate_runs_and_records_result():
    job = core.SaveMapJob()

    assert job.snapshot()["state"] == "idle"
    assert job.snapshot()["running"] is False

    token = job.begin("/map/run", now=1.0)
    assert token is not None
    assert job.begin("/map/other", now=1.5) is None

    running = job.snapshot()
    assert running["state"] == "running"
    assert running["running"] is True
    assert running["success"] is None
    assert running["destination"] == "/map/run"
    assert running["started_at"] == 1.0
    assert running["finished_at"] is None

    assert job.finish(token, True, "saved to /map/run", now=2.0) is True

    done = job.snapshot()
    assert done["state"] == "succeeded"
    assert done["running"] is False
    assert done["success"] is True
    assert done["message"] == "saved to /map/run"
    assert done["finished_at"] == 2.0

    assert job.begin("/map/next", now=3.0) is not None


def test_save_map_job_ignores_stale_callbacks_after_abandon():
    job = core.SaveMapJob()
    token = job.begin("/map/run", now=1.0)

    assert job.abandon_running("save map service unavailable", now=2.0) is True
    assert job.abandon_running("nothing running", now=2.5) is False
    failed = job.snapshot()
    assert failed["state"] == "failed"
    assert failed["success"] is False
    assert failed["message"] == "save map service unavailable"

    assert job.finish(token, True, "late success", now=3.0) is False
    assert job.snapshot()["state"] == "failed"

    next_token = job.begin("/map/retry", now=4.0)
    assert next_token is not None
    assert job.finish(token, True, "stale token", now=5.0) is False
    assert job.snapshot()["state"] == "running"
    assert job.finish(next_token, False, "export failed", now=6.0) is True
    assert job.snapshot()["state"] == "failed"


def test_start_save_map_sends_async_request_and_returns_running_state(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    monkeypatch.setenv("HOME", "/home/robot")
    future = _FakeFuture()
    client = _FakeSaveMapClient(ready=True, future=future)
    node = _FakeWebNode(client)

    response = node.start_save_map(
        {
            "destination": "/map/hesai_sim",
            "resolution": 0.05,
            "ground_resolution": 0.1,
            "save_ground_map": False,
            "save_tiled_map": True,
        }
    )

    assert response["ok"] is True
    assert response["state"] == "running"
    assert response["available"] is True
    assert response["destination"] == "/home/robot/map/hesai_sim"
    assert len(client.requests) == 1
    request = client.requests[0]
    assert request.destination == "/home/robot/map/hesai_sim"
    assert request.resolution == 0.05
    assert request.ground_resolution == 0.1
    assert request.save_ground_map is False
    assert request.save_tiled_map is True
    assert future.callbacks
    assert node.save_map_future is future

    future.response = _FakeSaveMapResponse(True, "saved to /home/robot/map/hesai_sim")
    future.resolve()

    assert node.save_map_future is None
    snapshot = node.save_map_job.snapshot()
    assert snapshot["state"] == "succeeded"
    assert snapshot["destination"] == "/home/robot/map/hesai_sim"
    assert snapshot["message"] == "saved to /home/robot/map/hesai_sim"


def test_start_save_map_rejects_escaping_destination_before_starting_job(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    monkeypatch.setenv("HOME", "/home/robot")
    client = _FakeSaveMapClient(ready=True)
    node = _FakeWebNode(client)

    with pytest.raises(ValueError, match=r"\.\."):
        node.start_save_map({"destination": "/map/../etc"})

    assert client.requests == []
    assert node.save_map_job.snapshot()["state"] == "idle"


def test_start_save_map_rejects_duplicate_while_running(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    monkeypatch.setenv("HOME", "/home/robot")
    client = _FakeSaveMapClient(ready=True)
    node = _FakeWebNode(client)
    node.start_save_map({"destination": "/map/run"})

    try:
        node.start_save_map({"destination": "/map/run_again"})
    except core.SaveMapBusyError as exc:
        assert "already" in str(exc)
    else:
        raise AssertionError("duplicate save map request was accepted")

    assert len(client.requests) == 1
    assert node.save_map_job.snapshot()["destination"] == "/home/robot/map/run"


def test_start_save_map_reports_unavailable_service(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    client = _FakeSaveMapClient(ready=False)
    node = _FakeWebNode(client)

    try:
        node.start_save_map({"destination": "/map/run"})
    except core.SaveMapUnavailableError as exc:
        assert "unavailable" in str(exc)
    else:
        raise AssertionError("missing service was accepted")

    assert client.requests == []
    assert node.save_map_job.snapshot()["state"] == "idle"


def test_handle_save_map_result_records_service_failure_response(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    future = _FakeFuture()
    node = _FakeWebNode(_FakeSaveMapClient(ready=True, future=future))
    node.start_save_map({"destination": "/map/run"})

    future.response = _FakeSaveMapResponse(False, "map export failed: disk full")
    future.resolve()

    snapshot = node.save_map_job.snapshot()
    assert snapshot["state"] == "failed"
    assert snapshot["success"] is False
    assert "disk full" in snapshot["message"]


def test_handle_save_map_result_records_service_call_error(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    future = _FakeFuture()
    node = _FakeWebNode(_FakeSaveMapClient(ready=True, future=future))
    node.start_save_map({"destination": "/map/run"})

    future.error = RuntimeError("service vanished")
    future.resolve()

    snapshot = node.save_map_job.snapshot()
    assert snapshot["state"] == "failed"
    assert "service vanished" in snapshot["message"]


def test_save_map_status_fails_stale_job_when_service_disappears(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    future = _FakeFuture()
    client = _FakeSaveMapClient(ready=True, future=future)
    node = _FakeWebNode(client)
    node.start_save_map({"destination": "/map/run"})

    client.ready = False
    status = node.save_map_status_payload()

    assert status["available"] is False
    assert status["state"] == "failed"
    assert "unavailable" in status["message"]
    assert future.cancelled is True
    assert node.save_map_future is None

    future.response = _FakeSaveMapResponse(True, "late success")
    future.resolve()

    assert node.save_map_job.snapshot()["state"] == "failed"


def test_cleanup_save_map_cancels_pending_request_and_destroys_client(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    future = _FakeFuture()
    client = _FakeSaveMapClient(ready=True, future=future)
    node = _FakeWebNode(client)
    node.start_save_map({"destination": "/map/run"})

    node.cleanup_save_map()

    assert node.destroyed_clients == [client]
    assert node.save_map_client is None
    assert future.cancelled is True
    assert node.save_map_future is None
    snapshot = node.save_map_job.snapshot()
    assert snapshot["state"] == "failed"
    assert "shutting down" in snapshot["message"]


def test_save_map_handler_post_starts_async_job_and_get_reports_result(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    monkeypatch.setenv("HOME", "/home/robot")
    future = _FakeFuture()
    node = _FakeWebNode(_FakeSaveMapClient(ready=True, future=future))
    server, thread = _serve_fake_web_node(node)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        status, body = _json_request(f"{base}/api/mapping/save_map")
        assert status == 200
        assert body["available"] is True
        assert body["state"] == "idle"

        status, body = _json_request(
            f"{base}/api/mapping/save_map",
            {"destination": "/map/run"},
            method="POST",
        )
        assert status == 200
        assert body["ok"] is True
        assert body["state"] == "running"
        assert body["destination"] == "/home/robot/map/run"

        status, body = _json_request(f"{base}/api/mapping/save_map")
        assert status == 200
        assert body["available"] is True
        assert body["state"] == "running"
        assert body["destination"] == "/home/robot/map/run"

        future.response = _FakeSaveMapResponse(True, "saved to /home/robot/map/run")
        future.resolve()

        status, body = _json_request(f"{base}/api/mapping/save_map")
        assert status == 200
        assert body["state"] == "succeeded"
        assert body["success"] is True
        assert body["message"] == "saved to /home/robot/map/run"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def test_save_map_handler_post_returns_409_for_duplicate(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    node = _FakeWebNode(_FakeSaveMapClient(ready=True))
    node.save_map_job.begin("/map/run", 1.0)
    server, thread = _serve_fake_web_node(node)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        status, body = _json_request(
            f"{base}/api/mapping/save_map",
            {"destination": "/map/run"},
            method="POST",
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 409
    assert body["ok"] is False
    assert "already" in body["message"]


def test_save_map_handler_post_returns_503_when_service_missing(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    node = _FakeWebNode(_FakeSaveMapClient(ready=False))
    server, thread = _serve_fake_web_node(node)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        status, body = _json_request(
            f"{base}/api/mapping/save_map",
            {"destination": "/map/run"},
            method="POST",
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 503
    assert body["ok"] is False
    assert "unavailable" in body["message"]


def test_save_map_handler_post_returns_400_for_invalid_payload(monkeypatch):
    monkeypatch.setattr(web_control_node, "SaveMapOptions", _FakeSaveMapOptions, raising=False)
    node = _FakeWebNode(_FakeSaveMapClient(ready=True))
    server, thread = _serve_fake_web_node(node)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        status, body = _json_request(
            f"{base}/api/mapping/save_map",
            {"resolution": 0.05},
            method="POST",
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    assert status == 400
    assert body["ok"] is False
    assert "destination" in body["message"]


def _make_map_dir(root: Path, name: str) -> Path:
    map_dir = root / name
    map_dir.mkdir(parents=True)
    (map_dir / "GlobalMap.pcd").write_bytes(b"global")
    (map_dir / "GroundMap.pcd").write_bytes(b"ground")
    return map_dir


def _tiled_manifest_document(**overrides):
    document = {
        "schema_version": 1,
        "frame_id": "map",
        "point_type": "XYZI",
        "tile_size_m": 20,
        "origin_xy": [0, 0],
        "source": {
            "global_map_name": "GlobalMap.pcd",
            "ground_map_name": "GroundMap.pcd",
        },
        "tiles": {
            "global": [
                {
                    "index": [0, 0],
                    "path": "global/0_0.pcd",
                    "point_count": 2,
                    "bytes": 4,
                }
            ],
            "ground": [
                {
                    "index": [0, 0],
                    "path": "ground/0_0.pcd",
                    "point_count": 2,
                    "bytes": 4,
                }
            ],
        },
    }
    document.update(overrides)
    return document


def _write_tiled_manifest(map_dir: Path, document) -> Path:
    tiled_dir = map_dir / "tiled"
    tiled_dir.mkdir(parents=True, exist_ok=True)
    (tiled_dir / "manifest.yaml").write_text(
        yaml.safe_dump(document), encoding="utf-8"
    )
    return tiled_dir


def _write_tile_files(tiled_dir: Path) -> None:
    (tiled_dir / "global").mkdir(parents=True, exist_ok=True)
    (tiled_dir / "ground").mkdir(parents=True, exist_ok=True)
    (tiled_dir / "global" / "0_0.pcd").write_bytes(b"tile")
    (tiled_dir / "ground" / "0_0.pcd").write_bytes(b"tile")


def test_map_catalog_lists_direct_child_maps_sorted_with_capabilities(tmp_path):
    from web_control.map_catalog import MapCatalog

    root = tmp_path / "map"
    alpha = _make_map_dir(root, "alpha")
    zeta = _make_map_dir(root, "zeta")
    (root / "incomplete").mkdir()
    (root / "GroundMap.pcd").write_bytes(b"stray file")
    (alpha / "sc_db.bin").write_bytes(b"scan context")

    catalog = MapCatalog(root)

    assert [entry["name"] for entry in catalog.list_maps()] == ["alpha", "zeta"]
    assert catalog.get("alpha") == {
        "name": "alpha",
        "global_map_path": str((alpha / "GlobalMap.pcd").resolve()),
        "ground_map_path": str((alpha / "GroundMap.pcd").resolve()),
        "tiled_path": None,
        "has_scan_context": True,
        "warning": None,
    }
    assert catalog.get("zeta")["has_scan_context"] is False


def test_map_catalog_requires_both_pcd_files_as_regular_files(tmp_path):
    from web_control.map_catalog import MapCatalog

    root = tmp_path / "map"
    _make_map_dir(root, "good")
    missing_ground = _make_map_dir(root, "missing_ground")
    (missing_ground / "GroundMap.pcd").unlink()
    directory_global = _make_map_dir(root, "directory_global")
    (directory_global / "GlobalMap.pcd").unlink()
    (directory_global / "GlobalMap.pcd").mkdir()

    catalog = MapCatalog(root)

    assert [entry["name"] for entry in catalog.list_maps()] == ["good"]
    for name in ("missing_ground", "directory_global"):
        with pytest.raises(ValueError):
            catalog.get(name)


@pytest.mark.skipif(os.geteuid() == 0, reason="root bypasses permission checks")
def test_map_catalog_rejects_unreadable_pcd_files(tmp_path):
    from web_control.map_catalog import MapCatalog

    root = tmp_path / "map"
    locked = _make_map_dir(root, "locked")
    (locked / "GlobalMap.pcd").chmod(0o000)
    try:
        catalog = MapCatalog(root)

        assert catalog.list_maps() == []
        with pytest.raises(ValueError):
            catalog.get("locked")
    finally:
        (locked / "GlobalMap.pcd").chmod(0o644)


def test_map_catalog_rejects_names_and_symlinks_that_escape_the_root(tmp_path):
    from web_control.map_catalog import MapCatalog

    root = tmp_path / "map"
    root.mkdir()
    outside = tmp_path / "outside"
    _make_map_dir(outside, "leak")
    (root / "leak").symlink_to(outside / "leak")
    inside = _make_map_dir(root, "inside")
    (inside / "GlobalMap.pcd").unlink()
    (inside / "GlobalMap.pcd").symlink_to(outside / "leak" / "GlobalMap.pcd")

    catalog = MapCatalog(root)

    assert catalog.list_maps() == []
    for name in (".", "..", "a/b", "../outside/leak", "/etc", "", "   ", None, 7):
        with pytest.raises(ValueError):
            catalog.get(name)
    for name in ("leak", "inside"):
        with pytest.raises(ValueError):
            catalog.get(name)


def test_map_catalog_reports_tiled_capability_for_valid_v1_manifest(tmp_path):
    from web_control.map_catalog import MapCatalog

    root = tmp_path / "map"
    tiled_map = _make_map_dir(root, "tiled_map")
    tiled_dir = _write_tiled_manifest(tiled_map, _tiled_manifest_document())
    _write_tile_files(tiled_dir)

    entry = MapCatalog(root).get("tiled_map")

    assert entry["tiled_path"] == str(tiled_dir.resolve())
    assert entry["warning"] is None
    assert entry["global_map_path"] == str((tiled_map / "GlobalMap.pcd").resolve())


def test_map_catalog_falls_back_to_global_map_for_malformed_tiled_manifests(tmp_path):
    from web_control.map_catalog import MapCatalog

    root = tmp_path / "map"
    broken_yaml = _make_map_dir(root, "broken_yaml")
    (broken_yaml / "tiled").mkdir()
    (broken_yaml / "tiled" / "manifest.yaml").write_text(
        "schema_version: [", encoding="utf-8"
    )

    wrong_schema = _make_map_dir(root, "wrong_schema")
    _write_tiled_manifest(wrong_schema, _tiled_manifest_document(schema_version=2))

    missing_tile = _make_map_dir(root, "missing_tile")
    _write_tiled_manifest(missing_tile, _tiled_manifest_document())

    escaping_tile = _make_map_dir(root, "escaping_tile")
    escaping_document = _tiled_manifest_document()
    escaping_document["tiles"]["global"][0]["path"] = "../GlobalMap.pcd"
    escaping_dir = _write_tiled_manifest(escaping_tile, escaping_document)
    _write_tile_files(escaping_dir)

    symlinked_tile = _make_map_dir(root, "symlinked_tile")
    symlinked_dir = _write_tiled_manifest(symlinked_tile, _tiled_manifest_document())
    _write_tile_files(symlinked_dir)
    (symlinked_dir / "global" / "0_0.pcd").unlink()
    (symlinked_dir / "global" / "0_0.pcd").symlink_to(
        symlinked_tile / "GlobalMap.pcd"
    )

    catalog = MapCatalog(root)

    for name in (
        "broken_yaml",
        "wrong_schema",
        "missing_tile",
        "escaping_tile",
        "symlinked_tile",
    ):
        entry = catalog.get(name)
        assert entry["tiled_path"] is None
        assert isinstance(entry["warning"], str) and entry["warning"]
        assert entry["global_map_path"] == str(
            (root / name / "GlobalMap.pcd").resolve()
        )


def test_map_catalog_missing_root_yields_empty_catalog(tmp_path):
    from web_control.map_catalog import MapCatalog

    catalog = MapCatalog(tmp_path / "absent")

    assert catalog.list_maps() == []
    with pytest.raises(ValueError):
        catalog.get("alpha")


def test_selection_store_load_treats_missing_empty_and_malformed_files_as_no_selection(
    tmp_path,
):
    from web_control.map_selection import SelectionStore

    path = tmp_path / "web_control" / "selected_map.json"
    store = SelectionStore(path)

    assert store.load() is None

    path.parent.mkdir(parents=True)
    for content in (
        "",
        "   ",
        "{not json",
        json.dumps([1, 2]),
        json.dumps({"name": 5}),
        json.dumps({"name": "../etc"}),
        json.dumps({"name": ""}),
    ):
        path.write_text(content, encoding="utf-8")
        assert store.load() is None

    path.write_text(json.dumps({"name": "site_a"}), encoding="utf-8")
    assert store.load() == "site_a"


def test_selection_store_logs_corrupt_saved_selection(tmp_path, caplog, capsys):
    from web_control.map_selection import SelectionStore

    path = tmp_path / "selected_map.json"
    path.write_text("{invalid json", encoding="utf-8")

    assert SelectionStore(path).load() is None

    log_output = caplog.text + capsys.readouterr().err
    assert "invalid saved map selection" in log_output
    assert str(path) in log_output


def test_selection_store_save_creates_parent_directory_and_persists_name(tmp_path):
    from web_control.map_selection import SelectionStore

    path = tmp_path / "config" / "web_control" / "selected_map.json"
    store = SelectionStore(path)

    store.save("site_a")

    assert path.is_file()
    assert SelectionStore(path).load() == "site_a"

    store.save("site_b")

    assert SelectionStore(path).load() == "site_b"


def test_selection_store_replaces_selection_file_atomically(tmp_path, monkeypatch):
    from web_control.map_selection import SelectionStore

    path = tmp_path / "web_control" / "selected_map.json"
    store = SelectionStore(path)
    store.save("site_a")

    replacements = []
    real_replace = os.replace

    def spying_replace(source, destination):
        source_path = Path(source)
        assert json.loads(source_path.read_text(encoding="utf-8")) == {
            "name": "site_b"
        }
        replacements.append((source_path, Path(destination)))
        real_replace(source, destination)

    monkeypatch.setattr(os, "replace", spying_replace)
    store.save("site_b")

    assert len(replacements) == 1
    temporary, destination = replacements[0]
    assert temporary.parent == path.parent
    assert destination == path
    assert store.load() == "site_b"
    assert [entry.name for entry in path.parent.iterdir()] == ["selected_map.json"]


def test_selection_store_rejects_names_that_are_not_single_path_components(tmp_path):
    from web_control.map_selection import SelectionStore

    path = tmp_path / "selected_map.json"
    store = SelectionStore(path)

    for name in ("", "   ", ".", "..", "../etc", "a/b", "a\\b", "/etc", None, 7):
        with pytest.raises(ValueError):
            store.save(name)

    assert not path.exists()


def test_selection_store_keeps_saved_name_after_the_map_directory_is_deleted(tmp_path):
    from web_control.map_catalog import MapCatalog
    from web_control.map_selection import SelectionStore

    root = tmp_path / "map"
    map_dir = _make_map_dir(root, "site_a")
    store = SelectionStore(tmp_path / "config" / "selected_map.json")
    store.save("site_a")

    map_dir.rename(tmp_path / "removed_site_a")

    assert store.load() == "site_a"
    with pytest.raises(ValueError):
        MapCatalog(root).get(store.load())


def test_selection_default_path_lives_under_the_runtime_home():
    from web_control.map_selection import default_selection_path

    home = Path("/home/robot")

    assert default_selection_path(home) == Path(
        "/home/robot/.config/web_control/selected_map.json"
    )


def test_map_selection_no_longer_writes_relocalization_parameter_cache():
    from web_control import map_selection

    assert not hasattr(map_selection, "write_localization_params")
    assert not hasattr(map_selection, "default_relocalization_params_path")
    assert "relocalization-selected" not in Path(map_selection.__file__).read_text(
        encoding="utf-8"
    )


def test_package_declares_pyyaml_exec_dependency():
    package_xml = (
        Path(__file__).resolve().parents[1] / "package.xml"
    ).read_text(encoding="utf-8")

    assert "<exec_depend>python3-yaml</exec_depend>" in package_xml


class _RecordingManagedProcess:
    """ManagedLaunchProcess stand-in that records starts instead of spawning."""

    def __init__(self, command, running=False):
        self.command = list(command)
        self.started = 0
        self._running = running

    def is_running(self):
        return self._running

    def start(self):
        self.started += 1


def _selection_node(tmp_path, selection=None, running=(), **overrides):
    """ROS-free node stub for the map selection API and start commands."""
    from web_control.map_catalog import MapCatalog
    from web_control.map_selection import SelectionStore

    parameters = {
        "localization_launch_package": "lio_sam_hesai",
        "localization_launch_file": "localization_with_nonground.launch.py",
        "localization_launch_args": "",
        "navigation_launch_package": "p2p_move_base",
        "navigation_launch_file": "p2p_move_base_localization.launch.py",
        "navigation_launch_args": "",
        "mapping_launch_package": "lio_sam_hesai",
        "mapping_launch_file": "mapping.launch.py",
        "mapping_launch_args": "",
    }
    root = Path(overrides.pop("map_root", tmp_path / "map"))
    parameters.update(overrides)

    node = type("_SelectionNodeStub", (), {})()
    node.get_parameter = lambda name: _ParameterStub(parameters[name])
    node.get_logger = lambda: _LoggerStub()
    node.now_sec = lambda: 10.0
    node.lock = threading.RLock()
    node.map_root = root
    node.map_catalog = MapCatalog(root)
    node.map_selection = SelectionStore(tmp_path / "config" / "selected_map.json")
    node.managed_logs = {
        target: TerminalLogBuffer(target, max_lines=5)
        for target in ("localization", "navigation", "mapping")
    }
    node.managed_processes = {
        "localization": _RecordingManagedProcess(
            [
                "ros2",
                "launch",
                "lio_sam_hesai",
                "localization_with_nonground.launch.py",
            ],
            running="localization" in running,
        ),
        "navigation": _RecordingManagedProcess(
            [
                "ros2",
                "launch",
                "p2p_move_base",
                "p2p_move_base_localization.launch.py",
            ],
            running="navigation" in running,
        ),
        "mapping": _RecordingManagedProcess(
            ["ros2", "launch", "lio_sam_hesai", "mapping.launch.py"],
            running="mapping" in running,
        ),
    }
    if selection is not None:
        node.map_selection.save(selection)
    from web_control.navigation_control import NavigationControl
    from web_control.navigation_store import NavigationStore

    node.navigation_store = NavigationStore(
        tmp_path / "config" / "paused_navigation.json"
    )
    node.navigation_control = NavigationControl(node.navigation_store)
    for name in (
        "maps_payload",
        "select_map",
        "require_valid_selection",
        "managed_start_command",
        "start_managed_target",
        "ui_preferences_payload",
        "save_ui_preferences",
    ):
        setattr(
            node,
            name,
            MethodType(getattr(web_control_node.WebControlNode, name), node),
        )
    return node


def _serve_and_request(node, path, payload=None, method="GET"):
    server, thread = _serve_fake_web_node(node)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        return _json_request(f"{base}{path}", payload, method)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def test_maps_handler_get_returns_sanitized_catalog_with_selection(tmp_path):
    root = tmp_path / "map"
    _make_map_dir(root, "site_a")
    tiled_map = _make_map_dir(root, "site_b")
    (tiled_map / "sc_db.bin").write_bytes(b"scan context")
    tiled_dir = _write_tiled_manifest(tiled_map, _tiled_manifest_document())
    _write_tile_files(tiled_dir)
    node = _selection_node(tmp_path, selection="site_a")

    status, body = _serve_and_request(node, "/api/maps")

    assert status == 200
    assert body["ok"] is True
    assert [entry["name"] for entry in body["maps"]] == ["site_a", "site_b"]
    by_name = {entry["name"]: entry for entry in body["maps"]}
    assert by_name["site_a"] == {
        "name": "site_a",
        "tiled": False,
        "has_scan_context": False,
        "warning": None,
    }
    assert by_name["site_b"]["tiled"] is True
    assert by_name["site_b"]["has_scan_context"] is True
    for entry in body["maps"]:
        assert set(entry) == {"name", "tiled", "has_scan_context", "warning"}
        assert "global_map_path" not in entry
        assert "ground_map_path" not in entry
        assert "tiled_path" not in entry
    assert body["selection"]["name"] == "site_a"
    assert body["selection"]["valid"] is True
    assert body["selection"]["entry"]["name"] == "site_a"


def test_maps_handler_reports_missing_root_and_no_selection(tmp_path):
    node = _selection_node(tmp_path)

    status, body = _serve_and_request(node, "/api/maps")

    assert status == 200
    assert body["ok"] is True
    assert body["maps"] == []
    assert body["selection"] == {
        "name": None,
        "valid": False,
        "entry": None,
        "message": "no map selected",
    }


def test_maps_handler_marks_selection_invalid_after_map_is_deleted(tmp_path):
    root = tmp_path / "map"
    map_dir = _make_map_dir(root, "site_a")
    node = _selection_node(tmp_path, selection="site_a")
    map_dir.rename(tmp_path / "removed_site_a")

    status, body = _serve_and_request(node, "/api/maps")

    assert status == 200
    assert body["selection"]["name"] == "site_a"
    assert body["selection"]["valid"] is False
    assert body["selection"]["entry"] is None
    assert body["selection"]["message"]


def test_maps_handler_select_persists_name_and_returns_state(tmp_path):
    root = tmp_path / "map"
    _make_map_dir(root, "site_a")
    _make_map_dir(root, "site_b")
    node = _selection_node(tmp_path)

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_b"}, "POST"
    )

    assert status == 200
    assert body["ok"] is True
    assert body["selection"]["name"] == "site_b"
    assert body["selection"]["valid"] is True
    assert body["selection"]["entry"]["name"] == "site_b"
    assert node.map_selection.load() == "site_b"

    status, body = _serve_and_request(node, "/api/maps")
    assert status == 200
    assert body["selection"]["name"] == "site_b"
    assert body["selection"]["valid"] is True


@pytest.mark.parametrize(
    "name", ["", "   ", ".", "..", "../etc", "a/b", "/etc", None, 7, "missing"]
)
def test_maps_handler_select_rejects_invalid_or_unknown_names(tmp_path, name):
    _make_map_dir(tmp_path / "map", "site_a")
    node = _selection_node(tmp_path)

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": name}, "POST"
    )

    assert status == 400
    assert body["ok"] is False
    assert node.map_selection.load() is None


@pytest.mark.parametrize("target", ["localization", "navigation"])
def test_maps_handler_select_returns_409_while_managed_process_runs(tmp_path, target):
    root = tmp_path / "map"
    _make_map_dir(root, "site_a")
    _make_map_dir(root, "site_b")
    node = _selection_node(tmp_path, selection="site_a", running=(target,))

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_b"}, "POST"
    )

    assert status == 409
    assert body["ok"] is False
    assert target in body["message"]
    assert "stop" in body["message"].lower()
    assert node.map_selection.load() == "site_a"


def test_maps_handler_select_returns_409_and_names_both_running_processes(tmp_path):
    root = tmp_path / "map"
    _make_map_dir(root, "site_a")
    _make_map_dir(root, "site_b")
    node = _selection_node(
        tmp_path, selection="site_a", running=("localization", "navigation")
    )

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_b"}, "POST"
    )

    assert status == 409
    assert "localization" in body["message"]
    assert "navigation" in body["message"]
    assert node.map_selection.load() == "site_a"


def test_maps_handler_select_allowed_while_only_mapping_runs(tmp_path):
    root = tmp_path / "map"
    _make_map_dir(root, "site_a")
    _make_map_dir(root, "site_b")
    node = _selection_node(tmp_path, selection="site_a", running=("mapping",))

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_b"}, "POST"
    )

    assert status == 200
    assert body["selection"]["name"] == "site_b"
    assert node.map_selection.load() == "site_b"


def test_ui_preferences_get_returns_defaults_and_saved_values(tmp_path):
    node = _selection_node(tmp_path)

    status, body = _serve_and_request(node, "/api/ui/preferences")

    assert status == 200
    assert body["ok"] is True
    assert body["joystick"] == {"size": 180, "x_ratio": 0.05, "y_ratio": 0.75}

    node.map_selection.save_joystick_preferences(
        {"size": 230, "x_ratio": 0.2, "y_ratio": 0.6}
    )

    status, body = _serve_and_request(node, "/api/ui/preferences")

    assert status == 200
    assert body["joystick"] == {"size": 230, "x_ratio": 0.2, "y_ratio": 0.6}


def test_ui_preferences_post_persists_complete_object_and_preserves_selection(tmp_path):
    _make_map_dir(tmp_path / "map", "site_a")
    node = _selection_node(tmp_path, selection="site_a")
    node.map_selection.path.write_text(
        json.dumps({"name": "site_a", "ui": {"theme": "dark"}, "extra": 1}) + "\n",
        encoding="utf-8",
    )

    status, body = _serve_and_request(
        node,
        "/api/ui/preferences",
        {"size": 210, "x_ratio": 0.15, "y_ratio": 0.65},
        "POST",
    )

    assert status == 200
    assert body["ok"] is True
    assert body["joystick"] == {"size": 210, "x_ratio": 0.15, "y_ratio": 0.65}
    assert node.map_selection.load() == "site_a"
    payload = json.loads(node.map_selection.path.read_text(encoding="utf-8"))
    assert payload == {
        "name": "site_a",
        "ui": {
            "theme": "dark",
            "joystick": {"size": 210, "x_ratio": 0.15, "y_ratio": 0.65},
        },
        "extra": 1,
    }


@pytest.mark.parametrize(
    "payload",
    [
        [],
        {},
        {"size": 139, "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": 261, "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": "210", "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": float("nan"), "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": 210, "x_ratio": -0.01, "y_ratio": 0.5},
        {"size": 210, "x_ratio": 0.5, "y_ratio": 1.01},
        {"size": 210, "x_ratio": 0.5},
    ],
)
def test_ui_preferences_post_rejects_invalid_payload_without_changing_file(
    tmp_path, payload
):
    node = _selection_node(tmp_path)
    node.map_selection.save_joystick_preferences(
        {"size": 200, "x_ratio": 0.2, "y_ratio": 0.8}
    )
    before = node.map_selection.path.read_text(encoding="utf-8")

    status, body = _serve_and_request(node, "/api/ui/preferences", payload, "POST")

    assert status == 400
    assert body["ok"] is False
    assert node.map_selection.path.read_text(encoding="utf-8") == before
    assert node.map_selection.load_joystick_preferences() == {
        "size": 200,
        "x_ratio": 0.2,
        "y_ratio": 0.8,
    }


def test_managed_start_command_uses_tiled_map_without_writing_cache(tmp_path):
    root = tmp_path / "map"
    tiled_map = _make_map_dir(root, "site_tiled")
    (tiled_map / "sc_db.bin").write_bytes(b"scan context")
    tiled_dir = _write_tiled_manifest(tiled_map, _tiled_manifest_document())
    _write_tile_files(tiled_dir)
    node = _selection_node(
        tmp_path,
        selection="site_tiled",
        localization_launch_args="use_rviz:=false debug_relocalization:=false",
        navigation_launch_args="use_rviz:=false",
    )

    localization = node.managed_start_command("localization")

    assert localization[:4] == [
        "ros2",
        "launch",
        "lio_sam_hesai",
        "localization_with_nonground.launch.py",
    ]
    assert "use_rviz:=false" in localization
    assert "debug_relocalization:=false" in localization
    assert [a for a in localization if a.startswith("map_path:=")] == [
        f"map_path:={tiled_dir.resolve()}"
    ]
    assert not any(a.startswith("params_file:=") for a in localization)
    assert not any(a.startswith("auto_relocalization:=") for a in localization)
    # The selected map is passed straight to the launch; the old generated
    # relocalization parameter cache is neither created nor referenced.
    assert not (tmp_path / ".cache").exists()
    assert not (tmp_path / "cache").exists()

    navigation = node.managed_start_command("navigation")

    assert navigation[:4] == [
        "ros2",
        "launch",
        "p2p_move_base",
        "p2p_move_base_localization.launch.py",
    ]
    assert "use_rviz:=false" in navigation
    expected_map_root = str((tiled_map / "GlobalMap.pcd").resolve().parent)
    assert [a for a in navigation if a.startswith("map_path:=")] == [
        f"map_path:={expected_map_root}"
    ]
    assert not any(
        a.startswith(("map_dir:=", "ground_dir:=")) for a in navigation
    )


def test_managed_start_command_uses_global_map_and_disables_auto_relocalization(
    tmp_path,
):
    root = tmp_path / "map"
    map_dir = _make_map_dir(root, "site_global")
    node = _selection_node(tmp_path, selection="site_global")

    localization = node.managed_start_command("localization")

    assert [a for a in localization if a.startswith("map_path:=")] == [
        f"map_path:={(map_dir / 'GlobalMap.pcd').resolve()}"
    ]
    assert [a for a in localization if a.startswith("auto_relocalization:=")] == [
        "auto_relocalization:=false"
    ]
    assert not any(a.startswith("params_file:=") for a in localization)
    assert not (tmp_path / ".cache").exists()
    assert not (tmp_path / "cache").exists()

    navigation = node.managed_start_command("navigation")

    expected_map_root = str((map_dir / "GlobalMap.pcd").resolve().parent)
    assert [a for a in navigation if a.startswith("map_path:=")] == [
        f"map_path:={expected_map_root}"
    ]
    assert not any(
        a.startswith(("map_dir:=", "ground_dir:=")) for a in navigation
    )


def test_managed_start_command_replaces_conflicting_configured_map_args(tmp_path):
    root = tmp_path / "map"
    map_dir = _make_map_dir(root, "site_a")
    node = _selection_node(
        tmp_path,
        selection="site_a",
        localization_launch_args=(
            "map_path:=/stale/map auto_relocalization:=true use_rviz:=false"
        ),
        navigation_launch_args=(
            "map_path:=/stale/map_root use_rviz:=false"
        ),
    )

    localization = node.managed_start_command("localization")
    assert [a for a in localization if a.startswith("map_path:=")] == [
        f"map_path:={(map_dir / 'GlobalMap.pcd').resolve()}"
    ]
    assert [a for a in localization if a.startswith("auto_relocalization:=")] == [
        "auto_relocalization:=false"
    ]
    assert "use_rviz:=false" in localization
    assert "auto_relocalization:=true" not in localization

    navigation = node.managed_start_command("navigation")
    expected_map_root = str((map_dir / "GlobalMap.pcd").resolve().parent)
    assert [a for a in navigation if a.startswith("map_path:=")] == [
        f"map_path:={expected_map_root}"
    ]
    assert "use_rviz:=false" in navigation


def test_managed_start_commands_require_and_revalidate_selection(tmp_path):
    root = tmp_path / "map"
    map_dir = _make_map_dir(root, "site_a")
    mapping_command = ["ros2", "launch", "lio_sam_hesai", "mapping.launch.py"]

    no_selection = _selection_node(tmp_path)
    with pytest.raises(ValueError):
        no_selection.managed_start_command("localization")
    with pytest.raises(ValueError):
        no_selection.managed_start_command("navigation")
    assert no_selection.managed_start_command("mapping") == mapping_command

    stale = _selection_node(tmp_path, selection="site_a")
    map_dir.rename(tmp_path / "removed_site_a")
    with pytest.raises(ValueError):
        stale.managed_start_command("localization")
    with pytest.raises(ValueError):
        stale.managed_start_command("navigation")
    assert stale.managed_start_command("mapping") == mapping_command


def test_managed_start_target_rejects_invalid_selection_but_starts_mapping(tmp_path):
    root = tmp_path / "map"
    _make_map_dir(root, "site_a")
    node = _selection_node(tmp_path)

    with pytest.raises(ValueError):
        node.start_managed_target("localization")
    with pytest.raises(ValueError):
        node.start_managed_target("navigation")
    assert node.managed_processes["localization"].started == 0
    assert node.managed_processes["navigation"].started == 0

    result = node.start_managed_target("mapping")

    assert result["ok"] is True
    assert node.managed_processes["mapping"].started == 1


def test_localization_and_navigation_starts_use_the_same_selected_candidate(tmp_path):
    root = tmp_path / "map"
    map_dir = _make_map_dir(root, "site_a")
    node = _selection_node(tmp_path, selection="site_a")

    node.start_managed_target("localization")
    node.start_managed_target("navigation")

    localization = node.managed_processes["localization"].command
    navigation = node.managed_processes["navigation"].command
    expected_global = str((map_dir / "GlobalMap.pcd").resolve())
    expected_map_root = str(Path(expected_global).parent)

    assert f"map_path:={expected_global}" in localization
    assert not any(a.startswith("params_file:=") for a in localization)
    assert f"map_path:={expected_map_root}" in navigation
    assert node.managed_processes["localization"].started == 1
    assert node.managed_processes["navigation"].started == 1


def _frontend_sources():
    """Return ``(index_html, app_js, styles_css)`` for browser-facing tests."""
    static_dir = Path(__file__).resolve().parents[1] / "static"
    return (
        (static_dir / "index.html").read_text(encoding="utf-8"),
        (static_dir / "app.js").read_text(encoding="utf-8"),
        (static_dir / "styles.css").read_text(encoding="utf-8"),
    )


def _js_function_body(app_js, signature):
    """Return the source of one top-level function, matched by braces."""
    start = app_js.index(signature)
    brace = app_js.index("{", start)
    depth = 0
    for index in range(brace, len(app_js)):
        if app_js[index] == "{":
            depth += 1
        elif app_js[index] == "}":
            depth -= 1
            if depth == 0:
                return app_js[start : index + 1]
    raise AssertionError(f"unterminated function starting with {signature!r}")


_MAP_PANEL_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("// --- Map selector");
const end = source.indexOf("// Per-target cursor");
if (start < 0 || end < 0) throw new Error("map selector block not found");
const block = source.slice(start, end);

function makeElement() {
  return {
    textContent: "",
    className: "",
    innerHTML: "",
    value: "",
    disabled: false,
    hidden: false,
    children: [],
    appendChild(child) { this.children.push(child); },
  };
}
const elements = {};
for (const id of [
  "mapSelect", "mapTiledLabel", "mapRelocalizationLabel", "mapManualInitialPose",
  "mapWarning", "mapStatus", "mapLockNote",
]) {
  elements[id] = makeElement();
}
global.document = {
  getElementById(id) { return elements[id] || null; },
  createElement() { return makeElement(); },
};

let state = {
  localizationRunning: false,
  navigationRunning: false,
  mapSelectPending: false,
  mapCatalog: null,
  mapError: null,
  mapSelectionError: null,
  mapSelectionNotice: null,
};
let captured = null;
global.getJson = async () => ({
  ok: true,
  map_root: "~/map",
  maps: [],
  selection: { name: null, valid: false, entry: null, message: "no map selected" },
});
global.postJson = async (url, payload) => {
  captured = { url, payload };
  throw new Error("stop navigation before selecting a different map");
};

eval(block);

function expect(condition, message) {
  if (!condition) throw new Error(message);
}

// A valid tiled map with sc_db.bin renders both capability labels.
state.mapCatalog = {
  map_root: "~/map",
  maps: [{ name: "site_a", tiled: true, has_scan_context: true, warning: null }],
  selection: {
    name: "site_a",
    valid: true,
    entry: { name: "site_a", tiled: true, has_scan_context: true, warning: null },
    message: null,
  },
};
updateMapPanel();
expect(elements.mapSelect.value === "site_a", "selected map is not reflected in the selector");
expect(!elements.mapSelect.disabled, "selector should be enabled for a valid selection");
expect(/tiled loading: available/i.test(elements.mapTiledLabel.textContent), "tiled label missing");
expect(/automatic relocalization: available/i.test(elements.mapRelocalizationLabel.textContent), "relocalization label missing");
expect(elements.mapManualInitialPose.hidden, "manual initial pose note should be hidden");

// A global map without sc_db.bin asks for a manual initial pose and warns.
const warning = "tiled manifest is invalid: bad schema";
state.mapCatalog.maps = [{ name: "site_b", tiled: false, has_scan_context: false, warning }];
state.mapCatalog.selection = {
  name: "site_b",
  valid: true,
  entry: { name: "site_b", tiled: false, has_scan_context: false, warning },
  message: null,
};
updateMapPanel();
expect(!elements.mapManualInitialPose.hidden, "manual initial pose note should be shown");
expect(!elements.mapWarning.hidden, "tile warning should be visible");
expect(/tile warning/i.test(elements.mapWarning.textContent), "tile warning was not surfaced");

// A saved but deleted selection stays visible as unavailable.
state.mapCatalog.maps = [];
state.mapCatalog.selection = {
  name: "gone",
  valid: false,
  entry: null,
  message: "map 'gone' is not a directory",
};
updateMapPanel();
expect(elements.mapSelect.children.some((option) => option.value === "gone"), "stale selection is not offered");
expect(/unavailable/i.test(elements.mapStatus.textContent), "stale selection is not reported");

// Running localization locks out selection without a server round trip.
state.mapCatalog = {
  map_root: "~/map",
  maps: [{ name: "site_a", tiled: false, has_scan_context: true, warning: null }],
  selection: {
    name: "site_a",
    valid: true,
    entry: { name: "site_a", tiled: false, has_scan_context: true, warning: null },
    message: null,
  },
};
state.localizationRunning = true;
updateMapSelectionLock();
expect(elements.mapSelect.disabled, "running localization must disable selection");
expect(/localization/.test(elements.mapLockNote.textContent), "lock note does not name localization");

// A rejected selection posts {name} and shows the server message.
(async () => {
  state.localizationRunning = false;
  await selectMap("site_b");
  expect(captured && captured.url === "/api/maps/select", "selection was not posted to the select endpoint");
  expect(captured.payload.name === "site_b", "selection payload is wrong");
  expect(/map selection failed/i.test(elements.mapStatus.textContent), "selection error was not surfaced");
  expect(/stop navigation/i.test(elements.mapStatus.textContent), "server message was not shown");
  console.log("map panel behavior ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_frontend_map_selector_behavior_executes_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _MAP_PANEL_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "map panel behavior ok" in completed.stdout


class _FakeCancelResponse:
    def __init__(self, goals_canceling, return_code=0):
        self.goals_canceling = goals_canceling
        self.return_code = return_code


class _FakeCancelGoalRequest:
    class _GoalId:
        def __init__(self):
            self.uuid = []

    class _Stamp:
        def __init__(self):
            self.sec = 0
            self.nanosec = 0

    class _GoalInfo:
        def __init__(self):
            self.goal_id = _FakeCancelGoalRequest._GoalId()
            self.stamp = _FakeCancelGoalRequest._Stamp()

    def __init__(self):
        self.goal_info = _FakeCancelGoalRequest._GoalInfo()


class _FakeCancelGoal:
    Request = _FakeCancelGoalRequest


class _FakeCancelClient:
    def __init__(self, ready=True, future=None, results=(1,), return_code=0):
        self.ready = ready
        self.future = future if future is not None else _FakeFuture()
        self.results = list(results)
        self.return_code = return_code
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        self.future.response = _FakeCancelResponse(
            [object() for _ in self.results], return_code=self.return_code
        )
        return self.future


class _FakeHeader:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class _FakePosition:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _FakeOrientation:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _FakePose:
    def __init__(self):
        self.position = _FakePosition()
        self.orientation = _FakeOrientation()


class _FakePoseStamped:
    def __init__(self):
        self.header = _FakeHeader()
        self.pose = _FakePose()


class _FakePublisher:
    def __init__(self, published):
        self.published = published

    def publish(self, msg):
        self.published.append(msg)


def _paused_record(phase="paused", map_name="site_a"):
    return {
        "schema_version": 1,
        "phase": phase,
        "x": 1.5,
        "y": -2.5,
        "z": 0.25,
        "yaw": 0.75,
        "frame": "map",
        "map_name": map_name,
        "saved_at": 5.0,
    }


def _navigation_node(
    tmp_path,
    record=None,
    selection="site_a",
    client=None,
    ready=True,
    results=(1,),
    methods=("state_payload",),
):
    from web_control.map_selection import SelectionStore
    from web_control.navigation_control import NavigationControl
    from web_control.navigation_store import NavigationStore

    web_control_node.CancelGoal = _FakeCancelGoal
    web_control_node.PoseStamped = _FakePoseStamped
    node = type("_NavigationNodeStub", (), {})()
    node.lock = threading.RLock()
    node.map_frame = "map"
    node.now_sec = lambda: 10.0
    node.ros_stamp = lambda: None
    node.get_logger = lambda: _LoggerStub()
    node.map_selection = SelectionStore(tmp_path / "config" / "selected_map.json")
    if selection is not None:
        node.map_selection.save(selection)
    node.navigation_store = NavigationStore(
        tmp_path / "config" / "paused_navigation.json"
    )
    if record is not None:
        node.navigation_store.save(record)
    node.navigation_control = NavigationControl(node.navigation_store)
    node.published_goal = None
    node.cancel_goal_client = (
        client if client is not None else _FakeCancelClient(ready=ready, results=results)
    )
    if client is None:
        node.cancel_goal_client.future.done_flag = True
    node.goal_publications = []
    node.goal_pub = _FakePublisher(node.goal_publications)
    node.action_status = SourceCache("action_status", 2.0)
    node.odom = SourceCache("odom", 2.0)
    node.layers = {
        name: SourceCache(name, 2.0, static_source=True)
        for name in ("local_map", "mapcloud", "mapground")
    }
    node.paths = {}
    node.viewers = ViewerRegistry(timeout_sec=10.0)
    # ``_spawn_cancel_retry`` is always available so timeout paths can be driven
    # deterministically; tests may override it with a recorder.
    bound = set(methods) | {"_spawn_cancel_retry"}
    for name in bound:
        setattr(
            node,
            name,
            MethodType(getattr(web_control_node.WebControlNode, name), node),
        )
    return node


def _arm_active_goal(node, pose=None):
    pose = pose or {"x": 1.5, "y": -2.5, "z": 0.25, "yaw": 0.75}
    # Publish first, then feed the status, mirroring the real node ordering so
    # the transition floor is taken from the previous goal stamp.
    node.navigation_control.record_goal_published(pose, "map", "site_a", 10.0)
    node.navigation_control.note_action_status("executing", 5, 10.0)
    node.published_goal = dict(pose)


class _GoalStamp:
    def __init__(self, goal_stamp):
        self.sec = goal_stamp // 1_000_000_000
        self.nanosec = goal_stamp % 1_000_000_000


class _GoalStatusInfo:
    def __init__(self, goal_stamp):
        self.stamp = _GoalStamp(goal_stamp)


class _GoalStatus:
    def __init__(self, value, goal_stamp):
        self.status = value
        self.goal_info = _GoalStatusInfo(goal_stamp)


class _GoalStatusArray:
    def __init__(self, statuses):
        self.status_list = statuses


def test_node_declares_cancel_goal_service_and_client():
    package_root = Path(__file__).resolve().parents[1]
    source = (package_root / "web_control" / "web_control_node.py").read_text(
        encoding="utf-8"
    )

    assert "from action_msgs.srv import CancelGoal" in source
    assert "CancelGoal = None" in source
    assert '"cancel_goal_service", "/p2p_move_base/_action/cancel_goal"' in source
    assert 'CancelGoal, str(self.get_parameter("cancel_goal_service").value)' in source

    config = (package_root / "config" / "web_control.yaml").read_text(encoding="utf-8")
    assert 'cancel_goal_service: "/p2p_move_base/_action/cancel_goal"' in config


def test_cancel_goal_import_is_isolated_from_main_ros_import():
    package_root = Path(__file__).resolve().parents[1]
    source = (package_root / "web_control" / "web_control_node.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)

    main_ros_try = None
    cancel_goal_try = None
    for statement in tree.body:
        if not isinstance(statement, ast.Try):
            continue
        statements = ast.unparse(statement)
        if "import rclpy" in statements:
            main_ros_try = statement
        if "from action_msgs.srv import CancelGoal" in statements:
            cancel_goal_try = statement

    assert main_ros_try is not None, "main ROS import try block not found"
    assert cancel_goal_try is not None, "isolated CancelGoal import try not found"
    assert cancel_goal_try is not main_ros_try, (
        "CancelGoal must not share the rclpy import try, or a missing service "
        "type would set rclpy to None and disable the whole node"
    )
    assert "from action_msgs.srv import CancelGoal" not in ast.unparse(main_ros_try)
    assert len(cancel_goal_try.handlers) == 1
    assert "CancelGoal = None" in ast.unparse(cancel_goal_try.handlers[0])


def test_action_status_callback_uses_selected_status():
    source = (
        Path(__file__).resolve().parents[1] / "web_control" / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "select_action_status(msg.status_list)" in source
    assert "self.navigation_control.note_action_status(" in source
    assert "self.navigation_control.note_no_active_goals(" in source


def test_state_payload_exposes_navigation_control_snapshot(tmp_path):
    node = _navigation_node(tmp_path, record=_paused_record())

    body = node.state_payload()

    assert body["navigation_control"]["state"] == "paused"
    assert body["navigation_control"]["can_resume"] is True
    assert body["navigation_control"]["paused_goal"]["x"] == 1.5
    assert body["navigation_control"]["paused_goal"]["map_name"] == "site_a"


class _EmptyGoalStatusArray:
    def __init__(self):
        self.status_list = []


def test_action_status_callback_empty_list_reconciles_pausing(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(phase="pausing"),
        methods=("action_status_callback", "state_payload"),
    )

    node.action_status_callback(_EmptyGoalStatusArray())

    assert node.action_status.data == {"status": "idle", "status_values": []}
    assert node.action_status.count == 0
    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "paused"
    assert snapshot["can_resume"] is True
    assert node.navigation_store.load()["phase"] == "paused"


def test_action_status_callback_empty_list_and_no_record_is_idle(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=("action_status_callback", "state_payload"),
    )

    node.action_status_callback(_EmptyGoalStatusArray())

    assert node.action_status.data == {"status": "idle", "status_values": []}
    assert node.action_status.count == 0
    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "idle"
    assert snapshot["paused_goal"] is None


def test_cancel_all_goals_sends_zero_uuid_and_zero_stamp(tmp_path):
    client = _FakeCancelClient(return_code=0, results=(1,))
    node = _navigation_node(tmp_path, client=client, methods=("_cancel_all_goals",))
    client.future.done_flag = True

    assert node._cancel_all_goals() == "accepted"
    request = client.requests[0]
    assert request.goal_info.goal_id.uuid == [0] * 16
    assert request.goal_info.stamp.sec == 0
    assert request.goal_info.stamp.nanosec == 0


def test_cancel_all_goals_reports_rejected_return_code(tmp_path):
    client = _FakeCancelClient(return_code=2, results=(1,))
    node = _navigation_node(tmp_path, client=client, methods=("_cancel_all_goals",))
    client.future.done_flag = True

    assert node._cancel_all_goals() == "rejected"


def test_cancel_all_goals_distinguishes_accepted_and_empty(tmp_path):
    accepted_client = _FakeCancelClient(return_code=0, results=(1,))
    accepted_node = _navigation_node(
        tmp_path, client=accepted_client, methods=("_cancel_all_goals",)
    )
    accepted_client.future.done_flag = True

    empty_client = _FakeCancelClient(return_code=0, results=())
    empty_node = _navigation_node(
        tmp_path, client=empty_client, methods=("_cancel_all_goals",)
    )
    empty_client.future.done_flag = True

    assert accepted_node._cancel_all_goals() == "accepted"
    assert empty_node._cancel_all_goals() == "empty"


def test_cancel_all_goals_maps_future_error_to_unavailable(tmp_path):
    future = _FakeFuture(error=RuntimeError("service crashed"))
    future.done_flag = True
    client = _FakeCancelClient(future=future)
    node = _navigation_node(tmp_path, client=client, methods=("_cancel_all_goals",))

    with pytest.raises(web_control_node.NavigationUnavailableError):
        node._cancel_all_goals()


def test_cancel_all_goals_times_out(tmp_path, monkeypatch):
    # The wait is bounded, so a service that never answers must fail the request
    # thread instead of blocking the HTTP worker forever.
    future = _FakeFuture()
    client = _FakeCancelClient(future=future)
    node = _navigation_node(tmp_path, client=client, methods=("_cancel_all_goals",))
    monkeypatch.setattr(web_control_node, "CANCEL_GOAL_TIMEOUT_SEC", 0.0)

    with pytest.raises(web_control_node.NavigationTimeoutError):
        node._cancel_all_goals()

    assert future.cancelled is True


def test_cancel_all_goals_maps_call_async_error_to_unavailable(tmp_path):
    class _RaisingCancelClient(_FakeCancelClient):
        def call_async(self, request):
            self.requests.append(request)
            raise RuntimeError("service handle gone")

    client = _RaisingCancelClient()
    node = _navigation_node(tmp_path, client=client, methods=("_cancel_all_goals",))

    with pytest.raises(web_control_node.NavigationUnavailableError):
        node._cancel_all_goals()


def test_pause_navigation_persists_pausing_then_canceled_reaches_paused(tmp_path):
    # The zero-UUID/zero-stamp request shape is asserted by Task 4's
    # ``test_cancel_all_goals_sends_zero_uuid_and_zero_stamp``; this test owns the
    # pause behavior around the accepted cancel.
    node = _navigation_node(
        tmp_path,
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)

    response = node.pause_navigation()

    assert response["navigation_control"]["state"] == "pausing"
    assert response["navigation_control"]["paused_goal"]["phase"] == "pausing"
    assert len(node.cancel_goal_client.requests) == 1

    node.navigation_control.note_action_status("canceled", 5, 11.0)
    assert node.navigation_control.snapshot("site_a")["state"] == "paused"
    assert node.navigation_store.load()["phase"] == "paused"


def test_pause_navigation_timeout_keeps_pausing_record(tmp_path, monkeypatch):
    future = _FakeFuture()
    client = _FakeCancelClient(ready=True, future=future)
    node = _navigation_node(
        tmp_path,
        client=client,
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)
    monkeypatch.setattr(web_control_node, "CANCEL_GOAL_TIMEOUT_SEC", 0.0)
    # Keep the bounded background retry from racing the assertion below.
    node._spawn_cancel_retry = lambda kind, token: None

    with pytest.raises(web_control_node.NavigationTimeoutError):
        node.pause_navigation()

    assert node.navigation_control.snapshot("site_a")["state"] == "pausing"
    assert node.navigation_store.load()["phase"] == "pausing"


def test_pause_navigation_unavailable_removes_record(tmp_path):
    node = _navigation_node(
        tmp_path,
        ready=False,
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)

    with pytest.raises(web_control_node.NavigationUnavailableError):
        node.pause_navigation()

    assert node.navigation_store.load() is None
    assert node.navigation_control.snapshot("site_a")["state"] == "navigating"


def test_pause_navigation_empty_cancel_removes_record(tmp_path):
    node = _navigation_node(
        tmp_path,
        results=(),
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)

    with pytest.raises(web_control_node.NavigationConflictError):
        node.pause_navigation()

    assert node.navigation_store.load() is None
    assert node.navigation_control.snapshot("site_a")["state"] == "navigating"


def test_pause_navigation_rejected_cancel_restores_navigating(tmp_path):
    client = _FakeCancelClient(return_code=1, results=(1,))
    node = _navigation_node(
        tmp_path,
        client=client,
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    client.future.done_flag = True
    _arm_active_goal(node)

    with pytest.raises(web_control_node.NavigationConflictError):
        node.pause_navigation()

    assert node.navigation_store.load() is None
    assert node.navigation_control.snapshot("site_a")["state"] == "navigating"


def test_clear_paused_goal_skips_cancel_service(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("state_payload", "_cancel_all_goals", "clear_navigation"),
    )

    response = node.clear_navigation()

    assert response["navigation_control"]["state"] == "idle"
    assert node.cancel_goal_client.requests == []
    assert node.navigation_store.load() is None


def test_resume_republishes_saved_pose_and_enters_resuming(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("state_payload", "_publish_goal_pose", "resume_navigation"),
    )

    response = node.resume_navigation()

    assert response["navigation_control"]["state"] == "resuming"
    message = node.goal_publications[-1]
    assert message.header.frame_id == "map"
    assert message.pose.position.x == 1.5
    assert message.pose.position.y == -2.5
    assert message.pose.position.z == 0.25
    assert node.navigation_store.load()["phase"] == "resuming"

    node.navigation_control.note_action_status("accepted", 9, 30.0)
    assert node.navigation_control.snapshot("site_a")["state"] == "navigating"
    assert node.navigation_store.load() is None


def test_clear_while_pausing_reuses_cancel(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "pause_navigation",
            "clear_navigation",
        ),
    )
    _arm_active_goal(node)
    node.pause_navigation()

    response = node.clear_navigation()

    assert response["navigation_control"]["state"] == "clearing"
    assert len(node.cancel_goal_client.requests) == 1

    node.navigation_control.note_action_status("canceled", 5, 12.0)
    assert node.navigation_control.snapshot("site_a")["state"] == "idle"


def test_pause_then_clear_cancel_failure_discards_record_and_keeps_goal(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "_cancel_all_goals",
            "pause_navigation",
            "clear_navigation",
            "state_payload",
        ),
    )
    _arm_active_goal(node)
    node.pause_navigation()
    assert node.navigation_store.load()["phase"] == "pausing"
    assert node.published_goal is not None

    # Clear reuses the pause cancellation, so nothing is settled yet.
    node.clear_navigation()
    assert node.navigation_control.snapshot("site_a")["state"] == "clearing"
    assert node.published_goal is not None

    # The in-flight cancellation failed (e.g. the service vanished).
    node.navigation_control.note_cancel_failed(11.0)

    assert node.navigation_store.load() is None
    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "navigating"
    assert snapshot["has_goal"] is True
    assert snapshot["can_clear"] is True
    assert node.published_goal is not None


def test_clear_before_active_sends_cancel_and_keeps_published_goal(tmp_path):
    client = _FakeCancelClient(return_code=0, results=())
    node = _navigation_node(
        tmp_path,
        client=client,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "publish_goal",
            "clear_navigation",
            "state_payload",
        ),
    )
    client.future.done_flag = True

    node.publish_goal({"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0})
    assert node.published_goal is not None

    # The goal has not been confirmed active yet, but Clear must still ask the
    # Action server to cancel it and keep the published goal until it settles.
    body = node.clear_navigation()

    assert body["navigation_control"]["state"] == "clearing"
    assert body["navigation_control"]["has_goal"] is True
    assert node.published_goal is not None
    assert len(client.requests) == 1


def test_clear_before_active_registration_retries_then_reaches_idle(
    tmp_path, monkeypatch
):
    client = _FakeCancelClient(return_code=0, results=())
    node = _navigation_node(
        tmp_path,
        client=client,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "publish_goal",
            "clear_navigation",
            "state_payload",
            "action_status_callback",
            "_retry_startup_cancel",
        ),
    )
    client.future.done_flag = True
    node.publish_goal({"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0})
    node.clear_navigation()
    assert node.navigation_control.snapshot("site_a")["state"] == "clearing"
    assert len(client.requests) == 1

    spawned = []

    class _RecordingThread:
        def __init__(self, target=None, args=(), name=None, daemon=None):
            spawned.append((target, args))

        def start(self):
            pass

    monkeypatch.setattr(web_control_node.threading, "Thread", _RecordingThread)

    # A newer accepted status confirms registration and queues exactly one retry.
    node.action_status_callback(_GoalStatusArray([_GoalStatus(1, 7)]))
    assert len(spawned) == 1
    retry_target, retry_args = spawned[0]
    assert retry_target == node._retry_startup_cancel
    assert retry_args[0] is True
    node.action_status_callback(_GoalStatusArray([_GoalStatus(2, 7)]))
    assert len(spawned) == 1
    assert node.navigation_control.snapshot("site_a")["state"] == "clearing"

    # The retried cancel is accepted; canceling keeps clearing, canceled settles.
    client.results = [1]
    node._retry_startup_cancel(True)
    assert len(client.requests) == 2

    node.action_status_callback(_GoalStatusArray([_GoalStatus(3, 7)]))
    assert node.navigation_control.snapshot("site_a")["state"] == "clearing"
    node.action_status_callback(_GoalStatusArray([_GoalStatus(5, 7)]))

    snapshot = node.navigation_control.snapshot("site_a")
    assert snapshot["state"] == "idle"
    assert snapshot["has_goal"] is False
    assert node.published_goal is None


def test_clear_before_active_accepted_cancel_spawns_no_retry(tmp_path, monkeypatch):
    # The initial CancelGoal matched the goal (accepted), so no registration
    # race occurred and a confirming status must not send a duplicate cancel.
    client = _FakeCancelClient(return_code=0, results=(1,))
    node = _navigation_node(
        tmp_path,
        client=client,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "publish_goal",
            "clear_navigation",
            "state_payload",
            "action_status_callback",
            "_retry_startup_cancel",
        ),
    )
    client.future.done_flag = True
    node.publish_goal({"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0})
    body = node.clear_navigation()
    assert body["navigation_control"]["state"] == "clearing"
    assert len(client.requests) == 1

    spawned = []

    class _RecordingThread:
        def __init__(self, target=None, args=(), name=None, daemon=None):
            spawned.append((target, args))

        def start(self):
            pass

    monkeypatch.setattr(web_control_node.threading, "Thread", _RecordingThread)

    node.action_status_callback(_GoalStatusArray([_GoalStatus(1, 7)]))
    assert spawned == []
    assert node.navigation_control.snapshot("site_a")["state"] == "clearing"

    # The original accepted cancel's terminal settles the clear to Idle without
    # ever issuing a second cancel request.
    node.action_status_callback(_GoalStatusArray([_GoalStatus(3, 7)]))
    assert node.navigation_control.snapshot("site_a")["state"] == "clearing"
    node.action_status_callback(_GoalStatusArray([_GoalStatus(5, 7)]))
    snapshot = node.navigation_control.snapshot("site_a")
    assert snapshot["state"] == "idle"
    assert snapshot["has_goal"] is False
    assert node.published_goal is None
    assert len(client.requests) == 1


def test_clear_registration_retry_empty_restores_navigating(tmp_path):
    client = _FakeCancelClient(return_code=0, results=())
    node = _navigation_node(
        tmp_path,
        client=client,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "publish_goal",
            "clear_navigation",
            "state_payload",
            "_retry_startup_cancel",
        ),
    )
    client.future.done_flag = True
    node.publish_goal({"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0})
    node.clear_navigation()
    assert node.navigation_control.snapshot("site_a")["state"] == "clearing"

    # The goal registers, but the retried cancel still matches no goal.
    node.navigation_control.note_action_status("accepted", 7, 11.0)
    node._retry_startup_cancel(True)

    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "navigating"
    assert snapshot["has_goal"] is True
    assert snapshot["can_clear"] is True
    assert node.published_goal is not None


def test_resume_endpoint_rejects_saved_frame_mismatch(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("state_payload", "_publish_goal_pose", "resume_navigation"),
    )
    node.map_frame = "odom"

    status, body = _serve_and_request(node, "/api/navigation/resume", {}, "POST")

    assert status == 409
    assert body["ok"] is False
    assert node.goal_publications == []
    assert node.navigation_store.load()["phase"] == "paused"
    assert node.navigation_control.snapshot("site_a")["state"] == "paused"


def test_clear_navigation_runtime_state_settles_control_and_goal(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "pause_navigation",
            "clear_navigation_runtime_state",
        ),
    )
    _arm_active_goal(node)
    node.pause_navigation()
    assert node.navigation_control.snapshot("site_a")["state"] == "pausing"

    node.clear_navigation_runtime_state()

    # Stopping navigation must not leave a phantom active goal in the browser.
    assert node.published_goal is None
    snapshot = node.navigation_control.snapshot("site_a")
    assert snapshot["state"] == "paused"
    assert snapshot["can_resume"] is True
    assert node.navigation_store.load()["phase"] == "paused"


def test_publish_goal_rejected_while_pausing(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "pause_navigation",
            "publish_goal",
        ),
    )
    _arm_active_goal(node)
    node.pause_navigation()

    with pytest.raises(web_control_node.NavigationConflictError):
        node.publish_goal({"x": 3.0, "y": 4.0, "yaw": 0.0})

    assert node.goal_publications == []
    assert node.navigation_control.snapshot("site_a")["state"] == "pausing"


def test_new_goal_discards_paused_record_after_publish(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("_publish_goal_pose", "publish_goal"),
    )

    response = node.publish_goal({"x": 5.0, "y": 6.0, "yaw": 1.0})

    assert response["navigation_control"]["state"] == "navigating"
    assert node.navigation_store.load() is None
    assert len(node.goal_publications) == 1
    assert node.goal_publications[0].pose.position.x == 5.0


def test_new_goal_publish_failure_keeps_paused_record(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("_publish_goal_pose", "publish_goal"),
    )

    def _fail_publish(pose, frame=None):
        raise RuntimeError("goal publish failed")

    node._publish_goal_pose = _fail_publish

    with pytest.raises(RuntimeError, match="goal publish failed"):
        node.publish_goal({"x": 5.0, "y": 6.0, "yaw": 1.0})

    assert node.goal_publications == []
    assert node.navigation_store.load() == _paused_record()
    snapshot = node.navigation_control.snapshot("site_a")
    assert snapshot["state"] == "paused"
    assert snapshot["paused_goal"]["x"] == 1.5


def test_select_map_clears_paused_goal_from_previous_map(tmp_path):
    from web_control.navigation_control import NavigationControl

    root = tmp_path / "map"
    _make_map_dir(root, "site_a")
    _make_map_dir(root, "site_b")
    node = _selection_node(tmp_path, selection="site_a")
    node.navigation_store.save(_paused_record(map_name="site_a"))
    # Mirror a restart: the control loads the persisted record at construction.
    node.navigation_control = NavigationControl(node.navigation_store)

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_b"}, "POST"
    )

    assert status == 200
    assert body["selection"]["name"] == "site_b"
    assert node.navigation_store.load() is None
    assert node.navigation_control.snapshot("site_b")["state"] == "idle"


def test_state_endpoint_reports_paused_navigation_controls(tmp_path):
    node = _navigation_node(tmp_path, record=_paused_record())

    status, body = _serve_and_request(node, "/api/state")

    assert status == 200
    assert body["navigation_control"]["state"] == "paused"
    assert body["navigation_control"]["can_resume"] is True
    assert body["navigation_control"]["can_clear"] is True
    assert body["navigation_control"]["can_pause"] is False


def test_pause_endpoint_returns_409_when_no_active_goal(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=("_cancel_all_goals", "pause_navigation"),
    )

    status, body = _serve_and_request(node, "/api/navigation/pause", {}, "POST")

    assert status == 409
    assert body["ok"] is False
    assert node.cancel_goal_client.requests == []


def test_clear_endpoint_returns_200_for_paused_goal(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("_cancel_all_goals", "clear_navigation"),
    )

    status, body = _serve_and_request(node, "/api/navigation/clear", {}, "POST")

    assert status == 200
    assert body["navigation_control"]["state"] == "idle"
    assert body["navigation_control"]["paused_goal"] is None


def test_pause_endpoint_returns_504_on_timeout(tmp_path, monkeypatch):
    node = _navigation_node(
        tmp_path,
        client=_FakeCancelClient(ready=True, future=_FakeFuture()),
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)
    monkeypatch.setattr(web_control_node, "CANCEL_GOAL_TIMEOUT_SEC", 0.0)
    node._spawn_cancel_retry = lambda kind, token: None

    status, body = _serve_and_request(node, "/api/navigation/pause", {}, "POST")

    assert status == 504
    assert body["ok"] is False
    assert node.navigation_control.snapshot("site_a")["state"] == "pausing"
    assert node.navigation_store.load()["phase"] == "pausing"


def test_pause_endpoint_returns_503_when_service_missing(tmp_path):
    node = _navigation_node(
        tmp_path,
        ready=False,
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)

    status, body = _serve_and_request(node, "/api/navigation/pause", {}, "POST")

    assert status == 503
    assert body["ok"] is False


def test_goal_endpoint_rejected_while_pausing(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "pause_navigation",
            "publish_goal",
        ),
    )
    _arm_active_goal(node)
    node.pause_navigation()

    status, body = _serve_and_request(
        node, "/api/goal", {"x": 3.0, "y": 4.0, "yaw": 0.0}, "POST"
    )

    assert status == 409
    assert body["ok"] is False


def test_resume_endpoint_republishes_paused_goal(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("state_payload", "_publish_goal_pose", "resume_navigation"),
    )

    status, body = _serve_and_request(node, "/api/navigation/resume", {}, "POST")

    assert status == 200
    assert body["ok"] is True
    assert body["navigation_control"]["state"] == "resuming"
    assert node.goal_publications[-1].pose.position.x == 1.5
    assert node.navigation_store.load()["phase"] == "resuming"


def test_clear_endpoint_returns_409_while_resuming(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=(
            "state_payload",
            "_publish_goal_pose",
            "resume_navigation",
            "clear_navigation",
        ),
    )
    node.resume_navigation()

    status, body = _serve_and_request(node, "/api/navigation/clear", {}, "POST")

    assert status == 409
    assert body["ok"] is False
    assert node.navigation_control.snapshot("site_a")["state"] == "resuming"
    assert node.navigation_store.load()["phase"] == "resuming"


# --- Regressions for the F1-F7 navigation transition hardening -------------


def test_clear_accept_before_empty_retries_once_to_idle(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "state_payload",
            "_publish_goal_pose",
            "publish_goal",
            "clear_navigation",
            "action_status_callback",
            "_retry_startup_cancel",
        ),
    )
    node.publish_goal({"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0})
    assert node.navigation_control.snapshot("site_a")["state"] == "navigating"

    calls = []

    def scripted_cancel():
        calls.append(1)
        if len(calls) == 1:
            # The accepted status lands while the HTTP thread is blocked inside
            # CancelGoal, before the empty result is returned.
            node.action_status_callback(_GoalStatusArray([_GoalStatus(1, 9)]))
            return "empty"
        return "accepted"

    node._cancel_all_goals = scripted_cancel
    spawned = []
    node._spawn_cancel_retry = lambda kind, token: spawned.append((kind, token))

    response = node.clear_navigation()

    assert response["navigation_control"]["state"] == "clearing"
    assert len(spawned) == 1
    assert spawned[0][0] == PENDING_CLEAR
    assert len(calls) == 1

    # Exactly one retry is sent, and its accepted result keeps clearing.
    node._retry_startup_cancel(True, spawned[0][1])
    assert len(calls) == 2
    assert len(spawned) == 1

    node.action_status_callback(_GoalStatusArray([_GoalStatus(5, 9)]))
    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "idle"
    assert snapshot["has_goal"] is False
    assert node.published_goal is None


def test_stale_pause_retry_blocks_publish_and_resume_then_skips(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(phase="pausing"),
        methods=(
            "state_payload",
            "_publish_goal_pose",
            "_cancel_all_goals",
            "publish_goal",
            "resume_navigation",
            "action_status_callback",
            "_retry_startup_cancel",
        ),
    )
    spawned = []
    node._spawn_cancel_retry = lambda kind, token: spawned.append((kind, token))

    # Startup sees an active goal and schedules the interrupted Pause retry.
    node.action_status_callback(_GoalStatusArray([_GoalStatus(2, 7)]))
    assert len(spawned) == 1
    token = spawned[0][1]
    assert node.navigation_control.retry_lease_active() is True

    # The old goal terminates before the delayed retry thread runs.
    node.action_status_callback(_GoalStatusArray([_GoalStatus(5, 7)]))
    assert node.navigation_control.snapshot("site_a")["state"] == "paused"

    # Operator resume/new publish are blocked until the lease is released.
    with pytest.raises(web_control_node.NavigationConflictError):
        node.resume_navigation()
    with pytest.raises(web_control_node.NavigationConflictError):
        node.publish_goal({"x": 5.0, "y": 6.0, "z": 0.0, "yaw": 0.0})
    assert node.goal_publications == []

    # The delayed retry revalidates, skips, and must not cancel the new goal.
    before = len(node.cancel_goal_client.requests)
    node._retry_startup_cancel(False, token)
    assert len(node.cancel_goal_client.requests) == before
    assert node.navigation_control.retry_lease_active() is False

    # Resume works once the stale lease is gone.
    node.resume_navigation()
    assert node.navigation_control.snapshot("site_a")["state"] == "resuming"


def test_pause_timeout_then_clear_issues_fresh_cancel(tmp_path, monkeypatch):
    node = _navigation_node(
        tmp_path,
        client=_FakeCancelClient(ready=True, future=_FakeFuture()),
        methods=(
            "state_payload",
            "_publish_goal_pose",
            "_cancel_all_goals",
            "pause_navigation",
            "clear_navigation",
        ),
    )
    _arm_active_goal(node)
    monkeypatch.setattr(web_control_node, "CANCEL_GOAL_TIMEOUT_SEC", 0.0)
    spawned = []
    node._spawn_cancel_retry = lambda kind, token: spawned.append((kind, token))

    with pytest.raises(web_control_node.NavigationTimeoutError):
        node.pause_navigation()

    assert node.navigation_control.snapshot("site_a")["state"] == "pausing"
    assert node.navigation_store.load()["phase"] == "pausing"
    assert len(spawned) == 1

    # Clear during the timed-out Pause sends a fresh cancel rather than reusing
    # a cancellation that no longer has a tracked completion.
    response = node.clear_navigation()

    assert response["navigation_control"]["state"] == "clearing"
    assert len(node.cancel_goal_client.requests) == 2

    node.navigation_control.note_action_status("canceled", 5, 12.0)
    assert node.navigation_control.snapshot("site_a")["state"] == "idle"


def test_clear_timeout_background_retry_recovers_to_navigating(tmp_path, monkeypatch):
    client = _FakeCancelClient(ready=True, future=_FakeFuture())
    node = _navigation_node(
        tmp_path,
        client=client,
        methods=(
            "state_payload",
            "_publish_goal_pose",
            "_cancel_all_goals",
            "publish_goal",
            "clear_navigation",
            "_retry_startup_cancel",
        ),
    )
    _arm_active_goal(node)
    monkeypatch.setattr(web_control_node, "CANCEL_GOAL_TIMEOUT_SEC", 0.0)
    spawned = []
    node._spawn_cancel_retry = lambda kind, token: spawned.append((kind, token))

    with pytest.raises(web_control_node.NavigationTimeoutError):
        node.clear_navigation()

    assert node.navigation_control.snapshot("site_a")["state"] == "clearing"
    assert spawned and spawned[0][0] == PENDING_CLEAR

    # Let the bounded retry report that nothing is canceling; the state must
    # become truthfully Navigating so the browser can Clear again.
    client.future.done_flag = True
    client.results = ()
    node._retry_startup_cancel(True, spawned[0][1])

    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "navigating"
    assert snapshot["has_goal"] is True
    assert snapshot["can_clear"] is True


@pytest.mark.parametrize("retry_result", ["accepted", "empty"])
def test_pause_timeout_after_concurrent_clear_retries_pending_clear(
    tmp_path, retry_result
):
    """A timeout after Clear overtook Pause must retry the stored Clear intent.

    Ordering: active -> Pause's CancelGoal is in flight -> Clear flips the
    pending intent to CLEAR and reuses that request -> the Pause call times out.
    The timeout must arm and retry the kind actually stored in the lease, not
    the Pause the caller assumed; a hardcoded Pause makes ``validate_retry``
    reject the background thread and leaves the state stuck clearing forever.
    """
    node = _navigation_node(
        tmp_path,
        methods=(
            "state_payload",
            "pause_navigation",
            "clear_navigation",
            "action_status_callback",
            "_retry_startup_cancel",
        ),
    )
    _arm_active_goal(node)
    spawned = []
    calls = []

    def scripted_cancel():
        calls.append(1)
        if len(calls) == 1:
            # Clear overtakes the in-flight Pause cancel and reuses it.
            response = node.clear_navigation()
            assert response["navigation_control"]["state"] == "clearing"
            raise web_control_node.NavigationTimeoutError(
                "cancel goal request timed out"
            )
        return retry_result

    node._cancel_all_goals = scripted_cancel
    node._spawn_cancel_retry = lambda kind, token: spawned.append((kind, token))

    with pytest.raises(web_control_node.NavigationTimeoutError):
        node.pause_navigation()

    # The armed lease stores the Clear intent, so the background retry must be
    # the Clear retry (kind -> clear=True) and must really send a cancel.
    assert len(spawned) == 1
    kind, token = spawned[0]
    assert kind == PENDING_CLEAR
    node._retry_startup_cancel(kind == PENDING_CLEAR, token)
    assert len(calls) == 2

    snapshot = node.state_payload()["navigation_control"]
    if retry_result == "accepted":
        # An accepted cancel stays clearing until the terminal status settles it.
        assert snapshot["state"] == "clearing"
        node.action_status_callback(_GoalStatusArray([_GoalStatus(5, 9)]))
        snapshot = node.state_payload()["navigation_control"]
        assert snapshot["state"] == "idle"
        assert snapshot["has_goal"] is False
    else:
        # A retry that matched no goal recovers to a manageable Navigating.
        assert snapshot["state"] == "navigating"
        assert snapshot["has_goal"] is True
        assert snapshot["can_clear"] is True


def test_restored_pausing_clear_before_first_status_issues_cancel(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(phase="pausing"),
        methods=(
            "state_payload",
            "_cancel_all_goals",
            "clear_navigation",
            "action_status_callback",
        ),
    )

    response = node.clear_navigation()

    assert response["navigation_control"]["state"] == "clearing"
    assert len(node.cancel_goal_client.requests) == 1

    # The startup reconciliation must not overwrite the explicit Clear.
    node.action_status_callback(_EmptyGoalStatusArray())
    assert node.navigation_control.snapshot("site_a")["state"] == "clearing"


def test_restored_paused_resume_before_status_does_not_double_publish(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=(
            "state_payload",
            "_publish_goal_pose",
            "resume_navigation",
            "action_status_callback",
        ),
    )

    node.resume_navigation()
    assert len(node.goal_publications) == 1
    assert node.navigation_control.snapshot("site_a")["state"] == "resuming"

    # A first status-less update must not revert the resume and invite a second
    # publish.
    node.action_status_callback(_EmptyGoalStatusArray())
    assert node.navigation_control.snapshot("site_a")["state"] == "resuming"
    assert len(node.goal_publications) == 1


def test_resume_publish_failure_restores_paused_and_ignores_stale_terminal(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=(
            "state_payload",
            "_publish_goal_pose",
            "resume_navigation",
            "action_status_callback",
        ),
    )

    def failing_publish(pose, frame=None):
        raise RuntimeError("publish failed")

    node._publish_goal_pose = failing_publish

    with pytest.raises(web_control_node.NavigationUnavailableError):
        node.resume_navigation()

    assert node.navigation_store.load()["phase"] == "paused"

    # A stale terminal must not delete the paused record after the reset.
    node.action_status_callback(_GoalStatusArray([_GoalStatus(6, 9)]))
    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "paused"
    assert snapshot["can_resume"] is True
    assert node.navigation_store.load()["phase"] == "paused"


def test_goal_publish_status_during_publish_is_not_floor_filtered(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "state_payload",
            "_publish_goal_pose",
            "publish_goal",
            "action_status_callback",
        ),
    )
    # An older active goal at stamp 5 makes the new goal's accepted genuinely
    # newer; the bug armed the floor with that same newer stamp after publish.
    node.navigation_control.record_goal_published(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    node.navigation_control.note_action_status("executing", 5, 10.0)
    node.published_goal = {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}

    def publishing(pose, frame=None):
        # Accepted for the new goal is processed inside the ROS publish, before
        # the state records the goal.
        node.action_status_callback(_GoalStatusArray([_GoalStatus(1, 9)]))

    node._publish_goal_pose = publishing

    node.publish_goal({"x": 5.0, "y": 6.0, "z": 0.0, "yaw": 1.0})

    node.action_status_callback(_GoalStatusArray([_GoalStatus(2, 9)]))
    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "navigating"
    assert snapshot["can_pause"] is True

    node.action_status_callback(_GoalStatusArray([_GoalStatus(4, 9)]))
    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "idle"


_NAVIGATION_PANEL_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("// --- Navigation pause/resume controls");
const end = source.indexOf("// --- End navigation pause/resume controls");
if (start < 0 || end < 0) throw new Error("navigation control block not found");
const block = source.slice(start, end);
const publishStart = source.indexOf("function updatePublishPoseButton()");
if (publishStart < 0) throw new Error("updatePublishPoseButton not found");
const publishEnd = source.indexOf("\n}", publishStart);
if (publishEnd < 0) throw new Error("updatePublishPoseButton end not found");
const publishBlock = source.slice(publishStart, publishEnd + 2);

function makeElement() {
  return { textContent: "", disabled: false };
}
const elements = {
  navigationPrimary: makeElement(),
  navigationClear: makeElement(),
  navigationControlState: makeElement(),
  publishResult: makeElement(),
  publishPose: makeElement(),
};
global.document = {
  getElementById(id) { return elements[id] || null; },
};

let state = { mode: "goal", navigationControl: null, lastGoal: null };
let clearedScenes = 0;
let poseUpdates = 0;
let markerUpdates = 0;
let polled = 0;
let captured = null;

function clearNavigationSceneState() { clearedScenes += 1; state.lastGoal = null; }
function updateTargetPoseDisplay() { poseUpdates += 1; }
function updateTargetMarkers() { markerUpdates += 1; }
function pollState() { polled += 1; }
global.postJson = async (url, payload) => {
  captured = { url, payload };
  return {
    ok: true,
    message: "pause requested",
    stamp: 100.0,
    navigation_control: {
      state: "pausing", has_goal: true, can_pause: false,
      can_resume: false, can_clear: true, paused_goal: null,
    },
  };
};

eval(publishBlock + "\n" + block);

function expect(condition, message) { if (!condition) throw new Error(message); }

updateNavigationControls({ state: "navigating", has_goal: true, can_pause: true, can_resume: false, can_clear: true, paused_goal: null });
expect(elements.navigationPrimary.textContent === "Pause Navigation", "navigating primary label");
expect(elements.navigationPrimary.disabled === false, "navigating primary enabled");
expect(elements.navigationClear.disabled === false, "navigating clear enabled");
expect(elements.navigationControlState.textContent === "Navigating", "navigating state label");
expect(elements.publishPose.disabled === false, "navigating publish enabled");

updateNavigationControls({ state: "pausing", has_goal: true, can_pause: false, can_resume: false, can_clear: true, paused_goal: null });
expect(elements.navigationPrimary.textContent === "Pausing...", "pausing label");
expect(elements.navigationPrimary.disabled === true, "pausing disabled");
expect(elements.publishPose.disabled === true, "pausing publish disabled");

updateNavigationControls({ state: "paused", has_goal: true, can_pause: false, can_resume: true, can_clear: true, paused_goal: { x: 1, y: 2, z: 0, yaw: 0.5 } });
expect(elements.navigationPrimary.textContent === "Resume Navigation", "paused label");
expect(elements.navigationPrimary.disabled === false, "paused primary enabled");
expect(elements.publishPose.disabled === false, "paused publish enabled");
expect(state.lastGoal && state.lastGoal.x === 1, "paused destination not shown");
expect(poseUpdates > 0 && markerUpdates > 0, "paused destination not rendered");

updateNavigationControls({ state: "resuming", has_goal: true, can_pause: false, can_resume: false, can_clear: false, paused_goal: null });
expect(elements.navigationPrimary.textContent === "Resuming...", "resuming label");
expect(elements.navigationPrimary.disabled === true, "resuming disabled");
expect(elements.publishPose.disabled === true, "resuming publish disabled");

const before = clearedScenes;
updateNavigationControls({ state: "clearing", has_goal: true, can_pause: false, can_resume: false, can_clear: false, paused_goal: null });
expect(elements.navigationClear.textContent === "Clearing...", "clearing label");
expect(elements.navigationClear.disabled === true, "clearing disabled");
expect(elements.publishPose.disabled === true, "clearing publish disabled");

updateNavigationControls({ state: "idle", has_goal: false, can_pause: false, can_resume: false, can_clear: false, paused_goal: null });
expect(clearedScenes > before, "terminal cleanup not run");
expect(elements.navigationPrimary.textContent === "Pause Navigation", "idle primary label");
expect(elements.navigationPrimary.disabled === true, "idle primary disabled");
expect(elements.navigationClear.disabled === true, "idle clear disabled");
expect(elements.publishPose.disabled === false, "idle publish enabled");

(async () => {
  await postNavigationCommand("/api/navigation/pause", "Pause failed");
  expect(captured && captured.url === "/api/navigation/pause", "command url wrong");
  expect(elements.navigationControlState.textContent === "Pausing", "command response not applied");
  expect(elements.publishPose.disabled === true, "command response did not disable publish");
  expect(polled > 0, "poll after command missing");
  console.log("navigation control behavior ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""


def test_navigation_page_has_pause_resume_controls():
    index_html = (
        Path(__file__).resolve().parents[1] / "static" / "index.html"
    ).read_text(encoding="utf-8")

    assert 'id="navigationPrimary"' in index_html
    assert 'id="navigationClear"' in index_html
    assert 'id="navigationControlState"' in index_html


def test_frontend_navigation_controls_behavior_executes_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _NAVIGATION_PANEL_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "navigation control behavior ok" in completed.stdout


def test_readme_documents_navigation_pause_and_cancel_service():
    readme = (Path(__file__).resolve().parents[1] / "README.md").read_text(
        encoding="utf-8"
    )

    assert "Pause Navigation" in readme
    assert "Resume Navigation" in readme
    assert "paused_navigation.json" in readme
    assert "cancel_goal_service" in readme


class _PathMessageStub:
    def __init__(self, points, frame_id="map"):
        self.header = SimpleNamespace(frame_id=frame_id)
        self.poses = [
            SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=x, y=y, z=z)
                )
            )
            for x, y, z in points
        ]


def _path_callback_node():
    class _PathCallbackNode:
        def __init__(self):
            self.lock = threading.RLock()
            self.paths = {"nav": SourceCache("nav", 2.0)}
            self.mapping_path = SourceCache("mapping_path", 2.0)

        def now_sec(self):
            return 10.0

    node = _PathCallbackNode()
    node.path_callback = MethodType(
        web_control_node.WebControlNode.path_callback, node
    )
    node.mapping_path_callback = MethodType(
        web_control_node.WebControlNode.mapping_path_callback, node
    )
    return node


def test_path_callbacks_preserve_z():
    node = _path_callback_node()

    node.path_callback(
        "nav", _PathMessageStub([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
    )
    node.mapping_path_callback(_PathMessageStub([(7.0, 8.0, 9.0)]))

    assert node.paths["nav"].data == {"points": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]}
    assert node.mapping_path.data == {"points": [[7.0, 8.0, 9.0]]}


_PATH_HEIGHT_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
function jsFunctionBody(signature) {
  const start = source.indexOf(signature);
  if (start < 0) throw new Error("missing function: " + signature);
  const brace = source.indexOf("{", start);
  let depth = 0;
  for (let index = brace; index < source.length; index += 1) {
    if (source[index] === "{") depth += 1;
    else if (source[index] === "}") {
      depth -= 1;
      if (depth === 0) return source.slice(start, index + 1);
    }
  }
  throw new Error("unterminated function: " + signature);
}
function rosToThree(x, y, z) { return { x, y, z }; }
eval(jsFunctionBody("function pathPointToThree"));
const upper = pathPointToThree([1, 2, 3]);
if (Math.abs(upper.z - 3.08) > 1e-9) throw new Error("3D path height lost");
const legacy = pathPointToThree([1, 2]);
if (Math.abs(legacy.z - 0.08) > 1e-9) throw new Error("legacy path fallback wrong");
console.log("frontend path height ok");
"""


def test_frontend_path_height_uses_ros_z():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js_path = Path(__file__).resolve().parents[1] / "static" / "app.js"
    app_js = app_js_path.read_text(encoding="utf-8")
    completed = subprocess.run(
        ["node", "-e", _PATH_HEIGHT_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js_path)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "frontend path height ok" in completed.stdout

    assert "pathPointToThree(point)" in _js_function_body(app_js, "function updatePaths")
    assert "pathPointToThree(point)" in _js_function_body(
        app_js, "function updateMappingPath"
    )


class _MutableClock:
    def __init__(self, now: float) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class _FakeVector3:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _FakeTwist:
    def __init__(self) -> None:
        self.linear = _FakeVector3()
        self.angular = _FakeVector3()


def _teleop_node(clock=None, published=None, **overrides):
    parameters = {
        "max_linear_speed": 1.5,
        "wheelbase": 0.65,
        "max_steer_angle": 0.461,
        "steering_exponent": 0.5,
        "deadzone": 0.08,
        "command_timeout": 0.4,
        "publish_rate": 20.0,
    }
    parameters.update(overrides)
    node = type("_TeleopNodeStub", (), {})()
    node.now_sec = lambda: 10.0
    node.get_logger = lambda: _LoggerStub()
    node.teleop = TeleopController(clock=clock or (lambda: 0.0), **parameters)
    node.cmd_vel_publications = published if published is not None else []
    node.cmd_vel_pub = _FakePublisher(node.cmd_vel_publications)
    for name in (
        "apply_teleop_command",
        "publish_teleop_twist",
        "teleop_timer_callback",
        "cleanup_teleop",
    ):
        setattr(
            node,
            name,
            MethodType(getattr(web_control_node.WebControlNode, name), node),
        )
    return node


def test_teleop_handler_accepts_command_and_returns_mapped_twist():
    node = _teleop_node()

    status, body = _serve_and_request(
        node,
        "/api/teleop",
        {
            "throttle": 0.5,
            "steering": 0.25,
            "client_id": "tab-session",
            "seq": 1,
        },
        "POST",
    )

    assert status == 200
    assert body["ok"] is True
    assert body["stale"] is False
    assert body["teleop"]["throttle"] == 0.5
    assert body["teleop"]["steering"] == 0.25
    assert body["teleop"]["linear_x"] == pytest.approx(0.75)
    assert body["teleop"]["active"] is True
    assert node.cmd_vel_publications == []


def test_teleop_handler_rejects_invalid_payloads_without_replacing_command():
    node = _teleop_node()
    node.teleop.apply_command(
        {"throttle": 0.5, "steering": 0.0, "client_id": "tab-session", "seq": 1},
        0.0,
    )

    for payload in (
        [],
        {},
        {"throttle": "fast", "steering": 0.0, "client_id": "tab-session", "seq": 2},
        {"throttle": 0.5},
        {
            "throttle": float("nan"),
            "steering": 0.0,
            "client_id": "tab-session",
            "seq": 2,
        },
    ):
        status, body = _serve_and_request(node, "/api/teleop", payload, "POST")
        assert status == 400, payload
        assert body["ok"] is False

    command = node.teleop.poll_publish(0.0)
    assert command is not None
    assert command["linear_x"] == pytest.approx(0.75)


def test_teleop_handler_ignores_stale_commands_after_newer_zero():
    node = _teleop_node()

    status, body = _serve_and_request(
        node,
        "/api/teleop",
        {"throttle": 0.0, "steering": 0.0, "client_id": "tab-session", "seq": 2},
        "POST",
    )
    assert status == 200
    assert body["ok"] is True
    assert body["stale"] is False

    status, body = _serve_and_request(
        node,
        "/api/teleop",
        {"throttle": 1.0, "steering": 0.0, "client_id": "tab-session", "seq": 1},
        "POST",
    )
    assert status == 200
    assert body["ok"] is True
    assert body["stale"] is True
    assert "teleop" not in body

    zero = node.teleop.poll_publish(0.0)
    assert zero is not None
    assert zero["linear_x"] == 0.0
    assert node.teleop.poll_publish(0.0) is None


def test_teleop_handler_rejects_invalid_client_identity():
    node = _teleop_node()

    for payload in (
        {"throttle": 0.5, "steering": 0.0},
        {"throttle": 0.5, "steering": 0.0, "client_id": "", "seq": 1},
        {"throttle": 0.5, "steering": 0.0, "client_id": "tab", "seq": 0},
        {"throttle": 0.5, "steering": 0.0, "client_id": "tab", "seq": True},
    ):
        status, body = _serve_and_request(node, "/api/teleop", payload, "POST")
        assert status == 400, payload
        assert body["ok"] is False

    assert node.teleop.poll_publish(0.0) is None


def test_teleop_timer_publishes_active_command_then_exactly_one_zero(monkeypatch):
    monkeypatch.setattr(web_control_node, "Twist", _FakeTwist)
    clock = _MutableClock(0.0)
    node = _teleop_node(clock=clock)

    node.teleop_timer_callback()
    assert node.cmd_vel_publications == []

    node.teleop.apply_command(
        {"throttle": 1.0, "steering": 0.0, "client_id": "tab-session", "seq": 1}, 0.0
    )
    node.teleop_timer_callback()
    assert len(node.cmd_vel_publications) == 1
    assert node.cmd_vel_publications[0].linear.x == pytest.approx(1.5)
    assert node.cmd_vel_publications[0].angular.z == pytest.approx(0.0)

    clock.now = 0.400001
    node.teleop_timer_callback()
    assert len(node.cmd_vel_publications) == 2
    assert node.cmd_vel_publications[1].linear.x == 0.0

    clock.now = 0.5
    node.teleop_timer_callback()
    assert len(node.cmd_vel_publications) == 2


def test_teleop_timer_publishes_nonzero_steering_angular_z(monkeypatch):
    monkeypatch.setattr(web_control_node, "Twist", _FakeTwist)
    node = _teleop_node(clock=_MutableClock(0.0))

    node.teleop.apply_command(
        {"throttle": 0.6, "steering": 0.25, "client_id": "tab-session", "seq": 1}, 0.0
    )
    node.teleop_timer_callback()

    assert len(node.cmd_vel_publications) == 1
    expected_angle = 0.25 ** 0.5 * 0.461
    twist = node.cmd_vel_publications[0]
    assert twist.linear.x == pytest.approx(0.9)
    assert twist.angular.z == pytest.approx(0.9 / 0.65 * math.tan(expected_angle))
    assert twist.angular.z > 0.0

    node.teleop.apply_command(
        {"throttle": 0.6, "steering": -0.25, "client_id": "tab-session", "seq": 2}, 0.0
    )
    node.teleop_timer_callback()
    assert node.cmd_vel_publications[1].angular.z < 0.0


def test_teleop_cleanup_publishes_exactly_one_zero_on_shutdown(monkeypatch):
    monkeypatch.setattr(web_control_node, "Twist", _FakeTwist)
    node = _teleop_node()
    node.teleop.apply_command(
        {"throttle": 0.5, "steering": 0.0, "client_id": "tab-session", "seq": 1}, 0.0
    )

    node.cleanup_teleop()

    assert len(node.cmd_vel_publications) == 1
    assert node.cmd_vel_publications[0].linear.x == 0.0
    assert node.cmd_vel_publications[0].angular.z == 0.0
    assert node.teleop.poll_publish(0.0) is None


def test_teleop_cleanup_is_idempotent_and_stop_is_terminal(monkeypatch):
    monkeypatch.setattr(web_control_node, "Twist", _FakeTwist)
    node = _teleop_node()
    node.teleop.apply_command(
        {"throttle": 0.5, "steering": 0.0, "client_id": "tab-session", "seq": 1}, 0.0
    )

    node.cleanup_teleop()
    node.cleanup_teleop()

    assert len(node.cmd_vel_publications) == 1
    assert node.cmd_vel_publications[0].linear.x == 0.0
    with pytest.raises(RuntimeError):
        node.apply_teleop_command(
            {
                "throttle": 1.0,
                "steering": 0.0,
                "client_id": "tab-session",
                "seq": 2,
            }
        )


def test_teleop_cleanup_swallows_twist_publish_failure(monkeypatch):
    monkeypatch.setattr(web_control_node, "Twist", _FakeTwist)
    node = _teleop_node()
    node.teleop.apply_command(
        {"throttle": 0.5, "steering": 0.0, "client_id": "tab-session", "seq": 1}, 0.0
    )
    errors = []

    class _RecordingLogger(_LoggerStub):
        def error(self, message, *args, **kwargs):
            errors.append(message)

    class _RaisingPublisher:
        def publish(self, msg):
            raise RuntimeError("cmd_vel publish failed")

    node.get_logger = lambda: _RecordingLogger()
    node.cmd_vel_pub = _RaisingPublisher()

    node.cleanup_teleop()  # must not raise so destroy_node can continue

    assert errors
    assert "cmd_vel publish failed" in errors[0]


def test_web_control_yaml_declares_teleop_defaults():
    package_root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load(
        (package_root / "config" / "web_control.yaml").read_text(encoding="utf-8")
    )
    parameters = config["web_control"]["ros__parameters"]

    assert parameters["teleop_cmd_vel_topic"] == "/cmd_vel"
    assert parameters["teleop_max_linear_speed"] == 1.5
    assert parameters["teleop_wheelbase"] == 0.65
    assert parameters["teleop_max_steer_angle"] == 0.461
    assert parameters["teleop_steering_exponent"] == 0.5
    assert parameters["teleop_deadzone"] == 0.08
    assert parameters["teleop_command_timeout"] == 0.4
    assert parameters["teleop_publish_rate"] == 20.0


def test_mapping_joystick_is_a_mapping_only_sibling_overlay():
    index_html, app_js, styles_css = _frontend_sources()

    view_start = index_html.index('id="view3d"')
    view_end = index_html.index('id="viewSystem"', view_start)
    viewport = index_html[view_start:view_end]

    assert 'id="mapCanvas3d"' in viewport
    assert 'id="mappingJoystick" class="joystick" hidden' in viewport
    assert viewport.index('id="mapCanvas3d"') < viewport.index('id="mappingJoystick"')
    assert 'id="joystickBase"' in viewport
    assert 'id="joystickKnob"' in viewport
    assert 'id="joystickStatus"' in viewport

    assert 'setMappingJoystickVisible(tab === "mapping")' in app_js
    assert ".joystick {" in styles_css
    assert "position: absolute;" in styles_css
    assert "--joystick-size" in styles_css
    assert ".stage.hud-lower-right .hud" in styles_css
    assert 'id="joystickSettings"' in viewport
    assert 'id="joystickMoveHandle"' in viewport
    assert 'id="joystickResizeHandle"' in viewport


_MAPPING_JOYSTICK_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Mapping joystick");
const blockEnd = source.indexOf("// --- End mapping joystick");
if (blockStart < 0 || blockEnd < 0) throw new Error("mapping joystick block not found");
const block = source.slice(blockStart, blockEnd);

function makeElement() {
  return {
    hidden: false,
    textContent: "",
    className: "",
    style: {},
    captured: [],
    listeners: {},
    classList: {
      values: new Set(),
      toggle(name, on) {
        if (on) this.values.add(name);
        else this.values.delete(name);
      },
      contains(name) { return this.values.has(name); },
    },
    setPointerCapture(id) { this.captured.push(id); },
    addEventListener(name, handler) { this.listeners[name] = handler; },
    getBoundingClientRect() { return { left: 0, top: 0, width: 100, height: 100 }; },
  };
}

const elements = {
  mappingJoystick: makeElement(),
  joystickKnob: makeElement(),
  joystickStatus: makeElement(),
};
const stage = makeElement();
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
const intervalHandlers = new Map();
let nextIntervalId = 1;
global.window = {
  addEventListener() {},
  setInterval(handler) {
    const id = nextIntervalId;
    nextIntervalId += 1;
    intervalHandlers.set(id, handler);
    return id;
  },
  clearInterval(id) { intervalHandlers.delete(id); },
};

const beacons = [];
try {
  Object.defineProperty(globalThis, "navigator", {
    value: { sendBeacon(url, body) { beacons.push({ url, body }); return true; } },
    configurable: true,
  });
} catch (error) {
  globalThis.navigator.sendBeacon = (url, body) => {
    beacons.push({ url, body });
    return true;
  };
}

let fakeNow = 0;
Date.now = () => fakeNow;

global.viewerSessionId = () => "tab-session";

const posts = [];
const deferred = [];
let deferRequests = false;
let failNextRequest = false;
global.postJson = async (url, payload) => {
  posts.push({ url, payload });
  if (failNextRequest) {
    failNextRequest = false;
    throw new Error("network down");
  }
  const active = payload.throttle !== 0 || payload.steering !== 0;
  if (deferRequests) {
    return await new Promise((resolve, reject) => {
      deferred.push({ resolve, reject, payload, active });
    });
  }
  return { ok: true, teleop: { active } };
};
global.fetchWithTimeout = async () => ({ ok: true });

eval(block + "\nglobalThis.__teleopState = teleop;");
const teleopState = globalThis.__teleopState;

function expect(condition, message) { if (!condition) throw new Error(message); }

expect(teleopState.clientId === "tab-session", "joystick reuses viewerSessionId for client_id");

const baseElement = makeElement();
let propagationStops = 0;
function makeEvent(pointerId, clientX, clientY) {
  return {
    pointerId,
    pointerType: "mouse",
    clientX,
    clientY,
    currentTarget: baseElement,
    preventDefault() {},
    stopPropagation() { propagationStops += 1; },
  };
}

const radius = 50;
const upLeft = joystickAxes(-40, -40, radius);
expect(upLeft.throttle > 0 && upLeft.steering > 0, "upper-left is forward and left");
const upRight = joystickAxes(40, -40, radius);
expect(upRight.throttle > 0 && upRight.steering < 0, "upper-right is forward and right");
const downLeft = joystickAxes(-40, 40, radius);
expect(downLeft.throttle < 0 && downLeft.steering > 0, "lower-left is reverse and left");
const downRight = joystickAxes(40, 40, radius);
expect(downRight.throttle < 0 && downRight.steering < 0, "lower-right is reverse and right");
const horizontal = joystickAxes(-40, 0, radius);
expect(horizontal.throttle === 0 && horizontal.steering > 0, "horizontal drag has zero throttle");
const centered = joystickAxes(2, -2, radius);
expect(centered.throttle === 0 && centered.steering === 0, "dead zone centers the axes");
const clamped = joystickAxes(-100, -100, radius);
expect(clamped.throttle === 1 && clamped.steering === 1, "offsets clamp to the unit circle");

(async () => {
  fakeNow = 0;
  await joystickPointerDown(makeEvent(1, 30, 30));
  expect(posts.length === 1, "pointerdown sends the first command");
  expect(posts[0].url === "/api/teleop", "pointerdown posts to /api/teleop");
  expect(posts[0].payload.client_id === teleopState.clientId, "active send carries the tab client_id");
  expect(Number.isInteger(posts[0].payload.seq) && posts[0].payload.seq > 0, "active send carries a positive integer seq");
  expect(posts[0].payload.throttle > 0 && posts[0].payload.steering > 0, "pointerdown posts the upper-left axes");
  expect(baseElement.captured.length === 1, "pointerdown captures the active pointer");
  expect(propagationStops >= 1, "pointerdown stops propagation to the map canvas");
  expect(elements.joystickKnob.style.transform === "translate(-20px, -20px)", "knob follows the pointer");
  expect(intervalHandlers.size === 1, "pointerdown starts one fixed-rate send loop");

  fakeNow = 100;
  await Array.from(intervalHandlers.values())[0]();
  expect(posts.length === 2, "stationary hold continues sending at 10 Hz");

  fakeNow = 150;
  await joystickPointerMove(makeEvent(1, 50, 50));
  expect(posts.length === 3, "returning to center sends one immediate zero");
  expect(posts[2].payload.throttle === 0, "center transition posts zero throttle");

  fakeNow = 250;
  await Array.from(intervalHandlers.values())[0]();
  expect(posts.length === 3, "center hold does not stream zero commands");

  await joystickPointerMove(makeEvent(1, 20, 20));
  expect(posts.length === 4, "leaving center sends one immediate active command");

  fakeNow = 300;
  await joystickPointerMove(makeEvent(1, 10, 10));
  expect(posts.length === 4, "active pointermove is throttled within 100 ms");

  fakeNow = 350;
  await Array.from(intervalHandlers.values())[0]();
  expect(posts.length === 5, "active hold sends again after 100 ms");

  fakeNow = 400;
  await joystickPointerMove(makeEvent(1, 20, 20));
  expect(posts.length === 5, "active pointermove shares the send throttle");

  const beforeSecondPointer = posts.length;
  await joystickPointerDown(makeEvent(2, 80, 80));
  expect(baseElement.captured.length === 1, "second pointer is not captured");
  expect(posts.length === beforeSecondPointer, "second pointer does not take control");

  fakeNow = 450;
  await joystickPointerUp(makeEvent(1, 20, 20));
  expect(posts[posts.length - 1].payload.throttle === 0, "release sends a zero throttle");
  expect(posts[posts.length - 1].payload.steering === 0, "release sends a zero steering");
  expect(posts[posts.length - 1].payload.client_id === posts[0].payload.client_id, "stop shares the active client_id");
  expect(posts[posts.length - 1].payload.seq > posts[0].payload.seq, "stop seq is higher than the active seq");
  expect(elements.joystickKnob.style.transform === "translate(0px, 0px)", "release centers the knob");
  expect(intervalHandlers.size === 0, "release clears the send loop");

  fakeNow = 300;
  await joystickPointerDown(makeEvent(3, -10, 90));
  fakeNow = 400;
  await joystickPointerCancel(makeEvent(3, -10, 90));
  expect(posts[posts.length - 1].payload.throttle === 0, "cancel sends a zero command");

  fakeNow = 500;
  await joystickPointerDown(makeEvent(4, 10, 10));
  fakeNow = 600;
  await handleJoystickBlur();
  expect(posts[posts.length - 1].payload.throttle === 0, "blur sends a zero command");

  fakeNow = 700;
  await joystickPointerDown(makeEvent(5, 10, 10));
  document.hidden = true;
  fakeNow = 800;
  await handleVisibilityChange();
  expect(posts[posts.length - 1].payload.throttle === 0, "hidden document sends a zero command");
  document.hidden = false;

  fakeNow = 900;
  await joystickPointerDown(makeEvent(6, 10, 10));
  await setMappingJoystickVisible(true);
  expect(elements.mappingJoystick.hidden === false, "joystick is visible for Mapping");
  expect(stage.classList.contains("mapping-active"), "HUD shift class is applied for Mapping");
  fakeNow = 1000;
  await setMappingJoystickVisible(false);
  expect(elements.mappingJoystick.hidden === true, "joystick is hidden away from Mapping");
  expect(!stage.classList.contains("mapping-active"), "HUD shift class is removed away from Mapping");
  expect(posts[posts.length - 1].payload.throttle === 0, "tab switch sends a zero command");

  fakeNow = 1100;
  await joystickPointerDown(makeEvent(7, 10, 10));
  const seqBeforePageHide = teleopState.seq;
  handlePageHide();
  expect(beacons.length === 1, "page shutdown sends one keepalive stop");
  const beacon = JSON.parse(beacons[0].body);
  expect(beacon.throttle === 0, "keepalive stop is a zero command");
  expect(beacon.client_id === teleopState.clientId, "keepalive stop carries the tab client_id");
  expect(Number.isInteger(beacon.seq) && beacon.seq > seqBeforePageHide, "keepalive stop increments seq");

  let previousSeq = 0;
  for (const post of posts) {
    expect(post.payload.client_id === teleopState.clientId, "every teleop post carries the tab client_id");
    expect(Number.isInteger(post.payload.seq) && post.payload.seq > previousSeq, "teleop seq strictly increases");
    previousSeq = post.payload.seq;
  }

  // A failed nonzero send clears the loop but keeps the stop obligation.
  teleopState.stopRequired = false;
  teleopState.pointerId = null;
  teleopState.active = false;
  fakeNow = 1200;
  failNextRequest = true;
  await joystickPointerDown(makeEvent(8, 20, 20));
  expect(intervalHandlers.size === 0, "failed active send clears the interval");
  expect(elements.joystickKnob.style.transform === "translate(0px, 0px)", "failed active send centers the knob");
  expect(teleopState.pointerId === null, "failed active send releases the pointer");
  expect(teleopState.stopRequired === true, "failed active send leaves stopRequired true");

  const beforeFailureRelease = posts.length;
  fakeNow = 1300;
  await joystickPointerUp(makeEvent(8, 20, 20));
  expect(posts.length === beforeFailureRelease + 1, "release after failure still sends a stop");
  expect(posts[posts.length - 1].payload.throttle === 0, "release after failure sends zero");
  expect(teleopState.stopRequired === false, "successful zero clears stopRequired");

  // stopRequired survives a failed zero and is only cleared by a successful zero.
  teleopState.stopRequired = true;
  failNextRequest = true;
  fakeNow = 1400;
  await stopTeleop("failed-zero");
  expect(teleopState.stopRequired === true, "failed zero request keeps stopRequired true");

  const beforeSuccessfulZero = posts.length;
  fakeNow = 1500;
  await stopTeleop("retry-zero");
  expect(teleopState.stopRequired === false, "successful zero clears stopRequired");
  expect(posts.length === beforeSuccessfulZero + 1, "retry zero is sent");
  const afterClearedZero = posts.length;
  fakeNow = 1550;
  await stopTeleop("already-stopped");
  expect(posts.length === afterClearedZero, "no continuous zero loop once stopRequired is clear");

  // Deferred/out-of-order: the stop must carry a higher seq than the slow active.
  teleopState.stopRequired = false;
  teleopState.pointerId = null;
  teleopState.active = false;
  teleopState.throttle = 0;
  teleopState.steering = 0;
  deferRequests = true;
  fakeNow = 1600;
  const downPromise = joystickPointerDown(makeEvent(10, 20, 20));
  const activePost = posts[posts.length - 1];
  expect(teleopState.stopRequired === true, "nonzero request sets stopRequired before it resolves");
  fakeNow = 1700;
  const upPromise = joystickPointerUp(makeEvent(10, 20, 20));
  const stopPost = posts[posts.length - 1];
  expect(stopPost.payload.throttle === 0 && stopPost.payload.steering === 0, "deferred stop is zero");
  expect(stopPost.payload.client_id === activePost.payload.client_id, "deferred stop shares client_id");
  expect(stopPost.payload.seq > activePost.payload.seq, "deferred stop seq is higher");
  const stopEntry = deferred.find((entry) => entry.payload === stopPost.payload);
  const activeEntry = deferred.find((entry) => entry.payload === activePost.payload);
  expect(stopEntry && activeEntry, "both deferred requests are pending");
  stopEntry.resolve({ ok: true, teleop: { active: false } });
  await upPromise;
  expect(teleopState.stopRequired === false, "out-of-order stop success clears stopRequired");
  activeEntry.resolve({ ok: true, stale: true });
  await downPromise;
  expect(teleopState.stopRequired === false, "stale active response does not re-arm stopRequired");
  deferRequests = false;

  console.log("mapping joystick ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""


def test_mapping_joystick_pointer_lifecycle_and_throttling_execute_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _MAPPING_JOYSTICK_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "mapping joystick ok" in completed.stdout


def test_mapping_joystick_exposes_adjustable_layout_controls():
    index_html, app_js, styles_css = _frontend_sources()

    assert 'id="joystickMoveHandle"' in index_html
    assert 'id="joystickResizeHandle"' in index_html
    assert 'id="joystickSettings"' in index_html
    assert 'id="joystickDone"' in index_html
    assert 'id="joystickSettingsError"' in index_html
    assert 'aria-label="Joystick settings"' in index_html

    assert "--joystick-size" in styles_css
    assert ".joystick.layout-mode" in styles_css
    assert ".stage.hud-lower-right .hud" in styles_css
    assert "function applyJoystickLayout()" in app_js
    assert "function clampedJoystickSize(value)" in app_js
    assert "function updateHudPlacement()" in app_js
    assert "function enterJoystickLayout()" in app_js
    assert "function exitJoystickLayout()" in app_js
    assert 'getJson("/api/ui/preferences")' in app_js
    assert 'postJson("/api/ui/preferences"' in app_js
    assert "teleop.layoutMode" in app_js


_JOYSTICK_LAYOUT_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const startA = source.indexOf("// --- Mapping joystick");
const endA = source.indexOf("// --- End mapping joystick");
const startB = source.indexOf("// --- Joystick layout");
const endB = source.indexOf("// --- End joystick layout");
if (startA < 0 || endA < 0 || startB < 0 || endB < 0) {
  throw new Error("joystick layout block not found");
}
const block = source.slice(startA, endA) + "\n" + source.slice(startB, endB);

function makeElement(id) {
  const classes = new Set();
  return {
    id,
    hidden: false,
    textContent: "",
    offsetWidth: 0,
    offsetHeight: 0,
    captured: [],
    attributes: {},
    style: {
      left: "0px",
      top: "0px",
      values: {},
      setProperty(name, value) { this.values[name] = value; },
    },
    classList: {
      toggle(name, on) {
        if (on === undefined) {
          if (classes.has(name)) classes.delete(name);
          else classes.add(name);
        } else if (on) {
          classes.add(name);
        } else {
          classes.delete(name);
        }
      },
      add(name) { classes.add(name); },
      remove(name) { classes.delete(name); },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
    setPointerCapture(pointerId) { this.captured.push(pointerId); },
    addEventListener() {},
    getBoundingClientRect() {
      const left = Number.parseFloat(this.style.left) || 0;
      const top = Number.parseFloat(this.style.top) || 0;
      const width = this.offsetWidth || 0;
      const height = this.offsetHeight || 0;
      return { left, top, right: left + width, bottom: top + height, width, height };
    },
  };
}

const elements = {
  view3d: makeElement("view3d"),
  mappingJoystick: makeElement("mappingJoystick"),
  joystickBase: makeElement("joystickBase"),
  joystickKnob: makeElement("joystickKnob"),
  joystickStatus: makeElement("joystickStatus"),
  joystickMoveHandle: makeElement("joystickMoveHandle"),
  joystickResizeHandle: makeElement("joystickResizeHandle"),
  joystickSettings: makeElement("joystickSettings"),
  joystickDone: makeElement("joystickDone"),
  joystickSettingsError: makeElement("joystickSettingsError"),
  hud: makeElement("hud"),
};
elements.view3d.clientWidth = 400;
elements.view3d.clientHeight = 300;
elements.view3d.getBoundingClientRect = () => ({ left: 0, top: 0, right: 400, bottom: 300, width: 400, height: 300 });
elements.mappingJoystick.offsetWidth = 100;
elements.mappingJoystick.offsetHeight = 140;
elements.joystickBase.offsetWidth = 100;
elements.joystickBase.offsetHeight = 100;
elements.hud.offsetWidth = 60;
elements.hud.offsetHeight = 20;

const stage = makeElement("stage");
stage.getBoundingClientRect = () => ({ left: 0, top: 0, right: 400, bottom: 300, width: 400, height: 300 });

global.state = { view: "mapping" };
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
global.window = {
  addEventListener() {},
  setInterval() { return 1; },
  clearInterval() {},
};
global.viewerSessionId = () => "tab-session";

const teleopPosts = [];
const uiPosts = [];
let failUiSave = false;
global.postJson = async (url, payload) => {
  if (url === "/api/ui/preferences") {
    if (failUiSave) throw new Error("network down");
    uiPosts.push(payload);
    return { ok: true, joystick: payload };
  }
  teleopPosts.push({ url, payload });
  const active = payload.throttle !== 0 || payload.steering !== 0;
  return { ok: true, teleop: { active } };
};
global.getJson = async () => ({
  ok: true,
  joystick: { size: 220, x_ratio: 0.25, y_ratio: 0.5 },
});
global.fetchWithTimeout = async () => ({ ok: true });

// Run the two blocks in an isolated function scope so their declarations do
// not leak into this harness and clash with the names destructured below.
const joystickApi = new Function(block + `
return {
  teleop, joystickLayout, refreshJoystickPreferences, joystickPointerDown,
  enterJoystickLayout, exitJoystickLayout, beginJoystickMove, moveJoystickGesture,
  beginJoystickResize, resizeJoystickGesture, finishJoystickGesture,
  applyJoystickLayout, JOYSTICK_VIEWPORT_MARGIN,
};`)();
const {
  teleop, joystickLayout, refreshJoystickPreferences, joystickPointerDown,
  enterJoystickLayout, exitJoystickLayout, beginJoystickMove, moveJoystickGesture,
  beginJoystickResize, resizeJoystickGesture, finishJoystickGesture,
  applyJoystickLayout, JOYSTICK_VIEWPORT_MARGIN,
} = joystickApi;

function expect(condition, message) { if (!condition) throw new Error(message); }

function makeEvent(pointerId, clientX, clientY, currentTarget) {
  return {
    pointerId,
    pointerType: "mouse",
    clientX,
    clientY,
    currentTarget: currentTarget || elements.joystickBase,
    preventDefault() {},
    stopPropagation() {},
  };
}

(async () => {
  await refreshJoystickPreferences();
  expect(joystickLayout.size === 220, "preferences size applied");
  expect(joystickLayout.xRatio === 0.25, "preferences x ratio applied");
  expect(joystickLayout.yRatio === 0.5, "preferences y ratio applied");
  expect(
    elements.mappingJoystick.style.values["--joystick-size"] === "220px",
    "size css variable applied",
  );

  await joystickPointerDown(makeEvent(1, 30, 30));
  expect(teleop.active === true, "drive gesture active before layout mode");
  const teleopBeforeLayout = teleopPosts.length;

  await enterJoystickLayout();
  expect(teleop.layoutMode === true, "layout mode disables drive");
  expect(joystickLayout.mode === true, "layout mode state set");
  expect(elements.mappingJoystick.classList.contains("layout-mode"), "layout class applied");
  expect(teleopPosts.length === teleopBeforeLayout + 1, "entering layout posts one stop");
  expect(teleopPosts[teleopPosts.length - 1].payload.throttle === 0, "layout stop is zero throttle");
  expect(teleopPosts[teleopPosts.length - 1].payload.steering === 0, "layout stop is zero steering");

  const teleopAfterLayout = teleopPosts.length;
  await joystickPointerDown(makeEvent(2, 30, 30));
  expect(teleopPosts.length === teleopAfterLayout, "drive gesture disabled in layout mode");
  expect(teleop.pointerId === null, "disabled gesture does not capture the pointer");

  const uiBeforeResize = uiPosts.length;
  beginJoystickResize(makeEvent(3, 0, 0, elements.joystickResizeHandle));
  resizeJoystickGesture(makeEvent(3, 500, 500, elements.joystickResizeHandle));
  expect(joystickLayout.size === 260, "resize clamps to max");
  resizeJoystickGesture(makeEvent(3, -500, -500, elements.joystickResizeHandle));
  expect(joystickLayout.size === 140, "resize clamps to min");
  expect(uiPosts.length === uiBeforeResize, "resize preview does not write");
  await finishJoystickGesture(makeEvent(3, -500, -500, elements.joystickResizeHandle));
  expect(uiPosts.length === uiBeforeResize + 1, "resize release writes once");
  expect(uiPosts[uiPosts.length - 1].size === 140, "resize write carries the clamped size");

  const uiBeforeMove = uiPosts.length;
  beginJoystickMove(makeEvent(4, 0, 0, elements.joystickMoveHandle));
  moveJoystickGesture(makeEvent(4, -9999, -9999, elements.joystickMoveHandle));
  expect(
    Number.parseFloat(elements.mappingJoystick.style.left) === JOYSTICK_VIEWPORT_MARGIN,
    "move clamps the left edge",
  );
  expect(
    Number.parseFloat(elements.mappingJoystick.style.top) === JOYSTICK_VIEWPORT_MARGIN,
    "move clamps the top edge",
  );
  moveJoystickGesture(makeEvent(4, 9999, 9999, elements.joystickMoveHandle));
  expect(
    Number.parseFloat(elements.mappingJoystick.style.left) <= 400 - 100 - JOYSTICK_VIEWPORT_MARGIN,
    "move clamps the right edge",
  );
  expect(
    Number.parseFloat(elements.mappingJoystick.style.top) <= 300 - 140 - JOYSTICK_VIEWPORT_MARGIN,
    "move clamps the bottom edge",
  );
  expect(uiPosts.length === uiBeforeMove, "move preview does not write");
  await finishJoystickGesture(makeEvent(4, 9999, 9999, elements.joystickMoveHandle));
  expect(uiPosts.length === uiBeforeMove + 1, "move release writes once");
  const savedPosition = uiPosts[uiPosts.length - 1];
  expect(savedPosition.x_ratio >= 0 && savedPosition.x_ratio <= 1, "move x ratio in range");
  expect(savedPosition.y_ratio >= 0 && savedPosition.y_ratio <= 1, "move y ratio in range");

  joystickLayout.xRatio = 0;
  joystickLayout.yRatio = 1;
  applyJoystickLayout();
  expect(stage.classList.contains("hud-lower-right"), "HUD avoids the lower-left joystick");
  joystickLayout.xRatio = 1;
  joystickLayout.yRatio = 1;
  applyJoystickLayout();
  expect(!stage.classList.contains("hud-lower-right"), "HUD returns to the lower-left");

  failUiSave = true;
  beginJoystickResize(makeEvent(5, 0, 0, elements.joystickResizeHandle));
  resizeJoystickGesture(makeEvent(5, 20, 20, elements.joystickResizeHandle));
  await finishJoystickGesture(makeEvent(5, 20, 20, elements.joystickResizeHandle));
  expect(joystickLayout.mode === true, "failed save keeps layout mode");
  expect(
    typeof joystickLayout.saveError === "string" && joystickLayout.saveError.length > 0,
    "failed save records an error",
  );
  expect(elements.joystickSettingsError.hidden === false, "failed save shows the error");
  failUiSave = false;
  beginJoystickResize(makeEvent(6, 0, 0, elements.joystickResizeHandle));
  resizeJoystickGesture(makeEvent(6, 0, 0, elements.joystickResizeHandle));
  await finishJoystickGesture(makeEvent(6, 0, 0, elements.joystickResizeHandle));
  expect(joystickLayout.saveError === null, "retry clears the error");
  expect(elements.joystickSettingsError.hidden === true, "retry hides the error");

  exitJoystickLayout();
  expect(teleop.layoutMode === false, "exit restores drive control");
  expect(joystickLayout.mode === false, "exit clears layout mode");
  expect(!elements.mappingJoystick.classList.contains("layout-mode"), "exit removes the layout class");

  console.log("joystick layout ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""


def test_mapping_joystick_layout_mode_persists_and_clamps_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _JOYSTICK_LAYOUT_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "joystick layout ok" in completed.stdout
