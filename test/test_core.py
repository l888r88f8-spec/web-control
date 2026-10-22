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
    assert normalize_layer_name("global_map") == "global_map"
    assert normalize_layer_name("obstacles") == "obstacles"
    try:
        normalize_layer_name("fullmap")
    except ValueError as exc:
        assert "unknown layer" in str(exc)
    else:
        raise AssertionError("removed PCD fullmap layer was accepted")
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
                name: SourceCache(
                    name,
                    stale_timeout_sec=2.0,
                    static_source=name != "obstacles",
                )
                for name in ("global_map", "local_map", "mapcloud", "mapground", "obstacles")
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
        {"points3d": [[1.0, 2.0, 3.0]]},
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
        node.layer_payload("fullmap", "3d")


def test_navigation_page_adds_localization_global_map_and_obstacles():
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

    assert web_control_node.LAYER_NAMES == {
        "global_map",
        "local_map",
        "mapcloud",
        "mapground",
        "obstacles",
    }

    # Both new navigation layers are enabled by default in the sidebar.
    assert 'data-layer="global_map" checked> global map' in index_html
    assert 'data-layer="obstacles" checked> obstacles' in index_html
    assert 'data-layer="local_map" checked> local map' in index_html
    assert 'data-layer="mapcloud" checked> navigation map' in index_html
    assert 'data-layer="mapground" checked> ground map' in index_html
    assert 'data-layer="fullmap"' not in index_html
    navigation_layers = app_js[
        app_js.index("const LAYERS = {") : app_js.index("const MAPPING_LAYERS = {")
    ]
    assert "global_map: { color: 0x6f8fb0, points3d: [], enabled: true" in navigation_layers
    assert "obstacles: { color: 0xff0000, points3d: [], enabled: true" in navigation_layers
    assert "pointSize: 5" in navigation_layers
    assert "flatColor: true" in navigation_layers
    assert "fullmap" not in navigation_layers

    # Direct localization global-map subscription (never the mapping page's
    # topic) plus the live aggregated obstacle cloud.
    assert (
        'self.declare_parameter("global_map_topic", "/lio_sam/localization/global_map")'
        in node_source
    )
    assert 'self.declare_parameter("obstacle_topic", "/aggregated_pc")' in node_source
    assert '"global_map": SourceCache("global_map"' in node_source
    assert '"obstacles": SourceCache("obstacles"' in node_source
    assert "def obstacle_callback(self, msg: PointCloud2)" in node_source
    assert 'self.get_parameter("obstacle_topic").value' in node_source
    assert "FullMapCache" not in node_source
    assert "full_map" not in node_source
    assert 'layer = query.get("layer", ["local_map"])[0]' in node_source
    assert 'global_map_topic: "/lio_sam/localization/global_map"' in config_yaml
    assert 'obstacle_topic: "/aggregated_pc"' in config_yaml


def test_navigation_global_map_is_direct_localization_topic_not_mapping():
    package_root = Path(__file__).resolve().parents[1]
    node_source = (
        package_root / "web_control" / "web_control_node.py"
    ).read_text(encoding="utf-8")
    config_yaml = (package_root / "config" / "web_control.yaml").read_text(
        encoding="utf-8"
    )

    assert (
        'self.declare_parameter("global_map_topic", "/lio_sam/localization/global_map")'
        in node_source
    )
    assert '"global_map": SourceCache("global_map"' in node_source
    assert "    global_map_topic:" in config_yaml
    # The mapping page keeps its independent live mapping cloud.
    assert "mapping_global_map_topic" in config_yaml
    assert 'self.declare_parameter("mapping_global_map_topic", "/lio_sam/mapping/map_global")' in node_source
    assert "FullMapCache" not in node_source
    assert "read_pcd" not in node_source


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


def test_frontend_hosts_liquid_glass_defs():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")

    defs_start = index_html.index('id="liquidGlassDefs"')
    defs_tag = index_html[index_html.rindex("<svg", 0, defs_start) : index_html.index(">", defs_start)]
    assert 'class="glass-defs"' in defs_tag
    assert 'aria-hidden="true"' in defs_tag
    assert 'focusable="false"' in defs_tag
    assert defs_start < index_html.index('<main class="app">')
    assert 'id="view3d"' in index_html
    assert 'id="viewSystem"' in index_html


def test_liquid_glass_helper_exports_provenance_and_static_assets():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    source = (static_dir / "liquid-glass.js").read_text(encoding="utf-8")

    for name in (
        "initLiquidGlass",
        "detectGlassMode",
        "applyGlassMode",
        "glassMode",
        "glassMotionReduced",
    ):
        assert f"export function {name}" in source
    assert "https://github.com/shuding/liquid-glass" in source
    assert "independently implemented" in source
    assert "data:image/png;base64," in source
    assert source.count('"feImage"') == 1
    assert source.count('"feDisplacementMap"') == 1
    assert "requestAnimationFrame" not in source
    assert "setInterval" not in source
    assert "ResizeObserver" not in source
    assert "localStorage" not in source
    assert "sessionStorage" not in source


def test_track_refraction_module_exports_pure_and_lifecycle_api():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    source = (static_dir / "track-refraction.js").read_text(encoding="utf-8")

    for name in (
        "smoothStep",
        "roundedRectSDF",
        "computeTrackDisplacementField",
        "initTrackRefraction",
    ):
        assert f"export function {name}" in source
    assert "https://github.com/shuding/liquid-glass" in source
    assert "THIRD_PARTY_NOTICES.md" in source
    assert "MIT" in source
    assert "Shu Ding" in source
    assert 'setAttribute("color-interpolation-filters", "sRGB")' in source
    assert '"feImage"' in source
    assert '"feDisplacementMap"' in source
    assert "liquidTrackFilter" in source
    assert "ResizeObserver" in source
    assert "matchMedia" in source
    assert 'addEventListener("resize"' in source
    assert "onGlassModeChange" in source
    assert "localStorage" not in source
    assert "sessionStorage" not in source


def test_liquid_glass_exposes_safe_mode_subscription():
    source = (
        Path(__file__).resolve().parents[1] / "static" / "liquid-glass.js"
    ).read_text(encoding="utf-8")

    assert "export function onGlassModeChange(listener)" in source
    assert "modeListeners.add(listener)" in source
    assert "modeListeners.delete(listener)" in source
    apply_body = _js_function_body(source, "export function applyGlassMode(mode)")
    assert "notifyGlassMode(resolved);" in apply_body
    notify_body = _js_function_body(source, "function notifyGlassMode(mode)")
    assert "try {" in notify_body
    assert "catch" in notify_body


def test_track_refraction_is_wired_after_init_liquid_glass():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    assert 'import { initTrackRefraction } from "./track-refraction.js";' in app_js
    assert "initLiquidGlass({ savedMode: readStoredGlassStrength() });\ninitTrackRefraction();" in app_js


def test_third_party_notices_license_and_packaging():
    root = Path(__file__).resolve().parents[1]
    notice = (root / "THIRD_PARTY_NOTICES.md").read_text(encoding="utf-8")
    assert "shuding/liquid-glass" in notice
    assert "MIT License" in notice
    assert "Copyright (c) 2025 Shu Ding" in notice
    assert "Permission is hereby granted, free of charge" in notice
    assert "adapt" in notice.lower()

    setup_py = (root / "setup.py").read_text(encoding="utf-8")
    assert "THIRD_PARTY_NOTICES.md" in setup_py
    assert "share/{package_name}/static" in setup_py
    assert "share/{package_name}/static/vendor" in setup_py
    assert "share/{package_name}/config" in setup_py
    assert "share/{package_name}/launch" in setup_py


def test_glass_full_track_uses_dynamic_filter_and_thumb_is_static():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    track = _css_rule_block(styles_css, ".glass-full .sliding-tabs")
    assert (
        "backdrop-filter: url(#liquidTrackFilter) blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1);"
        in track
    )
    assert (
        "-webkit-backdrop-filter: url(#liquidTrackFilter) blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1);"
        in track
    )
    shared = _css_rule_block(styles_css, ":where(.glass-full, .glass-light) .sliding-tabs")
    assert "background: var(--track-glass-bg);" in shared
    assert "border-color: var(--track-glass-edge);" in shared
    assert "0 4px 8px rgba(0, 0, 0, 0.25)" in shared
    assert "inset 0 -10px 25px rgba(0, 0, 0, 0.15)" in shared
    assert ".glass-full .sliding-tabs::before" not in styles_css

    static_start = styles_css.index(".glass-full .segmented::before,")
    static_body = styles_css[
        styles_css.index("{", static_start) + 1 : styles_css.index("}", static_start)
    ]
    assert "backdrop-filter: url(#liquidDrop) saturate(1.4);" in static_body
    assert "-webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);" in static_body
    assert ".glass-full .sidebar-toggle-lens {" in styles_css


def test_glass_light_track_fallback_and_track_tokens():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    light = _css_rule_block(styles_css, ".glass-light .sliding-tabs")
    assert "backdrop-filter: blur(22px) saturate(1.4);" in light
    assert "-webkit-backdrop-filter: blur(22px) saturate(1.4);" in light

    root = _css_rule_block(styles_css, ":root")
    assert "--track-glass-bg: rgba(255, 255, 255, 0.14);" in root
    assert "--track-glass-edge: rgba(255, 255, 255, 0.92);" in root
    assert "--track-glass-label: #f8fafc;" in root
    assert "--track-system-label: #111827;" in root
    assert "--track-glass-bg: rgba(8, 12, 20, 0.16);" in styles_css
    assert "--track-glass-label: #f5f7fb;" in styles_css
    assert "--track-system-label: #f5f7fb;" in styles_css

    off = _css_rule_block(styles_css, ".glass-off")
    assert "--track-glass-bg: rgba(255, 255, 255, 0.96);" in off
    assert "--track-glass-label: #111827;" in off
    assert "--track-glass-bg: rgba(28, 28, 34, 0.98);" in styles_css


def test_glass_system_mode_labels_use_dedicated_token():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )

    # Map views sit on a dark canvas, so translucent tracks keep the light
    # --track-glass-label for their inactive labels.
    map_start = styles_css.index(".glass-full .sliding-tabs .tab:not(.active),")
    map_rule = styles_css[map_start : styles_css.index("}", map_start)]
    assert ".glass-light .sliding-tabs .tab:not(.active)" in map_rule
    assert "color: var(--track-glass-label);" in map_rule

    # System has its own surface, so its inactive labels use the dedicated
    # scheme-aware --track-system-label token in every glass tier.
    system_selector = (
        ":where(.glass-full, .glass-light, .glass-off) .app.system-mode "
        ".sliding-tabs .tab:not(.active)"
    )
    system_rule = _css_rule_block(styles_css, system_selector)
    assert "color: var(--track-system-label);" in system_rule


def test_glass_track_rules_add_no_layout_or_animation_properties():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    forbidden = (
        "width:",
        "height:",
        "padding",
        "margin",
        "grid",
        "position:",
        "top:",
        "left:",
        "right:",
        "bottom:",
        "inset:",
        "display:",
        "flex",
        "gap:",
        "font-size",
        "transform:",
        "transition:",
        "animation:",
        "will-change:",
    )
    for selector in (
        ":where(.glass-full, .glass-light) .sliding-tabs",
        ".glass-full .sliding-tabs",
        ".glass-light .sliding-tabs",
        ".glass-off .sliding-tabs",
        (
            ":where(.glass-full, .glass-light, .glass-off) .app.system-mode "
            ".sliding-tabs .tab:not(.active)"
        ),
    ):
        block = _css_rule_block(styles_css, selector)
        for prop in forbidden:
            assert prop not in block, (selector, prop)


def test_liquid_glass_wiring_in_app():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    assert (
        'import { initLiquidGlass, glassMotionReduced, glassMode, onGlassModeChange, '
        'setGlassMode } from "./liquid-glass.js";' in app_js
    )
    assert "initLiquidGlass({ savedMode: readStoredGlassStrength() });" in app_js


def test_system_view_exposes_manual_glass_strength_selector():
    index_html = (Path(__file__).resolve().parents[1] / "static" / "index.html").read_text(
        encoding="utf-8"
    )
    assert 'id="viewSystem"' in index_html
    select_start = index_html.index('id="glassStrength"')
    select_body = index_html[select_start : index_html.index("</select>", select_start)]
    assert 'for="glassStrength"' in index_html
    assert "Glass Strength" in index_html
    for value, label in (("full", "Full"), ("light", "Light"), ("off", "Off")):
        assert f'value="{value}"' in select_body, value
        assert f">{label}<" in select_body, label
    # Only the three documented strengths are offered.
    assert select_body.count("<option") == 3
    # The selector lives inside the System view, not the sidebar.
    assert index_html.index('id="viewSystem"') < select_start


def test_liquid_glass_helper_has_no_capability_heuristics():
    source = (
        Path(__file__).resolve().parents[1] / "static" / "liquid-glass.js"
    ).read_text(encoding="utf-8")
    for forbidden in (
        "isChromiumEngine",
        "saveDataEnabled",
        "supportsDisplacement",
        "userAgentData",
        "navigator.userAgent",
        "navigator.connection",
        "CSS.supports",
        "prefers-reduced-transparency",
        "prefersReducedTransparency",
        "transparencyQuery",
    ):
        assert forbidden not in source, forbidden
    detect_body = _js_function_body(source, "export function detectGlassMode()")
    assert 'return "full";' in detect_body
    assert 'return "light";' not in detect_body
    assert 'return "off";' not in detect_body
    # Only explicit signals decide the strength.
    assert "manualOverride" in detect_body
    assert "queryOverride" in detect_body
    assert "savedMode" in detect_body


def test_app_persists_glass_strength_locally_not_server_side():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    assert 'const GLASS_STRENGTH_KEY = "web_control_glass_strength";' in app_js
    assert "window.localStorage.getItem(GLASS_STRENGTH_KEY)" in app_js
    assert "window.localStorage.setItem(GLASS_STRENGTH_KEY, mode)" in app_js
    block = app_js[
        app_js.index("// --- Glass strength preference") : app_js.index(
            "// --- End glass strength preference"
        )
    ]
    assert "/api/" not in block


_GLASS_STRENGTH_PREFERENCE_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Glass strength preference");
const blockEnd = source.indexOf("// --- End glass strength preference");
if (blockStart < 0 || blockEnd < 0) throw new Error("glass strength block not found");
const block = source.slice(blockStart, blockEnd);

function expect(condition, message) { if (!condition) throw new Error(message); }

const store = new Map();
global.localStorage = {
  getItem(key) { return store.has(key) ? store.get(key) : null; },
  setItem(key, value) { store.set(key, String(value)); },
  removeItem(key) { store.delete(key); },
};
const select = {
  value: "full",
  listeners: {},
  addEventListener(type, listener) { this.listeners[type] = listener; },
};
global.document = {
  getElementById(id) { return id === "glassStrength" ? select : null; },
};
global.window = global;
let applied = null;
global.setGlassMode = (mode) => { applied = mode; return mode; };
global.glassMode = () => (applied === null ? "full" : applied);

const api = new Function(block + `
return {
  readStoredGlassStrength,
  storeGlassStrength,
  updateGlassStrengthControl,
  applyGlassStrength,
  attachGlassStrengthControl,
};`)();

expect(api.readStoredGlassStrength() === null, "no saved value reads null");
store.set("web_control_glass_strength", "banana");
expect(api.readStoredGlassStrength() === null, "invalid saved value is ignored");
store.set("web_control_glass_strength", "light");
expect(api.readStoredGlassStrength() === "light", "valid saved value is read");
api.storeGlassStrength("banana");
expect(store.get("web_control_glass_strength") === "light", "invalid values are never stored");
expect(api.applyGlassStrength("off") === "off", "applyGlassStrength returns the applied mode");
expect(applied === "off", "applyGlassStrength applies the mode");
expect(store.get("web_control_glass_strength") === "off", "applyGlassStrength persists the mode");
expect(select.value === "off", "applyGlassStrength mirrors the selector");
api.attachGlassStrengthControl();
expect(typeof select.listeners.change === "function", "attach wires the change handler");
select.listeners.change({ target: { value: "full" } });
expect(applied === "full", "change applies the selector value");
expect(store.get("web_control_glass_strength") === "full", "change persists the selector value");

const denied = { getItem() { throw new Error("denied"); }, setItem() { throw new Error("denied"); } };
global.localStorage = denied;
expect(api.readStoredGlassStrength() === null, "denied storage reads null");
api.storeGlassStrength("light");
console.log("glass strength preference ok");
"""


def test_glass_strength_preference_persists_in_browser_storage():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _GLASS_STRENGTH_PREFERENCE_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "glass strength preference ok" in completed.stdout


_LIQUID_GLASS_MODE_SELECTION_NODE_HARNESS = r"""
const { pathToFileURL } = require("url");
const path = process.env.LIQUID_GLASS;

let mediaQueries = [];

function stub({ search = "", reducedMotion = false } = {}) {
  mediaQueries = [];
  globalThis.location = { search };
  globalThis.window = globalThis;
  globalThis.matchMedia = (query) => {
    mediaQueries.push(query);
    return {
      matches: /prefers-reduced-motion/.test(query) ? reducedMotion : false,
      addEventListener() {},
    };
  };
  globalThis.document = {
    documentElement: {
      lastMode: null,
      classList: { toggle() {} },
      setAttribute(name, value) { if (name === "data-glass-mode") this.lastMode = value; },
    },
    querySelector() { return null; },
  };
}

async function load(caseName) {
  return import(pathToFileURL(path).href + "?case=" + encodeURIComponent(caseName));
}

function expect(condition, message) { if (!condition) throw new Error(message); }

(async () => {
  // With no explicit signal, Full is the deterministic default.
  stub();
  let mod = await load("default");
  expect(mod.detectGlassMode() === "full", "no explicit choice defaults to full");

  // An invalid saved value falls through to the same Full default.
  stub();
  mod = await load("invalid-saved");
  mod.initLiquidGlass({ savedMode: "banana" });
  expect(mod.detectGlassMode() === "full", "invalid saved mode falls through to full");

  // prefers-reduced-motion only changes animation timing. It never changes the
  // strength, and the helper must not consult prefers-reduced-transparency.
  stub({ reducedMotion: true });
  mod = await load("reduced-motion");
  mod.initLiquidGlass();
  expect(mod.detectGlassMode() === "full", "reduced motion does not change the strength");
  expect(mod.glassMode() === "full", "reduced motion keeps the applied mode full");
  expect(
    mediaQueries.every((query) => !/transparency/i.test(query)),
    "no transparency media query is consulted",
  );

  // A valid saved choice is used when there is no override.
  stub();
  mod = await load("saved-mode");
  mod.initLiquidGlass({ savedMode: "light" });
  expect(mod.detectGlassMode() === "light", "saved mode is used");

  // The query override stays explicit and outranks the saved choice.
  stub({ search: "?glass=off" });
  mod = await load("query-over-saved");
  mod.initLiquidGlass({ savedMode: "full" });
  expect(mod.detectGlassMode() === "off", "query override wins over the saved choice");

  // The programmatic override wins over a conflicting query value and a saved
  // choice.
  stub({ search: "?glass=off" });
  mod = await load("override-programmatic");
  mod.initLiquidGlass({ savedMode: "light" });
  globalThis.__webControlSetGlassMode("full");
  expect(mod.detectGlassMode() === "full", "programmatic override wins over the query");

  // setGlassMode is the deterministic programmatic entry point.
  stub();
  mod = await load("set-mode");
  expect(mod.setGlassMode("light") === "light", "setGlassMode returns the applied mode");
  expect(mod.glassMode() === "light", "setGlassMode applies the mode");

  // applyGlassMode records and mirrors the applied mode.
  stub();
  mod = await load("apply-mode");
  mod.applyGlassMode("off");
  expect(mod.glassMode() === "off", "applyGlassMode stores the applied mode");
  expect(globalThis.document.documentElement.lastMode === "off", "data-glass-mode mirrors the mode");

  console.log("liquid glass mode selection ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_liquid_glass_mode_precedence_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    liquid_glass = Path(__file__).resolve().parents[1] / "static" / "liquid-glass.js"
    completed = subprocess.run(
        ["node", "-e", _LIQUID_GLASS_MODE_SELECTION_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "LIQUID_GLASS": str(liquid_glass)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "liquid glass mode selection ok" in completed.stdout


_GLASS_MODE_SUBSCRIPTION_NODE_HARNESS = r"""
const { pathToFileURL } = require("url");
const path = process.env.LIQUID_GLASS;

function expect(condition, message) { if (!condition) throw new Error(message); }

const classes = new Set();
globalThis.document = {
  documentElement: {
    classList: {
      toggle(name, on) { if (on) classes.add(name); else classes.delete(name); },
      contains(name) { return classes.has(name); },
    },
    setAttribute() {},
  },
  querySelector() { return null; },
};
globalThis.window = globalThis;
globalThis.navigator = { connection: {}, userAgent: "", userAgentData: { brands: [] } };
globalThis.CSS = { supports: () => true };
globalThis.matchMedia = () => ({ matches: false, addEventListener() {} });

(async () => {
  const mod = await import(pathToFileURL(path).href + "?subscribe");
  const seen = [];
  const unsubscribeBad = mod.onGlassModeChange(() => { throw new Error("boom"); });
  const unsubscribeGood = mod.onGlassModeChange((mode) => seen.push(mode));
  mod.applyGlassMode("light");
  expect(mod.glassMode() === "light", "mode applied despite a throwing listener");
  expect(classes.has("glass-light"), "root class applied despite a throwing listener");
  expect(seen.length === 1 && seen[0] === "light", "later listeners still run");
  const unsubscribeNoop = mod.onGlassModeChange(null);
  expect(typeof unsubscribeNoop === "function", "non-function listeners are ignored safely");
  unsubscribeNoop();
  unsubscribeBad();
  unsubscribeGood();
  mod.applyGlassMode("off");
  expect(mod.glassMode() === "off", "mode still switches after unsubscribe");
  expect(seen.length === 1, "unsubscribed listeners are not called");
  console.log("glass mode subscription ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_glass_mode_subscription_ignores_listener_errors_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    liquid_glass = Path(__file__).resolve().parents[1] / "static" / "liquid-glass.js"
    completed = subprocess.run(
        ["node", "-e", _GLASS_MODE_SUBSCRIPTION_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "LIQUID_GLASS": str(liquid_glass)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "glass mode subscription ok" in completed.stdout


_TRACK_REFRACTION_FIELD_NODE_HARNESS = r"""
const { pathToFileURL } = require("url");
const path = process.env.TRACK_REFRACTION;

function expect(condition, message) { if (!condition) throw new Error(message); }
function close(a, b, eps, message) {
  if (Math.abs(a - b) > eps) throw new Error(message + ": " + a + " vs " + b);
}

(async () => {
  const mod = await import(pathToFileURL(path).href + "?field");

  expect(mod.smoothStep(0, 1, 0.5) === 0.5, "smoothStep midpoint");
  expect(mod.smoothStep(0, 1, -2) === 0, "smoothStep clamps low");
  expect(mod.smoothStep(0, 1, 3) === 1, "smoothStep clamps high");
  expect(mod.smoothStep(5, 1, 3) === 0.5, "smoothStep descending midpoint");

  close(mod.roundedRectSDF(0, 0, 10, 6, 5), -6, 1e-9, "center distance");
  close(mod.roundedRectSDF(10, 0, 10, 6, 5), 0, 1e-9, "right edge");
  close(mod.roundedRectSDF(11, 0, 10, 6, 5), 1, 1e-9, "outside right");
  close(mod.roundedRectSDF(-10, 0, 10, 6, 5), 0, 1e-9, "left edge");
  close(mod.roundedRectSDF(0, 6, 10, 6, 5), 0, 1e-9, "top edge");
  close(
    mod.roundedRectSDF(4, 3, 10, 6, 5),
    mod.roundedRectSDF(-4, 3, 10, 6, 5),
    1e-9,
    "x symmetry",
  );
  expect(mod.roundedRectSDF(0, 0, 10, 6, 5) < 0, "inside is negative");
  expect(mod.roundedRectSDF(30, 30, 10, 6, 5) > 0, "outside is positive");

  const first = mod.computeTrackDisplacementField({ width: 320, height: 48, dpr: 2 });
  const second = mod.computeTrackDisplacementField({ width: 320, height: 48, dpr: 2 });
  expect(first.width === 640 && first.height === 96, "device dimensions honor DPR");
  expect(first.data.length === 640 * 96 * 4, "RGBA length");
  for (let i = 0; i < first.data.length; i += 1) {
    if (first.data[i] !== second.data[i]) throw new Error("field is not deterministic");
  }

  const centerIndex = ((first.height >> 1) * first.width + (first.width >> 1)) * 4;
  expect(first.data[centerIndex] === 128, "center R neutral");
  expect(first.data[centerIndex + 1] === 128, "center G neutral");
  expect(first.data[centerIndex + 2] === 0, "center B zero");
  expect(first.data[centerIndex + 3] === 255, "center alpha");

  const leftMidIndex = (first.height >> 1) * first.width * 4;
  expect(first.data[leftMidIndex] !== 128, "left rim mid-edge displaced horizontally");

  // Sample the field by CSS coordinate. Pixel centers sit at
  // x = (px + 0.5) / dpr - halfWidth, so invert that to find the index.
  function sampleChannel(field, cssWidth, cssHeight, cssDpr, cssX, cssY) {
    const halfW = cssWidth / 2;
    const halfH = cssHeight / 2;
    const px = Math.max(0, Math.min(field.width - 1, Math.round((cssX + halfW) * cssDpr - 0.5)));
    const py = Math.max(0, Math.min(field.height - 1, Math.round((cssY + halfH) * cssDpr - 0.5)));
    const i = (py * field.width + px) * 4;
    return { r: field.data[i], g: field.data[i + 1], px, py };
  }

  // A wide pill's straight top edge far from the rounded caps must refract
  // straight inward: the horizontal channel stays neutral and only G moves.
  // Center-radial displacement leaks a large horizontal component here.
  const topEdge = sampleChannel(first, 320, 48, 2, 60, -23.25);
  expect(topEdge.r === 128, "top edge far from caps keeps R neutral");
  expect(topEdge.g > 200, "top edge far from caps drives G inward");

  // The rounded right cap must refract horizontally inward: R moves and G
  // stays approximately neutral.
  const rightCap = sampleChannel(first, 320, 48, 2, 158, 0);
  expect(rightCap.r < 80, "right cap drives R inward");
  expect(Math.abs(rightCap.g - 128) <= 8, "right cap keeps G approximately neutral");

  let rimDisplaced = false;
  let alphaOk = true;
  let outsideNeutral = true;
  let displacementBounded = true;
  const strength = Math.max(2, 48 * 0.12);
  for (let py = 0; py < first.height; py += 1) {
    const cssY = (py + 0.5) / 2 - 24;
    for (let px = 0; px < first.width; px += 1) {
      const i = (py * first.width + px) * 4;
      if (first.data[i] !== 128 || first.data[i + 1] !== 128) rimDisplaced = true;
      if (first.data[i + 3] !== 255 || first.data[i + 2] !== 0) alphaOk = false;
      const cssX = (px + 0.5) / 2 - 160;
      if (mod.roundedRectSDF(cssX, cssY, 160, 24, 24) > 0) {
        if (first.data[i] !== 128 || first.data[i + 1] !== 128) outsideNeutral = false;
      }
      const dx = (first.data[i] / 255 - 0.5) * first.scale;
      const dy = (first.data[i + 1] / 255 - 0.5) * first.scale;
      if (Math.abs(dx) > strength + 0.05 || Math.abs(dy) > strength + 0.05) {
        displacementBounded = false;
      }
    }
  }
  expect(rimDisplaced, "rim is displaced");
  expect(alphaOk, "every pixel is opaque with B=0");
  // CSS border-radius clips the corner outside the pill, so keep every outside
  // pixel neutral rather than cementing displacement that is never visible.
  expect(outsideNeutral, "every pixel outside the pill stays neutral");
  expect(first.data[0] === 128, "corner outside the pill stays R neutral");
  expect(first.data[1] === 128, "corner outside the pill stays G neutral");
  expect(displacementBounded, "decoded displacement stays bounded by strength");

  const dpr1 = mod.computeTrackDisplacementField({ width: 320, height: 48, dpr: 1 });
  const dpr2 = mod.computeTrackDisplacementField({ width: 320, height: 48, dpr: 2 });
  expect(dpr1.width === 320 && dpr2.width === 640, "DPR doubles device width");
  close(dpr1.scale, dpr2.scale, 1e-9, "scale is DPR independent (CSS pixels)");
  expect(dpr1.scale > 0, "scale is positive");

  const tiny = mod.computeTrackDisplacementField({ width: 0, height: 0, dpr: 0 });
  expect(tiny.width === 1 && tiny.height === 1, "degenerate size clamps to one device pixel");
  expect(Number.isFinite(tiny.scale), "degenerate scale is finite");

  console.log("track refraction field ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_track_refraction_field_math_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    module = Path(__file__).resolve().parents[1] / "static" / "track-refraction.js"
    completed = subprocess.run(
        ["node", "-e", _TRACK_REFRACTION_FIELD_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "TRACK_REFRACTION": str(module)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "track refraction field ok" in completed.stdout


_TRACK_REFRACTION_LIFECYCLE_NODE_HARNESS = r"""
const { pathToFileURL } = require("url");
const path = process.env.TRACK_REFRACTION;

function expect(condition, message) { if (!condition) throw new Error(message); }

function makeElement(tag) {
  const element = {
    tag,
    tagName: tag.toUpperCase(),
    localName: tag,
    children: [],
    parentNode: null,
    attributes: {},
    style: {},
    id: "",
    setAttribute(name, value) {
      this.attributes[name] = String(value);
      if (name === "id") this.id = String(value);
    },
    getAttribute(name) {
      return Object.prototype.hasOwnProperty.call(this.attributes, name)
        ? this.attributes[name]
        : null;
    },
    setAttributeNS(_ns, name, value) { this.attributes[name] = String(value); },
    appendChild(child) { child.parentNode = this; this.children.push(child); return child; },
    append(...nodes) { nodes.forEach((node) => this.appendChild(node)); },
    remove() {
      if (!this.parentNode) return;
      const index = this.parentNode.children.indexOf(this);
      if (index >= 0) this.parentNode.children.splice(index, 1);
      this.parentNode = null;
    },
    querySelector(selector) {
      const id = selector.startsWith("#") ? selector.slice(1) : null;
      if (!id) return null;
      for (const child of this.children) {
        if (child.id === id) return child;
        const nested = child.querySelector ? child.querySelector(selector) : null;
        if (nested) return nested;
      }
      return null;
    },
    addEventListener() {},
    removeEventListener() {},
  };
  return element;
}

function makeCanvas() {
  const canvas = makeElement("canvas");
  canvas.width = 0;
  canvas.height = 0;
  canvas.puts = 0;
  canvas.dataUrls = 0;
  canvas.context = { putImageData(image) { canvas.puts += 1; canvas.lastImage = image; } };
  canvas.getContext = () => canvas.context;
  canvas.toDataURL = () => {
    canvas.dataUrls += 1;
    return "data:image/png;base64,canvas-" + canvas.width + "x" + canvas.height + "-" + canvas.dataUrls;
  };
  return canvas;
}

function makeDocument(track) {
  const defs = makeElement("defs");
  const document = {
    defs,
    created: [],
    getElementById(id) { return id === "viewTabs" ? track : null; },
    querySelector(selector) { return selector === "#liquidGlassDefs defs" ? defs : null; },
    createElementNS(_ns, tag) { const el = makeElement(tag); document.created.push(el); return el; },
    createElement(tag) { const el = tag === "canvas" ? makeCanvas() : makeElement(tag); document.created.push(el); return el; },
  };
  return document;
}

function makeTrack(width, height) {
  const track = makeElement("div");
  track.cssWidth = width;
  track.cssHeight = height;
  track.getBoundingClientRect = () => ({ width: track.cssWidth, height: track.cssHeight });
  return track;
}

const observerInstances = [];
class FakeResizeObserver {
  constructor(callback) { this.callback = callback; this.targets = []; this.disconnects = 0; observerInstances.push(this); }
  observe(target) { this.targets.push(target); }
  disconnect() { this.disconnects += 1; }
  trigger() { this.callback([], this); }
}

const frames = new Map();
let nextFrame = 1;
function requestAnimationFrame(cb) { const id = nextFrame++; frames.set(id, cb); return id; }
function cancelAnimationFrame(id) { frames.delete(id); }
function runFrames() {
  const pending = Array.from(frames.values());
  frames.clear();
  pending.forEach((cb) => cb());
}

function makeWindow() {
  const listeners = new Map();
  return {
    listeners,
    addEventListener(type, handler) {
      if (!listeners.has(type)) listeners.set(type, new Set());
      listeners.get(type).add(handler);
    },
    removeEventListener(type, handler) {
      if (listeners.has(type)) listeners.get(type).delete(handler);
    },
    dispatch(type) {
      if (listeners.has(type)) Array.from(listeners.get(type)).forEach((handler) => handler({ type }));
    },
  };
}

function makeMatchMedia() {
  const medias = [];
  function matchMedia(query) {
    const media = {
      query,
      listeners: new Set(),
      addEventListener(type, handler) { media.listeners.add(handler); },
      removeEventListener(type, handler) { media.listeners.delete(handler); },
    };
    medias.push(media);
    return media;
  }
  matchMedia.medias = medias;
  return matchMedia;
}

function makeModeChannel(initialMode) {
  let mode = initialMode;
  const listeners = new Set();
  return {
    getMode: () => mode,
    subscribe(listener) { listeners.add(listener); return () => { listeners.delete(listener); }; },
    listenerCount: () => listeners.size,
    set(nextMode) { mode = nextMode; Array.from(listeners).forEach((listener) => listener(nextMode)); },
  };
}

(async () => {
  const mod = await import(pathToFileURL(path).href + "?lifecycle");
  const track = makeTrack(320, 48);
  const document = makeDocument(track);
  const window = makeWindow();
  const matchMedia = makeMatchMedia();
  const channel = makeModeChannel("full");
  let dpr = 1;
  const options = {
    document,
    track,
    defs: document.defs,
    ResizeObserver: FakeResizeObserver,
    requestAnimationFrame,
    cancelAnimationFrame,
    window,
    matchMedia,
    getDevicePixelRatio: () => dpr,
    getGlassMode: channel.getMode,
    onGlassModeChange: channel.subscribe,
  };

  // Pre-seed the defs with an incomplete filter using the same id. init must
  // remove it and install exactly one complete filter, so the DOM never holds
  // duplicate #liquidTrackFilter nodes.
  const staleFilter = document.createElementNS("http://www.w3.org/2000/svg", "filter");
  staleFilter.setAttribute("id", "liquidTrackFilter");
  document.defs.appendChild(staleFilter);
  expect(
    document.defs.children.filter((child) => child.id === "liquidTrackFilter").length === 1,
    "incomplete filter pre-seeded",
  );

  const controller = mod.initTrackRefraction(options);
  expect(controller, "init returns a controller");
  const filter = document.defs.querySelector("#liquidTrackFilter");
  expect(filter !== null, "full install injects the filter");
  expect(
    document.defs.children.filter((child) => child.id === "liquidTrackFilter").length === 1,
    "incomplete filter is replaced, not duplicated",
  );
  expect(filter.children.some((child) => child.tag === "feImage"), "replacement filter has feImage");
  expect(
    filter.children.some((child) => child.tag === "feDisplacementMap"),
    "replacement filter has feDisplacementMap",
  );
  expect(filter.getAttribute("color-interpolation-filters") === "sRGB", "hyphenated color-interpolation-filters");
  expect(filter.getAttribute("filterUnits") === "userSpaceOnUse", "userSpaceOnUse filter units");
  expect(observerInstances.length === 1, "one ResizeObserver is created");
  expect(observerInstances[0].targets.length === 1, "the observer watches the track");
  expect(frames.size === 1, "install schedules one render frame");

  controller.render();
  controller.render();
  expect(frames.size === 1, "render requests coalesce");

  runFrames();
  const feImage = filter.children.find((child) => child.tag === "feImage");
  const displacement = filter.children.find((child) => child.tag === "feDisplacementMap");
  expect(feImage !== null && displacement !== null, "filter primitives exist");
  const canvas = document.created.find((el) => el.tag === "canvas");
  expect(canvas.width === 320 && canvas.height === 48, "canvas uses DPR 1 device size");
  expect(canvas.puts === 1, "the map is uploaded once");
  const firstHref = feImage.getAttribute("href");
  expect(typeof firstHref === "string" && firstHref.startsWith("data:image/png;base64,"), "feImage href is a data URL");
  expect(Number(displacement.getAttribute("scale")) > 0, "displacement scale is positive");
  expect(feImage.getAttribute("width") === "320", "feImage width is the CSS track width");
  expect(feImage.getAttribute("height") === "48", "feImage height is the CSS track height");
  expect(canvas.dataUrls === 1, "one data URL per render");

  controller.render();
  runFrames();
  expect(canvas.puts === 1, "unchanged size dedupes the upload");
  expect(canvas.dataUrls === 1, "unchanged size dedupes the data URL");

  track.cssWidth = 400;
  observerInstances[0].trigger();
  expect(frames.size === 1, "observer resize schedules one frame");
  runFrames();
  expect(canvas.width === 400, "resize rebuilds the canvas width");
  expect(canvas.puts === 2, "resize re-uploads the map");
  expect(canvas.dataUrls === 2, "resize rebuilds the data URL");
  expect(feImage.getAttribute("width") === "400", "resize updates the filter region");

  dpr = 2;
  window.dispatch("resize");
  runFrames();
  expect(canvas.width === 800, "DPR change rebuilds the canvas at device pixels");

  channel.set("light");
  expect(document.defs.querySelector("#liquidTrackFilter") === null, "light mode removes the filter");
  expect(observerInstances[0].disconnects === 1, "light mode disconnects the observer");
  expect(window.listeners.get("resize").size === 0, "light mode removes the window listener");

  controller.render();
  expect(frames.size === 0, "light mode never schedules a frame");

  channel.set("full");
  expect(document.defs.querySelector("#liquidTrackFilter") !== null, "re-enable reinstalls the filter");
  expect(observerInstances.length === 2, "re-enable creates a fresh observer");
  const putsBeforeReenable = canvas.puts;
  controller.render();
  runFrames();
  expect(canvas.puts === putsBeforeReenable + 1, "re-enable rebuilds the map");

  const again = mod.initTrackRefraction(options);
  expect(again === controller, "repeated init returns the same controller");
  expect(
    document.defs.children.filter((child) => child.id === "liquidTrackFilter").length === 1,
    "no duplicate filter",
  );

  controller.render();
  const captured = Array.from(frames.values());
  frames.clear();
  channel.set("off");
  const putsBeforeStale = canvas.puts;
  captured.forEach((cb) => cb());
  expect(canvas.puts === putsBeforeStale, "a stale callback cannot render after removal");

  channel.set("full");
  const putsBeforeDestroy = canvas.puts;
  controller.render();
  runFrames();
  expect(channel.listenerCount() >= 1, "controller is subscribed");
  const activeObserver = observerInstances[observerInstances.length - 1];
  controller.destroy();
  controller.destroy();
  expect(document.defs.querySelector("#liquidTrackFilter") === null, "destroy removes the filter");
  expect(channel.listenerCount() === 0, "destroy unsubscribes");
  expect(activeObserver.disconnects === 1, "destroy disconnects the active observer once");
  expect(window.listeners.get("resize").size === 0, "destroy removes the window listener");
  expect(matchMedia.medias[matchMedia.medias.length - 1].listeners.size === 0, "destroy removes the DPR watcher");
  controller.render();
  runFrames();
  expect(canvas.puts === putsBeforeDestroy + 1, "destroyed controller never renders again");

  console.log("track refraction lifecycle ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_track_refraction_lifecycle_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    module = Path(__file__).resolve().parents[1] / "static" / "track-refraction.js"
    completed = subprocess.run(
        ["node", "-e", _TRACK_REFRACTION_LIFECYCLE_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "TRACK_REFRACTION": str(module)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "track refraction lifecycle ok" in completed.stdout


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
    assert 'view === "mapping"' in app_js
    assert "mappingLayers: 1000" in app_js
    assert "setInterval(() => refreshController.tick(), REFRESH_RATES_MS.schedulerTick)" in app_js
    assert "setInterval(() => pollMapping3d(false), 5000)" not in app_js


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

    assert 'timedGetJson("/api/pose")' in app_js
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
    scheduler_start = app_js.index("// --- Data refresh scheduler")
    scheduler_block = app_js[
        scheduler_start : app_js.index("// --- End data refresh scheduler", scheduler_start)
    ]
    publish_start = app_js.index("async function publishPose()")
    publish_block = app_js[publish_start : app_js.index("async function stopManagedTarget", publish_start)]

    assert "hideCompletedPaths: false" in app_js
    assert "state.activeGoal = null;" in clear_block
    assert "state.lastGoal = null;" in clear_block
    assert "refreshController.invalidatePaths();" in clear_block
    assert "state.paths = {};" in clear_block
    assert "state.hideCompletedPaths = true;" in clear_block
    assert "updatePaths();" in clear_block
    assert "pathsVisible: () => state.showPaths && !state.hideCompletedPaths" in app_js
    assert "refreshController.invalidatePaths();" in publish_block
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


def test_startup_card_owns_navigation_managed_process_toggles():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")

    readiness_start = index_html.index("<h2>Startup</h2>")
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
    assert 'id="mappingProcessToggle"' in index_html
    assert 'data-managed-target="mapping"' in index_html
    assert 'data-start-target="mapping"' not in index_html
    assert 'data-stop-target="mapping"' not in index_html


def test_navigation_page_puts_map_above_one_startup_card():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")

    navigation = index_html[
        index_html.index('id="navigationSidebar"') : index_html.index(
            'id="mappingSidebar"'
        )
    ]
    # All startup controls share one card below the Map selector.
    assert navigation.index("<h2>Map</h2>") < navigation.index(
        "<h2>Startup</h2>"
    )
    assert navigation.count('<section class="startup-card">') == 1
    card = navigation.split('<section class="startup-card">', 1)[1].split(
        "</section>", 1
    )[0]
    positions = []
    for target in ("chassis", "imu", "lidar", "localization", "navigation"):
        marker = f'data-managed-target="{target}"'
        assert marker in card, target
        positions.append(card.index(marker))
    assert positions == sorted(positions)
    assert 'id="navigationReady"' in card
    assert "data-step=" not in navigation


def test_mapping_page_shares_hardware_buttons_in_one_startup_card():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")

    mapping = index_html[index_html.index('id="mappingSidebar"') :]
    assert mapping.count('<section class="startup-card">') == 1
    card = mapping.split('<section class="startup-card">', 1)[1].split(
        "</section>", 1
    )[0]
    positions = []
    for target in ("chassis", "imu", "lidar", "mapping"):
        marker = f'data-managed-target="{target}"'
        assert marker in card, target
        positions.append(card.index(marker))

    assert positions == sorted(positions)
    assert card.count('data-managed-target="mapping"') == 1
    assert 'id="mappingProcessToggle"' in card
    assert 'data-start-target="mapping"' not in card
    assert 'data-stop-target="mapping"' not in card
    assert "data-step=" not in mapping

    # Hardware controls are shared by both pages: one backend target, mounted
    # once per page, with the same state surface and log output on each.
    for target in ("chassis", "imu", "lidar"):
        assert index_html.count(f'data-managed-target="{target}"') == 2, target
        assert index_html.count(f'data-log-output="{target}"') == 2, target
    assert index_html.count("data-managed-status") == 2


def test_pose_and_map_cards_use_compact_layout():
    index_html, app_js, styles_css = _frontend_sources()
    metrics = _css_rule_block(styles_css, ".metrics")
    # Coordinates sit two pairs per row with a tighter type scale.
    assert "grid-template-columns: auto minmax(0, 1fr) auto minmax(0, 1fr);" in metrics
    assert "gap: 3px 10px;" in metrics
    assert "font-size: 12.5px;" in metrics

    assert "grid-template-columns: auto minmax(0, 1fr);" in styles_css[
        styles_css.index("@media (max-width: 520px)") :
    ]

    picker_button = _css_rule_block(styles_css, ".map-picker button")
    assert "min-height: 30px;" in picker_button
    assert "padding: 0 10px;" in picker_button

    result = _css_rule_block(styles_css, ".result")
    assert "min-height: 18px;" in result
    assert "font-size: 12.5px;" in result

    # The named cards opt into the compact treatment.
    for heading in ("Map", "Current Pose", "Target Pose"):
        section = index_html[: index_html.index(f"<h2>{heading}</h2>")]
        opening = section[section.rindex("<section") :]
        assert "compact-card" in opening, heading


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
const chassisNavButton = makeElement();
const chassisMapButton = makeElement();
const imuNavButton = makeElement();
const imuMapButton = makeElement();
const lidarNavButton = makeElement();
const lidarMapButton = makeElement();
const mappingButton = makeElement();
const elements = {
  localizationProcessToggle: localizationButton,
  navigationProcessToggle: navigationButton,
  mappingProcessToggle: mappingButton,
};
const managedButtons = {
  chassis: [chassisNavButton, chassisMapButton],
  imu: [imuNavButton, imuMapButton],
  lidar: [lidarNavButton, lidarMapButton],
  localization: [localizationButton],
  navigation: [navigationButton],
  mapping: [mappingButton],
};
global.document = {
  getElementById(id) { return elements[id] || null; },
  querySelectorAll(selector) {
    const match = /\[data-managed-target="([^"]+)"\]/.exec(selector);
    if (!match) return [];
    return managedButtons[match[1]] || [];
  },
};

const state = {
  localizationRunning: false,
  navigationRunning: false,
  chassisRunning: false,
  imuRunning: false,
  lidarRunning: false,
  mappingRunning: false,
  managedProcessPending: {
    chassis: null,
    imu: null,
    lidar: null,
    localization: null,
    navigation: null,
    mapping: null,
  },
};

function clearNavigationSceneState() {}
function clearNavigationLayers() {}
function invalidateNavigationMapCaches() {}
function clearMapSceneState() {}

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
let resolveMappingStart = null;
let resolveMappingStop = null;

function startManagedTarget(target) {
  startCalls[target] = (startCalls[target] || 0) + 1;
  if (target === "navigation") {
    return new Promise((resolve) => { resolveNavigationStart = resolve; });
  }
  if (target === "mapping") {
    return new Promise((resolve) => { resolveMappingStart = resolve; });
  }
  return Promise.resolve(true);
}

function stopManagedTarget(target) {
  stopCalls[target] = (stopCalls[target] || 0) + 1;
  if (target === "navigation") return Promise.resolve(navigationStopResult);
  if (target === "mapping") {
    return new Promise((resolve) => { resolveMappingStop = resolve; });
  }
  return Promise.resolve(true);
}

eval(source.match(/const MANAGED_PROCESS_LABELS = \{[\s\S]*?\};/)[0]
  + "\n" + jsFunctionBody("function setManagedProcessRunning")
  + "\n" + jsFunctionBody("function managedProcessRunning")
  + "\n" + jsFunctionBody("function managedProcessButtons")
  + "\n" + jsFunctionBody("function updateManagedProcessButton")
  + "\n" + jsFunctionBody("async function toggleManagedTarget"));

function expect(condition, message) { if (!condition) throw new Error(message); }

expect(source.includes("[data-managed-target]"), "toggle event selector missing");
expect(!source.includes("[data-start-target]"), "legacy mapping start selector still bound");
expect(!source.includes("[data-stop-target]"), "legacy mapping stop selector still bound");

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

  // Hardware toggles are mounted on both pages and share one backend process:
  // one click must drive the single shared start while both buttons follow.
  updateManagedProcessButton("chassis");
  expect(chassisNavButton.textContent === "Start hunter", "navigation chassis start label");
  expect(chassisMapButton.textContent === "Start hunter", "mapping chassis start label");
  const chassisPending = toggleManagedTarget("chassis");
  expect(chassisNavButton.disabled === true, "navigation chassis disabled while pending");
  expect(chassisMapButton.disabled === true, "mapping chassis disabled while pending");
  expect(startCalls.chassis === 1, "shared chassis start posted once");
  await chassisPending;
  expect(state.chassisRunning === true, "chassis running after shared start");
  expect(chassisNavButton.textContent === "Stop hunter", "navigation chassis stop label");
  expect(chassisMapButton.textContent === "Stop hunter", "mapping chassis stop label");
  expect(chassisNavButton.getAttribute("aria-pressed") === "true", "navigation chassis aria-pressed");
  expect(chassisMapButton.getAttribute("aria-pressed") === "true", "mapping chassis aria-pressed");

  // Mapping is a single managed toggle: the same button drives start and stop,
  // and its style follows the requested action while a request is in flight,
  // not just the running flag returned by the log poll.
  updateManagedProcessButton("mapping");
  expect(mappingButton.textContent === "Start Mapping", "initial mapping start label");
  expect(mappingButton.classList.contains("start-log"), "initial mapping start style");
  expect(!mappingButton.classList.contains("stop-log"), "initial mapping not stop style");
  expect(mappingButton.getAttribute("aria-pressed") === "false", "initial mapping aria-pressed");
  expect(mappingButton.getAttribute("aria-busy") === "false", "initial mapping aria-busy");

  const mappingPending = toggleManagedTarget("mapping");
  expect(mappingButton.disabled === true, "pending mapping disabled");
  expect(mappingButton.textContent === "Starting Mapping...", "pending mapping label");
  expect(mappingButton.classList.contains("start-log"), "pending start keeps start style");
  expect(!mappingButton.classList.contains("stop-log"), "pending start not stop style");
  expect(mappingButton.getAttribute("aria-busy") === "true", "pending mapping aria-busy");
  toggleManagedTarget("mapping");
  expect(startCalls.mapping === 1, "duplicate mapping start blocked");
  // Color follows the running state even while the start request is pending.
  state.mappingRunning = true;
  updateManagedProcessButton("mapping");
  expect(mappingButton.textContent === "Starting Mapping...", "pending start label survives poll");
  expect(!mappingButton.classList.contains("start-log"), "running mapping leaves start style while pending");
  expect(mappingButton.classList.contains("stop-log"), "running mapping is red while start is pending");
  resolveMappingStart(true);
  await mappingPending;
  expect(state.mappingRunning === true, "mapping running after start success");
  expect(mappingButton.textContent === "Stop Mapping", "mapping stop label after start");
  expect(mappingButton.classList.contains("stop-log"), "mapping stop style after start");
  expect(mappingButton.getAttribute("aria-pressed") === "true", "mapping aria-pressed after start");
  expect(mappingButton.disabled === false, "mapping re-enabled after start");

  const mappingStopPending = toggleManagedTarget("mapping");
  expect(mappingButton.textContent === "Stopping Mapping...", "pending stop label");
  expect(mappingButton.classList.contains("stop-log"), "pending stop keeps stop style");
  expect(!mappingButton.classList.contains("start-log"), "pending stop not start style");
  toggleManagedTarget("mapping");
  expect(stopCalls.mapping === 1, "duplicate mapping stop blocked");
  // Once polling confirms stopped, the pending request does not keep it red.
  state.mappingRunning = false;
  updateManagedProcessButton("mapping");
  expect(mappingButton.textContent === "Stopping Mapping...", "pending stop label survives poll");
  expect(!mappingButton.classList.contains("stop-log"), "stopped mapping is not red while stop is pending");
  expect(mappingButton.classList.contains("start-log"), "stopped mapping returns to start style while pending");
  resolveMappingStop(true);
  await mappingStopPending;
  expect(state.mappingRunning === false, "mapping stopped after stop success");
  expect(mappingButton.textContent === "Start Mapping", "mapping returns to start after poll");
  expect(mappingButton.classList.contains("start-log"), "mapping start style after stop");
  expect(mappingButton.getAttribute("aria-pressed") === "false", "mapping aria-pressed after stop");
  expect(mappingButton.disabled === false, "mapping re-enabled after stop");

  // Failed stop keeps the running/stop state; failed start stays on start.
  state.mappingRunning = true;
  updateManagedProcessButton("mapping");
  const failedStop = toggleManagedTarget("mapping");
  resolveMappingStop(false);
  await failedStop;
  expect(state.mappingRunning === true, "failed stop keeps mapping running");
  expect(mappingButton.textContent === "Stop Mapping", "failed stop keeps stop label");
  expect(mappingButton.classList.contains("stop-log"), "failed stop keeps stop style");
  expect(mappingButton.getAttribute("aria-pressed") === "true", "failed stop keeps aria-pressed");

  state.mappingRunning = false;
  updateManagedProcessButton("mapping");
  const failedStart = toggleManagedTarget("mapping");
  resolveMappingStart(false);
  await failedStart;
  expect(state.mappingRunning === false, "failed start keeps mapping stopped");
  expect(mappingButton.textContent === "Start Mapping", "failed start keeps start label");
  expect(mappingButton.classList.contains("start-log"), "failed start keeps start style");
  expect(mappingButton.getAttribute("aria-pressed") === "false", "failed start keeps aria-pressed");

  // All six managed targets share one class contract: stop-log while running,
  // start-log otherwise, on every mounted button.
  for (const [target, targetButtons] of Object.entries(managedButtons)) {
    setManagedProcessRunning(target, true);
    updateManagedProcessButton(target);
    for (const button of targetButtons) {
      expect(button.classList.contains("stop-log"), target + " running is stop-log");
      expect(!button.classList.contains("start-log"), target + " running not start-log");
    }
    setManagedProcessRunning(target, false);
    updateManagedProcessButton(target);
    for (const button of targetButtons) {
      expect(button.classList.contains("start-log"), target + " idle is start-log");
      expect(!button.classList.contains("stop-log"), target + " idle not stop-log");
    }
  }
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
                "chassis_launch_package": "hunter_base",
                "chassis_launch_file": "hunter_base.launch.py",
                "chassis_launch_args": "",
                "imu_launch_package": "fdilink_ahrs",
                "imu_launch_file": "ahrs_driver.launch.py",
                "imu_launch_args": "",
                "lidar_launch_package": "hesai_ros_driver",
                "lidar_launch_file": "start.py",
                "lidar_launch_args": "",
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
                for target in (
                    "chassis",
                    "imu",
                    "lidar",
                    "localization",
                    "navigation",
                    "mapping",
                )
            }
            self.managed_processes = {}
            self.lock = threading.RLock()
            self.logger = _LoggerStub()
            self.configure_managed_processes = MethodType(
                web_control_node.WebControlNode.configure_managed_processes, self
            )
            self.handle_managed_exit = MethodType(
                web_control_node.WebControlNode.handle_managed_exit, self
            )
            self.managed_start_command = MethodType(
                web_control_node.WebControlNode.managed_start_command, self
            )
            self.start_managed_target = MethodType(
                web_control_node.WebControlNode.start_managed_target, self
            )

        def get_parameter(self, name):
            return _ParameterStub(self.parameters[name])

        def get_logger(self):
            return self.logger

        def now_sec(self):
            return 10.0

    node = _LaunchNodeStub()
    node.configure_managed_processes()
    return node


def test_managed_launch_commands_add_no_implicit_use_sim():
    node = _managed_launch_node()
    commands = {
        target: process.command for target, process in node.managed_processes.items()
    }

    assert commands == {
        "chassis": ["ros2", "launch", "hunter_base", "hunter_base.launch.py"],
        "imu": ["ros2", "launch", "fdilink_ahrs", "ahrs_driver.launch.py"],
        "lidar": ["ros2", "launch", "hesai_ros_driver", "start.py"],
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
    assert list(commands) == [
        "chassis",
        "imu",
        "lidar",
        "localization",
        "navigation",
        "mapping",
    ]
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


def test_managed_launch_stale_reader_does_not_clobber_new_round(monkeypatch):
    from web_control.web_control_node import ManagedLaunchProcess

    class _FakeReaderProcess:
        def __init__(self, pid, gate=None, lines=()):
            self.pid = pid
            self.returncode = 0
            self._gate = gate
            self._lines = list(lines)

        @property
        def stdout(self):
            return iter(self._lines)

        def wait(self, timeout=None):
            if self._gate is not None:
                self._gate.wait(timeout=5)
            return self.returncode

        def poll(self):
            return self.returncode

    exits = []
    logs = TerminalLogBuffer("localization", max_lines=5)
    process = ManagedLaunchProcess(
        "localization",
        ["ros2", "launch", "lio_sam_hesai", "x.launch.py"],
        logs,
        _LoggerStub(),
        on_exit=exits.append,
    )

    old_gate = threading.Event()
    old_process = _FakeReaderProcess(111, gate=old_gate, lines=["old line\n"])
    process.generation = 1
    old_reader = threading.Thread(
        target=process._read_output, args=(old_process, 1), daemon=True
    )
    old_reader.start()

    # A fresh launch takes over while the old reader is still draining stdout.
    process.generation = 2
    logs.set_process_state(enabled=True, running=True, returncode=None)

    old_gate.set()
    old_reader.join(timeout=5)

    assert not old_reader.is_alive()
    # The stale reader must not flip the new round to stopped, nor fire the
    # exit callback that would clear a running target's runtime state.
    assert logs.snapshot()["running"] is True
    assert exits == []

    # The current round still settles normally.
    process._read_output(_FakeReaderProcess(222), 2)
    assert logs.snapshot()["running"] is False
    assert exits == ["localization"]


class _FakePreflightProcess:
    """Stand-in for a prerequisite Popen that never spawns a real command."""

    pid = 4242

    def __init__(self, argv, returncode=0, output="", gate=None, stderr=""):
        self.argv = [str(item) for item in argv]
        self.args = self.argv
        self.returncode = returncode
        self.output = output
        self.stderr = stderr
        self._gate = gate
        self._finished = gate is None
        self.signals = []
        self.killed = False

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        return False

    def communicate(self, input=None, timeout=None):
        if self._gate is not None:
            self._gate.wait(timeout=5)
        self._finished = True
        return self.output, self.stderr

    def poll(self):
        return self.returncode if self._finished else None

    def wait(self, timeout=None):
        return self.returncode

    def send_signal(self, sig):
        self.signals.append(sig)

    def kill(self):
        self.killed = True


class _PreflightSpawnRecorder:
    """subprocess.Popen replacement recording the chassis prerequisite order."""

    def __init__(
        self,
        returncodes=None,
        outputs=None,
        gate=None,
        popen_gate=None,
        errors=None,
    ):
        self.returncodes = returncodes or {}
        self.outputs = outputs or {}
        # ``gate`` keeps the spawned command running inside ``communicate``;
        # ``popen_gate`` freezes the spawn call itself so a test can drive the
        # cancelled-check/Popen/registration window deterministically.
        self.gate = gate
        self.popen_gate = popen_gate
        self.errors = errors or {}
        self.calls = []
        self.first_spawn = threading.Event()

    def __call__(self, argv, **kwargs):
        argv = tuple(str(item) for item in argv)
        self.calls.append(argv)
        self.first_spawn.set()
        if argv in self.errors:
            raise self.errors[argv]
        if self.popen_gate is not None:
            self.popen_gate.wait(timeout=5)
        return _FakePreflightProcess(
            argv,
            returncode=self.returncodes.get(argv, 0),
            output=self.outputs.get(argv, ""),
            gate=self.gate,
        )


def _chassis_sequence(monkeypatch, recorders, launched, exits=None):
    from web_control.web_control_node import (
        CHASSIS_CAN_PREFLIGHT_COMMANDS,
        ManagedSequenceLaunchProcess,
    )

    monkeypatch.setattr(web_control_node.subprocess, "Popen", recorders)
    monkeypatch.setattr(
        web_control_node.ManagedLaunchProcess,
        "_start_locked",
        lambda self: launched.append(list(self.command)),
    )
    logs = TerminalLogBuffer("chassis", max_lines=50)
    process = ManagedSequenceLaunchProcess(
        "chassis",
        [list(step) for step in CHASSIS_CAN_PREFLIGHT_COMMANDS],
        ["ros2", "launch", "hunter_base", "hunter_base.launch.py"],
        logs,
        _LoggerStub(),
        on_exit=exits.append if exits is not None else None,
    )
    return process, logs


def _can0_verification_json(
    *,
    flags=("NOARP", "ECHO", "UP", "LOWER_UP"),
    link_type="can",
    info_kind="can",
    bitrate=500000,
):
    """Build the JSON shape of ``ip -j -details link show can0``."""
    info_data = {"state": "ERROR-ACTIVE" if "UP" in flags else "STOPPED"}
    if bitrate is not None:
        info_data["bitrate"] = bitrate
    return json.dumps(
        [
            {
                "ifindex": 6,
                "ifname": "can0",
                "flags": list(flags),
                "mtu": 16,
                "operstate": "UP" if "UP" in flags else "DOWN",
                "link_type": link_type,
                "linkinfo": {
                    "info_kind": info_kind,
                    "info_data": info_data,
                },
            }
        ]
    )


def test_chassis_preflight_commands_are_non_interactive_and_ordered():
    from web_control.web_control_node import (
        CHASSIS_CAN_PREFLIGHT_COMMANDS,
        CHASSIS_CAN_VERIFY_COMMAND,
    )

    assert CHASSIS_CAN_VERIFY_COMMAND == (
        "ip",
        "-j",
        "-details",
        "link",
        "show",
        "can0",
    )
    assert "sudo" not in CHASSIS_CAN_VERIFY_COMMAND
    assert CHASSIS_CAN_PREFLIGHT_COMMANDS == (
        ("sudo", "-n", "modprobe", "can"),
        (
            "sudo",
            "-n",
            "ip",
            "link",
            "set",
            "can0",
            "up",
            "type",
            "can",
            "bitrate",
            "500000",
        ),
    )


def test_chassis_sequence_runs_preflight_in_order_then_launches(monkeypatch):
    from web_control.web_control_node import CHASSIS_CAN_PREFLIGHT_COMMANDS

    launched = []
    recorder = _PreflightSpawnRecorder()
    process, logs = _chassis_sequence(monkeypatch, recorder, launched)

    process.start()
    process.sequence_thread.join(timeout=5)

    assert recorder.calls == [tuple(step) for step in CHASSIS_CAN_PREFLIGHT_COMMANDS]
    assert launched == [["ros2", "launch", "hunter_base", "hunter_base.launch.py"]]
    assert logs.snapshot()["running"] is True
    assert logs.snapshot()["returncode"] is None


def test_chassis_sequence_aborts_before_ros_launch_on_first_preflight_failure(
    monkeypatch,
):
    from web_control.web_control_node import CHASSIS_CAN_PREFLIGHT_COMMANDS

    first = tuple(CHASSIS_CAN_PREFLIGHT_COMMANDS[0])
    launched = []
    exits = []
    recorder = _PreflightSpawnRecorder(
        returncodes={first: 1},
        outputs={first: "sudo: a password is required"},
    )
    process, logs = _chassis_sequence(monkeypatch, recorder, launched, exits)

    process.start()
    process.sequence_thread.join(timeout=5)

    assert recorder.calls == [first]
    assert launched == []
    snapshot = logs.snapshot()
    assert snapshot["running"] is False
    assert snapshot["returncode"] == 1
    assert any("not starting chassis" in line for line in snapshot["lines"])
    assert any("password is required" in line for line in snapshot["lines"])
    assert exits == ["chassis"]


def test_chassis_sequence_aborts_before_ros_launch_on_second_preflight_failure(
    monkeypatch,
):
    from web_control.web_control_node import CHASSIS_CAN_PREFLIGHT_COMMANDS

    first, second = (
        tuple(CHASSIS_CAN_PREFLIGHT_COMMANDS[0]),
        tuple(CHASSIS_CAN_PREFLIGHT_COMMANDS[1]),
    )
    launched = []
    recorder = _PreflightSpawnRecorder(returncodes={second: 2})
    process, logs = _chassis_sequence(monkeypatch, recorder, launched)

    process.start()
    process.sequence_thread.join(timeout=5)

    assert recorder.calls == [first, second]
    assert launched == []
    assert logs.snapshot()["returncode"] == 2


def test_chassis_sequence_start_stop_start_reuses_interface_left_up(monkeypatch):
    """A stopped chassis leaves can0 up; a verified restart must not abort."""
    from web_control.web_control_node import (
        CHASSIS_CAN_PREFLIGHT_COMMANDS,
        CHASSIS_CAN_VERIFY_COMMAND,
    )

    can_up = tuple(CHASSIS_CAN_PREFLIGHT_COMMANDS[1])
    verify = tuple(CHASSIS_CAN_VERIFY_COMMAND)
    interface_up = False

    class _InterfaceStateRecorder:
        def __init__(self):
            self.calls = []

        def __call__(self, argv, **_kwargs):
            nonlocal interface_up
            argv = tuple(str(item) for item in argv)
            self.calls.append(argv)
            if argv == verify:
                return _FakePreflightProcess(
                    argv, output=_can0_verification_json()
                )
            if argv != can_up:
                return _FakePreflightProcess(argv)
            if interface_up:
                # Real ``ip`` returns this when can0 is already administratively
                # up; the previous managed stop intentionally left it up.
                return _FakePreflightProcess(
                    argv,
                    returncode=2,
                    output="RTNETLINK answers: Device or resource busy\n",
                )
            interface_up = True
            return _FakePreflightProcess(argv)

    launched = []
    recorder = _InterfaceStateRecorder()
    process, logs = _chassis_sequence(monkeypatch, recorder, launched)

    process.start()
    process.sequence_thread.join(timeout=5)
    process.stop()

    process.start()
    process.sequence_thread.join(timeout=5)

    assert recorder.calls.count(can_up) == 2
    assert recorder.calls.count(verify) == 1
    assert launched == [
        ["ros2", "launch", "hunter_base", "hunter_base.launch.py"],
        ["ros2", "launch", "hunter_base", "hunter_base.launch.py"],
    ]
    assert logs.snapshot()["running"] is True
    assert any("verified UP/CAN/500000" in line for line in logs.snapshot()["lines"])


@pytest.mark.parametrize(
    "verification_output",
    [
        pytest.param(
            _can0_verification_json(bitrate=250000), id="wrong-bitrate"
        ),
        pytest.param(
            _can0_verification_json(flags=("NOARP", "ECHO")), id="interface-down"
        ),
        pytest.param(
            _can0_verification_json(link_type="ether", info_kind="ether"),
            id="wrong-type",
        ),
        pytest.param(_can0_verification_json(bitrate=None), id="missing-bitrate"),
        pytest.param("not-json", id="invalid-json"),
    ],
)
def test_chassis_sequence_ebusy_verification_failure_aborts(
    monkeypatch, verification_output
):
    from web_control.web_control_node import (
        CHASSIS_CAN_PREFLIGHT_COMMANDS,
        CHASSIS_CAN_VERIFY_COMMAND,
    )

    first = tuple(CHASSIS_CAN_PREFLIGHT_COMMANDS[0])
    second = tuple(CHASSIS_CAN_PREFLIGHT_COMMANDS[1])
    verify = tuple(CHASSIS_CAN_VERIFY_COMMAND)
    launched = []
    recorder = _PreflightSpawnRecorder(
        returncodes={second: 2},
        outputs={
            second: "RTNETLINK answers: Device or resource busy\n",
            verify: verification_output,
        },
    )
    process, logs = _chassis_sequence(monkeypatch, recorder, launched)

    process.start()
    process.sequence_thread.join(timeout=5)

    assert recorder.calls == [first, second, verify]
    assert launched == []
    snapshot = logs.snapshot()
    assert snapshot["running"] is False
    assert snapshot["returncode"] == 2
    lines = snapshot["lines"]
    assert any("not starting chassis" in line for line in lines)
    assert any("EBUSY is not enough" in line for line in lines)


def test_chassis_sequence_ebusy_verification_command_unavailable_aborts(
    monkeypatch,
):
    from web_control.web_control_node import (
        CHASSIS_CAN_PREFLIGHT_COMMANDS,
        CHASSIS_CAN_VERIFY_COMMAND,
    )

    first = tuple(CHASSIS_CAN_PREFLIGHT_COMMANDS[0])
    second = tuple(CHASSIS_CAN_PREFLIGHT_COMMANDS[1])
    verify = tuple(CHASSIS_CAN_VERIFY_COMMAND)
    launched = []
    recorder = _PreflightSpawnRecorder(
        returncodes={second: 2},
        outputs={second: "RTNETLINK answers: Device or resource busy\n"},
        errors={verify: FileNotFoundError("ip")},
    )
    process, logs = _chassis_sequence(monkeypatch, recorder, launched)

    process.start()
    process.sequence_thread.join(timeout=5)

    assert recorder.calls == [first, second, verify]
    assert launched == []
    assert logs.snapshot()["returncode"] == 2
    assert any(
        "read-only ip query is unavailable" in line
        for line in logs.snapshot()["lines"]
    )


def test_chassis_sequence_reports_missing_preflight_command(monkeypatch):
    launched = []

    def raise_missing(argv, **kwargs):
        raise FileNotFoundError("sudo")

    monkeypatch.setattr(web_control_node.subprocess, "Popen", raise_missing)
    monkeypatch.setattr(
        web_control_node.ManagedLaunchProcess,
        "_start_locked",
        lambda self: launched.append(list(self.command)),
    )
    logs = TerminalLogBuffer("chassis", max_lines=50)
    process = web_control_node.ManagedSequenceLaunchProcess(
        "chassis",
        [list(step) for step in web_control_node.CHASSIS_CAN_PREFLIGHT_COMMANDS],
        ["ros2", "launch", "hunter_base", "hunter_base.launch.py"],
        logs,
        _LoggerStub(),
    )

    process.start()
    process.sequence_thread.join(timeout=5)

    assert launched == []
    assert logs.snapshot()["returncode"] == 127


def test_chassis_sequence_repeated_start_does_not_duplicate(monkeypatch):
    gate = threading.Event()
    launched = []
    recorder = _PreflightSpawnRecorder(gate=gate)
    process, logs = _chassis_sequence(monkeypatch, recorder, launched)

    process.start()
    assert process.is_running() is True
    # Wait until the first prerequisite has actually been spawned before
    # issuing the duplicate starts; otherwise the assertion below races the
    # sequence thread's first scheduling.
    assert recorder.first_spawn.wait(timeout=5)
    process.start()
    process.start()

    # While the first prerequisite is still in flight only one sequence exists.
    assert len(recorder.calls) == 1
    gate.set()
    process.sequence_thread.join(timeout=5)
    # Exactly the two prerequisites ran, in order, with no duplicate sequence.
    assert recorder.calls == [
        ("sudo", "-n", "modprobe", "can"),
        (
            "sudo",
            "-n",
            "ip",
            "link",
            "set",
            "can0",
            "up",
            "type",
            "can",
            "bitrate",
            "500000",
        ),
    ]
    assert launched == [["ros2", "launch", "hunter_base", "hunter_base.launch.py"]]


def test_chassis_sequence_stop_cancels_inflight_preflight(monkeypatch):
    gate = threading.Event()
    launched = []
    exits = []
    recorder = _PreflightSpawnRecorder(gate=gate)
    process, logs = _chassis_sequence(monkeypatch, recorder, launched, exits)
    killpg_calls = []

    def fake_killpg(pgid, sig):
        killpg_calls.append((pgid, sig))
        # Release the fake prerequisite once its group is signalled, mirroring
        # a real command observing SIGINT.
        gate.set()

    monkeypatch.setattr(web_control_node.os, "killpg", fake_killpg)
    monkeypatch.setattr(web_control_node.os, "getpgid", lambda pid: 5000)

    process.start()
    assert recorder.first_spawn.wait(timeout=5)
    process.stop()
    process.sequence_thread.join(timeout=5)

    # The whole prerequisite process group is signalled, not just the leader,
    # and the sequence state is reset before stop returns.
    assert killpg_calls == [(5000, signal.SIGINT)]
    assert launched == []
    assert logs.snapshot()["running"] is False
    assert process.sequence_active is False
    assert process.is_running() is False
    assert exits == ["chassis"]
    assert not process.sequence_thread.is_alive()


def test_chassis_sequence_stop_closes_the_spawn_registration_window(monkeypatch):
    popen_gate = threading.Event()
    popen_entered = threading.Event()
    comm_gate = threading.Event()
    launched = []
    exits = []
    calls = []

    class SlowSpawnRecorder:
        def __call__(self, argv, **kwargs):
            argv = tuple(str(item) for item in argv)
            calls.append(argv)
            popen_entered.set()
            popen_gate.wait(timeout=5)
            return _FakePreflightProcess(argv, gate=comm_gate)

    process, logs = _chassis_sequence(
        monkeypatch, SlowSpawnRecorder(), launched, exits
    )
    killpg_calls = []

    def fake_killpg(pgid, sig):
        killpg_calls.append((pgid, sig))
        # Release the running prerequisite once its group is signalled.
        comm_gate.set()

    monkeypatch.setattr(web_control_node.os, "getpgid", lambda pid: 6000)
    monkeypatch.setattr(web_control_node.os, "killpg", fake_killpg)

    process.start()
    assert popen_entered.wait(timeout=5)

    stopper = threading.Thread(target=process.stop)
    stopper.start()
    # stop must wait out the spawn window instead of returning early while a
    # just-forked prerequisite is not yet registered for shutdown.
    stopper.join(timeout=0.3)
    assert stopper.is_alive()

    popen_gate.set()
    stopper.join(timeout=5)
    assert not stopper.is_alive()

    # The registered command was group-signalled, the sequence never advanced to
    # the second step, and nothing was left running after stop returned.
    assert calls == [("sudo", "-n", "modprobe", "can")]
    assert killpg_calls == [(6000, signal.SIGINT)]
    assert launched == []
    assert logs.snapshot()["running"] is False
    assert process.sequence_active is False
    assert process.prerequisite_process is None
    assert exits == ["chassis"]


def test_chassis_sequence_stop_closes_cancelled_session_when_new_round_starts(
    monkeypatch, tmp_path
):
    from web_control.log_store import ManagedLogStore
    from web_control.web_control_node import ManagedSequenceLaunchProcess

    store = ManagedLogStore(tmp_path / "logs")
    logs = TerminalLogBuffer("chassis", max_lines=50)
    exits = []
    process = ManagedSequenceLaunchProcess(
        "chassis",
        [["sudo", "-n", "modprobe", "can"]],
        ["ros2", "launch", "hunter_base", "hunter_base.launch.py"],
        logs,
        _LoggerStub(),
        on_exit=exits.append,
        log_store=store,
    )

    new_sessions = []

    def start_new_round_while_settling(prerequisite, done, pgid):
        # A second HTTP round wins the race after ``stop`` released the lock but
        # before it settles: it opens its own session and claims the runtime.
        with process.lock:
            process._begin_session_locked()
            new_sessions.append(process.session)
            process.sequence_generation += 1
            process.sequence_active = True
            process.sequence_cancelled = False
            process.log_buffer.set_process_state(
                enabled=True, running=True, returncode=None
            )

    monkeypatch.setattr(
        process, "_terminate_prerequisite", start_new_round_while_settling
    )

    with process.lock:
        process._begin_session_locked()
        old_session = process.session
        process.sequence_active = True
        process.sequence_generation += 1
        process.prerequisite_done = threading.Event()
        process.log_buffer.set_process_state(
            enabled=True, running=True, returncode=None
        )

    process.stop()

    new_session = new_sessions[0]
    assert old_session is not new_session
    # The cancelled round's file handle must always be released...
    assert old_session.closed is True
    assert process.session is new_session
    # ...while the newer round's session and running state stay untouched.
    assert new_session.closed is False
    assert logs.snapshot()["running"] is True
    assert exits == []
    active = {
        row["session"]: row["active"] for row in store.list_sessions("chassis")
    }
    assert active[old_session.session_id] is False
    assert active[new_session.session_id] is True


def test_managed_node_configures_hardware_targets_in_startup_order(monkeypatch):
    node = _managed_launch_node()
    from web_control.web_control_node import (
        CHASSIS_CAN_PREFLIGHT_COMMANDS,
        ManagedSequenceLaunchProcess,
    )

    assert list(node.managed_logs) == [
        "chassis",
        "imu",
        "lidar",
        "localization",
        "navigation",
        "mapping",
    ]
    assert list(node.managed_processes) == [
        "chassis",
        "imu",
        "lidar",
        "localization",
        "navigation",
        "mapping",
    ]
    chassis = node.managed_processes["chassis"]
    assert isinstance(chassis, ManagedSequenceLaunchProcess)
    assert chassis.prerequisites == [
        list(step) for step in CHASSIS_CAN_PREFLIGHT_COMMANDS
    ]
    assert chassis.command == ["ros2", "launch", "hunter_base", "hunter_base.launch.py"]

    launched = []
    recorder = _PreflightSpawnRecorder()
    monkeypatch.setattr(web_control_node.subprocess, "Popen", recorder)
    monkeypatch.setattr(
        web_control_node.ManagedLaunchProcess,
        "_start_locked",
        lambda self: launched.append(list(self.command)),
    )
    response = node.start_managed_target("chassis")
    chassis.sequence_thread.join(timeout=5)

    assert response["ok"] is True
    assert [tuple(step) for step in recorder.calls] == [
        tuple(step) for step in CHASSIS_CAN_PREFLIGHT_COMMANDS
    ]
    assert launched == [["ros2", "launch", "hunter_base", "hunter_base.launch.py"]]


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


def test_frontend_mounts_viewports_and_uses_inert():
    index_html, app_js, styles_css = _frontend_sources()

    assert 'id="view3d"' in index_html
    assert 'id="viewSystem"' in index_html
    assert index_html.index('id="view3d"') < index_html.index('id="viewSystem"')

    viewport = _css_rule_block(styles_css, ".viewport")
    assert "display: none;" not in viewport
    assert "position: absolute;" in viewport
    assert "opacity: 0;" in viewport
    assert "visibility: hidden;" in viewport
    assert "pointer-events: none;" in viewport

    active = _css_rule_block(styles_css, ".viewport.active")
    assert "display: block;" not in active
    assert "opacity: 1;" in active
    assert "visibility: visible;" in active
    assert "pointer-events: auto;" in active

    assert "view3d.inert = !isMapView;" in app_js
    assert 'view3d.setAttribute("aria-hidden", String(!isMapView));' in app_js
    assert 'viewSystem.inert = tab !== "system";' in app_js
    assert 'viewSystem.setAttribute("aria-hidden", String(tab !== "system"));' in app_js
    assert "panel.inert = !isActive;" in app_js
    assert 'panel.setAttribute("aria-hidden", String(!isActive));' in app_js


def test_frontend_view_motion_css_contract():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    viewport = _css_rule_block(styles_css, ".viewport")
    assert "transform" in viewport
    assert "var(--layout-ease)" in viewport
    assert "var(--fade-ms)" in viewport
    assert "filter" not in viewport

    active = _css_rule_block(styles_css, ".viewport.active")
    assert "translate3d(0, 0, 0) scale(1)" in active
    assert "var(--layout-ease)" in active
    assert "var(--fade-ms)" in active
    assert "filter" not in active

    inactive_3d = _css_rule_block(styles_css, "#view3d:not(.active)")
    assert "translate3d(-18px, 0, 0)" in inactive_3d
    assert "scale(0.992)" in inactive_3d
    assert "filter" not in inactive_3d

    inactive_system = _css_rule_block(styles_css, "#viewSystem:not(.active)")
    assert "translate3d(24px, 0, 0)" in inactive_system
    assert "scale(0.985)" in inactive_system
    assert "filter" not in inactive_system

    root = _css_rule_block(styles_css, ":root")
    assert "--layout-slide-ms: 650ms;" in root
    assert "--fade-ms: 480ms;" in root
    assert "--thumb-ms: 560ms;" in root
    assert "--layout-ease: cubic-bezier(0.4, 0, 0.2, 1);" in root

    panel = _css_rule_block(styles_css, ".panel")
    assert "transform var(--fade-ms) var(--layout-ease)" in panel
    assert "opacity var(--fade-ms) var(--layout-ease)" in panel

    app = _css_rule_block(styles_css, ".app")
    assert "grid-template-columns var(--layout-slide-ms) var(--layout-ease)" in app


def test_glass_no_filter_on_webgl_canvas():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    canvas = _css_rule_block(styles_css, "#mapCanvas3d")
    assert "background-color: var(--scene-bg);" in canvas
    assert "backdrop-filter" not in canvas
    assert "filter:" not in canvas
    view3d = _css_rule_block(styles_css, "#view3d")
    assert "backdrop-filter" not in view3d
    assert "filter:" not in view3d
    assert "--scene-bg: #0b0f19;" in styles_css


def test_frontend_layout_transition_state_machine_static():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )

    assert "const LAYOUT_SETTLE_FALLBACK_MS = 780;" in app_js
    for name in (
        "beginLayoutTransition",
        "settleLayoutTransition",
        "noteLayoutResize",
        "doDeferredLayoutFlush",
        "attachLayoutTransitionListener",
    ):
        assert f"function {name}(" in app_js
    assert "pageTransition" in app_js
    assert 'window.addEventListener("resize", noteLayoutResize);' in app_js
    assert 'window.addEventListener("resize", resizeAll);' not in app_js
    assert 'window.addEventListener("resize", applyJoystickLayout);' not in app_js

    # Settlement is geometry-aware: a transition only trusts the app's own grid
    # event when it recorded a grid geometry change.
    assert "expectsGrid" in app_js
    assert "options.gridChanged" in app_js
    assert "beginLayoutTransition({ gridChanged: true })" in app_js
    assert "beginLayoutTransition({ gridChanged: sidebarChanged })" in app_js

    begin_block = app_js[
        app_js.index("function beginLayoutTransition(") : app_js.index(
            "function settleLayoutTransition("
        )
    ]
    # expectsGrid is sticky: a pure begin during an in-flight grid run keeps
    # the grid expectation until settleLayoutTransition clears it.
    assert "const existingGridInFlight" in begin_block
    assert "pageTransition.active && pageTransition.expectsGrid" in begin_block
    assert "existingGridInFlight || Boolean(options.gridChanged)" in begin_block

    settle_body = _js_function_body(app_js, "function settleLayoutTransition(token)")
    assert "pageTransition.expectsGrid = false;" in settle_body

    handler = _js_function_body(app_js, "function handleLayoutTransitionEnd(event)")
    assert 'event.propertyName !== "grid-template-columns"' in handler
    assert "event.pseudoElement" in handler
    assert "pageTransition.expectsGrid" in handler
    assert "pageTransition.token" in handler
    assert 'document.querySelector(".app")' in handler
    # Opacity/transform (viewport, panel, thumb) must never settle the layout.
    handler_code = "\n".join(line.split("//", 1)[0] for line in handler.splitlines())
    assert "opacity" not in handler_code
    assert "transform" not in handler_code

    attach = _js_function_body(app_js, "function attachLayoutTransitionListener()")
    assert 'appEl.addEventListener("transitionend", handleLayoutTransitionEnd);' in attach
    assert "view3d" not in attach
    assert "viewTabs" not in attach

    flush_body = _js_function_body(app_js, "function doDeferredLayoutFlush()")
    assert "flushPendingMappingJoystickReveal();" in flush_body

    assert app_js.count("renderer.setSize(") == 1
    resize_body = _js_function_body(app_js, "function resizeAll()")
    assert "renderer.setSize(" in resize_body

    # The live viewport sync is gated, delegates to the cached resizeAll, and
    # runs before the camera update and draw in the same frame.
    render_body = _js_function_body(app_js, "function render()")
    assert render_body.index("syncMapViewportDuringGridTransition();") < render_body.index(
        "updateCamera();"
    ) < render_body.index("renderer.render(scene, camera);")
    sync_body = _js_function_body(
        app_js, "function syncMapViewportDuringGridTransition()"
    )
    assert "pageTransition.active" in sync_body
    assert "pageTransition.expectsGrid" in sync_body
    assert "mapViewActive()" in sync_body
    assert "glassMotionReduced()" in sync_body
    assert "resizeAll();" in sync_body
    assert "renderer.setSize(" not in sync_body
    assert "resizeAll();" not in app_js[
        app_js.index("function setTab(tab)") : app_js.index("function selectedPosePayload")
    ]


def test_frontend_white_flash_fix_present():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )

    assert "const SCENE_BG = 0x0b0f19;" in app_js
    assert "scene.background = new THREE.Color(SCENE_BG);" in app_js
    assert "renderer.setClearColor(0x0b0f19, 1);" in app_js

    render_body = _js_function_body(app_js, "function render()")
    assert "mapViewActive()" in render_body
    assert "requestAnimationFrame(render)" in render_body
    assert render_body.index("syncMapViewportDuringGridTransition();") < render_body.index(
        "updateCamera();"
    ) < render_body.index("renderer.render(scene, camera);")
    assert "cancelAnimationFrame(mapRenderFrame);" in render_body
    assert "function mapViewActive()" in app_js
    assert "function ensureMapRenderLoop()" in app_js
    assert "function pauseMapRenderLoop()" in app_js

    flush_body = _js_function_body(app_js, "function doDeferredLayoutFlush()")
    assert "resizeAll();" in flush_body
    assert "applyJoystickLayout();" in flush_body
    assert "updateHudPlacement();" in flush_body
    assert "renderer.render(scene, camera);" in flush_body

    assert "--scene-bg: #0b0f19;" in styles_css
    assert "background-color: var(--scene-bg);" in styles_css


def test_frontend_safety_invariants_in_set_tab():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    set_tab_block = app_js[
        app_js.index("function setTab(tab)") : app_js.index("function selectedPosePayload")
    ]

    # Safety first: the hide path (which stops teleop) runs before the layout
    # transition is armed, while the show path waits until the sidebar geometry
    # is known and the transition is armed so a deferred reveal can be recorded.
    hide_index = set_tab_block.index("setMappingJoystickVisible(false);")
    sidebar_index = set_tab_block.index("const sidebarChanged = applyEffectiveSidebar();")
    begin_index = set_tab_block.index(
        "beginLayoutTransition({ gridChanged: sidebarChanged });"
    )
    show_index = set_tab_block.index(
        "setMappingJoystickVisible(true, { deferReveal: deferJoystickReveal });"
    )
    assert hide_index < sidebar_index
    assert sidebar_index < begin_index < show_index
    # A re-entrant Mapping click must treat an in-flight grid-expecting
    # transition as deferring, not just a sidebar change detected this click.
    assert (
        "const deferJoystickReveal =\n"
        "    sidebarChanged || (pageTransition.active && pageTransition.expectsGrid);"
        in set_tab_block
    )
    assert "exitJoystickLayout();" in set_tab_block
    assert "resizeAll();" not in set_tab_block
    assert "markGlassDrop(viewTabs);" in set_tab_block
    assert "if (applyEffectiveSidebar()) beginLayoutTransition({ gridChanged: true });" in app_js
    assert "function applyEffectiveSidebar()" in app_js

    sidebar_block = app_js[
        app_js.index("// --- Sidebar collapse") : app_js.index("// --- End sidebar collapse")
    ]
    assert "localStorage" not in sidebar_block
    assert "sessionStorage" not in sidebar_block
    assert "function persistJoystickPreferences()" in app_js
    assert 'postJson("/api/ui/preferences"' in app_js
    assert "function selectMap(" in app_js


def test_frontend_sidebar_and_system_use_shared_collapse():
    index_html, app_js, styles_css = _frontend_sources()

    assert '<div class="panel-views">' in index_html
    assert index_html.index('class="panel-views"') < index_html.index('id="navigationSidebar"')
    panel_views = _css_rule_block(styles_css, ".panel-views")
    assert "position: relative;" in panel_views

    panel_view = _css_rule_block(styles_css, ".panel-view")
    assert "display: none;" not in panel_view
    assert "opacity: 0;" in panel_view
    active_panel_view = _css_rule_block(styles_css, ".panel-view.active")
    assert "display: block;" not in active_panel_view
    assert "position: relative;" in active_panel_view
    inactive_panel_view = _css_rule_block(styles_css, ".panel-view:not(.active)")
    assert "position: absolute;" in inactive_panel_view
    assert "inset: 0;" in inactive_panel_view

    combined_start = styles_css.index(".app.sidebar-collapsed .panel,")
    combined_body = styles_css[
        styles_css.index("{", combined_start) + 1 : styles_css.index("}", combined_start)
    ]
    assert "opacity: 0;" in combined_body
    assert "transform: translateX(12px);" in combined_body
    assert "pointer-events: none;" in combined_body

    toggle_start = styles_css.index(".app.system-mode .sidebar-toggle {")
    toggle_body = styles_css[toggle_start : styles_css.index("}", toggle_start)]
    assert "display: none;" not in toggle_body
    assert "opacity: 0;" in toggle_body
    assert "visibility: hidden;" in toggle_body

    assert 'const effective = sidebarState.collapsed || state.view === "system";' in app_js
    assert "function syncSidebarToggle(toggle)" in app_js
    assert "let appliedEffectiveCollapse = false;" in app_js


def test_system_tab_uses_shared_sidebar_collapse_model():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    set_tab_start = app_js.index("function setTab(tab)")
    set_tab_block = app_js[set_tab_start : app_js.index("function selectedPosePayload", set_tab_start)]

    assert 'document.querySelector(".app").classList.toggle("system-mode", tab === "system");' in set_tab_block
    assert 'const panelName = tab === "system" ? "" : (tab === "mapping" ? "mapping" : "navigation");' in set_tab_block
    assert "panel.dataset.panel === panelName" in set_tab_block
    assert "applyEffectiveSidebar();" in set_tab_block

    assert ".app.system-mode" in styles_css
    assert ".app.system-mode .panel" in styles_css
    system_panel_start = styles_css.index(".app.system-mode .panel")
    system_panel_block = styles_css[system_panel_start : styles_css.index("}", system_panel_start)]
    assert "display: none;" not in system_panel_block
    assert "opacity: 0;" in system_panel_block

    assert 'const effective = sidebarState.collapsed || state.view === "system";' in app_js


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


def test_glass_tokens_and_mode_classes():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    root = _css_rule_block(styles_css, ":root")
    for token in (
        "--scene-bg: #0b0f19;",
        "--glass-drop-ms: 380ms;",
        "--glass-drop-ease: cubic-bezier(0.18, 1.32, 0.32, 1);",
        "--layout-slide-ms: 650ms;",
        "--fade-ms: 480ms;",
        "--thumb-ms: 560ms;",
        "--joystick-reveal-ms: 420ms;",
        "--joystick-hide-ms: 220ms;",
        "--layout-ease: cubic-bezier(0.4, 0, 0.2, 1);",
        "--focus-ring: 2px;",
        "--touch-target: 44px;",
    ):
        assert token in root
    assert "--ios-spring: cubic-bezier(0.2, 0.85, 0.25, 1.08)" in root
    assert "--ios-contained-spring: cubic-bezier(0.2, 0.85, 0.25, 1)" in root
    defs = _css_rule_block(styles_css, ".glass-defs")
    assert "position: absolute;" in defs
    assert "width: 0;" in defs
    assert "height: 0;" in defs
    assert "overflow: hidden;" in defs
    assert "pointer-events: none;" in defs
    assert "display: none;" not in defs


def test_glass_baseline_reset_removes_stacked_blur():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    for selector in (
        ".tabs",
        ".sliding-tabs::before",
        ".segmented::before",
        ".system-card",
        ".cpu-core-panel",
        ".hud",
        ".joystick-base",
        ".panel",
        ".sidebar-toggle",
        ".panel section",
    ):
        assert "backdrop-filter" not in _css_rule_block(styles_css, selector), selector
    assert "filter: drop-shadow" not in styles_css
    assert "will-change: transform, box-shadow, filter;" not in styles_css
    assert "transition: transform 430ms var(--ios-spring)" not in styles_css
    assert "transition: transform 430ms var(--ios-contained-spring)" not in styles_css


def test_glass_full_tier_limits_displacement_to_track_and_two_static_lenses():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    assert ".glass-full .sliding-tabs {" in styles_css
    assert ".glass-full .segmented::before," in styles_css
    assert ".glass-full .sidebar-toggle-lens {" in styles_css
    assert ".glass-full .sliding-tabs::before" not in styles_css

    dynamic_start = styles_css.index(".glass-full .sliding-tabs {")
    dynamic_body = styles_css[
        styles_css.index("{", dynamic_start) + 1 : styles_css.index("}", dynamic_start)
    ]
    assert "url(#liquidTrackFilter)" in dynamic_body
    assert "url(#liquidDrop)" not in dynamic_body

    static_start = styles_css.index(".glass-full .segmented::before,")
    static_selectors = styles_css[static_start : styles_css.index("{", static_start)]
    assert static_selectors.count(".glass-full") == 2
    static_body = styles_css[
        styles_css.index("{", static_start) + 1 : styles_css.index("}", static_start)
    ]
    assert "backdrop-filter: url(#liquidDrop) saturate(1.4);" in static_body
    assert "-webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);" in static_body

    lens = _css_rule_block(styles_css, ".sidebar-toggle-lens")
    assert "position: absolute;" in lens
    assert "inset: 0;" in lens
    assert "border-radius: inherit;" in lens
    assert "background: var(--glass);" in lens
    assert "pointer-events: none;" in lens


def test_glass_light_tier_is_single_layer():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    anchor = ":where(.glass-full, .glass-light) .panel,"
    assert anchor in styles_css
    start = styles_css.index(anchor)
    end = styles_css.index("{", start)
    selectors = styles_css[start:end]
    assert selectors.count(":where(.glass-full, .glass-light)") == 6
    assert ".panel section" not in selectors
    body = styles_css[end : styles_css.index("}", end)]
    assert "backdrop-filter: blur(22px) saturate(1.35);" in body
    assert "-webkit-backdrop-filter: blur(22px) saturate(1.35);" in body


def test_glass_off_tokens_are_opaque():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    block = _css_rule_block(styles_css, ".glass-off")
    assert "--glass: rgba(255, 255, 255, 0.96);" in block
    assert "--glass-strong: rgba(255, 255, 255, 0.98);" in block
    assert "--panel-bg: rgba(255, 255, 255, 0.98);" in block
    assert "--terminal-bg: rgba(255, 255, 255, 0.98);" in block
    assert "@media (prefers-color-scheme: dark)" in styles_css
    assert "--panel-bg: rgba(15, 17, 24, 0.98);" in styles_css
    assert "--terminal-bg: rgba(5, 8, 12, 0.98);" in styles_css


def test_glass_tier_selectors_exclude_webgl_text_and_safety_controls():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    full_start = styles_css.index(".glass-full .sliding-tabs {")
    full_selectors = styles_css[full_start : styles_css.index("{", full_start)]
    static_start = styles_css.index(".glass-full .segmented::before,")
    full_selectors += styles_css[static_start : styles_css.index("{", static_start)]
    light_start = styles_css.index(".glass-light .sliding-tabs {")
    light_selectors = styles_css[light_start : styles_css.index("{", light_start)]
    where_start = styles_css.index(":where(.glass-full, .glass-light) .panel,")
    light_selectors += styles_css[where_start : styles_css.index("{", where_start)]
    for selector in (
        ".joystick-knob",
        ".terminal",
        ".terminal pre",
        ".source",
        "#navigationPrimary",
        "#navigationClear",
        "#publishPose",
        "#localizationProcessToggle",
        "#navigationProcessToggle",
        ".start-log",
        ".stop-log",
        ".joystick-settings-button",
        ".joystick-done-button",
        "#confirmSaveMap",
        "#cancelSaveMap",
    ):
        assert selector not in full_selectors, selector
        assert selector not in light_selectors, selector


def test_glass_focus_and_touch_targets():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    focus = _css_rule_block(styles_css, ":focus-visible")
    assert "outline: var(--focus-ring) solid var(--ios-blue);" in focus
    assert "outline-offset: 2px;" in focus
    coarse_start = styles_css.index("@media (pointer: coarse)")
    coarse = styles_css[coarse_start : styles_css.index("\n}", coarse_start)]
    assert "min-height: var(--touch-target);" in coarse
    assert "width: var(--touch-target);" in coarse
    for selector in (
        ".tab,",
        ".mode,",
        ".actions button,",
        ".start-log,",
        ".stop-log,",
        ".sidebar-toggle,",
        ".joystick-settings-button,",
        ".joystick-done-button,",
        ".map-picker button,",
        ".save-map-dialog-actions button",
    ):
        assert selector in coarse, selector


def test_glass_reduced_motion_fallback():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    motion_start = styles_css.index("@media (prefers-reduced-motion: reduce)")
    motion = styles_css[motion_start : styles_css.index("\n}", motion_start)]
    assert "--layout-slide-ms: 1ms;" in motion
    assert "--fade-ms: 1ms;" in motion
    assert "--thumb-ms: 1ms;" in motion
    assert "--joystick-reveal-ms: 1ms;" in motion
    assert "--joystick-hide-ms: 1ms;" in motion
    assert "--glass-drop-ms: 1ms;" in motion
    assert ".joystick {" in motion
    assert "animation: none;" in motion
    assert "body::before," in motion
    assert "body::after" in motion
    assert ".system-meter-fill" in motion
    assert "prefers-reduced-transparency" not in styles_css
    assert ".glass-off {" in styles_css


def test_glass_water_drop_motion():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    assert "transition: transform var(--thumb-ms) var(--glass-drop-ease)," in styles_css
    assert "will-change: transform;" in styles_css
    assert "@keyframes glassDropScale" in styles_css
    keyframes = styles_css[styles_css.index("@keyframes glassDropScale") :]
    assert "scale: 0.94;" in keyframes
    assert "scale: 1.035;" in keyframes
    animation_start = styles_css.index(".sliding-tabs.glass-drop::before,")
    assert ".segmented.glass-drop::before" in styles_css[
        animation_start : styles_css.index("{", animation_start)
    ]
    animation_body = styles_css[
        styles_css.index("{", animation_start) + 1 : styles_css.index("}", animation_start)
    ]
    assert "animation: glassDropScale var(--glass-drop-ms) var(--glass-drop-ease);" in animation_body
    assert "filter" not in animation_body
    # The orphan caustic dot is gone; the drop is carried by the capsule itself.
    assert ".sliding-tabs::after" not in styles_css
    assert ".segmented::after" not in styles_css
    assert ".sliding-tabs.glass-drop::after" not in styles_css


def test_glass_drop_class_is_timed_and_forced_reflowed():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    assert "function markGlassDrop(element)" in app_js
    assert "void element.offsetWidth;" in app_js
    assert 'element.addEventListener("animationend", clear, { once: true });' in app_js
    assert 'markGlassDrop(document.getElementById("modeSegment"));' in app_js
    assert "const GLASS_DROP_FALLBACK_MS = 520;" in app_js


_GLASS_DROP_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("const GLASS_DROP_FALLBACK_MS = 520;");
const end = source.indexOf("function setMode(mode)");
if (start < 0 || end < 0) throw new Error("glass drop block not found");
const block = source.slice(start, end);

const timers = new Map();
let nextTimer = 1;
const windowStub = {
  setTimeout(fn, ms) { const id = nextTimer++; timers.set(id, { fn, ms }); return id; },
  clearTimeout(id) { timers.delete(id); },
};

let reflows = 0;
function makeElement() {
  const classes = new Set();
  const listeners = new Set();
  const element = {
    classList: {
      add(name) { classes.add(name); },
      remove(name) { classes.delete(name); },
      contains(name) { return classes.has(name); },
    },
    addEventListener(type, handler) { listeners.add(handler); },
    removeEventListener(type, handler) { listeners.delete(handler); },
  };
  Object.defineProperty(element, "offsetWidth", { get() { reflows += 1; return 0; } });
  element.liveListeners = listeners;
  element.classes = classes;
  return element;
}

const api = new Function("window", block + "\nreturn { markGlassDrop };");
const { markGlassDrop } = api(windowStub);

function expect(condition, message) { if (!condition) throw new Error(message); }

const element = makeElement();
markGlassDrop(element);
markGlassDrop(element);
markGlassDrop(element);
expect(reflows === 3, "each mark forces exactly one reflow restart");
expect(element.liveListeners.size === 1, "rapid re-marks keep exactly one live listener");
expect(timers.size === 1, "rapid re-marks keep exactly one live fallback timeout");
expect(element.classes.has("glass-drop"), "the drop class is active after re-marking");

// Firing the surviving animationend retires the class, listener and timeout.
const listener = Array.from(element.liveListeners)[0];
listener();
expect(!element.classes.has("glass-drop"), "animationend clears the drop class");
expect(element.liveListeners.size === 0, "animationend removes the live listener");
expect(timers.size === 0, "animationend clears the pending fallback timeout");

// The fallback timeout alone also retires everything.
markGlassDrop(element);
expect(element.liveListeners.size === 1 && timers.size === 1, "a fresh mark re-arms cleanup");
const timer = Array.from(timers.values())[0];
expect(timer.ms === 520, "the fallback uses the documented delay");
timer.fn();
expect(!element.classes.has("glass-drop"), "the fallback clears the drop class");
expect(element.liveListeners.size === 0, "the fallback removes the live listener");
expect(timers.size === 0, "the fallback clears its own handle");

// A missing target is a safe no-op.
const reflowsBefore = reflows;
markGlassDrop(null);
expect(reflows === reflowsBefore && timers.size === 0, "marking a missing element is a no-op");

console.log("glass drop cleanup ok");
"""


def test_glass_drop_rapid_remark_keeps_single_cleanup_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _GLASS_DROP_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "glass drop cleanup ok" in completed.stdout


def test_mode_control_uses_ios_sliding_segmented_selector():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert 'id="modeSegment"' in index_html
    assert 'data-active="view"' in index_html
    assert 'document.getElementById("modeSegment").dataset.active = mode;' in app_js
    assert ".segmented::before" in styles_css
    assert "transform: translate3d(var(--thumb-x, 4px), 0, 0);" in styles_css
    assert "width: var(--thumb-w, calc((100% - 8px) / 3));" in styles_css
    assert "modeSegmentControl.sync(" in app_js


def test_frontend_uses_immersive_glass_light_environment():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "body::before" in styles_css
    assert "body::after" in styles_css
    assert "@keyframes ambientDrift" in styles_css
    assert "@keyframes sheenSweep" in styles_css
    assert "mix-blend-mode: screen" in styles_css
    # The mounted viewport animates transform/opacity, so a fixed background
    # attachment would force full-viewport repaints on every frame.
    assert "background-attachment" not in styles_css
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
    assert "viewTabs.dataset.active = tab;" in app_js
    assert ".sliding-tabs::before" in styles_css
    assert "transform: translate3d(var(--thumb-x, 4px), 0, 0);" in styles_css
    assert "width: var(--thumb-w, calc((100% - 8px) / 3));" in styles_css
    assert "viewTabsControl.sync(" in app_js


def test_sliding_selectors_use_transform_based_ios_spring_motion():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--ios-spring: cubic-bezier(0.2, 0.85, 0.25, 1.08)" in styles_css
    assert "--glass-drop-ease: cubic-bezier(0.18, 1.32, 0.32, 1);" in styles_css
    assert "will-change: transform;" in styles_css
    assert "will-change: transform, box-shadow, filter;" not in styles_css
    # The capsule moves to measured pixel offsets; percent-of-self translation
    # is what let its own border accumulate past the right edge.
    assert "transform: translate3d(var(--thumb-x, 4px), 0, 0)" in styles_css
    assert "transform: translate3d(calc(var(--tabs-index, 0) * 100%), 0, 0)" not in styles_css
    assert "transform: translate3d(calc(var(--mode-index, 0) * 100%), 0, 0)" not in styles_css
    assert "transition: transform var(--thumb-ms) var(--glass-drop-ease)" in styles_css
    assert ".segmented::before" in styles_css
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
    assert "backdrop-filter: url(#liquidDrop) saturate(1.4);" in styles_css
    assert "-webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);" in styles_css
    assert "var(--liquid-slider-noise)" in styles_css
    assert "background-blend-mode: soft-light" in styles_css
    assert "background-size: 80px 80px" in styles_css
    assert "radial-gradient(circle at 18% 0%, var(--liquid-slider-caustic), transparent 42%)" in styles_css
    assert "inset 0 1px 0 rgba(255, 255, 255, 0.72)" in styles_css
    assert "filter: drop-shadow(0 10px 24px rgba(10, 132, 255, 0.18));" not in styles_css


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
    segmented_before = _css_rule_block(styles_css, ".segmented::before")
    assert "transition: transform var(--thumb-ms) var(--glass-drop-ease)" in segmented_before
    assert "var(--ios-spring)" not in segmented_before
    assert "var(--ios-contained-spring)" not in segmented_before


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
    assert "grid-template-columns: minmax(420px, 1fr) var(--panel-width, 340px);" in styles_css
    assert "grid-template-rows: 1fr;" in styles_css
    assert "overflow-x: auto;" in styles_css
    mobile_app_block_start = styles_css.index("@media (max-width: 900px)")
    mobile_app_block_end = styles_css.index(".panel", mobile_app_block_start)
    assert "grid-template-columns: 1fr;" not in styles_css[mobile_app_block_start:mobile_app_block_end]
    assert "grid-template-rows: 52vh 48vh;" not in styles_css


def test_app_clips_collapsed_sidebar_horizontal_overflow():
    _, _, styles_css = _frontend_sources()

    app = _css_rule_block(styles_css, ".app")
    assert "overflow-x: hidden;" in app
    assert "overflow-x: clip;" in app
    # The hidden declaration is the fallback and must precede the preferred clip.
    assert app.index("overflow-x: hidden;") < app.index("overflow-x: clip;")

    # The intentional sub-760px horizontal panning contract stays on the root.
    root = _css_rule_block(styles_css, "html,\nbody")
    assert "overflow-x: auto;" in root

    # Panel vertical scrolling remains internal to the panel.
    panel = _css_rule_block(styles_css, ".panel")
    assert "overflow-y: auto;" in panel


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
        self.stopped = 0
        self._running = running
        self.log_buffer = None

    def is_running(self):
        return self._running

    def start(self):
        self.started += 1
        self._running = True
        if self.log_buffer is not None:
            self.log_buffer.set_process_state(
                enabled=True, running=True, returncode=None
            )

    def stop(self):
        self.stopped += 1
        self._running = False
        if self.log_buffer is not None:
            self.log_buffer.set_process_state(
                enabled=True, running=False, returncode=None
            )


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
    node.layers = {
        name: SourceCache(name, 2.0, static_source=name != "obstacles")
        for name in ("global_map", "local_map", "mapcloud", "mapground", "obstacles")
    }
    node.managed_logs = {
        target: TerminalLogBuffer(target, max_lines=5)
        for target in (
            "chassis",
            "imu",
            "lidar",
            "localization",
            "navigation",
            "mapping",
        )
    }
    node.managed_processes = {
        "chassis": _RecordingManagedProcess(
            ["ros2", "launch", "hunter_base", "hunter_base.launch.py"],
            running="chassis" in running,
        ),
        "imu": _RecordingManagedProcess(
            ["ros2", "launch", "fdilink_ahrs", "ahrs_driver.launch.py"],
            running="imu" in running,
        ),
        "lidar": _RecordingManagedProcess(
            ["ros2", "launch", "hesai_ros_driver", "start.py"],
            running="lidar" in running,
        ),
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
    for target, process in node.managed_processes.items():
        process.log_buffer = node.managed_logs[target]
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
        "stop_managed_target",
        "terminal_logs_payload",
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
        "mode": "auto",
        "effective_mode": None,
        "mode_valid": False,
        "mode_message": "no map selected",
        "auto_relocalization": True,
        "auto_relocalization_available": False,
        "effective_auto_relocalization": False,
        "auto_relocalization_message": "no map selected",
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


_VALID_TWO_STICK = {
    "throttle": {"size": 200, "x": 20, "y": 320},
    "steering": {"size": 200, "x": 520, "y": 320},
}


def _two_stick_defaults() -> dict:
    return {
        "throttle": {"size": 180, "x": None, "y": None},
        "steering": {"size": 180, "x": None, "y": None},
    }


def test_ui_preferences_get_returns_two_stick_defaults_and_saved_values(tmp_path):
    node = _selection_node(tmp_path)

    status, body = _serve_and_request(node, "/api/ui/preferences")

    assert status == 200
    assert body["ok"] is True
    # A null offset means "browser role default" (throttle lower-left, steering
    # lower-right); saved offsets are stable viewport pixels.
    assert body["joystick"] == _two_stick_defaults()

    node.map_selection.save_joystick_preferences(_VALID_TWO_STICK)

    status, body = _serve_and_request(node, "/api/ui/preferences")

    assert status == 200
    assert body["joystick"] == _VALID_TWO_STICK


def test_ui_preferences_post_persists_two_sticks_and_preserves_selection(tmp_path):
    _make_map_dir(tmp_path / "map", "site_a")
    node = _selection_node(tmp_path, selection="site_a")
    node.map_selection.path.write_text(
        json.dumps({"name": "site_a", "ui": {"theme": "dark"}, "extra": 1}) + "\n",
        encoding="utf-8",
    )

    status, body = _serve_and_request(
        node, "/api/ui/preferences", _VALID_TWO_STICK, "POST"
    )

    assert status == 200
    assert body["ok"] is True
    assert body["joystick"] == _VALID_TWO_STICK
    assert node.map_selection.load() == "site_a"
    payload = json.loads(node.map_selection.path.read_text(encoding="utf-8"))
    assert payload == {
        "name": "site_a",
        "ui": {"theme": "dark", "joystick": _VALID_TWO_STICK},
        "extra": 1,
    }


@pytest.mark.parametrize(
    "payload",
    [
        [],
        {},
        {"throttle": {"size": 200, "x": 20, "y": 320}},
        {"throttle": [], "steering": {"size": 200, "x": 20, "y": 320}},
        {
            "throttle": {"size": 139, "x": 20, "y": 320},
            "steering": {"size": 200, "x": 20, "y": 320},
        },
        {
            "throttle": {"size": 261, "x": 20, "y": 320},
            "steering": {"size": 200, "x": 20, "y": 320},
        },
        {
            "throttle": {"size": "200", "x": 20, "y": 320},
            "steering": {"size": 200, "x": 20, "y": 320},
        },
        {
            "throttle": {"size": float("nan"), "x": 20, "y": 320},
            "steering": {"size": 200, "x": 20, "y": 320},
        },
        {
            "throttle": {"size": 200, "x": -1, "y": 320},
            "steering": {"size": 200, "x": 20, "y": 320},
        },
        {
            "throttle": {"size": 200, "x": 20, "y": -0.5},
            "steering": {"size": 200, "x": 20, "y": 320},
        },
        {
            "throttle": {"size": 200, "x": 20},
            "steering": {"size": 200, "x": 20, "y": 320},
        },
        {
            "throttle": {"size": 200, "x": 20, "y": 320},
            "steering": {"size": 200, "x": "20", "y": 320},
        },
        {
            "throttle": {"size": 200, "x": True, "y": 320},
            "steering": {"size": 200, "x": 20, "y": 320},
        },
    ],
)
def test_ui_preferences_post_rejects_invalid_two_stick_payload(tmp_path, payload):
    node = _selection_node(tmp_path)
    node.map_selection.save_joystick_preferences(_VALID_TWO_STICK)
    before = node.map_selection.path.read_text(encoding="utf-8")

    status, body = _serve_and_request(node, "/api/ui/preferences", payload, "POST")

    assert status == 400
    assert body["ok"] is False
    assert node.map_selection.path.read_text(encoding="utf-8") == before
    assert node.map_selection.load_joystick_preferences() == _VALID_TWO_STICK


@pytest.mark.parametrize(
    "role,key",
    [
        ("throttle", "size"),
        ("throttle", "x"),
        ("throttle", "y"),
        ("steering", "size"),
        ("steering", "x"),
        ("steering", "y"),
    ],
)
def test_ui_preferences_post_rejects_overflowing_two_stick_value(tmp_path, role, key):
    node = _selection_node(tmp_path)
    node.map_selection.save_joystick_preferences(_VALID_TWO_STICK)
    before = node.map_selection.path.read_text(encoding="utf-8")
    payload = json.loads(json.dumps(_VALID_TWO_STICK))
    payload[role][key] = 10**400

    status, body = _serve_and_request(node, "/api/ui/preferences", payload, "POST")

    assert status == 400
    assert body["ok"] is False
    assert node.map_selection.path.read_text(encoding="utf-8") == before
    assert node.map_selection.load_joystick_preferences() == _VALID_TWO_STICK


def test_ui_preferences_get_repairs_invalid_two_stick_values_independently(tmp_path):
    node = _selection_node(tmp_path)
    node.map_selection.path.parent.mkdir(parents=True, exist_ok=True)
    node.map_selection.path.write_text(
        json.dumps(
            {
                "name": "site_a",
                "ui": {
                    "joystick": {
                        "throttle": {"size": 10**400, "x": -3, "y": 320},
                        "steering": {"size": 250, "x": 10**400, "y": "nope"},
                    }
                },
            }
        )
        + "\n",
        encoding="utf-8",
    )

    status, body = _serve_and_request(node, "/api/ui/preferences")

    assert status == 200
    assert body["joystick"] == {
        "throttle": {"size": 180, "x": None, "y": 320},
        "steering": {"size": 250, "x": None, "y": None},
    }


def test_ui_preferences_get_migrates_legacy_single_joystick(tmp_path):
    node = _selection_node(tmp_path)
    node.map_selection.path.parent.mkdir(parents=True, exist_ok=True)
    node.map_selection.path.write_text(
        json.dumps(
            {
                "name": "site_a",
                "ui": {"joystick": {"size": 210, "x_ratio": 0.2, "y_ratio": 0.6}},
            }
        )
        + "\n",
        encoding="utf-8",
    )

    status, body = _serve_and_request(node, "/api/ui/preferences")

    assert status == 200
    assert node.map_selection.load() == "site_a"
    assert body["joystick"] == {
        "throttle": {"size": 210, "x": None, "y": None},
        "steering": {"size": 210, "x": None, "y": None},
    }


def test_ui_preferences_get_migrates_legacy_invalid_size_to_defaults(tmp_path):
    node = _selection_node(tmp_path)
    node.map_selection.path.parent.mkdir(parents=True, exist_ok=True)
    node.map_selection.path.write_text(
        json.dumps(
            {
                "name": "site_a",
                "ui": {"joystick": {"size": 10**400, "x_ratio": 0.2, "y_ratio": 0.6}},
            }
        )
        + "\n",
        encoding="utf-8",
    )

    status, body = _serve_and_request(node, "/api/ui/preferences")

    assert status == 200
    assert body["joystick"] == _two_stick_defaults()


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
    assert [a for a in localization if a.startswith("auto_relocalization:=")] == [
        "auto_relocalization:=true"
    ]
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


def test_managed_hardware_targets_are_reachable_over_the_real_api(tmp_path):
    node = _selection_node(tmp_path)
    server, thread = _serve_fake_web_node(node)
    base = f"http://127.0.0.1:{server.server_address[1]}"

    def request(path, payload=None, method="GET"):
        return _json_request(f"{base}{path}", payload, method)

    try:
        # Log/state whitelist: both pages read the same three hardware targets.
        status, body = request("/api/logs")
        assert status == 200
        for target in ("chassis", "imu", "lidar"):
            assert target in body["logs"]
            assert body["logs"][target]["running"] is False

        # Each hardware target starts through the real POST route.
        for target in ("chassis", "imu", "lidar"):
            status, body = request(
                "/api/managed/start", {"target": target}, "POST"
            )
            assert status == 200, (target, body)
            assert body["ok"] is True
            assert node.managed_processes[target].started == 1

        # The running flag the browser polls travels back through /api/logs.
        status, body = request("/api/logs")
        assert status == 200
        for target in ("chassis", "imu", "lidar"):
            assert body["logs"][target]["running"] is True

        # The stop route reaches the same shared backend object.
        status, body = request(
            "/api/managed/stop", {"target": "imu"}, "POST"
        )
        assert status == 200
        assert body["ok"] is True
        assert node.managed_processes["imu"].stopped == 1
        status, body = request("/api/logs")
        assert body["logs"]["imu"]["running"] is False
        assert body["logs"]["chassis"]["running"] is True

        # Unknown targets are rejected before any process is touched.
        status, body = request(
            "/api/managed/start", {"target": "reactor"}, "POST"
        )
        assert status == 400
        assert body["ok"] is False
        assert "unknown managed target" in body["message"]
        status, body = request(
            "/api/managed/stop", {"target": "reactor"}, "POST"
        )
        assert status == 400
        for target in ("chassis", "imu", "lidar"):
            assert node.managed_processes[target].started == 1
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def _frontend_sources():
    """Return ``(index_html, app_js, styles_css)`` for browser-facing tests."""
    static_dir = Path(__file__).resolve().parents[1] / "static"
    return (
        (static_dir / "index.html").read_text(encoding="utf-8"),
        (static_dir / "app.js").read_text(encoding="utf-8"),
        (static_dir / "styles.css").read_text(encoding="utf-8"),
    )


def _css_rule_block(styles_css, selector):
    """Return one CSS rule body for an exact ``selector {`` match."""
    start = styles_css.index(f"{selector} {{")
    end = styles_css.index("}", start)
    return styles_css[start:end]


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
  "mapSelect", "mapMode", "mapWarning", "mapStatus", "mapLockNote",
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

// A valid tiled map keeps the selector usable without a persistent status line.
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
expect(elements.mapStatus.hidden && elements.mapStatus.textContent === "", "normal map status should be hidden");

// A global map without sc_db.bin still surfaces its actionable tile warning.
const warning = "tiled manifest is invalid: bad schema";
state.mapCatalog.maps = [{ name: "site_b", tiled: false, has_scan_context: false, warning }];
state.mapCatalog.selection = {
  name: "site_b",
  valid: true,
  entry: { name: "site_b", tiled: false, has_scan_context: false, warning },
  message: null,
};
updateMapPanel();
expect(!elements.mapWarning.hidden, "tile warning should be visible");
expect(/tile warning/i.test(elements.mapWarning.textContent), "tile warning was not surfaced");
expect(elements.mapStatus.hidden, "a non-actionable map status should stay hidden");

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


_LAYOUT_TRANSITION_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Page transition state machine");
const blockEnd = source.indexOf("// --- End page transition state machine");
if (blockStart < 0 || blockEnd < 0) throw new Error("page transition block not found");
const block = source.slice(blockStart, blockEnd);

let reduceMotion = false;
global.glassMotionReduced = () => reduceMotion;
let resizeCalls = 0;
let layoutCalls = 0;
let hudCalls = 0;
let renderCalls = 0;
global.resizeAll = () => { resizeCalls += 1; };
global.applyJoystickLayout = () => { layoutCalls += 1; };
global.updateHudPlacement = () => { hudCalls += 1; };
global.flushPendingMappingJoystickReveal = () => {};
global.renderer = { render: () => { renderCalls += 1; } };
global.scene = {};
global.camera = {};

const timers = new Map();
const frames = new Map();
let nextTimer = 1000;
let nextFrame = 1;
global.setTimeout = (fn) => { const id = nextTimer++; timers.set(id, fn); return id; };
global.clearTimeout = (id) => { timers.delete(id); };
global.requestAnimationFrame = (fn) => { const id = nextFrame++; frames.set(id, fn); return id; };
global.cancelAnimationFrame = (id) => { frames.delete(id); };

function flushTimers() {
  const pending = Array.from(timers.values());
  timers.clear();
  pending.forEach((fn) => fn());
}

function flushFrames() {
  const pending = Array.from(frames.values());
  frames.clear();
  pending.forEach((fn) => fn());
}

function makeElement() {
  const listeners = {};
  return {
    addEventListener(type, handler) { (listeners[type] = listeners[type] || []).push(handler); },
    dispatch(type, event) { (listeners[type] || []).forEach((handler) => handler(event)); },
  };
}

const appEl = makeElement();
const view3dEl = makeElement();
const viewSystemEl = makeElement();
const viewTabsEl = makeElement();
global.document = {
  querySelector(selector) { return selector === ".app" ? appEl : null; },
  getElementById(id) {
    if (id === "view3d") return view3dEl;
    if (id === "viewSystem") return viewSystemEl;
    if (id === "viewTabs") return viewTabsEl;
    return null;
  },
};

const transitionApi = new Function(block + `
return { pageTransition, beginLayoutTransition, settleLayoutTransition, noteLayoutResize, doDeferredLayoutFlush, attachLayoutTransitionListener };`)();
const {
  pageTransition, beginLayoutTransition, settleLayoutTransition,
  noteLayoutResize, doDeferredLayoutFlush, attachLayoutTransitionListener,
} = transitionApi;

function expect(condition, message) { if (!condition) throw new Error(message); }
function resetCounters() { resizeCalls = 0; layoutCalls = 0; hudCalls = 0; renderCalls = 0; }

attachLayoutTransitionListener();

// Rapid requests: the newest token wins and the previous fallback is cancelled.
resetCounters();
const firstToken = beginLayoutTransition();
const secondToken = beginLayoutTransition();
expect(secondToken === firstToken + 1, "each request bumps the token");
flushTimers();
expect(resizeCalls === 1, "one flush for rapid requests");
expect(layoutCalls === 1, "one joystick layout for rapid requests");
expect(renderCalls === 1, "one synchronous render for rapid requests");
expect(pageTransition.active === false, "controller returns to stable");
settleLayoutTransition(firstToken);
expect(resizeCalls === 1, "a stale token never flushes");

// Only the app's own grid event may settle a geometry-aware transition.
resetCounters();
beginLayoutTransition({ gridChanged: true });
appEl.dispatch("transitionend", { propertyName: "color", target: appEl });
appEl.dispatch("transitionend", { propertyName: "opacity", target: appEl });
appEl.dispatch("transitionend", { propertyName: "transform", target: appEl });
expect(resizeCalls === 0, "opacity/transform never settle a transition");
expect(pageTransition.active === true, "the transition stays pending after ignored events");
flushTimers();
expect(resizeCalls === 1, "the fallback settles after ignored opacity/transform events");

// Pseudo-element grid events are ignored.
resetCounters();
beginLayoutTransition({ gridChanged: true });
appEl.dispatch("transitionend", { propertyName: "grid-template-columns", target: appEl, pseudoElement: "::before" });
expect(resizeCalls === 0, "pseudo-element grid events never settle");
flushTimers();
expect(resizeCalls === 1, "the fallback settles after a pseudo-element event");

// A grid event from a non-.app target is ignored.
resetCounters();
beginLayoutTransition({ gridChanged: true });
appEl.dispatch("transitionend", { propertyName: "grid-template-columns", target: viewSystemEl });
expect(resizeCalls === 0, "grid events from non-app targets never settle");
flushTimers();
expect(resizeCalls === 1, "the fallback settles after a non-app grid event");

// The app's own grid event settles only a transition that expects the grid.
resetCounters();
beginLayoutTransition({ gridChanged: true });
appEl.dispatch("transitionend", { propertyName: "grid-template-columns", target: appEl });
expect(resizeCalls === 1, "the app grid event settles a geometry transition");
expect(renderCalls === 1, "a settled grid event performs the settle render");
expect(pageTransition.active === false, "the geometry transition is stable after settling");

// A pure view transition ignores a stale grid event and uses the fallback.
resetCounters();
beginLayoutTransition({ gridChanged: false });
appEl.dispatch("transitionend", { propertyName: "grid-template-columns", target: appEl });
expect(resizeCalls === 0, "a pure view transition never settles on a grid event");
expect(pageTransition.active === true, "the pure view transition stays pending");
flushTimers();
expect(resizeCalls === 1, "the pure view transition settles on the fallback");

// A pure begin while a grid run is still in flight must not clear the grid
// expectation: the grid is physically moving, so the app's own grid event still
// settles the combined run instead of being treated as stale.
resetCounters();
const gridRunToken = beginLayoutTransition({ gridChanged: true });
const pureOverlayToken = beginLayoutTransition({ gridChanged: false });
expect(pureOverlayToken === gridRunToken + 1, "the pure begin takes the newest token");
expect(pageTransition.expectsGrid === true, "an in-flight grid run keeps expectsGrid sticky");
appEl.dispatch("transitionend", { propertyName: "grid-template-columns", target: appEl });
expect(resizeCalls === 1, "the app grid event settles the combined in-flight run");
expect(pageTransition.active === false, "the combined run is stable after the grid event");

// Once settled, a later pure transition is genuinely grid-free: it must ignore
// the (now stale) grid event and settle on the fallback timer.
resetCounters();
beginLayoutTransition({ gridChanged: false });
expect(pageTransition.expectsGrid === false, "a fresh pure transition does not expect the grid");
appEl.dispatch("transitionend", { propertyName: "grid-template-columns", target: appEl });
expect(resizeCalls === 0, "a fresh pure transition ignores the app grid event");
expect(pageTransition.active === true, "the fresh pure transition stays pending");
flushTimers();
expect(resizeCalls === 1, "the fresh pure transition settles on the fallback");

// A stale token never flushes after the live transition has settled.
resetCounters();
beginLayoutTransition({ gridChanged: true });
const liveToken = beginLayoutTransition({ gridChanged: true });
appEl.dispatch("transitionend", { propertyName: "grid-template-columns", target: appEl });
expect(resizeCalls === 1, "the newest token settles on the app grid event");
settleLayoutTransition(liveToken - 1);
expect(resizeCalls === 1, "old tokens are ignored after settling");

// No transitionend: the fallback timer settles.
resetCounters();
beginLayoutTransition();
flushTimers();
expect(resizeCalls === 1, "the fallback settles without transitionend");
expect(renderCalls === 1, "the fallback renders once");

// Reduced motion settles on the next animation frame, not the fallback timer.
reduceMotion = true;
resetCounters();
beginLayoutTransition();
flushTimers();
expect(resizeCalls === 0, "reduced motion ignores the fallback timer");
flushFrames();
expect(resizeCalls === 1, "reduced motion settles on the next frame");
reduceMotion = false;

// A resize storm while pending coalesces into the single settle flush.
resetCounters();
beginLayoutTransition();
noteLayoutResize();
noteLayoutResize();
noteLayoutResize();
expect(resizeCalls === 0, "no setSize while a transition is pending");
flushFrames();
expect(resizeCalls === 0, "pending resize requests do not schedule a frame");
flushTimers();
expect(resizeCalls === 1, "the settle flush resizes once");
expect(layoutCalls === 1, "the settle flush applies joystick layout once");
expect(hudCalls === 1, "the settle flush updates HUD placement once");
expect(renderCalls === 1, "the settle flush renders exactly once");

// Flushing while pending defers instead of resizing mid-transition.
resetCounters();
beginLayoutTransition();
doDeferredLayoutFlush();
expect(resizeCalls === 0, "a direct flush during a transition is deferred");
flushTimers();
expect(resizeCalls === 1, "the deferred flush runs at settle");

console.log("page transition ok");
"""


def test_frontend_layout_transition_state_machine_executes_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _LAYOUT_TRANSITION_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "page transition ok" in completed.stdout


_LAYOUT_FALLBACK_NAMESPACE_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Page transition state machine");
const blockEnd = source.indexOf("// --- End page transition state machine");
if (blockStart < 0 || blockEnd < 0) throw new Error("page transition block not found");
const block = source.slice(blockStart, blockEnd);

// Separate fake timer and rAF namespaces that assign colliding numeric ids,
// mirroring how a real browser reuses small integer handles for both APIs.
const timers = new Map();
const frames = new Map();
let nextTimer = 5000;
let nextFrame = 5000;
global.setTimeout = (fn) => { const id = nextTimer++; timers.set(id, fn); return id; };
global.clearTimeout = (id) => { timers.delete(id); };
global.requestAnimationFrame = (fn) => { const id = nextFrame++; frames.set(id, fn); return id; };
global.cancelAnimationFrame = (id) => { frames.delete(id); };

let reduceMotion = false;
global.glassMotionReduced = () => reduceMotion;
global.resizeAll = () => {};
global.applyJoystickLayout = () => {};
global.updateHudPlacement = () => {};
global.flushPendingMappingJoystickReveal = () => {};
global.renderer = { render: () => {} };
global.scene = {};
global.camera = {};
global.document = {
  querySelector() { return null; },
  getElementById() { return null; },
};

const transitionApi = new Function(
  block + `
return { pageTransition, beginLayoutTransition, settleLayoutTransition };`,
)();
const { pageTransition, beginLayoutTransition, settleLayoutTransition } = transitionApi;

function expect(condition, message) { if (!condition) throw new Error(message); }

// A normal (timeout) transition fallback must never cancel an unrelated map rAF
// whose numeric handle happens to collide with the timeout id.
const unrelatedMapFrameId = global.requestAnimationFrame(() => {});
reduceMotion = false;
const timerToken = beginLayoutTransition();
expect(typeof pageTransition.fallbackTimerId === "number", "a timer fallback is tracked");
expect(pageTransition.fallbackFrameId === null, "no frame fallback is tracked mid-timer");
expect(pageTransition.fallbackTimerId === unrelatedMapFrameId,
  "the timeout handle collides with the unrelated map frame id");
settleLayoutTransition(timerToken);
expect(frames.has(unrelatedMapFrameId),
  "settling a timeout transition must not cancel the unrelated map rAF");
expect(timers.size === 0, "the transition timeout is cleared");
expect(pageTransition.fallbackTimerId === null && pageTransition.fallbackFrameId === null,
  "fallback bookkeeping resets after settle");

// A reduced-motion settle cancels only its own rAF and leaves unrelated timers
// and frames untouched even when numeric handles collide.
reduceMotion = true;
const survivingMapFrameId = global.requestAnimationFrame(() => {});
const unrelatedTimerId = global.setTimeout(() => {}, 0);
const frameToken = beginLayoutTransition();
const ownFrameId = pageTransition.fallbackFrameId;
expect(typeof ownFrameId === "number", "a frame fallback is tracked under reduced motion");
expect(pageTransition.fallbackTimerId === null, "no timer fallback is tracked under reduced motion");
const collidingTimerId = global.setTimeout(() => {}, 0);
settleLayoutTransition(frameToken);
expect(!frames.has(ownFrameId), "the reduced-motion settle cancels its own rAF");
expect(frames.has(survivingMapFrameId), "an unrelated map rAF survives the reduced-motion settle");
expect(timers.has(unrelatedTimerId) && timers.has(collidingTimerId),
  "unrelated timers survive the reduced-motion settle");

console.log("layout fallback namespaces ok");
"""


def test_layout_transition_fallback_namespaces_execute_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _LAYOUT_FALLBACK_NAMESPACE_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "layout fallback namespaces ok" in completed.stdout


_RENDER_LOOP_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("let mapRenderFrame = null;");
const end = source.indexOf("function canvasPoint(event)");
if (start < 0 || end < 0) throw new Error("map render loop block not found");
const block = source.slice(start, end);

const frames = new Map();
let nextFrame = 1;
let renderCalls = 0;
let cameraUpdates = 0;
let resizeCalls = 0;
const order = [];
let reduceMotion = false;
const state = { view: "3d" };
const renderer = { render() { renderCalls += 1; order.push("render"); } };
const scene = {};
const camera = {};
const updateCamera = () => { cameraUpdates += 1; order.push("camera"); };
const pageTransition = { active: false, expectsGrid: false };
const glassMotionReduced = () => reduceMotion;
const resizeAll = () => { resizeCalls += 1; order.push("resize"); };
const fpsMeter = { markFrame() {}, setActive() {} };
const requestAnimationFrame = (fn) => { const id = nextFrame++; frames.set(id, fn); return id; };
const cancelAnimationFrame = (id) => { frames.delete(id); };

const api = new Function(
  "state", "renderer", "scene", "camera", "updateCamera",
  "requestAnimationFrame", "cancelAnimationFrame",
  "pageTransition", "glassMotionReduced", "resizeAll", "fpsMeter",
  block + `
return { mapViewActive, render, ensureMapRenderLoop, pauseMapRenderLoop, frameId: () => mapRenderFrame };`,
)(state, renderer, scene, camera, updateCamera, requestAnimationFrame, cancelAnimationFrame,
   pageTransition, glassMotionReduced, resizeAll, fpsMeter);
const { mapViewActive, render, ensureMapRenderLoop, pauseMapRenderLoop, frameId } = api;

function expect(condition, message) { if (!condition) throw new Error(message); }
function flushFrame() {
  const pending = Array.from(frames.values());
  frames.clear();
  pending.forEach((fn) => fn());
}

// Repeated ensure in a map view produces exactly one pending rAF.
state.view = "3d";
expect(mapViewActive() === true, "3d is a map view");
ensureMapRenderLoop();
ensureMapRenderLoop();
ensureMapRenderLoop();
expect(frames.size === 1, "repeated ensure schedules one pending rAF");
expect(frameId() !== null, "the pending frame id is tracked");

// Running that frame renders once and schedules exactly one follow-up frame.
renderCalls = 0;
cameraUpdates = 0;
flushFrame();
expect(renderCalls === 1, "the map frame renders once");
expect(cameraUpdates === 1, "the map frame updates the camera once");
expect(frames.size === 1, "the map render schedules exactly one next frame");
expect(frameId() !== null, "the follow-up frame id is tracked");

// Pause cancels and nulls the pending rAF.
pauseMapRenderLoop();
expect(frames.size === 0, "pause cancels the pending frame");
expect(frameId() === null, "pause nulls the tracked frame id");

// System is not a map view: ensure and render never schedule.
state.view = "system";
expect(mapViewActive() === false, "system is not a map view");
ensureMapRenderLoop();
expect(frames.size === 0, "system never schedules through ensure");
renderCalls = 0;
render();
expect(renderCalls === 1, "a direct render still draws");
expect(frames.size === 0, "system does not reschedule a frame after render");
expect(frameId() === null, "system leaves no tracked frame");

// Mapping is a map view and schedules exactly one frame.
state.view = "mapping";
expect(mapViewActive() === true, "mapping is a map view");
ensureMapRenderLoop();
expect(frames.size === 1, "mapping schedules one pending frame");

// --- Live viewport sync during a grid-expecting transition ---
// A render frame in a map view during an active grid transition must resize the
// cached drawing buffer before updating the camera and drawing, so the old
// buffer is never stretched to the new CSS box.
pageTransition.active = true;
pageTransition.expectsGrid = true;
reduceMotion = false;
state.view = "3d";
order.length = 0;
resizeCalls = 0;
renderCalls = 0;
cameraUpdates = 0;
render();
expect(resizeCalls === 1, "an active grid transition in a map view resizes the buffer");
expect(order.join(">") === "resize>camera>render", "resize precedes camera and draw");

// Negative gate: a pure view transition never live-resizes.
pageTransition.expectsGrid = false;
resizeCalls = 0;
order.length = 0;
render();
expect(resizeCalls === 0, "a pure view transition never live-resizes");

// Negative gate: an inactive transition never live-resizes.
pageTransition.active = false;
pageTransition.expectsGrid = true;
resizeCalls = 0;
render();
expect(resizeCalls === 0, "an inactive transition never live-resizes");

// Negative gate: reduced motion keeps the next-frame settle path.
pageTransition.active = true;
pageTransition.expectsGrid = true;
reduceMotion = true;
resizeCalls = 0;
render();
expect(resizeCalls === 0, "reduced motion never live-resizes");

// Negative gate: a hidden/system view never live-resizes.
reduceMotion = false;
state.view = "system";
resizeCalls = 0;
render();
expect(resizeCalls === 0, "a system view never live-resizes");

console.log("render loop ok");
"""


def test_map_render_loop_lifecycle_executes_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _RENDER_LOOP_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "render loop ok" in completed.stdout


_RESIZE_CACHE_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const fnStart = source.indexOf("function resizeAll()");
if (fnStart < 0) throw new Error("resizeAll not found");
const declStart = source.lastIndexOf("let lastRenderWidth", fnStart);
let sliceStart = fnStart;
if (declStart >= 0 && fnStart - declStart < 400) sliceStart = declStart;
const brace = source.indexOf("{", fnStart);
let depth = 0;
let end = -1;
for (let i = brace; i < source.length; i += 1) {
  if (source[i] === "{") depth += 1;
  else if (source[i] === "}") {
    depth -= 1;
    if (depth === 0) { end = i + 1; break; }
  }
}
if (end < 0) throw new Error("unterminated resizeAll");
const block = source.slice(sliceStart, end);

const sizes = [];
let updateProjectionCalls = 0;
const renderer = { setSize(width, height, updateStyle) { sizes.push([width, height, updateStyle]); } };
const camera = { aspect: 0, updateProjectionMatrix() { updateProjectionCalls += 1; } };
let rect = { width: 800.4, height: 600.6 };
const mapCanvas3d = { getBoundingClientRect: () => rect };

const api = new Function(
  "renderer", "camera", "mapCanvas3d",
  block + "\nreturn { resizeAll };",
)(renderer, camera, mapCanvas3d);
const { resizeAll } = api;

function expect(condition, message) { if (!condition) throw new Error(message); }

const firstChanged = resizeAll();
const secondChanged = resizeAll();
expect(sizes.length === 1, "two same-size resizes must call renderer.setSize once");
expect(updateProjectionCalls === 1, "two same-size resizes must update the projection once");
expect(firstChanged === true, "the first resize reports a changed buffer");
expect(secondChanged === false, "an identical resize reports no change");

rect = { width: 900.2, height: 600.6 };
const thirdChanged = resizeAll();
expect(sizes.length === 2, "a changed size must call renderer.setSize again");
expect(updateProjectionCalls === 2, "a changed size must update the projection again");
expect(thirdChanged === true, "a changed size reports a changed buffer");
expect(sizes[0][0] === 800 && sizes[0][1] === 601, "the drawing buffer uses integer dimensions");
expect(sizes[1][0] === 900 && sizes[1][1] === 601, "the changed buffer uses integer dimensions");
expect(sizes[0][2] === false, "renderer.setSize must not update the canvas CSS style");
expect(sizes[1][2] === false, "renderer.setSize keeps the CSS style untouched on resize");

console.log("resize cache ok");
"""


def test_resize_all_skips_identical_drawing_buffer_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _RESIZE_CACHE_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "resize cache ok" in completed.stdout


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
        name: SourceCache(name, 2.0, static_source=name != "obstacles")
        for name in ("global_map", "local_map", "mapcloud", "mapground", "obstacles")
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


def test_readme_documents_liquid_glass_transitions():
    readme = (Path(__file__).resolve().parents[1] / "README.md").read_text(
        encoding="utf-8"
    )

    assert "Liquid Glass" in readme
    assert "shuding/liquid-glass" in readme
    assert "SVG" in readme
    assert "?glass=" in readme
    assert "prefers-reduced-motion" in readme
    assert "prefers-reduced-transparency" not in readme
    assert "Glass Strength" in readme
    assert "web_control_glass_strength" in readme
    assert "localStorage" in readme
    assert "650ms" in readme
    assert "--layout-slide-ms" in readme
    assert "--fade-ms" in readme
    assert "--thumb-ms" in readme
    assert "420ms" in readme
    assert "220ms" in readme
    assert "--joystick-reveal-ms" in readme
    assert "--joystick-hide-ms" in readme
    assert "无白闪的实时 resize" in readme
    assert "渲染循环每帧" in readme
    assert "唯一的 `renderer.setSize` 调用点" in readme
    assert "独立实现" in readme
    assert "liquidTrackFilter" in readme
    assert "THIRD_PARTY_NOTICES" in readme
    assert "MIT" in readme
    assert "未复制上游代码或素材" not in readme
    assert "无需附带上游 MIT 声明" not in readme
    assert "config/web_control.yaml" not in readme


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
    assert parameters["teleop_max_linear_speed"] == 1.0
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
    assert 'id="throttleStick"' in viewport
    assert 'id="steeringStick"' in viewport
    assert 'id="throttleBase"' in viewport
    assert 'id="steeringBase"' in viewport
    assert 'id="throttleKnob"' in viewport
    assert 'id="steeringKnob"' in viewport
    # The single-joystick status text is gone for good.
    assert 'id="joystickStatus"' not in index_html

    assert "setMappingJoystickVisible(true, { deferReveal: deferJoystickReveal });" in app_js
    assert "setMappingJoystickVisible(false);" in app_js
    assert ".joystick {" in styles_css
    assert "position: absolute;" in styles_css
    assert "--stick-size" in styles_css
    assert ".stage.hud-center .hud" in styles_css
    assert 'id="joystickSettings"' in viewport
    assert 'id="throttleMoveHandle"' in viewport
    assert 'id="steeringResizeHandle"' in viewport


def test_joystick_reveal_and_hide_css_contract():
    index_html, app_js, styles_css = _frontend_sources()

    base = _css_rule_block(styles_css, ".joystick")
    # A not-yet-revealed joystick must never be interactive.
    assert "pointer-events: none;" in base
    assert "opacity: 0;" in base

    concealed = _css_rule_block(styles_css, ".joystick.joystick-concealed")
    assert "opacity: 0;" in concealed
    assert "pointer-events: none;" in concealed
    assert "var(--joystick-hide-ms)" in concealed
    assert "var(--layout-ease)" in concealed

    revealed = _css_rule_block(styles_css, ".joystick.joystick-revealed")
    assert "opacity: 1;" in revealed
    assert "var(--joystick-reveal-ms)" in revealed
    assert "var(--layout-ease)" in revealed

    # The full-viewport layer is never itself interactive; only its sticks and
    # shared controls accept pointers so the map canvas keeps its gestures.
    interactive = _css_rule_block(
        styles_css, ".joystick.joystick-revealed .joystick-stick"
    )
    assert "pointer-events: auto;" in interactive

    for block in (base, concealed, revealed):
        assert "filter" not in block
        assert "backdrop-filter" not in block

    assert ".joystick[hidden]" in styles_css

    assert "const JOYSTICK_REVEAL_MS = 420;" in app_js
    assert "const JOYSTICK_HIDE_MS = 220;" in app_js
    assert "const mappingJoystickVisibility = {" in app_js
    assert "pendingReveal" in app_js
    assert "deferReveal" in app_js
    for name in (
        "cancelMappingJoystickVisibility",
        "concealMappingJoystick",
        "revealMappingJoystick",
        "scheduleMappingJoystickReveal",
        "flushPendingMappingJoystickReveal",
        "setMappingJoystickVisible",
    ):
        assert f"function {name}(" in app_js

    block = app_js[
        app_js.index("// --- Mapping joystick") : app_js.index("// --- End mapping joystick")
    ]
    assert "localStorage" not in block
    assert "sessionStorage" not in block
    assert "token !== mappingJoystickVisibility.token" in block
    assert "joystick.hidden" in app_js
    assert "flushPendingMappingJoystickReveal();" in app_js


_JOYSTICK_VISIBILITY_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Mapping joystick");
const blockEnd = source.indexOf("// --- End mapping joystick");
if (blockStart < 0 || blockEnd < 0) throw new Error("mapping joystick block not found");
const block = source.slice(blockStart, blockEnd);

function makeElement() {
  const classes = new Set();
  return {
    hidden: true,
    inert: false,
    attributes: {},
    classList: {
      add(name) { classes.add(name); },
      remove(name) { classes.delete(name); },
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

const joystick = makeElement();
const stage = makeElement();
global.document = {
  getElementById(id) { return id === "mappingJoystick" ? joystick : null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
};

const timers = new Map();
const frames = new Map();
// cancelAnimationFrame does not guarantee the callback never runs (it may
// already be queued), so keep the cancelled callbacks and let the test invoke
// them directly to prove the token guard, not the cancellation, is what
// suppresses stale reveals.
const cancelledFrames = [];
let nextTimer = 1;
let nextFrame = 1;
global.setTimeout = (fn) => { const id = nextTimer++; timers.set(id, fn); return id; };
global.clearTimeout = (id) => { timers.delete(id); };
global.requestAnimationFrame = (fn) => { const id = nextFrame++; frames.set(id, fn); return id; };
global.cancelAnimationFrame = (id) => {
  if (frames.has(id)) cancelledFrames.push(frames.get(id));
  frames.delete(id);
};
let reduceMotion = false;
global.glassMotionReduced = () => reduceMotion;
let layoutCalls = 0;
global.applyJoystickLayout = () => { layoutCalls += 1; };
let stopCalls = 0;
global.stopTeleop = async () => { stopCalls += 1; };
global.viewerSessionId = () => "tab-session";

const api = new Function(block + `
stopTeleop = global.stopTeleop;
return { setMappingJoystickVisible, mappingJoystickVisibility, flushPendingMappingJoystickReveal, teleop };`)();
const {
  setMappingJoystickVisible, mappingJoystickVisibility,
  flushPendingMappingJoystickReveal, teleop,
} = api;

function expect(condition, message) { if (!condition) throw new Error(message); }
function flushFrames() {
  const pending = Array.from(frames.values());
  frames.clear();
  pending.forEach((fn) => fn());
}
function flushTimers() {
  const pending = Array.from(timers.values());
  timers.clear();
  pending.forEach((fn) => fn());
}

(async () => {
  // Immediate show (3d -> Mapping): exposed while concealed, measured at final
  // geometry, then revealed on the next frame.
  layoutCalls = 0;
  await setMappingJoystickVisible(true);
  expect(joystick.hidden === false, "show removes hidden before measuring");
  expect(joystick.classList.contains("joystick-concealed"), "show starts concealed");
  expect(!joystick.classList.contains("joystick-revealed"), "show does not reveal before layout");
  expect(joystick.inert === true, "show blocks input until revealed");
  expect(joystick.getAttribute("aria-hidden") === "true", "concealed joystick is aria-hidden");
  expect(layoutCalls === 1, "immediate show applies joystick layout exactly once while concealed");
  expect(frames.size === 1, "immediate show schedules exactly one reveal frame");
  flushFrames();
  expect(joystick.classList.contains("joystick-revealed"), "the reveal frame reveals the joystick");
  expect(!joystick.classList.contains("joystick-concealed"), "the reveal frame clears concealment");
  expect(joystick.inert === false, "reveal restores interaction");
  expect(joystick.getAttribute("aria-hidden") === "false", "revealed joystick is exposed");
  expect(mappingJoystickVisibility.visible === true, "visibility state records the reveal");

  // Deferred show (System -> Mapping): stay concealed and unpositioned until the
  // settled-layout hook measures the final geometry and schedules the reveal.
  await setMappingJoystickVisible(false);
  flushTimers();
  layoutCalls = 0;
  await setMappingJoystickVisible(true, { deferReveal: true });
  expect(joystick.hidden === false, "deferred show exposes the concealed joystick");
  expect(joystick.classList.contains("joystick-concealed"), "deferred show stays concealed");
  expect(joystick.inert === true, "deferred show stays inert");
  expect(mappingJoystickVisibility.pendingReveal === true, "deferred show records a pending reveal");
  expect(!joystick.classList.contains("joystick-revealed"), "deferred show does not reveal early");
  expect(layoutCalls === 0, "deferred show does not measure mid-transition");
  expect(frames.size === 0, "deferred show schedules no reveal frame yet");
  // Simulate doDeferredLayoutFlush: reposition at settled geometry, then reveal.
  applyJoystickLayout();
  flushPendingMappingJoystickReveal();
  expect(layoutCalls === 1, "the settled-layout hook measures exactly once");
  expect(mappingJoystickVisibility.pendingReveal === false, "the hook consumes the pending reveal");
  expect(frames.size === 1, "the settled-layout hook schedules the reveal frame");
  expect(!joystick.classList.contains("joystick-revealed"), "the reveal still waits for its frame");
  flushFrames();
  expect(joystick.classList.contains("joystick-revealed"), "the deferred joystick reveals after settle");
  expect(mappingJoystickVisibility.visible === true, "deferred reveal records visibility");

  // Hiding before the transition settles prevents the pending reveal.
  await setMappingJoystickVisible(false);
  flushTimers();
  layoutCalls = 0;
  await setMappingJoystickVisible(true, { deferReveal: true });
  expect(mappingJoystickVisibility.pendingReveal === true, "second deferred show is pending");
  await setMappingJoystickVisible(false);
  expect(mappingJoystickVisibility.pendingReveal === false, "hide clears the pending reveal");
  flushPendingMappingJoystickReveal();
  expect(frames.size === 0, "a cancelled pending reveal schedules no frame");
  expect(layoutCalls === 0, "a cancelled pending reveal does not measure");
  expect(!joystick.classList.contains("joystick-revealed"), "hide before settle prevents reveal");
  flushTimers();
  expect(joystick.hidden === true, "hide commits display:none");

  // Rapid switch: a stale reveal frame must not resurrect a hidden joystick.
  const showing = setMappingJoystickVisible(true);
  await setMappingJoystickVisible(false);
  expect(joystick.classList.contains("joystick-concealed"), "hide conceals immediately");
  expect(joystick.inert === true, "hide blocks input immediately");
  expect(joystick.hidden === false, "hide keeps the element mounted during the fade");
  expect(timers.size === 1, "hide arms exactly one hidden timeout");
  expect(cancelledFrames.length === 1, "hide captures the cancelled reveal frame");
  // A browser may still execute a frame callback that was cancelled after it
  // was queued; invoking the captured callback directly proves the token guard
  // (not the cancellation) rejects the stale reveal.
  const staleReveal = cancelledFrames.pop();
  staleReveal();
  expect(
    !joystick.classList.contains("joystick-revealed"),
    "a captured stale reveal callback is ignored by the token guard",
  );
  flushFrames();
  expect(!joystick.classList.contains("joystick-revealed"), "a stale reveal frame is ignored");
  flushTimers();
  expect(joystick.hidden === true, "the hide timeout commits display:none");
  await showing;

  // Repeated immediate show cancels the older frame and still reveals.
  await setMappingJoystickVisible(true);
  await setMappingJoystickVisible(true);
  expect(frames.size === 1, "the newer show cancels the older reveal frame");
  flushFrames();
  expect(joystick.classList.contains("joystick-revealed"), "the newest show still reveals");

  // Hide stops teleop exactly once.
  teleop.active = true;
  const stopBefore = stopCalls;
  await setMappingJoystickVisible(false);
  expect(stopCalls === stopBefore + 1, "hide requests exactly one teleop stop");
  expect(joystick.hidden === false, "hide waits for the fade before display:none");
  flushTimers();
  expect(joystick.hidden === true, "hide commits display:none after the fade");
  teleop.active = false;

  // Reduced motion snaps both directions without frames or timers.
  reduceMotion = true;
  joystick.hidden = true;
  await setMappingJoystickVisible(true);
  expect(joystick.classList.contains("joystick-revealed"), "reduced motion reveals synchronously");
  expect(frames.size === 0, "reduced motion schedules no reveal frame");
  await setMappingJoystickVisible(false);
  expect(joystick.hidden === true, "reduced motion hides synchronously");
  expect(timers.size === 0, "reduced motion arms no hide timeout");

  console.log("joystick visibility ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""


def test_joystick_visibility_lifecycle_executes_under_node():
    completed = _run_app_node_harness(_JOYSTICK_VISIBILITY_NODE_HARNESS)
    assert "joystick visibility ok" in completed.stdout


_SET_TAB_REENTRANT_GRID_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");

function sliceBlock(startMarker, endMarker) {
  const start = source.indexOf(startMarker);
  const end = source.indexOf(endMarker, start);
  if (start < 0 || end < 0) throw new Error("missing block " + startMarker);
  return source.slice(start, end);
}
function sliceFunction(signature) {
  const start = source.indexOf(signature);
  if (start < 0) throw new Error("missing " + signature);
  let index = source.indexOf("{", start);
  let depth = 0;
  for (; index < source.length; index += 1) {
    const ch = source[index];
    if (ch === "{") depth += 1;
    else if (ch === "}") {
      depth -= 1;
      if (depth === 0) return source.slice(start, index + 1);
    }
  }
  throw new Error("unterminated " + signature);
}

const sidebarBlock = sliceBlock("// --- Sidebar collapse", "// --- End sidebar collapse");
const transitionBlock = sliceBlock(
  "// --- Page transition state machine",
  "// --- End page transition state machine",
);
const joystickBlock = sliceBlock("// --- Mapping joystick", "// --- End mapping joystick");
const setTabSource = sliceFunction("function setTab(tab)");
// The real live-resize gate reads pageTransition, so pulling it into this
// integration harness proves the state set by setTab actually enables resize.
const syncViewportSource = sliceFunction("function syncMapViewportDuringGridTransition()");

function makeClassList() {
  const classes = new Set();
  return {
    add(name) { classes.add(name); },
    remove(name) { classes.delete(name); },
    contains(name) { return classes.has(name); },
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
  };
}
function makeElement(id, dataset) {
  return {
    id,
    hidden: false,
    inert: false,
    attributes: {},
    dataset: dataset || {},
    classList: makeClassList(),
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
    contains() { return false; },
    focus() { global.document.activeElement = this; },
  };
}

const elements = {};
function register(id, classes) {
  const element = makeElement(id);
  (classes || []).forEach((name) => element.classList.add(name));
  elements[id] = element;
  return element;
}
const panels = [
  makeElement("panel-navigation", { panel: "navigation" }),
  makeElement("panel-mapping", { panel: "mapping" }),
  makeElement("panel-system", { panel: "" }),
];
panels[0].classList.add("active");

const view3d = register("view3d", ["active"]);
const viewSystem = register("viewSystem");
const viewTabs = register("viewTabs");
viewTabs.dataset.active = "3d";
register("tab3dMap", ["active"]);
register("tabMapping");
register("tabSystem");
const appEl = register("app");
register("controlPanel");
register("sidebarToggle");
const joystick = register("mappingJoystick");
joystick.hidden = true;
const stage = register("stage");

global.document = {
  body: makeElement("body"),
  activeElement: null,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) {
    if (selector === ".app") return appEl;
    if (selector === ".stage") return stage;
    if (selector === ".panel-view.active") {
      return panels.find((panel) => panel.classList.contains("active")) || null;
    }
    return null;
  },
  querySelectorAll(selector) {
    if (selector === "[data-panel]") return panels;
    return [];
  },
};

let layoutCalls = 0;
let liveResizeCalls = 0;
global.state = { view: "3d" };
global.mapViewActive = () => global.state.view === "3d" || global.state.view === "mapping";
global.layerGroup = { visible: false };
global.pathGroup = { visible: false };
global.markerGroup = { visible: false };
global.vehicleGroup = { visible: false };
global.mappingLayerGroup = { visible: false };
global.mappingPathGroup = { visible: false };
global.mappingVehicleGroup = { visible: false };
global.viewerSessionId = () => "tab-session";
global.stopTeleop = async () => {};
global.glassMotionReduced = () => false;
global.applyJoystickLayout = () => { layoutCalls += 1; };
global.resizeAll = () => { liveResizeCalls += 1; };
global.updateHudPlacement = () => {};
global.renderer = { render() {} };
global.scene = {};
global.camera = {};
global.exitJoystickLayout = () => {};
global.markGlassDrop = () => {};
global.ensureMapRenderLoop = () => {};
global.pauseMapRenderLoop = () => {};
global.pollMaps3d = () => {};
global.pollMappingState = () => {};
global.pollMapping3d = () => {};
global.pollMappingPath = () => {};
global.pollSystem = () => {};
global.refreshController = { tick() {} };
global.focusIfInside = () => {};

const timers = new Map();
const frames = new Map();
let nextTimer = 1;
let nextFrame = 1;
global.setTimeout = (fn) => { const id = nextTimer++; timers.set(id, fn); return id; };
global.clearTimeout = (id) => { timers.delete(id); };
global.requestAnimationFrame = (fn) => { const id = nextFrame++; frames.set(id, fn); return id; };
global.cancelAnimationFrame = (id) => { frames.delete(id); };

const api = new Function([
  sidebarBlock,
  transitionBlock,
  joystickBlock,
  // setTab repositions the segmented capsule through this module-level handle;
  // the harness declares it null so the existing switch path still runs.
  "let viewTabsControl = null;",
  setTabSource,
  syncViewportSource,
  `
stopTeleop = global.stopTeleop;
return {
  setTab,
  pageTransition,
  settleLayoutTransition,
  mappingJoystickVisibility,
  flushPendingMappingJoystickReveal,
  syncMapViewportDuringGridTransition,
};`,
].join("\n"))();
const {
  setTab, pageTransition, settleLayoutTransition,
  mappingJoystickVisibility, flushPendingMappingJoystickReveal,
  syncMapViewportDuringGridTransition,
} = api;

function expect(condition, message) { if (!condition) throw new Error(message); }
function flushFrames() {
  const pending = Array.from(frames.values());
  frames.clear();
  pending.forEach((fn) => fn());
}
function flushTimers() {
  const pending = Array.from(timers.values());
  timers.clear();
  pending.forEach((fn) => fn());
}

(async () => {
  // Normal 3d -> Mapping stays immediate: a pure view/panel transition is
  // active, but it does not expect the grid, so the reveal is not deferred.
  layoutCalls = 0;
  frames.clear();
  setTab("mapping");
  expect(pageTransition.active === true, "3d->Mapping arms a layout transition");
  expect(pageTransition.expectsGrid === false, "3d->Mapping does not expect the grid");
  expect(
    mappingJoystickVisibility.pendingReveal === false,
    "3d->Mapping does not defer the reveal",
  );
  expect(layoutCalls === 1, "3d->Mapping measures the joystick immediately");
  expect(frames.size === 1, "3d->Mapping schedules the reveal frame immediately");
  expect(joystick.classList.contains("joystick-concealed"), "immediate show starts concealed");
  expect(joystick.inert === true, "immediate show starts inert");
  flushFrames();
  expect(joystick.classList.contains("joystick-revealed"), "3d->Mapping reveals on its frame");

  // Return to a stable System state before exercising the grid case.
  setTab("system");
  flushTimers();
  frames.clear();
  layoutCalls = 0;

  // System -> Mapping: the sidebar grid starts sliding, so the reveal defers.
  setTab("mapping");
  expect(pageTransition.active === true, "System->Mapping arms the grid transition");
  expect(pageTransition.expectsGrid === true, "System->Mapping expects the grid");
  expect(mappingJoystickVisibility.pendingReveal === true, "System->Mapping defers the reveal");
  expect(joystick.classList.contains("joystick-concealed"), "deferred show stays concealed");
  expect(joystick.inert === true, "deferred show stays inert");
  expect(layoutCalls === 0, "deferred show does not measure mid-slide");
  expect(frames.size === 0, "deferred show schedules no reveal frame");

  // Re-entrant Mapping click while the grid is still in flight: the sidebar no
  // longer changes, so sidebarChanged is false, but the active transition still
  // expects the grid. The reveal must stay pending rather than clearing it and
  // measuring/revealing mid-slide.
  const inFlightToken = pageTransition.token;
  setTab("mapping");
  expect(pageTransition.token === inFlightToken, "the re-entrant click arms no new transition");
  expect(
    pageTransition.active === true && pageTransition.expectsGrid === true,
    "the grid transition is still in flight",
  );
  expect(
    mappingJoystickVisibility.pendingReveal === true,
    "the re-entrant click keeps the reveal pending",
  );
  expect(joystick.classList.contains("joystick-concealed"), "the re-entrant click stays concealed");
  expect(
    !joystick.classList.contains("joystick-revealed"),
    "the re-entrant click never reveals mid-slide",
  );
  expect(joystick.inert === true, "the re-entrant click stays inert");
  expect(layoutCalls === 0, "the re-entrant click does not measure mid-slide");
  expect(frames.size === 0, "the re-entrant click schedules no reveal frame");

  // Settling the grid repositions once and only then schedules the reveal.
  settleLayoutTransition(pageTransition.token);
  expect(pageTransition.active === false, "the grid transition settles");
  expect(layoutCalls === 1, "the settled flush measures exactly once");
  expect(
    mappingJoystickVisibility.pendingReveal === false,
    "the settled flush consumes the pending reveal",
  );
  expect(frames.size === 1, "the settled flush schedules exactly one reveal frame");
  expect(!joystick.classList.contains("joystick-revealed"), "the reveal still waits for its frame");
  flushFrames();
  expect(joystick.classList.contains("joystick-revealed"), "the deferred reveal completes after settle");

  // System -> Mapping -> 3d: the System->Mapping begin expects the grid, then a
  // pure Mapping->3d begin arrives while the grid slide is still physically
  // running. The pure begin must NOT clear expectsGrid, so live resize stays
  // eligible and a later Mapping reveal still defers until the grid settles.
  setTab("system");
  flushTimers();
  setTab("mapping");
  expect(pageTransition.active === true, "System->Mapping arms the sticky grid run");
  expect(pageTransition.expectsGrid === true, "System->Mapping expects the grid");
  setTab("3d");
  expect(pageTransition.active === true, "the pure begin keeps the run active");
  expect(
    pageTransition.expectsGrid === true,
    "the pure begin during the grid slide preserves expectsGrid",
  );
  liveResizeCalls = 0;
  syncMapViewportDuringGridTransition();
  expect(
    liveResizeCalls === 1,
    "live resize stays eligible while the sticky grid run is in flight",
  );

  // A later Mapping click in the same in-flight run must still defer its reveal.
  frames.clear();
  layoutCalls = 0;
  setTab("mapping");
  expect(
    mappingJoystickVisibility.pendingReveal === true,
    "the later Mapping reveal defers until the sticky grid run settles",
  );
  expect(joystick.classList.contains("joystick-concealed"), "the later reveal stays concealed");
  expect(layoutCalls === 0, "the later reveal does not measure mid-slide");
  expect(frames.size === 0, "the later reveal schedules no frame before settle");

  // The grid settles, and only then does the deferred reveal complete.
  settleLayoutTransition(pageTransition.token);
  expect(pageTransition.active === false, "the sticky grid run settles");
  expect(pageTransition.expectsGrid === false, "settling clears the sticky expectation");
  expect(layoutCalls === 1, "the settle measures the deferred reveal once");
  expect(
    mappingJoystickVisibility.pendingReveal === false,
    "the settle consumes the pending reveal",
  );
  flushFrames();
  expect(joystick.classList.contains("joystick-revealed"), "the later reveal completes after settle");

  // A hide while the deferred reveal is pending must cancel it.
  setTab("3d");
  flushTimers();
  setTab("system");
  flushTimers();
  frames.clear();
  layoutCalls = 0;
  setTab("mapping");
  expect(mappingJoystickVisibility.pendingReveal === true, "entering Mapping defers the reveal again");
  setTab("3d");
  expect(
    mappingJoystickVisibility.pendingReveal === false,
    "hide during the wait clears the pending reveal",
  );
  expect(joystick.classList.contains("joystick-concealed"), "hide during the wait stays concealed");
  expect(!joystick.classList.contains("joystick-revealed"), "hide during the wait prevents reveal");
  flushPendingMappingJoystickReveal();
  expect(frames.size === 0, "a cancelled pending reveal schedules no frame");
  expect(layoutCalls === 0, "a cancelled pending reveal does not measure");

  console.log("set tab reentrant grid ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""


def test_set_tab_reentrant_grid_defers_joystick_reveal_under_node():
    completed = _run_app_node_harness(_SET_TAB_REENTRANT_GRID_NODE_HARNESS)
    assert "set tab reentrant grid ok" in completed.stdout


def test_mapping_joystick_exposes_adjustable_layout_controls():
    index_html, app_js, styles_css = _frontend_sources()

    assert 'id="throttleMoveHandle"' in index_html
    assert 'id="steeringMoveHandle"' in index_html
    assert 'id="throttleResizeHandle"' in index_html
    assert 'id="steeringResizeHandle"' in index_html
    assert 'id="joystickSettings"' in index_html
    assert 'id="joystickDone"' in index_html
    assert 'id="joystickSettingsError"' in index_html
    assert 'aria-label="Joystick settings"' in index_html

    assert "--stick-size" in styles_css
    assert ".joystick.layout-mode" in styles_css
    assert ".stage.hud-center .hud" in styles_css
    assert "function applyJoystickLayout()" in app_js
    assert "joystick.hidden" in app_js
    assert "if (!viewportWidth || !viewportHeight) {" in app_js
    assert "const mappingJoystickVisibility = {" in app_js
    assert "function scheduleMappingJoystickReveal(" in app_js
    assert "function flushPendingMappingJoystickReveal(" in app_js
    assert "function clampedJoystickSize(value)" in app_js
    assert "function updateHudPlacement()" in app_js
    assert "function enterJoystickLayout()" in app_js
    assert "function exitJoystickLayout()" in app_js
    assert 'getJson("/api/ui/preferences")' in app_js
    assert 'postJson("/api/ui/preferences"' in app_js
    assert "teleop.layoutMode" in app_js


def _run_app_node_harness(harness):
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", harness],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    return completed


def test_sidebar_toggle_is_single_accessible_button():
    index_html, app_js, styles_css = _frontend_sources()

    assert index_html.count('id="sidebarToggle"') == 1
    assert 'aria-controls="controlPanel"' in index_html
    assert 'id="controlPanel"' in index_html
    assert 'class="sidebar-toggle"' in index_html

    assert "grid-template-columns: minmax(0, 1fr) var(--panel-width, 360px);" in styles_css
    assert ".app.sidebar-collapsed" in styles_css
    assert "grid-template-columns var(--layout-slide-ms) var(--layout-ease)" in styles_css
    assert ".app.sidebar-collapsed .panel" in styles_css
    assert ".app.system-mode .sidebar-toggle" in styles_css
    assert "var(--panel-width, 340px);" in styles_css

    assert "function setSidebarCollapsed(collapsed)" in app_js
    assert "function toggleSidebar()" in app_js
    assert "function observeMapStageResize()" in app_js
    assert "function applyEffectiveSidebar()" in app_js
    assert "new ResizeObserver" in app_js
    assert 'document.getElementById("sidebarToggle").addEventListener' in app_js


_SIDEBAR_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Sidebar collapse");
const blockEnd = source.indexOf("// --- End sidebar collapse");
if (blockStart < 0 || blockEnd < 0) throw new Error("sidebar block not found");
const block = source.slice(blockStart, blockEnd);
if (block.includes("localStorage") || block.includes("sessionStorage")) {
  throw new Error("sidebar state must not be persisted");
}

function makeElement() {
  const classes = new Set();
  return {
    hidden: false,
    inert: false,
    attributes: {},
    focused: false,
    classList: {
      toggle(name, on) { if (on) classes.add(name); else classes.delete(name); },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
    contains() { return false; },
    focus() { this.focused = true; },
  };
}

const appElement = makeElement();
const stageElement = makeElement();
const toggleElement = makeElement();
const panelElement = makeElement();
const elements = { sidebarToggle: toggleElement, controlPanel: panelElement };
const observers = [];
global.document = {
  activeElement: null,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) {
    if (selector === ".app") return appElement;
    if (selector === ".stage") return stageElement;
    return null;
  },
};
global.ResizeObserver = class {
  constructor(callback) { this.callback = callback; this.target = null; observers.push(this); }
  observe(target) { this.target = target; }
};
let transitions = 0;
let resizeNotes = 0;
global.state = { view: "3d" };
global.beginLayoutTransition = () => { transitions += 1; return transitions; };
global.noteLayoutResize = () => { resizeNotes += 1; };

const sidebarApi = new Function(block + `
return { sidebarState, applyEffectiveSidebar, setSidebarCollapsed, toggleSidebar, observeMapStageResize };`)();
const { sidebarState, applyEffectiveSidebar, setSidebarCollapsed, toggleSidebar, observeMapStageResize } = sidebarApi;

function expect(condition, message) { if (!condition) throw new Error(message); }

expect(sidebarState.collapsed === false, "sidebar starts expanded");
setSidebarCollapsed(false);
expect(toggleElement.getAttribute("aria-expanded") === "true", "expanded aria state");
expect(toggleElement.getAttribute("aria-label") === "Collapse sidebar", "expanded label");
expect(panelElement.inert === false, "expanded panel is interactive");
expect(transitions === 0, "no transition for an unchanged sidebar");

toggleSidebar();
expect(sidebarState.collapsed === true, "toggle collapses");
expect(appElement.classList.contains("sidebar-collapsed"), "collapsed class applied");
expect(toggleElement.getAttribute("aria-expanded") === "false", "collapsed aria state");
expect(toggleElement.getAttribute("aria-label") === "Expand sidebar", "collapsed label");
expect(panelElement.inert === true, "collapsed panel is inert");
expect(transitions === 1, "collapse starts one transition");

toggleSidebar();
expect(sidebarState.collapsed === false, "toggle expands");
expect(!appElement.classList.contains("sidebar-collapsed"), "collapsed class removed");
expect(toggleElement.getAttribute("aria-expanded") === "true", "re-expanded aria state");
expect(panelElement.inert === false, "re-expanded panel is interactive");
expect(transitions === 2, "expand starts one transition");

// Entering System collapses the effective sidebar but keeps the in-page state.
global.state.view = "system";
expect(applyEffectiveSidebar() === true, "entering system changes the effective collapse");
expect(appElement.classList.contains("sidebar-collapsed"), "system collapses the sidebar");
expect(sidebarState.collapsed === false, "in-page state is unchanged by system");
expect(toggleElement.getAttribute("aria-label") === "Collapse sidebar", "system keeps the in-page label");

// Leaving System restores the in-page expanded state.
global.state.view = "3d";
expect(applyEffectiveSidebar() === true, "leaving system changes the effective collapse");
expect(!appElement.classList.contains("sidebar-collapsed"), "leaving system restores expansion");
expect(applyEffectiveSidebar() === false, "an unchanged effective state starts no work");

observeMapStageResize();
expect(observers.length === 1, "one ResizeObserver created");
expect(observers[0].target === stageElement, "map stage observed");
observers[0].callback();
expect(resizeNotes === 1, "stage changes are coalesced through noteLayoutResize");

console.log("sidebar collapse ok");
"""


def test_sidebar_collapse_state_toggles_without_persistence_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _SIDEBAR_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "sidebar collapse ok" in completed.stdout


_SIDEBAR_FOCUS_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Sidebar collapse");
const blockEnd = source.indexOf("// --- End sidebar collapse");
if (blockStart < 0 || blockEnd < 0) throw new Error("sidebar block not found");
const block = source.slice(blockStart, blockEnd);

function makeElement() {
  const classes = new Set();
  return {
    hidden: false,
    inert: false,
    attributes: {},
    focused: false,
    containsActive: false,
    classList: {
      toggle(name, on) { if (on) classes.add(name); else classes.delete(name); },
      add(name) { classes.add(name); },
      remove(name) { classes.delete(name); },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
    contains() { return this.containsActive; },
    focus() { this.focused = true; },
  };
}

const appElement = makeElement();
const panelElement = makeElement();
const toggleElement = makeElement();
const systemTabElement = makeElement();
const elements = {
  sidebarToggle: toggleElement,
  controlPanel: panelElement,
  tabSystem: systemTabElement,
};
global.document = {
  activeElement: null,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) {
    if (selector === ".app") return appElement;
    return null;
  },
};
global.state = { view: "3d" };
global.beginLayoutTransition = () => 1;
global.noteLayoutResize = () => {};

const sidebarApi = new Function(block + `
return { sidebarState, applyEffectiveSidebar };`)();
const { sidebarState, applyEffectiveSidebar } = sidebarApi;

function expect(condition, message) { if (!condition) throw new Error(message); }
function resetFocus() {
  toggleElement.focused = false;
  systemTabElement.focused = false;
}

// Manual collapse in a navigation view focuses the visible toggle.
global.document.activeElement = panelElement;
panelElement.containsActive = true;
sidebarState.collapsed = true;
global.state.view = "3d";
resetFocus();
expect(applyEffectiveSidebar() === true, "manual collapse changes the effective state");
expect(toggleElement.focused === true, "navigation collapse focuses the visible toggle");
expect(systemTabElement.focused === false, "navigation collapse does not focus the system tab");

// Re-expanding hands no focus because the panel becomes interactive again.
sidebarState.collapsed = false;
expect(applyEffectiveSidebar() === true, "manual expand changes the effective state");

// Entering System focuses the system tab while the toggle is hidden.
global.state.view = "system";
resetFocus();
expect(applyEffectiveSidebar() === true, "system entry changes the effective state");
expect(systemTabElement.focused === true, "system entry focuses the system tab");
expect(toggleElement.focused === false, "system entry never focuses the hidden toggle");

// Leaving System restores the in-page expansion without stealing focus.
global.state.view = "3d";
resetFocus();
expect(applyEffectiveSidebar() === true, "leaving system changes the effective state");
expect(toggleElement.focused === false && systemTabElement.focused === false,
  "leaving system does not move focus");

// Focus outside the collapsing panel is left untouched in System mode.
global.document.activeElement = makeElement();
panelElement.containsActive = false;
global.state.view = "system";
resetFocus();
expect(applyEffectiveSidebar() === true, "a second system entry changes the effective state");
expect(systemTabElement.focused === false && toggleElement.focused === false,
  "focus outside the panel is never redirected");

console.log("sidebar focus target ok");
"""


def test_sidebar_system_entry_focuses_visible_tab_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _SIDEBAR_FOCUS_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "sidebar focus target ok" in completed.stdout


# --- Live latency and FPS indicators -----------------------------------------


def _html_div_block(html, opening):
    """Return one ``<div>...</div>`` block, matched by nesting depth."""
    start = html.index(opening)
    depth = 0
    index = start
    while index < len(html):
        open_at = html.find("<div", index)
        close_at = html.find("</div>", index)
        if close_at == -1:
            break
        if open_at != -1 and open_at < close_at:
            depth += 1
            index = open_at + len("<div")
        else:
            depth -= 1
            index = close_at + len("</div>")
            if depth == 0:
                return html[start:index]
    raise AssertionError(f"unterminated div starting at {opening!r}")


def test_sidebar_header_exposes_latency_and_fps_pills():
    index_html, _app_js, _styles_css = _frontend_sources()

    status_group = _html_div_block(index_html, '<div class="header-status">')

    # Compact status pills live in the existing upper-right status group.
    assert 'id="latencyStatus"' in status_group
    assert 'id="latencyValue"' in status_group
    assert 'id="fpsStatus"' in status_group
    assert 'id="fpsValue"' in status_group
    assert 'id="viewerCount"' in status_group
    assert 'id="serverState"' in status_group
    assert 'id="performanceStatus"' in status_group
    assert status_group.count('class="pill performance') == 1
    assert 'class="performance-separator"' in status_group

    # Unambiguous visible labels and a neutral placeholder.
    assert '<span class="performance-name">Latency</span>' not in status_group
    assert '<span class="performance-name">FPS</span>' in status_group
    assert '<span id="latencyValue" class="performance-value">--</span>' in status_group
    assert '<span id="fpsValue" class="performance-value">--</span>' in status_group

    # Accessible, non-chatty live regions with descriptive titles.
    assert 'role="status"' in status_group
    assert 'aria-live="off"' in status_group
    assert 'title="Browser-to-backend HTTP round-trip time"' in status_group
    assert 'title="WebGL map render frames per second"' in status_group


def test_frontend_measures_latency_from_existing_state_polls():
    _index_html, app_js, _styles_css = _frontend_sources()

    assert "function createLatencyMeter(options)" in app_js
    assert "async function timedGetJson(url)" in app_js
    assert "latencyMeter.record(startedAt, response.ok)" in app_js
    assert "latencyMeter.record(startedAt, false)" in app_js
    assert "const LATENCY_DISPLAY_INTERVAL_MS = 2000;" in app_js
    assert "displayIntervalMs: LATENCY_DISPLAY_INTERVAL_MS" in app_js

    # Reuse existing frequent lightweight polls instead of adding an endpoint.
    assert 'timedGetJson("/api/pose")' in app_js
    assert 'timedGetJson("/api/state")' in app_js
    assert 'timedGetJson("/api/logs"' not in app_js
    assert '"/api/latency"' not in app_js
    assert '"/api/ping"' not in app_js

    # Monotonic timing for the round trip.
    assert "performance.now()" in app_js

    poll_pose = _js_function_body(app_js, "async function pollPose()")
    assert 'timedGetJson("/api/pose")' in poll_pose
    poll_state = _js_function_body(app_js, "async function pollState()")
    assert 'timedGetJson("/api/state")' in poll_state


def test_frontend_fps_uses_webgl_render_loop_only():
    _index_html, app_js, _styles_css = _frontend_sources()

    assert "function createFpsMeter(options)" in app_js

    render_body = _js_function_body(app_js, "function render()")
    assert "fpsMeter.markFrame();" in render_body
    assert "requestAnimationFrame(render)" in render_body

    ensure_body = _js_function_body(app_js, "function ensureMapRenderLoop()")
    assert "fpsMeter.setActive(true);" in ensure_body
    pause_body = _js_function_body(app_js, "function pauseMapRenderLoop()")
    assert "fpsMeter.setActive(false);" in pause_body

    # The low-frequency refresh only repaints stale state; it never counts as a frame.
    assert "setInterval(refreshPerformanceIndicators, 500)" in app_js
    refresh_body = _js_function_body(app_js, "function refreshPerformanceIndicators()")
    assert "markFrame" not in refresh_body


_PERFORMANCE_METER_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("// --- Live performance meters");
const end = source.indexOf("// --- End live performance meters");
if (start < 0 || end < 0) throw new Error("performance meter block not found");
const block = source.slice(start, end);

const api = new Function(
  block + "\nreturn { createLatencyMeter, createFpsMeter };",
)();
const { createLatencyMeter, createFpsMeter } = api;

function expect(condition, message) { if (!condition) throw new Error(message); }

// --- Latency: EMA over measured round trips, with failure and staleness ---
let clock = 1000;
const latencyNow = () => clock;
const latencyEvents = [];
const latency = createLatencyMeter({
  now: latencyNow,
  staleMs: 3000,
  smoothing: 0.25,
  onChange: (text, available) => latencyEvents.push([text, available]),
});

expect(latency.text() === "--", "latency is unavailable before any sample");
expect(latency.available() === false, "latency is unavailable before any sample");

latency.record(960, true); // 40 ms
expect(latency.text() === "40 ms", "the first latency sample is shown");
expect(latency.available() === true, "a fresh latency sample is available");
expect(latencyEvents.length === 1, "the first latency sample is published immediately");

clock = 1100;
latency.record(1060, true); // another 40 ms keeps the average
expect(latency.text() === "40 ms", "steady samples keep the smoothed value");

clock = 1200;
latency.record(1140, true); // 60 -> 40 + 0.25 * 20 = 45
expect(latency.text() === "45 ms", "latency smooths toward recent samples");
expect(latencyEvents.length === 1, "live latency display is throttled within two seconds");

latency.record(1150, false);
expect(latency.text() === "--", "a failed request reports unavailable");
expect(latency.available() === false, "a failed request is not available");

clock = 1300;
latency.record(1250, true); // 50 -> 45 + 0.25 * 5 = 46.25 -> 46
expect(latency.text() === "46 ms", "latency recovers after a failure");
expect(latency.available() === true, "a recovered sample is available");

clock = 3300;
latency.record(3250, true);
expect(
  latencyEvents.at(-1)[0] === "47 ms" && latencyEvents.at(-1)[1] === true,
  "latency display publishes the latest smoothed value after two seconds",
);

clock = 6301; // more than staleMs since the last success
latency.refresh();
expect(latency.text() === "--", "a stale latency reports unavailable");
expect(latency.available() === false, "a stale latency is not available");

expect(
  latencyEvents.some(([text, available]) => text === "--" && available === false),
  "onChange publishes unavailable states",
);

// --- FPS: rendered frames over a window, smoothing, inactivity, staleness ---
clock = 0;
const fpsNow = () => clock;
const fps = createFpsMeter({
  now: fpsNow,
  windowMs: 1000,
  staleMs: 2000,
  smoothing: 0.4,
});

expect(fps.text() === "--", "fps is unavailable before the loop is active");
expect(fps.available() === false, "an inactive loop is unavailable");

fps.markFrame(); // t=0
clock = 250; fps.markFrame();
clock = 500; fps.markFrame();
clock = 1000; fps.markFrame();
expect(fps.text() === "4", "the first window counts 4 frames per second");
expect(fps.available() === true, "a completed window is available");

for (let t = 1100; t <= 2000; t += 100) {
  clock = t;
  fps.markFrame();
}
// 10 frames / 1000 ms, smoothed: 4 + 0.4 * (10 - 4) = 6.4 -> 6
expect(fps.text() === "6", "fps smooths toward the recent window");

fps.setActive(false);
expect(fps.text() === "--", "pausing the render loop hides fps");
expect(fps.available() === false, "a paused render loop is unavailable");

clock = 3000;
fps.markFrame();
expect(fps.text() === "--", "fps waits for a full window after reactivation");

clock = 4000;
fps.markFrame();
expect(fps.text() === "2", "a resumed window reports its own rate");
expect(fps.available() === true, "a resumed window is available");

clock = 6001; // more than staleMs since the last rendered frame
fps.refresh();
expect(fps.text() === "--", "a stale fps reports unavailable");
expect(fps.available() === false, "a stale fps is unavailable");

console.log("performance meters ok");
"""


def test_live_latency_and_fps_meters_execute_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _PERFORMANCE_METER_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "performance meters ok" in completed.stdout


_PERFORMANCE_DOM_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("// --- Live performance indicator UI");
const end = source.indexOf("// --- End live performance indicator UI");
if (start < 0 || end < 0) throw new Error("performance indicator UI block not found");
const block = source.slice(start, end);

function makeElement(value) {
  const element = {
    textContent: value,
    attributes: {},
    classes: new Set(),
  };
  element.setAttribute = (name, value) => { element.attributes[name] = value; };
  element.classList = {
    toggle(name, on) {
      if (on) element.classes.add(name);
      else element.classes.delete(name);
    },
  };
  return element;
}

const elements = {
  latencyStatus: makeElement(""),
  latencyValue: makeElement("--"),
  fpsStatus: makeElement(""),
  fpsValue: makeElement("--"),
};
global.document = { getElementById(id) { return elements[id] || null; } };

const api = new Function(
  block + "\nreturn { updateLatencyIndicator, updateFpsIndicator };",
)();
const { updateLatencyIndicator, updateFpsIndicator } = api;

function expect(condition, message) { if (!condition) throw new Error(message); }

updateLatencyIndicator("24 ms", true);
expect(elements.latencyValue.textContent === "24 ms", "latency value shows the sample");
expect(elements.latencyStatus.classes.has("is-idle") === false, "a live sample is not idle");
expect(
  elements.latencyStatus.attributes["aria-label"] === "Backend latency 24 ms",
  "the latency pill exposes an accessible label",
);

updateLatencyIndicator("--", false);
expect(elements.latencyValue.textContent === "--", "a stale latency shows --");
expect(elements.latencyStatus.classes.has("is-idle") === true, "a stale latency is idle");
expect(
  elements.latencyStatus.attributes["aria-label"] === "Backend latency unavailable",
  "a stale latency is announced as unavailable",
);

updateFpsIndicator("60", true);
expect(elements.fpsValue.textContent === "60", "fps value shows the rate");
expect(elements.fpsStatus.classes.has("is-idle") === false, "an active loop is not idle");
expect(
  elements.fpsStatus.attributes["aria-label"] === "Map frame rate 60",
  "the fps pill exposes an accessible label",
);

updateFpsIndicator("--", false);
expect(elements.fpsValue.textContent === "--", "an inactive loop shows --");
expect(elements.fpsStatus.classes.has("is-idle") === true, "an inactive loop is idle");
expect(
  elements.fpsStatus.attributes["aria-label"] === "Map frame rate unavailable",
  "an inactive loop is announced as unavailable",
);

console.log("performance indicator dom ok");
"""


def test_performance_indicator_dom_binding_executes_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _PERFORMANCE_DOM_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "performance indicator dom ok" in completed.stdout


_RENDER_LOOP_FPS_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("let mapRenderFrame = null;");
const end = source.indexOf("function canvasPoint(event)");
if (start < 0 || end < 0) throw new Error("map render loop block not found");
const block = source.slice(start, end);

const frames = new Map();
let nextFrame = 1;
let fpsFrames = 0;
let fpsActive = null;
const state = { view: "3d" };
const renderer = { render() {} };
const scene = {};
const camera = {};
const updateCamera = () => {};
const pageTransition = { active: false, expectsGrid: false };
const glassMotionReduced = () => false;
const resizeAll = () => {};
const requestAnimationFrame = (fn) => { const id = nextFrame++; frames.set(id, fn); return id; };
const cancelAnimationFrame = (id) => { frames.delete(id); };
const fpsMeter = {
  markFrame() { fpsFrames += 1; },
  setActive(value) { fpsActive = value; },
};

const api = new Function(
  "state", "renderer", "scene", "camera", "updateCamera",
  "requestAnimationFrame", "cancelAnimationFrame",
  "pageTransition", "glassMotionReduced", "resizeAll", "fpsMeter",
  block + "\nreturn { render, ensureMapRenderLoop, pauseMapRenderLoop };",
)(
  state, renderer, scene, camera, updateCamera, requestAnimationFrame,
  cancelAnimationFrame, pageTransition, glassMotionReduced, resizeAll, fpsMeter,
);
const { render, ensureMapRenderLoop, pauseMapRenderLoop } = api;

function expect(condition, message) { if (!condition) throw new Error(message); }
function flushFrame() {
  const pending = Array.from(frames.values());
  frames.clear();
  pending.forEach((fn) => fn());
}

state.view = "3d";
ensureMapRenderLoop();
expect(fpsActive === true, "starting the map loop activates fps");

fpsFrames = 0;
flushFrame();
expect(fpsFrames === 1, "each rendered map frame is counted");

state.view = "system";
pauseMapRenderLoop();
expect(fpsActive === false, "leaving the map view deactivates fps");
fpsFrames = 0;
render();
expect(fpsFrames === 0, "a draw outside the map view is never counted");

state.view = "mapping";
ensureMapRenderLoop();
expect(fpsActive === true, "re-entering a map view reactivates fps");

console.log("render loop fps ok");
"""


def test_map_render_loop_notches_fps_and_clears_when_inactive_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _RENDER_LOOP_FPS_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "render loop fps ok" in completed.stdout


def test_performance_indicator_styling_matches_glass_pills():
    _index_html, _app_js, styles_css = _frontend_sources()

    header_status = _css_rule_block(styles_css, ".header-status")
    assert "flex-wrap: nowrap;" in header_status
    assert "gap: 4px;" in header_status

    performance = _css_rule_block(styles_css, ".pill.performance")
    assert "white-space: nowrap;" in performance
    assert "font-variant-numeric: tabular-nums;" in performance

    assert ".pill.performance .performance-metric.is-idle" in styles_css
    assert ".pill.performance .performance-metric.latency" in styles_css
    assert ".pill.performance .performance-metric.fps" in styles_css

    media_start = styles_css.index("@media (max-width: 520px)")
    media_block = styles_css[media_start : styles_css.index("\n}", media_start)]
    assert ".header-status" in media_block


def _gated_stdout_process(gate, lines):
    class _GatedStdoutProcess:
        pid = 4321
        returncode = 0

        @property
        def stdout(self):
            for line in lines[:-1]:
                yield line
            gate.wait(timeout=5)
            yield lines[-1]

        def wait(self, timeout=None):
            return self.returncode

        def poll(self):
            return None

        def send_signal(self, _sig):
            pass

    return _GatedStdoutProcess()


def test_managed_stop_waits_for_reader_to_flush_trailing_lines(tmp_path, monkeypatch):
    from web_control.log_store import ManagedLogStore
    from web_control.web_control_node import ManagedLaunchProcess

    store = ManagedLogStore(tmp_path / "logs")
    logs = TerminalLogBuffer("chassis", max_lines=20)
    process = ManagedLaunchProcess(
        "chassis",
        ["ros2", "launch", "hunter_base", "x.py"],
        logs,
        _LoggerStub(),
        log_store=store,
    )
    with process.lock:
        process._begin_session_locked()
    session = process.session
    assert session is not None

    gate = threading.Event()
    fake = _gated_stdout_process(gate, ["leading line\n", "final trailing line\n"])
    process.process = fake
    process.pgid = 4321
    process.generation = 1
    done = threading.Event()
    process.reader_done = done
    reader = threading.Thread(
        target=process._read_output, args=(fake, 1, session, done), daemon=True
    )
    process.reader_thread = reader
    reader.start()

    monkeypatch.setattr(web_control_node.os, "killpg", lambda *_a, **_k: None)
    stopper = threading.Thread(target=process.stop)
    stopper.start()
    # Let ``stop`` reach its drain wait, then release the reader's final line.
    stopper.join(timeout=0.3)
    gate.set()
    stopper.join(timeout=5)

    assert not stopper.is_alive()
    content = session.path.read_text(encoding="utf-8")
    assert "final trailing line" in content
    assert session.closed is True


def test_stale_reader_without_session_never_writes_new_file(tmp_path):
    from web_control.log_store import ManagedLogStore
    from web_control.web_control_node import ManagedLaunchProcess

    store = ManagedLogStore(tmp_path / "logs")
    logs = TerminalLogBuffer("chassis", max_lines=20)
    process = ManagedLaunchProcess(
        "chassis",
        ["ros2", "launch", "hunter_base", "x.py"],
        logs,
        _LoggerStub(),
        log_store=store,
    )
    new_session = store.start_session("chassis", ["ros2", "launch", "new", "y.py"])
    logs.attach_session(new_session)
    new_session.write_line("new round line")

    class _OnceProcess:
        pid = 1
        returncode = 0

        @property
        def stdout(self):
            return iter(["stale old round line\n"])

        def wait(self, timeout=None):
            return 0

        def poll(self):
            return 0

    process.generation = 2
    done = threading.Event()
    # Explicit ``None`` means the previous round had no persistent session.
    process._read_output(_OnceProcess(), 1, None, done)

    assert done.is_set()
    content = new_session.path.read_text(encoding="utf-8")
    assert "stale old round line" not in content
    assert new_session.closed is False


def test_chassis_sequence_stop_records_inflight_preflight_output(
    monkeypatch, tmp_path
):
    from web_control.log_store import ManagedLogStore
    from web_control.web_control_node import (
        CHASSIS_CAN_PREFLIGHT_COMMANDS,
        ManagedSequenceLaunchProcess,
    )

    gate = threading.Event()
    first = tuple(CHASSIS_CAN_PREFLIGHT_COMMANDS[0])
    launched = []
    recorder = _PreflightSpawnRecorder(
        gate=gate, outputs={first: "CAN diagnostics line\n"}
    )
    monkeypatch.setattr(web_control_node.subprocess, "Popen", recorder)
    monkeypatch.setattr(
        web_control_node.ManagedLaunchProcess,
        "_start_locked",
        lambda self: launched.append(list(self.command)),
    )
    store = ManagedLogStore(tmp_path / "logs")
    logs = TerminalLogBuffer("chassis", max_lines=50)
    process = ManagedSequenceLaunchProcess(
        "chassis",
        [list(step) for step in CHASSIS_CAN_PREFLIGHT_COMMANDS],
        ["ros2", "launch", "hunter_base", "hunter_base.launch.py"],
        logs,
        _LoggerStub(),
        log_store=store,
    )

    monkeypatch.setattr(
        web_control_node.os, "killpg", lambda _pgid, _sig: gate.set()
    )
    monkeypatch.setattr(web_control_node.os, "getpgid", lambda _pid: 5000)

    process.start()
    assert recorder.first_spawn.wait(timeout=5)
    session = process.session
    process.stop()
    process.sequence_thread.join(timeout=5)

    assert session is not None
    assert "CAN diagnostics line" in session.path.read_text(encoding="utf-8")
    # The cancelled sequence never advanced to the second prerequisite.
    assert recorder.calls == [first]
    assert launched == []
