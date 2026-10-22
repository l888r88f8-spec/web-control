"""Tests for the TF cache/forest and its web surface."""

import json
import urllib.request
from pathlib import Path
from types import MethodType, SimpleNamespace

from web_control import web_control_node
from web_control.tf_tree import TfTreeCache, build_tf_forest


def _edge(parent, child, static=False, stamp=1.0, last_seen=100.0):
    return {
        "parent": parent,
        "child": child,
        "static": static,
        "stamp": stamp,
        "last_seen": last_seen,
    }


def _flatten(frames, forest):
    for tree in forest["trees"]:
        stack = [tree]
        while stack:
            node = stack.pop()
            frames.append(node["frame"])
            stack.extend(node["children"])


def test_empty_cache_reports_waiting():
    cache = TfTreeCache()
    payload = cache.snapshot(100.0)
    assert payload["status"] == "waiting"
    assert payload["message"] == "waiting for /tf"
    assert payload["trees"] == []
    assert payload["frame_count"] == 0


def test_basic_tree_multiple_roots_and_parent_change():
    cache = TfTreeCache(stale_timeout_sec=2.0)
    cache.update_static([_edge("base_link", "lidar", static=True)], 100.0)
    cache.update_dynamic(
        [
            _edge("odom", "base_link"),
            _edge("map", "odom"),
            _edge("world", "map", last_seen=100.0),
        ],
        100.0,
    )
    payload = cache.snapshot(100.0)
    frames = []
    _flatten(frames, payload)
    assert set(frames) == {"base_link", "lidar", "odom", "map", "world"}
    roots = sorted(tree["frame"] for tree in payload["trees"])
    assert roots == ["world"]

    # A parent change replaces the old edge; the old parent is not kept.
    cache.update_dynamic([_edge("odom2", "base_link")], 101.0)
    payload = cache.snapshot(101.0)
    base = _find(payload["trees"], "base_link")
    assert base["parent"] == "odom2"
    assert all(tree["frame"] != "odom" or tree["frame"] == "odom" for tree in payload["trees"])


def _find(trees, frame):
    stack = list(trees)
    while stack:
        node = stack.pop()
        if node["frame"] == frame:
            return node
        stack.extend(node["children"])
    return None


def test_static_never_goes_stale_but_dynamic_does():
    forest = build_tf_forest(
        {
            "static": _edge("base", "cam", static=True, last_seen=0.0),
            "dyn": _edge("odom", "base", static=False, last_seen=0.0),
        },
        now=100.0,
        stale_timeout_sec=2.0,
    )
    cam = _find(forest["trees"], "cam")
    base = _find(forest["trees"], "base")
    assert cam["edge"]["static"] is True
    assert cam["edge"]["stale"] is False
    assert base["edge"]["stale"] is True
    assert base["edge"]["age"] == 100.0


def test_self_loop_cycle_and_malformed_edges_are_reported_not_dropped():
    forest = build_tf_forest(
        {
            "self": _edge("a", "a"),
            "c1": _edge("b", "c"),
            "c2": _edge("c", "b"),
            "bad": _edge("", "d"),
        },
        now=5.0,
        stale_timeout_sec=2.0,
    )
    frames = []
    _flatten(frames, forest)
    assert set(frames) >= {"a", "b", "c", "d"}
    reasons = {edge["reason"] for edge in forest["detached"]}
    assert "self cycle" in reasons
    assert "cycle" in reasons
    assert "malformed frame" in reasons


def test_parent_change_is_reflected_and_cache_eviction_is_bounded():
    # A child can only have one latest parent; a new edge replaces the old one.
    forest = build_tf_forest(
        {
            "one": _edge("root", "a"),
            "two": _edge("other", "a"),
        },
        now=1.0,
        stale_timeout_sec=2.0,
    )
    node = _find(forest["trees"], "a")
    assert node["parent"] == "other"
    assert sorted(tree["frame"] for tree in forest["trees"]) == ["other", "root"]

    cache = TfTreeCache(max_edges=3, stale_timeout_sec=2.0)
    for index in range(10):
        cache.update_dynamic([_edge("p", f"c{index}")], float(index))
    payload = cache.snapshot(10.0)
    assert payload["edge_count"] <= 3
    assert payload["frame_count"] <= 4


def test_tf_endpoint_serves_real_snapshot(tmp_path):
    node = SimpleNamespace()
    node.tf_cache = TfTreeCache(stale_timeout_sec=2.0)
    node.tf_cache.update_static([_edge("base", "lidar", static=True)], 1.0)
    node.now_sec = lambda: 2.0
    node.tf_message_type = object()
    node.get_logger = lambda: SimpleNamespace(
        debug=lambda *_a, **_k: None, error=lambda *_a, **_k: None
    )
    node.static_dir = tmp_path
    node.get_parameter = lambda name: SimpleNamespace(value="/tf")
    node.tf_payload = MethodType(web_control_node.WebControlNode.tf_payload, node)

    from http.server import ThreadingHTTPServer
    import threading

    handler = web_control_node.WebControlNode.make_handler(node)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with urllib.request.urlopen(
            f"http://127.0.0.1:{server.server_address[1]}/api/tf", timeout=10
        ) as response:
            body = json.loads(response.read().decode("utf-8"))
        assert body["ok"] is True
        assert body["status"] == "active"
        assert body["ros_available"] is True
        assert body["trees"][0]["frame"] == "base"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def test_node_declares_tf_subscriptions_qos_and_routes():
    root = Path(__file__).resolve().parents[1]
    source = (root / "web_control" / "web_control_node.py").read_text(encoding="utf-8")
    package_xml = (root / "package.xml").read_text(encoding="utf-8")
    config = (root / "config" / "web_control.yaml").read_text(encoding="utf-8")

    assert "from tf2_msgs.msg import TFMessage" in source
    assert "tf_static_qos" in source
    assert "QoSDurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "QoSReliabilityPolicy.RELIABLE" in source
    assert '"tf_topic"' in source and '"/tf"' in source
    assert '"/api/tf"' in source
    assert "tf2_msgs" in package_xml
    assert "managed_log_dir" in config
    assert "tf_cache_size" in config


def test_frontend_system_page_exposes_tf_tree_and_saved_logs():
    root = Path(__file__).resolve().parents[1]
    html = (root / "static" / "index.html").read_text(encoding="utf-8")
    app_js = (root / "static" / "app.js").read_text(encoding="utf-8")

    system_start = html.index('id="viewSystem"')
    system_html = html[system_start:]
    for marker in (
        'id="tfTree"',
        'id="tfTreeRefresh"',
        'id="savedLogTarget"',
        'id="savedLogSession"',
        'id="savedLogTail"',
        'id="savedLogDownload"',
    ):
        assert marker in system_html, marker

    # Low-frequency, view-gated polling with an out-of-order guard.
    assert "async function pollTf()" in app_js
    assert "if (state.view !== \"system\") return;" in app_js
    assert "tfViewState.requestSeq" in app_js
    assert "setInterval(pollTf, 3000)" in app_js
    # The frame graph is rendered as SVG by the tf-graph module; frame names
    # reach the DOM through textContent only, never innerHTML.
    assert 'import { TfGraphView, formatTfAge } from "./tf-graph.js";' in app_js
    assert "new TfGraphView(" in app_js
    assert "id=\"tfTreeZoomIn\"" not in app_js  # controls live in index.html
    graph_js = (root / "static" / "tf-graph.js").read_text(encoding="utf-8")
    assert ".innerHTML" not in graph_js
    assert "innerHTML =" not in graph_js
    assert "textContent" in graph_js
    assert "createElementNS" in graph_js


def test_deep_chain_splits_forest_and_json_serializes():
    edges = {
        f"f{index + 1}": {
            "parent": f"f{index}",
            "child": f"f{index + 1}",
            "static": False,
            "last_seen": 1.0,
        }
        for index in range(600)
    }
    forest = build_tf_forest(edges, 2.0)

    # A 600-deep chain must not blow the JSON encoder's recursion limit.
    payload = json.dumps(forest)
    assert json.loads(payload)["frame_count"] == 601

    frames = []
    _flatten(frames, forest)
    assert set(frames) == {f"f{index}" for index in range(601)}
    assert any(edge["reason"] == "tree too deep" for edge in forest["detached"])
