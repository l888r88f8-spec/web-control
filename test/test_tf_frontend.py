"""Node-based behaviour tests for the System TF frame graph.

The graph lives in ``static/tf-graph.js``. These tests drive the real module
with a tiny fake DOM so layout, XSS-safety and interaction stay honest without
a browser.
"""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

_PREAMBLE = r"""
import { pathToFileURL } from "node:url";
const mod = await import(pathToFileURL(process.env.TF_GRAPH_JS).href);

class FakeEl {
  constructor(tag, ns) {
    this.tag = tag;
    this.ns = ns || null;
    this.children = [];
    this.attributes = {};
    this.textContent = "";
    this.listeners = {};
    this.hidden = false;
    this.clientWidth = 0;
    this.clientHeight = 0;
    this._innerHTML = null;
  }
  set innerHTML(value) {
    this._innerHTML = value;
    throw new Error("innerHTML assignment is forbidden for TF frames");
  }
  get innerHTML() { return this._innerHTML; }
  setAttribute(key, value) { this.attributes[key] = String(value); }
  getAttribute(key) { return key in this.attributes ? this.attributes[key] : null; }
  removeAttribute(key) { delete this.attributes[key]; }
  hasAttribute(key) { return key in this.attributes; }
  appendChild(child) { this.children.push(child); return child; }
  replaceChildren(...nodes) { this.children = nodes; }
  addEventListener(type, fn) { (this.listeners[type] = this.listeners[type] || []).push(fn); }
  removeEventListener(type, fn) {
    this.listeners[type] = (this.listeners[type] || []).filter((entry) => entry !== fn);
  }
  dispatchEvent(event) { (this.listeners[event.type] || []).forEach((fn) => fn(event)); }
  querySelector() { return null; }
  closest() { return null; }
  focus() { this.focused = true; }
  getBoundingClientRect() { return { left: 0, top: 0, width: this.clientWidth, height: this.clientHeight }; }
  setPointerCapture() {}
  releasePointerCapture() {}
}

const fakeDocument = {
  createElementNS(ns, tag) { return new FakeEl(tag, ns); },
  createElement(tag) { return new FakeEl(tag); },
  getElementById() { return null; },
};

function expect(condition, message) { if (!condition) throw new Error(message); }

function collectText(node, out) {
  if (!node) return out;
  if (node.textContent) out.push(node.textContent);
  (node.children || []).forEach((child) => collectText(child, out));
  return out;
}

function findAll(node, predicate, out) {
  if (!node) return out;
  if (predicate(node)) out.push(node);
  (node.children || []).forEach((child) => findAll(child, predicate, out));
  return out;
}
"""

_SAMPLE = r"""
function samplePayload(extra = {}) {
  return {
    ok: true,
    status: "active",
    message: null,
    frame_count: 6,
    trees: [
      {
        frame: "map",
        parent: null,
        edge: null,
        children: [
          {
            frame: "odom",
            parent: "map",
            edge: { parent: "map", child: "odom", static: false, age: 0.4, stale: false },
            children: [
              {
                frame: "lidar_frame_with_a_very_long_name",
                parent: "odom",
                edge: { parent: "odom", child: "lidar_frame_with_a_very_long_name", static: true, age: 900.0, stale: false },
                children: [],
              },
            ],
          },
          {
            frame: "hitch",
            parent: "map",
            edge: { parent: "map", child: "hitch", static: false, age: 9.0, stale: true },
            children: [],
          },
        ],
      },
      { frame: "world", parent: null, edge: null, children: [] },
    ],
    detached: [{ parent: "a", child: "a", static: false, age: 0.2, stale: false, reason: "self cycle" }],
    ...extra,
  };
}
"""

_LAYOUT_HARNESS = _PREAMBLE + _SAMPLE + r"""
const layout = mod.layoutTfGraph(samplePayload());
expect(layout.roots.includes("map") && layout.roots.includes("world"), "both roots laid out");
expect(layout.nodeById.get("world").parent === null, "world is a root");
expect(layout.nodeById.get("lidar_frame_with_a_very_long_name").width <= 196, "long label width capped");
expect(layout.nodes.every((node) => node.width <= 196), "no node stretches the canvas");
expect(mod.truncateFrameLabel("f".repeat(80)).length <= mod.TF_GRAPH_MAX_LABEL, "label truncates");
expect(mod.truncateFrameLabel("short") === "short", "short label untouched");
expect(mod.formatTfAge(0.4) === "400 ms", "sub-second age");
expect(mod.formatTfAge(900) === "900.0 s", "second age");
expect(mod.formatTfAge(null) === "--", "missing age");
expect(layout.detached.some((edge) => edge.reason === "self cycle"), "cycle edge still visible");
expect(layout.detached[0].parent === "a" && layout.detached[0].child === "a", "detached endpoints kept");

// A 400-deep chain must not recurse and must keep every frame.
let deep = { frame: "f0", children: [] };
let cursor = deep;
for (let index = 1; index <= 400; index += 1) {
  const child = { frame: `f${index}`, edge: { parent: `f${index - 1}`, child: `f${index}`, static: false, age: 0.1, stale: false }, children: [] };
  cursor.children = [child];
  cursor = child;
}
const deepLayout = mod.layoutTfGraph({ ok: true, frame_count: 401, trees: [deep], detached: [] });
expect(deepLayout.nodes.length === 401, "deep chain fully laid out");
expect(deepLayout.maxDepth === 400, "deep chain depth");

// A malformed root with no frame name is skipped without throwing.
const malformed = mod.layoutTfGraph({ frame_count: 0, trees: [{ frame: "", children: [] }], detached: [] });
expect(malformed.nodes.length === 0, "empty frame skipped");

// Arrow endpoints: every edge path has a marker and starts at the parent.
const host = new FakeEl("div");
host.clientWidth = 900;
host.clientHeight = 420;
const scene = mod.buildTfGraphSvg(fakeDocument, layout, { width: 900, height: 420 });
expect(scene.nodes.length === layout.nodes.length, "scene nodes rendered");
expect(scene.edges.length === layout.edges.length, "scene edges rendered");
for (const edge of scene.edges) {
  expect((edge.element.getAttribute("marker-end") || "").startsWith("url(#tfArrow"), "edge arrow endpoint");
  expect((edge.element.getAttribute("d") || "").startsWith("M "), "edge path starts at parent");
}
const labels = collectText(scene.svg, []).join(" | ");
expect(labels.includes("lidar_frame_with_a_very_long_name"), "full frame name kept in title/aria");

// Long-name siblings must never overlap: the slot pitch clears the widest node.
const wide = mod.layoutTfGraph({
  trees: [{
    frame: "base_link",
    parent: null,
    edge: null,
    children: ["camera_rear_optical_frame_long", "camera_front_optical_frame_long"].map((frame) => ({
      frame,
      parent: "base_link",
      children: [],
      edge: { parent: "base_link", child: frame, static: false, age: 0.1, stale: false },
    })),
  }],
  detached: [],
});
const siblings = wide.nodes.filter((node) => node.parent === "base_link").sort((a, b) => a.x - b.x);
expect(siblings.length === 2, "both long siblings laid out");
const siblingGap = (siblings[1].x - siblings[1].width / 2) - (siblings[0].x + siblings[0].width / 2);
expect(siblingGap >= 8, `long-name sibling gap too small: ${siblingGap}`);
const byDepth = new Map();
for (const node of layout.nodes) {
  if (!byDepth.has(node.depth)) byDepth.set(node.depth, []);
  byDepth.get(node.depth).push(node);
}
for (const row of byDepth.values()) {
  row.sort((a, b) => a.x - b.x);
  for (let i = 1; i < row.length; i += 1) {
    const rowGap = (row[i].x - row[i].width / 2) - (row[i - 1].x + row[i - 1].width / 2);
    expect(rowGap >= 0, `same-depth overlap: ${rowGap}`);
  }
}
console.log("tf graph layout ok");
"""

_RENDER_HARNESS = _PREAMBLE + _SAMPLE + r"""
const host = new FakeEl("div");
host.clientWidth = 800;
host.clientHeight = 400;
const selected = [];
const view = new mod.TfGraphView(host, { document: fakeDocument, onSelect: (node) => selected.push(node) });

const hostile = samplePayload();
hostile.trees.push({
  frame: "<img src=x onerror=alert(1)>",
  parent: null,
  edge: null,
  children: [{ frame: '\"><script>alert(2)</script>', parent: "<img src=x onerror=alert(1)>", edge: { parent: "<img src=x onerror=alert(1)>", child: '\"><script>alert(2)</script>', static: false, age: 0.1, stale: false }, children: [] }],
});
view.update(hostile);

const texts = collectText(view.scene.svg, []);
expect(texts.includes("<img src=x onerror=alert(1)>"), "hostile frame id rendered as text");
expect(texts.some((text) => text.includes("<script>alert(2)</script>")), "hostile child rendered as text");
expect(view.scene.nodes.every((record) => record.element._innerHTML === null), "no innerHTML used");

const first = view.scene.nodes[0].id;
view.select(first);
expect(view.selectedId === first, "selection recorded");
expect(selected.length === 1 && selected[0].frame === first, "selection callback gets real node");
expect((view.scene.nodes[0].element.getAttribute("class") || "").includes("is-selected"), "selection painted");
console.log("tf graph render ok");
"""

_INTERACTION_HARNESS = _PREAMBLE + _SAMPLE + r"""
const host = new FakeEl("div");
host.clientWidth = 900;
host.clientHeight = 420;
const view = new mod.TfGraphView(host, { document: fakeDocument });
view.update(samplePayload());
expect(view.initialized === true && view.needsFit === false, "first data fits the view");

// A plain refresh (same topology, changing ages) keeps pan/zoom and selection.
const keepId = "odom";
view.select(keepId);
view.panBy(-40, 25);
const kept = { ...view.transform };
const refreshed = samplePayload();
refreshed.trees[0].children[0].edge.age = 0.9;
view.update(refreshed);
expect(Math.abs(view.transform.x - kept.x) < 0.001 && Math.abs(view.transform.k - kept.k) < 0.001, "refresh keeps viewport");
expect(view.selectedId === keepId, "refresh keeps selection");

// Pan / wheel / keyboard.
const beforePan = { ...view.transform };
view.handlePointerDown({ pointerId: 1, clientX: 100, clientY: 100 });
view.handlePointerMove({ pointerId: 1, clientX: 140, clientY: 130 });
view.handlePointerUp({ pointerId: 1 });
expect(view.transform.x !== beforePan.x, "drag pans the graph");
const beforeWheel = view.transform.k;
view.handleWheel({ deltaY: -120, offsetX: 400, offsetY: 200, preventDefault() {} });
expect(view.transform.k > beforeWheel, "wheel zooms in");
view.handleKeyDown({ key: "+", preventDefault() {} });
view.handleKeyDown({ key: "ArrowLeft", preventDefault() {} });
view.handleKeyDown({ key: "0", preventDefault() {} });
expect(view.needsFit === false, "keyboard fit settles");

// Hidden view ignores interaction.
view.setActive(false);
const hidden = view.transform.k;
view.handleWheel({ deltaY: -400, offsetX: 10, offsetY: 10, preventDefault() {} });
expect(view.transform.k === hidden, "inactive view ignores wheel");

// Empty -> data transition and 0-size resize.
view.setActive(true);
view.update({ ok: false, status: "unavailable", trees: [], detached: [] });
expect(view.layout.nodes.length === 0, "placeholder state has no nodes");
const placeholder = collectText(view.scene.svg, []).join(" ");
expect(placeholder.includes("unavailable"), "unavailable placeholder");
view.update(samplePayload());
expect(view.layout.nodes.length > 0, "data after empty re-renders");

const empty = new FakeEl("div");
const zeroView = new mod.TfGraphView(empty, { document: fakeDocument });
zeroView.update(samplePayload());
expect(zeroView.needsFit === true, "0-size host defers fit");
empty.clientWidth = 640;
empty.clientHeight = 360;
zeroView.handleResize();
expect(zeroView.needsFit === false, "resize from 0 fits reliably");
console.log("tf graph interaction ok");
"""

_LEAK_HARNESS = _PREAMBLE + _SAMPLE + r"""
const host = new FakeEl("div");
host.clientWidth = 800;
host.clientHeight = 400;
const view = new mod.TfGraphView(host, { document: fakeDocument });
view.update(samplePayload());
expect(view.sceneListeners.length > 0, "scene listeners attached");
const stableScene = view.sceneListeners.length;
const firstNode = view.scene.nodes[0].element;
expect((firstNode.listeners.keydown || []).length === 1, "node keyboard listener attached");
const hostWheel = (host.listeners.wheel || []).length;
for (let i = 0; i < 25; i += 1) {
  const payload = samplePayload();
  payload.trees[0].children[0].edge.age = 0.1 + i;
  view.update(payload);
}
expect(view.sceneListeners.length === stableScene, `scene listeners stable: ${view.sceneListeners.length} vs ${stableScene}`);
expect((firstNode.listeners.keydown || []).length === 0, "replaced node listeners released");
expect((host.listeners.wheel || []).length === hostWheel, "host listeners not duplicated");
const hostTotal = Object.values(host.listeners).reduce((sum, list) => sum + list.length, 0);
expect(hostTotal <= 8, `host listener count bounded: ${hostTotal}`);
view.destroy();
expect(view.hostListeners.length === 0 && view.sceneListeners.length === 0, "destroy clears registries");
const afterDestroy = Object.values(host.listeners).reduce((sum, list) => sum + list.length, 0);
expect(afterDestroy === 0, "destroy removes host listeners");
console.log("tf graph listener lifecycle ok");
"""

_POINTER_HARNESS = _PREAMBLE + _SAMPLE + r"""
const host = new FakeEl("div");
host.clientWidth = 900;
host.clientHeight = 420;
const view = new mod.TfGraphView(host, { document: fakeDocument });
view.update(samplePayload());
view.panBy(-30, -20);
const target = view.scene.nodes.find((record) => record.id === "hitch").element;

// A real pointerdown/up tap on a frame selects it (no synthetic click needed).
view.handlePointerDown({ pointerId: 1, button: 0, clientX: 120, clientY: 90, target });
view.handlePointerUp({ pointerId: 1, button: 0, clientX: 121, clientY: 91, target });
expect(view.selectedId === "hitch", "pointer tap selects the frame");

// A drag on the same frame pans but must not select.
const beforeDrag = { ...view.transform };
view.selectedId = null;
view.handlePointerDown({ pointerId: 2, button: 0, clientX: 120, clientY: 90, target });
view.handlePointerMove({ pointerId: 2, clientX: 180, clientY: 150, preventDefault() {} });
view.handlePointerUp({ pointerId: 2, button: 0, clientX: 180, clientY: 150, target });
expect(view.selectedId === null, "drag does not select");
expect(view.transform.x !== beforeDrag.x, "drag still pans");
view.handleClick({ target });
expect(view.selectedId === null, "post-drag click stays suppressed");

// Right button must not pan or select.
const beforeRight = { ...view.transform };
view.handlePointerDown({ pointerId: 3, button: 2, clientX: 120, clientY: 90, target });
view.handlePointerMove({ pointerId: 3, clientX: 300, clientY: 300, preventDefault() {} });
view.handlePointerUp({ pointerId: 3, button: 2, clientX: 300, clientY: 300, target });
expect(view.transform.x === beforeRight.x && view.transform.y === beforeRight.y, "right button does not pan");

// Delegated click and Enter/Space activation still select.
view.handleClick({ target });
expect(view.selectedId === "hitch", "delegated click selects");
view.selectedId = null;
target.dispatchEvent({ type: "keydown", key: "Enter", preventDefault() {} });
expect(view.selectedId === "hitch", "Enter selects the frame");

// Two contacts are a pinch, never a frame tap.
view.selectedId = null;
view.handlePointerDown({ pointerId: 5, button: 0, clientX: 100, clientY: 100, target });
view.handlePointerDown({ pointerId: 6, button: 0, clientX: 200, clientY: 100, target });
view.handlePointerUp({ pointerId: 5, button: 0, clientX: 100, clientY: 100, target });
view.handlePointerUp({ pointerId: 6, button: 0, clientX: 200, clientY: 100, target });
expect(view.selectedId === null, "two-finger gesture does not select");

// Leaving the view clears a half-finished gesture.
view.handlePointerDown({ pointerId: 4, button: 0, clientX: 10, clientY: 10, target });
expect(view.pointers.size === 1, "gesture pointer tracked");
view.setActive(false);
expect(view.pointers.size === 0 && view._gesture === null, "inactive view clears gesture state");
console.log("tf graph pointer ok");
"""

_COORDS_HARNESS = _PREAMBLE + _SAMPLE + r"""
const host = new FakeEl("div");
host.clientWidth = 800;
host.clientHeight = 400;
host.getBoundingClientRect = () => ({ left: 100, top: 50, width: 800, height: 400 });
const view = new mod.TfGraphView(host, { document: fakeDocument });
view.update(samplePayload());
const before = { ...view.transform };
view.handleWheel({ deltaY: -120, clientX: 250, clientY: 150, preventDefault() {} });
const after = view.transform;
expect(after.k > before.k, "wheel zooms in");
const localX = 250 - 100;
const localY = 150 - 50;
expect(
  Math.abs((localX - before.x) / before.k - (localX - after.x) / after.k) < 0.01,
  "wheel anchors horizontally on the host-local pointer",
);
expect(
  Math.abs((localY - before.y) / before.k - (localY - after.y) / after.k) < 0.01,
  "wheel anchors vertically on the host-local pointer",
);

// Keyboard pan is a stable screen-pixel step even at low zoom.
view.transform = { x: 0, y: 0, k: 0.05 };
view.handleKeyDown({ key: "ArrowLeft", preventDefault() {} });
expect(Math.abs(view.transform.x - 48) < 0.001, `keyboard pan screen step: ${view.transform.x}`);
view.transform = { x: 0, y: 0, k: 0.05 };
view.handleKeyDown({ key: "ArrowUp", preventDefault() {} });
expect(Math.abs(view.transform.y - 48) < 0.001, `keyboard pan screen step y: ${view.transform.y}`);
console.log("tf graph coordinates ok");
"""

_FIT_HARNESS = _PREAMBLE + _SAMPLE + r"""
function withinBounds(view) {
  const t = view.transform;
  const size = view.hostSize();
  return (
    t.x >= -0.001 &&
    t.y >= -0.001 &&
    t.x + view.layout.width * t.k <= size.width + 0.001 &&
    t.y + view.layout.height * t.k <= size.height + 0.001
  );
}
function deepPayload(depth) {
  const root = { frame: "f0", parent: null, edge: null, children: [] };
  let cursor = root;
  for (let i = 1; i <= depth; i += 1) {
    const child = {
      frame: `f${i}`,
      parent: `f${i - 1}`,
      edge: { parent: `f${i - 1}`, child: `f${i}`, static: false, age: 0.1, stale: false },
      children: [],
    };
    cursor.children = [child];
    cursor = child;
  }
  return { ok: true, frame_count: depth + 1, trees: [root], detached: [] };
}
function widePayload(count) {
  const children = [];
  for (let i = 0; i < count; i += 1) {
    const frame = `camera_${i}_optical_frame_long`;
    children.push({
      frame,
      parent: "base_link",
      edge: { parent: "base_link", child: frame, static: false, age: 0.1, stale: false },
      children: [],
    });
  }
  return { ok: true, frame_count: count + 1, trees: [{ frame: "base_link", parent: null, edge: null, children }], detached: [] };
}

const host = new FakeEl("div");
host.clientWidth = 390;
host.clientHeight = 420;
const view = new mod.TfGraphView(host, { document: fakeDocument });
view.update({ ok: false, status: "waiting", message: "waiting for TF", frame_count: 0, trees: [], detached: [] });
expect(view.initialized === false, "empty waiting payload does not finish initialization");
view.update(widePayload(256));
expect(view.initialized === true && view.needsFit === false, "first real graph initializes and fits");
expect(withinBounds(view), "wide graph fully framed");
const fitWide = view.transform.k;
view.update(widePayload(256));
expect(Math.abs(view.transform.k - fitWide) < 0.001, "same-topology refresh keeps the user transform");

const deepView = new mod.TfGraphView(host, { document: fakeDocument });
deepView.update(deepPayload(128));
expect(withinBounds(deepView), `deep graph fully framed: k=${deepView.transform.k}`);
const fitDeep = deepView.transform.k;
expect(fitDeep > 0 && fitDeep < 0.12, "deep graph fits below the old 0.12 floor");
deepView.zoomBy(1.2);
expect(Math.abs(deepView.transform.k - fitDeep * 1.2) < 0.0001, "zoom is smooth from the deep fit scale");
console.log("tf graph fit ok");
"""

_SELECTION_HARNESS = _PREAMBLE + _SAMPLE + r"""
const host = new FakeEl("div");
host.clientWidth = 800;
host.clientHeight = 400;
const calls = [];
const view = new mod.TfGraphView(host, { document: fakeDocument, onSelect: (node) => calls.push(node) });
view.update(samplePayload());
view.select("odom");
expect(calls.length === 1 && calls[0].frame === "odom", "select notifies once");
const refreshed = samplePayload();
refreshed.trees[0].children[0].edge.age = 3.5;
view.update(refreshed);
const fresh = view.layout.nodeById.get("odom");
expect(calls.length === 2, "update re-notifies the selected frame");
expect(calls[1] === fresh && calls[1].edge.age === 3.5, "selection details show the new age");

// Moving the selected frame under a new parent refreshes parent details too.
const moved = samplePayload();
const odomTree = moved.trees[0].children.shift();
moved.trees[1].children = [odomTree];
view.update(moved);
expect(calls[calls.length - 1].parent === "world", "selection details show the new parent");

// Focus on the same frame survives the 3s DOM rebuild.
const focused = view.scene.nodes.find((record) => record.id === "odom").element;
fakeDocument.activeElement = focused;
view.update(samplePayload());
const refocused = view.scene.nodes.find((record) => record.id === "odom").element;
expect(refocused !== focused, "DOM node was replaced");
expect(refocused.focused === true, "focus follows the same frame across rebuilds");
fakeDocument.activeElement = host;
view.update(samplePayload());
expect(host.focused !== true, "host focus is never stolen");
console.log("tf graph selection ok");
"""


def _run_node_harness(harness):
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    graph_js = Path(__file__).resolve().parents[1] / "static" / "tf-graph.js"
    return subprocess.run(
        ["node", "--input-type=module", "-e", harness],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "TF_GRAPH_JS": str(graph_js)},
    )


def test_tf_graph_layout_multiple_roots_long_names_deep_chain_and_cycles():
    completed = _run_node_harness(_LAYOUT_HARNESS)
    assert completed.returncode == 0, completed.stderr
    assert "tf graph layout ok" in completed.stdout


def test_tf_graph_render_escapes_hostile_frame_ids():
    completed = _run_node_harness(_RENDER_HARNESS)
    assert completed.returncode == 0, completed.stderr
    assert "tf graph render ok" in completed.stdout


def test_tf_graph_interaction_preserves_viewport_and_handles_lifecycle():
    completed = _run_node_harness(_INTERACTION_HARNESS)
    assert completed.returncode == 0, completed.stderr
    assert "tf graph interaction ok" in completed.stdout


def test_tf_graph_render_releases_scene_listeners_and_destroy_clears_all():
    completed = _run_node_harness(_LEAK_HARNESS)
    assert completed.returncode == 0, completed.stderr
    assert "tf graph listener lifecycle ok" in completed.stdout


def test_tf_graph_pointer_sequence_selects_taps_and_ignores_drags_and_right_button():
    completed = _run_node_harness(_POINTER_HARNESS)
    assert completed.returncode == 0, completed.stderr
    assert "tf graph pointer ok" in completed.stdout


def test_tf_graph_pointer_coordinates_are_host_local_and_keyboard_pan_is_screen_pixels():
    completed = _run_node_harness(_COORDS_HARNESS)
    assert completed.returncode == 0, completed.stderr
    assert "tf graph coordinates ok" in completed.stdout


def test_tf_graph_fit_handles_empty_then_large_and_deep_wide_graphs():
    completed = _run_node_harness(_FIT_HARNESS)
    assert completed.returncode == 0, completed.stderr
    assert "tf graph fit ok" in completed.stdout


def test_tf_graph_refreshes_selected_details_and_keeps_frame_focus():
    completed = _run_node_harness(_SELECTION_HARNESS)
    assert completed.returncode == 0, completed.stderr
    assert "tf graph selection ok" in completed.stdout
