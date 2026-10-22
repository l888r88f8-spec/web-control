"""Runtime contracts for the browser-side data refresh scheduler.

The scheduler lives in static/app.js.  These tests execute that real block under
Node with a fake clock and controllable promises; they do not grep interval
literals.  They cover per-layer cadence, static-cache behaviour, visibility,
single-flight requests, retries, and stale-response rejection.
"""

import os
import shutil
import subprocess
from pathlib import Path

import pytest


_REFRESH_NODE_HARNESS = r"""
const fs = require("fs");
const assert = require("assert");

const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("// --- Data refresh scheduler");
const end = source.indexOf("// --- End data refresh scheduler");
if (start < 0 || end < 0 || end <= start) {
  throw new Error("data refresh scheduler block not found");
}
const block = source.slice(start, end);
const api = new Function(
  block + "\nreturn { REFRESH_RATES_MS, createRefreshController };",
)();

const NAV_NAMES = ["global_map", "local_map", "mapcloud", "mapground", "obstacles"];
const MAPPING_NAMES = ["global_map", "ground"];

assert.deepStrictEqual(api.REFRESH_RATES_MS, {
  navigationLayers: {
    global_map: 1000,
    local_map: 1000,
    mapcloud: 1000,
    mapground: 1000,
    obstacles: 200,
  },
  mappingLayers: 1000,
  paths: 200,
  mappingPath: 200,
  schedulerTick: 50,
});

function makeLayers(names) {
  const layers = {};
  for (const name of names) {
    layers[name] = { enabled: true, points3d: [], lastFetch3d: 0, staticLoaded: false };
  }
  return layers;
}

function makeHarness(options = {}) {
  const ctx = {
    now: 0,
    view: options.view || "3d",
    pathTopics: options.pathTopics || [],
    showPaths: options.showPaths !== false,
    hideCompleted: Boolean(options.hideCompleted),
    mappingPathVisible: options.mappingPathVisible !== false,
  };
  const navLayers = makeLayers(NAV_NAMES);
  const mappingLayers = makeLayers(MAPPING_NAMES);
  const requests = [];
  const applied = { navigation: [], mapping: [], paths: [], mappingPath: [] };
  let responder = options.responder || (() => ({ points: [[1, 2, 3]] }));

  const controller = api.createRefreshController({
    now: () => ctx.now,
    getJson: (url) => {
      const record = { url, at: ctx.now };
      requests.push(record);
      return responder(url, record, requests);
    },
    getView: () => ctx.view,
    navigationLayers: navLayers,
    mappingLayers,
    getPathTopics: () => ctx.pathTopics,
    pathsVisible: () => ctx.showPaths && !ctx.hideCompleted,
    mappingPathVisible: () => ctx.mappingPathVisible,
    applyNavigationLayer: (name) => applied.navigation.push(name),
    applyMappingLayer: (name) => applied.mapping.push(name),
    applyPath: (topic, points) => applied.paths.push({ topic, points }),
    applyMappingPath: (points) => applied.mappingPath.push(points),
  });

  function start(at = ctx.now) {
    ctx.now = at;
    return controller.tick(at);
  }
  async function tick(at = ctx.now) {
    await Promise.all(start(at));
  }
  function count(fragment) {
    return requests.filter((request) => request.url.includes(fragment)).length;
  }
  function layerCount(name) {
    return count(`/api/map3d?layer=${name}`);
  }
  function setResponder(next) {
    responder = next;
  }
  return {
    ctx, controller, navLayers, mappingLayers, requests, applied,
    start, tick, count, layerCount, setResponder,
  };
}

async function flush() {
  await Promise.resolve();
  await Promise.resolve();
}

(async () => {
  // Navigation: local_map 1 Hz, obstacles 5 Hz, static clouds only once after
  // their first nonempty load.  Mapping is idle while Navigation is visible.
  {
    const h = makeHarness();
    await h.tick(0);
    for (const name of NAV_NAMES) {
      assert.strictEqual(h.layerCount(name), 1, `${name} first request`);
    }
    for (let t = 50; t <= 5000; t += 50) await h.tick(t);
    assert.strictEqual(h.layerCount("local_map"), 6, "local_map runs at 1 Hz");
    assert.strictEqual(h.layerCount("obstacles"), 26, "obstacles run at 5 Hz");
    for (const name of ["global_map", "mapcloud", "mapground"]) {
      assert.strictEqual(h.layerCount(name), 1, `${name} cached after valid load`);
    }
    assert.strictEqual(h.count("/api/mapping/map3d"), 0, "mapping hidden from Navigation");
    assert.strictEqual(h.count("/api/mapping/path"), 0, "mapping path hidden from Navigation");
  }

  // Mapping clouds and trajectory keep their independent cadences and stop
  // entirely while another page is visible.  Returning to Mapping retains the
  // already fetched clouds and resumes at the configured rate.
  {
    const h = makeHarness({ view: "mapping", mappingPathVisible: true });
    for (let t = 0; t <= 3000; t += 1000) await h.tick(t);
    assert.strictEqual(h.count("/api/mapping/map3d?layer=global_map"), 4);
    assert.strictEqual(h.count("/api/mapping/map3d?layer=ground"), 4);
    assert.strictEqual(h.count("/api/mapping/path"), 4, "mapping trajectory runs at 5 Hz target");
    assert.strictEqual(h.count("/api/map3d"), 0, "navigation layers hidden on Mapping");

    h.ctx.mappingPathVisible = false;
    await h.tick(4000);
    assert.strictEqual(h.count("/api/mapping/path"), 4, "hidden mapping trajectory is gated");

    h.ctx.view = "3d";
    for (let t = 5000; t <= 7000; t += 1000) await h.tick(t);
    assert.strictEqual(h.count("/api/mapping/map3d"), 10, "mapping clouds stop when hidden");
    assert.strictEqual(h.count("/api/mapping/path"), 4, "mapping path stays hidden");

    h.ctx.view = "mapping";
    await h.tick(8000);
    assert.strictEqual(h.count("/api/mapping/map3d?layer=global_map"), 6);
    assert.strictEqual(h.count("/api/mapping/map3d?layer=ground"), 6);
  }

  // Navigation paths are scheduled independently at 5 Hz and honour both the
  // page and showPaths/hideCompletedPaths flags.  Mapping trajectory is 5 Hz.
  {
    const h = makeHarness({ pathTopics: ["/plan/a", "/plan/b"] });
    for (let t = 0; t <= 500; t += 100) await h.tick(t);
    assert.strictEqual(h.count("/api/path?topic=%2Fplan%2Fa"), 3);
    assert.strictEqual(h.count("/api/path?topic=%2Fplan%2Fb"), 3);

    h.ctx.hideCompleted = true;
    await h.tick(600);
    assert.strictEqual(h.count("/api/path?topic=%2Fplan%2Fa"), 3, "completed paths gated");
    h.ctx.hideCompleted = false;
    h.ctx.showPaths = false;
    await h.tick(800);
    assert.strictEqual(h.count("/api/path?topic=%2Fplan%2Fa"), 3, "hidden paths gated");
    h.ctx.showPaths = true;
    h.ctx.view = "mapping";
    await h.tick(1000);
    assert.strictEqual(h.count("/api/path?topic=%2Fplan%2Fa"), 3, "paths gated off Mapping");

    const m = makeHarness({ view: "mapping" });
    for (let t = 1000; t <= 1400; t += 200) await m.tick(t);
    assert.strictEqual(m.count("/api/mapping/path"), 3, "mapping path 5 Hz target");
    m.ctx.mappingPathVisible = false;
    await m.tick(1600);
    assert.strictEqual(m.count("/api/mapping/path"), 3);
  }

  // A slow request never allows a second request for the same resource, and
  // it cannot stall another layer's independent cadence.
  {
    let releaseGlobal;
    const h = makeHarness({
      responder: (url) => {
        if (url.includes("/api/map3d?layer=global_map")) {
          return new Promise((resolve) => { releaseGlobal = resolve; });
        }
        return { points: [[1, 2, 3]] };
      },
    });
    const pending = h.start(0);
    await flush();
    for (const t of [200, 400, 600, 800]) {
      h.start(t);
      await flush();
    }
    assert.strictEqual(h.layerCount("global_map"), 1, "one in-flight global_map");
    assert.strictEqual(h.layerCount("obstacles"), 5, "obstacles keep their own cadence");

    releaseGlobal({ points: [[4, 5, 6]] });
    await Promise.all(pending);
    await flush();
  }

  {
    let releaseObstacle;
    let obstacleCalls = 0;
    const h = makeHarness({
      responder: (url) => {
        if (url.includes("layer=obstacles")) {
          obstacleCalls += 1;
          if (obstacleCalls === 1) {
            return new Promise((resolve) => { releaseObstacle = resolve; });
          }
        }
        return { points: [[1, 2, 3]] };
      },
    });
    const pending = h.start(0);
    await flush();
    for (const t of [200, 400, 600]) {
      h.start(t);
      await flush();
    }
    assert.strictEqual(obstacleCalls, 1, "overlapping ticks do not queue obstacle work");
    releaseObstacle({ points: [[7, 8, 9]] });
    await Promise.all(pending);
    await flush();
    const next = h.start(800);
    assert.strictEqual(obstacleCalls, 2, "in-flight guard ends in finally");
    await Promise.all(next);
  }

  {
    let releaseSlowPath;
    let pathCalls = 0;
    const p = makeHarness({
      pathTopics: ["/slow"],
      responder: (url) => {
        if (url.includes("/api/path")) {
          pathCalls += 1;
          if (pathCalls === 1) return new Promise((resolve) => { releaseSlowPath = resolve; });
        }
        return { points: [[1, 2]] };
      },
    });
    const pathPending = p.start(0);
    await flush();
    p.start(200);
    await flush();
    p.start(400);
    await flush();
    assert.strictEqual(pathCalls, 1, "one in-flight path request");
    releaseSlowPath({ points: [[1, 2]] });
    await Promise.all(pathPending);
    const pathNext = p.start(600);
    assert.strictEqual(pathCalls, 2, "path retried after completion");
    await Promise.all(pathNext);
  }

  // Empty successes and errors never mark a static cloud loaded; it retries at
  // the waiting cadence and stops the moment a nonempty cloud arrives.
  {
    let calls = 0;
    const h = makeHarness({
      responder: (url) => {
        if (url.includes("/api/map3d?layer=global_map")) {
          calls += 1;
          if (calls === 1) return { points: [] };
          if (calls === 2) throw new Error("backend unavailable");
          return { points: [[7, 8, 9]] };
        }
        return { points: [[1, 2, 3]] };
      },
    });
    await h.tick(0);
    assert.strictEqual(calls, 1);
    assert.strictEqual(h.navLayers.global_map.staticLoaded, false, "empty is not cached");
    await h.tick(1000);
    assert.strictEqual(calls, 2);
    assert.strictEqual(h.navLayers.global_map.staticLoaded, false, "error is not cached");
    await h.tick(2000);
    assert.strictEqual(calls, 3);
    assert.deepStrictEqual(h.navLayers.global_map.points3d, [[7, 8, 9]]);
    assert.strictEqual(h.navLayers.global_map.staticLoaded, true);
    for (let t = 3000; t <= 7000; t += 1000) await h.tick(t);
    assert.strictEqual(calls, 3, "valid static cloud is never downloaded again");

    h.navLayers.global_map.enabled = false;
    await h.tick(8000);
    h.navLayers.global_map.enabled = true;
    await h.tick(9000);
    assert.strictEqual(calls, 3, "checkbox toggle retains the cached cloud");
    h.ctx.view = "mapping";
    await h.tick(10000);
    h.ctx.view = "3d";
    await h.tick(11000);
    assert.strictEqual(calls, 3, "tab switch retains the cached cloud");
  }

  // Invalidation clears the cache and increments a generation.  A delayed old
  // response cannot repopulate the layer after map/runtime invalidation; the
  // next request may then publish the valid new cloud.
  {
    let globalResolvers = [];
    let pathResolvers = [];
    const h = makeHarness({
      pathTopics: ["/old"],
      responder: (url) => {
        if (url.includes("/api/map3d?layer=global_map")) {
          return new Promise((resolve) => globalResolvers.push(resolve));
        }
        if (url.includes("/api/path")) {
          return new Promise((resolve) => pathResolvers.push(resolve));
        }
        return { points: [[1, 2, 3]] };
      },
    });
    const before = h.start(0);
    assert.strictEqual(globalResolvers.length, 1);
    assert.strictEqual(pathResolvers.length, 1);
    h.controller.invalidateNavigationLayers();
    h.controller.invalidatePaths();
    globalResolvers[0]({ points: [[9, 9, 9]] });
    pathResolvers[0]({ points: [[9, 9]] });
    await Promise.all(before);
    await flush();
    assert.deepStrictEqual(h.navLayers.global_map.points3d, [], "stale cloud rejected");
    assert.strictEqual(h.navLayers.global_map.staticLoaded, false);
    assert.strictEqual(h.applied.paths.length, 0, "stale path rejected");

    const after = h.start(0);
    assert.strictEqual(globalResolvers.length, 2, "new generation requests the new cloud");
    assert.strictEqual(pathResolvers.length, 2, "new generation requests the new path");
    globalResolvers[1]({ points: [[4, 5, 6]] });
    pathResolvers[1]({ points: [[4, 5]] });
    await Promise.all(after);
    await flush();
    assert.deepStrictEqual(h.navLayers.global_map.points3d, [[4, 5, 6]]);
    assert.strictEqual(h.navLayers.global_map.staticLoaded, true);
    assert.deepStrictEqual(h.applied.paths, [{ topic: "/old", points: [[4, 5]] }]);
  }

  // Explicit invalidation of every navigation layer is what a real map
  // selection does: all static caches wait for new nonempty data again.
  {
    const h = makeHarness();
    await h.tick(0);
    for (const name of ["global_map", "mapcloud", "mapground"]) {
      assert.strictEqual(h.navLayers[name].staticLoaded, true);
    }
    h.controller.invalidateNavigationLayers();
    for (const name of NAV_NAMES) assert.strictEqual(h.navLayers[name].staticLoaded, false);
    await h.tick(1000);
    for (const name of ["global_map", "mapcloud", "mapground"]) {
      assert.strictEqual(h.layerCount(name), 2, `${name} reloads after invalidation`);
    }
  }

  console.log("refresh scheduler ok");
})().catch((error) => {
  console.error(error && error.stack ? error.stack : error);
  process.exit(1);
});
"""


def test_refresh_scheduler_runtime_contracts_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _REFRESH_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "refresh scheduler ok" in completed.stdout


def test_update_status_stores_topics_without_polling_paths():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    start = app_js.index("function updateStatus(data)")
    end = app_js.index("function updateMappingStatus", start)
    block = app_js[start:end]

    assert "state.pathTopics = Object.keys(data.paths || {});" in block
    assert "pollPaths(" not in block
    assert "setInterval(() => refreshController.tick(), REFRESH_RATES_MS.schedulerTick)" in app_js
    assert "setInterval(pollPose, 100)" in app_js


def test_navigation_static_polling_has_no_legacy_shared_gate():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    assert "7000" not in app_js
    assert "setInterval(() => pollMaps3d(false), 5000)" not in app_js
    assert "pollPaths(topics)" not in app_js
    assert "pollMaps3d(true)" not in app_js


_APP_STARTUP_POLL_NODE_HARNESS = r"""
const fs = require("fs");
const assert = require("assert");

const source = fs.readFileSync(process.env.APP_JS, "utf8");
function extractFunction(signature) {
  const start = source.indexOf(signature);
  if (start < 0) throw new Error("missing function: " + signature);
  // Skip default parameter objects when locating the function body.
  const openParen = source.indexOf("(", start);
  let parenDepth = 0;
  let bodyStart = -1;
  for (let index = openParen; index < source.length; index += 1) {
    if (source[index] === "(") parenDepth += 1;
    else if (source[index] === ")") {
      parenDepth -= 1;
      if (parenDepth === 0) {
        bodyStart = source.indexOf("{", index);
        break;
      }
    }
  }
  if (bodyStart < 0) throw new Error("missing body: " + signature);
  const brace = bodyStart;
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

// These must stay the real declarations from app.js.  A deleted or renamed
// pollMappingState therefore fails this harness before start() is called.
const pollMappingStateSource = extractFunction("async function pollMappingState()");
const startSource = extractFunction("function start()");

const intervals = [];
const mappingUpdates = [];
const deps = {
  attachEvents() {},
  resizeAll() {},
  setTab(tab) { deps.tab = tab; },
  pollState() {},
  pollLogs() {},
  pollDiagnostics() {},
  heartbeatViewer() {},
  refreshController: {
    ticks: 0,
    tick() { this.ticks += 1; return Promise.resolve(); },
  },
  pollPose() { deps.poseCalls = (deps.poseCalls || 0) + 1; },
  refreshMaps() {},
  refreshJoystickPreferences() {},
  setSidebarCollapsed() {},
  setInterval(fn, delay) { intervals.push({ fn, delay }); return intervals.length; },
  REFRESH_RATES_MS: { schedulerTick: 50 },
  pollSystem() {},
  pollTf() {},
  refreshPerformanceIndicators() {},
  render() { deps.renderCalls = (deps.renderCalls || 0) + 1; },
  state: { view: "3d" },
  mappingPayload: null,
  requestedUrl: null,
  getJson(url) { deps.requestedUrl = url; return deps.getJsonImpl(url); },
  getJsonImpl: async () => deps.mappingPayload,
  updateMappingStatus(value) { mappingUpdates.push(["status", value]); },
  updateSaveMapStatus(value) { mappingUpdates.push(["save", value]); },
  updateRecordingStatus(value) { mappingUpdates.push(["recording", value]); },
};

const api = new Function("deps", [
  `const {
    attachEvents, resizeAll, setTab, pollState, pollLogs, pollDiagnostics,
    heartbeatViewer, refreshController, pollPose, refreshMaps,
    refreshJoystickPreferences, setSidebarCollapsed, setInterval,
    REFRESH_RATES_MS, pollSystem, pollTf, refreshPerformanceIndicators,
    render, state, getJson, updateMappingStatus, updateSaveMapStatus,
    updateRecordingStatus,
  } = deps;`,
  pollMappingStateSource,
  startSource,
  "return { start, pollMappingState };",
].join("\n"))(deps);

api.start();
assert.strictEqual(deps.tab, "3d", "start switches to the 3d tab");
assert.strictEqual(deps.renderCalls, 1, "start renders once");

const interval = (fn, delay) => intervals.find(
  (entry) => entry.fn === fn && entry.delay === delay,
);
assert.ok(interval(deps.pollPose, 100), "start installs the 100 ms pose timer");
assert.ok(
  interval(api.pollMappingState, 1000),
  "start installs the real 1 Hz mapping status timer",
);
const schedulerTimer = intervals.find((entry) => entry.delay === 50);
assert.ok(schedulerTimer, "start installs the 50 ms refresh scheduler timer");
assert.strictEqual(deps.refreshController.ticks, 1, "scheduler run once during start");
schedulerTimer.fn();
assert.strictEqual(deps.refreshController.ticks, 2, "scheduler timer calls controller.tick");

(async () => {
  deps.state.view = "mapping";
  deps.mappingPayload = {
    odom: { pose: { x: 1 } },
    save_map: { state: "ready" },
    recording: { state: "stopped" },
    layers: {},
    path: null,
  };
  await api.pollMappingState();
  assert.strictEqual(deps.requestedUrl, "/api/mapping/state");
  assert.deepStrictEqual(mappingUpdates, [
    ["status", deps.mappingPayload],
    ["save", deps.mappingPayload.save_map],
    ["recording", deps.mappingPayload.recording],
  ], "mapping status, save state, and recording state are all updated");

  mappingUpdates.length = 0;
  deps.getJsonImpl = async () => { throw new Error("mapping backend offline"); };
  await api.pollMappingState();
  assert.deepStrictEqual(mappingUpdates, [
    ["status", null],
    ["save", null],
    ["recording", null],
  ], "mapping poll error clears all three status sections");

  deps.requestedUrl = null;
  deps.state.view = "3d";
  await api.pollMappingState();
  assert.strictEqual(deps.requestedUrl, null, "mapping poll is gated to the mapping view");

  console.log("app startup and mapping poll ok");
})().catch((error) => {
  console.error(error && error.stack ? error.stack : error);
  process.exit(1);
});
"""


_DISPOSAL_NODE_HARNESS = r"""
const fs = require("fs");
const assert = require("assert");

const source = fs.readFileSync(process.env.APP_JS, "utf8");
function extractFunction(signature) {
  const start = source.indexOf(signature);
  if (start < 0) throw new Error("missing function: " + signature);
  // Skip default parameter objects when locating the function body.
  const openParen = source.indexOf("(", start);
  let parenDepth = 0;
  let bodyStart = -1;
  for (let index = openParen; index < source.length; index += 1) {
    if (source[index] === "(") parenDepth += 1;
    else if (source[index] === ")") {
      parenDepth -= 1;
      if (parenDepth === 0) {
        bodyStart = source.indexOf("{", index);
        break;
      }
    }
  }
  if (bodyStart < 0) throw new Error("missing body: " + signature);
  const brace = bodyStart;
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

class BufferGeometry {
  constructor() { this.attributes = {}; this.disposed = 0; }
  setAttribute(name, attribute) { this.attributes[name] = attribute; }
  dispose() { this.disposed += 1; }
}
class BufferAttribute {
  constructor(array, itemSize) { this.array = array; this.itemSize = itemSize; }
}
class PointsMaterial {
  constructor(options) { Object.assign(this, options); this.disposed = 0; }
  dispose() { this.disposed += 1; }
}
class Points {
  constructor(geometry, material) {
    this.geometry = geometry;
    this.material = material;
    this.visible = true;
  }
}
class Group {
  constructor() { this.children = []; }
  add(...objects) { this.children.push(...objects); }
  remove(object) {
    const index = this.children.indexOf(object);
    if (index >= 0) this.children.splice(index, 1);
  }
}
class CatmullRomCurve3 {
  constructor(points) { this.points = points; }
}
class TubeGeometry {
  constructor(curve, tubularSegments, radius, radialSegments, closed) {
    Object.assign(this, { curve, tubularSegments, radius, radialSegments, closed });
    this.disposed = 0;
  }
  dispose() { this.disposed += 1; }
}
class MeshBasicMaterial {
  constructor(options) { Object.assign(this, options); this.disposed = 0; }
  dispose() { this.disposed += 1; }
}
class Mesh {
  constructor(geometry, material) {
    this.geometry = geometry;
    this.material = material;
    this.name = "";
  }
}

const THREE = {
  BufferGeometry, BufferAttribute, PointsMaterial, Points, Group,
  CatmullRomCurve3, TubeGeometry, MeshBasicMaterial, Mesh,
};
const layers = {
  mapcloud: { enabled: true, color: 0xff0000, pointSize: 3, opacity: 0.5, points3d: [[1, 2, 3], [4, 5, 6]] },
};
const mappingLayers = {
  global_map: { enabled: true, color: 0x00ff00, pointSize: 4, opacity: 0.7, points3d: [[1, 2, 3]] },
};
const layerGroup = new THREE.Group();
const mappingLayerGroup = new THREE.Group();
const pathGroup = new THREE.Group();
const mappingPathGroup = new THREE.Group();
const state = {
  showPaths: true,
  hideCompletedPaths: false,
  paths: { "/plan": [[0, 0], [1, 1], [2, 2]] },
  showMappingPath: true,
  mappingPath: [[0, 0], [1, 1], [2, 2]],
};

const api = new Function(
  "THREE", "rosToThree", "pathPointToThree", "LAYERS", "MAPPING_LAYERS",
  "layerGroup", "mappingLayerGroup", "pathGroup", "mappingPathGroup", "state",
  [
    extractFunction("function disposeObjectResources"),
    extractFunction("function disposeGroupResources"),
    extractFunction("function makePointCloud"),
    extractFunction("function updateLayerObject"),
    extractFunction("function updateMappingLayerObject"),
    extractFunction("function updatePaths"),
    extractFunction("function updateMappingPath"),
    "return { updateLayerObject, updateMappingLayerObject, updatePaths, updateMappingPath };",
  ].join("\n"),
)(
  THREE,
  (x, y, z) => ({ x, y, z }),
  (point) => point,
  layers,
  mappingLayers,
  layerGroup,
  mappingLayerGroup,
  pathGroup,
  mappingPathGroup,
  state,
);

// Repeated cloud replacement must release the old GPU buffers and still apply
// the current visibility and point size to the replacement.
api.updateLayerObject("mapcloud");
const firstCloud = layers.mapcloud.object;
assert.strictEqual(firstCloud.visible, true);
assert.strictEqual(firstCloud.material.size, 3);

layers.mapcloud.points3d = [[7, 8, 9]];
layers.mapcloud.enabled = false;
layers.mapcloud.pointSize = 7;
api.updateLayerObject("mapcloud");
const secondCloud = layers.mapcloud.object;
assert.notStrictEqual(secondCloud, firstCloud);
assert.strictEqual(firstCloud.geometry.disposed, 1, "old cloud geometry disposed");
assert.strictEqual(firstCloud.material.disposed, 1, "old cloud material disposed");
assert.strictEqual(secondCloud.geometry.disposed, 0, "new cloud geometry retained");
assert.strictEqual(secondCloud.material.disposed, 0, "new cloud material retained");
assert.strictEqual(secondCloud.visible, false, "replacement keeps current visibility");
assert.strictEqual(secondCloud.material.size, 7, "replacement keeps current point size");
assert.strictEqual(layerGroup.children.length, 1, "one current cloud in the layer group");

api.updateMappingLayerObject("global_map");
const firstMappingCloud = mappingLayers.global_map.object;
mappingLayers.global_map.points3d = [[4, 5, 6]];
mappingLayers.global_map.enabled = false;
api.updateMappingLayerObject("global_map");
assert.strictEqual(firstMappingCloud.geometry.disposed, 1);
assert.strictEqual(firstMappingCloud.material.disposed, 1);
assert.strictEqual(mappingLayers.global_map.object.visible, false);
assert.strictEqual(mappingLayerGroup.children.length, 1);

// Repeated path replacement must dispose the old tube geometry and material,
// while the current visible path remains in the group.
api.updatePaths();
const firstPath = pathGroup.children[0];
assert.strictEqual(firstPath.name, "/plan");
state.paths["/plan"] = [[0, 0], [3, 3], [4, 4]];
api.updatePaths();
const secondPath = pathGroup.children[0];
assert.notStrictEqual(secondPath, firstPath);
assert.strictEqual(firstPath.geometry.disposed, 1, "old path geometry disposed");
assert.strictEqual(firstPath.material.disposed, 1, "old path material disposed");
assert.strictEqual(secondPath.geometry.disposed, 0);
assert.strictEqual(secondPath.material.disposed, 0);
assert.strictEqual(pathGroup.children.length, 1, "one current path remains");

api.updateMappingPath();
const firstMappingPath = mappingPathGroup.children[0];
state.mappingPath = [[0, 0], [5, 5], [6, 6]];
api.updateMappingPath();
const secondMappingPath = mappingPathGroup.children[0];
assert.notStrictEqual(secondMappingPath, firstMappingPath);
assert.strictEqual(firstMappingPath.geometry.disposed, 1);
assert.strictEqual(firstMappingPath.material.disposed, 1);
assert.strictEqual(secondMappingPath.geometry.disposed, 0);
assert.strictEqual(mappingPathGroup.children.length, 1);

// Hiding a path replaces it with nothing, but the removed resources still need
// to be released.
state.showPaths = false;
api.updatePaths();
assert.strictEqual(secondPath.geometry.disposed, 1);
assert.strictEqual(secondPath.material.disposed, 1);
assert.strictEqual(pathGroup.children.length, 0);

console.log("cloud and path disposal ok");
"""


def _run_app_node_harness(harness, marker):
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
    assert marker in completed.stdout


def test_app_start_defines_real_mapping_poll_and_installs_timers():
    _run_app_node_harness(
        _APP_STARTUP_POLL_NODE_HARNESS,
        "app startup and mapping poll ok",
    )


def test_cloud_and_path_replacement_disposes_owned_resources():
    _run_app_node_harness(
        _DISPOSAL_NODE_HARNESS,
        "cloud and path disposal ok",
    )
