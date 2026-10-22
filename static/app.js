import * as THREE from "./vendor/three.module.js";
import { initLiquidGlass, glassMotionReduced, glassMode, onGlassModeChange, setGlassMode } from "./liquid-glass.js";
import { initTrackRefraction } from "./track-refraction.js";
import { createSegmentedControl } from "./segmented-control.js";
import { TfGraphView, formatTfAge } from "./tf-graph.js";

// --- Glass strength preference
// Glass Strength is an explicit, browser-local appearance choice. It lives in
// localStorage rather than the server-side UI preferences because it selects a
// per-device rendering material, not a layout shared by every browser. Full is
// the default when nothing has been saved.
const GLASS_STRENGTH_KEY = "web_control_glass_strength";
const GLASS_STRENGTH_VALUES = ["full", "light", "off"];

function readStoredGlassStrength() {
  try {
    const value = window.localStorage.getItem(GLASS_STRENGTH_KEY);
    return GLASS_STRENGTH_VALUES.includes(value) ? value : null;
  } catch (_error) {
    return null;
  }
}

function storeGlassStrength(mode) {
  if (!GLASS_STRENGTH_VALUES.includes(mode)) return;
  try {
    window.localStorage.setItem(GLASS_STRENGTH_KEY, mode);
  } catch (_error) {
    // Storage can be unavailable; the current page still applies the choice.
  }
}

function updateGlassStrengthControl(mode) {
  const select = document.getElementById("glassStrength");
  if (select && select.value !== mode) select.value = mode;
}

function applyGlassStrength(mode) {
  const applied = setGlassMode(mode);
  storeGlassStrength(applied);
  updateGlassStrengthControl(applied);
  return applied;
}

function attachGlassStrengthControl() {
  const select = document.getElementById("glassStrength");
  if (!select) return;
  updateGlassStrengthControl(glassMode());
  select.addEventListener("change", (event) => applyGlassStrength(event.target.value));
}
// --- End glass strength preference

const LAYERS = {
  global_map: { color: 0x6f8fb0, points3d: [], enabled: true, object: null, lastFetch3d: 0, staticLoaded: false, opacity: 0.72 },
  local_map: { color: 0x8ea4b8, points3d: [], enabled: true, object: null, lastFetch3d: 0 },
  mapcloud: { color: 0x49b59f, points3d: [], enabled: true, object: null, lastFetch3d: 0, staticLoaded: false },
  mapground: { color: 0xd9a441, points3d: [], enabled: true, object: null, lastFetch3d: 0, staticLoaded: false },
  // The live navigation obstacle aggregate is a flat, deliberately larger red
  // overlay.  No height/intensity attribute is consumed, so its colour cannot
  // be tinted away by the map cloud pipeline.
  obstacles: { color: 0xff0000, points3d: [], enabled: true, object: null, lastFetch3d: 0, pointSize: 5, opacity: 1.0, flatColor: true },
};

const MAPPING_LAYERS = {
  global_map: { color: 0x8ea4b8, points3d: [], enabled: true, object: null, lastFetch3d: 0 },
  ground: { color: 0xd9a441, points3d: [], enabled: true, object: null, lastFetch3d: 0 },
};

const TARGET_MARKER_COLOR = 0x0a84ff;

const state = {
  mode: "view",
  view: "3d",
  selection: null,
  activeGoal: null,
  lastGoal: null,
  odom: null,
  navigationAction: null,
  navigationControl: null,
  paths: {},
  pathTopics: [],
  mappingOdom: null,
  mappingPath: [],
  mapSceneCleared: false,
  mapCatalog: null,
  mapError: null,
  mapSelectionError: null,
  mapSelectionNotice: null,
  mapSelectPending: false,
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
  saveMapState: null,
  recording: null,
  recordingTopics: [],
  recordingTopicsError: null,
  recordingBrowserPath: "",
  recordingBrowserEntries: [],
  recordingStartError: null,
  recordingActionError: null,
  recordingAutoName: null,
  recordingUsedName: null,
  recordingPending: false,
  hideCompletedPaths: false,
  showPaths: true,
  showVehicle: true,
  showMappingPath: true,
  showMappingVehicle: true,
  camera: { yaw: -0.72, pitch: 0.82, distance: 58, target: new THREE.Vector3(0, 0, 0) },
  pointer: { active: false, button: 0, start: null, previous: null, selecting: false, touches: new Map(), touchStart: null },
};

let saveMapSubmissionError = null;

initLiquidGlass({ savedMode: readStoredGlassStrength() });
initTrackRefraction();
onGlassModeChange(updateGlassStrengthControl);
updateGlassStrengthControl(glassMode());

const mapCanvas3d = document.getElementById("mapCanvas3d");
const hud = document.getElementById("hud");
const VIEWER_SESSION_KEY = "web_control_viewer_id";

const SCENE_BG = 0x0b0f19;
const scene = new THREE.Scene();
scene.background = new THREE.Color(SCENE_BG);
const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 5000);
const renderer = new THREE.WebGLRenderer({ canvas: mapCanvas3d, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setClearColor(0x0b0f19, 1);
const raycaster = new THREE.Raycaster();
raycaster.params.Points.threshold = 0.35;
const pointerNdc = new THREE.Vector2();
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);

const layerGroup = new THREE.Group();
const pathGroup = new THREE.Group();
const markerGroup = new THREE.Group();
const vehicleGroup = new THREE.Group();
const mappingLayerGroup = new THREE.Group();
const mappingPathGroup = new THREE.Group();
const mappingVehicleGroup = new THREE.Group();
scene.add(layerGroup, pathGroup, markerGroup, vehicleGroup, mappingLayerGroup, mappingPathGroup, mappingVehicleGroup);

scene.add(new THREE.GridHelper(120, 60, 0x8aa0b4, 0x334155));
const axes = new THREE.AxesHelper(4);
axes.position.set(0, 0.04, 0);
scene.add(axes);
scene.add(new THREE.HemisphereLight(0xffffff, 0x223044, 1.15));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(20, 38, 12);
scene.add(sun);

function rosToThree(x, y, z = 0) {
  return new THREE.Vector3(x, z, -y);
}

function threeToRos(point) {
  return { x: point.x, y: -point.z, z: point.y };
}

function fmt(value, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

function fmtPercent(value) {
  return Number.isFinite(value) ? `${value.toFixed(1)}%` : "--";
}

function fmtStorage(kb) {
  if (!Number.isFinite(kb)) return "--";
  const gib = kb / 1024 / 1024;
  if (gib >= 1) return `${gib.toFixed(1)} GiB`;
  return `${(kb / 1024).toFixed(0)} MiB`;
}

function usageLevel(value) {
  if (!Number.isFinite(value)) return "waiting";
  if (value >= 85) return "high";
  if (value >= 65) return "medium";
  return "low";
}

const FETCH_TIMEOUT_MS = 15000;

async function fetchWithTimeout(url, options) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), FETCH_TIMEOUT_MS);
  try {
    return await fetch(url, { ...options, signal: controller.signal });
  } catch (error) {
    if (error && error.name === "AbortError") throw new Error("Request timed out.");
    throw error;
  } finally {
    clearTimeout(timer);
  }
}

async function getJson(url) {
  const response = await fetchWithTimeout(url, { cache: "no-store" });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  return response.json();
}

async function getText(url) {
  const response = await fetchWithTimeout(url, { cache: "no-store" });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  return response.text();
}

async function postJson(url, payload) {
  const response = await fetchWithTimeout(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await response.json();
  if (!response.ok || !data.ok) throw new Error(data.message || response.statusText);
  return data;
}

// --- Live performance meters
// Latency and FPS come from signals the page already produces: the round-trip
// time of lightweight state polls (browser <-> web backend) and the WebGL map
// render loop's animation frames. Both meters smooth recent samples and fall
// back to "--" while unavailable or stale. The factories take an injected
// monotonic clock so their math and lifecycle stay testable.
const LATENCY_SMOOTHING = 0.25;
const LATENCY_STALE_MS = 3000;
const LATENCY_DISPLAY_INTERVAL_MS = 2000;
const FPS_WINDOW_MS = 750;
const FPS_SMOOTHING = 0.4;
const FPS_STALE_MS = 1500;

function createLatencyMeter(options) {
  const config = options || {};
  const now = config.now;
  const staleMs = Number.isFinite(config.staleMs) ? config.staleMs : LATENCY_STALE_MS;
  const smoothing = Number.isFinite(config.smoothing) ? config.smoothing : LATENCY_SMOOTHING;
  const displayIntervalMs = Number.isFinite(config.displayIntervalMs)
    ? config.displayIntervalMs
    : LATENCY_DISPLAY_INTERVAL_MS;
  const onChange = typeof config.onChange === "function" ? config.onChange : function () {};
  let smoothed = null;
  let lastSuccess = null;
  let lastPublishedAt = null;
  let lastPublishedAvailable = false;

  function state() {
    if (smoothed === null || lastSuccess === null || now() - lastSuccess > staleMs) {
      return { text: "--", available: false };
    }
    return { text: `${Math.round(smoothed)} ms`, available: true };
  }

  function publish() {
    const next = state();
    const timestamp = now();
    const availabilityChanged = next.available !== lastPublishedAvailable;
    const intervalElapsed = lastPublishedAt === null || timestamp - lastPublishedAt >= displayIntervalMs;
    if (availabilityChanged || intervalElapsed) {
      onChange(next.text, next.available);
      lastPublishedAt = timestamp;
      lastPublishedAvailable = next.available;
    }
    return next;
  }

  function record(startedAt, ok) {
    if (!ok) {
      lastSuccess = null;
      return publish();
    }
    const elapsed = now() - startedAt;
    if (!Number.isFinite(elapsed) || elapsed < 0) {
      lastSuccess = null;
      return publish();
    }
    smoothed = smoothed === null ? elapsed : smoothed + smoothing * (elapsed - smoothed);
    lastSuccess = now();
    return publish();
  }

  return {
    record,
    refresh: publish,
    text: () => state().text,
    available: () => state().available,
  };
}

function createFpsMeter(options) {
  const config = options || {};
  const now = config.now;
  const windowMs = Number.isFinite(config.windowMs) ? config.windowMs : FPS_WINDOW_MS;
  const staleMs = Number.isFinite(config.staleMs) ? config.staleMs : FPS_STALE_MS;
  const smoothing = Number.isFinite(config.smoothing) ? config.smoothing : FPS_SMOOTHING;
  const onChange = typeof config.onChange === "function" ? config.onChange : function () {};
  let active = false;
  let windowStart = null;
  let windowFrames = 0;
  let smoothed = null;
  let lastFrame = null;

  function state() {
    if (!active || smoothed === null || lastFrame === null || now() - lastFrame > staleMs) {
      return { text: "--", available: false };
    }
    return { text: `${Math.round(smoothed)}`, available: true };
  }

  function publish() {
    const next = state();
    onChange(next.text, next.available);
    return next;
  }

  function setActive(isActive) {
    active = Boolean(isActive);
    if (!active) {
      windowStart = null;
      windowFrames = 0;
      smoothed = null;
      lastFrame = null;
    }
    return publish();
  }

  function markFrame() {
    const timestamp = now();
    active = true;
    if (windowStart === null) {
      windowStart = timestamp;
      windowFrames = 0;
    }
    windowFrames += 1;
    lastFrame = timestamp;
    const elapsed = timestamp - windowStart;
    if (elapsed >= windowMs) {
      const fps = (windowFrames / elapsed) * 1000;
      smoothed = smoothed === null ? fps : smoothed + smoothing * (fps - smoothed);
      windowStart = timestamp;
      windowFrames = 0;
    }
    return publish();
  }

  return {
    markFrame,
    setActive,
    refresh: publish,
    text: () => state().text,
    available: () => state().available,
  };
}
// --- End live performance meters

// --- Live performance indicator UI
function updatePerformancePill(statusId, valueId, text, available, label) {
  const element = document.getElementById(statusId);
  const value = document.getElementById(valueId);
  if (!element || !value) return;
  const live = Boolean(available) && text !== "--";
  value.textContent = live ? text : "--";
  element.classList.toggle("is-idle", !live);
  element.setAttribute("aria-label", live ? `${label} ${text}` : `${label} unavailable`);
}

function updateLatencyIndicator(text, available) {
  updatePerformancePill("latencyStatus", "latencyValue", text, available, "Backend latency");
}

function updateFpsIndicator(text, available) {
  updatePerformancePill("fpsStatus", "fpsValue", text, available, "Map frame rate");
}
// --- End live performance indicator UI

const latencyMeter = createLatencyMeter({
  now: () => performance.now(),
  displayIntervalMs: LATENCY_DISPLAY_INTERVAL_MS,
  onChange: updateLatencyIndicator,
});
const fpsMeter = createFpsMeter({
  now: () => performance.now(),
  onChange: updateFpsIndicator,
});

// Time an existing frequent lightweight poll so the latency pill reflects the
// real browser -> web backend HTTP round trip (request send through response
// headers). HTTP and network failures are recorded so the pill falls back to
// "--" instead of showing a stale value.
async function timedGetJson(url) {
  const startedAt = performance.now();
  let sampled = false;
  try {
    const response = await fetchWithTimeout(url, { cache: "no-store" });
    sampled = true;
    latencyMeter.record(startedAt, response.ok);
    if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
    return await response.json();
  } catch (error) {
    // Network/abort failures never produced a response, so sample them here.
    if (!sampled) latencyMeter.record(startedAt, false);
    throw error;
  }
}

function refreshPerformanceIndicators() {
  latencyMeter.refresh();
  fpsMeter.refresh();
}

function viewerSessionId() {
  let viewerId = sessionStorage.getItem(VIEWER_SESSION_KEY);
  if (viewerId) return viewerId;
  if (window.crypto && window.crypto.randomUUID) {
    viewerId = window.crypto.randomUUID();
  } else {
    viewerId = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
  }
  sessionStorage.setItem(VIEWER_SESSION_KEY, viewerId);
  return viewerId;
}

function updateViewerCount(count) {
  const element = document.getElementById("viewerCount");
  if (!element) return;
  const viewers = Number.isFinite(count) ? Math.max(0, Math.round(count)) : 0;
  element.textContent = `${viewers} watching`;
}

async function heartbeatViewer() {
  try {
    const data = await postJson("/api/viewer", { viewer_id: viewerSessionId() });
    updateViewerCount(data.viewers);
  } catch (_error) {
    updateViewerCount(0);
  }
}

let lastRenderWidth = 0;
let lastRenderHeight = 0;

function resizeAll() {
  const rect = mapCanvas3d.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width));
  const height = Math.max(1, Math.round(rect.height));
  if (width === lastRenderWidth && height === lastRenderHeight) {
    return false;
  }
  lastRenderWidth = width;
  lastRenderHeight = height;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  return true;
}

function updateCamera() {
  const { yaw, pitch, distance, target } = state.camera;
  const cp = Math.cos(pitch);
  camera.position.set(
    target.x + Math.cos(yaw) * cp * distance,
    target.y + Math.sin(pitch) * distance,
    target.z + Math.sin(yaw) * cp * distance,
  );
  camera.lookAt(target);
}

let mapRenderFrame = null;

function mapViewActive() {
  return state.view === "3d" || state.view === "mapping";
}

// During the 650ms grid slide the canvas CSS box changes every frame while
// noteLayoutResize deliberately defers observer/window notifications. Re-sync
// the cached drawing buffer so the old buffer is never stretched, then let
// render() update the camera and draw in the same animation frame before paint.
// Gated to a live grid-expecting transition in a visible map view; joystick and
// HUD stay settle-only, and reduced motion keeps the next-frame settle path.
// resizeAll() remains the only renderer.setSize call site.
function syncMapViewportDuringGridTransition() {
  if (!pageTransition.active || !pageTransition.expectsGrid) return;
  if (!mapViewActive() || glassMotionReduced()) return;
  resizeAll();
}

function render() {
  if (mapRenderFrame !== null) {
    cancelAnimationFrame(mapRenderFrame);
    mapRenderFrame = null;
  }
  syncMapViewportDuringGridTransition();
  updateCamera();
  renderer.render(scene, camera);
  if (mapViewActive()) {
    fpsMeter.markFrame();
    mapRenderFrame = requestAnimationFrame(render);
  }
}

function ensureMapRenderLoop() {
  if (mapRenderFrame === null && mapViewActive()) {
    fpsMeter.setActive(true);
    mapRenderFrame = requestAnimationFrame(render);
  }
}

function pauseMapRenderLoop() {
  fpsMeter.setActive(false);
  if (mapRenderFrame !== null) {
    cancelAnimationFrame(mapRenderFrame);
    mapRenderFrame = null;
  }
}

function canvasPoint(event) {
  const rect = mapCanvas3d.getBoundingClientRect();
  return {
    x: event.clientX - rect.left,
    y: event.clientY - rect.top,
    button: event.button,
  };
}

function touchGesture() {
  const touches = Array.from(state.pointer.touches.values());
  if (touches.length < 2) return null;
  const first = touches[0];
  const second = touches[1];
  const center = { x: (first.x + second.x) / 2, y: (first.y + second.y) / 2 };
  const distance = Math.hypot(second.x - first.x, second.y - first.y);
  return { center, distance };
}

function groundIntersection(event) {
  const rect = mapCanvas3d.getBoundingClientRect();
  pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointerNdc, camera);
  const hit = new THREE.Vector3();
  return raycaster.ray.intersectPlane(groundPlane, hit) ? hit : null;
}

function pointCloudIntersection(event) {
  if (!LAYERS.mapground.object || !LAYERS.mapground.enabled) return null;
  const rect = mapCanvas3d.getBoundingClientRect();
  pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointerNdc, camera);
  const hits = raycaster.intersectObject(LAYERS.mapground.object, false);
  return hits.length ? hits[0].point : null;
}

function selectionIntersection(event) {
  if (state.mode === "initial") return groundIntersection(event);
  if (state.mode === "goal") return pointCloudIntersection(event);
  return null;
}

function setSelectionFromPoint(point, yaw = 0) {
  const pose = threeToRos(point);
  state.selection = { x: pose.x, y: pose.y, z: pose.z, yaw };
  updateSelectionDisplay();
  updateTargetMarkers();
}

function panCameraByScreenDelta(dx, dy) {
  const scale = state.camera.distance * 0.0018;
  const right = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 0);
  const up = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 1);
  state.camera.target.addScaledVector(right, -dx * scale);
  state.camera.target.addScaledVector(up, dy * scale);
}

function rotateCameraByScreenDelta(dx, dy) {
  state.camera.yaw += dx * 0.006;
  state.camera.pitch = Math.max(0.18, Math.min(1.38, state.camera.pitch + dy * 0.006));
}

function handleTouchPanZoom(event) {
  state.pointer.touches.set(event.pointerId, canvasPoint(event));
  const gesture = touchGesture();
  if (!gesture) return;
  if (!state.pointer.touchStart) {
    state.pointer.touchStart = { ...gesture, cameraDistance: state.camera.distance };
    return;
  }
  const startDistance = Math.max(1, state.pointer.touchStart.distance);
  const zoomRatio = startDistance / Math.max(1, gesture.distance);
  state.camera.distance = Math.max(5, Math.min(280, state.pointer.touchStart.cameraDistance * zoomRatio));
  const dx = gesture.center.x - state.pointer.touchStart.center.x;
  const dy = gesture.center.y - state.pointer.touchStart.center.y;
  panCameraByScreenDelta(dx, dy);
  state.pointer.touchStart = { ...gesture, cameraDistance: state.camera.distance };
}

function makePointCloud(points, color, options = {}) {
  const geometry = new THREE.BufferGeometry();
  const positions = new Float32Array(points.length * 3);
  points.forEach((point, index) => {
    const p = rosToThree(point[0], point[1], point[2] || 0);
    positions[index * 3] = p.x;
    positions[index * 3 + 1] = p.y;
    positions[index * 3 + 2] = p.z;
  });
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  const pointSize = Number.isFinite(options.pointSize) ? options.pointSize : 2;
  const opacity = Number.isFinite(options.opacity) ? options.opacity : 0.86;
  return new THREE.Points(
    geometry,
    new THREE.PointsMaterial({
      color,
      size: pointSize,
      sizeAttenuation: false,
      transparent: true,
      opacity,
      vertexColors: false,
    }),
  );
}

// Point clouds and path meshes own the GPU buffers created for them.  Release
// those resources when a refresh replaces the object; marker/vehicle groups are
// deliberately left alone because their cleanup is separate.
function disposeObjectResources(object) {
  if (!object) return;
  if (object.geometry && typeof object.geometry.dispose === "function") {
    object.geometry.dispose();
  }
  const materials = Array.isArray(object.material)
    ? object.material
    : object.material ? [object.material] : [];
  materials.forEach((material) => {
    if (material && typeof material.dispose === "function") material.dispose();
  });
}

function disposeGroupResources(group) {
  while (group.children.length) {
    const object = group.children[0];
    group.remove(object);
    disposeObjectResources(object);
  }
}

function updateLayerObject(name) {
  const layer = LAYERS[name];
  if (layer.object) {
    layerGroup.remove(layer.object);
    disposeObjectResources(layer.object);
  }
  layer.object = makePointCloud(layer.points3d, layer.color, layer);
  layer.object.visible = layer.enabled;
  layerGroup.add(layer.object);
}

function updateMappingLayerObject(name) {
  const layer = MAPPING_LAYERS[name];
  if (layer.object) {
    mappingLayerGroup.remove(layer.object);
    disposeObjectResources(layer.object);
  }
  layer.object = makePointCloud(layer.points3d, layer.color, layer);
  layer.object.visible = layer.enabled;
  mappingLayerGroup.add(layer.object);
}

function clearNavigationSceneState() {
  refreshController.invalidatePaths();
  state.paths = {};
  state.activeGoal = null;
  state.lastGoal = null;
  state.hideCompletedPaths = true;
  updatePaths();
  updateTargetPoseDisplay();
  updateTargetMarkers();
}

// mapcloud/mapground/obstacles are published by the navigation process; clear
// them immediately on navigation stop instead of waiting for the next poll to
// pull the now-empty backend cache.  global_map/local_map are
// localization-owned and stay until localization itself restarts.
function clearNavigationLayers() {
  ["mapcloud", "mapground", "obstacles"].forEach((name) => {
    LAYERS[name].points3d = [];
    LAYERS[name].lastFetch3d = 0;
    LAYERS[name].staticLoaded = false;
    updateLayerObject(name);
  });
  refreshController.invalidateNavigationLayers(["mapcloud", "mapground", "obstacles"]);
}

// A real map selection/loading-mode change or localization runtime change makes
// every navigation cloud stale.  Keep the UI responsive but wait for fresh
// nonempty static clouds instead of showing the previous map's points.
function invalidateNavigationMapCaches() {
  Object.keys(LAYERS).forEach((name) => {
    LAYERS[name].points3d = [];
    LAYERS[name].lastFetch3d = 0;
    LAYERS[name].staticLoaded = false;
    updateLayerObject(name);
  });
  refreshController.invalidateNavigationLayers();
}

function clearMapSceneState() {
  if (state.mapSceneCleared) return;
  Object.keys(LAYERS).forEach((name) => {
    LAYERS[name].points3d = [];
    LAYERS[name].lastFetch3d = 0;
    LAYERS[name].staticLoaded = false;
    updateLayerObject(name);
  });
  refreshController.invalidateNavigationLayers();
  state.odom = null;
  state.selection = null;
  clearNavigationSceneState();
  updateVehicleMarker();
  updateSelectionDisplay();
  state.mapSceneCleared = true;
}

function mapSourcesAreWaiting(layers) {
  return Object.keys(LAYERS).every((name) => {
    const summary = layers[name];
    return summary && summary.status === "waiting" && !summary.count;
  });
}

function clearGroup(group) {
  while (group.children.length) group.remove(group.children[0]);
}

function makeArrowMarker(pose, color) {
  const group = new THREE.Group();
  group.position.copy(rosToThree(pose.x, pose.y, (pose.z || 0) + 0.1));
  const yaw = pose.yaw || 0;
  const dir = new THREE.Vector3(Math.cos(yaw), 0, -Math.sin(yaw)).normalize();
  group.add(new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0.15, 0), 3.2, color, 0.9, 0.45));
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.9, 0.035, 10, 48),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9 }),
  );
  ring.rotation.x = Math.PI / 2;
  group.add(ring);
  return group;
}

function updateTargetMarkers() {
  clearGroup(markerGroup);
  if (state.selection && state.mode !== "view") {
    markerGroup.add(makeArrowMarker(state.selection, TARGET_MARKER_COLOR));
  }
  const goal = state.activeGoal || state.lastGoal;
  if (goal) {
    markerGroup.add(makeArrowMarker(goal, TARGET_MARKER_COLOR));
  }
}

function updateVehicleMarker() {
  clearGroup(vehicleGroup);
  if (!state.showVehicle || !state.odom || !state.odom.pose) return;
  vehicleGroup.add(makeArrowMarker({ ...state.odom.pose, layer: "mapcloud" }, 0x32d74b));
}

function updateMappingVehicleMarker() {
  clearGroup(mappingVehicleGroup);
  if (!state.showMappingVehicle || !state.mappingOdom || !state.mappingOdom.pose) return;
  mappingVehicleGroup.add(makeArrowMarker(state.mappingOdom.pose, 0x32d74b));
}

function pathPointToThree(point) {
  const z = point.length > 2 && Number.isFinite(point[2]) ? point[2] : 0;
  return rosToThree(point[0], point[1], z + 0.08);
}

function updatePaths() {
  disposeGroupResources(pathGroup);
  if (!state.showPaths) return;
  if (state.hideCompletedPaths) return;
  Object.entries(state.paths).forEach(([topic, path]) => {
    if (!path || path.length < 2) return;
    const points = path.map((point) => pathPointToThree(point));
    const pathCurve = new THREE.CatmullRomCurve3(points);
    const geometry = new THREE.TubeGeometry(pathCurve, Math.max(2, points.length * 2), 0.16, 8, false);
    const line = new THREE.Mesh(
      geometry,
      new THREE.MeshBasicMaterial({ color: 0x0a84ff, transparent: true, opacity: 0.9 }),
    );
    line.name = topic;
    pathGroup.add(line);
  });
}

function updateMappingPath() {
  disposeGroupResources(mappingPathGroup);
  if (!state.showMappingPath || !state.mappingPath || state.mappingPath.length < 2) return;
  const points = state.mappingPath.map((point) => pathPointToThree(point));
  const pathCurve = new THREE.CatmullRomCurve3(points);
  const geometry = new THREE.TubeGeometry(pathCurve, Math.max(2, points.length * 2), 0.16, 8, false);
  const line = new THREE.Mesh(
    geometry,
    new THREE.MeshBasicMaterial({ color: 0x32d74b, transparent: true, opacity: 0.9 }),
  );
  line.name = "mapping_path";
  mappingPathGroup.add(line);
}

async function pollState() {
  try {
    const data = await timedGetJson("/api/state");
    updateStatus(data);
    updateViewerCount(data.viewers);
    document.getElementById("serverState").textContent = "active";
    document.getElementById("serverState").className = "pill active";
  } catch (_error) {
    document.getElementById("serverState").textContent = "error";
    document.getElementById("serverState").className = "pill error";
  }
}

async function pollLogs() {
  try {
    const url = `/api/logs?chassis=${lastLogSeq.chassis}&imu=${lastLogSeq.imu}&lidar=${lastLogSeq.lidar}&localization=${lastLogSeq.localization}&navigation=${lastLogSeq.navigation}&mapping=${lastLogSeq.mapping}`;
    const data = await getJson(url);
    updateTerminalLog("chassis", data.logs && data.logs.chassis);
    updateTerminalLog("imu", data.logs && data.logs.imu);
    updateTerminalLog("lidar", data.logs && data.logs.lidar);
    updateTerminalLog("localization", data.logs && data.logs.localization);
    updateTerminalLog("navigation", data.logs && data.logs.navigation);
    updateTerminalLog("mapping", data.logs && data.logs.mapping);
  } catch (_error) {
    updateTerminalLog("chassis", null);
    updateTerminalLog("imu", null);
    updateTerminalLog("lidar", null);
    updateTerminalLog("localization", null);
    updateTerminalLog("navigation", null);
    updateTerminalLog("mapping", null);
  }
}

async function pollSystem() {
  if (state.view !== "system") return;
  try {
    updateSystem(await getJson("/api/system"));
  } catch (_error) {
    updateSystem(null);
  }
}

// --- Topic and wheel-motion diagnostics
// This poll is intentionally global: rate chips live on both Navigation and
// Mapping, while the backend timer and persistent log run regardless of UI.
const DIAGNOSTICS_STATUS_LABELS = {
  active: "正常",
  stale: "数据过期",
  waiting: "等待数据",
  no_publisher: "无发布者",
  unavailable: "不可用",
};

// Backend state is authoritative.  The visible switch is never sourced from
// localStorage, and a pending POST/stale poll cannot overwrite a newer choice.
const diagnosticsView = {
  pollSeq: 0,
  logging: {
    pending: false,
    pendingValue: null,
    enabled: null,
    error: null,
    writeError: null,
  },
};

function diagnosticsStatusLabel(status) {
  return DIAGNOSTICS_STATUS_LABELS[status] || status || "不可用";
}

function formatDiagnosticsAge(age) {
  if (!Number.isFinite(age)) return "--";
  if (age < 10) return `${age.toFixed(1)}s`;
  return `${Math.round(age)}s`;
}

function formatDiagnosticsHz(row) {
  // A rate from the last active window is historical once the feed becomes
  // stale; never present it as a current measurement.
  if (!row || row.status !== "active") return "--";
  if (!Number.isFinite(row.hz) || row.hz <= 0) return "--";
  return `${row.hz.toFixed(row.hz >= 10 ? 1 : 2)} Hz`;
}

function updateRateChips(data) {
  const topics = data && Array.isArray(data.topics) ? data.topics : [];
  const byKey = new Map(topics.map((row) => [row.key, row]));
  document.querySelectorAll("[data-diagnostics-chip]").forEach((chip) => {
    const key = chip.dataset.diagnosticsChip;
    const row = byKey.get(key);
    const name = key === "imu" ? "IMU" : "Lidar";
    if (!row) {
      chip.textContent = `${name} --`;
      chip.dataset.state = "unavailable";
      return;
    }
    chip.dataset.state = row.status;
    if (row.status === "active") {
      chip.textContent = `${name} ${formatDiagnosticsHz(row)}`;
    } else {
      chip.textContent = `${name} ${diagnosticsStatusLabel(row.status)}`;
    }
  });
}

function diagnosticsTopicRow(row) {
  const tr = document.createElement("tr");
  tr.dataset.topicKey = row.key || "";
  const values = [
    row.topic || "--",
    row.message_type || "--",
    Number.isFinite(row.publisher_count) ? String(row.publisher_count) : "--",
    formatDiagnosticsHz(row),
    formatDiagnosticsAge(row.last_age_sec),
    diagnosticsStatusLabel(row.status),
  ];
  values.forEach((value, index) => {
    const cell = document.createElement(index === 0 ? "th" : "td");
    if (index === 0) cell.scope = "row";
    cell.textContent = value;
    if (index === 5) cell.className = `diagnostics-state state-${row.status || "unavailable"}`;
    tr.appendChild(cell);
  });
  return tr;
}

function updateDiagnosticsMotion(motion) {
  const stateLabels = { moving: "移动", stationary: "静止", unknown: "未知" };
  const motionState = document.getElementById("diagnosticsMotionState");
  const source = document.getElementById("diagnosticsMotionSource");
  const speed = document.getElementById("diagnosticsSpeed");
  const distance = document.getElementById("diagnosticsDistance");
  const segment = document.getElementById("diagnosticsSegment");
  const validityText = document.getElementById("diagnosticsValidity");
  if (!motion) {
    if (motionState) motionState.textContent = "未知";
    if (source) source.textContent = "来源 --";
    if (speed) speed.textContent = "--";
    if (distance) distance.textContent = "--";
    if (segment) segment.textContent = "--";
    if (validityText) validityText.textContent = "轮速估计：未收到诊断数据";
    return;
  }
  let stateLabel = stateLabels[motion.state] || "未知";
  if (motion.state === "moving" && motion.direction) {
    stateLabel += motion.direction === "reverse" ? " · 倒车" : " · 前进";
  }
  if (motionState) motionState.textContent = stateLabel;
  if (source) source.textContent = `来源 ${motion.source_topic || motion.topic || "--"} · 轮速`;
  const validity = motion.validity || {};
  const speedCurrent = validity.available === true
    && validity.paused !== true
    && validity.valid !== false;
  if (speed) {
    speed.textContent = speedCurrent && Number.isFinite(motion.speed_mps)
      ? `${motion.speed_mps.toFixed(2)} m/s`
      : "--（当前不可用）";
  }
  const total = motion.total_distance_m;
  if (distance) {
    distance.textContent = Number.isFinite(total) ? `${total.toFixed(2)} m（轮速估计）` : "--（未收到里程计）";
  }
  if (segment) {
    const segDistance = motion.segment_distance_m;
    const segDuration = motion.segment_duration_sec;
    segment.textContent = validity.available === true && Number.isFinite(segDistance)
      ? `${segDistance.toFixed(2)} m / ${Number.isFinite(segDuration) ? segDuration.toFixed(1) : "--"} s`
      : "--";
  }
  if (validityText) {
    if (!validity.available) {
      validityText.textContent = "轮速估计：不可用（尚未收到有效底盘里程计）";
    } else if (validity.paused) {
      validityText.textContent = `轮速估计：已暂停（${validity.reason || "数据中断"}），累计里程保留且可能不完整`;
    } else if (validity.incomplete) {
      validityText.textContent = "轮速估计：当前有效，但本次会话曾中断，累计里程可能不完整";
    } else {
      validityText.textContent = "轮速估计：当前有效（仅统计本次网页节点会话）";
    }
  }
}

function renderDiagnosticsImu(obs) {
  const imu = document.getElementById("diagnosticsImu");
  if (!imu) return;
  if (!obs) {
    imu.textContent = "IMU 辅助观察不可用（接口不可用或无有效观测）";
    return;
  }
  const accel = obs.acceleration_mps2 || {};
  const gyro = obs.angular_velocity_rps || {};
  const vector = (v) => ["x", "y", "z"]
    .map((axis) => (Number.isFinite(v[axis]) ? v[axis].toFixed(2) : "--"))
    .join("/");
  const reading = `acc ${vector(accel)} m/s² · gyro ${vector(gyro)} rad/s`;
  if (obs.status === "stale") {
    const age = formatDiagnosticsAge(obs.last_age_sec);
    imu.textContent = `IMU 最近一次辅助观察（已过期 ${age}，非运动依据） ${reading}`;
  } else if (obs.status === "active" || obs.status == null) {
    imu.textContent = `IMU 辅助观察（非运动依据） ${reading}`;
  } else {
    imu.textContent = `IMU 辅助观察不可用（${diagnosticsStatusLabel(obs.status)}）`;
  }
}

function topicLoggingButtons() {
  return Array.from(document.querySelectorAll("[data-topic-logging-toggle]"));
}

function topicLoggingStates() {
  return Array.from(document.querySelectorAll("[data-topic-logging-state]"));
}

function renderTopicLogging() {
  const buttons = topicLoggingButtons();
  const states = topicLoggingStates();
  if (!buttons.length && !states.length) return;
  const logging = diagnosticsView.logging;
  const known = typeof logging.enabled === "boolean";
  const enabled = known ? logging.enabled : null;
  let message;
  let isError = false;
  if (logging.error) {
    message = logging.error;
    isError = true;
  } else if (logging.pending) {
    message = "正在保存...";
  } else if (logging.writeError) {
    message = `日志写入错误：${logging.writeError}`;
    isError = true;
  } else {
    message = "";
  }
  // Both pages share one authoritative state and stay visually synchronized.
  buttons.forEach((button) => {
    button.textContent = enabled === true ? "停止记录话题" : "开始记录话题";
    button.disabled = logging.pending || !known;
    button.classList.toggle("start-log", enabled !== true);
    button.classList.toggle("stop-log", enabled === true);
    button.setAttribute("aria-pressed", String(enabled === true));
    button.setAttribute("aria-busy", String(logging.pending));
  });
  states.forEach((stateText) => {
    stateText.textContent = message;
    stateText.hidden = message === "";
    stateText.className = isError
      ? "topic-logging-state is-error"
      : "topic-logging-state";
  });
}

function applyTopicLoggingFromData(data) {
  const logging = diagnosticsView.logging;
  // Never let a poll that raced a pending POST change the visible buttons.
  if (
    !logging.pending
    && data
    && typeof data.topic_logging_enabled === "boolean"
  ) {
    logging.enabled = data.topic_logging_enabled;
  }
  logging.writeError = data && data.logging && data.logging.error
    ? String(data.logging.error)
    : null;
  renderTopicLogging();
}

async function toggleTopicLogging() {
  const logging = diagnosticsView.logging;
  if (logging.pending || typeof logging.enabled !== "boolean") {
    renderTopicLogging();
    return;
  }
  // The buttons are actions, not checkboxes: derive the target from the
  // shared authoritative state so a click on a child target is harmless.
  const desired = !logging.enabled;
  const previous = logging.enabled;
  logging.pending = true;
  logging.pendingValue = desired;
  logging.error = null;
  renderTopicLogging();
  try {
    const data = await postJson("/api/diagnostics/logging", { enabled: desired });
    // A successful switch invalidates any GET that started before it.
    diagnosticsView.pollSeq += 1;
    if (!data || typeof data.topic_logging_enabled !== "boolean") {
      throw new Error("后台响应缺少话题日志状态");
    }
    logging.enabled = data.topic_logging_enabled;
    logging.writeError = data.logging && data.logging.error
      ? String(data.logging.error)
      : null;
  } catch (error) {
    const detail = error && error.message ? error.message : String(error);
    const token = ++diagnosticsView.pollSeq;
    let resynced = false;
    try {
      const fresh = await getJson("/api/diagnostics");
      if (token === diagnosticsView.pollSeq) {
        if (fresh && typeof fresh.topic_logging_enabled === "boolean") {
          logging.enabled = fresh.topic_logging_enabled;
          resynced = true;
        }
        logging.writeError = fresh && fresh.logging && fresh.logging.error
          ? String(fresh.logging.error)
          : null;
      }
    } catch (_resyncError) {
      // Keep the previous authoritative value if the resync is unreachable.
    }
    if (!resynced) logging.enabled = previous;
    // Persistent and actionable: later polls must not silently erase it.
    const recovery = resynced
      ? "已重新读取后台状态，请检查后重试。"
      : "未能确认后台状态，暂显示上次状态，请检查连接后重试。";
    logging.error = `保存话题日志开关失败：${detail}。${recovery}`;
  } finally {
    logging.pending = false;
    logging.pendingValue = null;
    renderTopicLogging();
  }
}

function updateDiagnostics(data) {
  const status = document.getElementById("diagnosticsStatus");
  const tbody = document.getElementById("diagnosticsTopics");
  updateRateChips(data);
  applyTopicLoggingFromData(data);
  if (!data || data.ok === false) {
    if (status) status.textContent = data && data.message ? data.message : "unavailable";
    if (tbody) {
      tbody.replaceChildren();
      const tr = document.createElement("tr");
      const td = document.createElement("td");
      td.colSpan = 6;
      td.className = "diagnostics-empty";
      td.textContent = "诊断接口不可用";
      tr.appendChild(td);
      tbody.appendChild(tr);
    }
    updateDiagnosticsMotion(null);
    renderDiagnosticsImu(null);
    return;
  }
  const topics = Array.isArray(data.topics) ? data.topics : [];
  if (tbody) {
    tbody.replaceChildren(...topics.map(diagnosticsTopicRow));
    if (!topics.length) {
      const tr = document.createElement("tr");
      const td = document.createElement("td");
      td.colSpan = 6;
      td.className = "diagnostics-empty";
      td.textContent = "未配置诊断话题";
      tr.appendChild(td);
      tbody.appendChild(tr);
    }
  }
  const logging = data.logging || {};
  const activeCount = topics.filter((row) => row.status === "active").length;
  if (status) {
    status.textContent = logging.error
      ? `日志错误：${logging.error}`
      : `${topics.length} 个话题 · ${activeCount} 个正常`;
    status.className = logging.error
      ? "system-panel-status is-error"
      : "system-panel-status";
  }
  updateDiagnosticsMotion(data.motion || null);
  renderDiagnosticsImu(data.imu_observation);
}

async function pollDiagnostics() {
  const token = ++diagnosticsView.pollSeq;
  try {
    const data = await getJson("/api/diagnostics");
    if (token !== diagnosticsView.pollSeq) return;
    updateDiagnostics(data);
  } catch (_error) {
    if (token !== diagnosticsView.pollSeq) return;
    updateDiagnostics(null);
  }
}

// --- System TF tree
// Low-frequency poll that only runs while the System view is visible, with a
// request token so a slow response from a previous view/tab can never
// overwrite the current tree.
const TF_AGE_STALE_LABEL = "stale";
const tfViewState = { requestSeq: 0, sessionsSeq: 0 };
let tfGraphView = null;

async function pollTf() {
  if (state.view !== "system") return;
  const token = ++tfViewState.requestSeq;
  try {
    const data = await getJson("/api/tf");
    if (token !== tfViewState.requestSeq || state.view !== "system") return;
    updateTfTree(data);
  } catch (_error) {
    if (token !== tfViewState.requestSeq || state.view !== "system") return;
    updateTfTree(null);
  }
}

function ensureTfGraphView() {
  const host = document.getElementById("tfTree");
  if (!host) return null;
  if (!tfGraphView) {
    tfGraphView = new TfGraphView(host, {
      // Only real /api/tf fields are surfaced; nothing is fabricated.
      onSelect: showTfSelection,
    });
  }
  return tfGraphView;
}

function showTfSelection(node) {
  const output = document.getElementById("tfSelection");
  if (!output) return;
  if (!node) {
    output.textContent = "Select a frame to inspect its transform.";
    return;
  }
  const edge = node.edge;
  const parts = [node.frame, `parent: ${node.parent || "none"}`];
  parts.push(`type: ${edge ? (edge.static ? "static" : "dynamic") : "root"}`);
  if (edge) {
    parts.push(`age: ${formatTfAge(edge.age)}`);
    if (edge.stale) parts.push(TF_AGE_STALE_LABEL);
  }
  parts.push(`children: ${Array.isArray(node.children) ? node.children.length : 0}`);
  output.textContent = parts.join(" · ");
}

function renderTfDetached(data) {
  const detachedBox = document.getElementById("tfDetached");
  if (!detachedBox) return;
  detachedBox.replaceChildren();
  const detached = data && Array.isArray(data.detached) ? data.detached : [];
  if (!detached.length) {
    detachedBox.hidden = true;
    return;
  }
  const heading = document.createElement("p");
  heading.className = "tf-detached-title";
  heading.textContent = "Unplaced edges";
  detachedBox.appendChild(heading);
  const list = document.createElement("ul");
  list.className = "tf-detached-list";
  detached.forEach((edge) => {
    const item = document.createElement("li");
    const kind = edge.static ? "static" : "dynamic";
    item.textContent = `${edge.parent || "?"} -> ${edge.child || "?"} (${kind}): ${edge.reason || "unplaced"}`;
    list.appendChild(item);
  });
  detachedBox.appendChild(list);
  detachedBox.hidden = false;
}

function updateTfTree(data) {
  const status = document.getElementById("tfTreeStatus");
  const view = ensureTfGraphView();
  if (!data) {
    if (status) status.textContent = "unavailable";
    if (view) {
      view.setActive(state.view === "system");
      view.update({ ok: false, status: "unavailable", trees: [], detached: [] });
    }
    renderTfDetached(null);
    return;
  }
  const trees = Array.isArray(data.trees) ? data.trees : [];
  if (status) {
    status.textContent = data.message ? data.message : `${data.frame_count} frames`;
  }
  if (view) {
    view.setActive(state.view === "system");
    view.update({ ...data, trees });
  }
  renderTfDetached(data);
}

// --- System saved (persistent) logs
function savedLogTarget() {
  const select = document.getElementById("savedLogTarget");
  return select ? select.value : "";
}

function savedLogSession() {
  const select = document.getElementById("savedLogSession");
  return select ? select.value : "";
}

function setSavedLogStatus(message) {
  const element = document.getElementById("savedLogStatus");
  if (element) element.textContent = message;
}

function updateSavedLogActions() {
  const hasSession = Boolean(savedLogSession());
  const view = document.getElementById("savedLogView");
  const download = document.getElementById("savedLogDownload");
  if (view) view.disabled = !hasSession;
  if (download) {
    if (hasSession) {
      download.href = `/api/logs/download?target=${encodeURIComponent(savedLogTarget())}&session=${encodeURIComponent(savedLogSession())}`;
      download.classList.remove("disabled");
    } else {
      download.href = "#";
      download.classList.add("disabled");
    }
  }
}

async function refreshSavedLogSessions() {
  const target = savedLogTarget();
  const label = MANAGED_PROCESS_LABELS[target] || target;
  const token = ++tfViewState.sessionsSeq;
  const select = document.getElementById("savedLogSession");
  if (!select || !target) return;
  try {
    const data = await getJson(`/api/logs/sessions?target=${encodeURIComponent(target)}`);
    if (token !== tfViewState.sessionsSeq || target !== savedLogTarget()) return;
    select.replaceChildren();
    const sessions = Array.isArray(data.sessions) ? data.sessions : [];
    sessions.forEach((session) => {
      const option = document.createElement("option");
      option.value = session.session;
      const when = new Date(session.started_at * 1000).toLocaleString();
      const active = session.active ? " (running)" : "";
      option.textContent = `${when}${active}`;
      select.appendChild(option);
    });
    if (!sessions.length) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "no saved runs";
      select.appendChild(option);
      setSavedLogStatus(
        data.persist_error ? `Log persistence error: ${data.persist_error}` : `No saved logs for ${label}.`,
      );
    } else if (data.persist_error) {
      setSavedLogStatus(`Log persistence error: ${data.persist_error}`);
    } else {
      setSavedLogStatus(`${sessions.length} saved run(s) for ${label}.`);
    }
    updateSavedLogActions();
  } catch (error) {
    if (token !== tfViewState.sessionsSeq) return;
    select.replaceChildren();
    setSavedLogStatus(`Saved log list failed: ${error.message}`);
    updateSavedLogActions();
  }
}

async function viewSavedLogTail() {
  const target = savedLogTarget();
  const session = savedLogSession();
  const output = document.getElementById("savedLogTail");
  if (!output || !session) return;
  const token = ++tfViewState.sessionsSeq;
  try {
    const text = await getText(
      `/api/logs/tail?target=${encodeURIComponent(target)}&session=${encodeURIComponent(session)}`,
    );
    if (token !== tfViewState.sessionsSeq || session !== savedLogSession()) return;
    output.textContent = text || "No output.";
    output.scrollTop = output.scrollHeight;
    setSavedLogStatus(`Showing tail of ${session}.`);
  } catch (error) {
    if (token !== tfViewState.sessionsSeq) return;
    setSavedLogStatus(`Read failed: ${error.message}`);
  }
}

// --- Data refresh scheduler
// Every resource below has its own deadline.  A task is eligible only when the
// current view exposes it, the layer is enabled, its own deadline has passed,
// and no request for that same resource is still in flight.  There is no queue:
// a slow global cloud can never delay obstacles, local_map, or paths.
const REFRESH_RATES_MS = Object.freeze({
  navigationLayers: Object.freeze({
    global_map: 1000, // retry cadence until the first nonempty cloud is cached
    local_map: 1000,
    mapcloud: 1000, // retry cadence until the first nonempty cloud is cached
    mapground: 1000, // retry cadence until the first nonempty cloud is cached
    obstacles: 200,
  }),
  mappingLayers: 1000,
  paths: 200,
  mappingPath: 200,
  schedulerTick: 50,
});

// These navigation clouds are static after their first valid nonempty load.
// The mapping overview intentionally keeps refreshing at 1 Hz while Mapping is
// active because it grows during a mapping run.
const STATIC_NAVIGATION_LAYERS = new Set(["global_map", "mapcloud", "mapground"]);

function createRefreshController({
  now = () => Date.now(),
  getJson,
  getView,
  navigationLayers,
  mappingLayers,
  getPathTopics = () => [],
  pathsVisible = () => true,
  mappingPathVisible = () => true,
  applyNavigationLayer = () => {},
  applyMappingLayer = () => {},
  applyPath = () => {},
  applyMappingPath = () => {},
}) {
  const navigationTasks = new Map();
  const mappingTasks = new Map();
  const pathTasks = new Map();
  const mappingPathTask = { key: "mappingPath", interval: REFRESH_RATES_MS.mappingPath, due: 0, inFlight: false, generation: 0 };

  function makeTask(key, interval) {
    return { key, interval, due: 0, inFlight: false, generation: 0 };
  }
  for (const name of Object.keys(navigationLayers)) {
    navigationTasks.set(name, makeTask(name, REFRESH_RATES_MS.navigationLayers[name] || REFRESH_RATES_MS.navigationLayers.obstacles));
  }
  for (const name of Object.keys(mappingLayers)) {
    mappingTasks.set(name, makeTask(name, REFRESH_RATES_MS.mappingLayers));
  }

  function invalidateTask(task) {
    task.generation += 1;
    task.due = 0;
  }

  function syncPathTasks() {
    const topics = Array.from(new Set(getPathTopics() || []));
    for (const topic of topics) {
      if (!pathTasks.has(topic)) pathTasks.set(topic, makeTask(topic, REFRESH_RATES_MS.paths));
    }
    for (const topic of Array.from(pathTasks.keys())) {
      if (!topics.includes(topic)) pathTasks.delete(topic);
    }
  }

  function navigationEligible(name, task, at) {
    const layer = navigationLayers[name];
    if (!layer || !layer.enabled || task.inFlight || at < task.due) return false;
    return !(STATIC_NAVIGATION_LAYERS.has(name) && layer.staticLoaded);
  }

  async function requestNavigationLayer(name, task, at) {
    task.inFlight = true;
    task.due = at + task.interval;
    const generation = task.generation;
    try {
      const data = await getJson(`/api/map3d?layer=${encodeURIComponent(name)}`);
      if (task.generation !== generation) return;
      const points = Array.isArray(data.points) ? data.points : [];
      const layer = navigationLayers[name];
      layer.points3d = points;
      layer.lastFetch3d = now();
      if (STATIC_NAVIGATION_LAYERS.has(name)) layer.staticLoaded = points.length > 0;
      applyNavigationLayer(name, layer);
    } catch (_error) {
      // Never cache an error.  The next deadline retries normally.
    } finally {
      task.inFlight = false;
    }
  }

  function mappingEligible(name, task, at) {
    const layer = mappingLayers[name];
    return Boolean(layer && layer.enabled && !task.inFlight && at >= task.due);
  }

  async function requestMappingLayer(name, task, at) {
    task.inFlight = true;
    task.due = at + task.interval;
    const generation = task.generation;
    try {
      const data = await getJson(`/api/mapping/map3d?layer=${encodeURIComponent(name)}`);
      if (task.generation !== generation) return;
      const layer = mappingLayers[name];
      layer.points3d = Array.isArray(data.points) ? data.points : [];
      layer.lastFetch3d = now();
      applyMappingLayer(name, layer);
    } catch (_error) {
      // Keep the last good mapping cloud and retry at the next deadline.
    } finally {
      task.inFlight = false;
    }
  }

  function pathEligible(task, at) {
    return !task.inFlight && at >= task.due;
  }

  async function requestPath(topic, task, at) {
    task.inFlight = true;
    task.due = at + task.interval;
    const generation = task.generation;
    try {
      const data = await getJson(`/api/path?topic=${encodeURIComponent(topic)}`);
      if (pathTasks.get(topic) !== task || task.generation !== generation) return;
      applyPath(topic, Array.isArray(data.points) ? data.points : []);
    } catch (_error) {
      // Keep the last path and retry at the next deadline.
    } finally {
      task.inFlight = false;
    }
  }

  async function requestMappingPath(at) {
    mappingPathTask.inFlight = true;
    mappingPathTask.due = at + mappingPathTask.interval;
    const generation = mappingPathTask.generation;
    try {
      const data = await getJson("/api/mapping/path");
      if (mappingPathTask.generation !== generation) return;
      applyMappingPath(Array.isArray(data.points) ? data.points : []);
    } catch (_error) {
      // Keep the last trajectory and retry at the next deadline.
    } finally {
      mappingPathTask.inFlight = false;
    }
  }

  function tick(at = now()) {
    const started = [];
    const view = getView();
    if (view === "3d") {
      for (const [name, task] of navigationTasks) {
        if (navigationEligible(name, task, at)) started.push(requestNavigationLayer(name, task, at));
      }
      if (pathsVisible()) {
        syncPathTasks();
        for (const [topic, task] of pathTasks) {
          if (pathEligible(task, at)) started.push(requestPath(topic, task, at));
        }
      }
    } else if (view === "mapping") {
      for (const [name, task] of mappingTasks) {
        if (mappingEligible(name, task, at)) started.push(requestMappingLayer(name, task, at));
      }
      if (mappingPathVisible() && pathEligible(mappingPathTask, at)) {
        started.push(requestMappingPath(at));
      }
    }
    return started;
  }

  function invalidateNavigationLayer(name) {
    const layer = navigationLayers[name];
    if (layer) {
      layer.points3d = [];
      layer.lastFetch3d = 0;
      layer.staticLoaded = false;
    }
    const task = navigationTasks.get(name);
    if (task) invalidateTask(task);
  }

  function invalidateNavigationLayers(names = Object.keys(navigationLayers)) {
    for (const name of names) invalidateNavigationLayer(name);
  }

  function invalidatePaths() {
    for (const task of pathTasks.values()) invalidateTask(task);
  }

  return {
    tick,
    invalidateNavigationLayer,
    invalidateNavigationLayers,
    invalidatePaths,
  };
}
// --- End data refresh scheduler

const refreshController = createRefreshController({
  now: () => Date.now(),
  getJson,
  getView: () => state.view,
  navigationLayers: LAYERS,
  mappingLayers: MAPPING_LAYERS,
  getPathTopics: () => state.pathTopics,
  pathsVisible: () => state.showPaths && !state.hideCompletedPaths,
  mappingPathVisible: () => state.showMappingPath,
  applyNavigationLayer(name, layer) {
    if (layer.points3d.length) state.mapSceneCleared = false;
    updateLayerObject(name);
  },
  applyMappingLayer(name) {
    updateMappingLayerObject(name);
  },
  applyPath(topic, points) {
    if (!state.showPaths || state.hideCompletedPaths || state.view !== "3d") return;
    state.paths[topic] = points;
    updatePaths();
  },
  applyMappingPath(points) {
    if (state.view !== "mapping" || !state.showMappingPath) return;
    state.mappingPath = points;
    updateMappingPath();
  },
});

async function pollMappingState() {
  if (state.view !== "mapping") return;
  try {
    const data = await getJson("/api/mapping/state");
    updateMappingStatus(data);
    updateSaveMapStatus(data.save_map);
    updateRecordingStatus(data.recording);
  } catch (_error) {
    updateMappingStatus(null);
    updateSaveMapStatus(null);
    updateRecordingStatus(null);
  }
}

async function pollPose() {
  if (state.view === "system") return;
  try {
    updateLivePose(await timedGetJson("/api/pose"));
  } catch (_error) {
    updateLivePose(null);
  }
}

function updateNavigationPose(odom) {
  state.odom = odom && odom.pose ? odom : null;
  const pose = state.odom ? state.odom.pose : null;
  document.getElementById("poseX").textContent = pose ? fmt(pose.x) : "--";
  document.getElementById("poseY").textContent = pose ? fmt(pose.y) : "--";
  document.getElementById("poseZ").textContent = pose ? fmt(pose.z) : "--";
  document.getElementById("poseYaw").textContent = pose ? fmt(pose.yaw, 3) : "--";
  document.getElementById("odomAge").textContent = state.odom && state.odom.age != null ? `${fmt(state.odom.age, 1)} s` : "--";
  updateVehicleMarker();
}

function updateMappingPose(odom) {
  state.mappingOdom = odom && odom.pose ? odom : null;
  const pose = state.mappingOdom ? state.mappingOdom.pose : null;
  document.getElementById("mappingPoseX").textContent = pose ? fmt(pose.x) : "--";
  document.getElementById("mappingPoseY").textContent = pose ? fmt(pose.y) : "--";
  document.getElementById("mappingPoseZ").textContent = pose ? fmt(pose.z) : "--";
  document.getElementById("mappingPoseYaw").textContent = pose ? fmt(pose.yaw, 3) : "--";
  document.getElementById("mappingOdomAge").textContent = state.mappingOdom && state.mappingOdom.age != null ? `${fmt(state.mappingOdom.age, 1)} s` : "--";
  updateMappingVehicleMarker();
}

function updateLivePose(data) {
  updateNavigationPose(data && data.odom);
  updateMappingPose(data && data.mapping_odom);
}

function updateStatus(data) {
  updateNavigationPose(data.odom);
  state.navigationAction = data.navigation_action || null;
  updateNavigationReadiness(data.navigation_ready);
  updateTargetPoseDisplay();
  maybeClearGoalFromActionStatus();
  updateNavigationControls(data.navigation_control);

  const sources = document.getElementById("sources");
  sources.innerHTML = "";
  Object.entries(data.layers || {}).forEach(([name, summary]) => sources.appendChild(sourceRow(name, summary)));
  Object.entries(data.paths || {}).forEach(([name, summary]) => sources.appendChild(sourceRow(name, summary)));
  sources.appendChild(sourceRow("odom", data.odom));
  if (mapSourcesAreWaiting(data.layers || {})) {
    clearMapSceneState();
  }
  // Store the current topic list here; the 5 Hz path scheduler reads it on its
  // own cadence so status polling never drives path downloads.
  state.pathTopics = Object.keys(data.paths || {});
}

function updateMappingStatus(data) {
  updateMappingPose(data && data.odom);

  const sources = document.getElementById("mappingSources");
  sources.innerHTML = "";
  if (!data) {
    sources.appendChild(sourceRow("mapping", { status: "waiting", age: null, frame_id: "", count: 0 }));
    return;
  }
  Object.entries(data.layers || {}).forEach(([name, summary]) => sources.appendChild(sourceRow(name, summary)));
  sources.appendChild(sourceRow("path", data.path));
  sources.appendChild(sourceRow("odom", data.odom));
}

function updateSaveMapStatus(status) {
  const button = document.getElementById("openSaveMap");
  const result = document.getElementById("saveMapStatus");
  const running = status && status.state === "running";
  button.disabled = !status || !status.available || running;
  if (running) {
    result.textContent = `Saving map to ${status.destination || "selected directory"} ...`;
  } else if (saveMapSubmissionError) {
    result.textContent = `Map save failed: ${saveMapSubmissionError}`;
  } else if (status && (status.state === "succeeded" || status.state === "failed")) {
    result.textContent = status.message || (status.state === "succeeded" ? "Map saved." : "Map save failed.");
  } else {
    result.textContent = status && status.available
      ? "Ready to save map."
      : "Start mapping to enable saving.";
  }
  if (status && status.state) {
    // Pick up a newly saved map directory the moment the export succeeds.
    if (status.state === "succeeded" && state.saveMapState !== "succeeded") {
      refreshMaps(true);
    }
    state.saveMapState = status.state;
  }
}

async function submitSaveMap(event) {
  event.preventDefault();
  const dialog = document.getElementById("saveMapDialog");
  const destinationInput = document.getElementById("saveMapDestination");
  const destination = destinationInput.value.trim();
  if (!destination.startsWith("/") || destination === "/") {
    destinationInput.setCustomValidity("Enter a path under the robot user's home directory, such as /map or /map/site.");
    destinationInput.reportValidity();
    return;
  }
  if (destination.split("/").includes("..")) {
    destinationInput.setCustomValidity("Destination cannot contain '..' path segments.");
    destinationInput.reportValidity();
    return;
  }
  destinationInput.setCustomValidity("");
  if (!window.confirm(`Save the map to ${destination} under the robot user's home directory?\n\nExisting map files in this directory will be overwritten. Unrelated files are preserved.`)) return;

  dialog.close();
  saveMapSubmissionError = null;
  document.getElementById("openSaveMap").disabled = true;
  document.getElementById("saveMapStatus").textContent = `Starting map save to ${destination} under the robot user's home directory ...`;
  try {
    await postJson("/api/mapping/save_map", {
      destination,
      save_ground_map: document.getElementById("saveMapGround").checked,
      save_tiled_map: document.getElementById("saveMapTiled").checked,
    });
    pollMappingState();
  } catch (error) {
    saveMapSubmissionError = error.message;
    document.getElementById("saveMapStatus").textContent = `Map save failed: ${saveMapSubmissionError}`;
  }
}


function recordingTimestamp(date = new Date()) {
  const pad = (value) => String(value).padStart(2, "0");
  return `mapping_${date.getFullYear()}${pad(date.getMonth() + 1)}${pad(date.getDate())}_${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}`;
}

function freshRecordingName(avoided = []) {
  const base = recordingTimestamp();
  const taken = new Set(avoided.filter(Boolean));
  if (!taken.has(base)) return base;
  let suffix = 2;
  while (taken.has(`${base}_${suffix}`)) suffix += 1;
  return `${base}_${suffix}`;
}

function updateRecordingStatus(recording) {
  state.recording = recording || null;
  const open = document.getElementById("openRecordBag");
  const stop = document.getElementById("stopRecording");
  const status = document.getElementById("recordingStatus");
  const output = document.getElementById("recordingOutput");
  if (!open || !stop || !status || !output) return;
  const current = recording && recording.state;
  const pending = state.recordingPending;
  const actionError = state.recordingActionError;
  open.disabled = pending || !recording || !recording.available || current === "recording" || current === "stopping";
  stop.disabled = pending || current !== "recording";
  status.classList.toggle("recording-status-error", Boolean(actionError) || current === "failed");
  output.textContent = recording && recording.output_dir ? `保存位置：${recording.output_dir}` : "";
  if (actionError) {
    status.textContent = actionError;
  } else if (!recording) {
    status.textContent = "无法获取录制状态。";
  } else if (current === "recording") {
    status.textContent = `正在录制 ${(recording.topics || []).length} 个话题。`;
  } else if (current === "stopping") {
    status.textContent = "正在停止并保存数据包，请稍候…";
  } else if (current === "stopped") {
    status.textContent = "录制完成，数据包已保存。";
  } else if (current === "failed") {
    status.textContent = `录制失败：${recording.message || "未知错误"}`;
  } else if (!recording.available) {
    status.textContent = "rosbag2 不可用，暂时无法录制。";
  } else {
    status.textContent = "当前未录制。";
  }
  if (current === "stopped" || current === "failed") {
    const logTail = (recording.log_tail || []).slice(-5).join(" | ");
    if (current === "failed" && logTail && !String(recording.message || "").includes(logTail)) {
      status.textContent += ` 日志：${logTail}`;
    }
  }
}

function renderRecordingTopics() {
  const container = document.getElementById("recordingTopics");
  if (!container) return;
  const checked = new Set(
    [...container.querySelectorAll("input[type=checkbox]:checked")].map((input) => input.value)
  );
  container.replaceChildren();
  const topics = state.recordingTopics || [];
  if (!topics.length) {
    const empty = document.createElement("p");
    empty.textContent = "当前没有可用话题；请先启动建图。";
    container.appendChild(empty);
  }
  for (const topic of topics) {
    const label = document.createElement("label");
    label.className = "recording-topic";
    const input = document.createElement("input");
    input.type = "checkbox";
    input.value = topic.name;
    input.checked = checked.has(topic.name);
    const name = document.createElement("span");
    name.textContent = topic.name;
    const types = document.createElement("small");
    types.textContent = (topic.types || []).join(", ");
    label.append(input, name, types);
    container.appendChild(label);
  }
  const error = document.getElementById("recordingTopicsError");
  if (error) error.textContent = state.recordingTopicsError || "";
}

async function refreshRecordingTopics() {
  const button = document.getElementById("refreshRecordingTopics");
  if (button) button.disabled = true;
  state.recordingTopicsError = null;
  try {
    const data = await getJson("/api/mapping/record/topics");
    state.recordingTopics = data.topics || [];
  } catch (error) {
    state.recordingTopicsError = `读取话题失败：${error.message}`;
  } finally {
    if (button) button.disabled = false;
    renderRecordingTopics();
  }
}

function renderRecordingBrowser() {
  const list = document.getElementById("recordingBrowserList");
  const path = document.getElementById("recordingBrowserPath");
  const up = document.getElementById("recordingBrowserUp");
  const use = document.getElementById("recordingBrowserUse");
  if (!list || !path || !up || !use) return;
  path.textContent = state.recordingBrowserPath || "";
  list.replaceChildren();
  for (const entry of state.recordingBrowserEntries || []) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "recording-browser-entry";
    button.textContent = entry.writable ? entry.name : `${entry.name}（不可写）`;
    button.addEventListener("click", () => loadRecordingDirectory(entry.path));
    list.appendChild(button);
  }
  if (!(state.recordingBrowserEntries || []).length) {
    const empty = document.createElement("p");
    empty.textContent = "没有子目录。";
    list.appendChild(empty);
  }
  up.disabled = !state.recordingBrowserParent;
  use.disabled = !state.recordingBrowserWritable;
}

async function loadRecordingDirectory(path, show = true) {
  const browser = document.getElementById("recordingBrowser");
  if (show && browser) browser.hidden = false;
  try {
    const data = await getJson(`/api/mapping/record/browse?path=${encodeURIComponent(path || "")}`);
    state.recordingBrowserPath = data.path || "";
    state.recordingBrowserParent = data.parent || null;
    state.recordingBrowserWritable = Boolean(data.writable);
    state.recordingBrowserEntries = data.entries || [];
    state.recordingDefaultDirectory = data.default_directory || data.path || "";
    renderRecordingBrowser();
  } catch (error) {
    state.recordingStartError = `读取目录失败：${error.message}`;
    const box = document.getElementById("recordingError");
    if (box) box.textContent = state.recordingStartError;
  }
}

async function openRecordBagDialog() {
  state.recordingStartError = null;
  state.recordingActionError = null;
  state.recordingTopicsError = null;
  const dialog = document.getElementById("recordBagDialog");
  const name = document.getElementById("recordingName");
  const directory = document.getElementById("recordingDirectory");
  const error = document.getElementById("recordingError");
  if (error) error.textContent = "";
  updateRecordingStatus(state.recording);
  renderRecordingTopics();
  if (name) {
    const previous = name.value.trim();
    const autoName = state.recordingAutoName;
    const usedName = state.recordingUsedName;
    if (!previous || previous === autoName || previous === usedName) {
      name.value = freshRecordingName([autoName, usedName]);
      state.recordingAutoName = name.value;
    }
  }
  if (dialog) dialog.showModal();
  await refreshRecordingTopics();
  if (directory && !directory.value) {
    await loadRecordingDirectory("", false);
    if (state.recordingDefaultDirectory) directory.value = state.recordingDefaultDirectory;
  }
  if (name) name.focus();
  if (name) name.select();
}

async function submitRecording(event) {
  event.preventDefault();
  const dialog = document.getElementById("recordBagDialog");
  const directoryInput = document.getElementById("recordingDirectory");
  const nameInput = document.getElementById("recordingName");
  const errorBox = document.getElementById("recordingError");
  const directory = directoryInput.value.trim();
  const name = nameInput.value.trim();
  const topics = [...document.querySelectorAll("#recordingTopics input[type=checkbox]:checked")].map((input) => input.value);
  let message = "";
  if (!topics.length) message = "请至少选择一个话题。";
  else if (!directory.startsWith("/") || directory.split("/").includes("..")) message = "请输入机器人上已存在的绝对目录，且不能包含 '..'。";
  else if (!/^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$/.test(name)) message = "文件夹名称只能包含字母、数字、点、下划线或短横线，且以字母或数字开头。";
  if (message) {
    errorBox.textContent = message;
    return;
  }
  errorBox.textContent = "";
  state.recordingStartError = null;
  state.recordingActionError = null;
  state.recordingPending = true;
  updateRecordingStatus(state.recording);
  const start = document.getElementById("startRecording");
  if (start) start.disabled = true;
  try {
    const data = await postJson("/api/mapping/record/start", { directory, name, topics });
    if (dialog) dialog.close();
    state.recordingUsedName = name;
    state.recordingActionError = null;
    updateRecordingStatus(data.recording);
    pollMappingState();
  } catch (error) {
    state.recordingStartError = error.message;
    errorBox.textContent = `开始录制失败：${error.message}`;
  } finally {
    state.recordingPending = false;
    if (start) start.disabled = false;
    updateRecordingStatus(state.recording);
  }
}

async function stopRecording() {
  state.recordingActionError = null;
  state.recordingPending = true;
  updateRecordingStatus(state.recording);
  const status = document.getElementById("recordingStatus");
  if (status) status.textContent = "正在停止并保存数据包，请稍候…";
  try {
    const data = await postJson("/api/mapping/record/stop", {});
    state.recordingActionError = null;
    updateRecordingStatus(data.recording);
  } catch (error) {
    state.recordingActionError = `停止录制失败：${error.message}`;
  } finally {
    state.recordingPending = false;
    updateRecordingStatus(state.recording);
    pollMappingState();
  }
}

function sourceRow(name, summary) {
  const row = document.createElement("div");
  row.className = "source";
  const label = document.createElement("div");
  label.className = "name";
  label.textContent = String(name).replace(/^\/+/, "");
  const pill = document.createElement("div");
  pill.className = `pill ${summary.status}`;
  pill.textContent = summary.status;
  const detail = document.createElement("div");
  detail.className = "detail";
  const age = summary.age == null ? "--" : `${fmt(summary.age, 1)}s`;
  detail.textContent = `frame ${summary.frame_id || "--"} | age ${age} | count ${summary.count || 0}`;
  row.append(label, pill, detail);
  return row;
}

function updateNavigationReadiness(readiness) {
  const element = document.getElementById("navigationReady");
  const ready = readiness && readiness.ready;
  element.textContent = readiness && readiness.message ? readiness.message : "waiting for relocalization";
  element.className = `readiness ${ready ? "ready" : "waiting"}`;
}

// --- Map selector ---------------------------------------------------------
//
// The catalog is fetched on demand (page load, Refresh, after a successful
// Save Map, or after a rejected selection) instead of on the fast poll loop.
// Process-running state used to lock selection comes from the existing
// terminal-log polling so the panel never issues its own aggressive GET.

const MAP_MODE_LABELS = {
  auto: "自动（有分块优先）",
  tiled: "分块",
  global: "全局",
};
let mapSelectionRequestSeq = 0;
// Catalog refreshes have their own generation so an older GET can neither
// overwrite a newer refresh nor data saved by a selection mutation.
let mapCatalogRefreshSeq = 0;

function mapModeLabel(mode) {
  return MAP_MODE_LABELS[mode] || "--";
}

function mapSelectionLockReason() {
  const running = [];
  if (state.localizationRunning) running.push("localization");
  if (state.navigationRunning) running.push("navigation");
  if (!running.length) return null;
  return `Stop ${running.join(" and ")} before changing the map, loading mode, or automatic relocalization.`;
}

function updateAutoRelocalizationControl() {
  const button = document.getElementById("autoRelocalizationToggle");
  if (!button) return;
  const payload = state.mapCatalog;
  const selection = payload && payload.selection ? payload.selection : null;
  const available = Boolean(
    selection &&
    selection.valid &&
    selection.auto_relocalization_available
  );
  const requested = Boolean(
    selection && selection.auto_relocalization
  );
  const effective = Boolean(available && requested);
  button.classList.toggle("start-log", !effective);
  button.classList.toggle("stop-log", effective);
  button.setAttribute("aria-pressed", String(effective));
  button.setAttribute("aria-busy", String(state.mapSelectPending));
  if (!available) {
    button.textContent = "自动重定位不可用";
    button.title = selection && selection.name
      ? (selection.auto_relocalization_message ||
        "所选地图缺少 sc_db.bin，无法自动重定位")
      : "地图状态未加载";
    button.disabled = true;
    return;
  }
  button.textContent = effective ? "关闭自动重定位" : "开启自动重定位";
  button.title = effective
    ? "关闭后需手动设置初始位姿"
    : "开启后在启动定位时自动估计初始位姿";
  button.disabled =
    mapSelectionLockReason() !== null || state.mapSelectPending;
}

function updateMapSelectionLock() {
  const select = document.getElementById("mapSelect");
  const modeSelect = document.getElementById("mapMode");
  const payload = state.mapCatalog;
  const maps = payload && Array.isArray(payload.maps) ? payload.maps : [];
  const selection = payload && payload.selection ? payload.selection : null;
  const reason = mapSelectionLockReason();
  if (select) {
    select.disabled = reason !== null || maps.length === 0 || state.mapSelectPending;
  }
  if (modeSelect) {
    // A mode only describes an already valid selection.  A persisted manual
    // tiled preference that no longer resolves is still recoverable here: the
    // map itself remains valid, so the user can choose Auto/Global.
    modeSelect.disabled =
      reason !== null ||
      state.mapSelectPending ||
      !selection ||
      !selection.name ||
      !selection.valid;
  }
  updateAutoRelocalizationControl();
  const note = document.getElementById("mapLockNote");
  if (note) {
    note.hidden = reason === null;
    note.textContent = reason || "";
  }
}

function updateMapPanel() {
  const select = document.getElementById("mapSelect");
  const modeSelect = document.getElementById("mapMode");
  const warning = document.getElementById("mapWarning");
  const status = document.getElementById("mapStatus");
  if (!select || !modeSelect || !warning || !status) return;

  const payload = state.mapCatalog;
  const maps = payload && Array.isArray(payload.maps) ? payload.maps : [];
  const selection = payload && payload.selection ? payload.selection : null;
  const selectedEntry = selection && selection.valid && selection.entry ? selection.entry : null;

  select.innerHTML = "";
  if (maps.length) {
    const placeholder = document.createElement("option");
    placeholder.value = "";
    placeholder.textContent = "Select a map...";
    select.appendChild(placeholder);
    maps.forEach((entry) => {
      const option = document.createElement("option");
      option.value = entry.name;
      option.textContent = entry.warning ? `${entry.name} (tile warning)` : entry.name;
      select.appendChild(option);
    });
  } else {
    const empty = document.createElement("option");
    empty.value = "";
    empty.textContent = "no maps found";
    select.appendChild(empty);
  }
  const names = new Set(maps.map((entry) => entry.name));
  if (selection && selection.name && !names.has(selection.name)) {
    const stale = document.createElement("option");
    stale.value = selection.name;
    stale.textContent = `${selection.name} (unavailable)`;
    select.appendChild(stale);
  }
  select.value = selection && selection.name ? selection.name : "";

  const warningText = selectedEntry && selectedEntry.warning ? `Tile warning: ${selectedEntry.warning}` : "";
  warning.hidden = warningText === "";
  warning.textContent = warningText;

  const savedMode = selection && selection.mode ? selection.mode : "auto";
  modeSelect.value = savedMode;

  let statusMessage = "";
  let statusIsError = false;
  if (state.mapSelectionError) {
    statusMessage = `Map selection failed: ${state.mapSelectionError}`;
    statusIsError = true;
  } else if (state.mapError) {
    statusMessage = `Map list unavailable: ${state.mapError}`;
    statusIsError = true;
  } else if (state.mapSelectionNotice) {
    statusMessage = state.mapSelectionNotice;
  } else if (selection && selection.name && !selection.valid) {
    statusMessage = `Selected map '${selection.name}' is unavailable: ${selection.message || "choose another map"}.`;
    statusIsError = true;
  } else if (selection && selection.name && selection.mode_valid === false) {
    statusMessage = `Selected map '${selection.name}' has an unavailable loading mode: ${selection.mode_message || "choose Auto or Global"}.`;
    statusIsError = true;
  }
  status.textContent = statusMessage;
  status.hidden = statusMessage === "";
  status.className = statusIsError ? "result error" : "result";

  updateMapSelectionLock();
}

async function refreshMaps(clearMessages = false) {
  const refreshId = ++mapCatalogRefreshSeq;
  const mutationId = mapSelectionRequestSeq;
  const mutationWasPending = state.mapSelectPending;
  if (clearMessages) {
    state.mapSelectionError = null;
    state.mapSelectionNotice = null;
  }
  // A GET issued during (or before) a selection mutation may carry the old
  // saved mode.  It must never overwrite the mutation response or a later
  // refresh, and a superseded GET failure must not clear a newer catalog.
  const isCurrent = () =>
    refreshId === mapCatalogRefreshSeq &&
    mutationId === mapSelectionRequestSeq &&
    !mutationWasPending &&
    !state.mapSelectPending;
  try {
    const data = await getJson("/api/maps");
    if (!isCurrent()) return;
    state.mapCatalog = data;
    state.mapError = null;
  } catch (error) {
    if (!isCurrent()) return;
    state.mapCatalog = null;
    state.mapError = error.message;
  }
  updateMapPanel();
}

async function selectMap(name) {
  const status = document.getElementById("mapStatus");
  if (!name) {
    updateMapPanel();
    return;
  }
  const requestId = ++mapSelectionRequestSeq;
  state.mapSelectionError = null;
  state.mapSelectionNotice = `Selecting map '${name}'...`;
  state.mapSelectPending = true;
  if (status) {
    status.textContent = state.mapSelectionNotice;
    status.className = "result";
    status.hidden = false;
  }
  updateMapSelectionLock();
  try {
    const data = await postJson("/api/maps/select", { name });
    if (requestId !== mapSelectionRequestSeq) return;
    state.mapSelectPending = false;
    if (state.mapCatalog) state.mapCatalog.selection = data.selection;
    state.mapSelectionNotice = null;
    updateMapPanel();
    invalidateNavigationMapCaches();
    void refreshController.tick();
  } catch (error) {
    if (requestId !== mapSelectionRequestSeq) return;
    state.mapSelectPending = false;
    state.mapSelectionError = error.message;
    state.mapSelectionNotice = null;
    // The server is authoritative; resync in case the UI state was stale.
    await refreshMaps();
  }
}

async function selectMapMode(mode) {
  const modeSelect = document.getElementById("mapMode");
  const status = document.getElementById("mapStatus");
  const selection = state.mapCatalog && state.mapCatalog.selection;
  const name = selection && selection.name;
  if (
    !name ||
    !selection.valid ||
    !Object.prototype.hasOwnProperty.call(MAP_MODE_LABELS, mode) ||
    (modeSelect && modeSelect.disabled)
  ) {
    updateMapPanel();
    return;
  }
  const requestId = ++mapSelectionRequestSeq;
  state.mapSelectionError = null;
  state.mapSelectionNotice = `Applying loading mode '${mapModeLabel(mode)}'...`;
  state.mapSelectPending = true;
  if (modeSelect) modeSelect.value = mode;
  if (status) {
    status.textContent = state.mapSelectionNotice;
    status.className = "result";
    status.hidden = false;
  }
  updateMapSelectionLock();
  try {
    const data = await postJson("/api/maps/select", { name, mode });
    if (requestId !== mapSelectionRequestSeq) return;
    state.mapSelectPending = false;
    if (state.mapCatalog) state.mapCatalog.selection = data.selection;
    state.mapSelectionNotice = null;
    updateMapPanel();
    invalidateNavigationMapCaches();
    void refreshController.tick();
  } catch (error) {
    if (requestId !== mapSelectionRequestSeq) return;
    state.mapSelectPending = false;
    state.mapSelectionError = error.message;
    state.mapSelectionNotice = null;
    // Never silently change the preference locally: restore the server's
    // persisted selection and mode after a rejected request.
    await refreshMaps();
  }
}

async function toggleAutoRelocalization() {
  const button = document.getElementById("autoRelocalizationToggle");
  const status = document.getElementById("mapStatus");
  const selection = state.mapCatalog && state.mapCatalog.selection;
  if (
    !selection ||
    !selection.valid ||
    !selection.auto_relocalization_available ||
    (button && button.disabled)
  ) {
    updateMapPanel();
    return;
  }
  const next = !(selection.effective_auto_relocalization === true);
  const requestId = ++mapSelectionRequestSeq;
  state.mapSelectionError = null;
  state.mapSelectionNotice = next
    ? "Enabling automatic relocalization..."
    : "Disabling automatic relocalization...";
  state.mapSelectPending = true;
  if (status) {
    status.textContent = state.mapSelectionNotice;
    status.className = "result";
    status.hidden = false;
  }
  updateMapSelectionLock();
  try {
    const data = await postJson("/api/maps/select", {
      auto_relocalization: next,
    });
    if (requestId !== mapSelectionRequestSeq) return;
    state.mapSelectPending = false;
    if (state.mapCatalog) state.mapCatalog.selection = data.selection;
    state.mapSelectionNotice = null;
    updateMapPanel();
    // A flag-only change does not alter the loaded geometry, so unlike a map
    // name/mode change it must not invalidate the cached map clouds.
  } catch (error) {
    if (requestId !== mapSelectionRequestSeq) return;
    state.mapSelectPending = false;
    state.mapSelectionError = error.message;
    state.mapSelectionNotice = null;
    // The server is authoritative; restore the last persisted flag instead of
    // leaving a failed toggle visually flipped.
    await refreshMaps();
  }
}

// Per-target cursor: the max line seq we have already rendered. The server
// only returns lines with seq > this, so polling is incremental. 0 means
// "nothing yet" and makes the server return the current full buffer, which we
// replace into the element (first render / reconnect).
const lastLogSeq = { chassis: 0, imu: 0, lidar: 0, localization: 0, navigation: 0, mapping: 0 };

function updateTerminalLog(target, summary) {
  const stateElements = document.querySelectorAll(`[data-log-state="${target}"]`);
  const outputElements = Array.from(
    document.querySelectorAll(`[data-log-output="${target}"]`),
  );
  const setState = (text, className) => {
    stateElements.forEach((element) => {
      element.textContent = text;
      element.className = className;
    });
  };
  if (!summary) {
    setState("error", "pill error");
    return;
  }
  setManagedProcessRunning(target, Boolean(summary.running));
  if (target === "localization" || target === "navigation") {
    updateMapSelectionLock();
  }
  updateManagedProcessButton(target);
  const lines = Array.isArray(summary.lines) ? summary.lines : [];
  const seq = typeof summary.seq === "number" ? summary.seq : undefined;
  const isInitial = lastLogSeq[target] === 0;

  // A full textContent rewrite would wipe an active text selection in a log;
  // only that case needs to defer. Incremental append adds a text node at the
  // end and does not perturb offsets in existing text, so a selection there
  // survives even while new lines keep arriving — no need to freeze. The same
  // target can be mounted on both the Navigation and Mapping pages, so every
  // matching output element is updated.
  const selection = window.getSelection();
  const selectingHere =
    selection &&
    selection.rangeCount > 0 &&
    selection.toString().length > 0 &&
    outputElements.some((element) => element.contains(selection.anchorNode));

  if (seq === undefined || isInitial) {
    // First render, reconnect, or legacy server without seq: replace contents.
    if (!selectingHere) {
      outputElements.forEach((outputElement) => {
        const atBottom =
          outputElement.scrollTop + outputElement.clientHeight >= outputElement.scrollHeight - 8;
        outputElement.textContent = lines.length ? lines.join("\n") : "No output.";
        if (atBottom) outputElement.scrollTop = outputElement.scrollHeight;
      });
      if (seq !== undefined) lastLogSeq[target] = seq;
    }
  } else if (lines.length) {
    // Incremental: append only the new lines, even while the user is selecting
    // — appending a text node preserves any selection in earlier text.
    outputElements.forEach((outputElement) => {
      const atBottom =
        outputElement.scrollTop + outputElement.clientHeight >= outputElement.scrollHeight - 8;
      outputElement.appendChild(document.createTextNode("\n" + lines.join("\n")));
      if (atBottom) outputElement.scrollTop = outputElement.scrollHeight;
    });
    lastLogSeq[target] = seq;
  } else if (seq !== undefined) {
    lastLogSeq[target] = seq;
  }
  if (!summary.enabled) {
    setState("disabled", "pill waiting");
  } else if (summary.running) {
    setState("running", "pill active");
  } else if (summary.returncode === 0) {
    setState("exited", "pill loaded");
  } else if (summary.returncode == null) {
    setState("waiting", "pill waiting");
  } else {
    setState(`error ${summary.returncode}`, "pill error");
  }
}

const MANAGED_PROCESS_LABELS = {
  chassis: "hunter",
  imu: "IMU",
  lidar: "Lidar",
  localization: "Localization",
  navigation: "Navigation",
  mapping: "Mapping",
  diagnostics: "话题与运动日志",
};

function setManagedProcessRunning(target, running) {
  const previous = managedProcessRunning(target);
  if (target === "localization") state.localizationRunning = running;
  else if (target === "navigation") state.navigationRunning = running;
  else if (target === "chassis") state.chassisRunning = running;
  else if (target === "imu") state.imuRunning = running;
  else if (target === "lidar") state.lidarRunning = running;
  else if (target === "mapping") state.mappingRunning = running;
  if (previous === running) return;
  if (target === "localization") {
    if (running) invalidateNavigationMapCaches();
    else clearMapSceneState();
  } else if (target === "navigation" && !running) {
    clearNavigationSceneState();
    clearNavigationLayers();
  }
}

function managedProcessRunning(target) {
  if (target === "localization") return state.localizationRunning;
  if (target === "navigation") return state.navigationRunning;
  if (target === "chassis") return state.chassisRunning;
  if (target === "imu") return state.imuRunning;
  if (target === "lidar") return state.lidarRunning;
  if (target === "mapping") return state.mappingRunning;
  return false;
}

function managedProcessButtons(target) {
  return Array.from(document.querySelectorAll(`[data-managed-target="${target}"]`));
}

function updateManagedProcessButton(target) {
  const running = managedProcessRunning(target);
  const pending = state.managedProcessPending[target];
  const label = MANAGED_PROCESS_LABELS[target] || target;
  const action = running ? "Stop" : "Start";
  const text = pending
    ? `${pending === "stop" ? "Stopping" : "Starting"} ${label}...`
    : `${action} ${label}`;
  // Hardware toggles are mounted on both pages and share one backend process,
  // so every matching button reflects the same running/pending state.
  managedProcessButtons(target).forEach((button) => {
    button.textContent = text;
    button.disabled = Boolean(pending);
    button.classList.toggle("start-log", !running);
    button.classList.toggle("stop-log", running);
    button.setAttribute("aria-pressed", String(running));
    button.setAttribute("aria-busy", String(Boolean(pending)));
  });
}

async function toggleManagedTarget(target) {
  if (state.managedProcessPending[target]) return;
  const running = managedProcessRunning(target);
  const nextRunning = !running;
  state.managedProcessPending[target] = running ? "stop" : "start";
  updateManagedProcessButton(target);
  try {
    const succeeded = running ? await stopManagedTarget(target) : await startManagedTarget(target);
    if (succeeded) {
      setManagedProcessRunning(target, nextRunning);
      if (target === "localization" || target === "navigation") {
        updateMapSelectionLock();
      }
    }
  } finally {
    state.managedProcessPending[target] = null;
    updateManagedProcessButton(target);
  }
}

function updateMeter(id, value) {
  const element = document.getElementById(id);
  if (!element) return;
  const percent = Number.isFinite(value) ? Math.max(0, Math.min(100, value)) : 0;
  element.style.width = `${percent}%`;
}

function updateStatusBadge(id, value) {
  const element = document.getElementById(id);
  if (!element) return;
  const level = usageLevel(value);
  element.className = `system-card-status load-${level}`;
  element.textContent = level === "waiting" ? "waiting" : level;
}

function createCpuCoreRow(core) {
  const row = document.createElement("div");
  row.className = "cpu-core-row";
  row.dataset.coreId = String(core.id);
  const label = document.createElement("span");
  label.className = "cpu-core-label";
  label.textContent = `cpu${core.id}`;
  const track = document.createElement("div");
  track.className = "cpu-core-track";
  const fill = document.createElement("div");
  fill.className = "cpu-core-fill";
  track.appendChild(fill);
  const value = document.createElement("span");
  value.className = "cpu-core-value";
  row.append(label, track, value);
  return row;
}

function updateCpuCores(cores) {
  const list = document.getElementById("systemCpuCores");
  const activeIds = new Set(cores.map((core) => String(core.id)));
  list.querySelectorAll(".cpu-core-row").forEach((row) => {
    if (!activeIds.has(row.dataset.coreId)) row.remove();
  });
  const empty = list.querySelector(".cpu-core-empty");
  if (empty && cores.length) empty.remove();
  if (!cores.length) {
    if (!empty) {
      const nextEmpty = document.createElement("div");
      nextEmpty.className = "cpu-core-empty";
      nextEmpty.textContent = "waiting for CPU sample";
      list.appendChild(nextEmpty);
    }
    return;
  }
  cores.forEach((core) => {
    const percent = Number.isFinite(core.percent) ? Math.max(0, Math.min(100, core.percent)) : 0;
    let row = list.querySelector(`[data-core-id="${core.id}"]`);
    if (!row) {
      row = createCpuCoreRow(core);
      list.appendChild(row);
    }
    row.className = `cpu-core-row load-${usageLevel(core.percent)}`;
    row.style.setProperty("--cpu-percent", `${percent}%`);
    const value = row.querySelector(".cpu-core-value");
    if (value) value.textContent = fmtPercent(core.percent);
  });
}

function updateSystem(data) {
  const cpuTotal = data && data.cpu ? data.cpu.total_percent : null;
  document.getElementById("systemCpuTotal").textContent = fmtPercent(cpuTotal);
  updateMeter("systemCpuMeter", cpuTotal);
  updateStatusBadge("systemCpuStatus", cpuTotal);
  const memory = data && data.memory ? data.memory : null;
  const swap = data && data.swap ? data.swap : null;
  document.getElementById("systemMemory").textContent = memory ? `${fmtPercent(memory.percent)} | ${fmtStorage(memory.used_kb)} / ${fmtStorage(memory.total_kb)}` : "--";
  document.getElementById("systemSwap").textContent = swap ? `${fmtPercent(swap.percent)} | ${fmtStorage(swap.used_kb)} / ${fmtStorage(swap.total_kb)}` : "--";
  updateMeter("systemMemoryMeter", memory ? memory.percent : null);
  updateMeter("systemSwapMeter", swap ? swap.percent : null);
  updateStatusBadge("systemMemoryStatus", memory ? memory.percent : null);
  updateStatusBadge("systemSwapStatus", swap ? swap.percent : null);
  updateCpuCores(data && data.cpu && data.cpu.cores ? data.cpu.cores : []);
}

function updateSelectionDisplay() {
  const s = state.selection;
  document.getElementById("selX").textContent = s ? fmt(s.x) : "--";
  document.getElementById("selY").textContent = s ? fmt(s.y) : "--";
  document.getElementById("selZ").textContent = s ? fmt(s.z || 0) : "--";
  document.getElementById("selYaw").textContent = s ? fmt(s.yaw, 3) : "--";
}

function updateTargetPoseDisplay() {
  const goal = state.activeGoal || state.lastGoal;
  document.getElementById("goalX").textContent = goal ? fmt(goal.x) : "--";
  document.getElementById("goalY").textContent = goal ? fmt(goal.y) : "--";
  document.getElementById("goalZ").textContent = goal ? fmt(goal.z || 0) : "--";
  document.getElementById("goalYaw").textContent = goal ? fmt(goal.yaw, 3) : "--";
}

// --- Navigation pause/resume controls
function updateNavigationControls(control) {
  const previous = state.navigationControl;
  state.navigationControl = control || null;
  const primary = document.getElementById("navigationPrimary");
  const clear = document.getElementById("navigationClear");
  const status = document.getElementById("navigationControlState");
  const value = control || {
    state: "idle",
    has_goal: false,
    can_pause: false,
    can_resume: false,
    can_clear: false,
    paused_goal: null,
  };
  const labels = {
    idle: "Idle",
    navigating: "Navigating",
    pausing: "Pausing",
    paused: "Paused",
    resuming: "Resuming",
    clearing: "Clearing",
  };
  status.textContent = labels[value.state] || "Idle";
  if (value.state === "pausing") {
    primary.textContent = "Pausing...";
    primary.disabled = true;
  } else if (value.state === "resuming") {
    primary.textContent = "Resuming...";
    primary.disabled = true;
  } else if (value.state === "paused") {
    primary.textContent = "Resume Navigation";
    primary.disabled = !value.can_resume;
  } else {
    primary.textContent = "Pause Navigation";
    primary.disabled = !value.can_pause;
  }
  clear.textContent = value.state === "clearing" ? "Clearing..." : "Clear Goal";
  clear.disabled = !value.can_clear;
  if (value.paused_goal) {
    state.lastGoal = { ...value.paused_goal };
    updateTargetPoseDisplay();
    updateTargetMarkers();
  }
  if (!value.has_goal && previous && previous.has_goal && !value.paused_goal) {
    clearNavigationSceneState();
  }
  updatePublishPoseButton();
}

async function postNavigationCommand(path, failureLabel) {
  const result = document.getElementById("publishResult");
  try {
    const response = await postJson(path, {});
    if (response.navigation_control) {
      updateNavigationControls(response.navigation_control);
    }
    result.textContent = `${response.message} @ ${new Date(response.stamp * 1000).toLocaleTimeString()}`;
    pollState();
  } catch (error) {
    // The server state stays authoritative; the periodic poll keeps rendering
    // the transition even after a timeout or conflict.
    result.textContent = `${failureLabel}: ${error.message}`;
    pollState();
  }
}
// --- End navigation pause/resume controls

// --- Mapping joystick
const JOYSTICK_SEND_INTERVAL_MS = 100;
const JOYSTICK_DEADZONE = 0.08;
const STICK_ROLES = ["throttle", "steering"];

const teleop = {
  clientId: viewerSessionId(),
  seq: 0,
  stopRequired: false,
  active: false,
  throttle: 0,
  steering: 0,
  lastSentAt: -Infinity,
  sendTimer: null,
  layoutMode: false,
  pointers: { throttle: null, steering: null },
};

// Normalize one stick offset to its single axis.  The throttle stick is
// vertical only (up is forward) and the steering stick is horizontal only
// (left is positive).  A displacement inside the dead zone is zero.
function stickAxisValue(role, offsetX, offsetY, radius) {
  if (!Number.isFinite(radius) || radius <= 0) return 0;
  const clamp = (value) => Math.max(-1, Math.min(1, value));
  const deadzone = (value) => (Math.abs(value) < JOYSTICK_DEADZONE ? 0 : value);
  if (role === "throttle") return deadzone(clamp(-offsetY / radius));
  return deadzone(clamp(-offsetX / radius));
}

// Keep each knob on its locked axis: x fixed for throttle, y fixed for steering.
function stickKnobOffset(role, offsetX, offsetY) {
  if (role === "throttle") return { dx: 0, dy: offsetY };
  return { dx: offsetX, dy: 0 };
}

// Read the pointer's displacement from the base center and project it onto the
// role's single axis.  Each stick is one-dimensional: the throttle reads only
// vertical travel and the steering reads only horizontal travel.  Discarding
// the perpendicular component (instead of radially clamping the 2D vector)
// keeps a diagonal gesture at full commanded magnitude.
function stickOffset(event, base, role) {
  const rect = base.getBoundingClientRect();
  const radius = Math.max(1, Math.min(rect.width, rect.height) / 2);
  const clamp = (value) => Math.max(-radius, Math.min(radius, value));
  const dx = event.clientX - (rect.left + rect.width / 2);
  const dy = event.clientY - (rect.top + rect.height / 2);
  if (role === "throttle") return { dx: 0, dy: clamp(dy), radius };
  return { dx: clamp(dx), dy: 0, radius };
}

function stickBaseElement(role) {
  return document.getElementById(role === "throttle" ? "throttleBase" : "steeringBase");
}

function stickKnobElement(role) {
  return document.getElementById(role === "throttle" ? "throttleKnob" : "steeringKnob");
}

function renderStickKnob(role, dx, dy) {
  const knob = stickKnobElement(role);
  if (!knob) return;
  const offset = stickKnobOffset(role, dx, dy);
  knob.style.transform = `translate(${offset.dx}px, ${offset.dy}px)`;
}

function anyTeleopAxis() {
  return teleop.throttle !== 0 || teleop.steering !== 0;
}

// A stick pointer can stay captured while its axis is centered (zero).  Loop
// lifetime tracks the pointer, not the axis value, so releasing one stick must
// not tear down the shared keepalive for a still-held centered stick.
function anyTeleopPointer() {
  return teleop.pointers.throttle !== null || teleop.pointers.steering !== null;
}

function resetStickAxis(role) {
  teleop.pointers[role] = null;
  teleop[role] = 0;
  renderStickKnob(role, 0, 0);
}

function resetTeleopAxes() {
  for (const role of STICK_ROLES) resetStickAxis(role);
}

// Teleop errors surface through the existing accessible settings error surface.
// That surface lives in the layout block, so guard the lookup: teleop safety
// must never depend on the UI being present.
function surfaceJoystickError(message) {
  try {
    joystickLayout.saveError = message;
    renderJoystickSettingsError();
  } catch (_error) {
    // No error surface in this scope; the watchdog still stops motion.
  }
}

function startTeleopSendLoop() {
  if (teleop.sendTimer !== null) return;
  teleop.sendTimer = window.setInterval(async () => {
    if (!teleop.active || !anyTeleopAxis()) return;
    await sendTeleopCommand(teleop.throttle, teleop.steering);
  }, JOYSTICK_SEND_INTERVAL_MS);
}

function stopTeleopSendLoop() {
  if (teleop.sendTimer === null) return;
  window.clearInterval(teleop.sendTimer);
  teleop.sendTimer = null;
}

async function sendTeleopCommand(throttle, steering, force = false) {
  const now = Date.now();
  if (!force && now - teleop.lastSentAt < JOYSTICK_SEND_INTERVAL_MS) return false;
  teleop.lastSentAt = now;
  const isStop = throttle === 0 && steering === 0;
  // Set the stop obligation before the request can reach the server so a
  // failure can never leave an un-stopped command behind.
  if (!isStop) teleop.stopRequired = true;
  teleop.seq += 1;
  const seq = teleop.seq;
  try {
    const response = await postJson("/api/teleop", {
      throttle,
      steering,
      client_id: teleop.clientId,
      seq,
    });
    if (response && response.stale) return true;
    // Concurrent requests can complete out of order.  Once a newer command has
    // been sent, an older accepted response (for example the combined stop sent
    // when one stick was released) must not overwrite active/stopRequired and
    // tear down the keepalive for a still-held stick.
    if (seq !== teleop.seq) return true;
    if (isStop) teleop.stopRequired = false;
    teleop.active = !isStop;
    return true;
  } catch (error) {
    // A newer command may already own the sticks: this failing request can be
    // an older in-flight request that was superseded before it rejected (for
    // example an active command overtaken by a newer combined one).  Its
    // failure must not tear down the live keepalive, recenter a still-held
    // stick, or surface a stop; only the latest seq runs the global safety
    // stop.  Resolve like the stale accepted response so callers can ignore it.
    if (seq !== teleop.seq) return true;
    stopTeleopSendLoop();
    resetTeleopAxes();
    teleop.active = false;
    teleop.lastSentAt = -Infinity;
    surfaceJoystickError(`drive stopped: ${error.message}`);
    return false;
  }
}

async function stopTeleop(reason, force = false) {
  const wasActive =
    teleop.active ||
    teleop.pointers.throttle !== null ||
    teleop.pointers.steering !== null ||
    teleop.stopRequired;
  stopTeleopSendLoop();
  resetTeleopAxes();
  teleop.active = false;
  teleop.lastSentAt = -Infinity;
  if (!wasActive && !force) return;
  await sendTeleopCommand(0, 0, true);
}

// Release one stick: zero only that axis and immediately send the combined
// remaining state.  The loop stays installed while any stick pointer remains
// down, even a centered one, so its next movement keeps the 10Hz keepalive.
// Releasing the last pointer is an all-stop.
async function stopStickAxis(role) {
  if (teleop.pointers[role] === null) return;
  resetStickAxis(role);
  if (anyTeleopPointer()) {
    startTeleopSendLoop();
    await sendTeleopCommand(teleop.throttle, teleop.steering, true);
    return;
  }
  stopTeleopSendLoop();
  teleop.active = false;
  teleop.lastSentAt = -Infinity;
  await sendTeleopCommand(0, 0, true);
}

function sendTeleopKeepaliveStop() {
  teleop.seq += 1;
  const body = JSON.stringify({
    throttle: 0,
    steering: 0,
    client_id: teleop.clientId,
    seq: teleop.seq,
  });
  if (navigator && typeof navigator.sendBeacon === "function") {
    navigator.sendBeacon("/api/teleop", body);
    return;
  }
  fetchWithTimeout("/api/teleop", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body,
    keepalive: true,
  }).catch(() => {});
}

async function stickPointerDown(role, event) {
  if (teleop.layoutMode) return;
  if (teleop.pointers[role] !== null) return;
  event.preventDefault();
  event.stopPropagation();
  teleop.pointers[role] = event.pointerId;
  teleop.active = true;
  startTeleopSendLoop();
  // No element pointer capture: the driving pointers are routed at the window
  // by pointerId, so releasing one contact can never retarget or cancel the
  // capture of the other.  Geometry comes from the role's own base rather than
  // event.currentTarget, which is the window once a move is routed there.
  const base = stickBaseElement(role);
  const offset = stickOffset(event, base, role);
  teleop[role] = stickAxisValue(role, offset.dx, offset.dy, offset.radius);
  renderStickKnob(role, offset.dx, offset.dy);
  await sendTeleopCommand(teleop.throttle, teleop.steering, true);
}

async function stickPointerMove(role, event) {
  if (event.pointerId !== teleop.pointers[role]) return;
  event.preventDefault();
  event.stopPropagation();
  const offset = stickOffset(event, stickBaseElement(role), role);
  const previousAxis = teleop[role];
  teleop[role] = stickAxisValue(role, offset.dx, offset.dy, offset.radius);
  renderStickKnob(role, offset.dx, offset.dy);
  // Send immediately whenever the changed axis crosses the dead zone so a
  // center <-> active transition is never delayed by the send throttle.
  const crossing = (previousAxis === 0) !== (teleop[role] === 0);
  await sendTeleopCommand(teleop.throttle, teleop.steering, crossing);
}

async function stickPointerUp(role, event) {
  const ownsPointer = event.pointerId === teleop.pointers[role];
  if (!ownsPointer) {
    // A failed send clears the captured pointers but keeps the stop
    // obligation; the next release with no live axis still delivers the zero.
    if (!teleop.stopRequired || anyTeleopPointer()) return;
    event.preventDefault();
    event.stopPropagation();
    await stopTeleop("release", true);
    return;
  }
  event.preventDefault();
  event.stopPropagation();
  await stopStickAxis(role);
}

async function stickPointerCancel(role, event) {
  const ownsPointer = event.pointerId === teleop.pointers[role];
  if (!ownsPointer) {
    if (!teleop.stopRequired || anyTeleopPointer()) return;
    event.preventDefault();
    event.stopPropagation();
    await stopTeleop("cancel", true);
    return;
  }
  event.preventDefault();
  event.stopPropagation();
  await stopStickAxis(role);
}

// Pointer ownership is keyed by pointerId for the whole page.  Move/up/cancel
// are observed once at the window and dispatched to the single role that owns
// the event's pointerId, so a terminal event for one contact can only ever
// release that contact.  No element pointer capture is used: each role's base
// only needs its pointerdown, and touch keeps its implicit capture so its move
// events keep bubbling through the window even when the finger leaves the base.
function stickRoleForPointer(pointerId) {
  for (const role of STICK_ROLES) {
    if (teleop.pointers[role] === pointerId) return role;
  }
  return null;
}

function handleStickPointerMove(event) {
  const role = stickRoleForPointer(event.pointerId);
  if (!role) return undefined;
  return stickPointerMove(role, event);
}

function handleStickPointerUp(event) {
  return routeStickTerminal("release", event);
}

function handleStickPointerCancel(event) {
  return routeStickTerminal("cancel", event);
}

// A real terminal event for a held pointer stops that stick for safety; only
// events for pointers nobody owns fall through to the stop-obligation check.
function routeStickTerminal(reason, event) {
  const role = stickRoleForPointer(event.pointerId);
  if (role) {
    return reason === "cancel" ? stickPointerCancel(role, event) : stickPointerUp(role, event);
  }
  // A failed send clears the captured pointers but keeps the stop obligation,
  // so the next terminal event for a now-unowned pointer must still deliver the
  // zero.  It may only do so when no live pointer remains: a duplicate or
  // cross-routed terminal event for one contact must never stop motion while
  // another contact is still down.
  if (!teleop.stopRequired || anyTeleopPointer()) return undefined;
  event.preventDefault();
  event.stopPropagation();
  return stopTeleop(reason, true);
}

// Wire the driving sticks: pointerdown on each base, everything afterwards at
// the window.  lostpointercapture is deliberately not observed; without capture
// it is not needed, and treating it as a cancel is what let one contact's
// capture-topology change recenter the other.
function attachStickPointerRouting() {
  for (const role of STICK_ROLES) {
    const base = stickBaseElement(role);
    if (!base || typeof base.addEventListener !== "function") continue;
    base.addEventListener("pointerdown", (event) => stickPointerDown(role, event));
    base.addEventListener("contextmenu", (event) => event.preventDefault());
  }
  window.addEventListener("pointermove", handleStickPointerMove, { passive: false });
  window.addEventListener("pointerup", handleStickPointerUp);
  window.addEventListener("pointercancel", handleStickPointerCancel);
}

async function handleJoystickBlur() {
  await stopTeleop("blur");
}

async function handleVisibilityChange() {
  if (document.hidden) {
    await stopTeleop("hidden");
    return;
  }
  noteLayoutResize();
}

function handlePageHide() {
  if (
    !teleop.active &&
    teleop.pointers.throttle === null &&
    teleop.pointers.steering === null &&
    !teleop.stopRequired
  ) {
    return;
  }
  stopTeleopSendLoop();
  resetTeleopAxes();
  teleop.active = false;
  teleop.lastSentAt = -Infinity;
  sendTeleopKeepaliveStop();
}

const JOYSTICK_REVEAL_MS = 420;
const JOYSTICK_HIDE_MS = 220;
const mappingJoystickVisibility = {
  token: 0,
  frameId: null,
  timerId: null,
  visible: false,
  pendingReveal: false,
};

function cancelMappingJoystickVisibility() {
  if (mappingJoystickVisibility.frameId !== null) {
    cancelAnimationFrame(mappingJoystickVisibility.frameId);
    mappingJoystickVisibility.frameId = null;
  }
  if (mappingJoystickVisibility.timerId !== null) {
    clearTimeout(mappingJoystickVisibility.timerId);
    mappingJoystickVisibility.timerId = null;
  }
}

function concealMappingJoystick(joystick) {
  if (!joystick) return;
  joystick.classList.remove("joystick-revealed");
  joystick.classList.add("joystick-concealed");
  joystick.inert = true;
  joystick.setAttribute("aria-hidden", "true");
}

function revealMappingJoystick(joystick, token) {
  if (token !== mappingJoystickVisibility.token) return; // stale request
  mappingJoystickVisibility.frameId = null;
  if (!joystick) return;
  joystick.classList.remove("joystick-concealed");
  joystick.classList.add("joystick-revealed");
  joystick.inert = false;
  joystick.setAttribute("aria-hidden", "false");
  mappingJoystickVisibility.visible = true;
}

function scheduleMappingJoystickReveal(joystick, token) {
  if (token !== mappingJoystickVisibility.token) return; // stale request
  if (glassMotionReduced()) {
    revealMappingJoystick(joystick, token);
    return;
  }
  mappingJoystickVisibility.frameId = requestAnimationFrame(() =>
    revealMappingJoystick(joystick, token),
  );
}

function flushPendingMappingJoystickReveal() {
  if (!mappingJoystickVisibility.pendingReveal) return;
  const token = mappingJoystickVisibility.token;
  mappingJoystickVisibility.pendingReveal = false;
  scheduleMappingJoystickReveal(document.getElementById("mappingJoystick"), token);
}

async function setMappingJoystickVisible(visible, options = {}) {
  const joystick = document.getElementById("mappingJoystick");
  const stage = document.querySelector(".stage");
  if (stage) stage.classList.toggle("mapping-active", visible);

  mappingJoystickVisibility.token += 1;
  const token = mappingJoystickVisibility.token;
  cancelMappingJoystickVisibility();
  mappingJoystickVisibility.pendingReveal = false;

  if (!visible) {
    // Safety first: disable control and start the fade before awaiting the stop.
    concealMappingJoystick(joystick);
    mappingJoystickVisibility.visible = false;
    const hideMs = glassMotionReduced() ? 0 : JOYSTICK_HIDE_MS;
    if (hideMs === 0) {
      if (joystick) joystick.hidden = true;
    } else {
      mappingJoystickVisibility.timerId = setTimeout(() => {
        mappingJoystickVisibility.timerId = null;
        if (token !== mappingJoystickVisibility.token) return; // stale request
        if (joystick) joystick.hidden = true;
      }, hideMs);
    }
    if (teleop.active || teleop.stopRequired || anyTeleopPointer()) await stopTeleop("tab");
    return;
  }

  // Expose the joystick while still visually concealed so it can be measured at
  // its final size, positioned, and only then faded into place.
  if (joystick) {
    joystick.hidden = false;
    concealMappingJoystick(joystick);
  }

  if (options.deferReveal) {
    // The sidebar grid is still sliding, so measuring now would persist a
    // mid-transition anchor. Stay concealed and inert until the settle flush
    // repositions the joystick at the final geometry.
    mappingJoystickVisibility.pendingReveal = true;
    return;
  }

  if (joystick) applyJoystickLayout();
  scheduleMappingJoystickReveal(joystick, token);
}
// --- End mapping joystick

// --- Joystick layout
const JOYSTICK_MIN_SIZE = 140;
const JOYSTICK_MAX_SIZE = 260;
const JOYSTICK_DEFAULT_SIZE = 180;
const JOYSTICK_VIEWPORT_MARGIN = 12;
// The CSS resize handle is 0.16 of the base and overhangs its edge by 0.08.
const JOYSTICK_RESIZE_HANDLE_OVERHANG_RATIO = 0.08;

const joystickLayout = {
  sticks: {
    throttle: { size: JOYSTICK_DEFAULT_SIZE, x: null, y: null },
    steering: { size: JOYSTICK_DEFAULT_SIZE, x: null, y: null },
  },
  mode: false,
  gesture: null,
  saveError: null,
  revision: 0,
};

function clampNumber(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function finiteNumberOr(value, fallback) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function finiteOffsetOrNull(value) {
  // A null/absent offset means "use the role default"; only a real number is
  // accepted, because Number(null) is 0 and would otherwise masquerade as one.
  if (typeof value !== "number") return null;
  return Number.isFinite(value) && value >= 0 ? value : null;
}

function clampedJoystickSize(value) {
  return clampNumber(value, JOYSTICK_MIN_SIZE, JOYSTICK_MAX_SIZE);
}

function joystickResizeOverhang(role) {
  return joystickLayout.sticks[role].size * JOYSTICK_RESIZE_HANDLE_OVERHANG_RATIO;
}

function joystickViewport() {
  return document.getElementById("view3d");
}

function joystickContainer() {
  return document.getElementById("mappingJoystick");
}

function stickElement(role) {
  return document.getElementById(role === "throttle" ? "throttleStick" : "steeringStick");
}

function applyStickSize(role) {
  const stick = stickElement(role);
  if (!stick) return;
  stick.style.setProperty("--stick-size", `${joystickLayout.sticks[role].size}px`);
}

function stickMaxLeft(size, viewportWidth, overhang) {
  return Math.max(
    JOYSTICK_VIEWPORT_MARGIN,
    viewportWidth - size - JOYSTICK_VIEWPORT_MARGIN - overhang,
  );
}

function stickMaxTop(size, viewportHeight, overhang) {
  return Math.max(
    JOYSTICK_VIEWPORT_MARGIN,
    viewportHeight - size - JOYSTICK_VIEWPORT_MARGIN - overhang,
  );
}

// The single coordinate reference and clamping routine shared by the live
// preview and the applied/saved geometry.  Offsets are viewport pixels from the
// top-left; a null offset means the role default (throttle lower-left, steering
// lower-right).  A saved in-bounds position is used unchanged, so collapsing or
// expanding the sidebar never rescales or rebases it.
function stickGeometry(role, viewportWidth, viewportHeight) {
  const stick = joystickLayout.sticks[role];
  const size = stick.size;
  const overhang = joystickResizeOverhang(role);
  const defaultLeft =
    role === "throttle"
      ? JOYSTICK_VIEWPORT_MARGIN
      : viewportWidth - size - JOYSTICK_VIEWPORT_MARGIN - overhang;
  const defaultTop = viewportHeight - size - JOYSTICK_VIEWPORT_MARGIN - overhang;
  const left = Number.isFinite(stick.x) ? stick.x : defaultLeft;
  const top = Number.isFinite(stick.y) ? stick.y : defaultTop;
  return {
    left: clampNumber(
      left,
      JOYSTICK_VIEWPORT_MARGIN,
      stickMaxLeft(size, viewportWidth, overhang),
    ),
    top: clampNumber(
      top,
      JOYSTICK_VIEWPORT_MARGIN,
      stickMaxTop(size, viewportHeight, overhang),
    ),
    size,
    overhang,
  };
}

function applyJoystickLayout() {
  const joystick = joystickContainer();
  const viewport = joystickViewport();
  for (const role of STICK_ROLES) applyStickSize(role);
  if (!viewport) {
    updateHudPlacement();
    return;
  }
  if (joystick && joystick.hidden) {
    updateHudPlacement();
    return;
  }
  const viewportWidth = viewport.clientWidth || 0;
  const viewportHeight = viewport.clientHeight || 0;
  if (!viewportWidth || !viewportHeight) {
    updateHudPlacement();
    return;
  }
  for (const role of STICK_ROLES) {
    const stick = stickElement(role);
    if (!stick) continue;
    const geometry = stickGeometry(role, viewportWidth, viewportHeight);
    stick.style.left = `${geometry.left}px`;
    stick.style.top = `${geometry.top}px`;
    // Materialize a null role default the first time it is shown at a real
    // viewport so a later sidebar collapse cannot recompute the steering
    // default from the wider viewport and drift the stick off its pixels.
    // Finite (including clamped-out) offsets are never rewritten, and this is
    // in-memory only: saving still happens through the layout gesture paths.
    const stored = joystickLayout.sticks[role];
    if (!Number.isFinite(stored.x)) stored.x = geometry.left;
    if (!Number.isFinite(stored.y)) stored.y = geometry.top;
  }
  updateHudPlacement();
}

// Copy the applied (possibly clamped) preview back into the in-memory model so
// a save writes exactly the geometry the operator sees.
function syncStickOffsetsFromDom(role) {
  const stick = stickElement(role);
  if (!stick) return;
  const left = Number.parseFloat(stick.style.left);
  const top = Number.parseFloat(stick.style.top);
  if (Number.isFinite(left)) joystickLayout.sticks[role].x = left;
  if (Number.isFinite(top)) joystickLayout.sticks[role].y = top;
}

function beginJoystickMove(role, event) {
  if (!joystickLayout.mode || joystickLayout.gesture) return;
  event.preventDefault();
  event.stopPropagation();
  const stick = stickElement(role);
  const viewport = joystickViewport();
  if (!stick || !viewport) return;
  // A new gesture invalidates any preference response still in flight so its
  // saved values cannot overwrite the live preview.
  joystickLayout.revision += 1;
  const stickRect = stick.getBoundingClientRect();
  const viewportRect = viewport.getBoundingClientRect();
  joystickLayout.gesture = {
    kind: "move",
    role,
    pointerId: event.pointerId,
    startX: event.clientX,
    startY: event.clientY,
    left: stickRect.left - viewportRect.left,
    top: stickRect.top - viewportRect.top,
  };
  const handle = event.currentTarget;
  if (handle && typeof handle.setPointerCapture === "function") {
    handle.setPointerCapture(event.pointerId);
  }
}

function moveJoystickGesture(event) {
  const gesture = joystickLayout.gesture;
  if (!gesture || gesture.kind !== "move" || event.pointerId !== gesture.pointerId) return;
  event.preventDefault();
  event.stopPropagation();
  const stick = stickElement(gesture.role);
  const viewport = joystickViewport();
  if (!stick || !viewport) return;
  const viewportWidth = viewport.clientWidth || 0;
  const viewportHeight = viewport.clientHeight || 0;
  const size = joystickLayout.sticks[gesture.role].size;
  const overhang = joystickResizeOverhang(gesture.role);
  const left = clampNumber(
    gesture.left + (event.clientX - gesture.startX),
    JOYSTICK_VIEWPORT_MARGIN,
    stickMaxLeft(size, viewportWidth, overhang),
  );
  const top = clampNumber(
    gesture.top + (event.clientY - gesture.startY),
    JOYSTICK_VIEWPORT_MARGIN,
    stickMaxTop(size, viewportHeight, overhang),
  );
  joystickLayout.sticks[gesture.role].x = left;
  joystickLayout.sticks[gesture.role].y = top;
  stick.style.left = `${left}px`;
  stick.style.top = `${top}px`;
  updateHudPlacement();
}

function beginJoystickResize(role, event) {
  if (!joystickLayout.mode || joystickLayout.gesture) return;
  event.preventDefault();
  event.stopPropagation();
  // A new gesture invalidates any preference response still in flight so its
  // saved values cannot overwrite the live preview.
  joystickLayout.revision += 1;
  joystickLayout.gesture = {
    kind: "resize",
    role,
    pointerId: event.pointerId,
    startX: event.clientX,
    startY: event.clientY,
    startSize: joystickLayout.sticks[role].size,
  };
  const handle = event.currentTarget;
  if (handle && typeof handle.setPointerCapture === "function") {
    handle.setPointerCapture(event.pointerId);
  }
}

function resizeJoystickGesture(event) {
  const gesture = joystickLayout.gesture;
  if (!gesture || gesture.kind !== "resize" || event.pointerId !== gesture.pointerId) return;
  event.preventDefault();
  event.stopPropagation();
  const delta = event.clientX - gesture.startX + (event.clientY - gesture.startY);
  joystickLayout.sticks[gesture.role].size = clampedJoystickSize(
    gesture.startSize + delta,
  );
  applyJoystickLayout();
}

async function finishJoystickGesture(event) {
  const gesture = joystickLayout.gesture;
  if (!gesture) return;
  if (event && event.pointerId !== undefined && event.pointerId !== gesture.pointerId) return;
  if (event) {
    event.preventDefault();
    event.stopPropagation();
  }
  const role = gesture.role;
  joystickLayout.gesture = null;
  applyJoystickLayout();
  syncStickOffsetsFromDom(role);
  await persistJoystickPreferences();
}

function renderJoystickSettingsError() {
  const element = document.getElementById("joystickSettingsError");
  if (!element) return;
  if (joystickLayout.saveError) {
    element.hidden = false;
    element.textContent = `Joystick error: ${joystickLayout.saveError}`;
  } else {
    element.hidden = true;
    element.textContent = "";
  }
}

function setJoystickHandleAttributes() {
  const settings = document.getElementById("joystickSettings");
  if (settings) settings.setAttribute("aria-pressed", String(joystickLayout.mode));
  const joystick = joystickContainer();
  if (joystick) joystick.classList.toggle("layout-mode", joystickLayout.mode);
}

async function enterJoystickLayout() {
  if (joystickLayout.mode) return;
  teleop.layoutMode = true;
  await stopTeleop("settings", true);
  // A tab change or Done press during the awaited stop clears
  // teleop.layoutMode; its continuation must not re-enable layout mode.
  if (!teleop.layoutMode) return;
  joystickLayout.mode = true;
  joystickLayout.saveError = null;
  setJoystickHandleAttributes();
  renderJoystickSettingsError();
  applyJoystickLayout();
}

function exitJoystickLayout() {
  if (!joystickLayout.mode && !teleop.layoutMode) return;
  teleop.layoutMode = false;
  joystickLayout.mode = false;
  joystickLayout.gesture = null;
  setJoystickHandleAttributes();
  renderJoystickSettingsError();
  applyJoystickLayout();
}

function applySavedStickPreferences(saved, fallbackToDefaults) {
  for (const role of STICK_ROLES) {
    const stick = joystickLayout.sticks[role];
    const source = saved && typeof saved === "object" ? saved[role] : null;
    if (source && typeof source === "object") {
      stick.size = clampedJoystickSize(
        finiteNumberOr(
          source.size,
          fallbackToDefaults ? JOYSTICK_DEFAULT_SIZE : stick.size,
        ),
      );
      stick.x = finiteOffsetOrNull(source.x);
      stick.y = finiteOffsetOrNull(source.y);
    } else if (fallbackToDefaults) {
      stick.size = JOYSTICK_DEFAULT_SIZE;
      stick.x = null;
      stick.y = null;
    }
  }
}

async function persistJoystickPreferences() {
  joystickLayout.revision += 1;
  const revision = joystickLayout.revision;
  // Resolve any role-default (null) offsets to the geometry on screen before
  // posting, so a resize-only gesture still saves concrete viewport pixels.
  applyJoystickLayout();
  for (const role of STICK_ROLES) syncStickOffsetsFromDom(role);
  try {
    const data = await postJson("/api/ui/preferences", {
      throttle: { ...joystickLayout.sticks.throttle },
      steering: { ...joystickLayout.sticks.steering },
    });
    if (revision !== joystickLayout.revision) return false;
    applySavedStickPreferences(data && data.joystick ? data.joystick : null, false);
    joystickLayout.saveError = null;
    renderJoystickSettingsError();
    applyJoystickLayout();
    return true;
  } catch (error) {
    if (revision !== joystickLayout.revision) return false;
    joystickLayout.saveError = error.message;
    renderJoystickSettingsError();
    applyJoystickLayout();
    return false;
  }
}

async function refreshJoystickPreferences() {
  const revision = joystickLayout.revision;
  try {
    const data = await getJson("/api/ui/preferences");
    if (revision !== joystickLayout.revision) return;
    applySavedStickPreferences(data && data.joystick ? data.joystick : null, true);
  } catch (_error) {
    // Keep the defaults already in joystickLayout when the request fails.
  }
  applyJoystickLayout();
}

function updateHudPlacement() {
  const stage = document.querySelector(".stage");
  const hud = document.getElementById("hud");
  const joystick = joystickContainer();
  if (!stage || !hud) return;
  if (state.view !== "mapping" || !joystick || joystick.hidden) {
    stage.classList.remove("hud-lower-right", "hud-center");
    return;
  }
  // Both lower corners hold a stick, so the HUD parks at the lower center
  // between them while Mapping is active.
  stage.classList.remove("hud-lower-right");
  stage.classList.toggle("hud-center", true);
}
// --- End joystick layout

// --- Sidebar collapse
const sidebarState = { collapsed: false };
// The effective collapse starts false because the initial DOM is expanded.
let appliedEffectiveCollapse = false;

function syncSidebarToggle(toggle) {
  if (!toggle) return;
  toggle.setAttribute("aria-expanded", String(!sidebarState.collapsed));
  toggle.setAttribute(
    "aria-label",
    sidebarState.collapsed ? "Expand sidebar" : "Collapse sidebar",
  );
}

function applyEffectiveSidebar() {
  const effective = sidebarState.collapsed || state.view === "system";
  const app = document.querySelector(".app");
  const panel = document.getElementById("controlPanel");
  const toggle = document.getElementById("sidebarToggle");
  if (appliedEffectiveCollapse === effective) {
    syncSidebarToggle(toggle);
    return false;
  }
  appliedEffectiveCollapse = effective;
  if (app) app.classList.toggle("sidebar-collapsed", effective);
  if (panel) {
    panel.inert = effective;
    panel.setAttribute("aria-hidden", String(effective));
  }
  syncSidebarToggle(toggle);
  if (effective && panel && panel.contains(document.activeElement)) {
    // System mode hides the toggle, so hand focus to the visible system tab.
    const focusTarget =
      state.view === "system"
        ? document.getElementById("tabSystem") || document.querySelector("#viewTabs .tab.active")
        : toggle;
    if (focusTarget) focusTarget.focus();
  }
  return true;
}

function setSidebarCollapsed(collapsed) {
  sidebarState.collapsed = Boolean(collapsed);
  if (applyEffectiveSidebar()) beginLayoutTransition({ gridChanged: true });
}

function toggleSidebar() {
  setSidebarCollapsed(!sidebarState.collapsed);
}

function observeMapStageResize() {
  if (typeof ResizeObserver !== "function") return;
  const stage = document.querySelector(".stage");
  if (!stage) return;
  const observer = new ResizeObserver(() => {
    noteLayoutResize();
  });
  observer.observe(stage);
}
// --- End sidebar collapse

// --- Page transition state machine
const LAYOUT_SETTLE_FALLBACK_MS = 780;

const pageTransition = {
  active: false,
  token: 0,
  pendingFlush: false,
  // A transition may settle early only when the grid geometry itself changed.
  // Pure view/panel/thumb transitions have no grid event to trust, so they
  // settle on the fallback timer instead.
  expectsGrid: false,
  fallbackTimerId: null,
  fallbackFrameId: null,
  resizeFrameId: null,
};

function cancelLayoutFallback() {
  if (pageTransition.fallbackTimerId !== null) {
    clearTimeout(pageTransition.fallbackTimerId);
    pageTransition.fallbackTimerId = null;
  }
  if (pageTransition.fallbackFrameId !== null) {
    cancelAnimationFrame(pageTransition.fallbackFrameId);
    pageTransition.fallbackFrameId = null;
  }
}

function beginLayoutTransition(options = {}) {
  // A grid run that is still physically moving stays sticky: a later pure
  // (tab/panel) begin must not clear expectsGrid, or live resize would stop and
  // a joystick reveal could measure mid-slide. settleLayoutTransition clears it.
  const existingGridInFlight =
    pageTransition.active && pageTransition.expectsGrid;
  pageTransition.token += 1;
  const token = pageTransition.token;
  pageTransition.active = true;
  pageTransition.pendingFlush = true;
  pageTransition.expectsGrid =
    existingGridInFlight || Boolean(options.gridChanged);
  cancelLayoutFallback();
  if (glassMotionReduced()) {
    pageTransition.fallbackFrameId = requestAnimationFrame(
      () => settleLayoutTransition(token),
    );
  } else {
    pageTransition.fallbackTimerId = setTimeout(
      () => settleLayoutTransition(token),
      LAYOUT_SETTLE_FALLBACK_MS,
    );
  }
  return token;
}

function settleLayoutTransition(token) {
  if (token !== pageTransition.token) return; // stale request
  // Cancel only through the API that produced the handle: timeout and frame
  // ids can numerically collide, so cross-cancelling would hit unrelated work.
  cancelLayoutFallback();
  pageTransition.active = false;
  pageTransition.expectsGrid = false;
  if (pageTransition.pendingFlush) {
    pageTransition.pendingFlush = false;
    doDeferredLayoutFlush();
  }
}

function noteLayoutResize() {
  if (pageTransition.active) {
    pageTransition.pendingFlush = true;
    return;
  }
  if (pageTransition.resizeFrameId !== null) return;
  pageTransition.resizeFrameId = requestAnimationFrame(() => {
    pageTransition.resizeFrameId = null;
    doDeferredLayoutFlush();
  });
}

function doDeferredLayoutFlush() {
  if (pageTransition.active) {
    // Never resize the drawing buffer mid-transition; the settle will flush.
    pageTransition.pendingFlush = true;
    return;
  }
  resizeAll(); // the only renderer.setSize call site
  applyJoystickLayout();
  updateHudPlacement();
  // Any reveal deferred for the grid slide may now measure settled geometry.
  flushPendingMappingJoystickReveal();
  renderer.render(scene, camera); // synchronous settle frame
}

function handleLayoutTransitionEnd(event) {
  // Only the app's own grid slide may end a transition early. The viewport,
  // panel, and thumb opacity/transform fades are shorter and must never settle
  // the longer grid animation, so transform/opacity and pseudo-element events
  // are ignored entirely.
  if (!pageTransition.active || !pageTransition.expectsGrid) return;
  if (event.propertyName !== "grid-template-columns") return;
  if (event.pseudoElement) return;
  if (event.target !== document.querySelector(".app")) return;
  settleLayoutTransition(pageTransition.token);
}

function attachLayoutTransitionListener() {
  const appEl = document.querySelector(".app");
  if (appEl) appEl.addEventListener("transitionend", handleLayoutTransitionEnd);
}
// --- End page transition state machine

function maybeClearGoalFromActionStatus() {
  if (!state.activeGoal || !state.navigationAction || !state.navigationAction.data) return;
  const status = state.navigationAction.data.status;
  const stamp = state.navigationAction.stamp || 0;
  if (status === "succeeded" && stamp >= state.activeGoal.publishedAt) {
    state.activeGoal = null;
    state.lastGoal = null;
    refreshController.invalidatePaths();
    state.paths = {};
    state.hideCompletedPaths = true;
    updateTargetPoseDisplay();
    updateTargetMarkers();
    updatePaths();
  }
}

const GLASS_DROP_FALLBACK_MS = 520;
const glassDropCleanups = new WeakMap();

function clearGlassDrop(element) {
  const pending = glassDropCleanups.get(element);
  if (!pending) return;
  glassDropCleanups.delete(element);
  element.removeEventListener("animationend", pending.clear);
  window.clearTimeout(pending.timerId);
  element.classList.remove("glass-drop");
}

function markGlassDrop(element) {
  if (!element) return;
  // Rapid re-marks must not stack listeners or timers: retire the prior
  // cleanup before arming the next one.
  clearGlassDrop(element);
  element.classList.remove("glass-drop");
  // One forced reflow restarts the keyframes when the same element is re-marked.
  void element.offsetWidth;
  element.classList.add("glass-drop");
  const clear = () => clearGlassDrop(element);
  element.addEventListener("animationend", clear, { once: true });
  const timerId = window.setTimeout(clear, GLASS_DROP_FALLBACK_MS);
  glassDropCleanups.set(element, { clear, timerId });
}

const TAB_ORDER = ["3d", "mapping", "system"];
let viewTabsControl = null;
let modeSegmentControl = null;

// One reusable drag controller positions and drags both segmented controls.
// Selection still flows through the existing click/keyboard handlers; a drag
// release calls the same setTab/setMode path exactly once.
function attachSegmentedControls() {
  const viewTabs = document.getElementById("viewTabs");
  if (viewTabs) {
    viewTabsControl = createSegmentedControl({
      track: viewTabs,
      buttons: Array.from(viewTabs.querySelectorAll(".tab")),
      onSelect: (index) => setTab(TAB_ORDER[index]),
    });
  }
  const modeSegment = document.getElementById("modeSegment");
  if (modeSegment) {
    const modeButtons = Array.from(modeSegment.querySelectorAll(".mode"));
    modeSegmentControl = createSegmentedControl({
      track: modeSegment,
      buttons: modeButtons,
      onSelect: (index) => setMode(modeButtons[index].dataset.mode),
    });
  }
}

function setMode(mode) {
  const previousMode = state.mode;
  state.mode = mode;
  document.getElementById("modeSegment").dataset.active = mode;
  let shouldClearSelection = false;
  if (previousMode !== mode) {
    shouldClearSelection = true;
  }
  if (mode === "view") {
    shouldClearSelection = true;
  }
  if (shouldClearSelection) {
    state.selection = null;
    updateSelectionDisplay();
  }
  document.querySelectorAll(".mode").forEach((button) => {
    button.classList.toggle("active", button.dataset.mode === mode);
  });
  if (modeSegmentControl) modeSegmentControl.sync();
  if (previousMode !== mode) markGlassDrop(document.getElementById("modeSegment"));
  updatePublishPoseButton();
  updateTargetMarkers();
}

function focusIfInside(container, target) {
  if (!container || !target) return;
  const active = document.activeElement;
  if (active && active !== document.body && container.contains(active)) target.focus();
}

function setTab(tab) {
  const view3d = document.getElementById("view3d");
  const viewSystem = document.getElementById("viewSystem");
  const viewTabs = document.getElementById("viewTabs");
  const previous3dActive = view3d.classList.contains("active");
  const previousSystemActive = viewSystem.classList.contains("active");
  const previousPanel = document.querySelector(".panel-view.active");
  const previousTab = viewTabs.dataset.active;

  state.view = tab;
  viewTabs.dataset.active = tab;
  document.getElementById("tab3dMap").classList.toggle("active", tab === "3d");
  document.getElementById("tabMapping").classList.toggle("active", tab === "mapping");
  document.getElementById("tabSystem").classList.toggle("active", tab === "system");
  if (viewTabsControl) viewTabsControl.sync();

  const isMapView = tab === "3d" || tab === "mapping";
  view3d.classList.toggle("active", isMapView);
  viewSystem.classList.toggle("active", tab === "system");
  view3d.inert = !isMapView;
  view3d.setAttribute("aria-hidden", String(!isMapView));
  viewSystem.inert = tab !== "system";
  viewSystem.setAttribute("aria-hidden", String(tab !== "system"));

  document.querySelector(".app").classList.toggle("system-mode", tab === "system");

  // Safety state commits synchronously: hiding the joystick also fires the
  // teleop stop, and leaving layout mode must never wait for the transition.
  // Showing waits until the sidebar geometry is known and the layout
  // transition is armed, so a System -> Mapping reveal can defer measuring the
  // joystick until the grid has settled.
  if (tab !== "mapping") {
    setMappingJoystickVisible(false);
    exitJoystickLayout();
  }

  const panelName = tab === "system" ? "" : (tab === "mapping" ? "mapping" : "navigation");
  document.querySelectorAll("[data-panel]").forEach((panel) => {
    const isActive = panel.dataset.panel === panelName;
    panel.classList.toggle("active", panel.dataset.panel === panelName);
    panel.inert = !isActive;
    panel.setAttribute("aria-hidden", String(!isActive));
  });

  layerGroup.visible = tab === "3d";
  pathGroup.visible = tab === "3d";
  markerGroup.visible = tab === "3d";
  vehicleGroup.visible = tab === "3d";
  mappingLayerGroup.visible = tab === "mapping";
  mappingPathGroup.visible = tab === "mapping";
  mappingVehicleGroup.visible = tab === "mapping";

  const viewChanged =
    view3d.classList.contains("active") !== previous3dActive ||
    viewSystem.classList.contains("active") !== previousSystemActive;
  const panelChanged = document.querySelector(".panel-view.active") !== previousPanel;
  const sidebarChanged = applyEffectiveSidebar();

  if (viewChanged || panelChanged || sidebarChanged) {
    if (previousTab !== tab) markGlassDrop(viewTabs);
    beginLayoutTransition({ gridChanged: sidebarChanged });
  }

  // Defer whenever the grid is still moving: either this click changed the
  // sidebar geometry, or an earlier click left a grid-expecting transition in
  // flight and a re-entrant Mapping click must not measure mid-slide.
  const deferJoystickReveal =
    sidebarChanged || (pageTransition.active && pageTransition.expectsGrid);

  if (tab === "mapping") {
    setMappingJoystickVisible(true, { deferReveal: deferJoystickReveal });
  }

  const activeTabButton =
    tab === "system"
      ? document.getElementById("tabSystem")
      : tab === "mapping"
        ? document.getElementById("tabMapping")
        : document.getElementById("tab3dMap");
  if (previous3dActive && !view3d.classList.contains("active")) {
    focusIfInside(view3d, activeTabButton);
  }
  if (previousSystemActive && !viewSystem.classList.contains("active")) {
    focusIfInside(viewSystem, activeTabButton);
  }
  if (previousPanel && previousPanel !== document.querySelector(".panel-view.active")) {
    focusIfInside(previousPanel, activeTabButton);
  }

  if (isMapView) {
    ensureMapRenderLoop();
  } else {
    pauseMapRenderLoop();
  }

  if (tab === "3d") {
    void refreshController.tick();
  } else if (tab === "mapping") {
    pollMappingState();
    void refreshController.tick();
  } else if (tab === "system") {
    pollSystem();
    if (typeof pollTf === "function") pollTf();
    if (typeof refreshSavedLogSessions === "function") refreshSavedLogSessions();
  }

  // Hidden System view must not poll or accept graph interaction.
  if (typeof ensureTfGraphView === "function") {
    const tfGraph = ensureTfGraphView();
    if (tfGraph) tfGraph.setActive(tab === "system");
  }
}

function selectedPosePayload(includeZ) {
  return { x: state.selection.x, y: state.selection.y, z: includeZ ? state.selection.z : 0, yaw: state.selection.yaw };
}

function updatePublishPoseButton() {
  const button = document.getElementById("publishPose");
  const control = state.navigationControl;
  const transitioning = Boolean(control) && ["pausing", "resuming", "clearing"].includes(control.state);
  button.disabled = state.mode === "view";
  if (state.mode === "goal" && transitioning) {
    button.disabled = true;
  }
  button.textContent = "Publish Pose";
}

async function publishPose() {
  const result = document.getElementById("publishResult");
  if (state.mode === "view") {
    result.textContent = "Switch to Initial Pose or Goal mode first.";
    return;
  }
  if (!state.selection) {
    result.textContent = "Select a pose in the 3D map first.";
    return;
  }
  try {
    if (state.mode === "initial") {
      const response = await postJson("/api/initialpose", selectedPosePayload(false));
      result.textContent = `${response.message} @ ${new Date(response.stamp * 1000).toLocaleTimeString()}`;
      return;
    }
    if (state.mode === "goal") {
      const payload = selectedPosePayload(true);
      const response = await postJson("/api/goal", payload);
      state.activeGoal = { ...payload, publishedAt: response.stamp };
      state.lastGoal = { ...state.activeGoal };
      state.hideCompletedPaths = false;
      refreshController.invalidatePaths();
      state.paths = {};
      state.selection = null;
      updateSelectionDisplay();
      updatePaths();
      updateTargetPoseDisplay();
      updateTargetMarkers();
      result.textContent = `${response.message} @ ${new Date(response.stamp * 1000).toLocaleTimeString()}`;
    }
  } catch (error) {
    result.textContent = `Publish failed: ${error.message}`;
  }
}

function setManagedActionStatus(message) {
  const result = document.getElementById("publishResult");
  const text = message == null ? "" : String(message);
  if (result) result.textContent = text;
  // The hardware startup card is mounted on both pages; both status lines show
  // the latest managed-process result or error and stay hidden when empty.
  document.querySelectorAll("[data-managed-status]").forEach((element) => {
    element.textContent = text;
    element.hidden = text === "";
  });
}

async function stopManagedTarget(target) {
  try {
    const response = await postJson("/api/managed/stop", { target });
    setManagedActionStatus(`${response.message} @ ${new Date(response.stamp * 1000).toLocaleTimeString()}`);
    if (target === "all" || target === "localization") {
      clearMapSceneState();
    } else if (target === "navigation") {
      clearNavigationSceneState();
      clearNavigationLayers();
    }
    pollState();
    await pollLogs();
    return true;
  } catch (error) {
    setManagedActionStatus(`Stop failed: ${error.message}`);
    return false;
  }
}

async function startManagedTarget(target) {
  try {
    const response = await postJson("/api/managed/start", { target });
    setManagedActionStatus(`${response.message} @ ${new Date(response.stamp * 1000).toLocaleTimeString()}`);
    await pollLogs();
    return true;
  } catch (error) {
    setManagedActionStatus(`Start failed: ${error.message}`);
    // Revalidate the selection so a stale/unavailable map becomes visible.
    refreshMaps();
    return false;
  }
}

function attachTopicLoggingEvents() {
  document.querySelectorAll("[data-topic-logging-toggle]").forEach((button) => {
    // Nested click targets bubble to the button; desired state is never read
    // from the event target.
    button.addEventListener("click", () => toggleTopicLogging());
  });
}

function attachEvents() {
  window.addEventListener("resize", noteLayoutResize);
  document.getElementById("tab3dMap").addEventListener("click", () => setTab("3d"));
  document.getElementById("tabMapping").addEventListener("click", () => setTab("mapping"));
  document.getElementById("tabSystem").addEventListener("click", () => setTab("system"));
  attachTopicLoggingEvents();
  const tfRefresh = document.getElementById("tfTreeRefresh");
  if (tfRefresh) tfRefresh.addEventListener("click", () => pollTf());
  const tfZoomIn = document.getElementById("tfTreeZoomIn");
  if (tfZoomIn) {
    tfZoomIn.addEventListener("click", () => {
      const view = ensureTfGraphView();
      if (view) view.zoomBy(1.25);
    });
  }
  const tfZoomOut = document.getElementById("tfTreeZoomOut");
  if (tfZoomOut) {
    tfZoomOut.addEventListener("click", () => {
      const view = ensureTfGraphView();
      if (view) view.zoomBy(0.8);
    });
  }
  const tfFit = document.getElementById("tfTreeFit");
  if (tfFit) {
    tfFit.addEventListener("click", () => {
      const view = ensureTfGraphView();
      if (view) view.fit();
    });
  }
  const savedLogTargetSelect = document.getElementById("savedLogTarget");
  if (savedLogTargetSelect) {
    savedLogTargetSelect.addEventListener("change", () => {
      const output = document.getElementById("savedLogTail");
      if (output) output.textContent = "";
      updateSavedLogActions();
      refreshSavedLogSessions();
    });
  }
  const savedLogRefreshButton = document.getElementById("savedLogRefresh");
  if (savedLogRefreshButton) {
    savedLogRefreshButton.addEventListener("click", () => refreshSavedLogSessions());
  }
  const savedLogViewButton = document.getElementById("savedLogView");
  if (savedLogViewButton) {
    savedLogViewButton.addEventListener("click", () => viewSavedLogTail());
  }
  // Lightweight accordion for secondary sidebar groups (logs, layers, sources).
  document.querySelectorAll(".collapsible .collapse-toggle").forEach((button) => {
    button.addEventListener("click", () => {
      const section = button.closest(".collapsible");
      if (!section) return;
      const body = section.querySelector(".collapse-body");
      const collapsed = section.hasAttribute("data-collapsed");
      if (collapsed) {
        section.removeAttribute("data-collapsed");
        if (body) body.hidden = false;
        button.setAttribute("aria-expanded", "true");
        button.textContent = "Hide";
      } else {
        section.setAttribute("data-collapsed", "true");
        if (body) body.hidden = true;
        button.setAttribute("aria-expanded", "false");
        button.textContent = "Show";
      }
    });
  });
  document.querySelectorAll(".mode").forEach((button) => button.addEventListener("click", () => setMode(button.dataset.mode)));
  document.querySelectorAll("input[data-layer]").forEach((input) => {
    input.addEventListener("change", () => {
      LAYERS[input.dataset.layer].enabled = input.checked;
      if (LAYERS[input.dataset.layer].object) LAYERS[input.dataset.layer].object.visible = input.checked;
      void refreshController.tick();
    });
  });
  document.getElementById("pathLayer").addEventListener("change", (event) => {
    state.showPaths = event.target.checked;
    updatePaths();
  });
  document.getElementById("vehicleLayer").addEventListener("change", (event) => {
    state.showVehicle = event.target.checked;
    updateVehicleMarker();
  });
  document.querySelectorAll("input[data-mapping-layer]").forEach((input) => {
    input.addEventListener("change", () => {
      MAPPING_LAYERS[input.dataset.mappingLayer].enabled = input.checked;
      if (MAPPING_LAYERS[input.dataset.mappingLayer].object) {
        MAPPING_LAYERS[input.dataset.mappingLayer].object.visible = input.checked;
      }
      void refreshController.tick();
    });
  });
  document.getElementById("mappingPathLayer").addEventListener("change", (event) => {
    state.showMappingPath = event.target.checked;
    updateMappingPath();
  });
  document.getElementById("mappingVehicleLayer").addEventListener("change", (event) => {
    state.showMappingVehicle = event.target.checked;
    updateMappingVehicleMarker();
  });
  document.getElementById("navigationPrimary").addEventListener("click", () => {
    const control = state.navigationControl;
    if (control && control.state === "paused") {
      postNavigationCommand("/api/navigation/resume", "Resume failed");
    } else {
      postNavigationCommand("/api/navigation/pause", "Pause failed");
    }
  });
  document.getElementById("navigationClear").addEventListener("click", () => postNavigationCommand("/api/navigation/clear", "Clear failed"));
  updatePublishPoseButton();
  document.getElementById("publishPose").addEventListener("click", publishPose);
  document.querySelectorAll("[data-managed-target]").forEach((button) =>
    button.addEventListener("click", () => toggleManagedTarget(button.dataset.managedTarget))
  );
  document.getElementById("mapSelect").addEventListener("change", (event) => selectMap(event.target.value));
  document.getElementById("mapMode").addEventListener("change", (event) => selectMapMode(event.target.value));
  document.getElementById("autoRelocalizationToggle").addEventListener("click", () => toggleAutoRelocalization());
  document.getElementById("refreshMaps").addEventListener("click", () => refreshMaps(true));
  document.getElementById("sidebarToggle").addEventListener("click", toggleSidebar);
  attachGlassStrengthControl();
  attachSegmentedControls();
  observeMapStageResize();
  attachLayoutTransitionListener();
  document.getElementById("openSaveMap").addEventListener("click", () => {
    saveMapSubmissionError = null;
    const dialog = document.getElementById("saveMapDialog");
    dialog.showModal();
    document.getElementById("saveMapDestination").focus();
  });
  document.getElementById("cancelSaveMap").addEventListener("click", () => document.getElementById("saveMapDialog").close());
  document.getElementById("saveMapDestination").addEventListener("input", (event) => event.target.setCustomValidity(""));
  document.getElementById("saveMapForm").addEventListener("submit", submitSaveMap);

  document.getElementById("openRecordBag").addEventListener("click", openRecordBagDialog);
  document.getElementById("stopRecording").addEventListener("click", stopRecording);
  document.getElementById("refreshRecordingTopics").addEventListener("click", refreshRecordingTopics);
  document.getElementById("browseRecordingDirectory").addEventListener("click", () => loadRecordingDirectory(document.getElementById("recordingDirectory").value.trim()));
  document.getElementById("recordingBrowserUp").addEventListener("click", () => loadRecordingDirectory(state.recordingBrowserParent || ""));
  document.getElementById("recordingBrowserUse").addEventListener("click", () => {
    document.getElementById("recordingDirectory").value = state.recordingBrowserPath || "";
    document.getElementById("recordingBrowser").hidden = true;
  });
  document.getElementById("cancelRecordBag").addEventListener("click", () => document.getElementById("recordBagDialog").close());
  document.getElementById("recordBagForm").addEventListener("submit", submitRecording);

  // Driving pointers are routed at the window by pointerId so one stick's
  // terminal event can never cancel the other.  Move/resize gestures are shared
  // but read their role from the captured gesture.
  attachStickPointerRouting();
  for (const role of STICK_ROLES) {
    const moveHandle = document.getElementById(
      role === "throttle" ? "throttleMoveHandle" : "steeringMoveHandle",
    );
    if (moveHandle) {
      moveHandle.addEventListener("pointerdown", (event) => beginJoystickMove(role, event));
      moveHandle.addEventListener("pointermove", moveJoystickGesture);
      moveHandle.addEventListener("pointerup", finishJoystickGesture);
      moveHandle.addEventListener("pointercancel", finishJoystickGesture);
      moveHandle.addEventListener("lostpointercapture", finishJoystickGesture);
    }
    const resizeHandle = document.getElementById(
      role === "throttle" ? "throttleResizeHandle" : "steeringResizeHandle",
    );
    if (resizeHandle) {
      resizeHandle.addEventListener("pointerdown", (event) => beginJoystickResize(role, event));
      resizeHandle.addEventListener("pointermove", resizeJoystickGesture);
      resizeHandle.addEventListener("pointerup", finishJoystickGesture);
      resizeHandle.addEventListener("pointercancel", finishJoystickGesture);
      resizeHandle.addEventListener("lostpointercapture", finishJoystickGesture);
    }
  }
  window.addEventListener("blur", handleJoystickBlur);
  window.addEventListener("pagehide", handlePageHide);
  document.addEventListener("visibilitychange", handleVisibilityChange);

  const joystickSettings = document.getElementById("joystickSettings");
  if (joystickSettings) joystickSettings.addEventListener("click", enterJoystickLayout);
  const joystickDone = document.getElementById("joystickDone");
  if (joystickDone) joystickDone.addEventListener("click", exitJoystickLayout);

  mapCanvas3d.addEventListener("contextmenu", (event) => event.preventDefault());
  mapCanvas3d.addEventListener("wheel", (event) => {
    event.preventDefault();
    state.camera.distance = Math.max(5, Math.min(280, state.camera.distance * (event.deltaY < 0 ? 0.88 : 1.13)));
  }, { passive: false });
  mapCanvas3d.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    mapCanvas3d.setPointerCapture(event.pointerId);
    const point = canvasPoint(event);
    if (event.pointerType === "touch") {
      state.pointer.touches.set(event.pointerId, point);
      state.pointer.touchStart = touchGesture();
      if (state.pointer.touchStart) {
        state.pointer.touchStart.cameraDistance = state.camera.distance;
        state.pointer.selecting = false;
        return;
      }
    }
    state.pointer = {
      active: true,
      button: event.button,
      start: point,
      previous: point,
      selecting: false,
      touches: state.pointer.touches,
      touchStart: state.pointer.touchStart,
    };
    if (event.pointerType === "touch") {
      state.pointer.touches.set(event.pointerId, point);
    }
    if (state.mode !== "view" && event.button === 0) {
      const hit = selectionIntersection(event);
      if (hit) {
        setSelectionFromPoint(hit, 0);
        state.pointer.selecting = true;
      }
    }
  });
  mapCanvas3d.addEventListener("pointermove", (event) => {
    if (event.pointerType === "touch" && state.pointer.touches.size >= 2) {
      event.preventDefault();
      handleTouchPanZoom(event);
      return;
    }
    if (!state.pointer.active || !state.pointer.previous) return;
    event.preventDefault();
    if (event.pointerType === "touch") {
      state.pointer.touches.set(event.pointerId, canvasPoint(event));
    }
    const point = canvasPoint(event);
    const dx = point.x - state.pointer.previous.x;
    const dy = point.y - state.pointer.previous.y;
    if (state.pointer.selecting && state.selection) {
      const hit = groundIntersection(event);
      if (hit) {
        const pose = threeToRos(hit);
        state.selection.yaw = Math.atan2(pose.y - state.selection.y, pose.x - state.selection.x);
        updateSelectionDisplay();
        updateTargetMarkers();
      }
    } else if (state.pointer.button === 2 || state.pointer.button === 1) {
      panCameraByScreenDelta(dx, dy);
    } else {
      rotateCameraByScreenDelta(dx, dy);
    }
    state.pointer.previous = point;
  });
  const endPointer = (event) => {
    state.pointer.touches.delete(event.pointerId);
    state.pointer.touchStart = touchGesture();
    if (state.pointer.touchStart) {
      state.pointer.touchStart.cameraDistance = state.camera.distance;
      return;
    }
    state.pointer.active = false;
    state.pointer.selecting = false;
    state.pointer.previous = null;
  };
  mapCanvas3d.addEventListener("pointerup", endPointer);
  mapCanvas3d.addEventListener("pointercancel", endPointer);
  mapCanvas3d.addEventListener("pointerleave", endPointer);
}

function start() {
  attachEvents();
  resizeAll();
  setTab("3d");
  pollState();
  pollLogs();
  pollDiagnostics();
  heartbeatViewer();
  void refreshController.tick();
  pollMappingState();
  pollPose();
  refreshMaps();
  refreshJoystickPreferences();
  setSidebarCollapsed(false);
  setInterval(pollState, 1000);
  setInterval(pollMappingState, 1000);
  setInterval(pollPose, 100);
  setInterval(pollLogs, 1000);
  setInterval(pollDiagnostics, 1000);
  setInterval(heartbeatViewer, 3000);
  setInterval(() => refreshController.tick(), REFRESH_RATES_MS.schedulerTick);
  setInterval(pollSystem, 1000);
  setInterval(pollTf, 3000);
  setInterval(refreshPerformanceIndicators, 500);
  render();
}

start();
