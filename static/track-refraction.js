// Attribution: the dynamic displacement-map generator in this file adapts
// helper functions and techniques from
// https://github.com/shuding/liquid-glass ("Copy-paste Liquid Glass effect
// using SVG filters"), MIT License, Copyright (c) 2025 Shu Ding. The full
// upstream license text and the adaptation description live in
// THIRD_PARTY_NOTICES.md at the repository root. The static LIQUID_DROP_MAP
// asset, the #liquidDrop filter markup, the capability helper, the stylesheet,
// and the transition state machine remain independently implemented.

import { glassMode, onGlassModeChange } from "./liquid-glass.js";

const FILTER_ID = "liquidTrackFilter";
const SVG_NS = "http://www.w3.org/2000/svg";
const XLINK_NS = "http://www.w3.org/1999/xlink";
const TRACK_ID = "viewTabs";
const DEFS_SELECTOR = "#liquidGlassDefs defs";
const DEFAULT_STRENGTH_RATIO = 0.12;
const DEFAULT_BAND_RATIO = 0.25;

// Adapted from upstream liquid-glass.js (smoothStep).
export function smoothStep(a, b, t) {
  const x = Math.max(0, Math.min(1, (t - a) / (b - a)));
  return x * x * (3 - 2 * x);
}

function vectorLength(x, y) {
  return Math.sqrt(x * x + y * y);
}

// Adapted from upstream liquid-glass.js (roundedRectSDF). `width` and `height`
// are half extents in the caller's units. Negative inside the shape, zero on
// the boundary, positive outside.
export function roundedRectSDF(x, y, width, height, radius) {
  const qx = Math.abs(x) - width + radius;
  const qy = Math.abs(y) - height + radius;
  return Math.min(Math.max(qx, qy), 0) + vectorLength(Math.max(qx, 0), Math.max(qy, 0)) - radius;
}

function positiveNumber(value, fallback) {
  return Number.isFinite(value) && value > 0 ? value : fallback;
}

// Generated per device pixel at the track's real border-box CSS size and DPR.
// The interior is neutral (R=G=128). On the rim band the refraction follows the
// analytic rounded-rectangle SDF outward normal negated to point inward, so a
// wide pill refracts straight down its long edges and sideways on its caps
// instead of radially from the center. Pixels outside the pill stay neutral:
// CSS `border-radius` clips them, so displacing them would only distort the
// invisible region. The displacement is bounded by `strength`, and `scale` is
// the feDisplacementMap scale in CSS pixels.
//
// The analytic normal uses qx = |x| - halfWidth + radius and
// qy = |y| - halfHeight + radius. When both are positive the point is on a
// rounded corner, so the outward normal is normalize(qx, qy); otherwise the
// closest edge wins (horizontal when qx > qy, vertical otherwise). The sign of
// x/y restores the quadrant.
export function computeTrackDisplacementField(options = {}) {
  const cssWidth = positiveNumber(Number(options.width), 1);
  const cssHeight = positiveNumber(Number(options.height), 1);
  const dpr = positiveNumber(Number(options.dpr), 1);
  const deviceWidth = Math.max(1, Math.round(cssWidth * dpr));
  const deviceHeight = Math.max(1, Math.round(cssHeight * dpr));
  const halfWidth = cssWidth / 2;
  const halfHeight = cssHeight / 2;
  const radius = Math.min(
    positiveNumber(Number(options.radius), halfHeight),
    halfWidth,
    halfHeight,
  );
  const band = positiveNumber(Number(options.band), cssHeight * DEFAULT_BAND_RATIO);
  const strength = positiveNumber(
    Number(options.strength),
    Math.max(2, cssHeight * DEFAULT_STRENGTH_RATIO),
  );
  const scale = strength * 2;
  const data = new Uint8ClampedArray(deviceWidth * deviceHeight * 4);

  for (let py = 0; py < deviceHeight; py += 1) {
    const y = (py + 0.5) / dpr - halfHeight;
    for (let px = 0; px < deviceWidth; px += 1) {
      const x = (px + 0.5) / dpr - halfWidth;
      const distance = roundedRectSDF(x, y, halfWidth, halfHeight, radius);
      let dx = 0;
      let dy = 0;
      if (distance <= 0) {
        const rim = smoothStep(band, 0, -distance);
        if (rim > 0) {
          const qx = Math.abs(x) - halfWidth + radius;
          const qy = Math.abs(y) - halfHeight + radius;
          let normalX;
          let normalY;
          if (qx > 0 && qy > 0) {
            const length = vectorLength(qx, qy) || 1;
            normalX = qx / length;
            normalY = qy / length;
          } else if (qx > qy) {
            normalX = 1;
            normalY = 0;
          } else {
            normalX = 0;
            normalY = 1;
          }
          // Outward normal times the sign of x/y, then negated for inward
          // refraction.
          dx = -Math.sign(x) * normalX * rim * strength;
          dy = -Math.sign(y) * normalY * rim * strength;
        }
      }
      const index = (py * deviceWidth + px) * 4;
      data[index] = Math.round((0.5 + dx / scale) * 255);
      data[index + 1] = Math.round((0.5 + dy / scale) * 255);
      data[index + 2] = 0;
      data[index + 3] = 255;
    }
  }

  return { width: deviceWidth, height: deviceHeight, dpr, scale, data };
}

function findChildByTag(element, tag) {
  const children = element && element.children ? element.children : [];
  for (let index = 0; index < children.length; index += 1) {
    const child = children[index];
    const name = child.localName || child.tagName || "";
    if (String(name).toLowerCase() === tag) return child;
  }
  return null;
}

function defaultImageData(data, width, height) {
  if (typeof ImageData === "function") return new ImageData(data, width, height);
  return { data, width, height };
}

const controllers = new WeakMap();

export function initTrackRefraction(options = {}) {
  const documentRef = options.document || (typeof document !== "undefined" ? document : null);
  if (!documentRef) return null;
  const track =
    options.track ||
    (typeof documentRef.getElementById === "function" ? documentRef.getElementById(TRACK_ID) : null);
  const defs =
    options.defs ||
    (typeof documentRef.querySelector === "function" ? documentRef.querySelector(DEFS_SELECTOR) : null);
  if (!track || !defs) return null;

  const existing = controllers.get(track);
  if (existing && existing.isAlive()) return existing;

  const windowRef =
    options.window !== undefined ? options.window : typeof window !== "undefined" ? window : null;
  const ResizeObserverCtor =
    options.ResizeObserver !== undefined
      ? options.ResizeObserver
      : typeof ResizeObserver !== "undefined"
        ? ResizeObserver
        : null;
  const raf =
    options.requestAnimationFrame ||
    (typeof requestAnimationFrame === "function"
      ? requestAnimationFrame.bind(globalThis)
      : (callback) => setTimeout(callback, 16));
  const caf =
    options.cancelAnimationFrame ||
    (typeof cancelAnimationFrame === "function"
      ? cancelAnimationFrame.bind(globalThis)
      : (id) => clearTimeout(id));
  const matchMediaRef =
    options.matchMedia !== undefined
      ? options.matchMedia
      : typeof matchMedia === "function"
        ? matchMedia.bind(globalThis)
        : null;
  const getDpr =
    options.getDevicePixelRatio ||
    (() => (typeof devicePixelRatio === "number" ? devicePixelRatio : 1));
  const getMode = options.getGlassMode || glassMode;
  const subscribe = options.onGlassModeChange || onGlassModeChange;
  const createCanvas =
    options.createCanvas ||
    (() => (typeof documentRef.createElement === "function" ? documentRef.createElement("canvas") : null));
  const makeImageData = options.makeImageData || defaultImageData;

  let alive = true;
  let installed = false;
  let generation = 0;
  let frameId = null;
  let filter = null;
  let feImage = null;
  let feDisplacementMap = null;
  let canvas = null;
  let context = null;
  let observer = null;
  let windowListenerAttached = false;
  let dprMedia = null;
  let dprMediaValue = null;
  let lastKey = null;
  let unsubscribe = null;

  function queryFilter() {
    return typeof defs.querySelector === "function" ? defs.querySelector(`#${FILTER_ID}`) : null;
  }

  function createFilter() {
    const nextFilter = documentRef.createElementNS(SVG_NS, "filter");
    nextFilter.setAttribute("id", FILTER_ID);
    nextFilter.setAttribute("filterUnits", "userSpaceOnUse");
    nextFilter.setAttribute("color-interpolation-filters", "sRGB");
    nextFilter.setAttribute("x", "0");
    nextFilter.setAttribute("y", "0");
    nextFilter.setAttribute("width", "1");
    nextFilter.setAttribute("height", "1");
    const image = documentRef.createElementNS(SVG_NS, "feImage");
    image.setAttribute("result", "trackMap");
    image.setAttribute("preserveAspectRatio", "none");
    image.setAttribute("width", "1");
    image.setAttribute("height", "1");
    const displacement = documentRef.createElementNS(SVG_NS, "feDisplacementMap");
    displacement.setAttribute("in", "SourceGraphic");
    displacement.setAttribute("in2", "trackMap");
    displacement.setAttribute("xChannelSelector", "R");
    displacement.setAttribute("yChannelSelector", "G");
    displacement.setAttribute("scale", "0");
    if (typeof nextFilter.append === "function") nextFilter.append(image, displacement);
    else {
      nextFilter.appendChild(image);
      nextFilter.appendChild(displacement);
    }
    defs.appendChild(nextFilter);
    return { nextFilter, image, displacement };
  }

  function syncDprWatcher() {
    if (!installed) return;
    const dpr = getDpr();
    if (dprMedia && dprMediaValue === dpr) return;
    removeDprWatcher();
    dprMediaValue = dpr;
    if (typeof matchMediaRef === "function") {
      dprMedia = matchMediaRef(`(resolution: ${dpr}dppx)`);
      if (dprMedia && typeof dprMedia.addEventListener === "function") {
        dprMedia.addEventListener("change", handleDprChange);
      }
    }
  }

  function removeDprWatcher() {
    if (dprMedia && typeof dprMedia.removeEventListener === "function") {
      dprMedia.removeEventListener("change", handleDprChange);
    }
    dprMedia = null;
    dprMediaValue = null;
  }

  function handleDprChange() {
    syncDprWatcher();
    scheduleRender();
  }

  function handleWindowResize() {
    syncDprWatcher();
    scheduleRender();
  }

  function removeNode(node) {
    if (!node) return;
    if (typeof node.remove === "function") node.remove();
    else if (node.parentNode && typeof node.parentNode.removeChild === "function") {
      node.parentNode.removeChild(node);
    }
  }

  // Remove every `#liquidTrackFilter` node, not just the first. A previous
  // partial install (or a duplicated id) can leave more than one behind, and
  // `querySelectorAll` catches them all in a real DOM. The fake/test DOM only
  // implements `querySelector`, so fall back to re-querying in a bounded loop.
  function removeFilterNodes() {
    if (typeof defs.querySelectorAll === "function") {
      const matches = defs.querySelectorAll(`#${FILTER_ID}`);
      if (matches && typeof matches.forEach === "function") {
        matches.forEach((node) => removeNode(node));
        return;
      }
      if (matches && typeof matches.length === "number") {
        for (let index = matches.length - 1; index >= 0; index -= 1) {
          removeNode(matches[index]);
        }
        return;
      }
    }
    for (let guard = 0; guard < 64; guard += 1) {
      const match = queryFilter();
      if (!match) return;
      removeNode(match);
    }
  }

  function install() {
    if (installed) return;
    installed = true;
    generation += 1;
    lastKey = null;
    filter = queryFilter();
    feImage = findChildByTag(filter, "feimage");
    feDisplacementMap = findChildByTag(filter, "fedisplacementmap");
    if (!filter || !feImage || !feDisplacementMap) {
      // Drop any incomplete or duplicate filter first so exactly one complete
      // #liquidTrackFilter remains after the replacement is appended.
      removeFilterNodes();
      const created = createFilter();
      filter = created.nextFilter;
      feImage = created.image;
      feDisplacementMap = created.displacement;
    }
    if (!canvas) {
      canvas = createCanvas();
      if (canvas && canvas.style) canvas.style.display = "none";
    }
    context = canvas && typeof canvas.getContext === "function" ? canvas.getContext("2d") : null;
    if (observer && typeof observer.disconnect === "function") observer.disconnect();
    observer = null;
    if (ResizeObserverCtor) {
      observer = new ResizeObserverCtor(() => scheduleRender());
      if (typeof observer.observe === "function") observer.observe(track);
    }
    if (!windowListenerAttached && windowRef && typeof windowRef.addEventListener === "function") {
      windowRef.addEventListener("resize", handleWindowResize);
      windowListenerAttached = true;
    }
    syncDprWatcher();
    scheduleRender();
  }

  function uninstall() {
    if (!installed) return;
    installed = false;
    generation += 1;
    if (frameId !== null) {
      caf(frameId);
      frameId = null;
    }
    if (observer && typeof observer.disconnect === "function") observer.disconnect();
    observer = null;
    if (windowListenerAttached && windowRef && typeof windowRef.removeEventListener === "function") {
      windowRef.removeEventListener("resize", handleWindowResize);
    }
    windowListenerAttached = false;
    removeDprWatcher();
    removeFilterNodes();
    filter = null;
    feImage = null;
    feDisplacementMap = null;
    lastKey = null;
  }

  function renderNow() {
    if (!installed || !filter || !feImage || !feDisplacementMap) return;
    const rect = typeof track.getBoundingClientRect === "function" ? track.getBoundingClientRect() : null;
    const width = (rect && rect.width) || track.clientWidth || track.offsetWidth || 0;
    const height = (rect && rect.height) || track.clientHeight || track.offsetHeight || 0;
    if (!(width > 0) || !(height > 0)) return;
    const dpr = getDpr();
    const key = `${width}|${height}|${dpr}`;
    if (key === lastKey) return;
    if (!canvas || !context || typeof context.putImageData !== "function" || typeof canvas.toDataURL !== "function") {
      return;
    }
    const field = computeTrackDisplacementField({ width, height, dpr });
    canvas.width = field.width;
    canvas.height = field.height;
    context.putImageData(makeImageData(field.data, field.width, field.height), 0, 0);
    const href = canvas.toDataURL();
    feImage.setAttribute("href", href);
    if (typeof feImage.setAttributeNS === "function") {
      feImage.setAttributeNS(XLINK_NS, "xlink:href", href);
    }
    feImage.setAttribute("width", String(width));
    feImage.setAttribute("height", String(height));
    filter.setAttribute("x", "0");
    filter.setAttribute("y", "0");
    filter.setAttribute("width", String(width));
    filter.setAttribute("height", String(height));
    feDisplacementMap.setAttribute("scale", String(field.scale));
    lastKey = key;
  }

  function scheduleRender() {
    if (!alive || !installed || frameId !== null) return;
    const token = generation;
    frameId = raf(() => {
      frameId = null;
      if (!alive || !installed || token !== generation) return;
      renderNow();
    });
  }

  function applyMode(mode) {
    if (mode === "full") install();
    else uninstall();
  }

  const controller = {
    track,
    render: scheduleRender,
    isAlive() {
      return alive;
    },
    destroy() {
      if (!alive) return;
      alive = false;
      uninstall();
      canvas = null;
      context = null;
      if (typeof unsubscribe === "function") unsubscribe();
      unsubscribe = null;
      controllers.delete(track);
    },
  };

  unsubscribe = subscribe((mode) => applyMode(mode));
  applyMode(typeof getMode === "function" ? getMode() : null);
  controllers.set(track, controller);
  return controller;
}
