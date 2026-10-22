// Provenance: the static LIQUID_DROP_MAP asset, the #liquidDrop filter markup,
// and this glass-mode helper are independently implemented for this
// repository. The dynamic tab-track generator in track-refraction.js adapts
// code from https://github.com/shuding/liquid-glass (MIT, Copyright (c) 2025
// Shu Ding); see THIRD_PARTY_NOTICES.md. The technique of feeding a static
// feImage displacement map into feDisplacementMap and referencing the filter
// from `backdrop-filter: url(#id)` follows that upstream demonstration.

const FILTER_ID = "liquidDrop";
const SVG_NS = "http://www.w3.org/2000/svg";
const GLASS_CLASSES = ["glass-full", "glass-light", "glass-off"];
const GLASS_MODES = ["full", "light", "off"];

// 128x64 RGBA PNG. R encodes horizontal displacement and G encodes vertical
// displacement, both neutral at 128. The field is flat and neutral through the
// middle with a short rim ramp near the pill boundary and a stronger bulge at
// the rounded ends. It is generated once with the Python generator documented
// in docs/superpowers/plans/2026-09-23-liquid-glass-page-transitions.md and is
// never regenerated at runtime.
const LIQUID_DROP_MAP =
  "data:image/png;base64,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";

let initialized = false;
let currentMode = null;
let manualOverride = null;
let savedMode = null;
let motionQuery = null;
let listenersAttached = false;
const modeListeners = new Set();

function prefersReduceMotion() {
  if (!motionQuery && typeof window !== "undefined" && typeof window.matchMedia === "function") {
    motionQuery = window.matchMedia("(prefers-reduced-motion: reduce)");
  }
  return Boolean(motionQuery && motionQuery.matches);
}

function overrideFromSearch() {
  if (typeof window === "undefined" || !window.location) return null;
  const match = /[?&]glass=(full|light|off)(?:&|$)/.exec(window.location.search || "");
  return match ? match[1] : null;
}

// The strength is chosen only by explicit signals: a runtime/manual override, an
// explicit ?glass= query, or the operator's saved browser-local choice. The
// browser engine, its CSS capabilities, and accessibility media queries never
// remap the mode; unsupported CSS simply degrades in place. Without any explicit
// choice, everyone gets Full.
export function detectGlassMode() {
  if (manualOverride) return manualOverride;
  const queryOverride = overrideFromSearch();
  if (queryOverride) return queryOverride;
  if (savedMode) return savedMode;
  return "full";
}

export function glassMode() {
  return currentMode;
}

export function onGlassModeChange(listener) {
  if (typeof listener !== "function") return () => {};
  modeListeners.add(listener);
  return () => {
    modeListeners.delete(listener);
  };
}

function notifyGlassMode(mode) {
  modeListeners.forEach((listener) => {
    try {
      listener(mode);
    } catch (_error) {
      // A broken subscriber must never break glass mode switching.
    }
  });
}

export function glassMotionReduced() {
  return prefersReduceMotion();
}

function injectLiquidFilter() {
  const defs = document.querySelector("#liquidGlassDefs defs");
  if (!defs || defs.querySelector(`#${FILTER_ID}`)) return;
  const filter = document.createElementNS(SVG_NS, "filter");
  filter.setAttribute("id", FILTER_ID);
  filter.setAttribute("x", "-12%");
  filter.setAttribute("y", "-60%");
  filter.setAttribute("width", "124%");
  filter.setAttribute("height", "220%");
  filter.setAttribute("color-interpolation-filters", "sRGB");
  const image = document.createElementNS(SVG_NS, "feImage");
  image.setAttribute("preserveAspectRatio", "none");
  image.setAttribute("result", "dropMap");
  image.setAttribute("href", LIQUID_DROP_MAP);
  const displacement = document.createElementNS(SVG_NS, "feDisplacementMap");
  displacement.setAttribute("in", "SourceGraphic");
  displacement.setAttribute("in2", "dropMap");
  displacement.setAttribute("scale", "6");
  displacement.setAttribute("xChannelSelector", "R");
  displacement.setAttribute("yChannelSelector", "G");
  filter.append(image, displacement);
  defs.appendChild(filter);
}

function removeLiquidFilter() {
  const filter = document.querySelector(`#liquidGlassDefs #${FILTER_ID}`);
  if (filter && filter.parentNode) filter.parentNode.removeChild(filter);
}

export function applyGlassMode(mode) {
  const resolved = GLASS_MODES.includes(mode) ? mode : detectGlassMode();
  currentMode = resolved;
  const root = document.documentElement;
  GLASS_CLASSES.forEach((name) => {
    root.classList.toggle(name, name === `glass-${resolved}`);
  });
  root.setAttribute("data-glass-mode", resolved);
  if (resolved === "full") injectLiquidFilter();
  else removeLiquidFilter();
  notifyGlassMode(resolved);
  return resolved;
}

function applyMotionClass() {
  const reduced = prefersReduceMotion();
  document.documentElement.classList.toggle("glass-motion-reduced", reduced);
}

// Apply an explicit strength and make it outrank the query and saved choice for
// the rest of the page. Persistence is the caller's responsibility so this
// module stays free of browser storage.
export function setGlassMode(mode) {
  manualOverride = GLASS_MODES.includes(mode) ? mode : null;
  return applyGlassMode(detectGlassMode());
}

function attachListenersOnce() {
  if (listenersAttached) return;
  listenersAttached = true;
  if (motionQuery && typeof motionQuery.addEventListener === "function") {
    motionQuery.addEventListener("change", applyMotionClass);
  }
  window.__webControlSetGlassMode = setGlassMode;
}

export function initLiquidGlass(options = {}) {
  if (initialized) return currentMode;
  initialized = true;
  if (options && GLASS_MODES.includes(options.mode)) manualOverride = options.mode;
  if (options && GLASS_MODES.includes(options.savedMode)) savedMode = options.savedMode;
  prefersReduceMotion();
  applyMotionClass();
  attachListenersOnce();
  return applyGlassMode(detectGlassMode());
}
