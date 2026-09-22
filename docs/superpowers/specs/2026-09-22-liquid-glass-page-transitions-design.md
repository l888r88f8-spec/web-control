# Liquid Glass Page Transitions Design

## Goal

Give the web control console a compact, low-cost "liquid glass" transition
language: water-drop motion on decorative active thumbs, restrained light glass
on structural surfaces, and glitch-free transitions between the Navigation,
Mapping, and System views plus the information sidebar. The design fixes the
white flash that appears when the Three.js canvas is resized during a layout
transition, keeps the map and System viewports mounted, and preserves every
teleoperation, map-selection, and joystick-persistence guarantee that exists
today.

This is a front-end-only production design. It touches three existing static
files, adds one static file, and updates the existing front-end contract tests.
It does not touch application Python, ROS, or configuration.

## Approved Direction

The following decisions are fixed and are not re-opened by this document:

1. Reference the technique in
   [`shuding/liquid-glass`](https://github.com/shuding/liquid-glass) but
   implement a compact low-cost version instead of copying its dynamic
   per-element generator.
2. Three treatment tiers: full SVG displacement on decorative active thumbs
   only; light glass on structural surfaces; no displacement on WebGL, text,
   dense data, or live/safety controls.
3. Shared hidden SVG defs, a small `static/liquid-glass.js` capability helper,
   Chromium-only full mode, light or off fallback for other engines, save-data,
   and reduced motion or transparency.
4. A medium water-drop feel of about `380ms` with elastic easing, a slight
   translate/scale, a soft meniscus impression, and a small edge droplet. The
   expensive filter properties themselves are never animated.
5. Fix the white flash by prohibiting `renderer.setSize` during layout
   transitions, deferring the stage resize and joystick clamp, then performing
   one resize and one synchronous `renderer.render` at settle, plus a canvas
   background that matches the scene.
6. Make the map/sidebar full-width change smooth.
7. Keep the map and System viewports mounted and switch them with
   opacity/visibility/pointer-events/inert, never `display: none`/`block`.
8. Let the sidebar leave and return smoothly with the System view while
   preserving the in-page collapsed state.
9. Drive every tab/toggle change through one transition state machine that
   tolerates rapid and reentrant actions and has a `transitionend` timeout
   fallback.
10. Preserve teleop stop safety, map interaction, selected-map semantics, and
    joystick persistence.
11. Meet accessibility requirements: inert hidden content, visible focus,
    reduced-motion support, and touch targets of at least `44px`.
12. Respect the performance rules: no filter on the WebGL canvas, no stacked
    per-section backdrop filters, and no per-frame shader-map generation.
13. Preserve the unrelated uncommitted `config/web_control.yaml` change; never
    edit, stage, or commit it.
14. The user compiles the ROS workspace. Implementation verification never runs
    `colcon`.

## Current Behavior and Defects Addressed

- `static/styles.css` switches views with `.viewport { display: none }` and
  `.viewport.active { display: block }`, so Navigation, Mapping, and System cut
  instantly instead of crossfading.
- `.app.system-mode .panel { display: none }` and
  `.app.system-mode .sidebar-toggle { display: none }` remove the sidebar from
  the DOM flow instantly on System, and restore it instantly on exit.
- `.app` transitions `grid-template-columns`, while the `ResizeObserver` in
  `observeMapStageResize()` calls `resizeAll()` on every observed frame.
  `resizeAll()` immediately calls
  `renderer.setSize(width, height, false)`. Because the renderer is created with
  `alpha: true` and the canvas has no opaque CSS background, the frames between a
  drawing-buffer resize and the next WebGL clear show the light stage/body
  background: the white flash.
- Backdrop filters are stacked. `.panel` and `.panel section` both apply
  `backdrop-filter`; `.tabs` and its `.sliding-tabs::before` thumb both apply
  `backdrop-filter`. Nested blur layers are expensive and compound on every
  repaint.
- The active thumbs transition `filter` (`filter: drop-shadow(...)`) and list
  `filter` in `will-change`, which forces a filter repaint on every animation
  frame.
- There is no `prefers-reduced-motion`, `prefers-reduced-transparency`, or
  save-data handling anywhere in the stylesheet or the module.

## Scope and Change Map

| File | Change |
| --- | --- |
| `static/index.html` | Add one hidden shared SVG defs host. Add a `panel-views` wrapper and one decorative lens layer for the sidebar toggle. Do not remove `#view3d` or `#viewSystem`. |
| `static/styles.css` | Add glass-mode classes and tokens, the mounted-viewport visibility model, the shared sidebar/System collapse model, tier rules, water-drop motion, focus and touch rules, and fallback rules. Remove the animated `filter` transition and the stacked per-section backdrop filters. |
| `static/app.js` | Import and initialize the capability helper. Add the layout-transition state machine, deferred resize and synchronous settle render, effective sidebar collapse, viewport inert/focus handling, and render-loop gating. |
| `static/liquid-glass.js` (new) | Capability detection, root class application, manual override, and one-time injection of the shared filter and static displacement map. |
| `test/test_core.py` | Update static-contract assertions and add focused Node harnesses for capability fallback, mounted viewports, rapid transitions, resize coalescing, and synchronous settle rendering. |

`setup.py` installs every file under `static/` through the existing glob, so the
new module requires no packaging change. No application Python source or
configuration file changes as part of this work.

## Attribution and License Decision

The reference project is
[`shuding/liquid-glass`](https://github.com/shuding/liquid-glass), described as a
"Copy-paste Liquid Glass effect using SVG filters." Its repository carries the
MIT License, Copyright (c) 2025 Shu Ding, and its README credits
[`shuding/svg-shaders`](https://github.com/shuding/svg-shaders). The upstream
project computes a displacement map dynamically per element and per layout.

Decision: this work is a **conceptual reimplementation**, not a copy. It uses the
publicly demonstrated idea that a `feImage` displacement map feeding a
`feDisplacementMap` can be referenced from `backdrop-filter: url(#id)` in
Chromium. Everything else is written for this repository: the static map asset,
the filter markup, the capability helper, the CSS, and the state machine. The
upstream dynamic generator is explicitly not ported.

Consequences:

- No MIT notice is required while the implementation remains a conceptual
  reimplementation with no substantial copied code or asset. The repository
  license is unchanged.
- `static/liquid-glass.js` carries a short provenance comment naming the upstream
  repository URL, the technique it demonstrates, and the statement that the
  filter and map here are independently implemented.
- If implementation ever copies a non-trivial block of upstream code, markup, or
  a generated asset, that step must stop and add the upstream MIT copyright and
  permission notice to the new file plus a `THIRD_PARTY_NOTICES` entry before
  review. A code review must not approve copied code without the notice.

## Component Treatment Table

"Full" means the element's backdrop is displaced by the shared SVG filter.
"Light" means a single `backdrop-filter: blur()` glass layer. "Off" means no
displacement and no backdrop filter; the surface uses an opaque or simple
translucent fill.

| Surface (selector) | Tier | Treatment | Rationale |
| --- | --- | --- | --- |
| Active tab thumb (`.sliding-tabs::before`) | Full | Shared `url(#liquidDrop)` displacement, static edge highlight, transform-only slide plus scale pulse. | Decorative, one visible instance, small fixed size. |
| Active mode thumb (`.segmented::before`) | Full | Same shared filter, same motion model. | Decorative, one visible instance, small fixed size. |
| Sidebar toggle lens (`.sidebar-toggle-lens`) | Full | Same shared filter at a smaller contribution. | Reinforces the sidebar transition; the smallest Full surface, listed last so it can be the first surface reduced if a rendering regression appears. |
| Sidebar container (`.panel`) | Light | One `blur()` plus saturate on the container only. | The single structural glass surface for the sidebar. |
| Sidebar sections (`.panel section`) | Light, flattened | Translucent fill and border, no own `backdrop-filter`. | Removes the nested blur inside `.panel`. |
| Sidebar view crossfade (`.panel-view`) | Off for displacement | Opacity crossfade only. | Content swap, not a glass surface. |
| HUD (`.hud`) | Light | One `blur()` glass pill. | Small fixed overlay. |
| Save dialog (`#saveMapDialog`, `#saveMapDialog::backdrop`) | Light | Glass panel plus dimmed backdrop. | Modal emphasis without displacement. |
| System cards (`.system-card`) | Light | One `blur()` each; cards are siblings, never nested. | Readable live data with a single glass layer. |
| CPU core panel (`.cpu-core-panel`) | Light | One `blur()`, sibling of the cards. | Same as cards. |
| Joystick base (`.joystick-base`) | Light | One `blur()` on the base only. | Control housing stays legible. |
| Tab bar container (`.sliding-tabs`) | Off | Translucent fill only, no `backdrop-filter`. | Avoids stacking a container blur under the displaced thumb and keeps Light glass limited to the approved surfaces. |
| Three.js canvas (`#mapCanvas3d`, `#view3d`) | Off | No `filter`, no `backdrop-filter`, opaque scene-matched background. | Avoids WebGL compositing cost and the white flash. |
| Terminal and log text (`.terminal`, `.terminal pre`, `.source`) | Off | Opaque terminal fill, no blur. | Text legibility and cheap scrolling. |
| Dense data (`.metrics`, `.system-meter`, `.cpu-core-row`, `.cpu-core-track`, `.cpu-core-fill`) | Off | Plain fills and the existing width transition. | No glass over dense numbers. |
| Live knob (`.joystick-knob`) | Off | Transform-only pointer tracking. | Zero added latency on the live control. |
| Safety and action labels (`#navigationPrimary`, `#navigationClear`, `#publishPose`, `#localizationProcessToggle`, `#navigationProcessToggle`, `.start-log`, `.stop-log`, `.joystick-settings-button`, `.joystick-done-button`, `#confirmSaveMap`, `#cancelSaveMap`) | Off | Opaque high-contrast fill, no displacement, no blur. | Safety-critical controls must never be visually ambiguous. |
| Controls and inputs (`.map-picker`, `select`, `input`, `.pill`, `.readiness`) | Off | Plain translucent or opaque fills. | Form affordances stay crisp. |

Tier membership is expressed only through selectors in `styles.css` and the root
glass-mode classes; no JavaScript assigns tiers per element.

## Reference Technique and Compact Reimplementation

A single hidden SVG in `index.html` hosts the defs:

```html
<svg id="liquidGlassDefs" class="glass-defs" aria-hidden="true" focusable="false" width="0" height="0">
  <defs></defs>
</svg>
```

The helper populates `<defs>` exactly once, and only in full mode, with one
filter and one static map:

```html
<filter id="liquidDrop" x="-12%" y="-60%" width="124%" height="220%" color-interpolation-filters="sRGB">
  <feImage preserveAspectRatio="none" result="dropMap"/>
  <feDisplacementMap in="SourceGraphic" in2="dropMap" scale="6" xChannelSelector="R" yChannelSelector="G"/>
</filter>
```

The helper sets `feImage.href` to the module constant `LIQUID_DROP_MAP`, which is
a complete `data:image/png;base64` string already defined in
`static/liquid-glass.js`; no value is deferred to a later step. The static map
is a `128x64` RGBA PNG: red encodes horizontal offset and green encodes vertical
offset, both neutral at `128`. The field is flat and neutral through the middle
with a short rim ramp near the pill boundary and a slightly stronger bulge at the
rounded ends. `preserveAspectRatio="none"` stretches the one map across both the
wide tab thumb and the narrower mode thumb, which is why one filter is
sufficient.

The map is never regenerated. It is not rebuilt on resize, on tab change, on
transition, or on any animation frame. There is no per-element map and no
shader. The only per-frame work in full mode is the compositor sliding a small
element whose filter is static.

The hidden SVG must not use `display: none`, because a filter referenced by
`url()` must remain resolvable. The class uses `position: absolute; width: 0;
height: 0; overflow: hidden; pointer-events: none;`.

## DOM Architecture

### Shared defs host

Insert the hidden `<svg id="liquidGlassDefs">` as the first child of `<body>`,
before `<main class="app">`. It is inert decoration and carries no layout.

### Viewports stay mounted

`#view3d` and `#viewSystem` remain two sibling `.viewport` elements inside
`.stage`, in their current order. They are never removed, cloned, or toggled
with `display`. `#view3d` contains the canvas, HUD, and joystick exactly as
today. `#viewSystem` contains the System board exactly as today.

### Sidebar content wrapper

Wrap the two existing panels (`#navigationSidebar` and `#mappingSidebar`) in a
single `<div class="panel-views">` placed after `<header>` inside
`#controlPanel`. The wrapper is `position: relative`; the active `.panel-view`
stays in normal flow, and an inactive `.panel-view` is absolutely positioned and
faded out. This lets sidebar content crossfade without changing the sidebar
column width or the map canvas.

### Sidebar toggle lens

Add one decorative child to `#sidebarToggle`:

```html
<span class="sidebar-toggle-lens" aria-hidden="true"></span>
```

The lens carries the full-displacement backdrop in full mode. The button keeps
its existing role, label, `aria-expanded`, and `aria-controls`, and keeps the
`.sidebar-toggle-arrow` element.

## CSS Architecture

### Tokens

Add to `:root`:

```css
--scene-bg: #0b0f19;
--glass-drop-ms: 380ms;
--glass-drop-ease: cubic-bezier(0.18, 1.32, 0.32, 1);
--layout-transition-ms: 260ms;
--focus-ring: 2px;
--touch-target: 44px;
```

`--scene-bg` matches `scene.background`, currently `0x0b0f19`. The existing
`--ios-spring` and `--ios-contained-spring` tokens stay for controls that
already use them; the new `--glass-drop-ease` is used only by the water-drop
motion.

### Mounted viewport visibility model

Replace the `display` rules for `.viewport`:

```css
.viewport {
  position: absolute;
  inset: 0;
  opacity: 0;
  visibility: hidden;
  pointer-events: none;
  transition: opacity var(--layout-transition-ms) ease,
              visibility 0s linear var(--layout-transition-ms);
}

.viewport.active {
  opacity: 1;
  visibility: visible;
  pointer-events: auto;
  transition: opacity var(--layout-transition-ms) ease,
              visibility 0s linear 0s;
}

#view3d { z-index: 0; background: var(--scene-bg); }
#viewSystem { z-index: 1; }
#mapCanvas3d { background-color: var(--scene-bg); }
```

The delayed `visibility` keeps the outgoing viewport hittable-free and painted
for the fade, then removes it from hit testing and accessibility. `inert` and
`aria-hidden` are applied by JavaScript as described below.

### Shared sidebar and System collapse model

Both the sidebar toggle and entering System drive one effective collapse value.
CSS no longer removes the panel or toggle with `display`:

```css
.app { transition: grid-template-columns var(--layout-transition-ms) ease; }
.app.sidebar-collapsed { --panel-width: 0px; }
.app.sidebar-collapsed .panel,
.app.system-mode .panel { opacity: 0; transform: translateX(12px); pointer-events: none; }
.app.system-mode .sidebar-toggle {
  opacity: 0;
  visibility: hidden;
  pointer-events: none;
  transition: opacity var(--layout-transition-ms) ease,
              visibility 0s linear var(--layout-transition-ms);
}
```

`.app.system-mode` no longer overrides `grid-template-columns` or sets
`display: none` on the panel. It remains only as a semantic hook for
System-specific stage styling and the toggle fade above. Because entering System
adds `.sidebar-collapsed` (through `applyEffectiveSidebar()`), `--panel-width`
animates to `0px` exactly like a manual collapse, so the stage grows smoothly to
full width instead of cutting.

### Sidebar view crossfade

```css
.panel-views { position: relative; }
.panel-view {
  opacity: 0;
  visibility: hidden;
  pointer-events: none;
  transition: opacity var(--layout-transition-ms) ease,
              visibility 0s linear var(--layout-transition-ms);
}
.panel-view.active {
  position: relative;
  opacity: 1;
  visibility: visible;
  pointer-events: auto;
  transition: opacity var(--layout-transition-ms) ease,
              visibility 0s linear 0s;
}
.panel-view:not(.active) { position: absolute; inset: 0; }
```

The active panel defines the wrapper height. The outgoing panel overlays it
while fading, so there is no double-height flash and no `display` change.

### Baseline reset

The tier classes are the single source of truth for blur and displacement. Every
existing base `backdrop-filter` declaration in `styles.css` is removed from the
rules it currently sits on: `.tabs`, `.tabs::before`, `.sliding-tabs::before`,
`.segmented::before`, `.system-card`, `.cpu-core-panel`, `.hud`,
`.joystick-base`, `.panel`, `.sidebar-toggle`, and `.panel section`. The base
rules keep their translucent fills, borders, and shadows, so a page with no glass
class still renders readable opaque-ish surfaces. Removing the base declarations
is what guarantees there is never a stacked layer, because no rule can blur
unless a tier selector adds it.

### Full displacement tier

```css
.glass-full .sliding-tabs::before,
.glass-full .segmented::before,
.glass-full .sidebar-toggle-lens {
  backdrop-filter: url(#liquidDrop) saturate(1.4);
  -webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);
}
```

The displaced thumb is the only glass on the tab bar; the bar itself stays the
translucent fill from the baseline reset.

The toggle lens needs geometry before its filter can show. The button already has
`position: absolute` and rounded ends; the lens fills it and the arrow sits above
it:

```css
.sidebar-toggle-lens {
  position: absolute;
  inset: 0;
  border-radius: inherit;
  overflow: hidden;
  background: var(--glass);
  pointer-events: none;
}
.sidebar-toggle-arrow { position: relative; z-index: 1; }
```

### Light glass tier

The approved Light surfaces are the sidebar container, HUD, dialog, System
cards, CPU core panel, and joystick base. Light glass applies in both full and
light modes, so the two modes differ only by the three displaced decorative
elements.

```css
:where(.glass-full, .glass-light) .panel,
:where(.glass-full, .glass-light) .hud,
:where(.glass-full, .glass-light) #saveMapDialog,
:where(.glass-full, .glass-light) .system-card,
:where(.glass-full, .glass-light) .cpu-core-panel,
:where(.glass-full, .glass-light) .joystick-base {
  backdrop-filter: blur(22px) saturate(1.35);
  -webkit-backdrop-filter: blur(22px) saturate(1.35);
}
```

`.panel section`, `.terminal`, and `.source` never receive a selector here, so
they keep the flat baseline fills and cannot stack under `.panel`.

### Off tier and reduced transparency

With the baseline reset there is nothing for `.glass-off` to unset. It only
swaps the translucent tokens for opaque ones so reduced transparency produces a
solid, high-contrast page:

```css
.glass-off {
  --glass: rgba(255, 255, 255, 0.96);
  --glass-strong: rgba(255, 255, 255, 0.98);
  --panel-bg: rgba(255, 255, 255, 0.98);
  --terminal-bg: rgba(255, 255, 255, 0.98);
}

@media (prefers-color-scheme: dark) {
  .glass-off {
    --glass: rgba(28, 28, 34, 0.98);
    --glass-strong: rgba(44, 44, 52, 0.98);
    --panel-bg: rgba(15, 17, 24, 0.98);
    --terminal-bg: rgba(5, 8, 12, 0.98);
  }
}
```

### Water-drop motion

The glass lens is `.sliding-tabs::before` and `.segmented::before`; the edge
droplet is a new `::after` on each control that shares the same
`--tabs-index`/`--mode-index` translate so it tracks the active thumb.

```css
.sliding-tabs::before,
.segmented::before {
  transform: translate3d(calc(var(--tabs-index, var(--mode-index, 0)) * 100%), 0, 0);
  transition: transform var(--glass-drop-ms) var(--glass-drop-ease),
              box-shadow var(--layout-transition-ms) ease;
  will-change: transform;
}

.sliding-tabs.glass-drop::before,
.segmented.glass-drop::before {
  animation: glassDropScale var(--glass-drop-ms) var(--glass-drop-ease);
}

@keyframes glassDropScale {
  0% { scale: 0.94; }
  55% { scale: 1.035; }
  100% { scale: 1; }
}

.sliding-tabs::after,
.segmented::after {
  content: "";
  position: absolute;
  top: 50%;
  left: 4px;
  width: 9px;
  height: 9px;
  margin-top: -4px;
  border-radius: 50%;
  pointer-events: none;
  opacity: 0;
  transform: translate3d(calc(var(--tabs-index, var(--mode-index, 0)) * 100%), 0, 0) scale(0.4);
  background: radial-gradient(circle at 35% 30%, rgba(255, 255, 255, 0.92), rgba(100, 210, 255, 0.32) 58%, transparent 72%);
  transition: transform var(--glass-drop-ms) var(--glass-drop-ease),
              opacity 140ms ease;
  will-change: transform, opacity;
}

.sliding-tabs.glass-drop::after,
.segmented.glass-drop::after {
  opacity: 0.9;
  transform: translate3d(calc(var(--tabs-index, var(--mode-index, 0)) * 100%), 0, 0) scale(1);
}
```

Rules enforced by this model:

- Only `transform`, `scale`, and `opacity` are animated. `filter`,
  `backdrop-filter`, `-webkit-backdrop-filter`, and `box-shadow` blur are never
  in a transition or animation property list. The existing
  `transition: ... filter 360ms ease` and `will-change: ... filter` are removed.
- The `glass-drop` class is added by JavaScript when the active index changes
  and removed on `animationend` with a timeout fallback. Re-adding it forces an
  animation restart with one layout read (`void element.offsetWidth`).
- The "soft blur" impression is the droplet's opacity ramp and the lens's static
  saturate, not an animated blur.

### Reduce-motion overrides

```css
@media (prefers-reduced-motion: reduce) {
  :root { --layout-transition-ms: 1ms; --glass-drop-ms: 1ms; }
  .sliding-tabs::before,
  .segmented::before,
  .sliding-tabs::after,
  .segmented::after,
  .viewport,
  .panel-view,
  .app,
  .panel { transition-duration: 1ms; }
  .sliding-tabs.glass-drop::before,
  .segmented.glass-drop::before { animation: none; }
  .sliding-tabs.glass-drop::after,
  .segmented.glass-drop::after { opacity: 0; }
  body::before,
  body::after { animation: none; }
  .system-meter-fill { animation: none; }
}
```

The helper also sets the `.glass-motion-reduced` root class so JavaScript can
settle transitions on the next animation frame instead of waiting for a
`transitionend` event that a `1ms`/zero-duration transition may not emit.

### Focus visibility and touch targets

```css
:focus-visible {
  outline: var(--focus-ring) solid var(--ios-blue);
  outline-offset: 2px;
}

@media (pointer: coarse) {
  .tab,
  .mode,
  .actions button,
  .start-log,
  .stop-log,
  .sidebar-toggle,
  .joystick-settings-button,
  .joystick-done-button,
  .map-picker button,
  .save-map-dialog-actions button { min-height: var(--touch-target); }
  .sidebar-toggle { width: var(--touch-target); }
}
```

Focus outlines are additive; no existing `outline: none` is introduced and any
existing suppression on these controls is removed where it would hide keyboard
focus.

## JavaScript Architecture

### `static/liquid-glass.js` (new capability helper)

Public surface:

```js
export function initLiquidGlass(options = {}) // applies root classes, injects defs, installs listeners
export function detectGlassMode()              // returns "full" | "light" | "off"
export function applyGlassMode(mode)           // reapplies classes and filter injection
export function glassMode()                    // reads the current applied mode
export function glassMotionReduced()           // true when prefers-reduced-motion: reduce matches
```

Behavior:

- `detectGlassMode()` evaluates conditions in this fixed order and returns the
  first match:
  1. A manual override is present (`?glass=` with value `full`, `light`, or
     `off` in `location.search`, or a prior
     `window.__webControlSetGlassMode(...)` call): the override value. The
     override is the explicit developer/operator escape hatch and wins over every
     automatic rule; it is absent in normal operation.
  2. `prefers-reduced-transparency: reduce` matches: `"off"`.
  3. `navigator.connection && navigator.connection.saveData === true`:
     `"light"`.
  4. The engine is Chromium (`navigator.userAgentData.brands` contains a
     `Chromium` brand, or `/Chromium\/|Chrome\//` matches
     `navigator.userAgent`) and `CSS.supports` accepts
     `backdrop-filter: url(#liquidDrop)` or the `-webkit-` form: `"full"`.
  5. Otherwise: `"light"`.
- Reduced motion is tracked independently of the glass mode. When
  `matchMedia("(prefers-reduced-motion: reduce)")` matches, the helper adds
  `glass-motion-reduced` to `<html>` and exposes the flag to `app.js` through the
  exported `glassMotionReduced()` function so the state machine can settle
  immediately.
- `applyGlassMode(mode)` sets exactly one of `glass-full`, `glass-light`, or
  `glass-off` on `document.documentElement` and mirrors it in a
  `data-glass-mode` attribute for tests. It injects the single filter into
  `#liquidGlassDefs defs` when the mode is `full`, and removes previously
  injected filter nodes for any other mode. Injection is idempotent: it does
  nothing if the filter already exists.
- Listeners are attached once: `prefers-reduced-motion` and
  `prefers-reduced-transparency` change events, and `navigator.connection`
  `change`. A change reapplies the mode. No listener regenerates the
  displacement map.
- The module contains no `requestAnimationFrame`, no `setInterval`, and no
  `ResizeObserver`. It never reads `localStorage` or `sessionStorage`. The manual
  override is memory-only for the current page load.
- A short provenance comment names the upstream repository and states that the
  filter and map are independently implemented.

### `app.js` integration

- Import at the top of the module and initialize before the render loop starts:
  `import { initLiquidGlass, glassMotionReduced } from "./liquid-glass.js";`
  followed by `initLiquidGlass();`.
- Renderer setup becomes:

  ```js
  const SCENE_BG = 0x0b0f19;
  scene.background = new THREE.Color(SCENE_BG);
  renderer.setClearColor(0x0b0f19, 1);
  ```

  The `alpha: true` construction stays for compatibility, but the clear color is
  fully opaque and matches `--scene-bg`, so a cleared or resized buffer is never
  white.
- Render-loop gating: `render()` continues to call `updateCamera()` and
  `renderer.render(scene, camera)`, but it reschedules the next frame only while
  the map viewport is active. When `state.view` leaves the map views, the
  pending frame is cancelled and no new frame is scheduled. When the map view
  becomes active again, `doDeferredLayoutFlush()` performs one synchronous
  `renderer.render(scene, camera)` before the loop is resumed. This keeps the
  hidden WebGL cost near zero and guarantees a correct settled frame.
- All resize entry points route through the transition state machine:
  `window.addEventListener("resize", noteLayoutResize)`,
  `observeMapStageResize()` calls `noteLayoutResize()`, and the standalone
  `window.addEventListener("resize", applyJoystickLayout)` is removed because
  the deferred flush already applies the joystick layout. The
  `visibilitychange` handler also calls `noteLayoutResize()` when the document
  becomes visible so a backgrounded-then-restored tab catches up.

### Layout transition state machine

A single controller owns every layout-changing action.

```js
// --- Page transition state machine
const LAYOUT_SETTLE_FALLBACK_MS = 420;

const pageTransition = {
  active: false,
  token: 0,
  pendingFlush: false,
  fallbackId: null,
  resizeFrameId: null,
};

function beginLayoutTransition() {
  pageTransition.token += 1;
  const token = pageTransition.token;
  pageTransition.active = true;
  pageTransition.pendingFlush = true;
  if (pageTransition.fallbackId !== null) {
    clearTimeout(pageTransition.fallbackId);
  }
  if (glassMotionReduced()) {
    pageTransition.fallbackId = requestAnimationFrame(
      () => settleLayoutTransition(token),
    );
  } else {
    pageTransition.fallbackId = setTimeout(
      () => settleLayoutTransition(token),
      LAYOUT_SETTLE_FALLBACK_MS,
    );
  }
  return token;
}

function settleLayoutTransition(token) {
  if (token !== pageTransition.token) return; // stale request
  if (pageTransition.fallbackId !== null) {
    cancelAnimationFrame(pageTransition.fallbackId);
    clearTimeout(pageTransition.fallbackId);
    pageTransition.fallbackId = null;
  }
  pageTransition.active = false;
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
  resizeAll();                 // the only renderer.setSize call site
  applyJoystickLayout();
  updateHudPlacement();
  renderer.render(scene, camera); // synchronous settle frame
}
// --- End page transition state machine
```

A single bubbling listener drives settling. The element references are resolved
once at attach time from the existing DOM ids:

```js
const appEl = document.querySelector(".app");
const view3dEl = document.getElementById("view3d");
const viewSystemEl = document.getElementById("viewSystem");
const viewTabsEl = document.getElementById("viewTabs");

appEl.addEventListener("transitionend", (event) => {
  if (!pageTransition.active) return;
  const layoutProps = new Set(["grid-template-columns", "opacity", "transform"]);
  const layoutTargets = new Set([appEl, view3dEl, viewSystemEl, viewTabsEl]);
  if (!layoutProps.has(event.propertyName)) return;
  if (!layoutTargets.has(event.target)) return;
  settleLayoutTransition(pageTransition.token);
});
```

The tab thumb's `transform` covers a Navigation/Mapping swap, `opacity` covers
the System crossfade, and `grid-template-columns` covers sidebar collapse and
System enter/leave. A mode change (`#modeSegment`) animates its own droplet but
is not a layout transition, so it is deliberately not a settling target; its
events are ignored by the `pageTransition.active` guard.

Because `transitionend` can be missed (zero-duration transitions, an element
removed from paint, a throttled background tab, an interrupted transition), the
`420ms` timeout always settles the active token. Rapid actions are safe because
each `beginLayoutTransition()` bumps the token and cancels the previous timeout;
only the newest token may settle and flush.

### Tab transition (`setTab`)

`setTab(tab)` keeps its current responsibilities and ordering, with these
changes:

1. Set `state.view`, `data-active`, and the active classes as today.
2. Call `applyEffectiveSidebar()` so entering or leaving System updates the
   shared collapse model.
3. Call `setMappingJoystickVisible(tab === "mapping")` unchanged. This
   synchronously hides the joystick and triggers the existing
   `stopTeleop("tab")` path when leaving Mapping. Safety state commits
   immediately, never after the transition.
4. Call `exitJoystickLayout()` when the tab is not Mapping, and
   `applyJoystickLayout()` is deferred to the settle flush instead of running
   mid-transition.
5. Toggle the map/System viewport `.active` classes, then set
   `view3d.inert`/`viewSystem.inert` and their `aria-hidden` values from the new
   tab. Toggle `.panel-view.active` as today and set `inert`/`aria-hidden` on
   each panel from whether it is active. Move focus to the newly active tab
   button if focus was inside the viewport that just became hidden.
6. Toggle layer-group visibility as today.
7. Call `beginLayoutTransition()` only when at least one active class actually
   changed, and remove the direct `resizeAll()` calls from the per-tab branches.
   The startup `setTab("3d")` is a no-op and starts no transition. Polling calls
   stay as today.
8. Add the `glass-drop` class to `#viewTabs` when the tab index changed, remove
   it on `animationend`, and re-add it with a forced reflow read for a
   reentrant change.

### Sidebar toggle transition

`setSidebarCollapsed(collapsed)` becomes:

```js
function setSidebarCollapsed(collapsed) {
  sidebarState.collapsed = Boolean(collapsed);
  if (applyEffectiveSidebar()) beginLayoutTransition();
}
```

`applyEffectiveSidebar()` computes
`const effective = sidebarState.collapsed || state.view === "system";` and, when
`effective` differs from the previously applied value:

1. Toggles `.sidebar-collapsed` on `.app` (starting the grid transition).
2. Sets `controlPanel.inert = effective` and mirrors it in `aria-hidden`.
3. Updates `#sidebarToggle` `aria-expanded`, `aria-label`, and arrow direction
   from `sidebarState.collapsed`, the in-page state, not the effective state.
4. Returns `true` so the caller starts exactly one layout transition after all
   state is committed. It returns `false` when the effective value did not
   change, including a plain Navigation/Mapping swap.
5. If focus was inside `#controlPanel` while collapsing, moves focus to
   `#sidebarToggle`.

`setTab` calls `applyEffectiveSidebar()` as one of its state commits and starts
its own single `beginLayoutTransition()` in step 7, so a System transition never
starts two transitions.

The toggle remains a single DOM button. The in-page collapsed state is
memory-only and is never written to the backend, `localStorage`, or
`sessionStorage`.

### System enter and leave

Entering System sets `state.view = "system"`, so `applyEffectiveSidebar()` sees
`effective === true` and animates the sidebar out exactly like a collapse, while
`sidebarState.collapsed` keeps whatever the operator chose. Leaving System sets
`effective` back to `sidebarState.collapsed`, so the sidebar returns only if the
operator had it open. The System board itself crossfades through the mounted
viewport model. `.system-mode` is retained for System-specific stage styling but
no longer removes the panel or the toggle with `display`.

### Safety invariants during transitions

- Leaving Mapping always calls `setMappingJoystickVisible(false)` and
  `exitJoystickLayout()` synchronously before any deferred work, preserving the
  teleop stop and layout-mode exit semantics. Deferring the clamp must never
  defer the stop.
- Map interaction is unaffected: the canvas keeps its pointer listeners, and the
  `inert` flag is only applied to the viewport that is not active.
- Selected-map semantics and the joystick preference API are untouched. The
  design reads no new preference and writes none.
- The joystick clamp runs once at settle, so a saved position stays usable after
  the sidebar or System transition without being fought by frame-by-frame
  clamping.

## State Transitions

### Controller states

| State | Meaning | Entry | Exit |
| --- | --- | --- | --- |
| `stable` | No layout transition is active. | Initial state; `settleLayoutTransition` with the current token. | `beginLayoutTransition()` from any user action. |
| `pending` | A target change is committed to the DOM and the layout transition is running. `pendingFlush` is `true`. | `beginLayoutTransition()`. | `transitionend` for a tracked property/target, or the fallback timer. |
| `settling` | The winning token flushes once. | `settleLayoutTransition(currentToken)`. | Flush completes; return to `stable`. |

### Event rules

| Event | Guard | Effect |
| --- | --- | --- |
| Tab, sidebar, or System request | none | Commit classes/state synchronously, including safety actions, then `beginLayoutTransition()`. A mode change is not a layout transition and does not call it. |
| New request while `pending` | always | Bump token, cancel the previous fallback, keep a single `pendingFlush`. The latest target wins. |
| `transitionend` | `active` and tracked property/target | `settleLayoutTransition(currentToken)`. |
| Fallback timer (`420ms`) | token equals current | `settleLayoutTransition(currentToken)`. |
| Reduced motion | `glassMotionReduced()` | Settle on the next animation frame instead of waiting for `transitionend`. |
| Stale settle | token does not equal current | Ignored; no flush. |
| Resize while `pending` | `active` | Set `pendingFlush = true`; no `setSize` during the transition. |
| Resize while `stable` | not `active` | Coalesce on the next animation frame and flush once. |
| Page hidden then visible | `visibilitychange` | `noteLayoutResize()` so the settled frame is correct. |

### Observable guarantees

- At most one `renderer.setSize` runs per settled transition, followed in the
  same task by one synchronous `renderer.render`.
- A double-click on a tab or the sidebar toggle cannot leave the controller
  stuck `pending`; the newest token always settles by event or timeout.
- A transition interrupted by a second request never fires a stale flush.
- No frame during a transition resizes the canvas drawing buffer.

## Fallback Behavior

| Condition | Glass mode | Motion | Result |
| --- | --- | --- | --- |
| Chromium, `CSS.supports` accepts the SVG-in-backdrop filter, no save-data, no reduced transparency | `full` | Full water drop | SVG displacement on the three decorative thumbs; light glass elsewhere. |
| Firefox, Safari, or an unrecognized engine | `light` | Full water drop | Blur glass on structural surfaces; no displacement; thumbs keep the translate/scale droplet motion. |
| `navigator.connection.saveData === true`, no reduced transparency | `light` | Full water drop | Displacement disabled to save data and battery; light glass remains. |
| `prefers-reduced-transparency: reduce` | `off` | Full water drop | No blur and no displacement; opaque tokens; motion still runs unless reduced motion also applies. |
| `prefers-reduced-motion: reduce` | Current mode | Instant | Durations collapse to `1ms`; drop keyframes off; the state machine settles on the next frame. |
| `?glass=` override with value `full`, `light`, or `off` | Override | Current motion setting | Forces the mode for testing or an operator escape hatch; memory-only. |
| `ResizeObserver` unavailable | Current mode | Full | `observeMapStageResize()` returns without observing, as today; window resize and settle flushes still keep the canvas sized. |
| `inert` unavailable | Current mode | Full | `inert` assignment is a no-op; `pointer-events` and `visibility` still block interaction. |
| JavaScript module fails to load | No class applied | CSS only | CSS defaults render the light/opaque surfaces and the mounted viewports remain usable; no displacement and no deferred-resize flash fix. |

## Accessibility

- Hidden viewports and inactive sidebar panels receive `inert` and
  `aria-hidden="true"`. The active viewport and panel clear both.
- When focus is inside content that becomes hidden, focus moves to the
  corresponding tab button or `#sidebarToggle`, so keyboard users are never
  stranded on an inert element.
- `:focus-visible` renders a visible `2px` `--ios-blue` outline with a `2px`
  offset on every interactive control. No rule in this design removes an
  outline.
- Touch targets on coarse pointers are at least `44x44px` for tabs, mode
  buttons, action buttons, the sidebar toggle, the joystick settings controls,
  and dialog buttons.
- `prefers-reduced-motion` removes slides, scales, the droplet, and the ambient
  background animations, and the state machine settles without waiting for a
  transition event.
- `prefers-reduced-transparency` removes blur and displacement and falls back to
  opaque surfaces, preserving contrast.
- The decorative lens, droplet, and hidden SVG are `aria-hidden` and
  `pointer-events: none`; they never enter the tab order or the accessibility
  tree.

## Performance Budget

- No `filter` or `backdrop-filter` on `#mapCanvas3d` or `#view3d`.
- One `backdrop-filter` layer per structural surface. The nested
  `.panel section` blur and the `.sliding-tabs`/`.segmented` container blur are
  removed in every mode by the baseline reset.
- The displacement map is created once per page load and never regenerated. No
  `requestAnimationFrame`, timer, resize, or transition regenerates it.
- Full displacement is limited to at most three small decorative elements
  (tab thumb, mode thumb, toggle lens). No full-viewport displacement.
- Only `transform`, `scale`, and `opacity` animate. Filter properties are
  static.
- The WebGL render loop is paused while the map viewport is hidden and resumes
  with one synchronous render.
- `renderer.setSize` runs at most once per settled layout transition instead of
  once per `ResizeObserver` frame.
- Under `prefers-reduced-motion`, `body::before`/`body::after` and the CPU meter
  glow stop animating.

## Testing Strategy

### Automated checks

`pytest` source and contract tests in `test/test_core.py`:

- Hidden defs and helper wiring: `index.html` contains
  `id="liquidGlassDefs"`, both `#view3d` and `#viewSystem` remain, `app.js`
  imports `./liquid-glass.js`, and `static/liquid-glass.js` exists and exports
  `initLiquidGlass`, `detectGlassMode`, `applyGlassMode`, `glassMode`, and
  `glassMotionReduced`.
- Capability rules: a Node harness loads the helper and asserts the fixed
  detection order, that an explicit override wins over every automatic rule, that
  reduced transparency returns `off`, that save-data returns `light`, and that a
  non-Chromium user agent returns `light`.
- No persisted glass state: the helper source contains no `localStorage` or
  `sessionStorage`.
- One-time map: the helper exposes exactly one filter injection path, contains
  no `requestAnimationFrame`/`setInterval`/`ResizeObserver`, and the injected
  filter markup uses `feImage` plus `feDisplacementMap` once.
- Tier contract: `styles.css` declares no `filter`/`backdrop-filter` on
  `#mapCanvas3d`; `.joystick-knob`, `.terminal pre`, and the safety action
  selectors are not in the full or light displacement selector lists.
- Mounted viewports: `styles.css` no longer switches `.viewport` with
  `display`; it uses `opacity`, `visibility`, and `pointer-events`, and
  `app.js` assigns `inert` on `#view3d`, `#viewSystem`, and `.panel-view`.
- Shared collapse model: `.app.system-mode .panel` and
  `.app.system-mode .sidebar-toggle` no longer use `display: none`;
  `applyEffectiveSidebar` exists and derives from
  `sidebarState.collapsed || state.view === "system"`.
- Transition state machine: `pageTransition`, `beginLayoutTransition`,
  `settleLayoutTransition`, `noteLayoutResize`, and `doDeferredLayoutFlush`
  exist; `resizeAll` is the only function that calls `renderer.setSize`, and
  after the single startup `resizeAll()` in `start()` the deferred flush is its
  only runtime caller; the fallback timeout constant is present.
- White-flash fix: `setClearColor(0x0b0f19, 1)` and the `--scene-bg` canvas rule
  are present; the `transitionend` listener filters on
  `grid-template-columns`, `opacity`, and `transform`.
- Safety semantics: `setTab` still calls `setMappingJoystickVisible` and
  `exitJoystickLayout` before deferred work; `setSidebarCollapsed` is not
  persisted; selected-map and joystick-preference code is unchanged.
- Accessibility: `:focus-visible`, `@media (pointer: coarse)`, `44px` touch
  target rules, and `prefers-reduced-motion` overrides are present.
- Attribution: the provenance comment names the upstream URL and the
  independent-implementation statement; no upstream LICENSE text is copied while
  the implementation stays a reimplementation.

Node harnesses (`node -e`, matching the existing plan style) exercise the
transition block with a fake DOM, fake `transitionend`, and fake timers:

- Rapid tab requests: two `beginLayoutTransition()` calls in one turn settle
  once, with the second token winning and the first timeout ignored.
- `transitionend` settles an active token; a stale event for an old token does
  nothing.
- No `transitionend`: the `420ms` fallback settles and flushes.
- Reduced motion: settle happens on the next animation frame, with no reliance
  on `transitionend`.
- Flush: exactly one `resizeAll` and exactly one synchronous
  `renderer.render` per settle, and `applyJoystickLayout` runs once.
- Resize storm: multiple `noteLayoutResize()` calls while pending produce one
  flush with no `setSize` during the pending window.

Syntax and compilation checks:

- `node --check static/app.js` and `node --check static/liquid-glass.js`.
- `python3 -m py_compile web_control/*.py`.
- `git diff --check`.

The user runs the ROS workspace build. Verification never runs `colcon`.

### Manual browser matrix

- Chromium desktop: full displacement visible on the tab thumb, mode thumb, and
  toggle lens; no white flash on repeated sidebar toggles, System enter/leave,
  and rapid Navigation/Mapping/System clicking.
- Chromium with `?glass=light` and `?glass=off`: each mode renders correctly.
- Chromium with DevTools "Emulate prefers-reduced-motion: reduce" and
  "prefers-reduced-transparency: reduce": motion collapses and surfaces become
  opaque.
- Firefox and Safari: light glass only, no displacement, no white flash, and the
  water-drop translate/droplet motion still runs.
- Touch emulation: all targets at least `44x44px`; keyboard Tab shows visible
  focus; focus is not trapped in a hidden viewport or collapsed sidebar.
- Teleop: pressing and holding the mapping joystick then switching tabs
  immediately stops the vehicle; the knob returns to center; the stop is not
  delayed by the transition.
- Map interaction: drag, wheel zoom, View/Initial Pose/Goal selection, and the
  selected map all behave as before across transitions.
- Joystick persistence: adjust position/size, switch tabs during the gesture and
  after it, reload, and confirm the saved layout restores and clamps once at
  settle.

## Risks and Mitigations

| Risk | Mitigation |
| --- | --- |
| A moving element with `backdrop-filter: url(#liquidDrop)` repaints its backdrop each frame. | Bound to three small, fixed-size decorative elements; static filter; displacement disabled under save-data and reduced transparency; manual `?glass=light` escape hatch. |
| `CSS.supports` accepts SVG-in-`backdrop-filter` but the runtime paint fails on some Chromium build. | Engine allowlist plus `CSS.supports`; the manual override downgrade path is documented and exposed; residual visual risk is caught in manual review and can be downgraded by flipping the mode. |
| `transitionend` is missed for zero-duration or interrupted transitions. | The `420ms` timeout fallback and the reduced-motion animation-frame settle always flush the active token. |
| Rapid or reentrant tab/toggle clicks stack resizes or leave the controller stuck. | One token, one fallback timer, one `pendingFlush`; stale tokens are ignored. |
| Pausing the render loop hides a needed frame. | Resume performs one synchronous `renderer.render` in the settle flush before restarting the loop. |
| Removing `display: none` from `.panel`/`.panel-view` changes existing layout or test expectations. | Animate the grid through `--panel-width` to zero, keep the `.panel-view.active` selector, and add the panel-views wrapper; update the existing sidebar and System tests to the new assertions. |
| `inert` plus focus relocation regresses keyboard flows. | Focus moves to a deterministic control (active tab button or toggle); a Node/pytest assertion plus manual keyboard pass covers it. |
| Reducing nested blur changes the visual identity of the sidebar. | The container keeps its blur and the sections keep translucent fills and borders; only the redundant inner blur layer is removed. |
| A future edit copies upstream code without a notice. | The provenance comment and the review rule in the Attribution section make the notice a required gate. |

## Non-Goals

- No backend, Python, ROS, launch, or message changes. No `colcon` run.
- No dynamic per-element displacement map, no shader, and no WebGL
  post-processing glass.
- No glass, blur, or displacement on the Three.js canvas, terminal/log text,
  dense data, the live joystick knob, or safety action labels.
- No change to teleoperation speed mapping, `/cmd_vel` safety sequencing,
  process gating, navigation control, map selection, or joystick persistence.
- No new persistence of any kind; the collapsed state and the glass mode stay in
  memory for the page load only.
- No framework, bundler, build step, or new dependency; plain ES modules and
  CSS.
- No animation sound or haptics.
- No commit by the implementer. Codex reviews and commits.
- No edit, stage, or commit of the existing uncommitted
  `config/web_control.yaml` change.

## Implementation Constraints

- Production changes are limited to `static/index.html`, `static/styles.css`,
  `static/app.js`, and the new `static/liquid-glass.js`.
- The implementation plan updates `test/test_core.py` to cover the new behavior.
  This design document itself changes no production file and no test file.
- Do not run `colcon`. The user compiles.
- Preserve the uncommitted `config/web_control.yaml` file byte-for-byte. After
  implementation, `git status --porcelain` must still show exactly
  ` M config/web_control.yaml` for that path, and it must never be staged or
  committed.
- Leave the working tree uncommitted for Codex review.

## Acceptance Criteria

1. Both `#view3d` and `#viewSystem` are present in the DOM at all times, and
   switching tabs crossfades them with opacity/visibility/pointer-events/inert
   and no `display` toggling.
2. Sidebar collapse and System enter/leave animate `--panel-width` to and from
   zero; the sidebar never disappears through `display: none`.
3. Re-entering Navigation or Mapping after System restores the in-page collapsed
   state exactly as the operator left it.
4. Repeated and rapid tab/toggle actions never produce a white flash, and each
   settled transition performs one `renderer.setSize` followed by one
   synchronous `renderer.render`.
5. Full SVG displacement appears only on the tab thumb, the mode thumb, and the
   sidebar toggle lens, and only in Chromium full mode.
6. Light glass appears only on the sidebar container, HUD, dialog, System cards,
   CPU core panel, and joystick base. No nested backdrop filter remains.
7. The WebGL canvas, terminal text, dense data, live knob, and safety labels have
   no displacement and no backdrop filter.
8. Save-data, reduced motion, reduced transparency, non-Chromium engines, and the
   manual override all select the documented fallback behavior.
9. Only `transform`, `scale`, and `opacity` animate; no filter property appears
   in a transition or animation.
10. Leaving Mapping still stops teleoperation synchronously, and map
    interaction, selected-map handling, and joystick persistence are unchanged.
11. Hidden content is inert and `aria-hidden`, focus is relocated when content
    hides, focus is visible, and coarse-pointer targets are at least `44px`.
12. The static displacement map is created once and never regenerated.
13. `node --check` passes for both JavaScript files, Python compiles, and the
    existing plus new tests pass.
14. `config/web_control.yaml` is untouched, and nothing is committed.
