# Dynamic Liquid Glass Tab Track Design

## Goal

Replace the flat, filter-free upper-left `#viewTabs` track with a genuinely
transparent, dynamically refracted liquid-glass surface. The whole
`.glass-full .sliding-tabs` border box receives an SVG displacement generated at
its real border-box size and current device pixel ratio, driven by
`ResizeObserver` and window/DPR updates, while the static `url(#liquidDrop)`
asset keeps displacing the mode thumb and the sidebar toggle lens. `glass-light`
gets a strong transparent blur fallback, `glass-off` stays opaque, and nothing
about layout, timing, safety, or the current slower transitions changes.

This design **supersedes two earlier decisions** recorded in
`docs/superpowers/specs/2026-09-22-liquid-glass-page-transitions-design.md`:

1. The decision that the tab bar container (`.sliding-tabs`) must stay
   filter-free ("Off — Translucent fill only, no `backdrop-filter`") is
   superseded. The container now carries the dynamic displacement in full mode
   and a transparent blur in light mode. The reason for the original rule —
   avoiding a nested blur under the displaced thumb — is preserved differently:
   the **thumb itself loses its displacement**, so there is exactly one
   refraction layer on the track instead of two.
2. The decision that this work is a pure conceptual reimplementation requiring
   no upstream notice ("No MIT notice is required while the implementation
   remains a conceptual reimplementation...") is superseded. The dynamic track
   generator now **adapts upstream source code** from
   [`shuding/liquid-glass`](https://github.com/shuding/liquid-glass) (MIT,
   Copyright (c) 2025 Shu Ding). `THIRD_PARTY_NOTICES.md` carries the full MIT
   license and the adaptation description, and `static/track-refraction.js`
   carries an attribution header pointing at it. The remaining pieces (the
   static `LIQUID_DROP_MAP`, the filter markup for `#liquidDrop`, the capability
   helper, the CSS, the transition state machine) remain independently
   implemented.

## Approved Direction

1. The whole `.glass-full .sliding-tabs` track uses
   `backdrop-filter: url(#liquidTrackFilter) blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1)`.
2. Full displacement moves off `.sliding-tabs::before`; the thumb keeps its
   gradient, rim, box-shadow, `transform` slide, and droplet animation so the
   selected tab stays distinct with no nested double refraction.
3. `.glass-full .segmented::before` and `.glass-full .sidebar-toggle-lens` keep
   the existing static `url(#liquidDrop)` displacement.
4. `.glass-light .sliding-tabs` gets a strong transparent blur/saturate
   fallback; `.glass-off` stays opaque.
5. The generated field is a bounded gradient/rim displacement suited to a wide
   pill: neutral through the middle, a rim ramp near the rounded-rect boundary
   whose direction follows the analytic rounded-rectangle inward normal (straight
   on the long edges, sideways on the caps), generated per device pixel with R/G
   encoding and a CSS-pixel `feDisplacementMap` scale. Pixels outside the pill
   stay neutral because CSS `border-radius` clips them.
6. The track becomes genuinely transparent through dedicated tokens, not the
   shared `--glass` token, with a bright edge highlight.
7. Inactive track labels use a readable light token in the translucent modes;
   `glass-off` restores dark labels on an opaque white track in the light
   scheme and keeps light labels on an opaque dark track in the dark scheme.
   The System view has its own surface, so its inactive labels use a dedicated
   scheme-aware `--track-system-label` token in every glass tier; map views keep
   the light `--track-glass-label` over their dark canvas.
8. No size, position, padding, grid, or layout property changes, and no new
   animated filter property.

## Current Behavior

`static/styles.css` defines the tab track as `.tabs` (transparent pill, 4px
padding, grid of three tabs) with a `var(--glass)` fill and `var(--shadow-soft)`
shadow. `static/liquid-glass.js` injects one static `#liquidDrop` filter into
`#liquidGlassDefs defs` in full mode and removes it otherwise. The full tier
applies `url(#liquidDrop)` to `.sliding-tabs::before`, `.segmented::before`, and
`.sidebar-toggle-lens`. There is no dynamic per-size generator, no
`ResizeObserver`, no DPR handling, and no way for other modules to observe glass
mode changes.

## Reference Technique and Adaptation

A single hidden SVG hosts the defs:

```html
<svg id="liquidGlassDefs" class="glass-defs" aria-hidden="true" focusable="false" width="0" height="0">
  <defs></defs>
</svg>
```

`static/track-refraction.js` populates one more filter in full mode:

```html
<filter id="liquidTrackFilter" filterUnits="userSpaceOnUse" x="0" y="0" width="320" height="48" color-interpolation-filters="sRGB">
  <feImage result="trackMap" preserveAspectRatio="none" width="320" height="48"/>
  <feDisplacementMap in="SourceGraphic" in2="trackMap" scale="12" xChannelSelector="R" yChannelSelector="G"/>
</filter>
```

`color-interpolation-filters` is the **hyphenated** SVG attribute. This is a
deliberate correction of the upstream camelCase
`colorInterpolationFilters` (which `setAttribute` does not recognize), and it is
asserted by the static contract test.

The upstream `smoothStep` and `roundedRectSDF` helpers are adapted. Upstream
operated in normalized `[0,1]` coordinates with half-extents that produce an
elliptical, aspect-dependent rim. This work evaluates the same SDF in **CSS
pixels** centered on the track, so a 320x48 pill gets a rim band of constant
pixel width rather than one stretched 6.7x on the long axis.

`computeTrackDisplacementField` walks every device pixel. For in-pill pixels in
the rim band it derives the **analytic rounded-rectangle outward normal** and
negates it for inward refraction. With `qx = |x| - halfWidth + radius` and
`qy = |y| - halfHeight + radius`, a point on a rounded corner
(`qx > 0 && qy > 0`) uses `normalize(qx, qy)`; otherwise the closer straight edge
wins (horizontal when `qx > qy`, vertical otherwise), and `Math.sign(x/y)`
restores the quadrant. A wide pill therefore refracts straight down its long
edges (only `G` moves) and sideways on its caps (only `R` moves). Pixels outside
the pill stay neutral, since CSS `border-radius` clips them and displacing them
would only distort invisible pixels. Each in-pill rim pixel encodes
`R = 0.5 + dx/(2*strength)` and `G = 0.5 + dy/(2*strength)` (neutral `128`), sets
`B = 0`, `A = 255`, and the returned `feDisplacementMap` `scale` is `2*strength`
in CSS pixels. The upstream canvas `putImageData` + `canvas.toDataURL()` +
`feImage.href` pipeline and the `backdrop-filter: url(#id)` technique are
preserved.

## Architecture

### `static/track-refraction.js` (new)

Pure exports (Node-testable, no DOM):

- `smoothStep(a, b, t)` — adapted upstream clamp-and-smoothstep.
- `roundedRectSDF(x, y, width, height, radius)` — adapted upstream
  rounded-box SDF, where `width`/`height` are half extents in the caller's
  units. Negative inside, `0` on the boundary, positive outside.
- `computeTrackDisplacementField({ width, height, dpr, radius, band, strength })`
  — returns `{ width, height, dpr, scale, data }`, where `width`/`height` are
  device pixels, `scale` is the CSS-pixel `feDisplacementMap` scale, and `data`
  is a `Uint8ClampedArray` of `width*height*4`. The interior and the region
  outside the pill are neutral (`R=G=128`); the rim is displaced along the
  negated analytic rounded-rectangle normal, alpha is `255`, and repeated calls
  with the same inputs are byte-identical.

Lifecycle export:

- `initTrackRefraction(options = {})` — resolves the track and defs, subscribes
  to glass mode changes, and returns an idempotent controller. It accepts
  injectable `document`, `track`, `defs`, `ResizeObserver`,
  `requestAnimationFrame`, `cancelAnimationFrame`, `window`, `matchMedia`,
  `getDevicePixelRatio`, `getGlassMode`, `onGlassModeChange`, `createCanvas` and
  `makeImageData`. Production callers pass nothing.

### `static/liquid-glass.js` (modified)

Add `export function onGlassModeChange(listener)` returning an unsubscribe
function. `applyGlassMode(mode)` notifies subscribers after the root classes and
filter injection are updated. Each listener runs inside `try/catch`, so a
throwing listener cannot break glass mode switching or block later listeners.
The existing `glassMode()` getter is reused for the initial sync.

### `static/app.js` (modified)

Import `initTrackRefraction` and call it immediately after `initLiquidGlass()`.
No `setTab`, no state-machine, no timing change.

### `static/styles.css` (modified)

Dedicated tokens:

```css
--track-glass-bg: rgba(255, 255, 255, 0.14);
--track-glass-edge: rgba(255, 255, 255, 0.92);
--track-glass-label: #f8fafc;
--track-system-label: #111827;
```

Dark scheme: `rgba(8, 12, 20, 0.16)`, `rgba(255, 255, 255, 0.36)`, `#f5f7fb`,
and `--track-system-label: #f5f7fb`. `glass-off` overrides: opaque white / bright
edge / `#111827` in the light scheme; opaque dark / dim edge / `#f5f7fb` in the
dark scheme. `--track-system-label` keeps the System view's inactive labels dark
on its light surface in the light scheme and light on its dark surface in the
dark scheme, in every glass tier.

Tier rules:

```css
:where(.glass-full, .glass-light) .sliding-tabs {
  background: var(--track-glass-bg);
  border-color: var(--track-glass-edge);
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.25), inset 0 -10px 25px rgba(0, 0, 0, 0.15);
}

.glass-full .sliding-tabs {
  backdrop-filter: url(#liquidTrackFilter) blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1);
  -webkit-backdrop-filter: url(#liquidTrackFilter) blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1);
}

.glass-light .sliding-tabs {
  backdrop-filter: blur(22px) saturate(1.4);
  -webkit-backdrop-filter: blur(22px) saturate(1.4);
}

.glass-full .sliding-tabs .tab:not(.active),
.glass-light .sliding-tabs .tab:not(.active) {
  color: var(--track-glass-label);
}

.glass-off .sliding-tabs {
  background: var(--track-glass-bg);
  border-color: var(--track-glass-edge);
}

.glass-off .sliding-tabs .tab:not(.active) {
  color: var(--track-glass-label);
}

.app.system-mode :where(.glass-full, .glass-light, .glass-off) .sliding-tabs .tab:not(.active) {
  color: var(--track-system-label);
}

.glass-full .segmented::before,
.glass-full .sidebar-toggle-lens {
  backdrop-filter: url(#liquidDrop) saturate(1.4);
  -webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);
}
```

The `.sliding-tabs::before` rule itself is untouched: gradient, rim, shadow,
`transform`, `will-change: transform`, and the `glassDropScale` animation all
stay. Only its selector is removed from the full-displacement list.

## Lifecycle and Race Handling

- **Full mode** installs (or reuses) the single `#liquidTrackFilter`, creates one
  canvas, observes the track with one `ResizeObserver`, attaches one window
  `resize` listener, and starts one DPR `matchMedia` watcher.
- **Light/off** disconnects the observer and window listener, cancels the
  pending frame, removes the `matchMedia` watcher, and removes the filter. The
  controller stays alive so re-enabling full mode reinstalls cleanly.
- Render requests are coalesced through one `requestAnimationFrame` handle.
- Every scheduled callback captures a generation token. `destroy()` and every
  mode removal bump the generation, so a stale callback that fires later is
  ignored.
- A render whose CSS size and DPR equal the previous successful render is
  deduped and does not rebuild the canvas or the data URL.
- Repeated `initTrackRefraction()` calls for the same track return the same
  controller, so there is never a duplicate filter or observer. Install first
  removes **every** matching `#liquidTrackFilter` node (via `querySelectorAll`
  when available, otherwise a bounded re-query loop) so a leftover incomplete or
  duplicated filter is replaced rather than left beside the new one; the same
  helper removes the filter on mode removal and `destroy()`.
- `destroy()` is idempotent: it removes the filter, disconnects the observer and
  listeners, unsubscribes from mode changes, cancels any frame, and is safe to
  call twice.
- The rim displacement points along the negated analytic rounded-rectangle
  outward normal, so it never pushes outward. Pixels outside the pill stay
  neutral (CSS `border-radius` clips them anyway), so the `feDisplacementMap`
  never samples outside the filter region.

## Constraints

- Front-end only: `static/track-refraction.js` (new), `static/liquid-glass.js`,
  `static/app.js`, `static/styles.css`, `README.md`, `THIRD_PARTY_NOTICES.md`
  (new), `setup.py`, `test/test_core.py`.
- No `config/web_control.yaml` edit, stage, or commit. Its
  `teleop_max_linear_speed` must stay `1.0`.
- No `colcon`. The user compiles.
- No new dependency, bundler, or framework.
- Never stage or commit; leave the tree dirty for review.
- Preserve every current transition duration and all map/joystick/navigation
  behavior.

## Testing Strategy

1. **Static contracts** in `test/test_core.py`: module exports, app wiring
   order, hyphenated filter attribute, CSS tier selectors/tokens (including the
   System `--track-system-label` rule), no layout properties in the new track
   rules, attribution header, `THIRD_PARTY_NOTICES.md`, and `setup.py` packaging.
2. **Pure generator Node harness**: `smoothStep` clamped; `roundedRectSDF`
   sign/symmetry; field determinism, neutral center, displaced rim, straight
   long-edge direction (`R` neutral, `G` inward) far from the caps, cap direction
   (`G` neutral, `R` inward), neutral pixels outside the pill, alpha, DPR device
   dimensions, and DPR-independent CSS-pixel scale.
3. **Lifecycle Node harness** with injected fake DOM/canvas/ResizeObserver/rAF/
   window/matchMedia/subscription: full install, size render, coalescing,
   unchanged-size dedupe, mode-switch removal, stale callback rejection,
   re-enable, no duplicate filter after a pre-seeded incomplete filter,
   idempotent destroy and unsubscribe.
4. **Updated existing glass selector/contract tests**: whole-track dynamic
   filter, thumb no longer displaced, static `url(#liquidDrop)` retained on the
   mode thumb and lens, and the WebGL/text/safety exclusions preserved.
5. **Updated README/notice tests**: dynamic track map, static map still used by
   the mode thumb/sidebar lens, MIT adaptation, and removal of the old
   no-attribution claim.

Verification: `node --check` on the three JS files, the focused
glass/tab/capability/README/packaging tests, the two new Node harnesses, the
adjacent resize/sidebar/transition/joystick regression tests, and
`git diff --check`. The full suite and `colcon` are not run.

## Non-Goals

- No backend, Python, ROS, launch, or configuration change.
- No dynamic per-frame map generation; the map is rebuilt only on a size or DPR
  change.
- No displacement or blur on the WebGL canvas, terminal/log text, dense data,
  the live joystick knob, or safety action buttons.
- No change to teleoperation, navigation, map selection, or joystick
  persistence.
- No persistence of glass mode or track geometry.

## Acceptance Criteria

1. `static/track-refraction.js` exports `smoothStep`, `roundedRectSDF`,
   `computeTrackDisplacementField`, and `initTrackRefraction`, and carries an
   attribution header pointing at `THIRD_PARTY_NOTICES.md`.
2. In full mode the whole `.sliding-tabs` track references
   `url(#liquidTrackFilter) blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1)`
   through both `backdrop-filter` and `-webkit-backdrop-filter`.
3. `.sliding-tabs::before` is no longer in any displacement selector list and
   keeps its existing gradient, rim, shadow, transform, and droplet animation.
4. `.glass-full .segmented::before` and `.glass-full .sidebar-toggle-lens` keep
   the static `url(#liquidDrop)` displacement.
5. `glass-light` gives the track a strong transparent blur; `glass-off` is
   opaque and uses readable label tokens in both color schemes. The System view's
   inactive labels use `--track-system-label` so they stay dark on its light
   surface in the light scheme while map views keep light
   `--track-glass-label` labels.
6. `ResizeObserver`/window/DPR updates are coalesced and token-guarded; repeated
   init never duplicates the filter, removes any leftover incomplete filter, and
   mode removal cleans up; `destroy()` is idempotent.
7. The generated field is deterministic, neutral at the center and outside the
   pill, follows the analytic rounded-rectangle inward normal at the rim (R
   neutral on the long edges, G neutral on the caps), stays alpha-255,
   is device-sized by DPR, and its `feDisplacementMap` scale is in CSS pixels.
8. No test regresses the WebGL/text/safety no-filter guarantees, the live resize
   behavior, sidebar overflow clipping, layout transitions, or joystick
   visibility.
9. `config/web_control.yaml` still reads `teleop_max_linear_speed: 1.0` and is
   untouched.
