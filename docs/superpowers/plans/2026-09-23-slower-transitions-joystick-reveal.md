# Slower Transitions and Joystick Reveal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Slow the page/layout, viewport fade, and liquid tab/thumb animations to the confirmed durations, make layout settlement geometry-aware so only the app's own grid slide can settle early (with a `780ms` fallback for pure view/content transitions), and replace the joystick's measure-while-hidden jump with a token-guarded concealed-then-position-then-reveal lifecycle that can defer its reveal while the sidebar grid is still sliding.

**Architecture:** Three focused edits to the existing no-dependency frontend. `static/styles.css` splits the single `--layout-transition-ms` token into `--layout-slide-ms` / `--fade-ms` / `--thumb-ms` plus `--joystick-reveal-ms` / `--joystick-hide-ms`, adds non-overshoot `--layout-ease`, gives the joystick a `pointer-events: none` base plus `.joystick-concealed` / `.joystick-revealed` states. `static/app.js` makes settlement geometry-aware: `beginLayoutTransition({ gridChanged })` records whether the transition expects the app grid and keeps that expectation sticky while a grid run is already in flight (`existingGridInFlight || Boolean(options.gridChanged)`), so a later pure tab/panel begin cannot stop live resize mid-slide; only a non-pseudo-element `grid-template-columns` `transitionend` on the `.app` element may settle early, and only while `expectsGrid` is true. Opacity, transform, thumb pseudo-element, child, and stale pure-view events cannot settle; pure content/view transitions use the `780ms` fallback, and reduced motion settles on the next animation frame. `applyJoystickLayout` rejects hidden and zero-sized measurements, and the joystick show/hide lifecycle is a token-guarded state machine in the existing "Mapping joystick" block whose show path can defer its reveal (`pendingReveal` / `deferReveal`) until `doDeferredLayoutFlush` repositions at settled geometry. Existing static and Node-harness tests in `test/test_core.py` are updated, and `README.md` timing docs are refreshed. No new dependency, no refactor, no persistence change.

**Tech Stack:** Vanilla ES modules, CSS custom properties/transitions, Three.js (untouched), Python `pytest` driving Node `-e` harnesses.

**Constraints (apply to every task):**
- Do **not** run `colcon`.
- Do **not** stage or commit anything. There are no commit steps in this plan.
- Preserve the pre-existing unrelated `config/web_control.yaml` edit (`teleop_max_linear_speed: 1.0`). Never edit that file.
- Do not animate `filter` or `backdrop-filter`; the restrained easing applies to `opacity`, `transform`, and `grid-template-columns` only.
- Visual reveal/hide state is memory-only; joystick size/position persistence (`persistJoystickPreferences` / `/api/ui/preferences`) stays exactly as-is.
- Run all commands from the repo root: `/home/robot/web_ws/src/web_control`.
- If `from web_control import ...` fails, prefix commands with `source /opt/ros/humble/setup.bash && source /home/robot/web_ws/install/setup.bash &&`.

**Timing contract locked by this plan:**

| Transition | Token | Value |
| --- | --- | --- |
| page/layout slide (`grid-template-columns`) | `--layout-slide-ms` | `650ms` |
| viewport/panel fade (`opacity`/`transform`) | `--fade-ms` | `480ms` |
| liquid tab/thumb slide | `--thumb-ms` | `560ms` |
| decorative water-drop keyframe | `--glass-drop-ms` | `380ms` (unchanged) |
| joystick reveal | `--joystick-reveal-ms` / `JOYSTICK_REVEAL_MS` | `420ms` |
| joystick hide | `--joystick-hide-ms` / `JOYSTICK_HIDE_MS` | `220ms` |
| layout settle fallback | `LAYOUT_SETTLE_FALLBACK_MS` | `780ms` |
| shared layout/fade/joystick easing | `--layout-ease` | `cubic-bezier(0.4, 0, 0.2, 1)` |

---

### Task 1: Split timing tokens and add restrained layout easing (CSS)

**Files:**
- Modify: `static/styles.css:40-42` (root tokens), and the transition declarations at lines `134`, `208-209`, `276-278`, `286-288`, `739-740`, `768-769`, `789-790`, `840-842`, `851-853`, `986-987`, `1464-1465`, `1499-1514`.
- Test: `test/test_core.py` — `test_glass_tokens_and_mode_classes` (1981), `test_glass_reduced_motion_and_transparency_fallbacks` (2136), `test_glass_water_drop_motion` (2152), `test_sliding_selectors_use_transform_based_ios_spring_motion` (2346), `test_mode_slider_uses_contained_spring_to_stay_inside_track` (2441), `test_frontend_view_motion_css_contract` (1717), `test_sidebar_toggle_is_single_accessible_button` (7374).

- [ ] **Step 1: Update the failing static assertions first**

In `test/test_core.py`, make these exact replacements before touching CSS.

Replace the token tuple in `test_glass_tokens_and_mode_classes` (currently lines 1986-1994):

```python
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
```

Replace the two reduced-motion token assertions in `test_glass_reduced_motion_and_transparency_fallbacks` (lines 2142-2143):

```python
    assert "--layout-slide-ms: 1ms;" in motion
    assert "--fade-ms: 1ms;" in motion
    assert "--thumb-ms: 1ms;" in motion
    assert "--joystick-reveal-ms: 1ms;" in motion
    assert "--joystick-hide-ms: 1ms;" in motion
    assert "--glass-drop-ms: 1ms;" in motion
    assert ".joystick {" in motion
```

Replace line 2156 in `test_glass_water_drop_motion`:

```python
    assert "transition: transform var(--thumb-ms) var(--glass-drop-ease)," in styles_css
```

Replace line 2357 in `test_sliding_selectors_use_transform_based_ios_spring_motion`:

```python
    assert "transition: transform var(--thumb-ms) var(--glass-drop-ease)" in styles_css
```

Replace line 2450 in `test_mode_slider_uses_contained_spring_to_stay_inside_track`:

```python
    assert "transition: transform var(--thumb-ms) var(--glass-drop-ease)" in segmented_before
```

In `test_frontend_view_motion_css_contract`, the function already assigns `viewport`, `active`, `root`, `panel`, and `app` from `_css_rule_block`. Only the assertion lines change:

- line 1724 `assert "var(--glass-drop-ease)" in viewport` → `assert "var(--layout-ease)" in viewport`
- insert after it: `assert "var(--fade-ms)" in viewport`
- line 1729 `assert "var(--glass-drop-ease)" in active` → `assert "var(--layout-ease)" in active`
- insert after it: `assert "var(--fade-ms)" in active`
- line 1743 `assert "--layout-transition-ms: 380ms;" in root` → replace with:

```python
    assert "--layout-slide-ms: 650ms;" in root
    assert "--fade-ms: 480ms;" in root
    assert "--thumb-ms: 560ms;" in root
    assert "--layout-ease: cubic-bezier(0.4, 0, 0.2, 1);" in root
```

- line 1745-1746 `.panel`:
  - `assert "transform var(--layout-transition-ms) var(--glass-drop-ease)" in panel` → `assert "transform var(--fade-ms) var(--layout-ease)" in panel`
  - insert after it: `assert "opacity var(--fade-ms) var(--layout-ease)" in panel`
- line 1748-1749 `.app`:
  - `assert "grid-template-columns var(--layout-transition-ms) var(--glass-drop-ease)" in app` → `assert "grid-template-columns var(--layout-slide-ms) var(--layout-ease)" in app`

Replace line 7384 in `test_sidebar_toggle_is_single_accessible_button`:

```python
    assert "grid-template-columns var(--layout-slide-ms) var(--layout-ease)" in styles_css
```

- [ ] **Step 2: Run the tests to verify they fail**

Run:

```bash
python3 -m pytest test/test_core.py::test_glass_tokens_and_mode_classes test/test_core.py::test_glass_reduced_motion_and_transparency_fallbacks test/test_core.py::test_glass_water_drop_motion test/test_core.py::test_sliding_selectors_use_transform_based_ios_spring_motion test/test_core.py::test_mode_slider_uses_contained_spring_to_stay_inside_track test/test_core.py::test_frontend_view_motion_css_contract test/test_core.py::test_sidebar_toggle_is_single_accessible_button -v
```

Expected: FAIL (missing `--layout-slide-ms`, missing `--thumb-ms`, `var(--glass-drop-ease)` still present in `.viewport`).

- [ ] **Step 3: Replace the root timing tokens**

In `static/styles.css`, replace lines 40-42:

```css
  --glass-drop-ms: 380ms;
  --glass-drop-ease: cubic-bezier(0.18, 1.32, 0.32, 1);
  --layout-transition-ms: 380ms;
```

with:

```css
  --glass-drop-ms: 380ms;
  --glass-drop-ease: cubic-bezier(0.18, 1.32, 0.32, 1);
  --layout-slide-ms: 650ms;
  --fade-ms: 480ms;
  --thumb-ms: 560ms;
  --joystick-reveal-ms: 420ms;
  --joystick-hide-ms: 220ms;
  --layout-ease: cubic-bezier(0.4, 0, 0.2, 1);
```

- [ ] **Step 4: Repoint every transition to the split tokens**

In `static/styles.css` make these exact replacements.

Line 134 (`.app`):

```css
  transition: grid-template-columns var(--layout-slide-ms) var(--layout-ease);
```

Lines 208-209 (`.tabs::before`, `.sliding-tabs::before`):

```css
  transition: transform var(--thumb-ms) var(--glass-drop-ease),
              box-shadow var(--fade-ms) ease;
```

Lines 276-278 (`.viewport`):

```css
  transition: opacity var(--fade-ms) var(--layout-ease),
              transform var(--fade-ms) var(--layout-ease),
              visibility 0s linear var(--fade-ms);
```

Lines 286-288 (`.viewport.active`):

```css
  transition: opacity var(--fade-ms) var(--layout-ease),
              transform var(--fade-ms) var(--layout-ease),
              visibility 0s linear 0s;
```

Lines 739-740 (`.panel`):

```css
  transition: opacity var(--fade-ms) var(--layout-ease),
              transform var(--fade-ms) var(--layout-ease);
```

Lines 768-769 (`.sidebar-toggle`):

```css
  transition: opacity var(--fade-ms) ease,
              visibility 0s linear 0s;
```

Lines 789-790 (`.app.system-mode .sidebar-toggle`):

```css
  transition: opacity var(--fade-ms) ease,
              visibility 0s linear var(--fade-ms);
```

Lines 840-842 (`.panel-view`):

```css
  transition: opacity var(--fade-ms) var(--layout-ease),
              transform var(--fade-ms) var(--layout-ease),
              visibility 0s linear var(--fade-ms);
```

Lines 851-853 (`.panel-view.active`):

```css
  transition: opacity var(--fade-ms) var(--layout-ease),
              transform var(--fade-ms) var(--layout-ease),
              visibility 0s linear 0s;
```

Lines 986-987 (`.segmented::before`):

```css
  transition: transform var(--thumb-ms) var(--glass-drop-ease),
              box-shadow var(--fade-ms) ease;
```

Lines 1464-1465 (`.sliding-tabs::after`, `.segmented::after`):

```css
  transition: transform var(--thumb-ms) var(--glass-drop-ease),
              opacity 140ms ease;
```

Do **not** change the `glassDropScale` animation at line 1435 (`animation: glassDropScale var(--glass-drop-ms) var(--glass-drop-ease);`); the decorative water-drop keyframe stays at `380ms`.

- [ ] **Step 5: Update the reduced-motion block**

Replace lines 1499-1514 in `static/styles.css` with:

```css
@media (prefers-reduced-motion: reduce) {
  :root {
    --layout-slide-ms: 1ms;
    --fade-ms: 1ms;
    --thumb-ms: 1ms;
    --joystick-reveal-ms: 1ms;
    --joystick-hide-ms: 1ms;
    --glass-drop-ms: 1ms;
  }

  .sliding-tabs::before,
  .segmented::before,
  .sliding-tabs::after,
  .segmented::after,
  .viewport,
  .panel-view,
  .app,
  .panel,
  .sidebar-toggle,
  .joystick {
    transition-duration: 1ms;
  }
```

Leave the remainder of the reduced-motion block (glass-drop animation off, background animation off, `.system-meter-fill`) unchanged.

- [ ] **Step 6: Run the tests to verify they pass**

Run the Step 2 command again.

Expected: PASS (7 passed).

- [ ] **Step 7: Stop and report**

Report the files changed and the passing command. Do not stage or commit.

---

### Task 2: Geometry-aware layout settlement and the 780ms fallback (JS)

**Files:**
- Modify: `static/app.js:1923` (`LAYOUT_SETTLE_FALLBACK_MS`), `static/app.js:1925-1936` (`pageTransition`), `static/app.js:1949-1967` (`beginLayoutTransition`), `static/app.js:1969-1980` (`settleLayoutTransition`), `static/app.js:2008-2018` (`handleLayoutTransitionEnd`).
- Test: `test/test_core.py` — `test_frontend_layout_transition_state_machine_static` (1772) and `_LAYOUT_TRANSITION_NODE_HARNESS` (4251).

- [ ] **Step 1: Update the failing static and Node assertions first**

In `test_frontend_layout_transition_state_machine_static`, the settle-trigger assertions become geometry-aware:

```python
    assert "const LAYOUT_SETTLE_FALLBACK_MS = 780;" in app_js

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
```

In `_LAYOUT_TRANSITION_NODE_HARNESS`, replace the old "tracked transitionend" block with the geometry-aware cases:

```js
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

// Reduced motion settles on the next animation frame, not the fallback timer.
reduceMotion = true;
resetCounters();
beginLayoutTransition();
flushTimers();
expect(resizeCalls === 0, "reduced motion ignores the fallback timer");
flushFrames();
expect(resizeCalls === 1, "reduced motion settles on the next frame");
reduceMotion = false;
```

- [ ] **Step 2: Run the tests to verify they fail**

Run:

```bash
python3 -m pytest test/test_core.py::test_frontend_layout_transition_state_machine_static test/test_core.py::test_frontend_layout_transition_state_machine_executes_under_node -v
```

Expected: FAIL (`LAYOUT_SETTLE_FALLBACK_MS` is `420`, `expectsGrid` does not exist, and the old handler lets a `transform` event settle the Node transition early).

- [ ] **Step 3: Make settlement geometry-aware and raise the fallback**

In `static/app.js`, keep the fallback and add the geometry expectation to the controller:

```js
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
```

`beginLayoutTransition` records whether the transition expects app grid geometry, keeps an already-in-flight grid expectation sticky across a later pure begin, and keeps the reduced-motion next-frame path:

```js
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
```

`settleLayoutTransition` clears the flag when the transition ends:

```js
  pageTransition.active = false;
  pageTransition.expectsGrid = false;
```

Replace the property check in `handleLayoutTransitionEnd` with the strict event filter:

```js
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
```

- [ ] **Step 4: Run the tests to verify they pass**

Run the Step 2 command again.

Expected: PASS (2 passed).

- [ ] **Step 5: Stop and report**

Report the files changed and the passing command. Do not stage or commit.

---

### Task 3: Joystick concealed-then-position-then-reveal lifecycle (CSS + JS)

**Files:**
- Modify: `static/styles.css:576-614` (`.joystick` base, `[hidden]`, `.joystick-concealed`, `.joystick-revealed`).
- Modify: `static/app.js:1369-1472` (`JOYSTICK_REVEAL_MS` / `JOYSTICK_HIDE_MS`, `mappingJoystickVisibility`, `cancelMappingJoystickVisibility`, `concealMappingJoystick`, `revealMappingJoystick`, `scheduleMappingJoystickReveal`, `flushPendingMappingJoystickReveal`, `setMappingJoystickVisible`), `static/app.js:1541-1587` (`applyJoystickLayout`).
- Test: `test/test_core.py` — new `test_joystick_reveal_and_hide_css_contract` (6588), new `_JOYSTICK_VISIBILITY_NODE_HARNESS` / `test_joystick_visibility_lifecycle_executes_under_node` (6639/6813), update `_MAPPING_JOYSTICK_NODE_HARNESS` (6818) and `test_mapping_joystick_exposes_adjustable_layout_controls` (7142).

- [ ] **Step 1: Write the failing static contract test**

Append this test to `test/test_core.py` immediately after `test_mapping_joystick_is_a_mapping_only_sibling_overlay`:

```python
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
    assert "pointer-events: auto;" in revealed
    assert "var(--joystick-reveal-ms)" in revealed
    assert "var(--layout-ease)" in revealed

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
    assert "if (joystick.hidden) {" in app_js
    assert "if (!width || !height) {" in app_js
    assert "flushPendingMappingJoystickReveal();" in app_js
```

- [ ] **Step 2: Write the failing Node lifecycle test**

Append this harness and test to `test/test_core.py` immediately after the new static test:

```python
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
let nextTimer = 1;
let nextFrame = 1;
global.setTimeout = (fn) => { const id = nextTimer++; timers.set(id, fn); return id; };
global.clearTimeout = (id) => { timers.delete(id); };
global.requestAnimationFrame = (fn) => { const id = nextFrame++; frames.set(id, fn); return id; };
global.cancelAnimationFrame = (id) => { frames.delete(id); };
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
```

`_run_app_node_harness` is defined at `test/test_core.py:7462` (module scope), so it is available to a test defined earlier in the file.

- [ ] **Step 3: Run both new tests to verify they fail**

Run:

```bash
python3 -m pytest test/test_core.py::test_joystick_reveal_and_hide_css_contract test/test_core.py::test_joystick_visibility_lifecycle_executes_under_node -v
```

Expected: FAIL (missing `.joystick-concealed` CSS, missing `mappingJoystickVisibility`, no `JOYSTICK_REVEAL_MS`).

- [ ] **Step 4: Add the joystick reveal/hide CSS**

In `static/styles.css`, replace lines 571-587:

```css
.joystick {
  --joystick-size: 180px;
  position: absolute;
  left: 0;
  top: 0;
  z-index: 6;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 6px;
  touch-action: none;
  user-select: none;
  /* Any not-yet-revealed state is inert; only .joystick-revealed re-enables. */
  pointer-events: none;
  opacity: 0;
  transform: translate3d(0, 12px, 0) scale(0.96);
  transition: opacity var(--joystick-hide-ms) var(--layout-ease),
              transform var(--joystick-hide-ms) var(--layout-ease);
}

.joystick[hidden] {
  display: none;
}

.joystick.joystick-concealed {
  opacity: 0;
  pointer-events: none;
  transform: translate3d(0, 12px, 0) scale(0.96);
  transition: opacity var(--joystick-hide-ms) var(--layout-ease),
              transform var(--joystick-hide-ms) var(--layout-ease);
}

.joystick.joystick-revealed {
  opacity: 1;
  pointer-events: auto;
  transform: translate3d(0, 0, 0) scale(1);
  transition: opacity var(--joystick-reveal-ms) var(--layout-ease),
              transform var(--joystick-reveal-ms) var(--layout-ease);
}
```

- [ ] **Step 5: Guard `applyJoystickLayout` against hidden and zero-sized measurements**

In `static/app.js`, inside `applyJoystickLayout`, reject the hidden `display:none` case and the degenerate viewport/joystick dimensions after the `!joystick || !viewport` early return:

```js
function applyJoystickLayout() {
  const joystick = joystickContainer();
  const viewport = joystickViewport();
  applyJoystickSize();
  if (!joystick || !viewport) return;
  // A hidden joystick is display:none, so every offset is zero. Measuring here
  // would persist a bogus anchor and show the joystick at the wrong place until
  // the next settle flush.
  if (joystick.hidden) {
    updateHudPlacement();
    return;
  }
  const viewportWidth = viewport.clientWidth || 0;
  const viewportHeight = viewport.clientHeight || 0;
  if (!viewportWidth || !viewportHeight) {
    updateHudPlacement();
    return;
  }
  const width = joystick.offsetWidth || 0;
  const height = joystick.offsetHeight || 0;
  if (!width || !height) {
    // A zero-sized joystick has no meaningful anchor; writing coordinates here
    // would place it from a degenerate measurement.
    updateHudPlacement();
    return;
  }
```

Leave the rest of the function unchanged.

- [ ] **Step 6: Replace `setMappingJoystickVisible` with the token-guarded lifecycle**

In `static/app.js`, replace the old `setMappingJoystickVisible` with the token-guarded state machine, including the deferred-reveal path used while the sidebar grid is still sliding:

```js
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
    if (teleop.active || teleop.stopRequired) await stopTeleop("tab");
    return;
  }

  // Expose the joystick while still visually concealed so it can be measured at
  // its final size, positioned, and only then faded/translated into place.
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
```

`setSidebarCollapsed` passes `beginLayoutTransition({ gridChanged: true })`; `setTab` passes `beginLayoutTransition({ gridChanged: sidebarChanged })` and calls `setMappingJoystickVisible(true, { deferReveal: deferJoystickReveal })`, where `deferJoystickReveal = sidebarChanged || (pageTransition.active && pageTransition.expectsGrid)`. An immediate 3d -> Mapping reveal therefore still positions/reveals without waiting, while a System -> Mapping reveal with a changing sidebar — or a re-entrant Mapping click during an in-flight grid-expecting transition — stays concealed until `doDeferredLayoutFlush` repositions at settled geometry and calls `flushPendingMappingJoystickReveal`. `applyJoystickLayout` is declared later in the module but function declarations hoist, so the call is valid.

- [ ] **Step 7: Update the existing pointer-lifecycle Node harness stubs**

The `_MAPPING_JOYSTICK_NODE_HARNESS` slices only the "Mapping joystick" block and now needs the new collaborators. In its `makeElement()` (currently lines 6826-6851) add `inert`, `attributes`, `classList.add`, `classList.remove`, and `setAttribute` so it reads:

```js
function makeElement() {
  return {
    hidden: false,
    inert: false,
    textContent: "",
    className: "",
    style: {},
    captured: [],
    listeners: {},
    attributes: {},
    classList: {
      values: new Set(),
      toggle(name, on) {
        if (on) this.values.add(name);
        else this.values.delete(name);
      },
      add(name) { this.values.add(name); },
      remove(name) { this.values.delete(name); },
      contains(name) { return this.values.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    setPointerCapture(id) { this.captured.push(id); },
    addEventListener(name, handler) { this.listeners[name] = handler; },
    getBoundingClientRect() { return { left: 0, top: 0, width: 100, height: 100 }; },
  };
}
```

Immediately after `global.fetchWithTimeout = async () => ({ ok: true });` (line 6914) add:

```js
global.glassMotionReduced = () => true;
global.applyJoystickLayout = () => {};
```

`glassMotionReduced = () => true` keeps the existing `expect(elements.mappingJoystick.hidden === true, ...)` assertion valid: the hide path commits `hidden` synchronously under reduced motion.

- [ ] **Step 8: Extend the adjustable-layout static test**

In `test_mapping_joystick_exposes_adjustable_layout_controls`, after the `applyJoystickLayout` assertion add:

```python
    assert "if (joystick.hidden) {" in app_js
    assert "if (!width || !height) {" in app_js
    assert "const mappingJoystickVisibility = {" in app_js
    assert "function scheduleMappingJoystickReveal(" in app_js
    assert "function flushPendingMappingJoystickReveal(" in app_js
```

- [ ] **Step 9: Run the joystick tests to verify they pass**

Run:

```bash
python3 -m pytest test/test_core.py::test_joystick_reveal_and_hide_css_contract test/test_core.py::test_joystick_visibility_lifecycle_executes_under_node test/test_core.py::test_mapping_joystick_is_a_mapping_only_sibling_overlay test/test_core.py::test_mapping_joystick_pointer_lifecycle_and_throttling_execute_under_node test/test_core.py::test_mapping_joystick_exposes_adjustable_layout_controls test/test_core.py::test_mapping_joystick_layout_mode_persists_and_clamps_under_node test/test_core.py::test_mapping_joystick_layout_entry_race_under_node test/test_core.py::test_mapping_joystick_idle_layout_entry_requests_zero_stop_under_node test/test_core.py::test_mapping_joystick_resize_handle_stays_inside_viewport_under_node test/test_core.py::test_mapping_joystick_ignores_stale_preference_responses_under_node -v
```

Expected: PASS (10 passed).

- [ ] **Step 10: Stop and report**

Report the files changed and the passing command. Do not stage or commit.

---

### Task 4: README timing documentation

**Files:**
- Modify: `README.md:45`, `README.md:58`, and add one bullet to the "Liquid Glass 页面过渡" section.
- Test: `test/test_core.py` — `test_readme_documents_liquid_glass_transitions` (6061).

- [ ] **Step 1: Extend the failing README assertions**

In `test_readme_documents_liquid_glass_transitions`, after the `saveData` assertion (line 6072) add:

```python
    assert "650ms" in readme
    assert "--layout-slide-ms" in readme
    assert "--fade-ms" in readme
    assert "--thumb-ms" in readme
    assert "420ms" in readme
    assert "220ms" in readme
    assert "--joystick-reveal-ms" in readme
    assert "--joystick-hide-ms" in readme
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest test/test_core.py::test_readme_documents_liquid_glass_transitions -v
```

Expected: FAIL (`650ms` is documented as `380ms` and the split tokens are absent).

- [ ] **Step 3: Update the sidebar timing line**

In `README.md`, line 45, replace:

```markdown
- 折叠时网格中的侧边栏轨道在约 `380ms` 内过渡到 `0px`，面板内容同时淡出并轻微右移；展开时反向播放，切换按钮跟随移动的边界，不会跳到新位置。
```

with:

```markdown
- 折叠时网格中的侧边栏轨道在约 `650ms` 内过渡到 `0px`，面板内容在 `480ms` 内淡出并轻微右移；展开时反向播放，切换按钮跟随移动的边界，不会跳到新位置。
```

- [ ] **Step 4: Update the reduced-motion bullet and add the joystick lifecycle bullet**

Replace line 58 of `README.md`:

```markdown
- **降低动态与透明度**：`prefers-reduced-motion: reduce` 把 `--layout-slide-ms`、`--fade-ms`、`--thumb-ms`、`--joystick-reveal-ms`、`--joystick-hide-ms` 和 `--glass-drop-ms` 压到 `1ms`、关闭关键帧与背景动画，状态机在下一帧直接结算；`prefers-reduced-transparency: reduce` 通过 `glass-off` 提供不透明表面。隐藏视图与折叠面板使用 `inert` 和 `aria-hidden`，焦点移到可见的标签或侧栏按钮，`:focus-visible` 为 2px 焦点环，粗指针设备点击目标不小于 44px。
```

Immediately after that line, insert:

```markdown
- **摇杆显隐生命周期**：进入 **Mapping** 时摇杆先以 `opacity: 0` 挂载并按最终尺寸与位置完成一次测量，再在 `--joystick-reveal-ms`（`420ms`）内淡入并轻微上移；离开 **Mapping** 时立即停止 teleop 并禁用指针交互，随后在 `--joystick-hide-ms`（`220ms`）内淡出，最后才设为 `hidden`。`--layout-ease: cubic-bezier(0.4, 0, 0.2, 1)` 为页面滑动、淡入淡出和摇杆显隐提供无过冲的收尾；快速切换标签页时用同一令牌作废过期的 `requestAnimationFrame`、隐藏定时器与结算回调，过期的显示帧不会把已隐藏的摇杆重新显示。显隐状态只存在当前页面内存，摇杆尺寸与位置持久化行为不变。
```

- [ ] **Step 5: Run the test to verify it passes**

Run:

```bash
python3 -m pytest test/test_core.py::test_readme_documents_liquid_glass_transitions -v
```

Expected: PASS.

- [ ] **Step 6: Stop and report**

Report the file changed and the passing command. Do not stage or commit.

---

### Task 5: Focused verification and preservation checks

**Files:**
- No source edits. Verification only.

- [ ] **Step 1: Run the full focused suite**

Run:

```bash
python3 -m pytest test/test_core.py -k "glass_tokens or glass_reduced_motion or glass_water_drop or glass_drop or view_motion or sliding_selectors or mode_slider_uses_contained or layout_transition or sidebar_toggle_is_single or readme_documents_liquid or joystick or frontend_layout_transition or frontend_mounts_viewports or frontend_white_flash or frontend_safety_invariants" -v
```

Expected: all selected tests PASS. If any Node test is skipped, confirm `node --version` reports a version (it is available at `/home/robot/.nvm/versions/node/v24.21.0/bin/node`).

- [ ] **Step 2: Confirm no CSS timing token was left behind**

Run:

```bash
grep -rn -- "--layout-transition-ms" static README.md test || echo "no stale --layout-transition-ms references"
```

Expected: `no stale --layout-transition-ms references`.

- [ ] **Step 3: Confirm no implementation file animates filter/backdrop-filter**

Run:

```bash
grep -n "transition:.*filter" static/styles.css || echo "no filter transitions"
```

Expected: `no filter transitions`.

- [ ] **Step 4: Confirm the pre-existing unrelated config edit is preserved, and nothing is staged**

Run:

```bash
git status --porcelain
git diff -- config/web_control.yaml
git diff --cached --stat
```

Expected:
- `git status --porcelain` lists the intended files (`static/styles.css`, `static/app.js`, `test/test_core.py`, `README.md`, and the new plan file) plus the pre-existing `M config/web_control.yaml`.
- `git diff -- config/web_control.yaml` shows only `-    teleop_max_linear_speed: 1.5 ...` / `+    teleop_max_linear_speed: 1.0 ...`.
- `git diff --cached --stat` prints nothing (nothing staged).

Do not stage or commit. Report the final status and test output.

---

## Self-Review

**Spec coverage (each confirmed design point → task):**
- page/layout slide `650ms` → Task 1 (`--layout-slide-ms`, `.app`).
- viewport/panel fade `480ms` → Task 1 (`--fade-ms`, `.viewport`, `.panel-view`, `.panel`).
- liquid tab/thumb `560ms` → Task 1 (`--thumb-ms`, thumb + after-dot transforms).
- joystick reveal `420ms` / hide `220ms` → Task 1 tokens + Task 3 classes and `JOYSTICK_REVEAL_MS` / `JOYSTICK_HIDE_MS`.
- layout fallback settlement `780ms` → Task 2 (`LAYOUT_SETTLE_FALLBACK_MS`).
- restrained non-overshoot easing → Task 1 (`--layout-ease` on slide/fade and Task 3 joystick reveal/hide).
- fix the actual joystick jump (measured while hidden, zero offsets, wrong anchor, repositions at settle) → Task 3 Step 5 hidden + zero-size guard, plus Task 3 Step 6 measure-while-concealed.
- show lifecycle: expose concealed → position at final geometry → fade/translate/scale in → Task 3 Step 6 immediate (`3d -> Mapping`) branch.
- deferred show lifecycle: remain concealed/inert with `pendingReveal` while the sidebar grid slides, then `doDeferredLayoutFlush` positions at settled geometry and `flushPendingMappingJoystickReveal` schedules the reveal → Task 2 `doDeferredLayoutFlush` assertion + Task 3 Step 6 `deferReveal` / `scheduleMappingJoystickReveal` / `flushPendingMappingJoystickReveal`.
- hide lifecycle: stop teleop and disable interaction immediately → fade out → hidden → Task 3 Step 6 hide branch.
- generation token for rAF/timers/settle on rapid tab switching → Task 3 `mappingJoystickVisibility.token` (reveal frame + hide timer + pending reveal) and Task 2 `pageTransition.token` settle path.
- visual state not persisted; joystick size/position persistence unchanged → Task 3 keeps `persistJoystickPreferences` untouched and the new static test asserts no `localStorage`/`sessionStorage` in the visibility block.
- reduced-motion path still works → Task 1 Step 5 (`1ms` tokens, `.joystick` transition-duration) and Task 3 reduced-motion synchronous branches (Node test).
- split layout/fade/thumb/joystick timing variables; no filter/backdrop-filter animation → Task 1 and verification Steps 2-3.
- geometry-aware settlement: `beginLayoutTransition({ gridChanged })` records `expectsGrid` and keeps it sticky while a grid run is in flight (`existingGridInFlight || Boolean(options.gridChanged)`), so a later pure begin cannot clear it; only a non-pseudo-element `grid-template-columns` event on `.app` may settle early; opacity/transform/thumb/child/stale pure-view events cannot → Task 2 static + Node assertions.
- `setSidebarCollapsed` passes `gridChanged: true`; `setTab` passes `sidebarChanged` and `deferReveal: deferJoystickReveal` (sidebar change or in-flight grid transition) → Task 2 and Task 3 static assertions plus the re-entrant grid Node harness.
- base `.joystick` is `pointer-events: none`; only `.joystick-revealed` re-enables → Task 3 Step 1 (static contract) and Step 4 CSS.
- update important static/Node tests, README timing docs → Tasks 1-4.
- no colcon / no staging / no commit / preserve `config/web_control.yaml` → Constraints and Task 5 Steps 4.

**Placeholder scan:** No `TBD`, `TODO`, "implement later", or "similar to Task N" text. Every code step contains the full replacement snippet; every run step has a concrete command and expected result.

**Type consistency:** `--layout-slide-ms`, `--fade-ms`, `--thumb-ms`, `--joystick-reveal-ms`, `--joystick-hide-ms`, `--layout-ease` are used identically in CSS, tests, and README. `LAYOUT_SETTLE_FALLBACK_MS`, `pageTransition.expectsGrid`, `expectsGrid`, `existingGridInFlight`, `JOYSTICK_REVEAL_MS`, `JOYSTICK_HIDE_MS`, `mappingJoystickVisibility` (`token` / `frameId` / `timerId` / `visible` / `pendingReveal`), `cancelMappingJoystickVisibility`, `concealMappingJoystick`, `revealMappingJoystick`, `scheduleMappingJoystickReveal`, `flushPendingMappingJoystickReveal`, `setMappingJoystickVisible`, `deferReveal`, and the `.joystick-concealed` / `.joystick-revealed` classes are spelled the same everywhere they appear. `--glass-drop-ms` and `--glass-drop-ease` remain only for the `glassDropScale` keyframe, matching the unchanged `GLASS_DROP_FALLBACK_MS = 520`.
