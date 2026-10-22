# Live Map Aspect Resize During Sidebar Transition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the Three.js drawing buffer from being stretched during the `650ms` `.app` `grid-template-columns` transition by having the map render loop re-sync the cached drawing buffer in the same animation frame as the camera update and draw, while keeping all observer/window notifications and joystick/HUD work settle-only.

**Architecture:** One new small production helper (`syncMapViewportDuringGridTransition`) beside the map render loop, called from `render()` immediately before `updateCamera()` and `renderer.render(scene, camera)`. It runs only when `pageTransition.active && pageTransition.expectsGrid && mapViewActive() && !glassMotionReduced()`, and it delegates to the already-cached `resizeAll()`, which stays the single `renderer.setSize` call site. `beginLayoutTransition` keeps `expectsGrid` sticky while a grid run is already in flight, so a pure tab/panel transition that begins mid-slide still satisfies this gate and live resize is not interrupted until the grid run settles. `noteLayoutResize` keeps deferring observer/window notifications, `doDeferredLayoutFlush` keeps doing the final settle correction, reduced motion keeps the existing next-frame settle path, and joystick/HUD stay settle-only. No CSS change, no second rAF loop, no changed animation durations.

**Tech Stack:** Vanilla ES module frontend (`static/app.js`), Three.js (untouched), Python `pytest` driving inline Node `-e` harnesses against the real `static/app.js` source.

**Constraints (apply to every task):**
- Work in the current uncommitted `main` working tree. Do **not** create a branch or worktree.
- Do **not** stage or commit anything. There are no commit steps in this plan.
- Do **not** run `colcon` and do **not** run the full test suite.
- Never modify, revert, stage, or commit `config/web_control.yaml`.
- Do not alter `static/styles.css`; the root cause is JS only.
- Run all commands from the repo root: `/home/robot/web_ws/src/web_control`.
- If `from web_control import ...` fails during collection, prefix pytest with `source /opt/ros/humble/setup.bash && source /home/robot/web_ws/install/setup.bash &&`.
- Keep every confirmed animation duration unchanged (`650ms` slide, `480ms` fade, `560ms` thumb, `420ms` reveal, `220ms` hide, `780ms` fallback, `380ms` drop).

**Root cause (for context):** During the `650ms` grid animation the canvas CSS box changes every frame, but `noteLayoutResize()` only sets `pageTransition.pendingFlush = true` and returns while `pageTransition.active`. `doDeferredLayoutFlush()` refuses to resize mid-transition. The render loop keeps drawing into the old-size buffer, so the browser stretches it until the synchronous settle resize/render. The fix corrects the buffer per animation frame only while a grid geometry transition is actually running in a visible map view.

---

### Task 1: RED — extend the render-loop Node harness and the static assertions

**Files:**
- Modify: `test/test_core.py` — `_RENDER_LOOP_NODE_HARNESS` (line 4561), `test_frontend_layout_transition_state_machine_static` (line 1772), `test_frontend_white_flash_fix_present` (line 1835).
- Test: `test/test_core.py::test_map_render_loop_lifecycle_executes_under_node`, `::test_frontend_layout_transition_state_machine_static`, `::test_frontend_white_flash_fix_present`.

- [x] **Step 1: Add live-resize fakes and instrumentation to the render-loop harness**

In `_RENDER_LOOP_NODE_HARNESS`, inside `test/test_core.py`, replace the counter/fake setup:

```python
const frames = new Map();
let nextFrame = 1;
let renderCalls = 0;
let cameraUpdates = 0;
const state = { view: "3d" };
const renderer = { render() { renderCalls += 1; } };
const scene = {};
const camera = {};
const updateCamera = () => { cameraUpdates += 1; };
const requestAnimationFrame = (fn) => { const id = nextFrame++; frames.set(id, fn); return id; };
const cancelAnimationFrame = (id) => { frames.delete(id); };
```

with:

```python
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
const requestAnimationFrame = (fn) => { const id = nextFrame++; frames.set(id, fn); return id; };
const cancelAnimationFrame = (id) => { frames.delete(id); };
```

- [x] **Step 2: Inject the new free variables into the harness Function**

Replace:

```python
const api = new Function(
  "state", "renderer", "scene", "camera", "updateCamera",
  "requestAnimationFrame", "cancelAnimationFrame",
  block + `
return { mapViewActive, render, ensureMapRenderLoop, pauseMapRenderLoop, frameId: () => mapRenderFrame };`,
)(state, renderer, scene, camera, updateCamera, requestAnimationFrame, cancelAnimationFrame);
```

with:

```python
const api = new Function(
  "state", "renderer", "scene", "camera", "updateCamera",
  "requestAnimationFrame", "cancelAnimationFrame",
  "pageTransition", "glassMotionReduced", "resizeAll",
  block + `
return { mapViewActive, render, ensureMapRenderLoop, pauseMapRenderLoop, frameId: () => mapRenderFrame };`,
)(state, renderer, scene, camera, updateCamera, requestAnimationFrame, cancelAnimationFrame,
   pageTransition, glassMotionReduced, resizeAll);
```

- [x] **Step 3: Add the live-resize assertions before the harness success log**

Replace:

```python
// Mapping is a map view and schedules exactly one frame.
state.view = "mapping";
expect(mapViewActive() === true, "mapping is a map view");
ensureMapRenderLoop();
expect(frames.size === 1, "mapping schedules one pending frame");

console.log("render loop ok");
```

with:

```python
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

// Negative gate: a pure view transition with no grid run in flight never
// live-resizes. (A pure begin during an in-flight grid run keeps expectsGrid
// sticky and is covered by the setTab integration harness, not here.)
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
```

- [x] **Step 4: Add the gated-helper and call-order assertions to the layout static test**

In `test_frontend_layout_transition_state_machine_static`, replace:

```python
    assert app_js.count("renderer.setSize(") == 1
    resize_body = _js_function_body(app_js, "function resizeAll()")
    assert "renderer.setSize(" in resize_body
```

with:

```python
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
```

- [x] **Step 5: Add the same call-order assertion to the white-flash static test**

In `test_frontend_white_flash_fix_present`, replace:

```python
    render_body = _js_function_body(app_js, "function render()")
    assert "mapViewActive()" in render_body
    assert "requestAnimationFrame(render)" in render_body
```

with:

```python
    render_body = _js_function_body(app_js, "function render()")
    assert "mapViewActive()" in render_body
    assert "requestAnimationFrame(render)" in render_body
    assert render_body.index("syncMapViewportDuringGridTransition();") < render_body.index(
        "updateCamera();"
    ) < render_body.index("renderer.render(scene, camera);")
```

- [x] **Step 6: Run the focused tests to verify RED**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_map_render_loop_lifecycle_executes_under_node \
  test/test_core.py::test_frontend_layout_transition_state_machine_static \
  test/test_core.py::test_frontend_white_flash_fix_present -v
```

Expected: all three FAIL. The Node harness must fail with
`an active grid transition in a map view resizes the buffer`; the two static tests
must fail with `ValueError: substring not found` while looking for
`syncMapViewportDuringGridTransition();` in the `render()` body (the helper does
not exist yet). Confirm the failures are for the missing live-resize behavior,
not for typos in unrelated assertions.

---

### Task 2: GREEN — add the gated helper and call it from `render()`

**Files:**
- Modify: `static/app.js` — insert the helper between `mapViewActive()` (line 207) and `render()` (line 211), and add one call inside `render()`.

- [x] **Step 1: Add the helper between `mapViewActive()` and `render()`**

In `static/app.js`, replace:

```javascript
function mapViewActive() {
  return state.view === "3d" || state.view === "mapping";
}

function render() {
  if (mapRenderFrame !== null) {
    cancelAnimationFrame(mapRenderFrame);
    mapRenderFrame = null;
  }
  updateCamera();
  renderer.render(scene, camera);
  if (mapViewActive()) {
    mapRenderFrame = requestAnimationFrame(render);
  }
}
```

with:

```javascript
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
    mapRenderFrame = requestAnimationFrame(render);
  }
}
```

- [x] **Step 2: Run the focused tests to verify GREEN**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_map_render_loop_lifecycle_executes_under_node \
  test/test_core.py::test_frontend_layout_transition_state_machine_static \
  test/test_core.py::test_frontend_white_flash_fix_present -v
```

Expected: all three PASS, including the preserved frame scheduling/pause
assertions and the four negative gates.

- [x] **Step 3: Re-run the page-transition Node harnesses to prove the deferral path is untouched**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_frontend_layout_transition_state_machine_executes_under_node \
  test/test_core.py::test_layout_transition_fallback_namespaces_execute_under_node -v
```

Expected: both PASS unchanged. `noteLayoutResize` still defers, the settle flush
still calls `resizeAll` once, and reduced motion still settles on the next frame.

---

### Task 3: README documentation and resize-cache regression test

**Files:**
- Modify: `README.md:46` (sidebar resize bullet), `README.md:61` (`无白闪的延后 resize` bullet).
- Modify: `test/test_core.py` — `test_readme_documents_liquid_glass_transitions` (line 6173).
- Note: the existing `docs/superpowers/plans/2026-09-23-slower-transitions-joystick-reveal.md` was checked and contains **no** settle-only resize claim (it only documents the joystick reveal lifecycle that waits for `doDeferredLayoutFlush`), so it needs no edit.

- [x] **Step 1: Extend the failing README assertions**

In `test_readme_documents_liquid_glass_transitions`, after:

```python
    assert "--joystick-hide-ms" in readme
```

add:

```python
    assert "无白闪的实时 resize" in readme
    assert "渲染循环每帧" in readme
    assert "唯一的 `renderer.setSize` 调用点" in readme
```

- [x] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest test/test_core.py::test_readme_documents_liquid_glass_transitions -v
```

Expected: FAIL on `无白闪的实时 resize` because the README still says the
pre-fix settle-only behavior.

- [x] **Step 3: Update the sidebar resize bullet**

In `README.md`, replace line 46:

```markdown
- 地图区域上的 `ResizeObserver` 与窗口 resize 在布局过渡期间只标记“待刷新”，过渡结算时才调用一次 Three.js `resizeAll()`，随后同步渲染一帧；画布宽高比与摇杆布局因此不会逐帧重建，也不会在过渡中途调整绘制缓冲区。
```

with:

```markdown
- 地图区域上的 `ResizeObserver` 与窗口 resize 在布局过渡期间只标记“待刷新”，不直接重建；当地图视图正处于会改变网格的侧栏/视图过渡时，渲染循环每帧在更新相机与绘制之前调用一次带缓存的 `resizeAll()`，保证绘制缓冲区与相机投影同帧校正、旧画面不被拉伸；过渡结算时仍执行最终 `resizeAll()` 校正，摇杆布局与 HUD 位置只在结算帧更新。纯视图过渡与降低动态模式下仍走原有的下一帧结算路径。
```

- [x] **Step 4: Update the liquid-glass resize bullet**

In `README.md`, replace line 61:

```markdown
- **无白闪的延后 resize**：布局过渡期间不调用 `renderer.setSize`；`ResizeObserver` 与窗口 resize 只标记待刷新，结算时只执行一次 `resizeAll()`，随后在同一任务内同步 `renderer.render`。画布清屏色 `setClearColor(0x0b0f19, 1)` 与 CSS `--scene-bg` 都匹配 `scene.background`；地图视图隐藏时暂停渲染循环，恢复时先同步渲染一帧。
```

with:

```markdown
- **无白闪的实时 resize**：`ResizeObserver` 与窗口 resize 在布局过渡期间只标记待刷新（结算时做最终校正）。当地图视图处于会改变网格的过渡（`pageTransition.active && expectsGrid`）且未降低动态时，渲染循环每帧在 `updateCamera()` 与 `renderer.render()` 之前调用带缓存的 `resizeAll()`——它仍是唯一的 `renderer.setSize` 调用点——因此绘制缓冲区与相机投影总在同一动画帧内先校正再绘制，旧缓冲区不会被拉伸到过渡结束。摇杆布局与 HUD 位置仍只在结算帧更新，纯视图过渡与 `prefers-reduced-motion: reduce` 仍走下一帧结算路径。画布清屏色 `setClearColor(0x0b0f19, 1)` 与 CSS `--scene-bg` 都匹配 `scene.background`；地图视图隐藏时暂停渲染循环，恢复时先同步渲染一帧。
```

- [x] **Step 5: Run the README test to verify it passes**

Run:

```bash
python3 -m pytest test/test_core.py::test_readme_documents_liquid_glass_transitions -v
```

Expected: PASS.

---

### Task 4: Focused verification and preservation checks

**Files:**
- No source edits. Verification only.

- [x] **Step 1: Syntax-check the frontend**

Run:

```bash
node --check static/app.js
```

Expected: exit 0, no output.

- [x] **Step 2: Confirm the single `renderer.setSize` call site is preserved**

Run:

```bash
grep -c "renderer.setSize(" static/app.js
```

Expected: `1`.

- [x] **Step 3: Confirm the helper and call order are in place**

Run:

```bash
grep -n "syncMapViewportDuringGridTransition\|function render()" static/app.js
```

Expected: the helper is defined between `mapViewActive` and `render`, and
`render()` calls it before `updateCamera()`.

- [x] **Step 4: Check the whitespace/conflict markers**

Run:

```bash
git diff --check
```

Expected: no output, exit 0.

- [x] **Step 5: Run the important related pytest tests**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_map_render_loop_lifecycle_executes_under_node \
  test/test_core.py::test_resize_all_skips_identical_drawing_buffer_under_node \
  test/test_core.py::test_frontend_layout_transition_state_machine_static \
  test/test_core.py::test_frontend_layout_transition_state_machine_executes_under_node \
  test/test_core.py::test_layout_transition_fallback_namespaces_execute_under_node \
  test/test_core.py::test_frontend_white_flash_fix_present \
  test/test_core.py::test_frontend_sidebar_and_system_use_shared_collapse \
  test/test_core.py::test_system_tab_uses_shared_sidebar_collapse_model \
  test/test_core.py::test_sidebar_toggle_is_single_accessible_button \
  test/test_core.py::test_sidebar_collapse_state_toggles_without_persistence_under_node \
  test/test_core.py::test_sidebar_system_entry_focuses_visible_tab_under_node \
  test/test_core.py::test_joystick_visibility_lifecycle_executes_under_node \
  test/test_core.py::test_set_tab_reentrant_grid_defers_joystick_reveal_under_node \
  test/test_core.py::test_mapping_joystick_resize_handle_stays_inside_viewport_under_node \
  test/test_core.py::test_readme_documents_liquid_glass_transitions -v
```

Expected: every selected test PASSES or is skipped only when `node` is absent
(`node --version` must be available). Do not run the full suite and do not run
`colcon`.

- [x] **Step 6: Confirm the untouched protected file**

Run:

```bash
git status --short
```

Expected: `config/web_control.yaml` is not listed as newly modified by this work;
the changed files are `README.md`, `static/app.js`, `test/test_core.py`, and the
new plan file (plus anything that was already dirty before this task started).
Do not stage or commit.

---

## Self-Review

**Spec coverage:**
- Helper near the render loop → Task 2 Step 1.
- Gate on `pageTransition.active && expectsGrid && mapViewActive() && !glassMotionReduced()`, where `expectsGrid` stays sticky across a pure tab/panel begin while a grid run is already in flight → Task 2 Step 1 + Task 1 Step 4.
- `resizeAll()` as the only `renderer.setSize` site → Task 4 Step 2 + Task 1 Step 4.
- Called from `render()` before `updateCamera()` and `renderer.render()` → Task 1 Steps 4-5 + Task 2 Step 1.
- `noteLayoutResize` still defers and `doDeferredLayoutFlush` still finalizes → Task 2 Step 3.
- No second rAF loop, joystick/HUD settle-only, reduced motion next-frame path → Task 2 Step 1 + Task 3 README.
- Durations unchanged, no CSS edit → Constraints + Task 4 Step 4 (no CSS diff).
- README and slower-transitions plan → Task 3.
- Node harness RED/GREEN plus negative gates and preserved scheduling → Task 1 Steps 1-6, Task 2 Step 2.
- Static white-flash/layout assertions → Task 1 Steps 4-5.

**Placeholder scan:** No TBD/TODO placeholders; every edit shows the complete literal code.

**Type consistency:** `syncMapViewportDuringGridTransition`, `pageTransition.active`, `pageTransition.expectsGrid` (kept sticky by `beginLayoutTransition` via `existingGridInFlight || Boolean(options.gridChanged)`), `mapViewActive`, `glassMotionReduced`, `resizeAll`, `updateCamera`, `renderer.render(scene, camera)`, and `renderer.setSize` are spelled identically in the production code, the Node harness, and the static assertions.
