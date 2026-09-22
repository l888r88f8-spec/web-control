# Liquid Glass Page Transitions Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the web control console a compact liquid-glass transition language — full SVG displacement on three decorative thumbs, light glass on structural surfaces, water-drop motion, glitch-free mounted Navigation/Mapping/System crossfades and a coordinated sidebar/System collapse — while preserving teleop, `/cmd_vel` sequencing, map interaction, selected-map semantics, and joystick persistence.

**Architecture:** A new `static/liquid-glass.js` ES module detects capability in a fixed order and applies exactly one `glass-full` / `glass-light` / `glass-off` class to `<html>`, injecting one static `feImage`+`feDisplacementMap` filter into a hidden shared defs host. `static/styles.css` expresses tier membership only through selectors and animates only `transform`/`scale`/`opacity`. `static/app.js` gains one layout-transition state machine with a token, a single `pendingFlush`, and a `420ms` fallback; `renderer.setSize` runs only inside `resizeAll()`, which the settle flush calls once, and the map/System viewports stay mounted and crossfade with opacity/visibility/pointer-events/inert.

**Tech Stack:** Browser JavaScript ES modules (`node -e` harnesses, Node ≥ 18 auto ESM syntax detection; verified on Node v24.21.0), HTML, CSS, Python 3.10 / pytest 6.2.5. No bundler, no new dependency, no Python/ROS/launch/config change.

---

## Commit Policy (read first)

The operator's instruction for this plan is to use **frequent focused commits**. Every commit step names its files explicitly and **never stages `config/web_control.yaml`**, which carries an unrelated uncommitted `teleop_max_linear_speed: 1.0` change. Implement directly on `main` (previously authorized). The plan document itself is left uncommitted for Codex review. The user compiles the ROS workspace; **never run `colcon`**.

## Locked interface decisions (used consistently by every task)

- **Upstream reference (provenance only):** `https://github.com/shuding/liquid-glass`. This plan is a conceptual reimplementation. No upstream code, markup, or generated asset is copied, so no MIT notice is added while the implementation stays a reimplementation. `static/liquid-glass.js` carries a provenance comment. If a future edit copies a non-trivial upstream block, stop and add the upstream MIT copyright and permission notice plus a `THIRD_PARTY_NOTICES` entry before review.
- **Module surface:** `initLiquidGlass(options = {})`, `detectGlassMode()`, `applyGlassMode(mode)`, `glassMode()`, `glassMotionReduced()`. Constants: `FILTER_ID = "liquidDrop"`, `LIQUID_DROP_MAP`.
- **Detection order (first match wins):** manual override (`manualOverride` or `?glass=full|light|off` in `location.search`); `prefers-reduced-transparency: reduce` → `"off"`; `navigator.connection.saveData === true` → `"light"`; Chromium engine AND `CSS.supports("backdrop-filter", "url(#liquidDrop)")` (or `-webkit-` form) → `"full"`; otherwise `"light"`.
- **Root classes:** exactly one of `glass-full`, `glass-light`, `glass-off` on `document.documentElement`, mirrored in `data-glass-mode`; `glass-motion-reduced` when `prefers-reduced-motion: reduce` matches. The manual override is memory-only, never persisted.
- **DOM ids/classes:** `#liquidGlassDefs` (`.glass-defs`), `.panel-views`, `.panel-view`, `.sidebar-toggle-lens`, `#view3d`, `#viewSystem`, `#viewTabs`, `#controlPanel`, `#sidebarToggle`, `.glass-drop`.
- **CSS tokens:** `--scene-bg: #0b0f19`, `--glass-drop-ms: 380ms`, `--glass-drop-ease: cubic-bezier(0.18, 1.32, 0.32, 1)`, `--layout-transition-ms: 260ms`, `--focus-ring: 2px`, `--touch-target: 44px`.
- **JS identifiers:** `SCENE_BG`, `mapViewActive`, `ensureMapRenderLoop`, `pauseMapRenderLoop`, `mapRenderFrame`, `LAYOUT_SETTLE_FALLBACK_MS = 420`, `GLASS_DROP_FALLBACK_MS = 520`, `pageTransition`, `beginLayoutTransition()`, `settleLayoutTransition(token)`, `noteLayoutResize()`, `doDeferredLayoutFlush()`, `attachLayoutTransitionListener()`, `handleLayoutTransitionEnd(event)`, `markGlassDrop(element)`, `sidebarState`, `appliedEffectiveCollapse`, `syncSidebarToggle(toggle)`, `applyEffectiveSidebar()`, `setSidebarCollapsed(collapsed)`, `toggleSidebar()`, `focusIfInside(container, target)`.
- **State-machine contract:** at most one `renderer.setSize` per settled transition, followed in the same task by one synchronous `renderer.render`; the newest token wins; stale tokens never flush; `transitionend` is filtered to property `grid-template-columns`/`opacity`/`transform` and target `.app`/`#view3d`/`#viewSystem`/`#viewTabs`; the `420ms` fallback and the reduced-motion animation-frame settle always flush the active token.

## Constraints honored by this plan

- Production changes are limited to `static/index.html`, `static/styles.css`, `static/app.js`, and the new `static/liquid-glass.js`. Tests change only in `test/test_core.py`. `README.md` gains a documentation section.
- No Python application, ROS, launch, `package.xml`, `setup.py`, or config change. `setup.py` already installs every file under `static/` by glob, so the new module needs no packaging change.
- **Never edit, stage, revert, or commit `config/web_control.yaml`.** It must remain ` M config/web_control.yaml` (and nothing else for that path) at every commit.
- Do not run `colcon`. Verification runs only `pytest`, `node --check`, `python3 -m py_compile`, and `git diff --check`.
- Preserve teleop stop safety, `/cmd_vel` sequencing, map interaction, selected-map semantics, and joystick persistence. The sidebar collapse state is browser-memory only.
- Full displacement is limited to `.sliding-tabs::before`, `.segmented::before`, and `.sidebar-toggle-lens`, only in `glass-full`. No `filter`/`backdrop-filter` on the WebGL canvas, terminal text, dense data, the live knob, or safety action labels.
- The static displacement map is generated once, embedded as a module constant, and never regenerated.

**Reading line numbers:** every `(currently ...)` line number refers to the repository state at the moment this plan was written, before any task ran. Later tasks shift those lines. The quoted rule, function, or marker text is the authoritative anchor; use line numbers only to orient.

---

## File Structure

- Modify: `static/index.html` — hidden `#liquidGlassDefs` host as the first child of `<body>`; `sidebar-toggle-lens` span; `panel-views` wrapper around the two sidebar panels.
- Create: `static/liquid-glass.js` — capability detection, root class application, manual override, one-time filter injection, static `LIQUID_DROP_MAP`.
- Modify: `static/styles.css` — tokens, baseline blur reset, full/light/off tiers, water-drop motion, mounted viewport model, shared sidebar/System collapse, panel-view crossfade, focus/touch/reduced-motion rules.
- Modify: `static/app.js` — helper import/init, `SCENE_BG` + `setClearColor`, render-loop gating, resize routing, layout-transition state machine, `setTab`/`setMode`/sidebar rewrite, inert/aria/focus handling, glass-drop marking.
- Modify: `test/test_core.py` — new static-contract tests, two new Node harnesses (capability detection and layout transition), updated sidebar harness, updated system-tab/sidebar/motion contract tests.
- Modify: `README.md` — liquid-glass mode/fallback/motion documentation and updated sidebar-transition notes.

---

## Static displacement map (reproducible, independently generated)

`LIQUID_DROP_MAP` is a `128x64` RGBA PNG. Red encodes horizontal displacement and green encodes vertical displacement, both neutral at `128`. The field is flat and neutral through the middle, with a short smoothstep rim ramp near the pill boundary and a stronger bulge at the rounded left/right ends. `preserveAspectRatio="none"` stretches the one map across the wide tab thumb and the narrower mode thumb, so one filter is sufficient. It is generated once with the Python 3 standard library (`zlib`, `struct`, `base64`) — no Pillow dependency — and embedded verbatim; it is never regenerated at runtime.

Reproduce the exact asset with:

```bash
python3 - <<'PY'
import base64, struct, zlib

W, H = 128, 64
rows = []
for y in range(H):
    v = y / (H - 1)
    row = bytearray()
    for x in range(W):
        u = x / (W - 1)
        ex = min(1.0, min(u, 1.0 - u) / 0.16)
        ey = min(1.0, min(v, 1.0 - v) / 0.24)
        rx = ex * ex * (3.0 - 2.0 * ex)
        ry = ey * ey * (3.0 - 2.0 * ey)
        hx = (1.0 - rx) * (1.0 if u < 0.5 else -1.0) * 0.60
        vy = (1.0 - ry) * (1.0 if v < 0.5 else -1.0) * 0.32
        r = 128 + int(round(127.0 * hx))
        g = 128 + int(round(127.0 * vy))
        row += bytes((r, g, 128, 255))
    rows.append(b"\x00" + bytes(row))

raw = b"".join(rows)

def chunk(tag, data):
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

png = b"\x89PNG\r\n\x1a\n"
png += chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 6, 0, 0, 0))
png += chunk(b"IDAT", zlib.compress(raw, 9))
png += chunk(b"IEND", b"")

print(base64.b64encode(png).decode("ascii"))
PY
```

Expected: one base64 line that starts with `iVBORw0KGgo` and equals the `LIQUID_DROP_MAP` value embedded in Task 1. Verification of the decoded asset: dimensions `128x64`, color type `6` (RGBA), bit depth `8`; center pixel `(128, 128, 128, 255)`; left edge `(204, 128, 128, 255)`; right edge `(52, 128, 128, 255)`; top `(128, 169, 128, 255)`; bottom `(128, 87, 128, 255)`.

---

## Task 1: Hidden defs host, capability helper, and app wiring

**Files:**
- Modify: `static/index.html` (insert before `<main class="app">`, currently line 10)
- Create: `static/liquid-glass.js`
- Modify: `static/app.js` (imports, currently lines 1-2; init call)
- Test: `test/test_core.py`

- [ ] **Step 1: Add the hidden shared defs host**

In `static/index.html`, immediately after `<body>` (currently line 9) and before `<main class="app">` (currently line 10), insert exactly:

```html
    <svg id="liquidGlassDefs" class="glass-defs" aria-hidden="true" focusable="false" width="0" height="0">
      <defs></defs>
    </svg>
```

Do not use `display: none`; a filter referenced by `url()` must stay resolvable. The `.glass-defs` class provides the hiding in Task 2.

- [ ] **Step 2: Write the failing Task 1 tests**

Add these four tests to `test/test_core.py` next to the other front-end contract tests (a good anchor is immediately after `test_frontend_uses_threejs_3d_map_without_2d_tab`, currently line 369). The Node harness constant is placed before its test function.

```python
def test_frontend_hosts_liquid_glass_defs():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")

    defs_start = index_html.index('id="liquidGlassDefs"')
    defs_tag = index_html[index_html.rindex("<svg", 0, defs_start) : index_html.index(">", defs_start)]
    assert 'class="glass-defs"' in defs_tag
    assert 'aria-hidden="true"' in defs_tag
    assert 'focusable="false"' in defs_tag
    assert defs_start < index_html.index('<main class="app">')
    assert 'id="view3d"' in index_html
    assert 'id="viewSystem"' in index_html


def test_liquid_glass_helper_exports_provenance_and_static_assets():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    source = (static_dir / "liquid-glass.js").read_text(encoding="utf-8")

    for name in (
        "initLiquidGlass",
        "detectGlassMode",
        "applyGlassMode",
        "glassMode",
        "glassMotionReduced",
    ):
        assert f"export function {name}" in source
    assert "https://github.com/shuding/liquid-glass" in source
    assert "independently implemented" in source
    assert "data:image/png;base64," in source
    assert source.count('"feImage"') == 1
    assert source.count('"feDisplacementMap"') == 1
    assert "requestAnimationFrame" not in source
    assert "setInterval" not in source
    assert "ResizeObserver" not in source
    assert "localStorage" not in source
    assert "sessionStorage" not in source


def test_liquid_glass_wiring_in_app():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    assert 'import { initLiquidGlass, glassMotionReduced } from "./liquid-glass.js";' in app_js
    assert "initLiquidGlass();" in app_js
```

Then, before the capability test function, add the harness constant and the test:

```python
_LIQUID_GLASS_CAPABILITY_NODE_HARNESS = r"""
const { pathToFileURL } = require("url");
const path = process.env.LIQUID_GLASS;

function stub({
  search = "",
  userAgent = "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 Chrome/120.0 Safari/537.36",
  brands = [{ brand: "Chromium", version: "120" }],
  saveData = false,
  reducedTransparency = false,
  reducedMotion = false,
  supports = true,
} = {}) {
  Object.defineProperty(globalThis, "navigator", {
    configurable: true,
    value: {
      userAgent,
      userAgentData: { brands },
      connection: { saveData, addEventListener() {} },
    },
  });
  globalThis.location = { search };
  globalThis.window = globalThis;
  globalThis.CSS = { supports: () => supports };
  globalThis.matchMedia = (query) => ({
    matches: /prefers-reduced-transparency/.test(query) ? reducedTransparency : reducedMotion,
    addEventListener() {},
  });
  globalThis.document = {
    documentElement: {
      lastMode: null,
      classList: { toggle() {} },
      setAttribute(name, value) { if (name === "data-glass-mode") this.lastMode = value; },
    },
    querySelector() { return null; },
  };
}

async function load(caseName) {
  return import(pathToFileURL(path).href + "?case=" + encodeURIComponent(caseName));
}

function expect(condition, message) { if (!condition) throw new Error(message); }

(async () => {
  // Chromium with CSS.supports and no restrictions is full mode.
  stub();
  let mod = await load("chromium");
  expect(mod.detectGlassMode() === "full", "chromium full mode");

  // Firefox is light even when CSS.supports is optimistic.
  stub({ userAgent: "Mozilla/5.0 (X11; Linux x86_64; rv:128.0) Gecko/20100101 Firefox/128.0", brands: [] });
  mod = await load("firefox");
  expect(mod.detectGlassMode() === "light", "firefox light mode");

  // Save-data downgrades a capable engine.
  stub({ saveData: true });
  mod = await load("save-data");
  expect(mod.detectGlassMode() === "light", "save-data light mode");

  // Reduced transparency disables glass entirely.
  stub({ reducedTransparency: true });
  mod = await load("reduced-transparency");
  expect(mod.detectGlassMode() === "off", "reduced transparency off mode");

  // The query override wins over every automatic rule.
  stub({ search: "?glass=off", reducedTransparency: true, saveData: true });
  mod = await load("override-off");
  expect(mod.detectGlassMode() === "off", "query override wins");
  stub({ search: "?glass=full", reducedTransparency: true });
  mod = await load("override-full");
  expect(mod.detectGlassMode() === "full", "query override can force full");

  // The programmatic override wins over a conflicting query value.
  stub({ search: "?glass=off" });
  mod = await load("override-programmatic");
  mod.initLiquidGlass();
  globalThis.__webControlSetGlassMode("full");
  expect(mod.detectGlassMode() === "full", "programmatic override wins over the query");

  // applyGlassMode records and mirrors the applied mode.
  stub();
  mod = await load("apply-mode");
  mod.applyGlassMode("off");
  expect(mod.glassMode() === "off", "applyGlassMode stores the applied mode");
  expect(globalThis.document.documentElement.lastMode === "off", "data-glass-mode mirrors the mode");

  console.log("liquid glass capabilities ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_liquid_glass_capability_detection_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    liquid_glass = Path(__file__).resolve().parents[1] / "static" / "liquid-glass.js"
    completed = subprocess.run(
        ["node", "-e", _LIQUID_GLASS_CAPABILITY_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "LIQUID_GLASS": str(liquid_glass)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "liquid glass capabilities ok" in completed.stdout
```

- [ ] **Step 3: Run the Task 1 tests to verify they fail**

Run:

```bash
python3 -m pytest -q \
  test/test_core.py::test_frontend_hosts_liquid_glass_defs \
  test/test_core.py::test_liquid_glass_helper_exports_provenance_and_static_assets \
  test/test_core.py::test_liquid_glass_wiring_in_app \
  test/test_core.py::test_liquid_glass_capability_detection_under_node
```

Expected: `test_frontend_hosts_liquid_glass_defs` passes (the host was added in Step 1); the other three FAIL with `FileNotFoundError` for `static/liquid-glass.js` and a missing import/init in `app.js`. This is the red state.

- [ ] **Step 4: Create `static/liquid-glass.js`**

Create `static/liquid-glass.js` with exactly this content. Keep `LIQUID_DROP_MAP` on one line.

```js
// Provenance: the idea of feeding a static feImage displacement map into
// feDisplacementMap and referencing the resulting filter from
// `backdrop-filter: url(#id)` follows the technique demonstrated by
// https://github.com/shuding/liquid-glass ("Copy-paste Liquid Glass effect
// using SVG filters"). The filter markup, the LIQUID_DROP_MAP asset, and this
// capability helper are independently implemented for this repository. No
// upstream code or asset is copied, so no upstream license notice is required
// while this remains a conceptual reimplementation.

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
  "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAIAAAABACAYAAADS1n9/AAAGk0lEQVR42u3SsQrC4GKAUR9KJODg4uLikuE2UCoUO2Ro4CJFSmqT1EqREgrpkKFSSCmkHVxcsrg4BIL4Tu254CsId/iH8wTfN/v8b/l/f/JmYuTFk4EHd3o6brQ01Fy5cKaiDH6q4ETOkQMZKXt2JMRs2bBmxZKIBfOv2ed/DMCbiZEXTwYe3OnpuNHSUHPlwpmKMvipghM5Rw5kpOzZkRCzZcOaFUsiFsy/Zp/eALyZGHnxZODBnZ6OGy0NNVcunKkog58qOJFz5EBGyp4dCTFbNqxZsSRiwfxr9vlvA/BmYuTFk4EHd3o6brQ01Fy5cKaiDH6q4ETOkQMZKXt2JMRs2bBmxZKIBfOv2ee/DMCbiZEXTwYe3OnpuNHSUHPlwpmKMvipghM5Rw5kpOzZkRCzZcOaFUsiFsy/Zp//NABvJkZePBl4cKen40ZLQ82VC2cqyuCnCk7kHDmQkbJnR0LMlg1rViyJWDD/mn3+wwC8mRh58WTgwZ2ejhstDTVXLpypKIOfKjiRc+RARsqeHQkxWzasWbEkYsH8a/ZpDcCbiZEXTwYe3OnpuNHSUHPlwpmKMvipghM5Rw5kpOzZkRCzZcOaFUsiFsy/Zp9/NwBvJkZePBl4cKen40ZLQ82VC2cqyuCnCk7kHDmQkbJnR0LMlg1rViyJWDD/mn3+zQC8mRh58WTgwZ2ejhstDTVXLpypKIOfKjiRc+RARsqeHQkxWzasWbEkYsH8a/b5VwPwZmLkxZOBB3d6Om60NNRcuXCmogx+quBEzpEDGSl7diTEbNmwZsWSiAXzr9nnXwzAm4mRF08GHtzp6bjR0lBz5cKZijL4qYITOUcOZKTs2ZEQs2XDmhVLIhbMv2affzYAbyZGXjwZeHCnp+NGS0PNlQtnKsrgpwpO5Bw5kJGyZ0dCzJYNa1YsiVgw/5p9/skAvJkYefFk4MGdno4bLQ01Vy6cqSiDnyo4kXPkQEbKnh0JMVs2rFmxJGLB/Gv2qQzAm4mRF08GHtzp6bjR0lBz5cKZijL4qYITOUcOZKTs2ZEQs2XDmhVLIhbMv2af0gC8mRh58WTgwZ2ejhstDTVXLpypKIOfKjiRc+RARsqeHQkxWzasWbEkYsH8KwwQBggDhAHCAGGAMEAYIAwQBggDhAHCAGGAMEAYIAwQBggDhAHCAGGAMEAYIAwQBggDhAHCAGGAMEAYIAwQBggDhAHCAGGAMEAYIAwQBggDhAHCAGGAMEAYIAwQBggDhAFCoDBAiBQGCMIAQRggCAMEYYDgz36AwgC8mRh58WTgwZ2ejhstDTVXLpypKIOfKjiRc+RARsqeHQkxWzasWbEkYsH8a/b5RwPwZmLkxZOBB3d6Om60NNRcuXCmogx+quBEzpEDGSl7diTEbNmwZsWSiAXzr9nnHwzAm4mRF08GHtzp6bjR0lBz5cKZijL4qYITOUcOZKTs2ZEQs2XDmhVLIhbMv2afvzcAbyZGXjwZeHCnp+NGS0PNlQtnKsrgpwpO5Bw5kJGyZ0dCzJYNa1YsiVgw/5p9/s4AvJkYefFk4MGdno4bLQ01Vy6cqSiDnyo4kXPkQEbKnh0JMVs2rFmxJGLB/Gv2+aMBeDMx8uLJwIM7PR03Whpqrlw4U1EGP1VwIufIgYyUPTsSYrZsWLNiScSC+dfs87cG4M3EyIsnAw/u9HTcaGmouXLhTEUZ/FTBiZwjBzJS9uxIiNmyYc2KJREL5l+zT2oA3kyMvHgy8OBOT8eNloaaKxfOVJTBTxWcyDlyICNlz46EmC0b1qxYErFg/jX7/I0BeDMx8uLJwIM7PR03Whpqrlw4U1EGP1VwIufIgYyUPTsSYrZsWLNiScSC+dfs89cG4M3EyIsnAw/u9HTcaGmouXLhTEUZ/FTBiZwjBzJS9uxIiNmyYc2KJREL5l+zz18ZgDcTIy+eDDy409Nxo6Wh5sqFMxVl8FMFJ3KOHMhI2bMjIWbLhjUrlkQsmH/NPn9pAN5MjLx4MvDgTk/HjZaGmisXzlSUwU8VnMg5ciAjZc+OhJgtG9asWBKxYP41+yQG4M3EyIsnAw/u9HTcaGmouXLhTEUZ/FTBiZwjBzJS9uxIiNmyYc2KJREL5l+zz18YgDcTIy+eDDy409Nxo6Wh5sqFMxVl8FMFJ3KOHMhI2bMjIWbLhjUrlkQsmH/NPn8wAG8mRl48GXhwp6fjRktDzZULZyrK4KcKTuQcOZCRsmdHQsyWDWtWLIlYMP/6f/k95KKAm/+DAAAAAElFTkSuQmCC";

let initialized = false;
let currentMode = null;
let manualOverride = null;
let motionQuery = null;
let transparencyQuery = null;
let listenersAttached = false;

function prefersReduceMotion() {
  if (!motionQuery && typeof window !== "undefined" && typeof window.matchMedia === "function") {
    motionQuery = window.matchMedia("(prefers-reduced-motion: reduce)");
  }
  return Boolean(motionQuery && motionQuery.matches);
}

function prefersReducedTransparency() {
  if (!transparencyQuery && typeof window !== "undefined" && typeof window.matchMedia === "function") {
    transparencyQuery = window.matchMedia("(prefers-reduced-transparency: reduce)");
  }
  return Boolean(transparencyQuery && transparencyQuery.matches);
}

function saveDataEnabled() {
  const connection = navigator.connection;
  return Boolean(connection && connection.saveData === true);
}

function isChromiumEngine() {
  const userAgentData = navigator.userAgentData;
  if (userAgentData && Array.isArray(userAgentData.brands)) {
    if (userAgentData.brands.some((brand) => /Chromium/i.test(brand.brand || ""))) return true;
  }
  return /Chromium\/|Chrome\//.test(navigator.userAgent || "");
}

function supportsDisplacement() {
  if (typeof CSS === "undefined" || typeof CSS.supports !== "function") return false;
  try {
    return (
      CSS.supports("backdrop-filter", "url(#liquidDrop)") ||
      CSS.supports("-webkit-backdrop-filter", "url(#liquidDrop)")
    );
  } catch (_error) {
    return false;
  }
}

function overrideFromSearch() {
  if (typeof window === "undefined" || !window.location) return null;
  const match = /[?&]glass=(full|light|off)(?:&|$)/.exec(window.location.search || "");
  return match ? match[1] : null;
}

export function detectGlassMode() {
  const override = manualOverride || overrideFromSearch();
  if (override) return override;
  if (prefersReducedTransparency()) return "off";
  if (saveDataEnabled()) return "light";
  if (isChromiumEngine() && supportsDisplacement()) return "full";
  return "light";
}

export function glassMode() {
  return currentMode;
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
  return resolved;
}

function applyMotionClass() {
  const reduced = prefersReduceMotion();
  document.documentElement.classList.toggle("glass-motion-reduced", reduced);
}

function attachListenersOnce() {
  if (listenersAttached) return;
  listenersAttached = true;
  if (motionQuery && typeof motionQuery.addEventListener === "function") {
    motionQuery.addEventListener("change", applyMotionClass);
  }
  if (transparencyQuery && typeof transparencyQuery.addEventListener === "function") {
    transparencyQuery.addEventListener("change", () => applyGlassMode(detectGlassMode()));
  }
  const connection = navigator.connection;
  if (connection && typeof connection.addEventListener === "function") {
    connection.addEventListener("change", () => applyGlassMode(detectGlassMode()));
  }
  window.__webControlSetGlassMode = (mode) => {
    manualOverride = GLASS_MODES.includes(mode) ? mode : null;
    return applyGlassMode(detectGlassMode());
  };
}

export function initLiquidGlass(options = {}) {
  if (initialized) return currentMode;
  initialized = true;
  if (options && GLASS_MODES.includes(options.mode)) manualOverride = options.mode;
  prefersReduceMotion();
  prefersReducedTransparency();
  applyMotionClass();
  attachListenersOnce();
  return applyGlassMode(detectGlassMode());
}
```

- [ ] **Step 5: Wire the helper into `static/app.js`**

Add the import directly under the Three.js import (currently line 1):

```js
import * as THREE from "./vendor/three.module.js";
import { initLiquidGlass, glassMotionReduced } from "./liquid-glass.js";
```

`glassMotionReduced` is consumed by the Task 3 state machine; the import is added now so the wiring contract is complete. Then, immediately after the `state` object and before `const mapCanvas3d = ...` (currently lines 48-49), call:

```js
initLiquidGlass();
```

- [ ] **Step 6: Run the Task 1 tests to verify they pass**

Run:

```bash
python3 -m pytest -q \
  test/test_core.py::test_frontend_hosts_liquid_glass_defs \
  test/test_core.py::test_liquid_glass_helper_exports_provenance_and_static_assets \
  test/test_core.py::test_liquid_glass_wiring_in_app \
  test/test_core.py::test_liquid_glass_capability_detection_under_node
```

Expected: `4 passed`.

- [ ] **Step 7: Syntax-check both JavaScript files**

Run:

```bash
node --check static/liquid-glass.js && node --check static/app.js
```

Expected: no output, exit code 0.

- [ ] **Step 8: Commit**

```bash
git add static/index.html static/liquid-glass.js static/app.js test/test_core.py
git commit -m "feat: add liquid glass capability helper and defs host"
git status --porcelain
```

Expected status after the commit: only ` M config/web_control.yaml` remains. If `config/web_control.yaml` appears staged, stop and unstage it (`git restore --staged config/web_control.yaml`) — never commit it.

---

## Task 2: Three-tier glass surfaces, water-drop motion, and accessibility

**Files:**
- Modify: `static/index.html` (sidebar toggle lens, currently line 70-72)
- Modify: `static/styles.css`
- Modify: `static/app.js` (`markGlassDrop`, `setMode`)
- Test: `test/test_core.py`

- [ ] **Step 1: Add the decorative sidebar toggle lens**

In `static/index.html`, inside the existing `#sidebarToggle` button (currently lines 70-72), add the lens before the arrow:

```html
        <button id="sidebarToggle" class="sidebar-toggle" type="button" aria-expanded="true" aria-controls="controlPanel" aria-label="Collapse sidebar">
          <span class="sidebar-toggle-lens" aria-hidden="true"></span>
          <span class="sidebar-toggle-arrow" aria-hidden="true">›</span>
        </button>
```

- [ ] **Step 2: Write the failing CSS contract tests (tiers, a11y, fallbacks)**

Add a CSS rule helper next to `_frontend_sources` (currently line 3341) and the new tests. Place the helper directly after `_frontend_sources`:

```python
def _css_rule_block(styles_css, selector):
    """Return one CSS rule body for an exact ``selector {`` match."""
    start = styles_css.index(f"{selector} {{")
    end = styles_css.index("}", start)
    return styles_css[start:end]
```

Add these tests near the existing visual-theme tests (a good anchor is after `test_frontend_uses_ios_inspired_visual_theme`, currently line 1617):

```python
def test_glass_tokens_and_mode_classes():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    root = _css_rule_block(styles_css, ":root")
    for token in (
        "--scene-bg: #0b0f19;",
        "--glass-drop-ms: 380ms;",
        "--glass-drop-ease: cubic-bezier(0.18, 1.32, 0.32, 1);",
        "--layout-transition-ms: 260ms;",
        "--focus-ring: 2px;",
        "--touch-target: 44px;",
    ):
        assert token in root
    assert "--ios-spring: cubic-bezier(0.2, 0.85, 0.25, 1.08)" in root
    assert "--ios-contained-spring: cubic-bezier(0.2, 0.85, 0.25, 1)" in root
    defs = _css_rule_block(styles_css, ".glass-defs")
    assert "position: absolute;" in defs
    assert "width: 0;" in defs
    assert "height: 0;" in defs
    assert "overflow: hidden;" in defs
    assert "pointer-events: none;" in defs
    assert "display: none;" not in defs


def test_glass_baseline_reset_removes_stacked_blur():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    for selector in (
        ".tabs",
        ".sliding-tabs::before",
        ".segmented::before",
        ".system-card",
        ".cpu-core-panel",
        ".hud",
        ".joystick-base",
        ".panel",
        ".sidebar-toggle",
        ".panel section",
    ):
        assert "backdrop-filter" not in _css_rule_block(styles_css, selector), selector
    assert "filter: drop-shadow" not in styles_css
    assert "will-change: transform, box-shadow, filter;" not in styles_css
    assert "transition: transform 430ms var(--ios-spring)" not in styles_css
    assert "transition: transform 430ms var(--ios-contained-spring)" not in styles_css


def test_glass_full_tier_limits_displacement_to_three_decorative_surfaces():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    assert ".glass-full .sliding-tabs::before," in styles_css
    assert ".glass-full .segmented::before," in styles_css
    assert ".glass-full .sidebar-toggle-lens {" in styles_css
    selector_start = styles_css.index(".glass-full .sliding-tabs::before,")
    selector_end = styles_css.index("{", selector_start)
    selectors = styles_css[selector_start:selector_end]
    assert selectors.count(".glass-full") == 3
    body = styles_css[selector_end : styles_css.index("}", selector_end)]
    assert "backdrop-filter: url(#liquidDrop) saturate(1.4);" in body
    assert "-webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);" in body
    lens = _css_rule_block(styles_css, ".sidebar-toggle-lens")
    assert "position: absolute;" in lens
    assert "inset: 0;" in lens
    assert "border-radius: inherit;" in lens
    assert "background: var(--glass);" in lens
    assert "pointer-events: none;" in lens


def test_glass_light_tier_is_single_layer():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    anchor = ":where(.glass-full, .glass-light) .panel,"
    assert anchor in styles_css
    start = styles_css.index(anchor)
    end = styles_css.index("{", start)
    selectors = styles_css[start:end]
    assert selectors.count(":where(.glass-full, .glass-light)") == 6
    assert ".panel section" not in selectors
    body = styles_css[end : styles_css.index("}", end)]
    assert "backdrop-filter: blur(22px) saturate(1.35);" in body
    assert "-webkit-backdrop-filter: blur(22px) saturate(1.35);" in body


def test_glass_off_tokens_are_opaque():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    block = _css_rule_block(styles_css, ".glass-off")
    assert "--glass: rgba(255, 255, 255, 0.96);" in block
    assert "--glass-strong: rgba(255, 255, 255, 0.98);" in block
    assert "--panel-bg: rgba(255, 255, 255, 0.98);" in block
    assert "--terminal-bg: rgba(255, 255, 255, 0.98);" in block
    assert "@media (prefers-color-scheme: dark)" in styles_css
    assert "--panel-bg: rgba(15, 17, 24, 0.98);" in styles_css
    assert "--terminal-bg: rgba(5, 8, 12, 0.98);" in styles_css


def test_glass_tier_selectors_exclude_webgl_text_and_safety_controls():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    full_start = styles_css.index(".glass-full .sliding-tabs::before,")
    full_selectors = styles_css[full_start : styles_css.index("{", full_start)]
    light_start = styles_css.index(":where(.glass-full, .glass-light) .panel,")
    light_selectors = styles_css[light_start : styles_css.index("{", light_start)]
    for selector in (
        ".joystick-knob",
        ".terminal",
        ".terminal pre",
        ".source",
        "#navigationPrimary",
        "#navigationClear",
        "#publishPose",
        "#localizationProcessToggle",
        "#navigationProcessToggle",
        ".start-log",
        ".stop-log",
        ".joystick-settings-button",
        ".joystick-done-button",
        "#confirmSaveMap",
        "#cancelSaveMap",
    ):
        assert selector not in full_selectors, selector
        assert selector not in light_selectors, selector


def test_glass_focus_and_touch_targets():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    focus = _css_rule_block(styles_css, ":focus-visible")
    assert "outline: var(--focus-ring) solid var(--ios-blue);" in focus
    assert "outline-offset: 2px;" in focus
    coarse_start = styles_css.index("@media (pointer: coarse)")
    coarse = styles_css[coarse_start : styles_css.index("\n}", coarse_start)]
    assert "min-height: var(--touch-target);" in coarse
    assert "width: var(--touch-target);" in coarse
    for selector in (
        ".tab,",
        ".mode,",
        ".actions button,",
        ".start-log,",
        ".stop-log,",
        ".sidebar-toggle,",
        ".joystick-settings-button,",
        ".joystick-done-button,",
        ".map-picker button,",
        ".save-map-dialog-actions button",
    ):
        assert selector in coarse, selector


def test_glass_reduced_motion_and_transparency_fallbacks():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    motion_start = styles_css.index("@media (prefers-reduced-motion: reduce)")
    motion = styles_css[motion_start : styles_css.index("\n}", motion_start)]
    assert "--layout-transition-ms: 1ms;" in motion
    assert "--glass-drop-ms: 1ms;" in motion
    assert "animation: none;" in motion
    assert "body::before," in motion
    assert "body::after" in motion
    assert ".system-meter-fill" in motion
    assert "prefers-reduced-transparency" not in styles_css
    assert ".glass-off {" in styles_css


def test_glass_water_drop_motion():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    assert "transition: transform var(--glass-drop-ms) var(--glass-drop-ease)," in styles_css
    assert "will-change: transform;" in styles_css
    assert "will-change: transform, opacity;" in styles_css
    assert "@keyframes glassDropScale" in styles_css
    keyframes = styles_css[styles_css.index("@keyframes glassDropScale") :]
    assert "scale: 0.94;" in keyframes
    assert "scale: 1.035;" in keyframes
    animation_start = styles_css.index(".sliding-tabs.glass-drop::before,")
    assert ".segmented.glass-drop::before" in styles_css[
        animation_start : styles_css.index("{", animation_start)
    ]
    animation_body = styles_css[
        styles_css.index("{", animation_start) + 1 : styles_css.index("}", animation_start)
    ]
    assert "animation: glassDropScale var(--glass-drop-ms) var(--glass-drop-ease);" in animation_body
    assert "filter" not in animation_body
    drop_start = styles_css.index(".sliding-tabs::after,")
    drop_body = styles_css[
        styles_css.index("{", drop_start) + 1 : styles_css.index("}", drop_start)
    ]
    assert "transform: translate3d(calc(var(--tabs-index, var(--mode-index, 0)) * 100%), 0, 0) scale(0.4);" in drop_body
    assert "opacity: 0;" in drop_body
    assert "filter" not in drop_body
    assert ".sliding-tabs.glass-drop::after," in styles_css
    assert "opacity: 0.9;" in styles_css


def test_glass_drop_class_is_timed_and_forced_reflowed():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    assert "function markGlassDrop(element)" in app_js
    assert "void element.offsetWidth;" in app_js
    assert 'element.addEventListener("animationend", clear, { once: true });' in app_js
    assert 'markGlassDrop(document.getElementById("modeSegment"));' in app_js
    assert "const GLASS_DROP_FALLBACK_MS = 520;" in app_js
```

- [ ] **Step 3: Update the three existing motion/material contract tests**

Replace the body of `test_sliding_selectors_use_ios_26_liquid_glass_material` (currently lines 1698-1716) with:

```python
def test_sliding_selectors_use_ios_26_liquid_glass_material():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--liquid-slider-fill" in styles_css
    assert "--liquid-slider-rim" in styles_css
    assert "--liquid-slider-caustic" in styles_css
    assert "--liquid-slider-noise" in styles_css
    assert "data:image/svg+xml" in styles_css
    assert "feTurbulence" in styles_css
    assert "backdrop-filter: url(#liquidDrop) saturate(1.4);" in styles_css
    assert "-webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);" in styles_css
    assert "var(--liquid-slider-noise)" in styles_css
    assert "background-blend-mode: soft-light" in styles_css
    assert "background-size: 80px 80px" in styles_css
    assert "radial-gradient(circle at 18% 0%, var(--liquid-slider-caustic), transparent 42%)" in styles_css
    assert "inset 0 1px 0 rgba(255, 255, 255, 0.72)" in styles_css
    assert "filter: drop-shadow(0 10px 24px rgba(10, 132, 255, 0.18));" not in styles_css
```

Replace the body of `test_sliding_selectors_use_transform_based_ios_spring_motion` (currently lines 1681-1695) with:

```python
def test_sliding_selectors_use_transform_based_ios_spring_motion():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--ios-spring: cubic-bezier(0.2, 0.85, 0.25, 1.08)" in styles_css
    assert "--glass-drop-ease: cubic-bezier(0.18, 1.32, 0.32, 1);" in styles_css
    assert "will-change: transform;" in styles_css
    assert "will-change: transform, box-shadow, filter;" not in styles_css
    assert "transform: translate3d(calc(var(--tabs-index, 0) * 100%), 0, 0)" in styles_css
    assert "transform: translate3d(calc(var(--mode-index, 0) * 100%), 0, 0)" in styles_css
    assert "transition: transform var(--glass-drop-ms) var(--glass-drop-ease)" in styles_css
    assert '.sliding-tabs[data-active="system"] {' in styles_css
    assert '.segmented[data-active="goal"] {' in styles_css
    assert "transition: left 280ms" not in styles_css
    assert "transform: translateX(100%)" not in styles_css
    assert "transform: translateX(200%)" not in styles_css
```

Replace the body of `test_mode_slider_uses_contained_spring_to_stay_inside_track` (currently lines 1774-1784) with:

```python
def test_mode_slider_uses_contained_spring_to_stay_inside_track():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--ios-contained-spring: cubic-bezier(0.2, 0.85, 0.25, 1);" in styles_css
    segmented_before = styles_css[
        styles_css.index(".segmented::before") : styles_css.index(".segmented[data-active=\"initial\"]")
    ]
    assert "transition: transform var(--glass-drop-ms) var(--glass-drop-ease)" in segmented_before
    assert "var(--ios-spring)" not in segmented_before
    assert "var(--ios-contained-spring)" not in segmented_before
```

- [ ] **Step 4: Run the Task 2 tests to verify they fail**

Run:

```bash
python3 -m pytest -q \
  test/test_core.py::test_glass_tokens_and_mode_classes \
  test/test_core.py::test_glass_baseline_reset_removes_stacked_blur \
  test/test_core.py::test_glass_full_tier_limits_displacement_to_three_decorative_surfaces \
  test/test_core.py::test_glass_light_tier_is_single_layer \
  test/test_core.py::test_glass_off_tokens_are_opaque \
  test/test_core.py::test_glass_tier_selectors_exclude_webgl_text_and_safety_controls \
  test/test_core.py::test_glass_focus_and_touch_targets \
  test/test_core.py::test_glass_reduced_motion_and_transparency_fallbacks \
  test/test_core.py::test_glass_water_drop_motion \
  test/test_core.py::test_glass_drop_class_is_timed_and_forced_reflowed \
  test/test_core.py::test_sliding_selectors_use_ios_26_liquid_glass_material \
  test/test_core.py::test_sliding_selectors_use_transform_based_ios_spring_motion \
  test/test_core.py::test_mode_slider_uses_contained_spring_to_stay_inside_track
```

Expected: 13 FAIL (missing tokens, tier rules, motion, fallbacks, `markGlassDrop`); `test_sliding_selectors_use_ios_26_liquid_glass_material` additionally fails on the missing `url(#liquidDrop)` rule.

- [ ] **Step 5: Add the tokens to `:root`**

In `static/styles.css`, after `--radius-lg: 22px;` (currently line 38) and before the closing `}` of `:root`, add:

```css
  --scene-bg: #0b0f19;
  --glass-drop-ms: 380ms;
  --glass-drop-ease: cubic-bezier(0.18, 1.32, 0.32, 1);
  --layout-transition-ms: 260ms;
  --focus-ring: 2px;
  --touch-target: 44px;
```

- [ ] **Step 6: Reset the baseline blur and animated filter declarations**

Make these exact replacements in `static/styles.css`. The base rules keep their translucent fills, borders, and shadows; only the blur/filter declarations are removed, so a page with no glass class still renders readable surfaces and no rule can stack a second blur.

Replace the `.app` transition (currently line 128):

```css
  transition: grid-template-columns var(--layout-transition-ms) ease;
```

Remove both `backdrop-filter` lines from `.tabs` (currently lines 162-163). Remove both from `.sliding-tabs::before` (currently lines 210-211) and replace its `filter`/`will-change`/`transition` lines (currently lines 212-215) with:

```css
  will-change: transform;
  transition: transform var(--glass-drop-ms) var(--glass-drop-ease),
              box-shadow var(--layout-transition-ms) ease;
```

Remove both `backdrop-filter` lines from `.system-card` (currently lines 314-315). Remove both from `.cpu-core-panel` (currently lines 436-437). Remove both from `.hud` (currently lines 547-548). Remove both from `.joystick-base` (currently lines 598-599).

Update `.panel` (currently lines 720-723) to remove both blur lines and use the token:

```css
  transition: opacity var(--layout-transition-ms) ease, transform var(--layout-transition-ms) ease;
```

Remove both `backdrop-filter` lines from `.sidebar-toggle` (currently lines 750-751), and add at the end of that rule:

```css
  transition: opacity var(--layout-transition-ms) ease,
              visibility 0s linear 0s;
```

Remove both `backdrop-filter` lines from `.panel section` (currently lines 824-825).

Remove both `backdrop-filter` lines from `.segmented::before` (currently lines 940-941) and replace its `filter`/`will-change`/`transition` lines (currently lines 942-945) with:

```css
  will-change: transform;
  transition: transform var(--glass-drop-ms) var(--glass-drop-ease),
              box-shadow var(--layout-transition-ms) ease;
```

Add `position: relative; z-index: 1;` to the existing `.sidebar-toggle-arrow` rule (currently lines 754-759) so the arrow sits above the lens:

```css
.sidebar-toggle-arrow {
  position: relative;
  z-index: 1;
  display: inline-block;
  font-size: 18px;
  line-height: 1;
  transition: transform 260ms ease;
}
```

- [ ] **Step 7: Append the glass tier, focus, touch, and fallback rules**

Insert the following block in `static/styles.css` immediately before `@media (max-width: 900px)` (currently line 1339). Every selector here is new; the tiers are the single source of truth for blur and displacement.

```css
/* --- Liquid glass tiers */
.glass-defs {
  position: absolute;
  width: 0;
  height: 0;
  overflow: hidden;
  pointer-events: none;
}

.glass-full .sliding-tabs::before,
.glass-full .segmented::before,
.glass-full .sidebar-toggle-lens {
  backdrop-filter: url(#liquidDrop) saturate(1.4);
  -webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);
}

.sidebar-toggle-lens {
  position: absolute;
  inset: 0;
  border-radius: inherit;
  overflow: hidden;
  background: var(--glass);
  pointer-events: none;
}

:where(.glass-full, .glass-light) .panel,
:where(.glass-full, .glass-light) .hud,
:where(.glass-full, .glass-light) #saveMapDialog,
:where(.glass-full, .glass-light) .system-card,
:where(.glass-full, .glass-light) .cpu-core-panel,
:where(.glass-full, .glass-light) .joystick-base {
  backdrop-filter: blur(22px) saturate(1.35);
  -webkit-backdrop-filter: blur(22px) saturate(1.35);
}

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
  .save-map-dialog-actions button {
    min-height: var(--touch-target);
  }

  .sidebar-toggle {
    width: var(--touch-target);
  }
}

@media (prefers-reduced-motion: reduce) {
  :root {
    --layout-transition-ms: 1ms;
    --glass-drop-ms: 1ms;
  }

  .sliding-tabs::before,
  .segmented::before,
  .sliding-tabs::after,
  .segmented::after,
  .viewport,
  .panel-view,
  .app,
  .panel {
    transition-duration: 1ms;
  }

  .sliding-tabs.glass-drop::before,
  .segmented.glass-drop::before {
    animation: none;
  }

  .sliding-tabs.glass-drop::after,
  .segmented.glass-drop::after {
    opacity: 0;
  }

  body::before,
  body::after {
    animation: none;
  }

  .system-meter-fill {
    animation: none;
  }
}
```

- [ ] **Step 8: Append the water-drop motion rules**

Insert the following immediately after the tier block from Step 7 (still before `@media (max-width: 900px)`). The lens is `.sliding-tabs::before` / `.segmented::before`; the edge droplet is a new `::after` that shares the active index. Only `transform`, `scale`, and `opacity` animate.

```css
/* --- Liquid glass water-drop motion */
.sliding-tabs.glass-drop::before,
.segmented.glass-drop::before {
  animation: glassDropScale var(--glass-drop-ms) var(--glass-drop-ease);
}

@keyframes glassDropScale {
  0% {
    scale: 0.94;
  }
  55% {
    scale: 1.035;
  }
  100% {
    scale: 1;
  }
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

- [ ] **Step 9: Add `markGlassDrop` and mark the mode segment in `static/app.js`**

Add this constant and function immediately before `function setMode(mode)` (currently line 1756):

```js
const GLASS_DROP_FALLBACK_MS = 520;

function markGlassDrop(element) {
  if (!element) return;
  element.classList.remove("glass-drop");
  // One forced reflow restarts the keyframes when the same element is re-marked.
  void element.offsetWidth;
  element.classList.add("glass-drop");
  const clear = () => element.classList.remove("glass-drop");
  element.addEventListener("animationend", clear, { once: true });
  window.setTimeout(clear, GLASS_DROP_FALLBACK_MS);
}
```

Then, inside `setMode`, after the `.mode` active class loop (currently line 1773) and before `updatePublishPoseButton();`, add:

```js
  if (previousMode !== mode) markGlassDrop(document.getElementById("modeSegment"));
```

- [ ] **Step 10: Run the Task 2 tests to verify they pass**

Run:

```bash
python3 -m pytest -q \
  test/test_core.py::test_glass_tokens_and_mode_classes \
  test/test_core.py::test_glass_baseline_reset_removes_stacked_blur \
  test/test_core.py::test_glass_full_tier_limits_displacement_to_three_decorative_surfaces \
  test/test_core.py::test_glass_light_tier_is_single_layer \
  test/test_core.py::test_glass_off_tokens_are_opaque \
  test/test_core.py::test_glass_tier_selectors_exclude_webgl_text_and_safety_controls \
  test/test_core.py::test_glass_focus_and_touch_targets \
  test/test_core.py::test_glass_reduced_motion_and_transparency_fallbacks \
  test/test_core.py::test_glass_water_drop_motion \
  test/test_core.py::test_glass_drop_class_is_timed_and_forced_reflowed \
  test/test_core.py::test_sliding_selectors_use_ios_26_liquid_glass_material \
  test/test_core.py::test_sliding_selectors_use_transform_based_ios_spring_motion \
  test/test_core.py::test_mode_slider_uses_contained_spring_to_stay_inside_track
```

Expected: `13 passed`.

- [ ] **Step 11: Run the neighboring visual-theme tests to catch regressions**

Run:

```bash
python3 -m pytest -q \
  test/test_core.py::test_frontend_uses_ios_inspired_visual_theme \
  test/test_core.py::test_mode_control_uses_ios_sliding_segmented_selector \
  test/test_core.py::test_frontend_uses_immersive_glass_light_environment \
  test/test_core.py::test_frontend_applies_apple_liquid_glass_guidance \
  test/test_core.py::test_top_tabs_use_sliding_segmented_selector \
  test/test_core.py::test_top_tab_slider_uses_equal_width_segments \
  test/test_core.py::test_top_tab_hover_does_not_move_or_draw_extra_blue_button \
  test/test_core.py::test_mode_segment_hover_does_not_draw_offset_blue_button \
  test/test_core.py::test_mode_track_is_visible_in_light_theme \
  test/test_core.py::test_mode_slider_is_clipped_and_mode_text_is_centered \
  test/test_core.py::test_frontend_follows_system_light_and_dark_color_scheme
```

Expected: `11 passed`.

- [ ] **Step 12: Syntax-check both JavaScript files**

Run:

```bash
node --check static/app.js && node --check static/liquid-glass.js
```

Expected: no output, exit code 0.

- [ ] **Step 13: Commit**

```bash
git add static/index.html static/styles.css static/app.js test/test_core.py
git commit -m "feat: add liquid glass tiers and water-drop motion"
git status --porcelain
```

Expected status after the commit: only ` M config/web_control.yaml` remains.

---

## Task 3: Layout transition state machine, mounted viewports, and sidebar/System coordination

**Files:**
- Modify: `static/index.html` (panel-views wrapper, currently lines 84 and 194-243)
- Modify: `static/styles.css`
- Modify: `static/app.js`
- Test: `test/test_core.py`

- [ ] **Step 1: Wrap the sidebar panels**

In `static/index.html`, insert `<div class="panel-views">` immediately after the closing `</header>` (currently line 82), before `<div id="navigationSidebar"...>` (currently line 84). Then insert the matching `</div>` immediately after the closing `</div>` of `#mappingSidebar` (currently line 243) and before `</aside>`. The result:

```html
        </header>

        <div class="panel-views">
        <div id="navigationSidebar" class="panel-view active" data-panel="navigation">
        ...
        </div>

        <div id="mappingSidebar" class="panel-view" data-panel="mapping">
        ...
        </div>
        </div>
      </aside>
```

The two existing `.panel-view` elements, their ids, classes, and order are unchanged.

- [ ] **Step 2: Write the failing Task 3 static tests**

Add these tests near `test_system_tab_hides_sidebar_and_expands_stage` (currently line 1533). They use the `_css_rule_block` helper added in Task 2.

```python
def test_frontend_mounts_viewports_and_uses_inert():
    index_html, app_js, styles_css = _frontend_sources()

    assert 'id="view3d"' in index_html
    assert 'id="viewSystem"' in index_html
    assert index_html.index('id="view3d"') < index_html.index('id="viewSystem"')

    viewport = _css_rule_block(styles_css, ".viewport")
    assert "display: none;" not in viewport
    assert "position: absolute;" in viewport
    assert "opacity: 0;" in viewport
    assert "visibility: hidden;" in viewport
    assert "pointer-events: none;" in viewport

    active = _css_rule_block(styles_css, ".viewport.active")
    assert "display: block;" not in active
    assert "opacity: 1;" in active
    assert "visibility: visible;" in active
    assert "pointer-events: auto;" in active

    assert "view3d.inert = !isMapView;" in app_js
    assert 'view3d.setAttribute("aria-hidden", String(!isMapView));' in app_js
    assert 'viewSystem.inert = tab !== "system";' in app_js
    assert 'viewSystem.setAttribute("aria-hidden", String(tab !== "system"));' in app_js
    assert "panel.inert = !isActive;" in app_js
    assert 'panel.setAttribute("aria-hidden", String(!isActive));' in app_js


def test_glass_no_filter_on_webgl_canvas():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    canvas = _css_rule_block(styles_css, "#mapCanvas3d")
    assert "background-color: var(--scene-bg);" in canvas
    assert "backdrop-filter" not in canvas
    assert "filter:" not in canvas
    view3d = _css_rule_block(styles_css, "#view3d")
    assert "backdrop-filter" not in view3d
    assert "filter:" not in view3d
    assert "--scene-bg: #0b0f19;" in styles_css


def test_frontend_layout_transition_state_machine_static():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )

    assert "const LAYOUT_SETTLE_FALLBACK_MS = 420;" in app_js
    for name in (
        "beginLayoutTransition",
        "settleLayoutTransition",
        "noteLayoutResize",
        "doDeferredLayoutFlush",
        "attachLayoutTransitionListener",
    ):
        assert f"function {name}(" in app_js
    assert "pageTransition" in app_js
    assert 'window.addEventListener("resize", noteLayoutResize);' in app_js
    assert 'window.addEventListener("resize", resizeAll);' not in app_js
    assert 'window.addEventListener("resize", applyJoystickLayout);' not in app_js
    for property_name in ("grid-template-columns", "opacity", "transform"):
        assert property_name in app_js
    assert app_js.count("renderer.setSize(") == 1
    resize_body = _js_function_body(app_js, "function resizeAll()")
    assert "renderer.setSize(" in resize_body
    assert "resizeAll();" not in app_js[
        app_js.index("function setTab(tab)") : app_js.index("function selectedPosePayload")
    ]


def test_frontend_white_flash_fix_present():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )

    assert "const SCENE_BG = 0x0b0f19;" in app_js
    assert "scene.background = new THREE.Color(SCENE_BG);" in app_js
    assert "renderer.setClearColor(0x0b0f19, 1);" in app_js

    render_body = _js_function_body(app_js, "function render()")
    assert "mapViewActive()" in render_body
    assert "requestAnimationFrame(render)" in render_body
    assert "cancelAnimationFrame(mapRenderFrame);" in render_body
    assert "function mapViewActive()" in app_js
    assert "function ensureMapRenderLoop()" in app_js
    assert "function pauseMapRenderLoop()" in app_js

    flush_body = _js_function_body(app_js, "function doDeferredLayoutFlush()")
    assert "resizeAll();" in flush_body
    assert "applyJoystickLayout();" in flush_body
    assert "updateHudPlacement();" in flush_body
    assert "renderer.render(scene, camera);" in flush_body

    assert "--scene-bg: #0b0f19;" in styles_css
    assert "background-color: var(--scene-bg);" in styles_css


def test_frontend_safety_invariants_in_set_tab():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    set_tab_block = app_js[
        app_js.index("function setTab(tab)") : app_js.index("function selectedPosePayload")
    ]

    assert 'setMappingJoystickVisible(tab === "mapping");' in set_tab_block
    assert "exitJoystickLayout();" in set_tab_block
    assert set_tab_block.index("setMappingJoystickVisible") < set_tab_block.index(
        "beginLayoutTransition();"
    )
    assert "resizeAll();" not in set_tab_block
    assert "markGlassDrop(viewTabs);" in set_tab_block
    assert "if (applyEffectiveSidebar()) beginLayoutTransition();" in app_js
    assert "function applyEffectiveSidebar()" in app_js

    sidebar_block = app_js[
        app_js.index("// --- Sidebar collapse") : app_js.index("// --- End sidebar collapse")
    ]
    assert "localStorage" not in sidebar_block
    assert "sessionStorage" not in sidebar_block
    assert "function persistJoystickPreferences()" in app_js
    assert 'postJson("/api/ui/preferences"' in app_js
    assert "function selectMap(" in app_js


def test_frontend_sidebar_and_system_use_shared_collapse():
    index_html, app_js, styles_css = _frontend_sources()

    assert '<div class="panel-views">' in index_html
    assert index_html.index('class="panel-views"') < index_html.index('id="navigationSidebar"')
    panel_views = _css_rule_block(styles_css, ".panel-views")
    assert "position: relative;" in panel_views

    panel_view = _css_rule_block(styles_css, ".panel-view")
    assert "display: none;" not in panel_view
    assert "opacity: 0;" in panel_view
    active_panel_view = _css_rule_block(styles_css, ".panel-view.active")
    assert "display: block;" not in active_panel_view
    assert "position: relative;" in active_panel_view
    inactive_panel_view = _css_rule_block(styles_css, ".panel-view:not(.active)")
    assert "position: absolute;" in inactive_panel_view
    assert "inset: 0;" in inactive_panel_view

    combined_start = styles_css.index(".app.sidebar-collapsed .panel,")
    combined_body = styles_css[
        styles_css.index("{", combined_start) + 1 : styles_css.index("}", combined_start)
    ]
    assert "opacity: 0;" in combined_body
    assert "transform: translateX(12px);" in combined_body
    assert "pointer-events: none;" in combined_body

    toggle_start = styles_css.index(".app.system-mode .sidebar-toggle {")
    toggle_body = styles_css[toggle_start : styles_css.index("}", toggle_start)]
    assert "display: none;" not in toggle_body
    assert "opacity: 0;" in toggle_body
    assert "visibility: hidden;" in toggle_body

    assert 'const effective = sidebarState.collapsed || state.view === "system";' in app_js
    assert "function syncSidebarToggle(toggle)" in app_js
    assert "let appliedEffectiveCollapse = false;" in app_js
```

- [ ] **Step 3: Write the failing layout-transition Node harness**

Add this harness constant and test next to the other Node harness tests (a good anchor is immediately after `test_frontend_map_selector_behavior_executes_under_node`, currently line 3513).

```python
_LAYOUT_TRANSITION_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Page transition state machine");
const blockEnd = source.indexOf("// --- End page transition state machine");
if (blockStart < 0 || blockEnd < 0) throw new Error("page transition block not found");
const block = source.slice(blockStart, blockEnd);

let reduceMotion = false;
global.glassMotionReduced = () => reduceMotion;
let resizeCalls = 0;
let layoutCalls = 0;
let hudCalls = 0;
let renderCalls = 0;
global.resizeAll = () => { resizeCalls += 1; };
global.applyJoystickLayout = () => { layoutCalls += 1; };
global.updateHudPlacement = () => { hudCalls += 1; };
global.renderer = { render: () => { renderCalls += 1; } };
global.scene = {};
global.camera = {};

const timers = new Map();
const frames = new Map();
let nextTimer = 1000;
let nextFrame = 1;
global.setTimeout = (fn) => { const id = nextTimer++; timers.set(id, fn); return id; };
global.clearTimeout = (id) => { timers.delete(id); };
global.requestAnimationFrame = (fn) => { const id = nextFrame++; frames.set(id, fn); return id; };
global.cancelAnimationFrame = (id) => { frames.delete(id); };

function flushTimers() {
  const pending = Array.from(timers.values());
  timers.clear();
  pending.forEach((fn) => fn());
}

function flushFrames() {
  const pending = Array.from(frames.values());
  frames.clear();
  pending.forEach((fn) => fn());
}

function makeElement() {
  const listeners = {};
  return {
    addEventListener(type, handler) { (listeners[type] = listeners[type] || []).push(handler); },
    dispatch(type, event) { (listeners[type] || []).forEach((handler) => handler(event)); },
  };
}

const appEl = makeElement();
const view3dEl = makeElement();
const viewSystemEl = makeElement();
const viewTabsEl = makeElement();
global.document = {
  querySelector(selector) { return selector === ".app" ? appEl : null; },
  getElementById(id) {
    if (id === "view3d") return view3dEl;
    if (id === "viewSystem") return viewSystemEl;
    if (id === "viewTabs") return viewTabsEl;
    return null;
  },
};

const transitionApi = new Function(block + `
return { pageTransition, beginLayoutTransition, settleLayoutTransition, noteLayoutResize, doDeferredLayoutFlush, attachLayoutTransitionListener };`)();
const {
  pageTransition, beginLayoutTransition, settleLayoutTransition,
  noteLayoutResize, doDeferredLayoutFlush, attachLayoutTransitionListener,
} = transitionApi;

function expect(condition, message) { if (!condition) throw new Error(message); }
function resetCounters() { resizeCalls = 0; layoutCalls = 0; hudCalls = 0; renderCalls = 0; }

attachLayoutTransitionListener();

// Rapid requests: the newest token wins and the previous fallback is cancelled.
resetCounters();
const firstToken = beginLayoutTransition();
const secondToken = beginLayoutTransition();
expect(secondToken === firstToken + 1, "each request bumps the token");
flushTimers();
expect(resizeCalls === 1, "one flush for rapid requests");
expect(layoutCalls === 1, "one joystick layout for rapid requests");
expect(renderCalls === 1, "one synchronous render for rapid requests");
expect(pageTransition.active === false, "controller returns to stable");
settleLayoutTransition(firstToken);
expect(resizeCalls === 1, "a stale token never flushes");

// A tracked transitionend settles; untracked properties and targets are ignored.
resetCounters();
beginLayoutTransition();
appEl.dispatch("transitionend", { propertyName: "color", target: viewSystemEl });
appEl.dispatch("transitionend", { propertyName: "opacity", target: makeElement() });
expect(resizeCalls === 0, "untracked events are ignored");
appEl.dispatch("transitionend", { propertyName: "opacity", target: viewSystemEl });
expect(resizeCalls === 1, "a tracked transitionend settles");
expect(renderCalls === 1, "a tracked transitionend performs the settle render");

// A stale event for an old token does not flush.
resetCounters();
beginLayoutTransition();
const liveToken = beginLayoutTransition();
appEl.dispatch("transitionend", { propertyName: "grid-template-columns", target: appEl });
expect(resizeCalls === 1, "the newest token settles");
settleLayoutTransition(liveToken - 1);
expect(resizeCalls === 1, "old tokens are ignored after settling");

// No transitionend: the fallback timer settles.
resetCounters();
beginLayoutTransition();
flushTimers();
expect(resizeCalls === 1, "the fallback settles without transitionend");
expect(renderCalls === 1, "the fallback renders once");

// Reduced motion settles on the next animation frame, not the fallback timer.
reduceMotion = true;
resetCounters();
beginLayoutTransition();
flushTimers();
expect(resizeCalls === 0, "reduced motion ignores the fallback timer");
flushFrames();
expect(resizeCalls === 1, "reduced motion settles on the next frame");
reduceMotion = false;

// A resize storm while pending coalesces into the single settle flush.
resetCounters();
beginLayoutTransition();
noteLayoutResize();
noteLayoutResize();
noteLayoutResize();
expect(resizeCalls === 0, "no setSize while a transition is pending");
flushFrames();
expect(resizeCalls === 0, "pending resize requests do not schedule a frame");
flushTimers();
expect(resizeCalls === 1, "the settle flush resizes once");
expect(layoutCalls === 1, "the settle flush applies joystick layout once");
expect(hudCalls === 1, "the settle flush updates HUD placement once");
expect(renderCalls === 1, "the settle flush renders exactly once");

// Flushing while pending defers instead of resizing mid-transition.
resetCounters();
beginLayoutTransition();
doDeferredLayoutFlush();
expect(resizeCalls === 0, "a direct flush during a transition is deferred");
flushTimers();
expect(resizeCalls === 1, "the deferred flush runs at settle");

console.log("page transition ok");
"""


def test_frontend_layout_transition_state_machine_executes_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _LAYOUT_TRANSITION_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "page transition ok" in completed.stdout
```

- [ ] **Step 4: Update the system-tab, sidebar static, and sidebar harness tests**

Replace `test_system_tab_hides_sidebar_and_expands_stage` (currently lines 1533-1548) with:

```python
def test_system_tab_uses_shared_sidebar_collapse_model():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    set_tab_start = app_js.index("function setTab(tab)")
    set_tab_block = app_js[set_tab_start : app_js.index("function selectedPosePayload", set_tab_start)]

    assert 'document.querySelector(".app").classList.toggle("system-mode", tab === "system");' in set_tab_block
    assert 'const panelName = tab === "system" ? "" : (tab === "mapping" ? "mapping" : "navigation");' in set_tab_block
    assert "panel.dataset.panel === panelName" in set_tab_block
    assert "applyEffectiveSidebar();" in set_tab_block

    assert ".app.system-mode" in styles_css
    assert ".app.system-mode .panel" in styles_css
    system_panel_start = styles_css.index(".app.system-mode .panel")
    system_panel_block = styles_css[system_panel_start : styles_css.index("}", system_panel_start)]
    assert "display: none;" not in system_panel_block
    assert "opacity: 0;" in system_panel_block

    assert 'const effective = sidebarState.collapsed || state.view === "system";' in app_js
```

In `test_sidebar_toggle_is_single_accessible_button` (currently lines 6259-6278), replace the line:

```python
    assert "transition: grid-template-columns 260ms ease;" in styles_css
```

with:

```python
    assert "transition: grid-template-columns var(--layout-transition-ms) ease;" in styles_css
```

and add after `assert "function observeMapStageResize()" in app_js`:

```python
    assert "function applyEffectiveSidebar()" in app_js
```

Replace the whole `_SIDEBAR_NODE_HARNESS` string (currently lines 6281-6364) with:

```python
_SIDEBAR_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Sidebar collapse");
const blockEnd = source.indexOf("// --- End sidebar collapse");
if (blockStart < 0 || blockEnd < 0) throw new Error("sidebar block not found");
const block = source.slice(blockStart, blockEnd);
if (block.includes("localStorage") || block.includes("sessionStorage")) {
  throw new Error("sidebar state must not be persisted");
}

function makeElement() {
  const classes = new Set();
  return {
    hidden: false,
    inert: false,
    attributes: {},
    focused: false,
    classList: {
      toggle(name, on) { if (on) classes.add(name); else classes.delete(name); },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
    contains() { return false; },
    focus() { this.focused = true; },
  };
}

const appElement = makeElement();
const stageElement = makeElement();
const toggleElement = makeElement();
const panelElement = makeElement();
const elements = { sidebarToggle: toggleElement, controlPanel: panelElement };
const observers = [];
global.document = {
  activeElement: null,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) {
    if (selector === ".app") return appElement;
    if (selector === ".stage") return stageElement;
    return null;
  },
};
global.ResizeObserver = class {
  constructor(callback) { this.callback = callback; this.target = null; observers.push(this); }
  observe(target) { this.target = target; }
};
let transitions = 0;
let resizeNotes = 0;
global.state = { view: "3d" };
global.beginLayoutTransition = () => { transitions += 1; return transitions; };
global.noteLayoutResize = () => { resizeNotes += 1; };

const sidebarApi = new Function(block + `
return { sidebarState, applyEffectiveSidebar, setSidebarCollapsed, toggleSidebar, observeMapStageResize };`)();
const { sidebarState, applyEffectiveSidebar, setSidebarCollapsed, toggleSidebar, observeMapStageResize } = sidebarApi;

function expect(condition, message) { if (!condition) throw new Error(message); }

expect(sidebarState.collapsed === false, "sidebar starts expanded");
setSidebarCollapsed(false);
expect(toggleElement.getAttribute("aria-expanded") === "true", "expanded aria state");
expect(toggleElement.getAttribute("aria-label") === "Collapse sidebar", "expanded label");
expect(panelElement.inert === false, "expanded panel is interactive");
expect(transitions === 0, "no transition for an unchanged sidebar");

toggleSidebar();
expect(sidebarState.collapsed === true, "toggle collapses");
expect(appElement.classList.contains("sidebar-collapsed"), "collapsed class applied");
expect(toggleElement.getAttribute("aria-expanded") === "false", "collapsed aria state");
expect(toggleElement.getAttribute("aria-label") === "Expand sidebar", "collapsed label");
expect(panelElement.inert === true, "collapsed panel is inert");
expect(transitions === 1, "collapse starts one transition");

toggleSidebar();
expect(sidebarState.collapsed === false, "toggle expands");
expect(!appElement.classList.contains("sidebar-collapsed"), "collapsed class removed");
expect(toggleElement.getAttribute("aria-expanded") === "true", "re-expanded aria state");
expect(panelElement.inert === false, "re-expanded panel is interactive");
expect(transitions === 2, "expand starts one transition");

// Entering System collapses the effective sidebar but keeps the in-page state.
global.state.view = "system";
expect(applyEffectiveSidebar() === true, "entering system changes the effective collapse");
expect(appElement.classList.contains("sidebar-collapsed"), "system collapses the sidebar");
expect(sidebarState.collapsed === false, "in-page state is unchanged by system");
expect(toggleElement.getAttribute("aria-label") === "Collapse sidebar", "system keeps the in-page label");

// Leaving System restores the in-page expanded state.
global.state.view = "3d";
expect(applyEffectiveSidebar() === true, "leaving system changes the effective collapse");
expect(!appElement.classList.contains("sidebar-collapsed"), "leaving system restores expansion");
expect(applyEffectiveSidebar() === false, "an unchanged effective state starts no work");

observeMapStageResize();
expect(observers.length === 1, "one ResizeObserver created");
expect(observers[0].target === stageElement, "map stage observed");
observers[0].callback();
expect(resizeNotes === 1, "stage changes are coalesced through noteLayoutResize");

console.log("sidebar collapse ok");
"""
```

- [ ] **Step 5: Run the Task 3 tests to verify they fail**

Run:

```bash
python3 -m pytest -q \
  test/test_core.py::test_frontend_mounts_viewports_and_uses_inert \
  test/test_core.py::test_glass_no_filter_on_webgl_canvas \
  test/test_core.py::test_frontend_layout_transition_state_machine_static \
  test/test_core.py::test_frontend_white_flash_fix_present \
  test/test_core.py::test_frontend_safety_invariants_in_set_tab \
  test/test_core.py::test_frontend_sidebar_and_system_use_shared_collapse \
  test/test_core.py::test_frontend_layout_transition_state_machine_executes_under_node \
  test/test_core.py::test_system_tab_uses_shared_sidebar_collapse_model \
  test/test_core.py::test_sidebar_toggle_is_single_accessible_button \
  test/test_core.py::test_sidebar_collapse_state_toggles_without_persistence_under_node
```

Expected: all 10 FAIL (the state machine, mounted-viewport CSS, panel-views wrapper, collapse model, and updated harness do not exist yet).

- [ ] **Step 6: Replace the viewport display rules**

In `static/styles.css`, replace the current `.viewport` / `.viewport.active` rules (currently lines 275-283) with:

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

#view3d {
  z-index: 0;
  background: var(--scene-bg);
}

#viewSystem {
  z-index: 1;
}

#mapCanvas3d {
  background-color: var(--scene-bg);
}
```

- [ ] **Step 7: Replace the sidebar/System collapse CSS and panel-view crossfade**

In `static/styles.css`, delete the `.app.system-mode { grid-template-columns: minmax(0, 1fr); }` rule (currently lines 135-137) and the `.app.system-mode .panel { display: none; }` rule (currently lines 139-141). Keep `.app.sidebar-collapsed { --panel-width: 0px; }` (currently lines 131-133).

Replace the existing `.app.sidebar-collapsed .panel` rule (currently lines 726-730) with the combined collapse rule:

```css
.app.sidebar-collapsed .panel,
.app.system-mode .panel {
  opacity: 0;
  transform: translateX(12px);
  pointer-events: none;
}
```

Replace the `.app.system-mode .sidebar-toggle` rule (currently lines 765-767) with:

```css
.app.system-mode .sidebar-toggle {
  opacity: 0;
  visibility: hidden;
  pointer-events: none;
  transition: opacity var(--layout-transition-ms) ease,
              visibility 0s linear var(--layout-transition-ms);
}
```

Replace the `.panel-view` / `.panel-view.active` rules (currently lines 807-813) with:

```css
.panel-views {
  position: relative;
}

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

.panel-view:not(.active) {
  position: absolute;
  inset: 0;
}
```

- [ ] **Step 8: Add the scene color, render-loop gating, and resize routing in `static/app.js`**

Replace the scene/renderer setup (currently lines 53-57) with:

```js
const SCENE_BG = 0x0b0f19;
const scene = new THREE.Scene();
scene.background = new THREE.Color(SCENE_BG);
const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 5000);
const renderer = new THREE.WebGLRenderer({ canvas: mapCanvas3d, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setClearColor(0x0b0f19, 1);
```

Replace the `render()` function (currently lines 189-193) with:

```js
let mapRenderFrame = null;

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

function ensureMapRenderLoop() {
  if (mapRenderFrame === null && mapViewActive()) {
    mapRenderFrame = requestAnimationFrame(render);
  }
}

function pauseMapRenderLoop() {
  if (mapRenderFrame !== null) {
    cancelAnimationFrame(mapRenderFrame);
    mapRenderFrame = null;
  }
}
```

Replace `handleVisibilityChange` (currently lines 1311-1313) with:

```js
async function handleVisibilityChange() {
  if (document.hidden) {
    await stopTeleop("hidden");
    return;
  }
  noteLayoutResize();
}
```

In `attachEvents`, replace `window.addEventListener("resize", resizeAll);` (currently line 1905) with `window.addEventListener("resize", noteLayoutResize);`, and delete `window.addEventListener("resize", applyJoystickLayout);` (currently line 2005). Add `attachLayoutTransitionListener();` immediately after `observeMapStageResize();` (currently line 1961).

- [ ] **Step 9: Replace the sidebar block in `static/app.js`**

Replace the whole `// --- Sidebar collapse` block (currently lines 1706-1739) with:

```js
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
  if (effective && panel && toggle && panel.contains(document.activeElement)) {
    toggle.focus();
  }
  return true;
}

function setSidebarCollapsed(collapsed) {
  sidebarState.collapsed = Boolean(collapsed);
  if (applyEffectiveSidebar()) beginLayoutTransition();
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
```

- [ ] **Step 10: Add the page transition state machine**

Insert this block in `static/app.js` immediately after `// --- End sidebar collapse` and before `function maybeClearGoalFromActionStatus()`. It references only hoisted function declarations and the imported `glassMotionReduced`.

```js
// --- Page transition state machine
const LAYOUT_SETTLE_FALLBACK_MS = 420;
const LAYOUT_TRANSITION_PROPS = new Set(["grid-template-columns", "opacity", "transform"]);

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
  resizeAll(); // the only renderer.setSize call site
  applyJoystickLayout();
  updateHudPlacement();
  renderer.render(scene, camera); // synchronous settle frame
}

const layoutTransitionTargets = new Set();

function handleLayoutTransitionEnd(event) {
  if (!pageTransition.active) return;
  if (!LAYOUT_TRANSITION_PROPS.has(event.propertyName)) return;
  if (!layoutTransitionTargets.has(event.target)) return;
  settleLayoutTransition(pageTransition.token);
}

function attachLayoutTransitionListener() {
  const targets = [
    document.querySelector(".app"),
    document.getElementById("view3d"),
    document.getElementById("viewSystem"),
    document.getElementById("viewTabs"),
  ];
  targets.forEach((element) => {
    if (element) layoutTransitionTargets.add(element);
  });
  const appEl = document.querySelector(".app");
  if (appEl) appEl.addEventListener("transitionend", handleLayoutTransitionEnd);
}
// --- End page transition state machine
```

- [ ] **Step 11: Rewrite `setTab` and add the focus helper**

Add `focusIfInside` immediately before `function setTab(tab)` (currently line 1778), then replace the whole `setTab` function (currently lines 1778-1816) with:

```js
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
  setMappingJoystickVisible(tab === "mapping");
  if (tab !== "mapping") {
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
    beginLayoutTransition();
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
    pollMaps3d(true);
  } else if (tab === "mapping") {
    pollMappingState();
    pollMapping3d(true);
    pollMappingPath();
  } else if (tab === "system") {
    pollSystem();
  }
}
```

`start()` keeps its existing `resizeAll();` before `setTab("3d");` (currently lines 2091-2093); the startup `setTab("3d")` is a no-op because `appliedEffectiveCollapse` starts `false` and no active class changes, so no transition starts.

- [ ] **Step 12: Run the Task 3 tests to verify they pass**

Run:

```bash
python3 -m pytest -q \
  test/test_core.py::test_frontend_mounts_viewports_and_uses_inert \
  test/test_core.py::test_glass_no_filter_on_webgl_canvas \
  test/test_core.py::test_frontend_layout_transition_state_machine_static \
  test/test_core.py::test_frontend_white_flash_fix_present \
  test/test_core.py::test_frontend_safety_invariants_in_set_tab \
  test/test_core.py::test_frontend_sidebar_and_system_use_shared_collapse \
  test/test_core.py::test_frontend_layout_transition_state_machine_executes_under_node \
  test/test_core.py::test_system_tab_uses_shared_sidebar_collapse_model \
  test/test_core.py::test_sidebar_toggle_is_single_accessible_button \
  test/test_core.py::test_sidebar_collapse_state_toggles_without_persistence_under_node
```

Expected: `10 passed`.

- [ ] **Step 13: Run the full interaction regression set**

Run:

```bash
python3 -m pytest -q \
  test/test_core.py::test_frontend_map_selector_behavior_executes_under_node \
  test/test_core.py::test_frontend_navigation_controls_behavior_executes_under_node \
  test/test_core.py::test_mapping_joystick_is_a_mapping_only_sibling_overlay \
  test/test_core.py::test_frontend_exposes_mapping_tab_and_mapping_sidebar \
  test/test_core.py::test_frontend_exposes_system_tab_and_metric_targets \
  test/test_core.py::test_frontend_polls_system_api_only_for_system_tab \
  test/test_core.py::test_frontend_follows_system_light_and_dark_color_scheme \
  test/test_core.py::test_mobile_layout_keeps_control_panel_as_side_panel \
  test/test_teleop.py test/test_map_selection.py
```

Expected: all pass (349-pass baseline behavior). If `test_mapping_joystick_is_a_mapping_only_sibling_overlay` fails, read its exact assertions and restore the anchor it checks; the joystick markup must not move.

- [ ] **Step 14: Syntax-check the JavaScript**

Run:

```bash
node --check static/app.js && node --check static/liquid-glass.js
```

Expected: no output, exit code 0.

- [ ] **Step 15: Commit**

```bash
git add static/index.html static/styles.css static/app.js test/test_core.py
git commit -m "feat: add layout transition state machine and mounted viewports"
git status --porcelain
```

Expected status after the commit: only ` M config/web_control.yaml` remains.

---

## Task 4: README documentation, provenance verification, and final verification

**Files:**
- Modify: `README.md`
- Test: `test/test_core.py`

- [ ] **Step 1: Write the failing README test**

Add near `test_readme_documents_navigation_pause_and_cancel_service` (currently line 4951):

```python
def test_readme_documents_liquid_glass_transitions():
    readme = (Path(__file__).resolve().parents[1] / "README.md").read_text(
        encoding="utf-8"
    )

    assert "Liquid Glass" in readme
    assert "shuding/liquid-glass" in readme
    assert "SVG" in readme
    assert "?glass=" in readme
    assert "prefers-reduced-motion" in readme
    assert "prefers-reduced-transparency" in readme
    assert "saveData" in readme
    assert "独立实现" in readme
    assert "config/web_control.yaml" not in readme
```

- [ ] **Step 2: Run the README test to verify it fails**

Run:

```bash
python3 -m pytest -q test/test_core.py::test_readme_documents_liquid_glass_transitions
```

Expected: FAIL on `assert "Liquid Glass" in readme`.

- [ ] **Step 3: Add the README section and update the sidebar notes**

Insert a new section in `README.md` immediately after the `## 折叠右侧信息栏` section (which currently ends at line 49), before `## 暂停与恢复导航（Pause / Resume / Clear）`:

```markdown
## Liquid Glass 页面过渡

页面采用三档玻璃材质，并用同一套过渡状态机切换 **Navigation / Mapping / System** 视图与右侧信息栏。实现只使用 `static/liquid-glass.js`（能力探测）、`static/styles.css`（材质与动效）和 `static/app.js`（状态机），不新增框架、打包步骤或依赖。

- **三档材质**：`glass-full` 只对三个小型装饰元素（顶部标签滑块、模式滑块、侧栏切换镜片）使用共享的 SVG `feImage` + `feDisplacementMap` 位移滤镜 `url(#liquidDrop)`；`glass-light` 只对侧栏容器、HUD、保存对话框、System 卡片、CPU 核心面板和摇杆底座各加一层 `blur()`；`glass-off` 使用不透明高对比度颜色。WebGL 画布、终端/日志文本、密集数据、实时摇杆旋钮和安全操作按钮既没有位移也没有背景模糊。
- **能力探测顺序**：手动覆盖优先，其次 `prefers-reduced-transparency: reduce` 返回 `off`，其次 `navigator.connection.saveData === true` 返回 `light`，其次 Chromium 且 `CSS.supports` 接受 `backdrop-filter: url(#liquidDrop)` 返回 `full`，否则返回 `light`。
- **手动覆盖**：在 URL 上使用 `?glass=full`、`?glass=light` 或 `?glass=off`，或在运行时调用 `window.__webControlSetGlassMode("full" | "light" | "off")`，可强制材质档位用于测试或操作员降级。覆盖只保存在当前页面内存中，不写入后端、`localStorage` 或 `sessionStorage`。
- **无障碍与降级**：`prefers-reduced-motion: reduce` 把过渡时长压缩到 `1ms` 并关闭位移关键帧与背景动画，状态机在下一帧直接结算；`prefers-reduced-transparency: reduce` 通过 `glass-off` 提供不透明表面。隐藏视图与折叠侧栏使用 `inert` 与 `aria-hidden`，焦点会移动到可见的标签按钮或侧栏切换按钮；`:focus-visible` 显示 2px 焦点环，粗指针设备上的点击目标不小于 44px。
- **视图与侧栏过渡**：`#view3d` 与 `#viewSystem` 始终挂载，通过 `opacity`/`visibility`/`pointer-events`/`inert` 交叉淡入淡出，不再使用 `display` 切换。进入 System 与手动折叠共用同一个 `--panel-width` 网格过渡和同一套面板淡出规则；返回 Navigation/Mapping 时恢复进入 System 前的页面内折叠状态。
- **白色闪烁修复**：布局过渡期间禁止 `renderer.setSize`；`ResizeObserver` 与窗口 resize 只标记待刷新，结算时只调用一次 `resizeAll()`，随后在同一个任务内同步执行一次 `renderer.render`。画布清屏色 `setClearColor(0x0b0f19, 1)` 与 CSS `--scene-bg` 都匹配 `scene.background`。地图视图隐藏时 WebGL 渲染循环暂停，恢复时先同步渲染一帧再继续。
- **静态位移贴图**：`128x64` RGBA PNG，R/G 分别编码水平/垂直位移（中性值 `128`），在 `static/liquid-glass.js` 中作为 `LIQUID_DROP_MAP` 常量一次性内嵌，页面加载时只注入一次，不会在 resize、换标签或动画帧中重新生成。
- **来源说明**：位移技术参考 [shuding/liquid-glass](https://github.com/shuding/liquid-glass) 所演示的静态 `feImage` + `feDisplacementMap` 思路；本仓库的滤镜标记、位移贴图和能力探测均为独立实现，未复制上游代码或素材，因此当前不需要附带上游 MIT 许可声明。若今后复制了任何非平凡的上游代码、标记或素材，必须先补充上游版权与许可声明及 `THIRD_PARTY_NOTICES` 记录再提交评审。
```

Then update the existing `## 折叠右侧信息栏` bullets (currently lines 46 and 49) to match the new behavior:

- Replace the bullet at line 46 (`- 地图区域上的 ResizeObserver 在过渡期间调用现有的 Three.js resizeAll() ...`) with:
  `- 地图区域上的 ResizeObserver 与窗口 resize 在布局过渡期间只标记“待刷新”，过渡结算时才调用一次 Three.js resizeAll()，随后同步渲染一帧；画布宽高比与摇杆布局因此不会逐帧重建，也不会在过渡中途调整绘制缓冲区。`
- Replace the bullet at line 49 (`- **System** 标签页继续使用现有的隐藏面板布局并隐藏该切换按钮；...`) with:
  `- **System** 标签页与侧栏折叠共用同一个有效折叠值（`sidebarState.collapsed || state.view === "system"`）和同一段 `--panel-width` 过渡，不再用 display 隐藏面板；侧栏切换按钮在 System 中淡出并置为不可点击，返回 **Navigation** 或 **Mapping** 时恢复当前页面内的折叠状态并显示同一个按钮。`

- [ ] **Step 4: Run the README test to verify it passes**

Run:

```bash
python3 -m pytest -q test/test_core.py::test_readme_documents_liquid_glass_transitions
```

Expected: `1 passed`.

- [ ] **Step 5: Verify provenance and forbidden-content invariants**

Run:

```bash
grep -n "shuding/liquid-glass\|independently implemented" static/liquid-glass.js
grep -rn "localStorage\|sessionStorage" static/liquid-glass.js || true
grep -rn "colcon" static/ README.md || true
git diff --check
```

Expected: the two provenance lines print; the storage greps print nothing; the `colcon` grep prints nothing; `git diff --check` prints nothing and exits 0.

- [ ] **Step 6: Verify the helper source never regenerates the map and animates no filter**

Run:

```bash
grep -c "requestAnimationFrame\|setInterval\|ResizeObserver" static/liquid-glass.js || true
grep -n "transition:.*filter\|filter.*transition\|animation:.*filter" static/styles.css || true
```

Expected: the first command prints `0`; the second prints nothing.

- [ ] **Step 7: Run the focused final regression**

Run:

```bash
python3 -m pytest -q \
  test/test_core.py::test_frontend_hosts_liquid_glass_defs \
  test/test_core.py::test_liquid_glass_helper_exports_provenance_and_static_assets \
  test/test_core.py::test_liquid_glass_wiring_in_app \
  test/test_core.py::test_liquid_glass_capability_detection_under_node \
  test/test_core.py::test_glass_tokens_and_mode_classes \
  test/test_core.py::test_glass_baseline_reset_removes_stacked_blur \
  test/test_core.py::test_glass_full_tier_limits_displacement_to_three_decorative_surfaces \
  test/test_core.py::test_glass_light_tier_is_single_layer \
  test/test_core.py::test_glass_off_tokens_are_opaque \
  test/test_core.py::test_glass_tier_selectors_exclude_webgl_text_and_safety_controls \
  test/test_core.py::test_glass_focus_and_touch_targets \
  test/test_core.py::test_glass_reduced_motion_and_transparency_fallbacks \
  test/test_core.py::test_glass_water_drop_motion \
  test/test_core.py::test_glass_drop_class_is_timed_and_forced_reflowed \
  test/test_core.py::test_frontend_mounts_viewports_and_uses_inert \
  test/test_core.py::test_glass_no_filter_on_webgl_canvas \
  test/test_core.py::test_frontend_layout_transition_state_machine_static \
  test/test_core.py::test_frontend_white_flash_fix_present \
  test/test_core.py::test_frontend_safety_invariants_in_set_tab \
  test/test_core.py::test_frontend_sidebar_and_system_use_shared_collapse \
  test/test_core.py::test_frontend_layout_transition_state_machine_executes_under_node \
  test/test_core.py::test_system_tab_uses_shared_sidebar_collapse_model \
  test/test_core.py::test_sidebar_toggle_is_single_accessible_button \
  test/test_core.py::test_sidebar_collapse_state_toggles_without_persistence_under_node \
  test/test_core.py::test_sliding_selectors_use_ios_26_liquid_glass_material \
  test/test_core.py::test_sliding_selectors_use_transform_based_ios_spring_motion \
  test/test_core.py::test_mode_slider_uses_contained_spring_to_stay_inside_track \
  test/test_core.py::test_readme_documents_liquid_glass_transitions
```

Expected: `28 passed`.

- [ ] **Step 8: Run the syntax and compile checks**

Run:

```bash
node --check static/app.js && node --check static/liquid-glass.js
python3 -m py_compile web_control/*.py
git diff --check
```

Expected: all silent, exit code 0.

- [ ] **Step 9: Run the full pytest suite and report the known failure separately**

Run:

```bash
python3 -m pytest -q
```

Expected: exactly one failure, `test/test_core.py::test_web_control_yaml_declares_teleop_defaults`, with `assert 1.0 == 1.5` at the `teleop_max_linear_speed` line. This is the operator's intentional unrelated `config/web_control.yaml` change, is **not** part of this work, and must be reported separately. Every other test passes (baseline was `349 passed, 1 failed`; this plan adds 22 new tests and renames one, so expect `371 passed, 1 failed`).

- [ ] **Step 10: Verify the config file is untouched and never staged**

Run:

```bash
git status --porcelain
git diff --stat -- config/web_control.yaml
git diff --name-only --cached
```

Expected: `git status --porcelain` is exactly ` M config/web_control.yaml` (plus the untracked plan file if it is still untracked); `git diff --stat -- config/web_control.yaml` shows the same one-line change that existed before this work (`teleop_max_linear_speed: 1.5` → `1.0`); `git diff --name-only --cached` prints nothing. If `config/web_control.yaml` ever appears staged, run `git restore --staged config/web_control.yaml`.

- [ ] **Step 11: Commit the documentation**

```bash
git add README.md test/test_core.py
git commit -m "docs: document liquid glass transitions and fallbacks"
git status --porcelain
```

Expected status after the commit: only ` M config/web_control.yaml` (and the untracked plan file).

---

## Self-Review

### 1. Spec coverage

- Approved direction items 1-3 (conceptual reimplementation, three tiers, hidden defs + helper + Chromium-only full + light/off fallback): Task 1 (helper, defs host, provenance) and Task 2 (tier selectors, `glass-off`, reduced-transparency fallback via the helper).
- Items 4-5 (medium water drop, no animated filter properties, white-flash fix): Task 2 motion CSS + `markGlassDrop`; Task 3 `SCENE_BG`/`setClearColor`, render gating, deferred resize, synchronous settle render.
- Items 6-8 (map/sidebar full-width smoothness, mounted viewports, shared sidebar/System collapse): Task 3 CSS and `setTab`/`applyEffectiveSidebar`.
- Item 9 (one state machine, rapid/reentrant, fallback): Task 3 state machine + Node harness.
- Items 10-12 (safety, accessibility, performance rules): Task 3 `setTab` ordering and focus/inert; Task 2 focus/touch/reduced-motion; tier tests exclude WebGL/text/dense-data/knob/safety selectors.
- Item 13 (`config/web_control.yaml` preserved): Commit policy + Task 4 Step 10.
- Item 14 (never run `colcon`): Commit policy + verification only runs pytest/node/py_compile/git.
- Component treatment table: `glass-full` rule has exactly the three decorative selectors; the `glass-light` rule has exactly the six structural selectors; the baseline reset removes every previous blur/filter declaration; `.panel section`, `.terminal`, `.source`, `.sliding-tabs` container, canvas, dense data, knob, and safety labels receive no tier selector.
- Reference technique (single hidden defs, one filter, static map, `preserveAspectRatio="none"`, never regenerated, not `display: none`): Task 1 and the reproducible map section.
- DOM architecture (defs first child, mounted viewports, panel-views wrapper, toggle lens): Task 1 Step 1, Task 2 Step 1, Task 3 Step 1.
- CSS architecture (tokens, mounted viewport model, shared collapse, panel-view crossfade, baseline reset, full/light/off, motion, reduced motion, focus/touch): Task 2 and Task 3.
- JavaScript architecture (helper surface and detection order, app integration, state machine, `setTab`, sidebar toggle, System enter/leave, safety invariants): Task 1 and Task 3.
- State transitions table and observable guarantees: encoded in the Task 3 state-machine code and exercised by the Node harness (rapid tokens, stale events, fallback, reduced motion, resize storm, single flush).
- Fallback behavior table: helper detection order; `ResizeObserver` guard unchanged; `inert` assignment degrades to an expando; CSS-only defaults remain usable without the module.
- Accessibility and performance budget: Task 2 focus/touch/reduced-motion and tier exclusion tests; Task 3 canvas/`#view3d` filter-free test and render gating.
- Testing strategy: static contract tests, two new Node harnesses, the updated sidebar harness, the syntax/compile checks, and the full-suite run with the known failure separated.
- Non-goals (no persistence, no framework, no sound/haptics, no commit of `config/web_control.yaml`): helper has no storage APIs and tests assert that; no new files besides `static/liquid-glass.js`; Commits never target the config file.

Gaps found and fixed inline: (a) the initial spec snippet's combined thumb rule duplicated the per-selector `transform` declarations, so the plan edits the existing per-selector rules in place and keeps the existing equal-width/transform contract tests valid; (b) the first draft of `render()` could start two animation loops when `setTab("3d")` ran before `start()`'s direct `render()`, so `render()` now cancels any pending frame before scheduling; (c) `focusIfInside` was initially referenced from the sliced sidebar block, so `applyEffectiveSidebar` inlines its focus check and the sidebar Node harness needs no extra global.

### 2. Placeholder scan

No `TBD`, `TODO`, "implement later", "fill in details", "add appropriate error handling", or "similar to Task N" text. Every code step shows full code. Every test step shows the full test body. Every command names its exact file or `pytest` node id and its expected result. The displacement map has both a complete generator and the exact embedded base64, so no value is deferred.

### 3. Identifier consistency

- Helper: `initLiquidGlass`, `detectGlassMode`, `applyGlassMode`, `glassMode`, `glassMotionReduced`, `LIQUID_DROP_MAP`, `FILTER_ID` are used with the same names in Task 1 code, Task 1 tests, and Task 3's `glassMotionReduced()` call.
- State machine: `LAYOUT_SETTLE_FALLBACK_MS`, `pageTransition`, `beginLayoutTransition`, `settleLayoutTransition`, `noteLayoutResize`, `doDeferredLayoutFlush`, `attachLayoutTransitionListener`, `handleLayoutTransitionEnd`, `layoutTransitionTargets` match between code, static tests, and the Node harness.
- Render loop: `mapRenderFrame`, `mapViewActive`, `ensureMapRenderLoop`, `pauseMapRenderLoop`, `SCENE_BG` match between Task 3 code and static tests.
- Sidebar: `sidebarState`, `appliedEffectiveCollapse`, `syncSidebarToggle`, `applyEffectiveSidebar`, `setSidebarCollapsed`, `toggleSidebar` match between code, static tests, and the updated sidebar harness. The harness returns `applyEffectiveSidebar` too, and the test destructures it.
- Motion: `GLASS_DROP_FALLBACK_MS`, `markGlassDrop`, `glass-drop` match between Task 2 CSS, Task 2 JS, and the Task 2 test.
- DOM/CSS: `liquidGlassDefs`/`glass-defs`, `sidebar-toggle-lens`, `panel-views`/`panel-view`, `view3d`/`viewSystem`/`viewTabs`, `controlPanel`, `sidebarToggle` are identical everywhere they appear.

---

## Execution notes

- Implement tasks in order. Each task leaves the tree green except for the intentional `config/web_control.yaml` speed assertion, which is reported separately and never touched.
- The plan document is not committed by its author; Codex reviews and commits it.
- After implementation, the user compiles the ROS workspace. Do not run `colcon`.
