# Dynamic Liquid Glass Tab Track Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the whole upper-left `#viewTabs` track a dynamically generated SVG displacement (`#liquidTrackFilter`, sized to the real border box and DPR) in `glass-full`, a strong transparent blur in `glass-light`, and an opaque `glass-off`, while moving the full displacement off the selected-tab thumb and retaining the static `url(#liquidDrop)` on the mode thumb and sidebar lens.

**Architecture:** One new ES module `static/track-refraction.js` exports pure generator helpers (`smoothStep`, `roundedRectSDF`, `computeTrackDisplacementField`) and an idempotent lifecycle (`initTrackRefraction`) that owns one filter, one canvas, one `ResizeObserver`, one window `resize` listener, one DPR `matchMedia` watcher, and one coalesced generation-guarded render. `static/liquid-glass.js` gains an `onGlassModeChange` subscription that notifies listeners inside `try/catch`; `static/app.js` initializes the controller immediately after `initLiquidGlass()`; `static/styles.css` swaps the full-tier selector list so `.sliding-tabs` itself is displaced and the thumb is not. The generator adapts `shuding/liquid-glass` (MIT, Copyright (c) 2025 Shu Ding), documented by `THIRD_PARTY_NOTICES.md`, installed through `setup.py`.

**Tech Stack:** Vanilla ES modules, SVG filters, 2D canvas, `ResizeObserver`, `matchMedia`; Python `pytest` drives inline Node `-e` harnesses against the real module source.

**Constraints (apply to every task):**
- Work in the current uncommitted `main` working tree. Do **not** create a branch or worktree.
- Do **not** stage or commit anything. There are no commit steps in this plan.
- Do **not** run `colcon` and do **not** run the full test suite.
- Never modify, revert, stage, or commit `config/web_control.yaml`; it must stay `teleop_max_linear_speed: 1.0`.
- Do not change any existing transition duration token or the layout/state machine.
- Run all commands from the repo root: `/home/robot/web_ws/src/web_control`.
- If `from web_control import ...` fails during collection, prefix pytest with `source /opt/ros/humble/setup.bash && source /home/robot/web_ws/install/setup.bash &&`.

---

### Task 1: RED — static contracts and updated glass selector tests

**Files:**
- Modify: `test/test_core.py` — add new tests after `test_liquid_glass_helper_exports_provenance_and_static_assets` (line 385), and rewrite `test_glass_full_tier_limits_displacement_to_three_decorative_surfaces` (line 2114) and `test_glass_tier_selectors_exclude_webgl_text_and_safety_controls` (line 2166).

- [ ] **Step 1: Add the module/attribution/wiring static tests**

Insert after `test_liquid_glass_helper_exports_provenance_and_static_assets`:

```python
def test_track_refraction_module_exports_pure_and_lifecycle_api():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    source = (static_dir / "track-refraction.js").read_text(encoding="utf-8")

    for name in (
        "smoothStep",
        "roundedRectSDF",
        "computeTrackDisplacementField",
        "initTrackRefraction",
    ):
        assert f"export function {name}" in source
    assert "https://github.com/shuding/liquid-glass" in source
    assert "THIRD_PARTY_NOTICES.md" in source
    assert "MIT" in source
    assert "Shu Ding" in source
    assert 'setAttribute("color-interpolation-filters", "sRGB")' in source
    assert '"feImage"' in source
    assert '"feDisplacementMap"' in source
    assert "liquidTrackFilter" in source
    assert "ResizeObserver" in source
    assert "matchMedia" in source
    assert 'addEventListener("resize"' in source
    assert "onGlassModeChange" in source
    assert "localStorage" not in source
    assert "sessionStorage" not in source


def test_liquid_glass_exposes_safe_mode_subscription():
    source = (
        Path(__file__).resolve().parents[1] / "static" / "liquid-glass.js"
    ).read_text(encoding="utf-8")

    assert "export function onGlassModeChange(listener)" in source
    assert "modeListeners.add(listener)" in source
    assert "modeListeners.delete(listener)" in source
    apply_body = _js_function_body(source, "export function applyGlassMode(mode)")
    assert "notifyGlassMode(resolved);" in apply_body
    notify_body = _js_function_body(source, "function notifyGlassMode(mode)")
    assert "try {" in notify_body
    assert "catch" in notify_body


def test_track_refraction_is_wired_after_init_liquid_glass():
    app_js = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    assert 'import { initTrackRefraction } from "./track-refraction.js";' in app_js
    assert "initLiquidGlass();\ninitTrackRefraction();" in app_js


def test_third_party_notices_license_and_packaging():
    root = Path(__file__).resolve().parents[1]
    notice = (root / "THIRD_PARTY_NOTICES.md").read_text(encoding="utf-8")
    assert "shuding/liquid-glass" in notice
    assert "MIT License" in notice
    assert "Copyright (c) 2025 Shu Ding" in notice
    assert "Permission is hereby granted, free of charge" in notice
    assert "adapt" in notice.lower()

    setup_py = (root / "setup.py").read_text(encoding="utf-8")
    assert "THIRD_PARTY_NOTICES.md" in setup_py
    assert "share/{package_name}/static" in setup_py
    assert "share/{package_name}/static/vendor" in setup_py
    assert "share/{package_name}/config" in setup_py
    assert "share/{package_name}/launch" in setup_py
```

- [ ] **Step 2: Add the CSS tier/token/no-layout tests**

Insert after the block inserted in Step 1:

```python
def test_glass_full_track_uses_dynamic_filter_and_thumb_is_static():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    track = _css_rule_block(styles_css, ".glass-full .sliding-tabs")
    assert (
        "backdrop-filter: url(#liquidTrackFilter) blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1);"
        in track
    )
    assert (
        "-webkit-backdrop-filter: url(#liquidTrackFilter) blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1);"
        in track
    )
    shared = _css_rule_block(styles_css, ":where(.glass-full, .glass-light) .sliding-tabs")
    assert "background: var(--track-glass-bg);" in shared
    assert "border-color: var(--track-glass-edge);" in shared
    assert "0 4px 8px rgba(0, 0, 0, 0.25)" in shared
    assert "inset 0 -10px 25px rgba(0, 0, 0, 0.15)" in shared
    assert ".glass-full .sliding-tabs::before" not in styles_css

    static_start = styles_css.index(".glass-full .segmented::before,")
    static_body = styles_css[
        styles_css.index("{", static_start) + 1 : styles_css.index("}", static_start)
    ]
    assert "backdrop-filter: url(#liquidDrop) saturate(1.4);" in static_body
    assert "-webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);" in static_body
    assert ".glass-full .sidebar-toggle-lens {" in styles_css


def test_glass_light_track_fallback_and_track_tokens():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    light = _css_rule_block(styles_css, ".glass-light .sliding-tabs")
    assert "backdrop-filter: blur(22px) saturate(1.4);" in light
    assert "-webkit-backdrop-filter: blur(22px) saturate(1.4);" in light

    root = _css_rule_block(styles_css, ":root")
    assert "--track-glass-bg: rgba(255, 255, 255, 0.14);" in root
    assert "--track-glass-edge: rgba(255, 255, 255, 0.92);" in root
    assert "--track-glass-label: #f8fafc;" in root
    assert "--track-system-label: #111827;" in root
    assert "--track-glass-bg: rgba(8, 12, 20, 0.16);" in styles_css
    assert "--track-glass-label: #f5f7fb;" in styles_css
    assert "--track-system-label: #f5f7fb;" in styles_css

    off = _css_rule_block(styles_css, ".glass-off")
    assert "--track-glass-bg: rgba(255, 255, 255, 0.96);" in off
    assert "--track-glass-label: #111827;" in off
    assert "--track-glass-bg: rgba(28, 28, 34, 0.98);" in styles_css


def test_glass_system_mode_labels_use_dedicated_token():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )

    map_start = styles_css.index(".glass-full .sliding-tabs .tab:not(.active),")
    map_rule = styles_css[map_start : styles_css.index("}", map_start)]
    assert ".glass-light .sliding-tabs .tab:not(.active)" in map_rule
    assert "color: var(--track-glass-label);" in map_rule

    system_selector = (
        ".app.system-mode :where(.glass-full, .glass-light, .glass-off) "
        ".sliding-tabs .tab:not(.active)"
    )
    system_rule = _css_rule_block(styles_css, system_selector)
    assert "color: var(--track-system-label);" in system_rule


def test_glass_track_rules_add_no_layout_or_animation_properties():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    forbidden = (
        "width:",
        "height:",
        "padding",
        "margin",
        "grid",
        "position:",
        "top:",
        "left:",
        "right:",
        "bottom:",
        "inset:",
        "display:",
        "flex",
        "gap:",
        "font-size",
        "transform:",
        "transition:",
        "animation:",
        "will-change:",
    )
    for selector in (
        ":where(.glass-full, .glass-light) .sliding-tabs",
        ".glass-full .sliding-tabs",
        ".glass-light .sliding-tabs",
        ".glass-off .sliding-tabs",
        (
            ".app.system-mode :where(.glass-full, .glass-light, .glass-off) "
            ".sliding-tabs .tab:not(.active)"
        ),
    ):
        block = _css_rule_block(styles_css, selector)
        for prop in forbidden:
            assert prop not in block, (selector, prop)
```

- [ ] **Step 3: Rewrite the full-tier selector-count test**

Replace `test_glass_full_tier_limits_displacement_to_three_decorative_surfaces` entirely with:

```python
def test_glass_full_tier_limits_displacement_to_track_and_two_static_lenses():
    styles_css = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    assert ".glass-full .sliding-tabs {" in styles_css
    assert ".glass-full .segmented::before," in styles_css
    assert ".glass-full .sidebar-toggle-lens {" in styles_css
    assert ".glass-full .sliding-tabs::before" not in styles_css

    dynamic_start = styles_css.index(".glass-full .sliding-tabs {")
    dynamic_body = styles_css[
        styles_css.index("{", dynamic_start) + 1 : styles_css.index("}", dynamic_start)
    ]
    assert "url(#liquidTrackFilter)" in dynamic_body
    assert "url(#liquidDrop)" not in dynamic_body

    static_start = styles_css.index(".glass-full .segmented::before,")
    static_selectors = styles_css[static_start : styles_css.index("{", static_start)]
    assert static_selectors.count(".glass-full") == 2
    static_body = styles_css[
        styles_css.index("{", static_start) + 1 : styles_css.index("}", static_start)
    ]
    assert "backdrop-filter: url(#liquidDrop) saturate(1.4);" in static_body
    assert "-webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);" in static_body

    lens = _css_rule_block(styles_css, ".sidebar-toggle-lens")
    assert "position: absolute;" in lens
    assert "inset: 0;" in lens
    assert "border-radius: inherit;" in lens
    assert "background: var(--glass);" in lens
    assert "pointer-events: none;" in lens
```

- [ ] **Step 4: Update the safety-exclusion anchor test**

In `test_glass_tier_selectors_exclude_webgl_text_and_safety_controls`, replace:

```python
    full_start = styles_css.index(".glass-full .sliding-tabs::before,")
    full_selectors = styles_css[full_start : styles_css.index("{", full_start)]
    light_start = styles_css.index(":where(.glass-full, .glass-light) .panel,")
    light_selectors = styles_css[light_start : styles_css.index("{", light_start)]
```

with:

```python
    full_start = styles_css.index(".glass-full .sliding-tabs {")
    full_selectors = styles_css[full_start : styles_css.index("{", full_start)]
    static_start = styles_css.index(".glass-full .segmented::before,")
    full_selectors += styles_css[static_start : styles_css.index("{", static_start)]
    light_start = styles_css.index(".glass-light .sliding-tabs {")
    light_selectors = styles_css[light_start : styles_css.index("{", light_start)]
    where_start = styles_css.index(":where(.glass-full, .glass-light) .panel,")
    light_selectors += styles_css[where_start : styles_css.index("{", where_start)]
```

- [ ] **Step 5: Update the README contract test**

In `test_readme_documents_liquid_glass_transitions`, after `assert "独立实现" in readme`, add:

```python
    assert "liquidTrackFilter" in readme
    assert "THIRD_PARTY_NOTICES" in readme
    assert "MIT" in readme
    assert "未复制上游代码或素材" not in readme
    assert "无需附带上游 MIT 声明" not in readme
```

- [ ] **Step 6: Run the focused tests to verify RED**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_track_refraction_module_exports_pure_and_lifecycle_api \
  test/test_core.py::test_liquid_glass_exposes_safe_mode_subscription \
  test/test_core.py::test_track_refraction_is_wired_after_init_liquid_glass \
  test/test_core.py::test_third_party_notices_license_and_packaging \
  test/test_core.py::test_glass_full_track_uses_dynamic_filter_and_thumb_is_static \
  test/test_core.py::test_glass_light_track_fallback_and_track_tokens \
  test/test_core.py::test_glass_track_rules_add_no_layout_or_animation_properties \
  test/test_core.py::test_glass_full_tier_limits_displacement_to_track_and_two_static_lenses \
  test/test_core.py::test_glass_tier_selectors_exclude_webgl_text_and_safety_controls \
  test/test_core.py::test_readme_documents_liquid_glass_transitions -v
```

Expected: all FAIL for missing files/behaviour (`FileNotFoundError`,
`ValueError: substring not found`, `AssertionError`), never for syntax errors in
the test file.

---

### Task 2: RED — pure generator Node harness

**Files:**
- Modify: `test/test_core.py` — add `_TRACK_REFRACTION_FIELD_NODE_HARNESS` and `test_track_refraction_field_math_under_node` after Task 1's tests.

- [ ] **Step 1: Add the harness and test**

```python
_TRACK_REFRACTION_FIELD_NODE_HARNESS = r"""
const { pathToFileURL } = require("url");
const path = process.env.TRACK_REFRACTION;

function expect(condition, message) { if (!condition) throw new Error(message); }
function close(a, b, eps, message) {
  if (Math.abs(a - b) > eps) throw new Error(message + ": " + a + " vs " + b);
}

(async () => {
  const mod = await import(pathToFileURL(path).href + "?field");

  expect(mod.smoothStep(0, 1, 0.5) === 0.5, "smoothStep midpoint");
  expect(mod.smoothStep(0, 1, -2) === 0, "smoothStep clamps low");
  expect(mod.smoothStep(0, 1, 3) === 1, "smoothStep clamps high");
  expect(mod.smoothStep(5, 1, 3) === 0.5, "smoothStep descending midpoint");

  close(mod.roundedRectSDF(0, 0, 10, 6, 5), -6, 1e-9, "center distance");
  close(mod.roundedRectSDF(10, 0, 10, 6, 5), 0, 1e-9, "right edge");
  close(mod.roundedRectSDF(11, 0, 10, 6, 5), 1, 1e-9, "outside right");
  close(mod.roundedRectSDF(-10, 0, 10, 6, 5), 0, 1e-9, "left edge");
  close(mod.roundedRectSDF(0, 6, 10, 6, 5), 0, 1e-9, "top edge");
  close(
    mod.roundedRectSDF(4, 3, 10, 6, 5),
    mod.roundedRectSDF(-4, 3, 10, 6, 5),
    1e-9,
    "x symmetry",
  );
  expect(mod.roundedRectSDF(0, 0, 10, 6, 5) < 0, "inside is negative");
  expect(mod.roundedRectSDF(30, 30, 10, 6, 5) > 0, "outside is positive");

  const first = mod.computeTrackDisplacementField({ width: 320, height: 48, dpr: 2 });
  const second = mod.computeTrackDisplacementField({ width: 320, height: 48, dpr: 2 });
  expect(first.width === 640 && first.height === 96, "device dimensions honor DPR");
  expect(first.data.length === 640 * 96 * 4, "RGBA length");
  for (let i = 0; i < first.data.length; i += 1) {
    if (first.data[i] !== second.data[i]) throw new Error("field is not deterministic");
  }

  const centerIndex = ((first.height >> 1) * first.width + (first.width >> 1)) * 4;
  expect(first.data[centerIndex] === 128, "center R neutral");
  expect(first.data[centerIndex + 1] === 128, "center G neutral");
  expect(first.data[centerIndex + 2] === 0, "center B zero");
  expect(first.data[centerIndex + 3] === 255, "center alpha");

  const leftMidIndex = (first.height >> 1) * first.width * 4;
  expect(first.data[leftMidIndex] !== 128, "left rim mid-edge displaced horizontally");

  // Sample the field by CSS coordinate. Pixel centers sit at
  // x = (px + 0.5) / dpr - halfWidth, so invert that to find the index.
  function sampleChannel(field, cssWidth, cssHeight, cssDpr, cssX, cssY) {
    const halfW = cssWidth / 2;
    const halfH = cssHeight / 2;
    const px = Math.max(0, Math.min(field.width - 1, Math.round((cssX + halfW) * cssDpr - 0.5)));
    const py = Math.max(0, Math.min(field.height - 1, Math.round((cssY + halfH) * cssDpr - 0.5)));
    const i = (py * field.width + px) * 4;
    return { r: field.data[i], g: field.data[i + 1], px, py };
  }

  // A wide pill's straight top edge far from the rounded caps must refract
  // straight inward: the horizontal channel stays neutral and only G moves.
  const topEdge = sampleChannel(first, 320, 48, 2, 60, -23.25);
  expect(topEdge.r === 128, "top edge far from caps keeps R neutral");
  expect(topEdge.g > 200, "top edge far from caps drives G inward");

  // The rounded right cap must refract horizontally inward: R moves and G
  // stays approximately neutral.
  const rightCap = sampleChannel(first, 320, 48, 2, 158, 0);
  expect(rightCap.r < 80, "right cap drives R inward");
  expect(Math.abs(rightCap.g - 128) <= 8, "right cap keeps G approximately neutral");

  let rimDisplaced = false;
  let alphaOk = true;
  let outsideNeutral = true;
  let displacementBounded = true;
  const strength = Math.max(2, 48 * 0.12);
  for (let py = 0; py < first.height; py += 1) {
    const cssY = (py + 0.5) / 2 - 24;
    for (let px = 0; px < first.width; px += 1) {
      const i = (py * first.width + px) * 4;
      if (first.data[i] !== 128 || first.data[i + 1] !== 128) rimDisplaced = true;
      if (first.data[i + 3] !== 255 || first.data[i + 2] !== 0) alphaOk = false;
      const cssX = (px + 0.5) / 2 - 160;
      if (mod.roundedRectSDF(cssX, cssY, 160, 24, 24) > 0) {
        if (first.data[i] !== 128 || first.data[i + 1] !== 128) outsideNeutral = false;
      }
      const dx = (first.data[i] / 255 - 0.5) * first.scale;
      const dy = (first.data[i + 1] / 255 - 0.5) * first.scale;
      if (Math.abs(dx) > strength + 0.05 || Math.abs(dy) > strength + 0.05) {
        displacementBounded = false;
      }
    }
  }
  expect(rimDisplaced, "rim is displaced");
  expect(alphaOk, "every pixel is opaque with B=0");
  // CSS border-radius clips the corner outside the pill, so keep every outside
  // pixel neutral rather than cementing displacement that is never visible.
  expect(outsideNeutral, "every pixel outside the pill stays neutral");
  expect(first.data[0] === 128, "corner outside the pill stays R neutral");
  expect(first.data[1] === 128, "corner outside the pill stays G neutral");
  expect(displacementBounded, "decoded displacement stays bounded by strength");

  const dpr1 = mod.computeTrackDisplacementField({ width: 320, height: 48, dpr: 1 });
  const dpr2 = mod.computeTrackDisplacementField({ width: 320, height: 48, dpr: 2 });
  expect(dpr1.width === 320 && dpr2.width === 640, "DPR doubles device width");
  close(dpr1.scale, dpr2.scale, 1e-9, "scale is DPR independent (CSS pixels)");
  expect(dpr1.scale > 0, "scale is positive");

  const tiny = mod.computeTrackDisplacementField({ width: 0, height: 0, dpr: 0 });
  expect(tiny.width === 1 && tiny.height === 1, "degenerate size clamps to one device pixel");
  expect(Number.isFinite(tiny.scale), "degenerate scale is finite");

  console.log("track refraction field ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_track_refraction_field_math_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    module = Path(__file__).resolve().parents[1] / "static" / "track-refraction.js"
    completed = subprocess.run(
        ["node", "-e", _TRACK_REFRACTION_FIELD_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "TRACK_REFRACTION": str(module)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "track refraction field ok" in completed.stdout
```

- [ ] **Step 2: Run the harness test to verify RED**

Run:

```bash
python3 -m pytest test/test_core.py::test_track_refraction_field_math_under_node -v
```

Expected: FAIL with a Node `ERR_MODULE_NOT_FOUND` for
`static/track-refraction.js`.

---

### Task 3: RED — lifecycle Node harness

**Files:**
- Modify: `test/test_core.py` — add `_TRACK_REFRACTION_LIFECYCLE_NODE_HARNESS` and `test_track_refraction_lifecycle_under_node` after Task 2's test.

- [ ] **Step 1: Add the fake DOM harness**

```python
_TRACK_REFRACTION_LIFECYCLE_NODE_HARNESS = r"""
const { pathToFileURL } = require("url");
const path = process.env.TRACK_REFRACTION;

function expect(condition, message) { if (!condition) throw new Error(message); }

function makeElement(tag) {
  const element = {
    tag,
    tagName: tag.toUpperCase(),
    localName: tag,
    children: [],
    parentNode: null,
    attributes: {},
    style: {},
    id: "",
    setAttribute(name, value) {
      this.attributes[name] = String(value);
      if (name === "id") this.id = String(value);
    },
    getAttribute(name) {
      return Object.prototype.hasOwnProperty.call(this.attributes, name)
        ? this.attributes[name]
        : null;
    },
    setAttributeNS(_ns, name, value) { this.attributes[name] = String(value); },
    appendChild(child) { child.parentNode = this; this.children.push(child); return child; },
    append(...nodes) { nodes.forEach((node) => this.appendChild(node)); },
    remove() {
      if (!this.parentNode) return;
      const index = this.parentNode.children.indexOf(this);
      if (index >= 0) this.parentNode.children.splice(index, 1);
      this.parentNode = null;
    },
    querySelector(selector) {
      const id = selector.startsWith("#") ? selector.slice(1) : null;
      if (!id) return null;
      for (const child of this.children) {
        if (child.id === id) return child;
        const nested = child.querySelector ? child.querySelector(selector) : null;
        if (nested) return nested;
      }
      return null;
    },
    addEventListener() {},
    removeEventListener() {},
  };
  return element;
}

function makeCanvas() {
  const canvas = makeElement("canvas");
  canvas.width = 0;
  canvas.height = 0;
  canvas.puts = 0;
  canvas.dataUrls = 0;
  canvas.context = { putImageData(image) { canvas.puts += 1; canvas.lastImage = image; } };
  canvas.getContext = () => canvas.context;
  canvas.toDataURL = () => {
    canvas.dataUrls += 1;
    return "data:image/png;base64,canvas-" + canvas.width + "x" + canvas.height + "-" + canvas.dataUrls;
  };
  return canvas;
}

function makeDocument(track) {
  const defs = makeElement("defs");
  const document = {
    defs,
    created: [],
    getElementById(id) { return id === "viewTabs" ? track : null; },
    querySelector(selector) { return selector === "#liquidGlassDefs defs" ? defs : null; },
    createElementNS(_ns, tag) { const el = makeElement(tag); document.created.push(el); return el; },
    createElement(tag) { const el = tag === "canvas" ? makeCanvas() : makeElement(tag); document.created.push(el); return el; },
  };
  return document;
}

function makeTrack(width, height) {
  const track = makeElement("div");
  track.cssWidth = width;
  track.cssHeight = height;
  track.getBoundingClientRect = () => ({ width: track.cssWidth, height: track.cssHeight });
  return track;
}

const observerInstances = [];
class FakeResizeObserver {
  constructor(callback) { this.callback = callback; this.targets = []; this.disconnects = 0; observerInstances.push(this); }
  observe(target) { this.targets.push(target); }
  disconnect() { this.disconnects += 1; }
  trigger() { this.callback([], this); }
}

const frames = new Map();
let nextFrame = 1;
function requestAnimationFrame(cb) { const id = nextFrame++; frames.set(id, cb); return id; }
function cancelAnimationFrame(id) { frames.delete(id); }
function runFrames() {
  const pending = Array.from(frames.values());
  frames.clear();
  pending.forEach((cb) => cb());
}

function makeWindow() {
  const listeners = new Map();
  return {
    listeners,
    addEventListener(type, handler) {
      if (!listeners.has(type)) listeners.set(type, new Set());
      listeners.get(type).add(handler);
    },
    removeEventListener(type, handler) {
      if (listeners.has(type)) listeners.get(type).delete(handler);
    },
    dispatch(type) {
      if (listeners.has(type)) Array.from(listeners.get(type)).forEach((handler) => handler({ type }));
    },
  };
}

function makeMatchMedia() {
  const medias = [];
  function matchMedia(query) {
    const media = {
      query,
      listeners: new Set(),
      addEventListener(type, handler) { media.listeners.add(handler); },
      removeEventListener(type, handler) { media.listeners.delete(handler); },
    };
    medias.push(media);
    return media;
  }
  matchMedia.medias = medias;
  return matchMedia;
}

function makeModeChannel(initialMode) {
  let mode = initialMode;
  const listeners = new Set();
  return {
    getMode: () => mode,
    subscribe(listener) { listeners.add(listener); return () => { listeners.delete(listener); }; },
    listenerCount: () => listeners.size,
    set(nextMode) { mode = nextMode; Array.from(listeners).forEach((listener) => listener(nextMode)); },
  };
}

(async () => {
  const mod = await import(pathToFileURL(path).href + "?lifecycle");
  const track = makeTrack(320, 48);
  const document = makeDocument(track);
  const window = makeWindow();
  const matchMedia = makeMatchMedia();
  const channel = makeModeChannel("full");
  let dpr = 1;
  const options = {
    document,
    track,
    defs: document.defs,
    ResizeObserver: FakeResizeObserver,
    requestAnimationFrame,
    cancelAnimationFrame,
    window,
    matchMedia,
    getDevicePixelRatio: () => dpr,
    getGlassMode: channel.getMode,
    onGlassModeChange: channel.subscribe,
  };

  // Pre-seed an incomplete filter under the same id. init must remove it and
  // install exactly one complete filter, so the DOM never holds duplicate
  // #liquidTrackFilter nodes.
  const staleFilter = document.createElementNS("http://www.w3.org/2000/svg", "filter");
  staleFilter.setAttribute("id", "liquidTrackFilter");
  document.defs.appendChild(staleFilter);
  expect(
    document.defs.children.filter((child) => child.id === "liquidTrackFilter").length === 1,
    "incomplete filter pre-seeded",
  );

  const controller = mod.initTrackRefraction(options);
  expect(controller, "init returns a controller");
  const filter = document.defs.querySelector("#liquidTrackFilter");
  expect(filter !== null, "full install injects the filter");
  expect(
    document.defs.children.filter((child) => child.id === "liquidTrackFilter").length === 1,
    "incomplete filter is replaced, not duplicated",
  );
  expect(filter.children.some((child) => child.tag === "feImage"), "replacement filter has feImage");
  expect(
    filter.children.some((child) => child.tag === "feDisplacementMap"),
    "replacement filter has feDisplacementMap",
  );
  expect(filter.getAttribute("color-interpolation-filters") === "sRGB", "hyphenated color-interpolation-filters");
  expect(filter.getAttribute("filterUnits") === "userSpaceOnUse", "userSpaceOnUse filter units");
  expect(observerInstances.length === 1, "one ResizeObserver is created");
  expect(observerInstances[0].targets.length === 1, "the observer watches the track");
  expect(frames.size === 1, "install schedules one render frame");

  controller.render();
  controller.render();
  expect(frames.size === 1, "render requests coalesce");

  runFrames();
  const feImage = filter.children.find((child) => child.tag === "feImage");
  const displacement = filter.children.find((child) => child.tag === "feDisplacementMap");
  expect(feImage !== null && displacement !== null, "filter primitives exist");
  const canvas = document.created.find((el) => el.tag === "canvas");
  expect(canvas.width === 320 && canvas.height === 48, "canvas uses DPR 1 device size");
  expect(canvas.puts === 1, "the map is uploaded once");
  const firstHref = feImage.getAttribute("href");
  expect(typeof firstHref === "string" && firstHref.startsWith("data:image/png;base64,"), "feImage href is a data URL");
  expect(Number(displacement.getAttribute("scale")) > 0, "displacement scale is positive");
  expect(feImage.getAttribute("width") === "320", "feImage width is the CSS track width");
  expect(feImage.getAttribute("height") === "48", "feImage height is the CSS track height");
  expect(canvas.dataUrls === 1, "one data URL per render");

  controller.render();
  runFrames();
  expect(canvas.puts === 1, "unchanged size dedupes the upload");
  expect(canvas.dataUrls === 1, "unchanged size dedupes the data URL");

  track.cssWidth = 400;
  observerInstances[0].trigger();
  expect(frames.size === 1, "observer resize schedules one frame");
  runFrames();
  expect(canvas.width === 400, "resize rebuilds the canvas width");
  expect(canvas.puts === 2, "resize re-uploads the map");
  expect(canvas.dataUrls === 2, "resize rebuilds the data URL");
  expect(feImage.getAttribute("width") === "400", "resize updates the filter region");

  dpr = 2;
  window.dispatch("resize");
  runFrames();
  expect(canvas.width === 800, "DPR change rebuilds the canvas at device pixels");

  channel.set("light");
  expect(document.defs.querySelector("#liquidTrackFilter") === null, "light mode removes the filter");
  expect(observerInstances[0].disconnects === 1, "light mode disconnects the observer");
  expect(window.listeners.get("resize").size === 0, "light mode removes the window listener");

  controller.render();
  expect(frames.size === 0, "light mode never schedules a frame");

  channel.set("full");
  expect(document.defs.querySelector("#liquidTrackFilter") !== null, "re-enable reinstalls the filter");
  expect(observerInstances.length === 2, "re-enable creates a fresh observer");
  const putsBeforeReenable = canvas.puts;
  controller.render();
  runFrames();
  expect(canvas.puts === putsBeforeReenable + 1, "re-enable rebuilds the map");

  const again = mod.initTrackRefraction(options);
  expect(again === controller, "repeated init returns the same controller");
  expect(
    document.defs.children.filter((child) => child.id === "liquidTrackFilter").length === 1,
    "no duplicate filter",
  );

  controller.render();
  const captured = Array.from(frames.values());
  frames.clear();
  channel.set("off");
  const putsBeforeStale = canvas.puts;
  captured.forEach((cb) => cb());
  expect(canvas.puts === putsBeforeStale, "a stale callback cannot render after removal");

  channel.set("full");
  const putsBeforeDestroy = canvas.puts;
  controller.render();
  runFrames();
  expect(channel.listenerCount() >= 1, "controller is subscribed");
  const activeObserver = observerInstances[observerInstances.length - 1];
  controller.destroy();
  controller.destroy();
  expect(document.defs.querySelector("#liquidTrackFilter") === null, "destroy removes the filter");
  expect(channel.listenerCount() === 0, "destroy unsubscribes");
  expect(activeObserver.disconnects === 1, "destroy disconnects the active observer once");
  expect(window.listeners.get("resize").size === 0, "destroy removes the window listener");
  expect(matchMedia.medias[matchMedia.medias.length - 1].listeners.size === 0, "destroy removes the DPR watcher");
  controller.render();
  runFrames();
  expect(canvas.puts === putsBeforeDestroy + 1, "destroyed controller never renders again");

  console.log("track refraction lifecycle ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_track_refraction_lifecycle_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    module = Path(__file__).resolve().parents[1] / "static" / "track-refraction.js"
    completed = subprocess.run(
        ["node", "-e", _TRACK_REFRACTION_LIFECYCLE_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "TRACK_REFRACTION": str(module)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "track refraction lifecycle ok" in completed.stdout
```

- [ ] **Step 2: Run the harness test to verify RED**

Run:

```bash
python3 -m pytest test/test_core.py::test_track_refraction_lifecycle_under_node -v
```

Expected: FAIL with `ERR_MODULE_NOT_FOUND` for `static/track-refraction.js`.

---

### Task 4: GREEN — create `static/track-refraction.js`

**Files:**
- Create: `static/track-refraction.js`

- [ ] **Step 1: Write the full module**

Create `static/track-refraction.js` with the attribution header plus this exact body:

```javascript
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
// The interior and the region outside the pill are neutral (R=G=128). On the
// rim band the refraction follows the analytic rounded-rectangle SDF outward
// normal negated to point inward, so a wide pill refracts straight down its
// long edges and sideways on its caps instead of radially from the center. The
// displacement is bounded by `strength`, and `scale` is the feDisplacementMap
// scale in CSS pixels.
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
  // partial install (or a duplicated id) can leave more than one behind;
  // `querySelectorAll` catches them all in a real DOM, and the fake/test DOM
  // falls back to a bounded re-query loop.
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
      removeFilterNodes();
      const created = createFilter();
      filter = created.nextFilter;
      feImage = created.image;
      feDisplacementMap = created.displacement;
    }
    canvas = createCanvas();
    if (canvas && canvas.style) canvas.style.display = "none";
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
    canvas = null;
    context = null;
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
```

- [ ] **Step 2: Run the three new Node/static tests to verify GREEN**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_track_refraction_module_exports_pure_and_lifecycle_api \
  test/test_core.py::test_track_refraction_field_math_under_node \
  test/test_core.py::test_track_refraction_lifecycle_under_node -v
```

Expected: all PASS.

---

### Task 5: GREEN — `onGlassModeChange` in `static/liquid-glass.js`

**Files:**
- Modify: `static/liquid-glass.js` — provenance header (lines 1-8), add the listener set near the module state (line 29), add `onGlassModeChange` and `notifyGlassMode` after `glassMode()` (line 85), and call `notifyGlassMode(resolved)` in `applyGlassMode`.

- [ ] **Step 1: Update the provenance header**

Replace lines 1-8 with a header that keeps the phrase `independently implemented` but no longer claims no upstream notice is required:

```javascript
// Provenance: the static LIQUID_DROP_MAP asset, the #liquidDrop filter markup,
// and this capability helper are independently implemented for this
// repository. The dynamic tab-track generator in track-refraction.js adapts
// code from https://github.com/shuding/liquid-glass (MIT, Copyright (c) 2025
// Shu Ding); see THIRD_PARTY_NOTICES.md. The technique of feeding a static
// feImage displacement map into feDisplacementMap and referencing the filter
// from `backdrop-filter: url(#id)` follows that upstream demonstration.
```

- [ ] **Step 2: Add the subscription state and API**

After `let listenersAttached = false;` add:

```javascript
const modeListeners = new Set();
```

After `export function glassMode() { ... }` add:

```javascript
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
```

- [ ] **Step 3: Notify in `applyGlassMode`**

Replace:

```javascript
  if (resolved === "full") injectLiquidFilter();
  else removeLiquidFilter();
  return resolved;
```

with:

```javascript
  if (resolved === "full") injectLiquidFilter();
  else removeLiquidFilter();
  notifyGlassMode(resolved);
  return resolved;
```

- [ ] **Step 4: Run the subscription and capability tests to verify GREEN**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_liquid_glass_exposes_safe_mode_subscription \
  test/test_core.py::test_liquid_glass_helper_exports_provenance_and_static_assets \
  test/test_core.py::test_liquid_glass_capability_detection_under_node -v
```

Expected: all PASS.

---

### Task 6: GREEN — wire `initTrackRefraction` into `static/app.js`

**Files:**
- Modify: `static/app.js` — add the import at line 2-3 and call it after `initLiquidGlass();` (line 50).

- [ ] **Step 1: Add the import**

Replace:

```javascript
import { initLiquidGlass, glassMotionReduced } from "./liquid-glass.js";
```

with:

```javascript
import { initLiquidGlass, glassMotionReduced } from "./liquid-glass.js";
import { initTrackRefraction } from "./track-refraction.js";
```

- [ ] **Step 2: Initialize immediately after `initLiquidGlass()`**

Replace:

```javascript
initLiquidGlass();

const mapCanvas3d = document.getElementById("mapCanvas3d");
```

with:

```javascript
initLiquidGlass();
initTrackRefraction();

const mapCanvas3d = document.getElementById("mapCanvas3d");
```

- [ ] **Step 3: Run the wiring test and syntax check**

Run:

```bash
python3 -m pytest test/test_core.py::test_track_refraction_is_wired_after_init_liquid_glass -v
node --check static/app.js
```

Expected: PASS and exit 0.

---

### Task 7: GREEN — CSS whole-track treatment

**Files:**
- Modify: `static/styles.css` — `:root` tokens (after line 33), dark tokens (after line 76), the `.glass-off` block (lines 1445-1459), and the full tier (lines 1428-1433).

- [ ] **Step 1: Add the light track tokens**

After `--mode-track-glow: rgba(10, 132, 255, 0.16);` in `:root` add:

```css
  --track-glass-bg: rgba(255, 255, 255, 0.14);
  --track-glass-edge: rgba(255, 255, 255, 0.92);
  --track-glass-label: #f8fafc;
  --track-system-label: #111827;
```

- [ ] **Step 2: Add the dark track tokens**

After `--mode-track-glow: rgba(100, 210, 255, 0.08);` in the dark `:root` add:

```css
    --track-glass-bg: rgba(8, 12, 20, 0.16);
    --track-glass-edge: rgba(255, 255, 255, 0.36);
    --track-glass-label: #f5f7fb;
    --track-system-label: #f5f7fb;
```

- [ ] **Step 3: Replace the full-tier selector list**

Replace:

```css
.glass-full .sliding-tabs::before,
.glass-full .segmented::before,
.glass-full .sidebar-toggle-lens {
  backdrop-filter: url(#liquidDrop) saturate(1.4);
  -webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);
}
```

with:

```css
:where(.glass-full, .glass-light) .sliding-tabs {
  background: var(--track-glass-bg);
  border-color: var(--track-glass-edge);
  box-shadow:
    0 4px 8px rgba(0, 0, 0, 0.25),
    inset 0 -10px 25px rgba(0, 0, 0, 0.15);
}

.glass-full .sliding-tabs .tab:not(.active),
.glass-light .sliding-tabs .tab:not(.active) {
  color: var(--track-glass-label);
}

.glass-full .sliding-tabs {
  backdrop-filter: url(#liquidTrackFilter) blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1);
  -webkit-backdrop-filter: url(#liquidTrackFilter) blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1);
}

.glass-light .sliding-tabs {
  backdrop-filter: blur(22px) saturate(1.4);
  -webkit-backdrop-filter: blur(22px) saturate(1.4);
}

.glass-full .segmented::before,
.glass-full .sidebar-toggle-lens {
  backdrop-filter: url(#liquidDrop) saturate(1.4);
  -webkit-backdrop-filter: url(#liquidDrop) saturate(1.4);
}
```

- [ ] **Step 4: Extend the `glass-off` tokens**

In the light `.glass-off` block add:

```css
  --track-glass-bg: rgba(255, 255, 255, 0.96);
  --track-glass-edge: rgba(255, 255, 255, 0.92);
  --track-glass-label: #111827;
```

In the dark `.glass-off` block add:

```css
    --track-glass-bg: rgba(28, 28, 34, 0.98);
    --track-glass-edge: rgba(255, 255, 255, 0.16);
    --track-glass-label: #f5f7fb;
```

After the `.glass-off` media query add:

```css
.glass-off .sliding-tabs {
  background: var(--track-glass-bg);
  border-color: var(--track-glass-edge);
}

.glass-off .sliding-tabs .tab:not(.active) {
  color: var(--track-glass-label);
}

/* System has its own light/dark surface, so its inactive labels use the
   scheme-aware --track-system-label in every glass tier. Map views keep the
   light --track-glass-label over their dark canvas. Color only. */
.app.system-mode :where(.glass-full, .glass-light, .glass-off) .sliding-tabs .tab:not(.active) {
  color: var(--track-system-label);
}
```

- [ ] **Step 5: Run the CSS tests to verify GREEN**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_glass_full_track_uses_dynamic_filter_and_thumb_is_static \
  test/test_core.py::test_glass_light_track_fallback_and_track_tokens \
  test/test_core.py::test_glass_track_rules_add_no_layout_or_animation_properties \
  test/test_core.py::test_glass_full_tier_limits_displacement_to_track_and_two_static_lenses \
  test/test_core.py::test_glass_tier_selectors_exclude_webgl_text_and_safety_controls \
  test/test_core.py::test_glass_baseline_reset_removes_stacked_blur \
  test/test_core.py::test_glass_light_tier_is_single_layer \
  test/test_core.py::test_glass_off_tokens_are_opaque \
  test/test_core.py::test_glass_water_drop_motion -v
```

Expected: all PASS.

---

### Task 8: GREEN — notices, packaging, README

**Files:**
- Create: `THIRD_PARTY_NOTICES.md`
- Modify: `setup.py` line 16
- Modify: `README.md` lines 56, 63, 64

- [ ] **Step 1: Create `THIRD_PARTY_NOTICES.md`**

Copy the exact upstream MIT text and the adaptation description (the plan stores the full text; see the source file created in this task).

- [ ] **Step 2: Install the notice through `setup.py`**

Replace:

```python
        (f"share/{package_name}", ["package.xml"]),
```

with:

```python
        (f"share/{package_name}", ["package.xml", "THIRD_PARTY_NOTICES.md"]),
```

- [ ] **Step 3: Update the README Liquid Glass bullets**

Replace bullets 56, 63, and 64 with the dynamic-track-map wording from the spec.

- [ ] **Step 4: Run the notice/README tests to verify GREEN**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_third_party_notices_license_and_packaging \
  test/test_core.py::test_readme_documents_liquid_glass_transitions -v
```

Expected: both PASS.

---

### Task 9: Verification and adjacent regressions

**Files:**
- No source edits. Verification only.

- [ ] **Step 1: Syntax-check the three modules**

Run:

```bash
node --check static/track-refraction.js && node --check static/liquid-glass.js && node --check static/app.js
```

Expected: exit 0, no output.

- [ ] **Step 2: Run the new and updated focused tests**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_track_refraction_module_exports_pure_and_lifecycle_api \
  test/test_core.py::test_track_refraction_field_math_under_node \
  test/test_core.py::test_track_refraction_lifecycle_under_node \
  test/test_core.py::test_liquid_glass_exposes_safe_mode_subscription \
  test/test_core.py::test_liquid_glass_capability_detection_under_node \
  test/test_core.py::test_track_refraction_is_wired_after_init_liquid_glass \
  test/test_core.py::test_third_party_notices_license_and_packaging \
  test/test_core.py::test_glass_full_track_uses_dynamic_filter_and_thumb_is_static \
  test/test_core.py::test_glass_light_track_fallback_and_track_tokens \
  test/test_core.py::test_glass_track_rules_add_no_layout_or_animation_properties \
  test/test_core.py::test_glass_full_tier_limits_displacement_to_track_and_two_static_lenses \
  test/test_core.py::test_glass_tier_selectors_exclude_webgl_text_and_safety_controls \
  test/test_core.py::test_readme_documents_liquid_glass_transitions -v
```

Expected: every selected test PASS.

- [ ] **Step 3: Run adjacent regression checks**

Run:

```bash
python3 -m pytest \
  test/test_core.py::test_map_render_loop_lifecycle_executes_under_node \
  test/test_core.py::test_frontend_layout_transition_state_machine_static \
  test/test_core.py::test_frontend_layout_transition_state_machine_executes_under_node \
  test/test_core.py::test_frontend_white_flash_fix_present \
  test/test_core.py::test_joystick_visibility_lifecycle_executes_under_node \
  test/test_core.py::test_set_tab_reentrant_grid_defers_joystick_reveal_under_node \
  test/test_core.py::test_mapping_joystick_resize_handle_stays_inside_viewport_under_node \
  test/test_core.py::test_sidebar_collapse_state_toggles_without_persistence_under_node \
  test/test_core.py::test_top_tabs_use_sliding_segmented_selector \
  test/test_core.py::test_sliding_selectors_use_ios_26_liquid_glass_material -v
```

Expected: every selected test PASS.

- [ ] **Step 4: Check whitespace and the protected config**

Run:

```bash
git diff --check
grep -n "teleop_max_linear_speed" config/web_control.yaml
git status --short
```

Expected: `git diff --check` empty; the speed line reads `teleop_max_linear_speed: 1.0`;
`config/web_control.yaml` remains dirty from before this work and is never staged.

---

## Self-Review

**Spec coverage:**
- Whole-track `url(#liquidTrackFilter)` full filter → Task 7 + Task 1 Step 2 + Task 1 Step 3.
- Displacement moved off `.sliding-tabs::before`; static `url(#liquidDrop)` retained on `.segmented::before`/`.sidebar-toggle-lens` → Task 7 Step 3 + Task 1 Step 3.
- Bounded gradient/rim map at real border-box + DPR, analytic rounded-rect inward normal, neutral outside the pill → Task 4 `computeTrackDisplacementField` + Task 2.
- `ResizeObserver` + window resize/DPR, coalesced, generation-guarded, full/light/off lifecycle, idempotent destroy, removes every matching filter so repeated init never duplicates the id → Task 4 `initTrackRefraction` + Task 3.
- `onGlassModeChange` with safe listener errors → Task 5 + Task 1 Step 1.
- `app.js` init immediately after `initLiquidGlass()` → Task 6.
- Dedicated transparent tokens, light fallback, off opacity, readable labels, and the System-only `--track-system-label` rule → Task 7 + Task 1 Step 2.
- No layout properties, no animated filter → Task 7 + Task 1 Step 2.
- Attribution header, notice, MIT license, `setup.py` packaging → Task 8 + Task 1 Step 1.
- README rewrite → Task 8 Step 3 + Task 1 Step 5.
- Preserved safety/text/WebGL no-filter tests → Task 1 Step 4.
- No `config/web_control.yaml` change, no colcon, no staging → Constraints + Task 9 Step 4.

**Placeholder scan:** No TBD/TODO placeholders; Tasks 1-7 show the literal code and Task 8 uses the spec and this plan for the notice text.

**Type consistency:** `smoothStep`, `roundedRectSDF`, `computeTrackDisplacementField({ width, height, dpr }) -> { width, height, dpr, scale, data }`, `initTrackRefraction(options) -> controller`, `controller.render()`, `controller.destroy()`, `controller.isAlive()`, `onGlassModeChange(listener)`, `notifyGlassMode(mode)`, `#liquidTrackFilter`, `--track-glass-bg/edge/label`, and `--track-system-label` are spelled identically across production code, harnesses, and static assertions.
