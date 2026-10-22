"""Focused tests for the reusable segmented-control drag controller.

The capsule (the ``::before`` pill) used to be positioned with a CSS-only
``translate3d(index * 100%)`` against a fixed one-third width. Because
``*{box-sizing:border-box}`` does not reach pseudo-elements, the capsule's own
1px borders were added on top of every ``100%`` step, and because the
shrink-to-fit ``grid-template-columns: repeat(3, minmax(96px, 1fr))`` track did
not force equal columns, the capsule drifted right and overflowed the track at
the rightmost segment. These tests pin the pixel-measured replacement behaviour
and the reusable drag controller before the production edit lands.
"""

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

STATIC = Path(__file__).resolve().parents[1] / "static"
MODULE_JS = STATIC / "segmented-control.js"


def _read(name):
    return (STATIC / name).read_text(encoding="utf-8")


def _run_node(script, env=None):
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    completed = subprocess.run(
        ["node", "-e", script],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, **(env or {})},
    )
    assert completed.returncode == 0, completed.stderr
    return completed.stdout


_GEOMETRY_NODE = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.MODULE_JS, "utf8").replace(/^export /gm, "");
const api = new Function("window", source + "\nreturn { buildSegmentLayout, thumbXForIndex, nearestSegmentIndex, dragThumbX, clamp };");
const mod = api({ addEventListener() {}, removeEventListener() {} });

function expect(condition, message) { if (!condition) throw new Error(message); }

// Build a realistic shrink-to-fit track: 1px border, 4px padding, buttons laid
// out inside the content box. Equal-width and ragged layouts both appear.
function layoutFor(trackLeft, borderLeft, borderRight, padding, widths) {
  const trackWidth = borderLeft + borderRight + padding * 2 + widths.reduce((a, b) => a + b, 0);
  let x = trackLeft + borderLeft + padding;
  const rects = widths.map((width) => {
    const rect = { left: x, width };
    x += width;
    return rect;
  });
  return mod.buildSegmentLayout(
    { left: trackLeft, width: trackWidth },
    rects,
    borderLeft,
    borderRight,
  );
}

function close(a, b) { return Math.abs(a - b) < 1e-9; }

// Exact left/middle/right positions at multiple widths, with equal columns.
for (const trackWidth of [280, 310, 406, 520]) {
  const column = (trackWidth - 10) / 3;
  const layout = layoutFor(40, 1, 1, 4, [column, column, column]);
  expect(close(mod.thumbXForIndex(layout, 0), 4), "left x at width " + trackWidth);
  expect(close(mod.thumbXForIndex(layout, 1), 4 + column), "middle x at width " + trackWidth);
  expect(close(mod.thumbXForIndex(layout, 2), 4 + column * 2), "right x at width " + trackWidth);
  for (let index = 0; index < 3; index += 1) {
    const x = mod.thumbXForIndex(layout, index);
    expect(x + layout.segments[index].width <= layout.innerWidth + 1e-9,
      "capsule " + index + " overflows at width " + trackWidth);
  }
}

// Ragged columns (the real shrink-to-fit failure) still fit exactly, because the
// capsule takes each segment's measured width instead of one third.
const ragged = layoutFor(100, 1, 1, 4, [104, 96, 96]);
expect(close(mod.thumbXForIndex(ragged, 0), 4), "ragged left x");
expect(close(mod.thumbXForIndex(ragged, 1), 108), "ragged middle x");
expect(close(mod.thumbXForIndex(ragged, 2), 204), "ragged right x");
expect(ragged.segments[2].width === 96, "ragged right width");
expect(204 + 96 <= ragged.innerWidth + 1e-9, "ragged right capsule fits");

// Clamp keeps drag x inside the inner track for any overshoot.
expect(close(mod.dragThumbX(ragged, 96, -500), 0), "drag clamps at the left edge");
expect(close(mod.dragThumbX(ragged, 96, 5000), ragged.innerWidth - 96), "drag clamps at the right edge");

// Nearest-segment snap uses capsule centres, not the raw left edge.
expect(mod.nearestSegmentIndex(ragged, 4 + 52) === 0, "left centre snaps to 0");
expect(mod.nearestSegmentIndex(ragged, 108 + 48) === 1, "middle centre snaps to 1");
expect(mod.nearestSegmentIndex(ragged, 204 + 48) === 2, "right centre snaps to 2");
expect(mod.nearestSegmentIndex(ragged, 0) === 0, "far left snaps to 0");
expect(mod.nearestSegmentIndex(ragged, 10000) === 2, "far right snaps to 2");

console.log("segmented geometry ok");
"""


def test_segmented_geometry_exact_positions_and_no_overflow():
    if not MODULE_JS.exists():
        pytest.fail("static/segmented-control.js module is missing")
    stdout = _run_node(_GEOMETRY_NODE, {"MODULE_JS": str(MODULE_JS)})
    assert "segmented geometry ok" in stdout


_CONTROLLER_NODE = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.MODULE_JS, "utf8").replace(/^export /gm, "");

function makeStyle() {
  const props = new Map();
  return {
    setProperty(name, value) { props.set(name, String(value)); },
    getPropertyValue(name) { return props.has(name) ? props.get(name) : ""; },
  };
}

function makeElement(rect) {
  const listeners = new Map();
  const classes = new Set();
  const el = {
    rect,
    style: makeStyle(),
    classList: {
      add(name) { classes.add(name); },
      remove(name) { classes.delete(name); },
      contains(name) { return classes.has(name); },
    },
    addEventListener(type, handler) {
      if (!listeners.has(type)) listeners.set(type, []);
      listeners.get(type).push(handler);
    },
    removeEventListener(type, handler) {
      const list = listeners.get(type) || [];
      const index = list.indexOf(handler);
      if (index >= 0) list.splice(index, 1);
    },
    setPointerCapture(id) { el.captured = id; },
    releasePointerCapture(id) { if (el.captured === id) el.captured = null; },
    getBoundingClientRect() { return Object.assign({}, el.rect); },
    dispatch(type, event) {
      for (const handler of Array.from(listeners.get(type) || [])) handler(event);
    },
  };
  Object.defineProperty(el, "offsetWidth", { get() { return el.rect.width; } });
  el.classes = classes;
  el.listeners = listeners;
  return el;
}

const windowListeners = new Map();
const windowStub = {
  addEventListener(type, handler) {
    if (!windowListeners.has(type)) windowListeners.set(type, []);
    windowListeners.get(type).push(handler);
  },
  removeEventListener(type, handler) {
    const list = windowListeners.get(type) || [];
    const index = list.indexOf(handler);
    if (index >= 0) list.splice(index, 1);
  },
  dispatch(type, event) {
    for (const handler of Array.from(windowListeners.get(type) || [])) handler(event);
  },
  getComputedStyle() { return { borderLeftWidth: "1px", borderRightWidth: "1px" }; },
  setTimeout() { return 1; },
  clearTimeout() {},
};
const api = new Function(
  "window",
  source + "\nreturn { createSegmentedControl, SEGMENT_DRAG_THRESHOLD_PX };",
);
const mod = api(windowStub);

function expect(condition, message) { if (!condition) throw new Error(message); }
function pointer(id, x) {
  return {
    pointerId: id,
    clientX: x,
    button: 0,
    defaultPrevented: false,
    preventDefault() { this.defaultPrevented = true; },
  };
}

const track = makeElement({ left: 100, top: 0, width: 310, height: 40 });
const buttons = [
  makeElement({ left: 105, width: 100 }),
  makeElement({ left: 205, width: 100 }),
  makeElement({ left: 305, width: 100 }),
];
buttons[0].classList.add("active");

const selected = [];
const control = mod.createSegmentedControl({
  track,
  buttons,
  onSelect: (index) => selected.push(index),
});

expect(track.style.getPropertyValue("--thumb-x") === "4px", "initial thumb x");
expect(track.style.getPropertyValue("--thumb-w") === "100px", "initial thumb width");

// Tiny jitter stays a click: no drag class, no selection.
track.dispatch("pointerdown", pointer(1, 150));
windowStub.dispatch("pointermove", pointer(1, 152));
expect(!track.classList.contains("thumb-dragging"), "jitter must not start a drag");
expect(selected.length === 0, "jitter must not select");
windowStub.dispatch("pointerup", pointer(1, 152));
expect(selected.length === 0, "click release must not select through the controller");

// A real drag follows the pointer continuously, then selects exactly once.
track.dispatch("pointerdown", pointer(2, 150));
const dragMove = pointer(2, 270);
windowStub.dispatch("pointermove", dragMove);
expect(track.classList.contains("thumb-dragging"), "drag class active while dragging");
expect(track.style.getPropertyValue("--thumb-x") === "124px", "thumb follows the pointer");
expect(dragMove.defaultPrevented, "drag move prevents the default gesture");
windowStub.dispatch("pointerup", pointer(2, 270));
expect(selected.length === 1 && selected[0] === 1, "release selects the nearest segment once");
expect(!track.classList.contains("thumb-dragging"), "drag class cleared on release");
expect(track.style.getPropertyValue("--thumb-x") === "104px", "release snaps to segment 1");

// The synthetic click that follows a drag must not trigger the button path.
let prevented = false;
let stopped = false;
const suppressedClick = {
  preventDefault() { prevented = true; },
  stopPropagation() { stopped = true; },
  stopImmediatePropagation() { stopped = true; },
};
track.dispatch("click", suppressedClick);
expect(prevented && stopped, "drag release suppresses the follow-up click");
prevented = false;
stopped = false;
track.dispatch("click", suppressedClick);
expect(!prevented, "a later real click is not suppressed");

// Cancel snaps back to the current segment and never selects.
track.dispatch("pointerdown", pointer(3, 150));
windowStub.dispatch("pointermove", pointer(3, 260));
windowStub.dispatch("pointercancel", pointer(3, 260));
expect(selected.length === 1, "cancel must not select");
expect(!track.classList.contains("thumb-dragging"), "cancel clears the drag class");
expect(track.style.getPropertyValue("--thumb-x") === "4px", "cancel snaps back to the active segment");

// Lost pointer capture is handled like a cancel.
track.dispatch("pointerdown", pointer(4, 150));
windowStub.dispatch("pointermove", pointer(4, 260));
track.dispatch("lostpointercapture", pointer(4, 260));
expect(selected.length === 1, "lost capture must not select");
expect(track.style.getPropertyValue("--thumb-x") === "4px", "lost capture snaps back");

// Resize recomputes exact pixel positions; the rightmost capsule stays inside.
track.rect.width = 410;
buttons[0].rect = { left: 105, width: 130 };
buttons[1].rect = { left: 235, width: 130 };
buttons[2].rect = { left: 365, width: 130 };
control.refresh();
control.setActive(2);
const resizedX = parseFloat(track.style.getPropertyValue("--thumb-x"));
const resizedWidth = parseFloat(track.style.getPropertyValue("--thumb-w"));
expect(Math.abs(resizedX - 264) < 1e-9, "resized right x");
expect(Math.abs(resizedWidth - 130) < 1e-9, "resized right width");
expect(resizedX + resizedWidth <= 408 + 1e-9, "rightmost capsule stays inside after resize");

console.log("segmented controller ok");
"""


def test_segmented_controller_drag_click_cancel_and_resize():
    if not MODULE_JS.exists():
        pytest.fail("static/segmented-control.js module is missing")
    stdout = _run_node(_CONTROLLER_NODE, {"MODULE_JS": str(MODULE_JS)})
    assert "segmented controller ok" in stdout


def _css_rule_block(styles_css, selector):
    start = styles_css.index(f"{selector} {{")
    end = styles_css.index("}", start)
    return styles_css[start:end]


def test_capsule_uses_measured_pixel_position_not_index_percent():
    styles = _read("styles.css")
    for selector in (".sliding-tabs::before", ".segmented::before"):
        block = _css_rule_block(styles, selector)
        assert "left: 0;" in block, selector
        assert "width: var(--thumb-w, calc((100% - 8px) / 3));" in block, selector
        assert "transform: translate3d(var(--thumb-x, 4px), 0, 0);" in block, selector
        assert "box-sizing: border-box;" in block, selector
        assert "* 100%" not in block, selector
    assert "--tabs-index" not in styles
    assert "--mode-index" not in styles


def test_capsule_disables_transition_while_dragging():
    styles = _read("styles.css")
    block = _css_rule_block(styles, ".sliding-tabs.thumb-dragging::before,\n.segmented.thumb-dragging::before")
    assert "transition: none;" in block


def test_left_orb_after_is_gone():
    styles = _read("styles.css")
    assert ".sliding-tabs::after" not in styles
    assert ".segmented::after" not in styles
    assert "glassDropScale" in styles


def _hex_to_rgb(value):
    value = value.lstrip("#")
    return tuple(int(value[index : index + 2], 16) for index in (0, 2, 4))


def _relative_luminance(rgb):
    channels = []
    for channel in rgb:
        scaled = channel / 255.0
        channels.append(scaled / 12.92 if scaled <= 0.03928 else ((scaled + 0.055) / 1.055) ** 2.4)
    return 0.2126 * channels[0] + 0.7152 * channels[1] + 0.0722 * channels[2]


def _contrast(foreground, background):
    light = max(_relative_luminance(foreground), _relative_luminance(background))
    dark = min(_relative_luminance(foreground), _relative_luminance(background))
    return (light + 0.05) / (dark + 0.05)


def test_system_tab_labels_are_legible_in_every_scheme():
    styles = _read("styles.css")
    inactive = re.findall(r"--track-system-label:\s*(#[0-9a-fA-F]{6});", styles)
    active = re.findall(r"--track-system-active-label:\s*(#[0-9a-fA-F]{6});", styles)
    assert len(inactive) == 2, inactive
    assert len(active) == 2, active

    # Worst-case surfaces per scheme: the light System view is near white (and
    # glass-off is fully opaque white); the dark System view is near black. A
    # light-scheme label is only ever drawn over a light surface and vice versa.
    light_surfaces = [(255, 255, 255), (0xF7, 0xFB, 0xFF)]
    dark_surfaces = [(0x0B, 0x0F, 0x19), (0x1C, 0x1C, 0x22)]
    cases = [
        (_hex_to_rgb(inactive[0]), light_surfaces),
        (_hex_to_rgb(active[0]), light_surfaces),
        (_hex_to_rgb(inactive[1]), dark_surfaces),
        (_hex_to_rgb(active[1]), dark_surfaces),
    ]
    for foreground, surfaces in cases:
        for background in surfaces:
            assert _contrast(foreground, background) >= 4.5, (foreground, background)

    # Glass tier lives on the root element, outside `.app`; the selector must
    # follow that actual DOM ancestry so the System overrides really match.
    assert (
        ":where(.glass-full, .glass-light, .glass-off) .app.system-mode "
        ".sliding-tabs .tab:not(.active)" in styles
    )
    assert (
        ":where(.glass-full, .glass-light, .glass-off) .app.system-mode "
        ".sliding-tabs .tab.active" in styles
    )
    assert "var(--track-system-active-label)" in styles


def test_app_wires_both_segmented_controls_through_one_controller():
    app = _read("app.js")
    assert 'import { createSegmentedControl } from "./segmented-control.js";' in app
    assert "attachSegmentedControls()" in app
    assert "viewTabsControl" in app
    assert "modeSegmentControl" in app
    # Both existing click/keyboard handlers must remain the single switch path.
    assert 'document.getElementById("tab3dMap").addEventListener("click", () => setTab("3d"));' in app
    assert 'document.querySelectorAll(".mode").forEach((button) => button.addEventListener("click", () => setMode(button.dataset.mode)));' in app
    # Programmatic switches reposition the capsule.
    assert "viewTabsControl.sync(" in app
    assert "modeSegmentControl.sync(" in app
