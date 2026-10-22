"""Frontend tests for the dual-stick mapping joystick redesign.

These execute the real ``static/app.js`` blocks under Node with a minimal DOM
stub.  They cover the two independent sticks, simultaneous pointers, the
combined ``/api/teleop`` payload, per-axis release, all-stop safety, stable
viewport-pixel offsets, preview/Done geometry parity, and the icon controls.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest


def _run_app_harness(harness: str) -> subprocess.CompletedProcess:
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", harness],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    return completed


_FRONTEND_PREAMBLE = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
function sliceBlock(startMarker, endMarker) {
  const start = source.indexOf(startMarker);
  const end = source.indexOf(endMarker, start);
  if (start < 0 || end < 0) throw new Error("missing block " + startMarker);
  return source.slice(start, end);
}

function makeElement(id) {
  const classes = new Set();
  return {
    id,
    hidden: false,
    inert: false,
    textContent: "",
    captured: [],
    attributes: {},
    style: {
      left: "0px",
      top: "0px",
      values: {},
      setProperty(name, value) { this.values[name] = value; },
    },
    classList: {
      add(name) { classes.add(name); },
      remove(name) { classes.delete(name); },
      toggle(name, on) {
        if (on === undefined) {
          if (classes.has(name)) classes.delete(name);
          else classes.add(name);
        } else if (on) {
          classes.add(name);
        } else {
          classes.delete(name);
        }
      },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
    addEventListener() {},
    setPointerCapture(pointerId) { this.captured.push(pointerId); },
    getBoundingClientRect() {
      const left = Number.parseFloat(this.style.left) || 0;
      const top = Number.parseFloat(this.style.top) || 0;
      const width = this.offsetWidth || 0;
      const height = this.offsetHeight || 0;
      return { left, top, right: left + width, bottom: top + height, width, height };
    },
  };
}

function expect(condition, message) { if (!condition) throw new Error(message); }
"""


_DUAL_STICK_TELEOP_HARNESS = (
    _FRONTEND_PREAMBLE
    + r"""
const block = sliceBlock("// --- Mapping joystick", "// --- End mapping joystick");

const elements = {
  mappingJoystick: makeElement("mappingJoystick"),
  throttleBase: makeElement("throttleBase"),
  steeringBase: makeElement("steeringBase"),
  throttleKnob: makeElement("throttleKnob"),
  steeringKnob: makeElement("steeringKnob"),
};
const stage = makeElement("stage");
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
for (const base of [elements.throttleBase, elements.steeringBase]) {
  base.offsetWidth = 100;
  base.offsetHeight = 100;
}
const intervalHandlers = new Map();
let nextIntervalId = 1;
global.window = {
  addEventListener() {},
  setInterval(handler) {
    const id = nextIntervalId;
    nextIntervalId += 1;
    intervalHandlers.set(id, handler);
    return id;
  },
  clearInterval(id) { intervalHandlers.delete(id); },
};
const frames = new Map();
global.requestAnimationFrame = (fn) => { const id = frames.size + 1; frames.set(id, fn); return id; };
global.cancelAnimationFrame = (id) => { frames.delete(id); };
global.setTimeout = (fn) => { fn(); return 1; };
global.clearTimeout = () => {};

const beacons = [];
try {
  Object.defineProperty(globalThis, "navigator", {
    value: { sendBeacon(url, body) { beacons.push({ url, body }); return true; } },
    configurable: true,
  });
} catch (error) {
  globalThis.navigator.sendBeacon = (url, body) => {
    beacons.push({ url, body });
    return true;
  };
}

let fakeNow = 0;
Date.now = () => fakeNow;
global.viewerSessionId = () => "tab-session";
global.glassMotionReduced = () => true;
global.applyJoystickLayout = () => {};
global.noteLayoutResize = () => {};

const posts = [];
const deferred = [];
let deferRequests = false;
let failNextRequest = false;
global.postJson = async (url, payload) => {
  posts.push({ url, payload });
  if (failNextRequest) {
    failNextRequest = false;
    throw new Error("network down");
  }
  const active = payload.throttle !== 0 || payload.steering !== 0;
  if (deferRequests) {
    return await new Promise((resolve, reject) => {
      deferred.push({ resolve, reject, payload, active });
    });
  }
  return { ok: true, teleop: { active } };
};
global.fetchWithTimeout = async () => ({ ok: true });

const api = new Function(block + `
return {
  teleop, stickAxisValue, stickKnobOffset, stickPointerDown, stickPointerMove,
  stickPointerUp, stickPointerCancel, handleJoystickBlur, handleVisibilityChange,
  handlePageHide, stopTeleop, sendTeleopCommand,
};`)();
const {
  teleop, stickAxisValue, stickKnobOffset, stickPointerDown, stickPointerMove,
  stickPointerUp, stickPointerCancel, handleJoystickBlur, handleVisibilityChange,
  handlePageHide, stopTeleop, sendTeleopCommand,
} = api;

function baseFor(role) { return role === "throttle" ? elements.throttleBase : elements.steeringBase; }
function makeEvent(role, pointerId, clientX, clientY) {
  return {
    pointerId,
    pointerType: "mouse",
    clientX,
    clientY,
    currentTarget: baseFor(role),
    preventDefault() {},
    stopPropagation() {},
  };
}
function lastPost() { return posts[posts.length - 1].payload; }

(async () => {
  expect(teleop.clientId === "tab-session", "teleop reuses the viewer session id");

  // Axis locking: the throttle stick is vertical only and the steering stick
  // is horizontal only, both with a center dead zone.
  expect(stickAxisValue("throttle", 40, -40, 50) > 0, "throttle up is forward");
  expect(stickAxisValue("throttle", 40, 40, 50) < 0, "throttle down is reverse");
  expect(stickAxisValue("throttle", 40, 0, 50) === 0, "throttle ignores horizontal travel");
  expect(stickAxisValue("steering", 40, -40, 50) < 0, "steering right is negative");
  expect(stickAxisValue("steering", -40, 40, 50) > 0, "steering left is positive");
  expect(stickAxisValue("steering", 0, -40, 50) === 0, "steering ignores vertical travel");
  expect(stickAxisValue("throttle", 2, -2, 50) === 0, "dead zone centers throttle");
  expect(stickAxisValue("steering", -100, 0, 50) === 1, "steering clamps to the unit range");
  const lockedThrottle = stickKnobOffset("throttle", 30, -20);
  expect(lockedThrottle.dx === 0 && lockedThrottle.dy === -20, "throttle knob is locked to x=0");
  const lockedSteering = stickKnobOffset("steering", 30, -20);
  expect(lockedSteering.dx === 30 && lockedSteering.dy === 0, "steering knob is locked to y=0");

  // Simultaneous two-pointer control: press both sticks and send one combined
  // payload with the current throttle and steering values.
  fakeNow = 0;
  await stickPointerDown("throttle", makeEvent("throttle", 1, 50, 10));
  expect(posts.length === 1, "throttle down sends the combined command");
  expect(posts[0].url === "/api/teleop", "combined command uses the existing endpoint");
  expect(posts[0].payload.client_id === teleop.clientId, "command carries the tab client id");
  expect(Number.isInteger(posts[0].payload.seq) && posts[0].payload.seq > 0, "command carries a positive seq");
  expect(posts[0].payload.throttle > 0 && posts[0].payload.steering === 0, "throttle-only payload");
  expect(elements.throttleBase.captured.length === 0, "throttle takes no element pointer capture");
  expect(teleop.pointers.throttle === 1, "throttle owns its pointer id");
  expect(elements.throttleKnob.style.transform === "translate(0px, -40px)", "throttle knob stays on x=0");
  expect(intervalHandlers.size === 1, "one shared keepalive loop runs");

  fakeNow = 100;
  await stickPointerDown("steering", makeEvent("steering", 2, 90, 50));
  expect(posts.length === 2, "second stick pointer sends a fresh combined payload");
  expect(lastPost().throttle > 0 && lastPost().steering < 0, "combined payload carries both axes");
  expect(elements.steeringBase.captured.length === 0, "steering takes no element pointer capture");
  expect(teleop.pointers.steering === 2, "steering owns its pointer id");
  expect(elements.steeringKnob.style.transform === "translate(40px, 0px)", "steering knob stays on y=0");

  const beforeSameStick = posts.length;
  await stickPointerDown("throttle", makeEvent("throttle", 3, 50, 50));
  expect(posts.length === beforeSameStick, "a second pointer on the same stick is ignored");
  expect(teleop.pointers.throttle === 1, "a second same-stick pointer is not owned");

  // Move steering while throttle is held: the combined payload keeps the live
  // throttle and updates steering.
  fakeNow = 200;
  await stickPointerMove("steering", makeEvent("steering", 2, 10, 50));
  expect(posts.length === 3, "steering move sends after the throttle interval");
  expect(lastPost().throttle > 0 && lastPost().steering > 0, "combined update keeps both live values");

  // Hold keepalive streams the combined state.
  fakeNow = 300;
  await Array.from(intervalHandlers.values())[0]();
  expect(posts.length === 4, "keepalive streams while either axis is active");
  expect(lastPost().throttle > 0 && lastPost().steering > 0, "keepalive carries both axes");

  // Releasing the throttle zeros only throttle and immediately sends the still
  // live steering value.
  fakeNow = 400;
  await stickPointerUp("throttle", makeEvent("throttle", 1, 50, 10));
  expect(posts.length === 5, "releasing one stick sends immediately");
  expect(lastPost().throttle === 0, "released axis is zeroed");
  expect(lastPost().steering > 0, "remaining axis is preserved");
  expect(elements.throttleKnob.style.transform === "translate(0px, 0px)", "released stick recenters");
  expect(intervalHandlers.size === 1, "keepalive continues for the remaining axis");

  fakeNow = 500;
  await Array.from(intervalHandlers.values())[0]();
  expect(lastPost().throttle === 0 && lastPost().steering > 0, "remaining axis keeps streaming");

  // Releasing the last stick is an all-stop.
  fakeNow = 600;
  await stickPointerUp("steering", makeEvent("steering", 2, 10, 50));
  expect(lastPost().throttle === 0 && lastPost().steering === 0, "last release sends zero");
  expect(elements.steeringKnob.style.transform === "translate(0px, 0px)", "last release recenters");
  expect(intervalHandlers.size === 0, "all-stop ends the keepalive loop");

  // Cancel path zeroes the cancelled axis and sends the combined state.
  fakeNow = 700;
  await stickPointerDown("steering", makeEvent("steering", 4, 90, 50));
  fakeNow = 750;
  await stickPointerCancel("steering", makeEvent("steering", 4, 90, 50));
  expect(lastPost().throttle === 0 && lastPost().steering === 0, "cancel sends zero when nothing else is held");

  fakeNow = 800;
  await stickPointerDown("throttle", makeEvent("throttle", 5, 50, 10));
  await stickPointerDown("steering", makeEvent("steering", 6, 90, 50));
  fakeNow = 900;
  await stickPointerCancel("steering", makeEvent("steering", 6, 90, 50));
  expect(lastPost().throttle > 0 && lastPost().steering === 0, "cancel zeroes only the cancelled axis");
  fakeNow = 950;
  await stickPointerCancel("throttle", makeEvent("throttle", 5, 50, 10));
  expect(lastPost().throttle === 0 && lastPost().steering === 0, "cancelling the last axis stops all");

  // Blur, hidden document, and page shutdown all stop both axes.
  fakeNow = 1000;
  await stickPointerDown("throttle", makeEvent("throttle", 7, 50, 10));
  fakeNow = 1100;
  await handleJoystickBlur();
  expect(lastPost().throttle === 0 && lastPost().steering === 0, "blur stops both axes");

  fakeNow = 1200;
  await stickPointerDown("steering", makeEvent("steering", 8, 90, 50));
  document.hidden = true;
  fakeNow = 1300;
  await handleVisibilityChange();
  expect(lastPost().throttle === 0 && lastPost().steering === 0, "hidden document stops both axes");
  document.hidden = false;

  fakeNow = 1400;
  await stickPointerDown("throttle", makeEvent("throttle", 9, 50, 10));
  const seqBeforePageHide = teleop.seq;
  handlePageHide();
  expect(beacons.length === 1, "page shutdown sends one keepalive stop");
  const beacon = JSON.parse(beacons[0].body);
  expect(beacon.throttle === 0 && beacon.steering === 0, "keepalive stop zeroes both axes");
  expect(beacon.client_id === teleop.clientId, "keepalive stop carries the client id");
  expect(Number.isInteger(beacon.seq) && beacon.seq > seqBeforePageHide, "keepalive stop increments seq");

  let previousSeq = 0;
  for (const post of posts) {
    expect(post.payload.client_id === teleop.clientId, "every teleop post carries the client id");
    expect(Number.isInteger(post.payload.seq) && post.payload.seq > previousSeq, "teleop seq strictly increases");
    previousSeq = post.payload.seq;
  }

  // A failed send stops the loop, releases both pointers, and keeps the stop
  // obligation so a later zero is still delivered.
  teleop.stopRequired = false;
  failNextRequest = true;
  fakeNow = 1500;
  await stickPointerDown("throttle", makeEvent("throttle", 10, 50, 10));
  expect(intervalHandlers.size === 0, "failed send clears the keepalive loop");
  expect(elements.throttleKnob.style.transform === "translate(0px, 0px)", "failed send recenters the knob");
  expect(teleop.pointers.throttle === null, "failed send releases the pointer");
  expect(teleop.stopRequired === true, "failed send keeps the stop obligation");
  const beforeFailureRelease = posts.length;
  fakeNow = 1600;
  await stickPointerUp("throttle", makeEvent("throttle", 10, 50, 10));
  expect(posts.length === beforeFailureRelease + 1, "release after a failure still sends a stop");
  expect(lastPost().throttle === 0 && lastPost().steering === 0, "stop after a failure is zero");
  expect(teleop.stopRequired === false, "a successful zero clears the obligation");

  // Deferred/out-of-order: the stop must outrank a slow active request.
  deferRequests = true;
  teleop.stopRequired = false;
  fakeNow = 1700;
  const downPromise = stickPointerDown("throttle", makeEvent("throttle", 11, 50, 10));
  const activePost = posts[posts.length - 1].payload;
  expect(teleop.stopRequired === true, "nonzero request arms the stop obligation immediately");
  fakeNow = 1800;
  const upPromise = stickPointerUp("throttle", makeEvent("throttle", 11, 50, 10));
  const stopPost = posts[posts.length - 1].payload;
  expect(stopPost.throttle === 0 && stopPost.steering === 0, "deferred stop is zero");
  expect(stopPost.seq > activePost.seq, "deferred stop seq is higher");
  const stopEntry = deferred.find((entry) => entry.payload === stopPost);
  const activeEntry = deferred.find((entry) => entry.payload === activePost);
  stopEntry.resolve({ ok: true, teleop: { active: false } });
  await upPromise;
  expect(teleop.stopRequired === false, "out-of-order stop success clears the obligation");
  activeEntry.resolve({ ok: true, stale: true });
  await downPromise;
  expect(teleop.stopRequired === false, "stale active response cannot re-arm the stop obligation");

  console.log("dual stick teleop ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""
)


def test_dual_stick_axis_locking_combined_commands_and_stop_safety_under_node():
    completed = _run_app_harness(_DUAL_STICK_TELEOP_HARNESS)
    assert "dual stick teleop ok" in completed.stdout


# Regression: with one stick released and the other still held, an older accepted
# response (the combined stop sent at release) can be delivered after the newer
# active response because the browser may complete concurrent HTTP requests out
# of order.  That late stop must not clear the shared active flag, or the
# still-held stick's 10Hz keepalive dies and the watchdog zeroes its command.
# This harness registers the REAL pointer listeners instead of calling the
# teleop helpers directly.
_CONCURRENT_RELEASE_HARNESS = (
    _FRONTEND_PREAMBLE
    + r"""
const block = sliceBlock("// --- Mapping joystick", "// --- End mapping joystick");

function makeListeningElement(id) {
  const listeners = {};
  const classes = new Set();
  return {
    id,
    hidden: false,
    inert: false,
    textContent: "",
    captured: [],
    attributes: {},
    listeners,
    style: {
      left: "0px",
      top: "0px",
      values: {},
      setProperty(name, value) { this.values[name] = value; },
    },
    classList: {
      add(name) { classes.add(name); },
      remove(name) { classes.delete(name); },
      toggle(name, on) {
        if (on === undefined) {
          if (classes.has(name)) classes.delete(name);
          else classes.add(name);
        } else if (on) {
          classes.add(name);
        } else {
          classes.delete(name);
        }
      },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
    addEventListener(type, handler) { (listeners[type] = listeners[type] || []).push(handler); },
    setPointerCapture(pointerId) { this.captured.push(pointerId); },
    releasePointerCapture() {},
    getBoundingClientRect() {
      const left = Number.parseFloat(this.style.left) || 0;
      const top = Number.parseFloat(this.style.top) || 0;
      const width = this.offsetWidth || 0;
      const height = this.offsetHeight || 0;
      return { left, top, right: left + width, bottom: top + height, width, height };
    },
  };
}

const elements = {
  mappingJoystick: makeListeningElement("mappingJoystick"),
  throttleBase: makeListeningElement("throttleBase"),
  steeringBase: makeListeningElement("steeringBase"),
  throttleKnob: makeListeningElement("throttleKnob"),
  steeringKnob: makeListeningElement("steeringKnob"),
};
const stage = makeListeningElement("stage");
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
for (const base of [elements.throttleBase, elements.steeringBase]) {
  base.offsetWidth = 100;
  base.offsetHeight = 100;
}

const intervalHandlers = new Map();
let nextIntervalId = 1;
global.window = {
  addEventListener() {},
  setInterval(handler) {
    const id = nextIntervalId;
    nextIntervalId += 1;
    intervalHandlers.set(id, handler);
    return id;
  },
  clearInterval(id) { intervalHandlers.delete(id); },
};
global.requestAnimationFrame = () => 1;
global.cancelAnimationFrame = () => {};
global.setTimeout = (fn) => { fn(); return 1; };
global.clearTimeout = () => {};
try {
  Object.defineProperty(globalThis, "navigator", {
    value: { sendBeacon() { return true; } },
    configurable: true,
  });
} catch (error) {
  globalThis.navigator.sendBeacon = () => true;
}
let fakeNow = 0;
Date.now = () => fakeNow;
global.viewerSessionId = () => "tab-session";
global.glassMotionReduced = () => true;
global.applyJoystickLayout = () => {};
global.noteLayoutResize = () => {};

const posts = [];
const deferred = [];
let deferRequests = false;
global.postJson = async (url, payload) => {
  posts.push({ url, payload });
  const active = payload.throttle !== 0 || payload.steering !== 0;
  if (deferRequests) {
    return await new Promise((resolve) => deferred.push({ resolve, payload, active }));
  }
  return { ok: true, teleop: { active } };
};
global.fetchWithTimeout = async () => ({ ok: true });

const api = new Function(block + `
return { teleop, stickPointerDown, stickPointerMove, stickPointerUp, stickPointerCancel };`)();
const { teleop } = api;

// Wire the listeners exactly as attachEvents does for each stick base.
for (const role of ["throttle", "steering"]) {
  const base = role === "throttle" ? elements.throttleBase : elements.steeringBase;
  base.addEventListener("pointerdown", (event) => api.stickPointerDown(role, event));
  base.addEventListener("pointermove", (event) => api.stickPointerMove(role, event));
  base.addEventListener("pointerup", (event) => api.stickPointerUp(role, event));
  base.addEventListener("pointercancel", (event) => api.stickPointerCancel(role, event));
  base.addEventListener("lostpointercapture", (event) => api.stickPointerCancel(role, event));
}

function baseFor(role) { return role === "throttle" ? elements.throttleBase : elements.steeringBase; }
function makeEvent(role, pointerId, clientX, clientY) {
  return {
    pointerId,
    pointerType: "touch",
    clientX,
    clientY,
    currentTarget: baseFor(role),
    preventDefault() {},
    stopPropagation() {},
  };
}
function fire(role, type, pointerId, clientX, clientY) {
  const base = baseFor(role);
  return Promise.all(
    (base.listeners[type] || []).map((handler) =>
      handler(makeEvent(role, pointerId, clientX, clientY)),
    ),
  );
}
function drain() { return new Promise((resolve) => setImmediate(resolve)); }

(async () => {
  // Two fingers: throttle forward, steering displaced but still inside its dead
  // zone, so both pointers are captured while the steering axis commands zero.
  deferRequests = false;
  await fire("throttle", "pointerdown", 1, 50, 0);
  await fire("steering", "pointerdown", 2, 52, 50);
  expect(teleop.pointers.throttle === 1 && teleop.pointers.steering === 2, "both pointers captured");
  expect(teleop.throttle > 0, "throttle commands forward");
  expect(teleop.steering === 0, "steering is inside its dead zone");
  const steeringKnobBefore = elements.steeringKnob.style.transform;
  expect(steeringKnobBefore === "translate(2px, 0px)", "steering knob holds its sub-dead-zone offset");

  // Release the throttle: the combined command is a stop because the remaining
  // steering axis is zero.  The still-held steering pointer then moves, sending
  // a newer active command.  Both requests are in flight concurrently.
  deferRequests = true;
  fakeNow = 100;
  void fire("throttle", "pointerup", 1, 50, 0);
  await drain();
  void fire("throttle", "lostpointercapture", 1, 50, 0);
  await drain();
  const stopEntry = deferred[deferred.length - 1];
  expect(stopEntry.payload.throttle === 0 && stopEntry.payload.steering === 0, "release sends a combined stop");
  expect(teleop.pointers.steering === 2, "the steering pointer stays captured after the release");
  expect(elements.steeringKnob.style.transform === steeringKnobBefore, "release leaves the steering knob untouched");

  // A further duplicate terminal event for the released pointer is a no-op and
  // must not reach the global reset while the other pointer is still down.
  fakeNow = 150;
  void fire("throttle", "pointercancel", 1, 50, 0);
  await drain();
  expect(teleop.pointers.steering === 2, "a duplicate terminal event keeps the held pointer down");
  expect(teleop.pointers.throttle === null, "a duplicate terminal event cannot re-capture the released pointer");
  expect(elements.steeringKnob.style.transform === steeringKnobBefore, "a duplicate terminal event cannot recenter the held knob");

  fakeNow = 200;
  void fire("steering", "pointermove", 2, 10, 50);
  await drain();
  const activeEntry = deferred[deferred.length - 1];
  expect(activeEntry.payload.steering > 0, "the held stick sends a newer active command");
  expect(activeEntry.payload.seq > stopEntry.payload.seq, "the active command is newer");

  // The server accepted both in send order; the HTTP responses arrive newest
  // first and the older stop last.  The late stop must not clear active.
  activeEntry.resolve({ ok: true, teleop: { active: true } });
  await drain();
  stopEntry.resolve({ ok: true, teleop: { active: false } });
  await drain();

  expect(teleop.steering > 0, "the held stick keeps its live value");
  expect(elements.steeringKnob.style.transform === "translate(-40px, 0px)", "the held knob keeps its position");
  expect(teleop.pointers.steering === 2, "the held pointer stays down");
  expect(teleop.active === true, "a late stop response cannot clear the shared active state");

  // The remaining stick must keep its 10Hz combined command loop.
  deferRequests = false;
  const postsBeforeKeepalive = posts.length;
  fakeNow = 300;
  await Array.from(intervalHandlers.values())[0]();
  expect(posts.length === postsBeforeKeepalive + 1, "the keepalive still streams for the held stick");
  const keepalive = posts[posts.length - 1].payload;
  expect(keepalive.throttle === 0 && keepalive.steering > 0, "the keepalive carries the combined state");

  console.log("concurrent release keepalive ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""
)


def test_release_keeps_the_other_stick_keepalive_alive_under_out_of_order_responses():
    completed = _run_app_harness(_CONCURRENT_RELEASE_HARNESS)
    assert "concurrent release keepalive ok" in completed.stdout


# Regression: a superseded in-flight failure must not tear down the newer
# request's live state.  With two real captured pointers, an older active
# request can reject after the newer combined active request has already
# succeeded; that old failure must not center both knobs or kill the 10Hz loop
# that keeps the held sticks alive.  Only the latest seq owns the global stop.
_STICK_FAILURE_HARNESS = (
    _FRONTEND_PREAMBLE
    + r"""
const block = sliceBlock("// --- Mapping joystick", "// --- End mapping joystick");

function makeListeningElement(id) {
  const listeners = {};
  const classes = new Set();
  return {
    id,
    hidden: false,
    inert: false,
    textContent: "",
    captured: [],
    attributes: {},
    listeners,
    style: {
      left: "0px",
      top: "0px",
      values: {},
      setProperty(name, value) { this.values[name] = value; },
    },
    classList: {
      add(name) { classes.add(name); },
      remove(name) { classes.delete(name); },
      toggle(name, on) {
        if (on === undefined) {
          if (classes.has(name)) classes.delete(name);
          else classes.add(name);
        } else if (on) {
          classes.add(name);
        } else {
          classes.delete(name);
        }
      },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
    addEventListener(type, handler) { (listeners[type] = listeners[type] || []).push(handler); },
    setPointerCapture(pointerId) { this.captured.push(pointerId); },
    releasePointerCapture() {},
    getBoundingClientRect() {
      const left = Number.parseFloat(this.style.left) || 0;
      const top = Number.parseFloat(this.style.top) || 0;
      const width = this.offsetWidth || 0;
      const height = this.offsetHeight || 0;
      return { left, top, right: left + width, bottom: top + height, width, height };
    },
  };
}

const elements = {
  mappingJoystick: makeListeningElement("mappingJoystick"),
  throttleBase: makeListeningElement("throttleBase"),
  steeringBase: makeListeningElement("steeringBase"),
  throttleKnob: makeListeningElement("throttleKnob"),
  steeringKnob: makeListeningElement("steeringKnob"),
};
const stage = makeListeningElement("stage");
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
for (const base of [elements.throttleBase, elements.steeringBase]) {
  base.offsetWidth = 100;
  base.offsetHeight = 100;
}

const intervalHandlers = new Map();
let nextIntervalId = 1;
global.window = {
  addEventListener() {},
  setInterval(handler) {
    const id = nextIntervalId;
    nextIntervalId += 1;
    intervalHandlers.set(id, handler);
    return id;
  },
  clearInterval(id) { intervalHandlers.delete(id); },
};
global.requestAnimationFrame = () => 1;
global.cancelAnimationFrame = () => {};
global.setTimeout = (fn) => { fn(); return 1; };
global.clearTimeout = () => {};
try {
  Object.defineProperty(globalThis, "navigator", {
    value: { sendBeacon() { return true; } },
    configurable: true,
  });
} catch (error) {
  globalThis.navigator.sendBeacon = () => true;
}
let fakeNow = 0;
Date.now = () => fakeNow;
global.viewerSessionId = () => "tab-session";
global.glassMotionReduced = () => true;
global.applyJoystickLayout = () => {};
global.noteLayoutResize = () => {};
global.joystickLayout = { saveError: null };
let renderErrorCalls = 0;
global.renderJoystickSettingsError = () => { renderErrorCalls += 1; };

const posts = [];
const deferred = [];
let deferRequests = false;
global.postJson = async (url, payload) => {
  posts.push({ url, payload });
  const active = payload.throttle !== 0 || payload.steering !== 0;
  if (deferRequests) {
    return await new Promise((resolve, reject) => deferred.push({ resolve, reject, payload, active }));
  }
  return { ok: true, teleop: { active } };
};
global.fetchWithTimeout = async () => ({ ok: true });

const api = new Function(block + `
return {
  teleop, stickPointerDown, stickPointerMove, stickPointerUp,
  stickPointerCancel, sendTeleopCommand,
};`)();
const { teleop, sendTeleopCommand } = api;

// Wire the listeners exactly as attachEvents does for each stick base.
for (const role of ["throttle", "steering"]) {
  const base = role === "throttle" ? elements.throttleBase : elements.steeringBase;
  base.addEventListener("pointerdown", (event) => api.stickPointerDown(role, event));
  base.addEventListener("pointermove", (event) => api.stickPointerMove(role, event));
  base.addEventListener("pointerup", (event) => api.stickPointerUp(role, event));
  base.addEventListener("pointercancel", (event) => api.stickPointerCancel(role, event));
}

function baseFor(role) { return role === "throttle" ? elements.throttleBase : elements.steeringBase; }
function makeEvent(role, pointerId, clientX, clientY) {
  return {
    pointerId,
    pointerType: "touch",
    clientX,
    clientY,
    currentTarget: baseFor(role),
    preventDefault() {},
    stopPropagation() {},
  };
}
function fire(role, type, pointerId, clientX, clientY) {
  const base = baseFor(role);
  return Promise.all(
    (base.listeners[type] || []).map((handler) =>
      handler(makeEvent(role, pointerId, clientX, clientY)),
    ),
  );
}
function drain() { return new Promise((resolve) => setImmediate(resolve)); }

(async () => {
"""
)


def _stick_failure_scenario(source: str) -> str:
    return (
        _STICK_FAILURE_HARNESS
        + source
        + "\n})().catch((error) => { console.error(error); process.exit(1); });\n"
    )


def test_superseded_stick_failure_keeps_both_sticks_alive_under_node():
    completed = _run_app_harness(
        _stick_failure_scenario(
            r"""
// Both role orders: whichever stick sends the older active request must not be
// reset when its superseded request fails after the newer combined command has
// already succeeded.
async function runSupersededScenario(olderRole, newerRole, olderPoint, newerPoint) {
  deferRequests = true;
  fakeNow = 100;
  void fire(olderRole, "pointerdown", olderPoint.id, olderPoint.x, olderPoint.y);
  await drain();
  const olderEntry = deferred[deferred.length - 1];
  fakeNow = 200;
  void fire(newerRole, "pointerdown", newerPoint.id, newerPoint.x, newerPoint.y);
  await drain();
  const newerEntry = deferred[deferred.length - 1];
  expect(olderEntry && newerEntry, "both stick requests are in flight");
  expect(olderEntry.payload.seq < newerEntry.payload.seq, "the newer combined command has the higher seq");

  // The newer combined active request succeeds while the older is in flight.
  newerEntry.resolve({ ok: true, teleop: { active: true } });
  await drain();
  const lastSentAtAfterNewer = teleop.lastSentAt;
  const olderKnobBefore = elements[olderRole + "Knob"].style.transform;
  const newerKnobBefore = elements[newerRole + "Knob"].style.transform;
  expect(olderKnobBefore !== "translate(0px, 0px)", "the older knob holds its live offset");
  expect(newerKnobBefore !== "translate(0px, 0px)", "the newer knob holds its live offset");

  // Now the superseded older request fails.  It must change nothing globally.
  olderEntry.reject(new Error("network down"));
  await drain();

  expect(teleop.pointers[olderRole] === olderPoint.id, "the older pointer stays captured");
  expect(teleop.pointers[newerRole] === newerPoint.id, "the newer pointer stays captured");
  expect(teleop[olderRole] !== 0, "the older stick keeps its live value");
  expect(teleop[newerRole] !== 0, "the newer stick keeps its live value");
  expect(elements[olderRole + "Knob"].style.transform === olderKnobBefore, "a superseded failure cannot recenter the older knob");
  expect(elements[newerRole + "Knob"].style.transform === newerKnobBefore, "a superseded failure cannot recenter the newer knob");
  expect(teleop.active === true, "a superseded failure cannot clear active");
  expect(intervalHandlers.size === 1, "a superseded failure cannot stop the keepalive loop");
  expect(teleop.lastSentAt === lastSentAtAfterNewer, "a superseded failure cannot reset lastSentAt");
  expect(joystickLayout.saveError === null, "a superseded failure is not surfaced as a stop");
  expect(renderErrorCalls === 0, "a superseded failure does not render an error");

  // The two held sticks keep their shared 10Hz keepalive.
  deferRequests = false;
  const postsBeforeKeepalive = posts.length;
  fakeNow = 300;
  await Array.from(intervalHandlers.values())[0]();
  expect(posts.length === postsBeforeKeepalive + 1, "the keepalive still streams after a superseded failure");
  const keepalive = posts[posts.length - 1].payload;
  expect(keepalive.throttle !== 0 && keepalive.steering !== 0, "the keepalive carries both live axes");

  // Cleanup so the next role order starts from a stopped state.
  await fire(olderRole, "pointerup", olderPoint.id, olderPoint.x, olderPoint.y);
  await fire(newerRole, "pointerup", newerPoint.id, newerPoint.x, newerPoint.y);
  expect(intervalHandlers.size === 0 && teleop.active === false, "cleanup stops both sticks");
  posts.length = 0;
  deferred.length = 0;
  joystickLayout.saveError = null;
  renderErrorCalls = 0;
}

await runSupersededScenario(
  "throttle", "steering",
  { id: 11, x: 50, y: 10 }, { id: 12, x: 90, y: 50 },
);
await runSupersededScenario(
  "steering", "throttle",
  { id: 21, x: 90, y: 50 }, { id: 22, x: 50, y: 10 },
);

// A superseded failure resolves like a stale accepted response rather than an
// authoritative global stop, so its caller can safely ignore it.
deferRequests = true;
const olderResult = sendTeleopCommand(0.5, 0, true);
const olderFailure = deferred[deferred.length - 1];
const newerResult = sendTeleopCommand(0, 0, true);
const newerSuccess = deferred[deferred.length - 1];
newerSuccess.resolve({ ok: true, teleop: { active: false } });
await drain();
olderFailure.reject(new Error("network down"));
await drain();
expect(await newerResult === true, "the latest accepted command resolves true");
expect(await olderResult === true, "a superseded failure resolves like a stale response");
expect(joystickLayout.saveError === null, "a superseded direct failure is not surfaced");
expect(renderErrorCalls === 0, "a superseded direct failure does not render an error");

console.log("superseded stick failure ok");
"""
        )
    )
    assert "superseded stick failure ok" in completed.stdout


def test_latest_stick_failure_still_stops_and_resets_both_axes_under_node():
    completed = _run_app_harness(
        _stick_failure_scenario(
            r"""
// Safety proof: when the LATEST request fails, the global safety stop must run
// exactly as before: both axes recenter, both pointers release, the loop stops,
// and the stop error surfaces.
deferRequests = true;
fakeNow = 0;
void fire("throttle", "pointerdown", 31, 50, 10);
await drain();
void fire("steering", "pointerdown", 32, 90, 50);
await drain();
const latest = deferred[deferred.length - 1];
expect(latest.payload.throttle > 0 && latest.payload.steering < 0, "the latest combined request is active");

latest.reject(new Error("network down"));
await drain();

expect(teleop.pointers.throttle === null && teleop.pointers.steering === null, "latest failure releases both pointers");
expect(teleop.throttle === 0 && teleop.steering === 0, "latest failure centers both axes");
expect(elements.throttleKnob.style.transform === "translate(0px, 0px)", "latest failure centers the throttle knob");
expect(elements.steeringKnob.style.transform === "translate(0px, 0px)", "latest failure centers the steering knob");
expect(teleop.active === false, "latest failure clears active");
expect(intervalHandlers.size === 0, "latest failure stops the send loop");
expect(teleop.lastSentAt === -Infinity, "latest failure resets the send throttle");
expect(typeof joystickLayout.saveError === "string" && joystickLayout.saveError.includes("drive stopped"), "latest failure surfaces the stop error");
expect(renderErrorCalls === 1, "latest failure renders the error surface");

console.log("latest stick failure ok");
"""
        )
    )
    assert "latest stick failure ok" in completed.stdout


# Regression: hold the throttle forward, then add a steering contact and release
# only the steering.  A real browser can change the pointer-capture topology
# while the throttle pointer is still down and emit a terminal capture event for
# that earlier contact.  When each base registered ``lostpointercapture`` as a
# per-role cancel, that spurious terminal event recentered the still-held
# throttle knob.  This harness models the browser event flow against the app's
# real routing: pointerdown on each base, with move/up/cancel observed at the
# window and dispatched by pointerId ownership.  While the routing helper is
# absent it falls back to the pre-fix per-base wiring so the exact bug still
# reproduces on the old code.
_POINTER_OWNERSHIP_HARNESS = (
    _FRONTEND_PREAMBLE
    + r"""
const block = sliceBlock("// --- Mapping joystick", "// --- End mapping joystick");

function makeListeningElement(id) {
  const listeners = {};
  const classes = new Set();
  return {
    id,
    hidden: false,
    inert: false,
    textContent: "",
    captured: [],
    attributes: {},
    listeners,
    style: {
      left: "0px",
      top: "0px",
      values: {},
      setProperty(name, value) { this.values[name] = value; },
    },
    classList: {
      add(name) { classes.add(name); },
      remove(name) { classes.delete(name); },
      toggle(name, on) {
        if (on === undefined) {
          if (classes.has(name)) classes.delete(name);
          else classes.add(name);
        } else if (on) {
          classes.add(name);
        } else {
          classes.delete(name);
        }
      },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
    addEventListener(type, handler) { (listeners[type] = listeners[type] || []).push(handler); },
    setPointerCapture(pointerId) { this.captured.push(pointerId); },
    releasePointerCapture() {},
    getBoundingClientRect() {
      const left = Number.parseFloat(this.style.left) || 0;
      const top = Number.parseFloat(this.style.top) || 0;
      const width = this.offsetWidth || 0;
      const height = this.offsetHeight || 0;
      return { left, top, right: left + width, bottom: top + height, width, height };
    },
  };
}

const elements = {
  mappingJoystick: makeListeningElement("mappingJoystick"),
  throttleBase: makeListeningElement("throttleBase"),
  steeringBase: makeListeningElement("steeringBase"),
  throttleKnob: makeListeningElement("throttleKnob"),
  steeringKnob: makeListeningElement("steeringKnob"),
};
const stage = makeListeningElement("stage");
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
for (const base of [elements.throttleBase, elements.steeringBase]) {
  base.offsetWidth = 100;
  base.offsetHeight = 100;
}

const intervalHandlers = new Map();
let nextIntervalId = 1;
const windowListeners = {};
global.window = {
  addEventListener(type, handler) { (windowListeners[type] = windowListeners[type] || []).push(handler); },
  setInterval(handler) {
    const id = nextIntervalId;
    nextIntervalId += 1;
    intervalHandlers.set(id, handler);
    return id;
  },
  clearInterval(id) { intervalHandlers.delete(id); },
};
global.requestAnimationFrame = () => 1;
global.cancelAnimationFrame = () => {};
global.setTimeout = (fn) => { fn(); return 1; };
global.clearTimeout = () => {};
try {
  Object.defineProperty(globalThis, "navigator", {
    value: { sendBeacon() { return true; } },
    configurable: true,
  });
} catch (error) {
  globalThis.navigator.sendBeacon = () => true;
}
let fakeNow = 0;
Date.now = () => fakeNow;
global.viewerSessionId = () => "tab-session";
global.glassMotionReduced = () => true;
global.applyJoystickLayout = () => {};
global.noteLayoutResize = () => {};

const posts = [];
global.postJson = async (url, payload) => {
  posts.push({ url, payload });
  return { ok: true, teleop: { active: payload.throttle !== 0 || payload.steering !== 0 } };
};
global.fetchWithTimeout = async () => ({ ok: true });

const api = new Function(block + `
return {
  teleop, stickPointerDown, stickPointerMove, stickPointerUp, stickPointerCancel,
  attachStickPointerRouting: typeof attachStickPointerRouting === "function"
    ? attachStickPointerRouting
    : null,
};`)();
const { teleop } = api;

// Install the app's real pointer routing when it exists.  Otherwise install the
// pre-fix wiring so the harness reproduces the live throttle recenter.
if (typeof api.attachStickPointerRouting === "function") {
  api.attachStickPointerRouting();
} else {
  for (const role of ["throttle", "steering"]) {
    const base = role === "throttle" ? elements.throttleBase : elements.steeringBase;
    base.addEventListener("pointerdown", (event) => api.stickPointerDown(role, event));
    base.addEventListener("pointermove", (event) => api.stickPointerMove(role, event));
    base.addEventListener("pointerup", (event) => api.stickPointerUp(role, event));
    base.addEventListener("pointercancel", (event) => api.stickPointerCancel(role, event));
    base.addEventListener("lostpointercapture", (event) => api.stickPointerCancel(role, event));
  }
}

function baseFor(role) { return role === "throttle" ? elements.throttleBase : elements.steeringBase; }
function makeEvent(role, type, pointerId, clientX, clientY) {
  return {
    type,
    pointerId,
    pointerType: "touch",
    clientX,
    clientY,
    currentTarget: baseFor(role),
    preventDefault() {},
    stopPropagation() {},
  };
}
// Model a browser: a captured pointer event is targeted at the owning base and
// then bubbles to the window.  lostpointercapture is dispatched at the base
// that lost capture.
function fire(role, type, pointerId, clientX, clientY) {
  const event = makeEvent(role, type, pointerId, clientX, clientY);
  const handlers = (baseFor(role).listeners[type] || []).concat(windowListeners[type] || []);
  return Promise.all(handlers.map((handler) => handler(event)));
}
function drain() { return new Promise((resolve) => setImmediate(resolve)); }

(async () => {
  // Exact reported sequence: throttle pointer 11 down/move, steering pointer 22
  // down/move, then release only the later steering contact.  The browser also
  // emits a spurious lostpointercapture for the still-held throttle contact.
  await fire("throttle", "pointerdown", 11, 50, 0);
  await fire("throttle", "pointermove", 11, 50, 0);
  expect(teleop.throttle > 0, "throttle commands forward");
  await fire("steering", "pointerdown", 22, 90, 50);
  await fire("steering", "pointermove", 22, 90, 50);
  expect(teleop.pointers.throttle === 11, "the throttle pointer is owned");
  expect(teleop.pointers.steering === 22, "the steering pointer is owned");

  await fire("steering", "pointerup", 22, 90, 50);
  await fire("steering", "lostpointercapture", 22, 90, 50);
  await fire("throttle", "lostpointercapture", 11, 50, 0);
  await drain();

  expect(teleop.pointers.throttle === 11, "a steering terminal cannot release the throttle pointer");
  expect(teleop.pointers.steering === null, "the released steering pointer is cleared");
  expect(teleop.throttle > 0, "releasing steering cannot zero the throttle axis");
  expect(teleop.steering === 0, "releasing steering zeroes only steering");
  expect(
    elements.throttleKnob.style.transform === "translate(0px, -50px)",
    "the still-held throttle knob keeps its forward position",
  );
  expect(
    elements.steeringKnob.style.transform === "translate(0px, 0px)",
    "the released steering knob recenters",
  );

  // Duplicate terminal events for the already-released steering pointer are
  // ignored and must not touch the still-held throttle.
  const throttleBefore = elements.throttleKnob.style.transform;
  const steeringBefore = elements.steeringKnob.style.transform;
  await fire("steering", "lostpointercapture", 22, 90, 50);
  await fire("steering", "pointercancel", 22, 90, 50);
  await fire("steering", "pointerup", 22, 90, 50);
  await drain();
  expect(teleop.pointers.throttle === 11, "duplicate steering terminals cannot release the throttle pointer");
  expect(teleop.throttle > 0, "duplicate steering terminals cannot zero the throttle axis");
  expect(
    elements.throttleKnob.style.transform === throttleBefore,
    "duplicate steering terminals cannot recenter the held throttle knob",
  );
  expect(
    elements.steeringKnob.style.transform === steeringBefore,
    "duplicate steering terminals cannot move the released steering knob",
  );

  // A genuine cancellation of the still-held throttle pointer must still stop it
  // for safety; only spurious capture-topology events are discarded.
  await fire("throttle", "pointercancel", 11, 50, 0);
  await drain();
  expect(
    teleop.pointers.throttle === null && teleop.pointers.steering === null,
    "a real throttle cancel releases the pointer",
  );
  expect(teleop.throttle === 0 && teleop.steering === 0, "a real throttle cancel zeros the axes");
  expect(
    elements.throttleKnob.style.transform === "translate(0px, 0px)",
    "a real throttle cancel recenters the throttle knob",
  );
  expect(intervalHandlers.size === 0, "the final terminal event stops the send loop");
  expect(posts[posts.length - 1].payload.throttle === 0, "the final terminal event sends a stop");

  console.log("pointer ownership asymmetric reset ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""
)


def test_releasing_later_steering_never_recenters_held_throttle_under_node():
    completed = _run_app_harness(_POINTER_OWNERSHIP_HARNESS)
    assert "pointer ownership asymmetric reset ok" in completed.stdout


# Minimal teleop-only harness: the real Mapping joystick block with 100x100
# bases so pointer coordinates map to an exact 50px radius.  Each scenario
# appends its own ``(async () => { ... })`` body.
_TELEOP_ONLY_HARNESS = (
    _FRONTEND_PREAMBLE
    + r"""
const block = sliceBlock("// --- Mapping joystick", "// --- End mapping joystick");

const elements = {
  mappingJoystick: makeElement("mappingJoystick"),
  throttleBase: makeElement("throttleBase"),
  steeringBase: makeElement("steeringBase"),
  throttleKnob: makeElement("throttleKnob"),
  steeringKnob: makeElement("steeringKnob"),
};
const stage = makeElement("stage");
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
for (const base of [elements.throttleBase, elements.steeringBase]) {
  base.offsetWidth = 100;
  base.offsetHeight = 100;
}
const intervalHandlers = new Map();
let nextIntervalId = 1;
global.window = {
  addEventListener() {},
  setInterval(handler) {
    const id = nextIntervalId;
    nextIntervalId += 1;
    intervalHandlers.set(id, handler);
    return id;
  },
  clearInterval(id) { intervalHandlers.delete(id); },
};
global.requestAnimationFrame = () => 1;
global.cancelAnimationFrame = () => {};
global.setTimeout = (fn) => { fn(); return 1; };
global.clearTimeout = () => {};
try {
  Object.defineProperty(globalThis, "navigator", {
    value: { sendBeacon() { return true; } },
    configurable: true,
  });
} catch (error) {
  globalThis.navigator.sendBeacon = () => true;
}
let fakeNow = 0;
Date.now = () => fakeNow;
global.viewerSessionId = () => "tab-session";
global.glassMotionReduced = () => true;
global.applyJoystickLayout = () => {};
global.noteLayoutResize = () => {};

const posts = [];
global.postJson = async (url, payload) => {
  posts.push({ url, payload });
  return { ok: true, teleop: { active: payload.throttle !== 0 || payload.steering !== 0 } };
};
global.fetchWithTimeout = async () => ({ ok: true });

const api = new Function(block + `
return {
  teleop, stickAxisValue, stickKnobOffset, stickPointerDown, stickPointerMove,
  stickPointerUp, stickPointerCancel, setMappingJoystickVisible,
};`)();
const {
  teleop, stickAxisValue, stickKnobOffset, stickPointerDown, stickPointerMove,
  stickPointerUp, stickPointerCancel, setMappingJoystickVisible,
} = api;

function baseFor(role) { return role === "throttle" ? elements.throttleBase : elements.steeringBase; }
function makeEvent(role, pointerId, clientX, clientY) {
  return {
    pointerId,
    pointerType: "mouse",
    clientX,
    clientY,
    currentTarget: baseFor(role),
    preventDefault() {},
    stopPropagation() {},
  };
}
function lastPost() { return posts[posts.length - 1].payload; }

(async () => {
"""
)


def _teleop_scenario(source: str) -> str:
    return (
        _TELEOP_ONLY_HARNESS
        + source
        + "\n})().catch((error) => { console.error(error); process.exit(1); });\n"
    )


def test_stick_axis_projection_uses_only_its_own_displacement_under_node():
    completed = _run_app_harness(
        _teleop_scenario(
            r"""
// A far diagonal pointer on the throttle base: the raw vertical displacement
// is 150px against a 50px radius, and the horizontal displacement is 50px.
// The throttle axis is vertical only, so it must reach full scale and the
// knob must stay locked to x=0 instead of losing magnitude to a radial clamp.
fakeNow = 0;
await stickPointerDown("throttle", makeEvent("throttle", 1, 100, -100));
expect(lastPost().throttle === 1, "far diagonal throttle reaches full scale");
expect(lastPost().steering === 0, "throttle never bleeds into steering");
expect(
  elements.throttleKnob.style.transform === "translate(0px, -50px)",
  "throttle knob is locked to x=0 at full travel",
);

// Perpendicular-only travel must not change the throttle magnitude.
fakeNow = 100;
await stickPointerMove("throttle", makeEvent("throttle", 1, 100000, -100));
expect(lastPost().throttle === 1, "perpendicular travel leaves throttle unchanged");
expect(
  elements.throttleKnob.style.transform === "translate(0px, -50px)",
  "throttle knob ignores horizontal travel",
);
fakeNow = 200;
await stickPointerUp("throttle", makeEvent("throttle", 1, 100000, -100));

// Mirror image for the horizontal steering stick.
fakeNow = 300;
await stickPointerDown("steering", makeEvent("steering", 2, 100, -100));
expect(lastPost().steering === -1, "far diagonal steering reaches full scale");
expect(lastPost().throttle === 0, "steering never bleeds into throttle");
expect(
  elements.steeringKnob.style.transform === "translate(50px, 0px)",
  "steering knob is locked to y=0 at full travel",
);
fakeNow = 400;
await stickPointerMove("steering", makeEvent("steering", 2, 100, 100000));
expect(lastPost().steering === -1, "perpendicular travel leaves steering unchanged");
expect(
  elements.steeringKnob.style.transform === "translate(50px, 0px)",
  "steering knob ignores vertical travel",
);

console.log("axis projection ok");
"""
        )
    )
    assert "axis projection ok" in completed.stdout


def test_centered_remaining_pointer_keeps_keepalive_alive_under_node():
    completed = _run_app_harness(
        _teleop_scenario(
            r"""
// Hold a full-forward throttle and a centered steering pointer.  Both
// pointers are down, but only the throttle axis is nonzero.
fakeNow = 0;
await stickPointerDown("throttle", makeEvent("throttle", 1, 50, 0));
expect(lastPost().throttle > 0, "throttle pointer drives forward");
fakeNow = 100;
await stickPointerDown("steering", makeEvent("steering", 2, 50, 50));
expect(teleop.pointers.steering === 2, "centered steering pointer is tracked");
expect(teleop.steering === 0, "centered steering pointer commands zero");
expect(intervalHandlers.size === 1, "one shared loop serves both pointers");

// Release the throttle while the centered steering pointer stays down: the
// immediate combined command is sent and the loop must survive on the pointer
// alone, even though every axis is currently zero.
fakeNow = 200;
await stickPointerUp("throttle", makeEvent("throttle", 1, 50, 0));
expect(lastPost().throttle === 0, "release zeros the released axis");
expect(lastPost().steering === 0, "release sends the combined remaining command");
expect(teleop.pointers.steering === 2, "the remaining pointer stays down");
expect(intervalHandlers.size === 1, "the send loop survives a zero-valued remaining pointer");

// Moving the still-held pointer to a nonzero axis must resume the normal
// 10 Hz keepalive without another pointerdown.
fakeNow = 300;
await stickPointerMove("steering", makeEvent("steering", 2, 90, 50));
expect(lastPost().steering < 0, "moving the remaining pointer sends immediately");
const postsBeforeKeepalive = posts.length;
fakeNow = 400;
await Array.from(intervalHandlers.values())[0]();
expect(posts.length === postsBeforeKeepalive + 1, "keepalive resumes without another pointerdown");
expect(lastPost().steering < 0, "keepalive carries the live steering value");

// Releasing the last pointer still runs the terminal all-stop.
fakeNow = 500;
await stickPointerUp("steering", makeEvent("steering", 2, 90, 50));
expect(lastPost().throttle === 0 && lastPost().steering === 0, "last release sends the terminal zero");
expect(intervalHandlers.size === 0, "last release stops the send loop");

// Cancel obeys the same rule: a centered remaining pointer keeps the loop.
fakeNow = 600;
await stickPointerDown("throttle", makeEvent("throttle", 3, 50, 0));
fakeNow = 700;
await stickPointerDown("steering", makeEvent("steering", 4, 50, 50));
fakeNow = 800;
await stickPointerCancel("steering", makeEvent("steering", 4, 50, 50));
expect(teleop.pointers.throttle === 3, "cancel leaves the other pointer down");
expect(lastPost().throttle > 0 && lastPost().steering === 0, "cancel sends the combined remaining command");
expect(intervalHandlers.size === 1, "cancel keeps the loop for the centered remaining pointer");
fakeNow = 900;
await stickPointerCancel("throttle", makeEvent("throttle", 3, 50, 0));
expect(lastPost().throttle === 0 && lastPost().steering === 0, "cancelling the last pointer sends the terminal zero");
expect(intervalHandlers.size === 0, "cancelling the last pointer stops the loop");

console.log("pointer lifecycle ok");
"""
        )
    )
    assert "pointer lifecycle ok" in completed.stdout


def test_centered_captured_pointer_leaving_mapping_stops_under_node():
    completed = _run_app_harness(
        _teleop_scenario(
            r"""
// A pointer held at the exact center is captured but commands zero, so the
// immediate post is a stop and teleop.active/stopRequired both stay false while
// pointers[role] is non-null.
fakeNow = 0;
await stickPointerDown("steering", makeEvent("steering", 5, 50, 50));
expect(teleop.pointers.steering === 5, "the centered steering pointer is captured");
expect(teleop.steering === 0, "the centered pointer commands zero");
expect(
  teleop.active === false && teleop.stopRequired === false,
  "a centered capture leaves active and stopRequired false",
);
expect(intervalHandlers.size === 1, "the shared send loop is installed for the pointer");

// Leaving Mapping must tear down the stale capture even though no axis is live.
const postsBeforeLeave = posts.length;
await setMappingJoystickVisible(false);
expect(
  teleop.pointers.throttle === null && teleop.pointers.steering === null,
  "leaving Mapping releases every captured pointer",
);
expect(teleop.throttle === 0 && teleop.steering === 0, "leaving Mapping zeros every axis");
expect(intervalHandlers.size === 0, "leaving Mapping stops the send loop");
expect(posts.length === postsBeforeLeave + 1, "leaving Mapping sends exactly one terminal command");
expect(
  lastPost().throttle === 0 && lastPost().steering === 0,
  "the terminal command is a zero stop",
);

console.log("centered pointer leaves mapping ok");
"""
        )
    )
    assert "centered pointer leaves mapping ok" in completed.stdout


_TWO_STICK_LAYOUT_HARNESS = (
    _FRONTEND_PREAMBLE
    + r"""
const teleopBlock = sliceBlock("// --- Mapping joystick", "// --- End mapping joystick");
const layoutBlock = sliceBlock("// --- Joystick layout", "// --- End joystick layout");
const block = teleopBlock + "\n" + layoutBlock;

const elements = {
  view3d: makeElement("view3d"),
  mappingJoystick: makeElement("mappingJoystick"),
  throttleStick: makeElement("throttleStick"),
  steeringStick: makeElement("steeringStick"),
  throttleBase: makeElement("throttleBase"),
  steeringBase: makeElement("steeringBase"),
  throttleKnob: makeElement("throttleKnob"),
  steeringKnob: makeElement("steeringKnob"),
  throttleMoveHandle: makeElement("throttleMoveHandle"),
  steeringMoveHandle: makeElement("steeringMoveHandle"),
  throttleResizeHandle: makeElement("throttleResizeHandle"),
  steeringResizeHandle: makeElement("steeringResizeHandle"),
  joystickSettings: makeElement("joystickSettings"),
  joystickDone: makeElement("joystickDone"),
  joystickSettingsError: makeElement("joystickSettingsError"),
  hud: makeElement("hud"),
};
elements.view3d.clientWidth = 900;
elements.view3d.clientHeight = 600;
elements.view3d.getBoundingClientRect = () => ({ left: 0, top: 0, right: 900, bottom: 600, width: 900, height: 600 });
elements.hud.offsetWidth = 60;
elements.hud.offsetHeight = 20;
for (const stick of [elements.throttleStick, elements.steeringStick]) {
  stick.offsetWidth = 180;
  stick.offsetHeight = 180;
}

const stage = makeElement("stage");
stage.getBoundingClientRect = () => ({ left: 0, top: 0, right: 900, bottom: 600, width: 900, height: 600 });

global.state = { view: "mapping" };
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
global.window = {
  addEventListener() {},
  setInterval() { return 1; },
  clearInterval() {},
};

let fakeNow = 0;
Date.now = () => fakeNow;
global.viewerSessionId = () => "tab-session";
global.glassMotionReduced = () => true;
global.noteLayoutResize = () => {};

const teleopPosts = [];
const uiPosts = [];
let getPayload = {
  ok: true,
  joystick: {
    throttle: { size: 220, x: 40, y: 200 },
    steering: { size: 160, x: 250, y: 200 },
  },
};
let failUiSave = false;
global.getJson = async () => getPayload;
global.postJson = async (url, payload) => {
  if (url === "/api/ui/preferences") {
    if (failUiSave) throw new Error("network down");
    uiPosts.push(payload);
    return { ok: true, joystick: payload };
  }
  teleopPosts.push({ url, payload });
  return { ok: true, teleop: { active: payload.throttle !== 0 || payload.steering !== 0 } };
};
global.fetchWithTimeout = async () => ({ ok: true });

const api = new Function(block + `
return {
  teleop, joystickLayout, refreshJoystickPreferences, persistJoystickPreferences,
  enterJoystickLayout, exitJoystickLayout, stopTeleop,
  beginJoystickMove, moveJoystickGesture, beginJoystickResize, resizeJoystickGesture,
  finishJoystickGesture, applyJoystickLayout, JOYSTICK_VIEWPORT_MARGIN,
};`)();
const {
  teleop, joystickLayout, refreshJoystickPreferences, persistJoystickPreferences,
  enterJoystickLayout, exitJoystickLayout, stopTeleop,
  beginJoystickMove, moveJoystickGesture, beginJoystickResize, resizeJoystickGesture,
  finishJoystickGesture, applyJoystickLayout, JOYSTICK_VIEWPORT_MARGIN,
} = api;

function left(role) {
  return Number.parseFloat((role === "throttle" ? elements.throttleStick : elements.steeringStick).style.left);
}
function top(role) {
  return Number.parseFloat((role === "throttle" ? elements.throttleStick : elements.steeringStick).style.top);
}
function makeEvent(pointerId, clientX, clientY, currentTarget) {
  return {
    pointerId,
    pointerType: "mouse",
    clientX,
    clientY,
    currentTarget,
    preventDefault() {},
    stopPropagation() {},
  };
}

(async () => {
  await refreshJoystickPreferences();
  expect(joystickLayout.sticks.throttle.size === 220, "throttle size from preferences");
  expect(joystickLayout.sticks.steering.size === 160, "steering size from preferences");
  expect(joystickLayout.sticks.throttle.x === 40, "throttle pixel offset from preferences");
  expect(joystickLayout.sticks.throttle.y === 200, "throttle pixel offset from preferences");
  expect(elements.throttleStick.style.values["--stick-size"] === "220px", "throttle size variable applied");
  expect(elements.steeringStick.style.values["--stick-size"] === "160px", "steering size variable applied");
  expect(left("throttle") === 40 && top("throttle") === 200, "throttle uses saved pixel offsets");
  expect(left("steering") === 250 && top("steering") === 200, "steering uses saved pixel offsets");

  // Sidebar collapse grows the viewport; in-bounds sticks must not move.
  elements.view3d.clientWidth = 1100;
  applyJoystickLayout();
  expect(left("throttle") === 40 && top("throttle") === 200, "throttle stays stable when the sidebar collapses");
  expect(left("steering") === 250 && top("steering") === 200, "steering stays stable when the sidebar collapses");

  // Sidebar expand shrinks the viewport; a still-in-bounds stick stays put.
  elements.view3d.clientWidth = 700;
  applyJoystickLayout();
  expect(left("steering") === 250, "an in-bounds stick is not rescaled when the sidebar expands");

  // Clamp only when the saved pixel position is actually outside the viewport.
  elements.view3d.clientWidth = 300;
  applyJoystickLayout();
  const maxLeft = 300 - 160 - JOYSTICK_VIEWPORT_MARGIN - 160 * 0.08;
  expect(Math.abs(left("steering") - maxLeft) < 1e-6, "an overflowing stick clamps to the viewport");
  expect(joystickLayout.sticks.steering.x === 250, "clamping never rebases the saved offset");
  elements.view3d.clientWidth = 900;
  applyJoystickLayout();
  expect(left("steering") === 250, "restoring the viewport restores the saved offset");

  // Role defaults apply when the backend reports null offsets.
  getPayload = {
    ok: true,
    joystick: {
      throttle: { size: 180, x: null, y: null },
      steering: { size: 180, x: null, y: null },
    },
  };
  await refreshJoystickPreferences();
  expect(left("throttle") === JOYSTICK_VIEWPORT_MARGIN, "default throttle sits lower-left");
  expect(
    top("throttle") === 600 - 180 - JOYSTICK_VIEWPORT_MARGIN - 180 * 0.08,
    "default throttle sits at the bottom above the resize overhang",
  );
  expect(
    left("steering") === 900 - 180 - JOYSTICK_VIEWPORT_MARGIN - 180 * 0.08,
    "default steering sits lower-right above the resize overhang",
  );
  expect(
    top("steering") === 600 - 180 - JOYSTICK_VIEWPORT_MARGIN - 180 * 0.08,
    "default steering sits at the bottom above the resize overhang",
  );

  // Restore explicit offsets and enter layout mode.
  getPayload = {
    ok: true,
    joystick: {
      throttle: { size: 200, x: 40, y: 300 },
      steering: { size: 180, x: 500, y: 300 },
    },
  };
  await refreshJoystickPreferences();
  await enterJoystickLayout();
  expect(joystickLayout.mode === true, "gear enters layout mode");
  expect(elements.mappingJoystick.classList.contains("layout-mode"), "layout class applied");

  // Move throttles and confirm the preview equals the geometry saved on release.
  const uiBeforeMove = uiPosts.length;
  beginJoystickMove("throttle", makeEvent(3, 0, 0, elements.throttleMoveHandle));
  moveJoystickGesture(makeEvent(3, 200, -500, elements.throttleMoveHandle));
  const previewLeft = left("throttle");
  const previewTop = top("throttle");
  expect(top("throttle") === JOYSTICK_VIEWPORT_MARGIN, "move clamps the top edge");
  expect(uiPosts.length === uiBeforeMove, "move preview does not persist");
  await finishJoystickGesture(makeEvent(3, 200, -500, elements.throttleMoveHandle));
  expect(uiPosts.length === uiBeforeMove + 1, "move release persists once");
  expect(uiPosts[uiPosts.length - 1].throttle.x === previewLeft, "saved x equals the preview");
  expect(uiPosts[uiPosts.length - 1].throttle.y === previewTop, "saved y equals the preview");

  // Done must keep exactly the previewed geometry.
  exitJoystickLayout();
  expect(joystickLayout.mode === false, "check exits layout mode");
  expect(left("throttle") === previewLeft && top("throttle") === previewTop, "Done keeps the previewed geometry");
  expect(left("steering") === 500 && top("steering") === 300, "resizing/moving one stick leaves the other alone");

  // Independent resize, one write per gesture, and viewport overhang clamping.
  await enterJoystickLayout();
  const uiBeforeResize = uiPosts.length;
  beginJoystickResize("steering", makeEvent(4, 0, 0, elements.steeringResizeHandle));
  resizeJoystickGesture(makeEvent(4, 500, 500, elements.steeringResizeHandle));
  expect(joystickLayout.sticks.steering.size === 260, "steering resize clamps to max");
  expect(joystickLayout.sticks.throttle.size === 200, "throttle size is independent");
  expect(uiPosts.length === uiBeforeResize, "resize preview does not persist");
  await finishJoystickGesture(makeEvent(4, 500, 500, elements.steeringResizeHandle));
  expect(uiPosts.length === uiBeforeResize + 1, "resize release persists once");

  const overhang = joystickLayout.sticks.steering.size * 0.08;
  expect(
    left("steering") + joystickLayout.sticks.steering.size + overhang <= 900,
    "resize handle stays inside the right edge",
  );
  expect(
    top("steering") + joystickLayout.sticks.steering.size + overhang <= 600,
    "resize handle stays inside the bottom edge",
  );

  // A failed save keeps layout mode and surfaces the error; retry clears it.
  failUiSave = true;
  beginJoystickResize("throttle", makeEvent(5, 0, 0, elements.throttleResizeHandle));
  resizeJoystickGesture(makeEvent(5, 20, 20, elements.throttleResizeHandle));
  await finishJoystickGesture(makeEvent(5, 20, 20, elements.throttleResizeHandle));
  expect(joystickLayout.mode === true, "failed save keeps layout mode");
  expect(typeof joystickLayout.saveError === "string" && joystickLayout.saveError.length > 0, "failed save records an error");
  expect(elements.joystickSettingsError.hidden === false, "failed save shows the error surface");
  failUiSave = false;
  beginJoystickResize("throttle", makeEvent(6, 0, 0, elements.throttleResizeHandle));
  resizeJoystickGesture(makeEvent(6, 0, 0, elements.throttleResizeHandle));
  await finishJoystickGesture(makeEvent(6, 0, 0, elements.throttleResizeHandle));
  expect(joystickLayout.saveError === null, "retry clears the error");
  expect(elements.joystickSettingsError.hidden === true, "retry hides the error surface");
  exitJoystickLayout();

  // Hidden layers and zero-sized viewports never write geometry.
  const throttleLeftBefore = elements.throttleStick.style.left;
  elements.mappingJoystick.hidden = true;
  applyJoystickLayout();
  expect(elements.throttleStick.style.left === throttleLeftBefore, "hidden layer is never repositioned");
  elements.mappingJoystick.hidden = false;
  elements.view3d.clientWidth = 0;
  applyJoystickLayout();
  expect(elements.throttleStick.style.left === throttleLeftBefore, "zero-width viewport is never repositioned");
  elements.view3d.clientWidth = 900;

  // HUD parks between the two lower-corner sticks.
  applyJoystickLayout();
  expect(stage.classList.contains("hud-center"), "HUD moves to the lower center between both sticks");

  console.log("two stick layout ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""
)


def test_two_stick_pixel_offsets_preview_parity_and_clamping_under_node():
    completed = _run_app_harness(_TWO_STICK_LAYOUT_HARNESS)
    assert "two stick layout ok" in completed.stdout


_TWO_STICK_ENTRY_SCENARIOS = r"""
const teleopBlock = sliceBlock("// --- Mapping joystick", "// --- End mapping joystick");
const layoutBlock = sliceBlock("// --- Joystick layout", "// --- End joystick layout");
const block = teleopBlock + "\n" + layoutBlock;

const elements = {
  view3d: makeElement("view3d"),
  mappingJoystick: makeElement("mappingJoystick"),
  throttleStick: makeElement("throttleStick"),
  steeringStick: makeElement("steeringStick"),
  throttleBase: makeElement("throttleBase"),
  steeringBase: makeElement("steeringBase"),
  throttleKnob: makeElement("throttleKnob"),
  steeringKnob: makeElement("steeringKnob"),
  joystickSettings: makeElement("joystickSettings"),
  joystickDone: makeElement("joystickDone"),
  joystickSettingsError: makeElement("joystickSettingsError"),
  hud: makeElement("hud"),
};
elements.view3d.clientWidth = 900;
elements.view3d.clientHeight = 600;
elements.view3d.getBoundingClientRect = () => ({ left: 0, top: 0, right: 900, bottom: 600, width: 900, height: 600 });

const stage = makeElement("stage");
global.state = { view: "mapping" };
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
global.window = { addEventListener() {}, setInterval() { return 1; }, clearInterval() {} };
global.viewerSessionId = () => "tab-session";
global.glassMotionReduced = () => true;
global.noteLayoutResize = () => {};

const teleopPosts = [];
const uiPosts = [];
global.handleTeleopPost = async (url, payload) => {
  teleopPosts.push({ url, payload });
  return { ok: true, teleop: { active: payload.throttle !== 0 || payload.steering !== 0 } };
};
global.postJson = async (url, payload) => {
  if (url === "/api/ui/preferences") {
    uiPosts.push(payload);
    return { ok: true, joystick: payload };
  }
  return global.handleTeleopPost(url, payload);
};
global.getJson = async () => ({
  ok: true,
  joystick: {
    throttle: { size: 220, x: 40, y: 200 },
    steering: { size: 160, x: 250, y: 200 },
  },
});
global.fetchWithTimeout = async () => ({ ok: true });

const api = new Function(block + `
return {
  teleop, joystickLayout, refreshJoystickPreferences, persistJoystickPreferences,
  enterJoystickLayout, exitJoystickLayout, stopTeleop,
  beginJoystickResize, resizeJoystickGesture, finishJoystickGesture,
  applyJoystickLayout, JOYSTICK_VIEWPORT_MARGIN,
};`)();
const {
  teleop, joystickLayout, refreshJoystickPreferences, persistJoystickPreferences,
  enterJoystickLayout, exitJoystickLayout, stopTeleop,
  beginJoystickResize, resizeJoystickGesture, finishJoystickGesture,
  applyJoystickLayout, JOYSTICK_VIEWPORT_MARGIN,
} = api;

function makeEvent(pointerId, clientX, clientY, currentTarget) {
  return {
    pointerId,
    pointerType: "mouse",
    clientX,
    clientY,
    currentTarget: currentTarget || elements.throttleBase,
    preventDefault() {},
    stopPropagation() {},
  };
}

(async () => {
"""


def _scenario(source: str) -> str:
    return (
        _FRONTEND_PREAMBLE
        + _TWO_STICK_ENTRY_SCENARIOS
        + source
        + "\n})().catch((error) => { console.error(error); process.exit(1); });\n"
    )


def test_two_stick_layout_entry_race_under_node():
    completed = _run_app_harness(
        _scenario(
            r"""
teleop.active = true;
teleop.pointers.throttle = 7;
let resolveStop = null;
global.handleTeleopPost = async (url, payload) => {
  teleopPosts.push({ url, payload });
  return new Promise((resolve) => {
    resolveStop = () => resolve({ ok: true, teleop: { active: false } });
  });
};
const entering = enterJoystickLayout();
expect(teleop.layoutMode === true, "entry guards before awaiting the stop");
expect(typeof resolveStop === "function", "entry awaits one stop request");
exitJoystickLayout();
expect(teleop.layoutMode === false, "exiting during the stop clears the guard");
resolveStop();
await entering;
expect(joystickLayout.mode === false, "stale entry continuation cannot restore layout mode");
expect(!elements.mappingJoystick.classList.contains("layout-mode"), "stale continuation cannot restore the class");
console.log("two stick entry race ok");
"""
        )
    )
    assert "two stick entry race ok" in completed.stdout


def test_two_stick_idle_layout_entry_requests_zero_stop_under_node():
    completed = _run_app_harness(
        _scenario(
            r"""
await enterJoystickLayout();
expect(teleopPosts.length === 1, "idle layout entry still requests exactly one stop");
const stop = teleopPosts[0].payload;
expect(stop.throttle === 0 && stop.steering === 0, "idle entry stop is a zero command");
const postsAfterEntry = teleopPosts.length;
await stopTeleop("blur");
expect(teleopPosts.length === postsAfterEntry, "an already-stopped ordinary stop stays silent");
console.log("two stick idle entry ok");
"""
        )
    )
    assert "two stick idle entry ok" in completed.stdout


def test_two_stick_null_offsets_resolve_before_save_under_node():
    completed = _run_app_harness(
        _scenario(
            r"""
await refreshJoystickPreferences();
joystickLayout.sticks.throttle.x = null;
joystickLayout.sticks.throttle.y = null;
joystickLayout.sticks.steering.x = null;
joystickLayout.sticks.steering.y = null;
await enterJoystickLayout();
beginJoystickResize("throttle", makeEvent(1, 0, 0, elements.throttleStick));
resizeJoystickGesture(makeEvent(1, 10, 10, elements.throttleStick));
await finishJoystickGesture(makeEvent(1, 10, 10, elements.throttleStick));
const saved = uiPosts[uiPosts.length - 1];
expect(typeof saved.throttle.x === "number" && typeof saved.throttle.y === "number", "throttle offsets resolve before save");
expect(typeof saved.steering.x === "number" && typeof saved.steering.y === "number", "steering offsets resolve before save");
console.log("two stick null offset resolution ok");
"""
        )
    )
    assert "two stick null offset resolution ok" in completed.stdout


def test_two_stick_null_defaults_materialize_and_stay_put_under_node():
    completed = _run_app_harness(
        _scenario(
            r"""
function shownLeft(role) {
  return Number.parseFloat(
    (role === "throttle" ? elements.throttleStick : elements.steeringStick).style.left,
  );
}
function shownTop(role) {
  return Number.parseFloat(
    (role === "throttle" ? elements.throttleStick : elements.steeringStick).style.top,
  );
}

// Start from null role-default offsets on a state where the layout has no real
// viewport: defaults must stay unmaterialized.
elements.mappingJoystick.hidden = true;
elements.view3d.clientWidth = 0;
global.getJson = async () => ({
  ok: true,
  joystick: {
    throttle: { size: 180, x: null, y: null },
    steering: { size: 180, x: null, y: null },
  },
});
await refreshJoystickPreferences();
expect(joystickLayout.sticks.throttle.x === null, "hidden layout does not materialize throttle x");
expect(joystickLayout.sticks.steering.x === null, "hidden layout does not materialize steering x");
expect(
  joystickLayout.sticks.throttle.y === null && joystickLayout.sticks.steering.y === null,
  "hidden layout keeps both y defaults",
);

elements.mappingJoystick.hidden = false;
elements.view3d.clientWidth = 0;
applyJoystickLayout();
expect(
  joystickLayout.sticks.throttle.x === null && joystickLayout.sticks.steering.x === null,
  "zero-sized viewport does not materialize defaults",
);

// First real apply at the expanded-sidebar width: each null offset becomes the
// exact clamped pixel geometry that is shown.
elements.view3d.clientWidth = 900;
applyJoystickLayout();
const firstThrottleLeft = shownLeft("throttle");
const firstThrottleTop = shownTop("throttle");
const firstSteeringLeft = shownLeft("steering");
const firstSteeringTop = shownTop("steering");
expect(joystickLayout.sticks.throttle.x === firstThrottleLeft, "throttle x materializes to the shown pixel");
expect(joystickLayout.sticks.throttle.y === firstThrottleTop, "throttle y materializes to the shown pixel");
expect(joystickLayout.sticks.steering.x === firstSteeringLeft, "steering x materializes to the shown pixel");
expect(joystickLayout.sticks.steering.y === firstSteeringTop, "steering y materializes to the shown pixel");

// Wider collapsed-sidebar width: the materialized in-bounds pixels must not
// drift, especially the steering default that previously tracked viewportWidth.
elements.view3d.clientWidth = 1100;
applyJoystickLayout();
expect(
  shownLeft("throttle") === firstThrottleLeft && shownTop("throttle") === firstThrottleTop,
  "throttle stays at its materialized pixels when the sidebar collapses",
);
expect(
  shownLeft("steering") === firstSteeringLeft && shownTop("steering") === firstSteeringTop,
  "steering stays at its materialized pixels when the sidebar collapses",
);
expect(joystickLayout.sticks.steering.x === firstSteeringLeft, "the stored steering x is unchanged");
expect(joystickLayout.sticks.steering.y === firstSteeringTop, "the stored steering y is unchanged");

// Materializing is in-memory only: no preference write is triggered.
expect(uiPosts.length === 0, "materializing defaults never persists on its own");
console.log("two stick default materialize ok");
"""
        )
    )
    assert "two stick default materialize ok" in completed.stdout


def test_two_stick_stale_preference_responses_are_ignored_under_node():
    completed = _run_app_harness(
        _scenario(
            r"""
joystickLayout.mode = true;
const pendingUi = [];
global.postJson = async (url, payload) => {
  if (url !== "/api/ui/preferences") return { ok: true, teleop: { active: false } };
  return await new Promise((resolve) => pendingUi.push({ payload, resolve }));
};

joystickLayout.sticks.throttle.size = 150;
const firstSave = persistJoystickPreferences();
joystickLayout.sticks.throttle.size = 200;
const secondSave = persistJoystickPreferences();
expect(pendingUi.length === 2, "two preference saves are in flight");
pendingUi[1].resolve({
  ok: true,
  joystick: {
    throttle: { size: 200, x: 20, y: 200 },
    steering: { size: 160, x: 250, y: 200 },
  },
});
await secondSave;
expect(joystickLayout.sticks.throttle.size === 200, "newer save result applies");
expect(joystickLayout.sticks.throttle.x === 20, "newer save offsets apply");
pendingUi[0].resolve({
  ok: true,
  joystick: {
    throttle: { size: 150, x: 10, y: 100 },
    steering: { size: 160, x: 250, y: 200 },
  },
});
await firstSave;
expect(joystickLayout.sticks.throttle.size === 200, "older save cannot overwrite a newer size");
expect(joystickLayout.sticks.throttle.x === 20, "older save cannot overwrite newer offsets");

joystickLayout.sticks.throttle.size = 160;
const gestureSave = persistJoystickPreferences();
const gestureEntry = pendingUi[pendingUi.length - 1];
beginJoystickResize("throttle", makeEvent(9, 0, 0, elements.throttleStick));
resizeJoystickGesture(makeEvent(9, 30, 30, elements.throttleStick));
const gestureSize = joystickLayout.sticks.throttle.size;
expect(gestureSize !== 160, "gesture preview changed the size");
gestureEntry.resolve({
  ok: true,
  joystick: {
    throttle: { size: 140, x: 900, y: 900 },
    steering: { size: 160, x: 250, y: 200 },
  },
});
await gestureSave;
expect(joystickLayout.sticks.throttle.size === gestureSize, "older save cannot overwrite an in-progress gesture");
expect(joystickLayout.sticks.throttle.x === 20, "older save cannot overwrite in-progress gesture offsets");
console.log("two stick preference revision ok");
"""
        )
    )
    assert "two stick preference revision ok" in completed.stdout


def test_mapping_joystick_dom_is_dual_icon_and_status_free():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    view_start = index_html.index('id="view3d"')
    view_end = index_html.index('id="viewSystem"', view_start)
    viewport = index_html[view_start:view_end]

    # Two independent sticks with per-stick move/resize handles.
    assert 'id="throttleStick"' in viewport
    assert 'id="steeringStick"' in viewport
    assert 'id="throttleBase"' in viewport
    assert 'id="steeringBase"' in viewport
    assert 'id="throttleKnob"' in viewport
    assert 'id="steeringKnob"' in viewport
    assert 'id="throttleMoveHandle"' in viewport
    assert 'id="steeringMoveHandle"' in viewport
    assert 'id="throttleResizeHandle"' in viewport
    assert 'id="steeringResizeHandle"' in viewport

    # Shared gear/check icon controls with accessible labels and titles.
    assert 'id="joystickSettings"' in viewport
    assert 'id="joystickDone"' in viewport
    assert 'aria-label="Joystick settings"' in viewport
    assert 'title="Joystick settings"' in viewport
    assert 'aria-label="Confirm joystick layout"' in viewport
    assert 'title="Done"' in viewport
    assert "<svg" in viewport
    assert ">Settings<" not in viewport
    assert ">Done<" not in viewport

    # The visible idle/active/error status text is gone, but the settings error
    # surface and an accessible status mechanism remain.
    assert 'id="joystickStatus"' not in index_html
    assert "setJoystickStatus" not in app_js
    assert "joystick-status" not in styles_css
    assert 'id="joystickSettingsError"' in viewport
    assert 'role="status"' in viewport

    # Per-stick layout mode handles and shared controls are styled.
    assert ".joystick-stick" in styles_css
    assert ".joystick.layout-mode .joystick-move-handle" in styles_css
    assert ".joystick.layout-mode .joystick-resize-handle" in styles_css
    assert ".joystick.layout-mode .joystick-done-button" in styles_css
    assert ".joystick-icon-button" in styles_css

    # Lifecycle: entering Mapping shows both sticks; leaving hides them.
    assert "setMappingJoystickVisible(true, { deferReveal: deferJoystickReveal });" in app_js
    assert "setMappingJoystickVisible(false);" in app_js
    assert "exitJoystickLayout();" in app_js
