# Adjustable Joystick and Collapsible Sidebar Design

## Goal

Allow an operator to resize and reposition the Mapping joystick, persist those
joystick settings with the selected map, and smoothly collapse the right
information sidebar to enlarge the map viewport.

## Joystick Settings Interface

The Mapping joystick gains a small settings button that travels with the
joystick. Activating it first requests a teleoperation stop, ends the active
send loop, and enters a layout mode in which vehicle control is disabled.

Layout mode shows:

- a move handle above the joystick for repositioning the whole control;
- a resize handle at the lower-right edge for resizing the circular base;
- a `Done` action that exits layout mode and restores vehicle control.

The move and resize handles use Pointer Events and pointer capture, so mouse,
pen, and single-touch input behave consistently. Their events stop propagation
and never reach the Three.js map canvas.

## Size Adjustment

Dragging the resize handle changes the joystick base diameter continuously.
The base stays circular and is clamped to `140px` through `260px`, with a
default of `180px` when no saved setting exists. The center knob, cross axes,
settings controls, and status presentation scale with the base through CSS
custom properties derived from the chosen diameter.

The resize operation previews continuously in the browser. It writes one
backend update when the pointer is released or cancelled rather than writing
during every pointer movement.

## Position Adjustment

Dragging the move handle repositions the complete joystick container within
the shared 3D map viewport. Position is stored as normalized center
coordinates, `x_ratio` and `y_ratio`, relative to the viewport dimensions.

The browser clamps the joystick, handles, and status area into the visible map
area during movement, restoration, window resizing, sidebar animation, and tab
changes. A saved position therefore remains usable at different resolutions.
The default position is near the lower-left corner.

Moving previews continuously and writes one backend update when the pointer is
released or cancelled. Leaving Mapping while layout mode is active exits the
mode without enabling teleoperation on another tab.

## Persistence in selected_map.json

Joystick settings share the existing file:

```text
~/.config/web_control/selected_map.json
```

The expanded structure is:

```json
{
  "name": "hesai_sim2",
  "ui": {
    "joystick": {
      "size": 180,
      "x_ratio": 0.05,
      "y_ratio": 0.75
    }
  }
}
```

The existing legacy form, `{"name":"..."}`, remains valid and supplies
default joystick settings. `SelectionStore.save(name)` becomes a locked
read-modify-write operation that updates only `name`. Joystick setting writes
update only `ui.joystick`. Both use the existing atomic file replacement and
preserve unknown top-level and nested keys so the two updates cannot erase one
another.

The store validates map names with the existing validator. Joystick size must
be a finite number within `[140, 260]`; position ratios must be finite numbers
within `[0, 1]`. Invalid or partially missing saved UI data falls back to the
corresponding defaults without invalidating a valid selected map.

The settings are global for this web-control server, so all browsers load the
same saved joystick layout and the last completed adjustment wins.

## UI Preferences API

The web node exposes focused endpoints:

- `GET /api/ui/preferences` returns validated joystick settings;
- `POST /api/ui/preferences` accepts the complete joystick setting object,
  validates it, atomically merges it into `selected_map.json`, and returns the
  saved result.

Malformed, nonfinite, or out-of-range values return HTTP 400 without changing
the file. Failed saves leave the last valid file intact and show an error in
the joystick settings UI. The in-memory preview remains in layout mode so the
operator can continue adjusting and retry the save.

## HUD Avoidance

The HUD no longer relies on one fixed Mapping offset. It normally remains in
the lower-left corner. If the joystick container overlaps that corner, the HUD
moves to the lower-right corner; if the joystick overlaps the lower-right, the
HUD uses the lower-left. Repositioning or resizing the joystick and resizing
the map triggers the same deterministic calculation. Other tabs retain their
existing HUD placement.

## Collapsible Sidebar

One toggle button is positioned on the boundary between the map stage and the
right information sidebar, vertically centered. The same DOM button remains
present for both states; only its arrow direction and accessibility state
change.

Collapsing animates the grid sidebar track from its current width to `0px` over
approximately `260ms`. Panel content fades and shifts slightly right during the
same transition. The toggle follows the moving boundary to the right edge, so
there is no replacement button or visual jump. Expanding reverses the motion.

A `ResizeObserver` watches the map stage and calls the existing renderer resize
logic during the grid transition, keeping the Three.js canvas aspect and
resolution synchronized with the visible map. The toggle exposes
`aria-expanded` and `aria-controls="controlPanel"` and remains keyboard
operable.

The collapsed state is browser-memory state only. It is not written to the
backend, session storage, or local storage. Every page load starts with the
sidebar expanded. The System tab continues to use its existing panel-hidden
layout and hides the sidebar toggle; returning to Navigation or Mapping
restores the current in-page collapsed state and shows the same toggle.

## Component Boundaries

- `SelectionStore` owns locked compatible JSON merging, selection access, and
  validated joystick preference access.
- `WebControlNode` exposes the two preference HTTP routes using the existing
  store and error mapping.
- The joystick frontend owns layout-mode pointer lifecycles, preview,
  persistence calls, viewport clamping, and HUD avoidance.
- A small sidebar frontend component owns its in-memory collapsed state,
  accessibility attributes, CSS class, and renderer resize observation.

No teleoperation speed mapping, `/cmd_vel` safety sequencing, process gating,
map selection semantics, launch files, or ROS message dependencies change.

## Verification

Important automated tests will cover:

- legacy selection JSON compatibility and preservation of `ui` during map
  selection updates;
- preservation of `name` during joystick updates, unknown-key preservation,
  validation, atomic writes, and concurrent store calls;
- preference GET/POST success and invalid-payload behavior;
- layout mode stopping teleoperation and disabling drive gestures;
- pointer-captured move and resize, size limits, proportional knob scaling,
  viewport clamping, one save per completed gesture, restoration, and save
  failure behavior;
- one sidebar toggle element, smooth class/state transitions,
  `aria-expanded`, panel content behavior, no persistence, and renderer resize
  during collapse;
- JavaScript syntax, Python compilation, and one full package regression.

The user will run the ROS workspace build, so implementation verification will
not run `colcon`.
