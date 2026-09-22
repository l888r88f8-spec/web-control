# Navigation Process Controls Design

## Goal

Move the localization and navigation process controls into the existing
Navigation Readiness card. Replace each separate Start and Stop pair with one
button whose label and action follow the corresponding managed process state.

## Layout

The Navigation Readiness card keeps its existing heading and readiness status.
Two full-width buttons appear between them, in this order:

1. `Start Localization` or `Stop Localization`
2. `Start Navigation` or `Stop Navigation`

The button text already names the process, so no separate Localization or
Navigation labels are shown. The existing localization and navigation action
rows are removed from Terminal Logs. Their state pills and log output remain.
The mapping Start and Stop controls remain in the Mapping panel.

## Behavior

Each button is a single managed-process toggle:

- When its process is stopped, it displays `Start <Process>` and posts the
  process name to `/api/managed/start`.
- When its process is running, it displays `Stop <Process>` and posts the
  process name to `/api/managed/stop`.
- While its request is pending, it is disabled to prevent duplicate commands.
- After a successful request, the page refreshes process logs and state so the
  server remains the source of truth for the displayed action.
- After a failed request, the button is re-enabled, keeps the action implied by
  the latest known process state, and reports the existing error message in
  `publishResult`.

The existing stop side effects remain: stopping localization clears map scene
state and reloads the map, while stopping navigation clears navigation scene
state and navigation layers. Existing map-selection validation and server error
handling also remain.

## State and Accessibility

The existing `localizationRunning` and `navigationRunning` values from the log
poll determine each button's normal label and action. A separate client-side
pending flag per process prevents a request for one process from disabling the
other button.

Each button exposes its current state through its visible text, disabled state,
and an accessibility attribute that reflects whether its process is running.
Start and Stop visual styles continue to use the existing positive and danger
colors, with text as the primary distinction.

## Scope

The change is limited to the navigation page HTML, browser behavior, styling,
and frontend-focused tests. Existing managed-process HTTP endpoints and backend
process management do not change. Goal Pause, Resume, and Clear Goal controls
are unrelated and remain in the Target Pose section.

## Verification

Automated tests will verify:

- both toggles are inside Navigation Readiness, below its heading and above its
  readiness status;
- the old localization and navigation Start/Stop pairs are absent from Terminal
  Logs while mapping controls remain;
- labels and actions switch independently between Start and Stop based on the
  latest known process state;
- a pending command disables only its own button and prevents duplicate posts;
- successful and failed requests preserve the existing refresh, cleanup, and
  error-reporting behavior;
- JavaScript syntax, Python compilation, and the full test suite pass.

The user will run the ROS workspace build, so implementation verification will
not run `colcon`.
