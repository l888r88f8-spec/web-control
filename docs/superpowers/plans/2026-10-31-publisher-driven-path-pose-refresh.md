# Publisher-Driven Path/Pose Refresh Plan

**Goal:** Make Navigation paths, Mapping trajectory, and vehicle pose update from new ROS messages, with latest-only coalescing under load and no normal fixed path/pose frontend polling.

**Status (2026-10-31):** Implemented. Focused backend/frontend contracts pass; full `nice -n 15 python -m pytest test -q` -> **735 passed** (final source and review regressions, 84.31 s). `node --check static/app.js`, `py_compile` for the changed node/runtime modules, and `git diff --check` pass. No ROS runtime activation or hardware validation was performed. Transport choice: path revisions use the existing GET snapshot (arrays stay out of SSE/hub/node-held locks); pose is tiny and travels inline in SSE to avoid one HTTP request per odometry frame.

## Design

- Reuse the existing `CloudRevisionHub`, per-server `epoch`, per-layer generations, `/api/cloud/events` SSE route, and cloud GET single-flight logic. Do not add ROS subscriptions or dependencies.
- Extend metadata entries to include lightweight path revisions and tiny pose payloads:
  - `("path", topic)` metadata only (version/generation/status/count).
  - `("pose", "nav"|"mapping")` with a small pose dict, never point arrays.
- Extend `/api/cloud/events` with validated `paths` and `pose=nav|mapping` resources (`pose=1` remains compatible). One EventSource per visible view carries enabled clouds, configured paths, mapping path, and pose. Path topics are accepted only when already configured on the backend.
- Browser path resources fetch only on a new revision through the existing `/api/path` and `/api/mapping/path` endpoints, one request in flight, newest pending revision coalesced. Pose arrives inline in SSE, avoiding per-odometry HTTP requests.
- Remove the 200 ms path/trajectory deadlines and the 100 ms pose timer. The 50 ms scheduler only drains pending revisions (no cadence). Hidden/system views close the stream and do not fetch; showing a view applies the latest snapshot.
- HTTP path/pose snapshots expose version/generation/epoch under the cache lock so late old responses are rejected. Lifecycle clears bump state generations, clear affected display immediately, and cannot be resurrected by in-flight work.
- Keep `/api/state` and `/api/mapping/state` for status/control, but stop applying their pose fields so old status polls cannot overwrite newer pushed pose.

## Tests First

- Backend: pose/path callbacks publish metadata without arrays; HTTP payloads carry version/generation/epoch; clears bump generations; SSE includes path/pose, validates paths, filters per client, and renews demand only for cloud keys.
- Frontend (executed Node): >5 Hz path events and >10 Hz inline pose events update at unchanged fake clock; slow path GET coalesces to latest; mapping path independent; hidden/system stops; clear/epoch/generation rejects late responses and clears immediately; path toggle/hide-completed preserved; one EventSource with no clouds; static cloud contract unchanged.
- Replace old fixed-rate assertions with the new event-driven behavior instead of deleting coverage.

## Verification

- Focused pytest red/green under the Harness namespace (real localhost SSE binds are allowed there).
- Full `nice -n 15 python -m pytest test -q` under Harness.
- `node --check static/app.js`, `python -m py_compile` changed modules, `git diff --check`.
- Update README behavior and baseline count; no CPU percentage or hardware claims. No commits, installs, ROS runtime actions, or non-web edits.

## Final review

- Removed paths reject late GETs; pose fallback uses the same epoch/generation checks and permits only one in-flight request. Both regressions failed before fixes and passed afterwards.
- Integration tests verify the visible view selects its own pose, all clouds can be disabled while pose updates continue, and hidden pages close the stream.
- No runtime activation was performed. Restart the web node and reload the browser when the robot is idle.
