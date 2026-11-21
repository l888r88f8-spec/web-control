# Publisher-Driven Cloud Refresh Implementation Plan

> **For agentic workers:** Execute inline in this session. Tests first; no commits.

**Goal:** Make dynamic point-cloud display follow publisher frames end-to-end (latest-only under load) without adding ROS subscriptions or fixed 1/5 Hz browser deadlines.

**Status (2026-10-31):** Implemented. Focused backend/frontend contracts pass; full `nice -n 15 python -m pytest test -q` -> **725 passed** (baseline 708 plus feature tests). `node --check static/app.js`, `py_compile` for the changed node/runtime modules, and `git diff --check` pass. Follow-up review fixed three races: stale concurrent metadata snapshots, old in-flight geometry after lifecycle clears, and browser cursors after a web-service restart (per-server `epoch`). Remaining limitation: this is Python/browser test evidence only; no robot/hardware or runtime ROS activation was performed.

**Architecture:** The existing single decoder worker loses its dynamic rate ceilings and keeps only the newest pending frame per layer. A stdlib `CloudRevisionHub` publishes metadata-only revisions (version/status/frame/count) to per-client SSE streams served by the existing threaded HTTP server. The browser keeps one EventSource for the visible view, fetches only changed revisions through the existing cloud GET endpoints (one in flight per layer, latest followup coalesced), and keeps paths/pose/static-map behavior unchanged.

**Tech Stack:** Python 3.10 stdlib (`threading`, `http.server`), NumPy decoder, vanilla JS `EventSource`, existing pytest/Node harness.

---

## Files

- `web_control/pointcloud_runtime.py` — remove dynamic rate ceilings; add bounded `CloudRevisionHub`.
- `web_control/web_control_node.py` — expose metadata snapshots, publish after frame/clear, SSE route `/api/cloud/events`, lifecycle start/stop.
- `static/app.js` — remove cloud fixed deadlines; add revision-driven fetch controller and one EventSource lifecycle.
- `test/test_publisher_driven_cloud.py` — backend unit + real localhost SSE behavior.
- `test/test_cloud_stream_frontend.py` — executed Node contract tests.
- Existing tests that encode 1/5 Hz cloud caps will be updated to the new contract.
- `README.md` — minimal behavior/cost note; no CPU percentage claim.

## Task 1 — Backend RED

- [x] Add worker tests with injected clock: `local_map`, `obstacles`, and mapping layers are ready immediately (including >5 Hz), and a slow decode leaves one newest pending frame.
- [x] Add `CloudRevisionHub` tests: duplicate metadata does not notify, changed version/status does, two independent clients both see a change, stop unblocks waiters.
- [x] Add localhost SSE tests with a fake node + real `ThreadingHTTPServer`: initial snapshot, per-client layer filtering, no duplicate event for unchanged metadata, reconnect snapshot, clean hub stop/HTTP shutdown.
- [x] Run `pytest test/test_publisher_driven_cloud.py -q` and record RED, then GREEN.

## Task 2 — Frontend RED

- [x] Add executed-Node tests for `createRefreshController`: revision notification fetches at unchanged fake clock, duplicate version is a no-op, slow in-flight fetch coalesces to the newest version and refetches, reset/generation rejects stale responses, missing initial version fetches, static layer loads once.
- [x] Add executed-Node tests for `createCloudStreamController`: one EventSource per desired signature, switching closes the old one first, hidden closes it, snapshot/cloud metadata parsed, unavailable SSE starts bounded fallback, reconnect uses bounded delay.
- [x] Run `node` harness via pytest and record RED, then GREEN.

## Task 3 — Minimal backend GREEN

- [x] In `pointcloud_runtime.py`, ignore rate ceilings for dynamic keys (static latched maps keep their one-shot input-driven behavior); add `CloudRevisionHub` with condition waits, heartbeat-friendly `wait_for_changes`, metadata-only snapshots, and bounded stop.
- [x] In `web_control_node.py`, add `cloud_metadata_snapshot()`, publish after applied frames and clears, create/start/stop the hub with the HTTP server, validate `group`/`layers`, and stream `snapshot`/`cloud` events plus 1 s heartbeats.
- [x] SSE handler renews demand for exactly its own watched keys, writes only metadata, bounds socket writes/timeouts, and exits on disconnect or hub stop.
- [x] Run focused tests green; update old rate assertions to the new contract.

## Task 4 — Minimal frontend GREEN

- [x] Replace cloud fixed deadlines with revision-driven per-layer tasks; keep path 5 Hz, mapping trajectory 5 Hz, pose 100 ms.
- [x] Track latest/applied/requested version; one in-flight GET per layer; on completion refetch only if the latest revision is newer.
- [x] Add one `createCloudStreamController`; sync on view/layer/visibility changes; close before opening on signature changes; bounded reconnect/fallback only when SSE is unavailable.
- [x] Static navigation maps still fetch only until their first valid nonempty load (and after reset); unchanged revisions never refetch.

## Task 5 — Verification

- [x] Full `pytest` under `nice -n 15`: 727 passed, 0 failed (final source, including renderer-clear and HTTP-fallback restart regressions).
- [x] `node --check static/app.js`; `python -m py_compile` changed Python files; `git diff --check`; inspect no driver/slam edits.
- [x] Update `README.md`: dynamic layers follow input when able; latest frame may be dropped under load; static maps still load once; SSE is metadata-only and falls back/reconnects; no CPU percentage claims.
- [x] Report exact modified files, RED/GREEN evidence, limitations, and activation (restart web node + hard refresh; no ROS/build changes).
- [x] Regression tests: concurrent refresh ordering/status barriers, lock-order publish, navigation/mapping lifecycle in-flight clear, static generation reload, server-epoch reconnect and retired-source rejection.
- [x] Independent renderer/fallback review: two added Node regressions failed before the fixes and passed after; clears update rendered geometry before another GET completes, and HTTP-discovered server restarts accept lower version counters.

## Explicit non-goals

- No new ROS subscriptions, publishers, launches, hardware actions, installs, runtime parameter changes, commits/stash/reset/push, or heavy benchmarks.
- No edits outside this web repo.
- Paths/trajectory and pose cadence unchanged; static localization/navigation maps unchanged unless a real lifecycle reset invalidates them.
