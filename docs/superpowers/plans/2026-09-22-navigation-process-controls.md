# Navigation Process Controls Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Put one state-aware localization button and one state-aware navigation button inside Navigation Readiness while preserving the existing managed-process APIs and cleanup behavior.

**Architecture:** The readiness card owns two frontend-only process toggle buttons. Existing log polling supplies the running state, a per-process pending action prevents duplicate requests, and the current start/stop functions continue to call the same backend endpoints. Terminal cards retain their status pills and output, while mapping keeps its existing separate controls.

**Tech Stack:** HTML, CSS, browser JavaScript, Node.js test harness, Python/pytest.

---

### Task 1: Add managed-process toggles to Navigation Readiness

**Files:**
- Modify: `static/index.html:76-79,137-163`
- Modify: `static/app.js:15-40,824-895,1181-1264`
- Modify: `static/styles.css:825-858,1008-1030`
- Test: `test/test_core.py:606-626`

- [ ] **Step 1: Write failing layout and behavior tests**

Replace `test_frontend_exposes_managed_start_controls` with assertions that
slice the Navigation Readiness section and both terminal articles:

```python
def test_navigation_readiness_owns_managed_process_toggles():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")

    readiness_start = index_html.index("<h2>Navigation Readiness</h2>")
    readiness_end = index_html.index("</section>", readiness_start)
    readiness = index_html[readiness_start:readiness_end]
    terminal_start = index_html.index("<h2>Terminal Logs</h2>")
    terminal_end = index_html.index("<h2>Layers</h2>", terminal_start)
    terminals = index_html[terminal_start:terminal_end]

    assert readiness.index('data-managed-target="localization"') < readiness.index(
        'data-managed-target="navigation"'
    )
    assert readiness.index('data-managed-target="navigation"') < readiness.index(
        'id="navigationReady"'
    )
    assert "Start Localization" in readiness
    assert "Start Navigation" in readiness
    assert 'data-start-target="localization"' not in terminals
    assert 'data-stop-target="localization"' not in terminals
    assert 'data-start-target="navigation"' not in terminals
    assert 'data-stop-target="navigation"' not in terminals
    assert 'data-start-target="mapping"' in index_html
    assert 'data-stop-target="mapping"' in index_html
```

Add a Node harness test named
`test_frontend_managed_process_toggles_follow_running_and_pending_state`. It
must load the real `static/app.js`, extract `managedProcessRunning`,
`updateManagedProcessButton`, and `toggleManagedTarget`, and provide minimal
stubs for `document.getElementById`, `startManagedTarget`, and
`stopManagedTarget`. Verify all of these behaviors:

```javascript
updateManagedProcessButton("localization");
expect(localizationButton.textContent === "Start Localization", "initial localization label");
expect(localizationButton.disabled === false, "initial localization enabled");
expect(localizationButton.getAttribute("aria-pressed") === "false", "initial aria state");

state.localizationRunning = true;
updateManagedProcessButton("localization");
expect(localizationButton.textContent === "Stop Localization", "running localization label");
expect(localizationButton.classList.contains("stop-log"), "running stop style");

const pending = toggleManagedTarget("navigation");
expect(navigationButton.disabled === true, "pending navigation disabled");
toggleManagedTarget("navigation");
expect(startCalls.navigation === 1, "duplicate navigation start blocked");
expect(localizationButton.disabled === false, "localization remains independent");
resolveNavigationStart();
await pending;
expect(navigationButton.disabled === false, "navigation re-enabled after request");

state.navigationRunning = true;
await toggleManagedTarget("navigation");
expect(stopCalls.navigation === 1, "running navigation stops");
```

The harness must also assert that the toggle event selector exists in
`attachEvents` and that the existing mapping selectors remain:

```javascript
expect(source.includes('[data-managed-target]'), "toggle event selector missing");
expect(source.includes('[data-start-target]'), "mapping start selector missing");
expect(source.includes('[data-stop-target]'), "mapping stop selector missing");
```

- [ ] **Step 2: Run focused tests and verify RED**

Run:

```bash
python3 -m pytest -q test/test_core.py -k "navigation_readiness_owns_managed_process_toggles or frontend_managed_process_toggles"
```

Expected: the layout test fails because the readiness buttons do not exist,
and the Node test fails because the three toggle functions do not exist.

- [ ] **Step 3: Move the controls in HTML**

Change the Navigation Readiness section to:

```html
<section>
  <h2>Navigation Readiness</h2>
  <div class="actions process-controls">
    <button id="localizationProcessToggle" class="start-log" data-managed-target="localization" type="button" aria-pressed="false">Start Localization</button>
    <button id="navigationProcessToggle" class="start-log" data-managed-target="navigation" type="button" aria-pressed="false">Start Navigation</button>
  </div>
  <div id="navigationReady" class="readiness waiting">waiting for relocalization</div>
</section>
```

Delete only the `.terminal-actions` blocks from the Localization and Navigation
terminal articles. Keep their headings, state pills, and `<pre>` elements.
Leave the Mapping panel controls unchanged.

- [ ] **Step 4: Add minimal readiness-card styling**

Reuse `.actions`, `.start-log`, `.stop-log`, and the existing disabled button
rules. Add only spacing needed for the readiness card:

```css
.process-controls {
  margin-bottom: 12px;
}

.process-controls .start-log,
.process-controls .stop-log {
  padding: 8px;
}
```

Extend the existing disabled selectors so a pending process button receives
the same disabled presentation as other action buttons:

```css
.actions button:disabled,
.process-controls button:disabled {
  border-color: rgba(148, 163, 184, 0.22);
  background: rgba(148, 163, 184, 0.12);
  color: rgba(148, 163, 184, 0.72);
  cursor: not-allowed;
  box-shadow: none;
}
```

Do the same for the adjacent disabled-hover rule. Do not change Mode segmented
control CSS.

- [ ] **Step 5: Implement independent state-aware toggles**

Add per-process pending actions to `state`:

```javascript
managedProcessPending: { localization: null, navigation: null },
```

Add these functions near `updateTerminalLog`:

```javascript
function managedProcessRunning(target) {
  return target === "localization" ? state.localizationRunning : state.navigationRunning;
}

function updateManagedProcessButton(target) {
  const button = document.getElementById(`${target}ProcessToggle`);
  if (!button) return;
  const running = managedProcessRunning(target);
  const pending = state.managedProcessPending[target];
  const label = target === "localization" ? "Localization" : "Navigation";
  const action = running ? "Stop" : "Start";
  button.textContent = pending ? `${pending === "stop" ? "Stopping" : "Starting"} ${label}...` : `${action} ${label}`;
  button.disabled = Boolean(pending);
  button.classList.toggle("start-log", !running);
  button.classList.toggle("stop-log", running);
  button.setAttribute("aria-pressed", String(running));
  button.setAttribute("aria-busy", String(Boolean(pending)));
}

async function toggleManagedTarget(target) {
  if (state.managedProcessPending[target]) return;
  const running = managedProcessRunning(target);
  state.managedProcessPending[target] = running ? "stop" : "start";
  updateManagedProcessButton(target);
  try {
    if (running) await stopManagedTarget(target);
    else await startManagedTarget(target);
  } finally {
    state.managedProcessPending[target] = null;
    updateManagedProcessButton(target);
  }
}
```

After `updateTerminalLog` stores `localizationRunning` or
`navigationRunning`, call `updateManagedProcessButton(target)`. This ensures a
successful or failed subsequent poll restores the button from server state.

In `attachEvents`, keep the existing `[data-start-target]` and
`[data-stop-target]` listeners for Mapping, and add:

```javascript
document.querySelectorAll("[data-managed-target]").forEach((button) =>
  button.addEventListener("click", () => toggleManagedTarget(button.dataset.managedTarget))
);
```

Do not alter the HTTP endpoints or the scene cleanup inside
`stopManagedTarget`.

- [ ] **Step 6: Run focused tests and verify GREEN**

Run:

```bash
python3 -m pytest -q test/test_core.py -k "navigation_readiness_owns_managed_process_toggles or frontend_managed_process_toggles"
```

Expected: both tests pass.

- [ ] **Step 7: Run full verification**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY python3 -m pytest -q test
node --check static/app.js
python3 -m py_compile web_control/*.py
git diff --check
```

Expected: every command exits zero. Do not run `colcon`.

- [ ] **Step 8: Commit the implementation**

```bash
git add static/index.html static/app.js static/styles.css test/test_core.py
git commit -m "feat: add navigation process toggles"
```
