# Three-Dimensional Path Rendering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve ROS path height through the web API and render navigation and mapping paths on their real floor.

**Architecture:** Path callbacks serialize each pose as `[x, y, z]`. A small browser helper converts either new three-value points or legacy two-value points into Three.js coordinates and adds the existing `0.08 m` visibility offset.

**Tech Stack:** ROS 2 Python messages, Python/pytest, browser JavaScript, Three.js, Node.js.

---

### Task 1: Preserve and render path height

**Files:**
- Modify: `web_control/web_control_node.py:974-998`
- Modify: `static/app.js:377-406`
- Test: `test/test_core.py`

- [ ] **Step 1: Write failing backend and frontend tests**

Add a backend test that constructs navigation and mapping `Path`-like messages
whose poses have distinct `z` values, invokes both callbacks, and asserts that
the cached points are exactly `[x, y, z]`.

Add a Node harness that extracts `pathPointToThree`, supplies this stub:

```javascript
function rosToThree(x, y, z) { return { x, y, z }; }
```

and asserts:

```javascript
const upper = pathPointToThree([1, 2, 3]);
if (Math.abs(upper.z - 3.08) > 1e-9) throw new Error("3D path height lost");
const legacy = pathPointToThree([1, 2]);
if (Math.abs(legacy.z - 0.08) > 1e-9) throw new Error("legacy path fallback wrong");
```

Also assert both `updatePaths` and `updateMappingPath` call
`pathPointToThree(point)`.

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
python3 -m pytest -q test/test_core.py -k "path_callbacks_preserve_z or frontend_path_height"
```

Expected: backend assertions fail because callbacks return `[x, y]`, and the
Node harness fails because `pathPointToThree` does not exist.

- [ ] **Step 3: Implement the minimal backend change**

Change both callback comprehensions to:

```python
points = [
    [
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
    ]
    for pose in msg.poses
]
```

- [ ] **Step 4: Implement the browser conversion helper**

Insert before `updatePaths`:

```javascript
function pathPointToThree(point) {
  const z = point.length > 2 && Number.isFinite(point[2]) ? point[2] : 0;
  return rosToThree(point[0], point[1], z + 0.08);
}
```

Use it in both renderers:

```javascript
const points = path.map((point) => pathPointToThree(point));
```

```javascript
const points = state.mappingPath.map((point) => pathPointToThree(point));
```

- [ ] **Step 5: Run focused and full verification**

Run:

```bash
python3 -m pytest -q test/test_core.py -k "path_callbacks_preserve_z or frontend_path_height"
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY python3 -m pytest -q test
node --check static/app.js
python3 -m py_compile web_control/*.py
git diff --check
```

Expected: every command exits zero and pytest reports no failures.

- [ ] **Step 6: Commit**

```bash
git add web_control/web_control_node.py static/app.js test/test_core.py
git commit -m "fix: render paths at their ROS height"
```
