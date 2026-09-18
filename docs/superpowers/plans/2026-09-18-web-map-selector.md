# Web Map Selector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Let the web operator select one `~/map/<name>` directory for the next web-managed localization and navigation starts, and remember the selection.

**Architecture:** A pure map catalog validates and describes direct children of `~/map`; a small state store persists only the chosen name. The web node exposes catalog/select APIs and injects paths into each launch immediately before starting. The browser renders the catalog and blocks selection while either affected process runs.

**Tech Stack:** ROS 2 Python (`rclpy`, `ament_index_python`), PyYAML, existing stdlib HTTP server, vanilla JavaScript.

---

### Task 1: Catalog and path validation

**Files:** Create `web_control/map_catalog.py`; modify `test/test_core.py`.

- [x] Write failing tests using `tmp_path` for a valid map with `GlobalMap.pcd` and `GroundMap.pcd`, an incomplete map, a direct-child symlink escaping the root, names `.`/`..`/`a/b`, and a malformed tiled manifest. Test sorted names and `sc_db.bin` capability.
- [x] Run `python3 -m pytest -q test/test_core.py -k map_catalog` and confirm the new tests fail because the module is missing.
- [x] Implement `MapCatalog(root: Path).list_maps() -> list[dict]` and `.get(name: str) -> dict`; reject arbitrary paths, resolve the root/child/files to prevent symlink escape, and require both PCDs to be readable regular files. Return `name`, `global_map_path`, `ground_map_path`, `tiled_path` (or `None`), `has_scan_context`, and `warning`. A tiled manifest is valid only when YAML parses to schema version 1 and each referenced tile file is a regular file within `tiled/`; otherwise keep the global map candidate with a warning.
- [x] Re-run the focused tests and `python3 -m pytest -q test/test_core.py`.

### Task 2: Persistent selection and localization parameter file

**Files:** Create `web_control/map_selection.py`; modify `package.xml`, `test/test_core.py`.

- [x] Write failing tests for `SelectionStore(path).load()/save(name)`: empty/malformed file, atomic replacement, reload, and a selected map deleted after persistence.
- [x] Run focused tests to confirm red.
- [x] Implement atomic JSON selection at `~/.config/web_control/selected_map.json` with `os.replace`. Store only the map directory name; validate against `MapCatalog` on each start. Keep `<exec_depend>python3-yaml</exec_depend>` for the tiled manifest parsing in `map_catalog.py`.
- [x] Re-run focused and full Python tests.

### Task 3: Web node API and launch integration

**Files:** Modify `web_control/web_control_node.py`, `test/test_core.py`.

- [x] Write failing tests for `GET /api/maps`, `POST /api/maps/select`, 400 invalid names, 409 while localization/navigation runs, stale selection after map deletion, and command arguments after selecting a tiled or global map. Exercise both localization and navigation starts; assert they use the same candidate.
- [x] Run focused tests to confirm red.
- [x] Construct catalog/store from `Path.home()` during web node initialization. Add the two HTTP routes. If no valid selection, reject localization/navigation Start with a readable error; leave Mapping Start untouched. On selection, check both managed processes under the existing process locks, then persist and return selected state. Revalidate on every Start.
- [x] At localization Start, pass `map_path:=<selected tiled directory or GlobalMap.pcd>` to `localization_with_nonground.launch.py` and `auto_relocalization:=false` when `sc_db.bin` is absent. At navigation Start, pass `map_dir:=<selected>/GlobalMap.pcd` and `ground_dir:=<selected>/GroundMap.pcd`. Build argument arrays without a shell, retain other configured args, and prevent an extra configured map argument from overriding the selection.
- [x] Re-run focused and full Python tests.

### Task 4: Browser control and documentation

**Files:** Modify `static/index.html`, `static/app.js`, `static/styles.css` (if needed), `README.md`, and meaningful browser-facing tests in `test/test_core.py` where feasible.

- [x] Add a selector next to localization/navigation controls, a refresh action, selected/invalid state, tiled and automatic-relocalization labels, and “Manual initial pose required” for maps without `sc_db.bin`.
- [x] Fetch `GET /api/maps` on load/refresh and after Save Map succeeds; send `{name}` to `POST /api/maps/select`. Disable selection while affected processes run and show the server's conflict/error message. The server remains authoritative if the UI state is stale.
- [x] Update README with map directory requirements, persistence path, restart semantics, manual initial pose, and the fact that direct CLI launches keep their own defaults.
- [x] Run `node --check static/app.js`, `python3 -m pytest -q test/test_core.py`, `python3 -m py_compile web_control/*.py`, and `git diff --check`. Do not run `colcon build`; the user compiles.

### Task 5: Review

- [x] Review the diff for unrelated changes and map-path escape routes. Verify the only `slam_ws` change is the `localization_with_nonground.launch.py` map/auto_relocalization arguments and that `navigation_ws` is untouched.
- [x] Confirm both launch commands contain the chosen map's paths and cannot silently fall back to their old defaults. Report test evidence and remaining runtime verification the user must perform after compiling.
