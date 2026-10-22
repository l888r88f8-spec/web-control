"""Regression tests for the next-start localization loading-mode preference."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import threading
from pathlib import Path

import pytest

from test_core import (
    _make_map_dir,
    _selection_node,
    _serve_and_request,
    _tiled_manifest_document,
    _write_tile_files,
    _write_tiled_manifest,
)
from web_control import web_control_node
from web_control.map_selection import SelectionStore


def _tiled_choice(root: Path, name: str = "site_tiled") -> tuple[Path, Path]:
    map_dir = _make_map_dir(root, name)
    (map_dir / "sc_db.bin").write_bytes(b"scan context")
    tiled_dir = _write_tiled_manifest(map_dir, _tiled_manifest_document())
    _write_tile_files(tiled_dir)
    return map_dir, tiled_dir


def _localization_map_arg(node) -> str:
    command = node.managed_start_command("localization")
    values = [arg for arg in command if arg.startswith("map_path:=")]
    assert len(values) == 1
    return values[0].split(":=", 1)[1]


def _navigation_map_arg(node) -> str:
    command = node.managed_start_command("navigation")
    values = [arg for arg in command if arg.startswith("map_path:=")]
    assert len(values) == 1
    return values[0].split(":=", 1)[1]


def test_selection_store_mode_contract_preserves_unknown_and_old_name_only(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps(
            {
                "name": "site_a",
                "mode": "tiled",
                "ui": {"theme": "dark"},
                "extra": {"keep": True},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    store = SelectionStore(path)

    store.save("site_b")

    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload == {
        "name": "site_b",
        "mode": "tiled",
        "ui": {"theme": "dark"},
        "extra": {"keep": True},
    }
    assert store.load_mode() == "tiled"

    store.save("site_c", "global")
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["name"] == "site_c"
    assert payload["mode"] == "global"
    assert payload["ui"] == {"theme": "dark"}
    assert payload["extra"] == {"keep": True}


@pytest.mark.parametrize("mode", [None, [], {}, True, False, 1, "AUTO", "nope"])
def test_selection_store_sanitizes_unknown_persisted_mode_to_auto(tmp_path, mode):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps({"name": "site_a", "mode": mode}) + "\n", encoding="utf-8"
    )

    assert SelectionStore(path).load_mode() == "auto"
    assert SelectionStore(path).load() == "site_a"


def test_selection_store_rejects_explicit_invalid_mode_without_writing(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text('{"name": "site_a", "mode": "tiled"}\n', encoding="utf-8")
    before = path.read_text(encoding="utf-8")
    store = SelectionStore(path)

    for mode in ([], {}, True, 1, "AUTO", "nope"):
        with pytest.raises(ValueError):
            store.save("site_b", mode)
        assert path.read_text(encoding="utf-8") == before


def test_modes_force_both_localization_paths_from_the_same_tiled_map(tmp_path):
    root = tmp_path / "map"
    map_dir, tiled_dir = _tiled_choice(root)
    node = _selection_node(tmp_path)

    node.map_selection.save("site_tiled", "auto")
    assert _localization_map_arg(node) == str(tiled_dir.resolve())

    node.map_selection.save("site_tiled", "global")
    assert _localization_map_arg(node) == str((map_dir / "GlobalMap.pcd").resolve())

    node.map_selection.save("site_tiled", "tiled")
    assert _localization_map_arg(node) == str(tiled_dir.resolve())


def test_get_reports_selected_and_effective_mode_without_paths(tmp_path):
    root = tmp_path / "map"
    _map_dir, tiled_dir = _tiled_choice(root)
    node = _selection_node(tmp_path, selection="site_tiled")
    node.map_selection.save("site_tiled", "auto")

    status, body = _serve_and_request(node, "/api/maps")

    assert status == 200
    selection = body["selection"]
    assert selection["mode"] == "auto"
    assert selection["effective_mode"] == "tiled"
    assert selection["mode_valid"] is True
    assert selection["mode_message"] is None
    assert selection["entry"] == {
        "name": "site_tiled",
        "tiled": True,
        "has_scan_context": True,
        "warning": None,
    }
    serialized = json.dumps(body)
    assert str(tiled_dir.resolve()) not in serialized
    assert str((root / "site_tiled" / "GlobalMap.pcd").resolve()) not in serialized


def test_post_global_mode_uses_global_even_with_valid_tiled_sibling(tmp_path):
    root = tmp_path / "map"
    map_dir, _tiled_dir = _tiled_choice(root)
    node = _selection_node(tmp_path, selection="site_tiled")

    status, body = _serve_and_request(
        node,
        "/api/maps/select",
        {"name": "site_tiled", "mode": "global"},
        "POST",
    )

    assert status == 200
    assert body["selection"]["mode"] == "global"
    assert body["selection"]["effective_mode"] == "global"
    assert node.map_selection.load() == "site_tiled"
    assert node.map_selection.load_mode() == "global"
    assert _localization_map_arg(node) == str((map_dir / "GlobalMap.pcd").resolve())


def test_name_only_selection_keeps_existing_mode(tmp_path):
    root = tmp_path / "map"
    _map_dir, tiled_dir = _tiled_choice(root)
    _make_map_dir(root, "site_other")
    node = _selection_node(tmp_path, selection="site_tiled")
    node.map_selection.save("site_tiled", "global")

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_other"}, "POST"
    )

    assert status == 200
    assert body["selection"]["mode"] == "global"
    assert body["selection"]["effective_mode"] == "global"
    assert node.map_selection.load() == "site_other"
    assert node.map_selection.load_mode() == "global"
    assert _localization_map_arg(node) != str(tiled_dir.resolve())


@pytest.mark.parametrize("mode", [None, [], {}, True, False, 1, "AUTO", "nope"])
def test_post_invalid_mode_is_400_and_preserves_file(tmp_path, mode):
    root = tmp_path / "map"
    _tiled_choice(root)
    node = _selection_node(tmp_path, selection="site_tiled")
    node.map_selection.save("site_tiled", "tiled")
    before = node.map_selection.path.read_text(encoding="utf-8")

    status, body = _serve_and_request(
        node,
        "/api/maps/select",
        {"name": "site_tiled", "mode": mode},
        "POST",
    )

    assert status == 400
    assert body["ok"] is False
    assert node.map_selection.path.read_text(encoding="utf-8") == before


def test_manual_tiled_missing_or_broken_is_explicit_and_never_written(tmp_path):
    root = tmp_path / "map"
    plain_dir = _make_map_dir(root, "site_plain")
    broken_dir = _make_map_dir(root, "site_broken")
    (broken_dir / "tiled").mkdir()
    (broken_dir / "tiled" / "manifest.yaml").write_text(
        "schema_version: 2\ntiles: {}\n", encoding="utf-8"
    )
    node = _selection_node(tmp_path, selection="site_plain")

    for name, detail in (("site_plain", "no usable tiled"), ("site_broken", "no usable tiled")):
        node.map_selection.save(name, "global")
        before = node.map_selection.path.read_text(encoding="utf-8")
        status, body = _serve_and_request(
            node, "/api/maps/select", {"name": name, "mode": "tiled"}, "POST"
        )
        assert status == 400
        assert detail in body["message"]
        assert node.map_selection.path.read_text(encoding="utf-8") == before


def test_auto_falls_back_global_when_tiled_missing(tmp_path):
    root = tmp_path / "map"
    map_dir = _make_map_dir(root, "site_plain")
    node = _selection_node(tmp_path, selection="site_plain")
    node.map_selection.save("site_plain", "auto")

    status, body = _serve_and_request(node, "/api/maps")

    assert status == 200
    assert body["selection"]["mode"] == "auto"
    assert body["selection"]["effective_mode"] == "global"
    assert body["selection"]["mode_valid"] is True
    assert _localization_map_arg(node) == str((map_dir / "GlobalMap.pcd").resolve())


def test_stale_manual_tiled_after_manifest_break_blocks_start_but_allows_recovery(
    tmp_path,
):
    root = tmp_path / "map"
    map_dir, tiled_dir = _tiled_choice(root)
    node = _selection_node(
        tmp_path,
        selection="site_tiled",
        localization_launch_args="use_rviz:=false",
        navigation_launch_args="use_rviz:=false",
    )
    node.map_selection.save("site_tiled", "tiled")
    (tiled_dir / "manifest.yaml").write_text("schema_version: 2\n", encoding="utf-8")

    status, body = _serve_and_request(node, "/api/maps")

    assert status == 200
    selection = body["selection"]
    assert selection["valid"] is True
    assert selection["mode"] == "tiled"
    assert selection["mode_valid"] is False
    assert selection["effective_mode"] is None
    assert "tiled" in selection["mode_message"]
    with pytest.raises(web_control_node.MapSelectionError, match="tiled"):
        node.managed_start_command("localization")
    # Navigation only needs the valid map and keeps its GlobalMap parent root.
    assert _navigation_map_arg(node) == str((map_dir / "GlobalMap.pcd").resolve().parent)

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_tiled", "mode": "auto"}, "POST"
    )
    assert status == 200
    assert body["selection"]["mode"] == "auto"
    assert body["selection"]["effective_mode"] == "global"
    assert _localization_map_arg(node) == str((map_dir / "GlobalMap.pcd").resolve())


def test_tiles_removed_after_save_are_rechecked_at_start(tmp_path):
    root = tmp_path / "map"
    map_dir, tiled_dir = _tiled_choice(root)
    node = _selection_node(tmp_path, selection="site_tiled")
    node.map_selection.save("site_tiled", "tiled")
    shutil.rmtree(tiled_dir)

    with pytest.raises(web_control_node.MapSelectionError, match="tiled"):
        node.managed_start_command("localization")

    node.map_selection.save("site_tiled", "auto")
    assert _localization_map_arg(node) == str((map_dir / "GlobalMap.pcd").resolve())


def test_name_switch_with_manual_tiled_unsupported_fails_and_resyncs(tmp_path):
    root = tmp_path / "map"
    _map_dir, _tiled_dir = _tiled_choice(root, "site_tiled")
    _make_map_dir(root, "site_plain")
    node = _selection_node(tmp_path, selection="site_tiled")
    node.map_selection.save("site_tiled", "tiled")
    before = node.map_selection.path.read_text(encoding="utf-8")

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_plain"}, "POST"
    )

    assert status == 400
    assert "tiled" in body["message"]
    assert node.map_selection.path.read_text(encoding="utf-8") == before

    status, body = _serve_and_request(node, "/api/maps")
    assert status == 200
    assert body["selection"]["name"] == "site_tiled"
    assert body["selection"]["mode"] == "tiled"


def test_mode_only_change_persists_across_store_and_page_restart(tmp_path):
    root = tmp_path / "map"
    _tiled_choice(root)
    node = _selection_node(tmp_path, selection="site_tiled")

    status, body = _serve_and_request(
        node, "/api/maps/select", {"mode": "global"}, "POST"
    )

    assert status == 200
    assert body["selection"]["name"] == "site_tiled"
    assert body["selection"]["mode"] == "global"
    assert node.map_selection.load() == "site_tiled"
    assert node.map_selection.load_mode() == "global"

    restarted = SelectionStore(node.map_selection.path)
    assert restarted.load() == "site_tiled"
    assert restarted.load_mode() == "global"

    status, body = _serve_and_request(node, "/api/maps")
    assert status == 200
    assert body["selection"]["mode"] == "global"
    assert body["selection"]["effective_mode"] == "global"


def test_mode_only_requires_a_selected_map(tmp_path):
    root = tmp_path / "map"
    _tiled_choice(root)
    node = _selection_node(tmp_path)

    status, body = _serve_and_request(
        node, "/api/maps/select", {"mode": "global"}, "POST"
    )

    assert status == 400
    assert "no map selected" in body["message"]
    assert not node.map_selection.path.exists()


@pytest.mark.parametrize("target", ["localization", "navigation"])
def test_mode_change_409_while_localization_or_navigation_runs(tmp_path, target):
    root = tmp_path / "map"
    _tiled_choice(root)
    node = _selection_node(tmp_path, selection="site_tiled", running=(target,))
    node.map_selection.save("site_tiled", "auto")
    before = node.map_selection.path.read_text(encoding="utf-8")

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_tiled", "mode": "global"}, "POST"
    )

    assert status == 409
    assert target in body["message"]
    assert "stop" in body["message"].lower()
    assert node.map_selection.path.read_text(encoding="utf-8") == before


def test_mode_change_allowed_while_only_mapping_runs(tmp_path):
    root = tmp_path / "map"
    _tiled_choice(root)
    node = _selection_node(tmp_path, selection="site_tiled", running=("mapping",))

    status, _body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_tiled", "mode": "global"}, "POST"
    )

    assert status == 200
    assert node.map_selection.load_mode() == "global"


def test_forced_global_and_auto_relocalization_do_not_change_nav_root(tmp_path):
    root = tmp_path / "map"
    map_dir, _tiled_dir = _tiled_choice(root)
    node = _selection_node(
        tmp_path,
        selection="site_tiled",
        localization_launch_args="use_rviz:=false auto_relocalization:=true",
        navigation_launch_args="use_rviz:=false",
    )
    (map_dir / "sc_db.bin").unlink()
    node.map_selection.save("site_tiled", "global")

    localization = node.managed_start_command("localization")
    assert _localization_map_arg(node) == str((map_dir / "GlobalMap.pcd").resolve())
    assert "auto_relocalization:=false" in localization
    assert "auto_relocalization:=true" not in localization
    # No unsupported map_mode launch argument is invented.
    assert not any(arg.startswith("map_mode:=") for arg in localization)

    assert _navigation_map_arg(node) == str((map_dir / "GlobalMap.pcd").resolve().parent)


class _ObservedLock:
    """RLock proxy that reports when the watched thread has to wait."""

    def __init__(self, watch_name: str) -> None:
        self._lock = threading.RLock()
        self._watch_name = watch_name
        self._owner: threading.Thread | None = None
        self._depth = 0
        self.attempted = threading.Event()
        self.blocked = threading.Event()

    def __enter__(self):
        current = threading.current_thread()
        if current.name == self._watch_name:
            self.attempted.set()
            if self._owner is not None and self._owner is not current:
                self.blocked.set()
        self._lock.acquire()
        self._owner = current
        self._depth += 1
        return self

    def __exit__(self, exc_type, exc, traceback):
        self._depth -= 1
        if self._depth == 0:
            self._owner = None
        self._lock.release()


def _start_and_release(lock: _ObservedLock, release: threading.Event, saved: threading.Event):
    """Release the paused name reader only after the competing writer runs."""
    assert lock.attempted.wait(5), "competing selection never reached the node lock"
    if lock.blocked.is_set():
        # The first request holds the node lock, so the competing request waits
        # for the transaction to finish before it can validate and save.
        release.set()
    else:
        # Without the lock the competing request has already saved; resume the
        # paused reader so the old interleaving is deterministic.
        assert saved.wait(5), "competing selection did not reach its save"
        release.set()


def test_concurrent_name_and_mode_selections_are_one_transaction(tmp_path):
    root = tmp_path / "map"
    _tiled_choice(root, "tiled_map")
    _make_map_dir(root, "global_only")
    node = _selection_node(tmp_path, selection="tiled_map")
    path = node.map_selection.path
    path.write_text(
        json.dumps(
            {
                "name": "tiled_map",
                "mode": "auto",
                "ui": {"theme": "dark"},
                "extra": {"keep": True},
            }
        )
        + "\n",
        encoding="utf-8",
    )

    name_read = threading.Event()
    release_name = threading.Event()
    mode_saved = threading.Event()
    original_load_mode = node.map_selection.load_mode
    original_save = node.map_selection.save

    def paused_load_mode():
        value = original_load_mode()
        if threading.current_thread().name == "test-name-only":
            name_read.set()
            assert release_name.wait(5), "name-only selection did not resume"
        return value

    def tracked_save(name, mode=None):
        original_save(name, mode)
        if threading.current_thread().name == "test-mode-only":
            mode_saved.set()

    node.map_selection.load_mode = paused_load_mode
    node.map_selection.save = tracked_save
    node_lock = _ObservedLock("test-mode-only")
    node.lock = node_lock
    outcome = {}

    def select_name_only():
        try:
            outcome["name_only"] = node.select_map({"name": "global_only"})
        except BaseException as exc:  # noqa: BLE001 - asserted below
            outcome["name_only_error"] = exc

    def select_mode_only():
        try:
            outcome["mode_only"] = node.select_map({"mode": "tiled"})
        except BaseException as exc:  # noqa: BLE001 - asserted below
            outcome["mode_only_error"] = exc

    name_thread = threading.Thread(target=select_name_only, name="test-name-only")
    mode_thread = threading.Thread(target=select_mode_only, name="test-mode-only")
    name_thread.start()
    assert name_read.wait(5), "name-only selection did not read the saved mode"
    mode_thread.start()
    _start_and_release(node_lock, release_name, mode_saved)
    name_thread.join(5)
    mode_thread.join(5)

    assert not name_thread.is_alive() and not mode_thread.is_alive()
    assert "name_only_error" not in outcome
    assert outcome["name_only"]["selection"]["name"] == "global_only"
    # global_only has no usable tiled map, so the competing tiled preference is
    # rejected instead of being combined with the new name.
    assert "mode_only_error" in outcome
    assert "tiled" in str(outcome["mode_only_error"])
    persisted = json.loads(path.read_text(encoding="utf-8"))
    assert persisted == {
        "name": "global_only",
        "mode": "auto",
        "ui": {"theme": "dark"},
        "extra": {"keep": True},
    }
    panel = node.maps_payload()["selection"]
    assert panel["name"] == "global_only"
    assert panel["mode"] == "auto"
    assert panel["mode_valid"] is True


def test_maps_payload_reads_name_and_mode_from_one_transaction(tmp_path):
    root = tmp_path / "map"
    _tiled_choice(root, "tiled_map")
    _make_map_dir(root, "global_only")
    node = _selection_node(tmp_path, selection="tiled_map")
    node.map_selection.save("tiled_map", "auto")

    name_read = threading.Event()
    release_name = threading.Event()
    selection_saved = threading.Event()
    original_load = node.map_selection.load
    original_save = node.map_selection.save

    def paused_load():
        value = original_load()
        if threading.current_thread().name == "test-maps-payload":
            name_read.set()
            assert release_name.wait(5), "maps payload did not resume"
        return value

    def tracked_save(name, mode=None):
        original_save(name, mode)
        if threading.current_thread().name == "test-map-select":
            selection_saved.set()

    node.map_selection.load = paused_load
    node.map_selection.save = tracked_save
    node_lock = _ObservedLock("test-map-select")
    node.lock = node_lock
    outcome = {}

    def read_payload():
        outcome["payload"] = node.maps_payload()

    def select_other_map():
        outcome["selection"] = node.select_map(
            {"name": "global_only", "mode": "global"}
        )

    payload_thread = threading.Thread(target=read_payload, name="test-maps-payload")
    select_thread = threading.Thread(target=select_other_map, name="test-map-select")
    payload_thread.start()
    assert name_read.wait(5), "maps payload did not read the saved name"
    select_thread.start()
    _start_and_release(node_lock, release_name, selection_saved)
    payload_thread.join(5)
    select_thread.join(5)

    assert not payload_thread.is_alive() and not select_thread.is_alive()
    selection = outcome["payload"]["selection"]
    assert (selection["name"], selection["mode"]) == ("tiled_map", "auto")
    assert (selection["effective_mode"], selection["mode_valid"]) == ("tiled", True)
    assert outcome["selection"]["selection"]["name"] == "global_only"
    assert node.map_selection.load() == "global_only"
    assert node.map_selection.load_mode() == "global"


_MAP_MODE_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("// --- Map selector");
const end = source.indexOf("// Per-target cursor");
if (start < 0 || end < 0) throw new Error("map selector block not found");
const block = source.slice(start, end);

function makeElement() {
  return {
    textContent: "", className: "", innerHTML: "", value: "",
    disabled: false, hidden: false, children: [],
    appendChild(child) { this.children.push(child); },
  };
}
const elements = {};
for (const id of [
  "mapSelect", "mapMode", "mapWarning", "mapStatus", "mapLockNote",
]) {
  elements[id] = makeElement();
}
global.document = {
  getElementById(id) { return elements[id] || null; },
  createElement() { return makeElement(); },
};

let state = {
  localizationRunning: false, navigationRunning: false,
  mapSelectPending: false, mapCatalog: null, mapError: null,
  mapSelectionError: null, mapSelectionNotice: null,
};
let serverCatalog = null;
let captured = null;
let failPost = null;
let deferNextGet = false;
let deferNextPost = false;
const deferredGets = [];
const deferredPosts = [];
const clone = (value) => JSON.parse(JSON.stringify(value));
function postSuccess(payload) {
  const selection = { ...state.mapCatalog.selection, ...payload, mode_valid: true };
  if (payload.mode === "global") selection.effective_mode = "global";
  if (payload.mode === "tiled") selection.effective_mode = "tiled";
  return { ok: true, selection };
}
global.getJson = async () => {
  if (deferNextGet) {
    deferNextGet = false;
    const snapshot = clone(serverCatalog);
    return new Promise((resolve, reject) => {
      deferredGets.push({ resolve: () => resolve(snapshot), reject });
    });
  }
  return clone(serverCatalog);
};
function invalidateNavigationMapCaches() {}
const refreshController = { tick() {} };
global.postJson = async (url, payload) => {
  captured = { url, payload };
  if (deferNextPost) {
    deferNextPost = false;
    return new Promise((resolve, reject) => {
      deferredPosts.push({
        resolve: () => resolve(postSuccess(payload)),
        reject,
      });
    });
  }
  if (failPost) throw new Error(failPost);
  return postSuccess(payload);
};

eval(block);

function expect(condition, message) {
  if (!condition) throw new Error(message);
}

function deferredGet() {
  deferNextGet = true;
  const promise = refreshMaps();
  const pending = deferredGets.shift();
  if (!pending) throw new Error("deferred catalog GET did not start");
  return { promise, pending };
}

function catalog(selection) {
  return {
    ok: true, map_root: "~/map",
    maps: [{ name: "site_a", tiled: true, has_scan_context: true, warning: null }],
    selection,
  };
}

(async () => {
  serverCatalog = catalog({
    name: "site_a", valid: true,
    entry: { name: "site_a", tiled: true, has_scan_context: true, warning: null },
    message: null, mode: "auto", effective_mode: "tiled",
    mode_valid: true, mode_message: null,
  });
  state.mapCatalog = serverCatalog;
  updateMapPanel();
  expect(elements.mapMode.value === "auto", "saved mode is not reflected");
  expect(elements.mapStatus.hidden && elements.mapStatus.textContent === "", "normal mode status should be hidden");
  expect(!elements.mapMode.disabled, "mode must be editable with a valid selection");

  state.localizationRunning = true;
  updateMapSelectionLock();
  expect(elements.mapSelect.disabled && elements.mapMode.disabled, "running localization must lock both controls");
  expect(/localization/.test(elements.mapLockNote.textContent) && /loading mode/.test(elements.mapLockNote.textContent), "lock note must explain stopping before a mode change");
  state.localizationRunning = false;
  updateMapSelectionLock();

  await selectMapMode("global");
  expect(captured.url === "/api/maps/select", "mode change did not use the select endpoint");
  expect(captured.payload.name === "site_a" && captured.payload.mode === "global", "mode payload is wrong");
  expect(state.mapSelectPending === false, "pending state was not cleared");
  expect(elements.mapMode.value === "global", "successful mode was not reflected");
  expect(elements.mapStatus.hidden && elements.mapStatus.textContent === "", "successful mode change should leave no persistent notice");

  // A rejected change restores the server's saved selection/mode and unlocks.
  serverCatalog = catalog({
    name: "site_a", valid: true,
    entry: { name: "site_a", tiled: true, has_scan_context: true, warning: null },
    message: null, mode: "auto", effective_mode: "tiled",
    mode_valid: true, mode_message: null,
  });
  state.mapCatalog = JSON.parse(JSON.stringify(serverCatalog));
  updateMapPanel();
  failPost = "mode change rejected";
  await selectMapMode("tiled");
  expect(elements.mapMode.value === "auto", "failed mode change was not restored from server state");
  expect(/Map selection failed: mode change rejected/.test(elements.mapStatus.textContent), "failure was not surfaced");
  expect(!elements.mapMode.disabled && !elements.mapSelect.disabled, "failed change must leave recovery controls enabled");
  expect(state.mapSelectPending === false, "failed change left the panel pending");

  // A stale saved manual tiled mode stays recoverable, not disabled.
  failPost = null;
  state.mapSelectionError = null;
  state.mapSelectionNotice = null;
  serverCatalog = catalog({
    name: "site_a", valid: true,
    entry: { name: "site_a", tiled: false, has_scan_context: true, warning: "tiled manifest is invalid" },
    message: null, mode: "tiled", effective_mode: null,
    mode_valid: false, mode_message: "no usable tiled map; choose Auto or Global",
  });
  state.mapCatalog = JSON.parse(JSON.stringify(serverCatalog));
  updateMapPanel();
  expect(!elements.mapMode.disabled, "invalid manual tiled must not disable mode recovery");
  expect(/unavailable loading mode/.test(elements.mapStatus.textContent), "mode error missing from map status");
  expect(!elements.mapStatus.hidden, "mode error must be visible");

  // No selection disables mode changes gracefully.
  serverCatalog = catalog({
    name: null, valid: false, entry: null, message: "no map selected",
    mode: "auto", effective_mode: null, mode_valid: false,
    mode_message: "no map selected",
  });
  state.mapCatalog = JSON.parse(JSON.stringify(serverCatalog));
  updateMapPanel();
  expect(elements.mapMode.disabled, "mode must be disabled without a selected map");
  expect(elements.mapStatus.hidden && elements.mapStatus.textContent === "", "no-selection default should not be displayed");

  // --- Catalog refresh races -------------------------------------------
  failPost = null;
  state.mapSelectionError = null;
  state.mapSelectionNotice = null;
  state.mapSelectPending = false;
  state.mapError = null;
  const validEntry = { name: "site_a", tiled: true, has_scan_context: true, warning: null };
  const autoSelection = {
    name: "site_a", valid: true, entry: validEntry, message: null,
    mode: "auto", effective_mode: "tiled", mode_valid: true, mode_message: null,
  };
  const globalSelection = {
    ...autoSelection, mode: "global", effective_mode: "global",
  };
  serverCatalog = catalog(autoSelection);
  state.mapCatalog = clone(serverCatalog);
  updateMapPanel();

  // A GET started before a mode change must not apply after the POST succeeds.
  const beforePost = deferredGet();
  await selectMapMode("global");
  beforePost.pending.resolve();
  await beforePost.promise;
  expect(state.mapCatalog.selection.mode === "global", "GET started before a POST overwrote the new saved mode");
  expect(elements.mapMode.value === "global", "GET started before a POST reverted the mode control");

  serverCatalog = catalog(autoSelection);
  state.mapCatalog = clone(serverCatalog);
  state.mapSelectionError = null;
  state.mapSelectionNotice = null;
  state.mapSelectPending = false;
  updateMapPanel();

  // A GET issued while a mode POST is pending carries the old saved state.  Its
  // late response must not undo the successful POST.
  deferNextPost = true;
  const pendingMode = selectMapMode("global");
  const pendingPost = deferredPosts.shift();
  expect(pendingPost && state.mapSelectPending, "mode POST did not remain pending");
  const staleGet = deferredGet();
  pendingPost.resolve();
  await pendingMode;
  expect(state.mapCatalog.selection.mode === "global", "successful mode POST was not applied");
  staleGet.pending.resolve();
  await staleGet.promise;
  expect(state.mapCatalog.selection.mode === "global", "stale GET issued during a mutation overwrote the new saved mode");
  expect(elements.mapMode.value === "global", "stale GET reverted the mode control");

  // An older failed GET must not clear the catalog installed by a newer GET.
  serverCatalog = catalog(autoSelection);
  const oldFailure = deferredGet();
  serverCatalog = catalog(globalSelection);
  await refreshMaps();
  expect(state.mapCatalog && state.mapCatalog.selection.mode === "global", "newer catalog was not installed");
  oldFailure.pending.reject(new Error("stale catalog GET failed"));
  await oldFailure.promise;
  expect(state.mapCatalog && state.mapCatalog.selection.mode === "global", "stale GET failure cleared the newer catalog");
  expect(state.mapError === null, "stale GET failure set a catalog error");

  console.log("map mode behavior ok");
  console.log("map refresh race behavior ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_frontend_map_mode_posts_restores_locks_and_resyncs_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _MAP_MODE_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "map mode behavior ok" in completed.stdout
    assert "map refresh race behavior ok" in completed.stdout
