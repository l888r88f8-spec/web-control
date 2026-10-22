"""Focused tests for persistent AUTOMATIC RELOCALIZATION control."""

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
from web_control.map_selection import (
    DEFAULT_AUTO_RELOCALIZATION,
    SelectionStore,
    saved_auto_relocalization,
    validate_auto_relocalization,
)


def _capable_map(root: Path, name: str = "site_a") -> Path:
    map_dir = _make_map_dir(root, name)
    (map_dir / "sc_db.bin").write_bytes(b"scan context")
    return map_dir


def test_auto_relocalization_bool_schema_and_saved_defaults(tmp_path):
    path = tmp_path / "selected_map.json"
    store = SelectionStore(path)
    assert DEFAULT_AUTO_RELOCALIZATION is True
    assert store.load_auto_relocalization() is True

    for value in (None, 0, 1, 0.0, "true", "false", [], {}):
        with pytest.raises(ValueError):
            validate_auto_relocalization(value)
        assert saved_auto_relocalization(value) is True

    assert validate_auto_relocalization(False) is False
    path.write_text(
        json.dumps({"name": "site_a", "auto_relocalization": "true"}) + "\n",
        encoding="utf-8",
    )
    assert store.load_auto_relocalization() is True
    path.write_text(
        json.dumps({"name": "site_a", "auto_relocalization": False}) + "\n",
        encoding="utf-8",
    )
    assert store.load_auto_relocalization() is False


def test_flag_update_preserves_name_mode_ui_and_unknown_keys(tmp_path):
    path = tmp_path / "selected_map.json"
    original = {
        "name": "site_a",
        "mode": "tiled",
        "auto_relocalization": True,
        "ui": {
            "joystick": {"throttle": {"size": 180, "x": None, "y": None}},
            "topic_logging_enabled": False,
            "theme": "dark",
        },
        "extra": {"keep": [1, 2, 3]},
    }
    path.write_text(json.dumps(original) + "\n", encoding="utf-8")
    store = SelectionStore(path)

    assert store.save_auto_relocalization(False) is False

    assert json.loads(path.read_text(encoding="utf-8")) == {
        **original,
        "auto_relocalization": False,
    }
    with pytest.raises(ValueError):
        store.save_auto_relocalization(0)
    assert json.loads(path.read_text(encoding="utf-8"))["auto_relocalization"] is False

    store.save("site_b", "global", True)
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["name"] == "site_b"
    assert payload["mode"] == "global"
    assert payload["auto_relocalization"] is True
    assert payload["ui"] == original["ui"]
    assert payload["extra"] == original["extra"]


def test_get_reports_requested_available_and_effective_state(tmp_path):
    root = tmp_path / "map"
    _capable_map(root, "site_capable")
    _make_map_dir(root, "site_plain")
    node = _selection_node(tmp_path, selection="site_capable")

    status, body = _serve_and_request(node, "/api/maps")
    assert status == 200
    selection = body["selection"]
    assert selection["auto_relocalization"] is True
    assert selection["auto_relocalization_available"] is True
    assert selection["effective_auto_relocalization"] is True
    assert selection["auto_relocalization_message"] is None

    node.map_selection.save("site_plain", "auto")
    status, body = _serve_and_request(node, "/api/maps")
    assert status == 200
    selection = body["selection"]
    assert selection["valid"] is True
    assert selection["auto_relocalization"] is True
    assert selection["auto_relocalization_available"] is False
    assert selection["effective_auto_relocalization"] is False
    assert "sc_db.bin" in selection["auto_relocalization_message"]


def test_flag_only_post_updates_without_rewriting_name_or_mode(tmp_path):
    root = tmp_path / "map"
    _capable_map(root)
    node = _selection_node(tmp_path, selection="site_a")
    node.map_selection.save("site_a", "global", False)
    node.map_selection.path.write_text(
        json.dumps(
            {
                "name": "site_a",
                "mode": "global",
                "auto_relocalization": False,
                "ui": {"topic_logging_enabled": False, "theme": "dark"},
                "extra": 7,
            }
        )
        + "\n",
        encoding="utf-8",
    )

    status, body = _serve_and_request(
        node, "/api/maps/select", {"auto_relocalization": True}, "POST"
    )

    assert status == 200
    assert body["selection"]["auto_relocalization"] is True
    assert body["selection"]["effective_auto_relocalization"] is True
    assert body["selection"]["mode"] == "global"
    assert json.loads(node.map_selection.path.read_text(encoding="utf-8")) == {
        "name": "site_a",
        "mode": "global",
        "auto_relocalization": True,
        "ui": {"topic_logging_enabled": False, "theme": "dark"},
        "extra": 7,
    }


@pytest.mark.parametrize("value", [1, 0, "false", None, [], {}])
def test_post_rejects_non_boolean_auto_flag_without_writing(tmp_path, value):
    root = tmp_path / "map"
    _capable_map(root)
    node = _selection_node(tmp_path, selection="site_a")
    node.map_selection.save("site_a", "auto", False)
    before = node.map_selection.path.read_text(encoding="utf-8")

    status, body = _serve_and_request(
        node,
        "/api/maps/select",
        {"auto_relocalization": value},
        "POST",
    )

    assert status == 400
    assert body["ok"] is False
    assert node.map_selection.path.read_text(encoding="utf-8") == before


def test_explicit_enable_without_database_is_400_but_disable_is_allowed(tmp_path):
    root = tmp_path / "map"
    _make_map_dir(root, "site_plain")
    node = _selection_node(tmp_path, selection="site_plain")
    node.map_selection.save("site_plain", "auto", False)
    before = node.map_selection.path.read_text(encoding="utf-8")

    status, body = _serve_and_request(
        node, "/api/maps/select", {"auto_relocalization": True}, "POST"
    )
    assert status == 400
    assert "sc_db.bin" in body["message"]
    assert node.map_selection.path.read_text(encoding="utf-8") == before

    status, body = _serve_and_request(
        node, "/api/maps/select", {"auto_relocalization": False}, "POST"
    )
    assert status == 200
    assert body["selection"]["auto_relocalization"] is False
    assert body["selection"]["effective_auto_relocalization"] is False
    assert node.map_selection.load() == "site_plain"
    assert node.map_selection.load_mode() == "auto"


def test_switching_to_unsupported_map_keeps_true_but_effective_false(tmp_path):
    root = tmp_path / "map"
    _capable_map(root, "site_capable")
    _make_map_dir(root, "site_plain")
    node = _selection_node(tmp_path, selection="site_capable")
    node.map_selection.save("site_capable", "auto", True)

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_plain"}, "POST"
    )

    assert status == 200
    assert body["selection"]["auto_relocalization"] is True
    assert body["selection"]["effective_auto_relocalization"] is False
    assert node.map_selection.load() == "site_plain"
    assert node.map_selection.load_auto_relocalization() is True


@pytest.mark.parametrize("target", ["localization", "navigation"])
def test_flag_change_returns_409_while_localization_or_navigation_runs(
    tmp_path, target
):
    root = tmp_path / "map"
    _capable_map(root)
    node = _selection_node(tmp_path, selection="site_a", running=(target,))
    node.map_selection.save("site_a", "auto", False)
    before = node.map_selection.path.read_text(encoding="utf-8")

    status, body = _serve_and_request(
        node, "/api/maps/select", {"auto_relocalization": True}, "POST"
    )

    assert status == 409
    assert target in body["message"]
    assert "stop" in body["message"].lower()
    assert node.map_selection.path.read_text(encoding="utf-8") == before


def _localization_auto_args(node) -> list[str]:
    return [
        arg
        for arg in node.managed_start_command("localization")
        if arg.startswith("auto_relocalization:=")
    ]


def test_launch_always_explicitly_overrides_configured_auto_flag(tmp_path):
    root = tmp_path / "map"
    _capable_map(root)
    node = _selection_node(
        tmp_path,
        selection="site_a",
        localization_launch_args=(
            "auto_relocalization:=false use_rviz:=false "
            "auto_relocalization:=false"
        ),
    )
    node.map_selection.save("site_a", "auto", True)

    assert _localization_auto_args(node) == ["auto_relocalization:=true"]

    node.map_selection.save_auto_relocalization(False)
    assert _localization_auto_args(node) == ["auto_relocalization:=false"]


@pytest.mark.parametrize("mode", ["auto", "tiled", "global"])
def test_missing_database_forces_false_irrespective_of_mode(tmp_path, mode):
    root = tmp_path / "map"
    map_dir = _make_map_dir(root, "site_a")
    tiled_dir = _write_tiled_manifest(map_dir, _tiled_manifest_document())
    _write_tile_files(tiled_dir)
    node = _selection_node(
        tmp_path,
        selection="site_a",
        localization_launch_args="auto_relocalization:=true use_rviz:=false",
    )
    node.map_selection.save("site_a", mode, True)

    assert _localization_auto_args(node) == ["auto_relocalization:=false"]
    # Navigation still uses the GlobalMap parent directory, not the tiled path.
    navigation = node.managed_start_command("navigation")
    map_args = [arg for arg in navigation if arg.startswith("map_path:=")]
    assert map_args == [
        f"map_path:={str((map_dir / 'GlobalMap.pcd').resolve().parent)}"
    ]


def test_persistence_io_failure_returns_503_without_flipping_state(
    tmp_path, monkeypatch
):
    root = tmp_path / "map"
    _capable_map(root)
    node = _selection_node(tmp_path, selection="site_a")
    node.map_selection.save("site_a", "auto", True)
    before = node.map_selection.path.read_text(encoding="utf-8")

    def fail(_enabled):
        raise OSError("disk full")

    monkeypatch.setattr(node.map_selection, "save_auto_relocalization", fail)

    status, body = _serve_and_request(
        node, "/api/maps/select", {"auto_relocalization": False}, "POST"
    )

    assert status == 503
    assert body["ok"] is False
    assert node.map_selection.path.read_text(encoding="utf-8") == before
    assert node.map_selection.load_auto_relocalization() is True
    status, panel = _serve_and_request(node, "/api/maps")
    assert status == 200
    assert panel["selection"]["auto_relocalization"] is True
    assert panel["selection"]["effective_auto_relocalization"] is True


def test_concurrent_name_and_flag_updates_do_not_overwrite_each_other(tmp_path):
    root = tmp_path / "map"
    _capable_map(root, "site_a")
    _capable_map(root, "site_b")
    node = _selection_node(tmp_path, selection="site_a")
    node.map_selection.save("site_a", "auto", False)

    name_flag_read = threading.Event()
    release_name = threading.Event()
    flag_saved = threading.Event()
    original_load_auto = node.map_selection.load_auto_relocalization
    original_save_auto = node.map_selection.save_auto_relocalization

    def paused_load_auto():
        value = original_load_auto()
        if threading.current_thread().name == "test-name-only":
            name_flag_read.set()
            assert release_name.wait(5), "name-only selection did not resume"
        return value

    def tracked_save_auto(enabled):
        value = original_save_auto(enabled)
        if threading.current_thread().name == "test-flag-only":
            flag_saved.set()
        return value

    node.map_selection.load_auto_relocalization = paused_load_auto
    node.map_selection.save_auto_relocalization = tracked_save_auto

    class ObservedLock:
        def __init__(self):
            self._lock = threading.RLock()
            self.attempted = threading.Event()
            self.blocked = threading.Event()
            self._owner = None
            self._depth = 0

        def __enter__(self):
            current = threading.current_thread()
            if current.name == "test-flag-only":
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

    node.lock = ObservedLock()
    outcome = {}

    def select_name():
        try:
            outcome["name"] = node.select_map({"name": "site_b"})
        except BaseException as exc:  # noqa: BLE001 - asserted below
            outcome["name_error"] = exc

    def select_flag():
        try:
            outcome["flag"] = node.select_map({"auto_relocalization": True})
        except BaseException as exc:  # noqa: BLE001 - asserted below
            outcome["flag_error"] = exc

    name_thread = threading.Thread(target=select_name, name="test-name-only")
    flag_thread = threading.Thread(target=select_flag, name="test-flag-only")
    name_thread.start()
    assert name_flag_read.wait(5), "name-only selection did not read the flag"
    flag_thread.start()
    assert node.lock.attempted.wait(5), "flag update never reached the node lock"
    release_name.set()
    name_thread.join(5)
    flag_thread.join(5)

    assert "name_error" not in outcome and "flag_error" not in outcome
    assert outcome["name"]["selection"]["name"] == "site_b"
    assert outcome["flag"]["selection"]["auto_relocalization"] is True
    assert flag_saved.is_set()
    persisted = json.loads(node.map_selection.path.read_text(encoding="utf-8"))
    assert persisted["name"] == "site_b"
    assert persisted["auto_relocalization"] is True


_AUTO_RELOCALIZATION_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("// --- Map selector");
const end = source.indexOf("// Per-target cursor");
if (start < 0 || end < 0) throw new Error("map selector block not found");
const block = source.slice(start, end);

function makeElement() {
  return {
    textContent: "", className: "", innerHTML: "", value: "",
    disabled: false, hidden: false, children: [], title: "",
    attributes: {}, classes: new Set(),
    appendChild(child) { this.children.push(child); },
    setAttribute(name, value) { this.attributes[name] = String(value); },
    getAttribute(name) { return this.attributes[name] ?? null; },
    classList: {
      toggle(name, force) {
        if (force) this.owner.classes.add(name);
        else this.owner.classes.delete(name);
      },
      contains(name) { return this.owner.classes.has(name); },
    },
  };
}
const elements = {};
for (const id of [
  "mapSelect", "mapMode", "mapWarning", "mapStatus", "mapLockNote",
  "autoRelocalizationToggle",
]) {
  elements[id] = makeElement();
  elements[id].classList.owner = elements[id];
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
  const selection = { ...state.mapCatalog.selection };
  if (typeof payload.auto_relocalization === "boolean") {
    selection.auto_relocalization = payload.auto_relocalization;
    selection.effective_auto_relocalization =
      payload.auto_relocalization && selection.auto_relocalization_available;
    selection.auto_relocalization_message =
      selection.auto_relocalization_available
        ? null
        : "selected map has no sc_db.bin; automatic relocalization is unavailable";
  }
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
function invalidateNavigationMapCaches() { throw new Error("flag-only change must not invalidate geometry"); }
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

function catalog(hasScanContext, requested) {
  const entry = {
    name: "site_a", tiled: false,
    has_scan_context: hasScanContext, warning: null,
  };
  return {
    ok: true, map_root: "~/map", maps: [entry],
    selection: {
      name: "site_a", valid: true, entry, message: null,
      mode: "auto", effective_mode: "global",
      mode_valid: true, mode_message: null,
      auto_relocalization: requested,
      auto_relocalization_available: hasScanContext,
      effective_auto_relocalization: hasScanContext && requested,
      auto_relocalization_message: hasScanContext
        ? null
        : "selected map has no sc_db.bin; automatic relocalization is unavailable",
    },
  };
}

function install(hasScanContext, requested) {
  serverCatalog = catalog(hasScanContext, requested);
  state.mapCatalog = clone(serverCatalog);
  state.mapError = null;
  state.mapSelectionError = null;
  state.mapSelectionNotice = null;
  state.mapSelectPending = false;
  updateMapPanel();
}

(async () => {
  const button = elements.autoRelocalizationToggle;

  install(true, true);
  expect(!button.disabled, "capable map must enable the toggle");
  expect(button.getAttribute("aria-pressed") === "true", "effective true must set aria-pressed");
  expect(button.textContent === "关闭自动重定位", "enabled button label is wrong");

  install(true, false);
  expect(!button.disabled, "disabled preference must remain editable");
  expect(button.getAttribute("aria-pressed") === "false", "effective false must clear aria-pressed");
  expect(button.textContent === "开启自动重定位", "disabled button label is wrong");

  install(false, true);
  expect(button.disabled, "missing sc_db.bin must disable the toggle");
  expect(button.textContent === "自动重定位不可用", "unsupported label is wrong");
  expect(/sc_db\.bin/.test(button.title), "unsupported reason missing from title");
  expect(button.getAttribute("aria-pressed") === "false", "unsupported map must report effective false");

  install(true, true);
  state.localizationRunning = true;
  updateMapSelectionLock();
  expect(button.disabled, "running localization must lock the toggle");
  state.localizationRunning = false;
  updateMapSelectionLock();
  expect(!button.disabled, "toggle must unlock after localization stops");

  install(true, true);
  deferNextPost = true;
  const pendingToggle = toggleAutoRelocalization();
  const pendingPost = deferredPosts.shift();
  expect(pendingPost && state.mapSelectPending, "toggle POST did not remain pending");
  expect(button.disabled, "pending toggle must disable the button");
  expect(button.getAttribute("aria-busy") === "true", "pending aria-busy missing");
  pendingPost.resolve();
  await pendingToggle;
  expect(captured.url === "/api/maps/select", "toggle did not use the select endpoint");
  expect(
    JSON.stringify(captured.payload) === JSON.stringify({ auto_relocalization: false }),
    "flag-only payload is wrong"
  );
  expect(!button.disabled && button.textContent === "开启自动重定位", "successful toggle was not applied");
  expect(button.getAttribute("aria-pressed") === "false", "successful toggle did not clear aria-pressed");

  install(true, true);
  const staleBeforePost = deferredGet();
  await toggleAutoRelocalization();
  staleBeforePost.pending.resolve();
  await staleBeforePost.promise;
  expect(
    state.mapCatalog.selection.effective_auto_relocalization === false,
    "GET started before the flag POST overwrote the new state"
  );
  expect(button.getAttribute("aria-pressed") === "false", "stale GET reverted the button");

  install(true, true);
  deferNextPost = true;
  const pendingSecond = toggleAutoRelocalization();
  const secondPost = deferredPosts.shift();
  const staleDuringPost = deferredGet();
  secondPost.resolve();
  await pendingSecond;
  staleDuringPost.pending.resolve();
  await staleDuringPost.promise;
  expect(
    state.mapCatalog.selection.effective_auto_relocalization === false,
    "GET issued during the flag POST overwrote the new state"
  );
  expect(button.getAttribute("aria-pressed") === "false", "stale pending GET reverted the button");

  install(true, true);
  failPost = "unable to save map selection: disk full";
  await toggleAutoRelocalization();
  failPost = null;
  expect(
    state.mapCatalog.selection.auto_relocalization === true,
    "failed save left the UI flag flipped"
  );
  expect(button.getAttribute("aria-pressed") === "true", "failed save flipped aria-pressed");
  expect(button.textContent === "关闭自动重定位", "failed save flipped the label");
  expect(/Map selection failed: unable to save map selection/.test(elements.mapStatus.textContent), "save failure was not surfaced");
  expect(!button.disabled, "recovery toggle must stay enabled after a failed save");

  console.log("auto relocalization behavior ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_auto_toggle_is_one_button_in_the_map_card_not_startup_grid():
    index_html = (
        Path(__file__).resolve().parents[1] / "static" / "index.html"
    ).read_text(encoding="utf-8")
    assert index_html.count('id="autoRelocalizationToggle"') == 1
    map_start = index_html.index("<h2>Map</h2>")
    map_end = index_html.index("</section>", map_start)
    map_card = index_html[map_start:map_end]
    assert 'id="autoRelocalizationToggle"' in map_card
    assert 'aria-pressed="false"' in map_card
    assert " disabled" in map_card

    startup_start = index_html.index("<h2>Startup</h2>")
    startup_end = index_html.index("</section>", startup_start)
    startup = index_html[startup_start:startup_end]
    assert 'id="autoRelocalizationToggle"' not in startup
    assert startup.count('class="start-log') == 6


def test_frontend_auto_relocalization_pending_locks_stale_polls_and_rollback():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _AUTO_RELOCALIZATION_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "auto relocalization behavior ok" in completed.stdout
