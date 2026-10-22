import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from web_control import recording


def _wait_state(recorder, expected, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        snapshot = recorder.status()
        if snapshot["state"] in expected:
            return snapshot
        time.sleep(0.02)
    raise AssertionError(f"recorder stayed in {recorder.status()['state']}")


def _script(tmp_path, body):
    path = tmp_path / f"fake_{time.time_ns()}.py"
    path.write_text(body)
    return [sys.executable, str(path)]


SIGINT_SCRIPT = """
import pathlib, signal, sys, time
out = pathlib.Path(sys.argv[sys.argv.index("-o") + 1])
out.mkdir(parents=True, exist_ok=False)
def stop(_sig, _frame):
    (out / "metadata.yaml").write_text("version: 5\\n")
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
print("fake rosbag recording", flush=True)
while True:
    time.sleep(0.05)
"""


def test_resolve_recording_output_accepts_writable_absolute_directory(tmp_path):
    bags = tmp_path / "bags"
    bags.mkdir()
    assert recording.resolve_recording_output(str(bags), "run_01", tmp_path) == bags / "run_01"


@pytest.mark.parametrize("directory", ["bags", "", "/", "/does/not/exist", "/tmp/../tmp"])
def test_resolve_recording_output_rejects_bad_directories(tmp_path, directory):
    with pytest.raises(ValueError):
        recording.resolve_recording_output(directory, "run_01", tmp_path)


def test_resolve_recording_output_rejects_outside_root_and_symlink_escape(tmp_path):
    root = tmp_path / "root"
    root.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    (root / "link").symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="outside"):
        recording.resolve_recording_output(str(outside), "run", root)
    with pytest.raises(ValueError, match="outside"):
        recording.resolve_recording_output(str(root / "link"), "run", root)


def test_resolve_recording_output_rejects_existing_target_and_bad_name(tmp_path):
    (tmp_path / "run").mkdir()
    with pytest.raises(recording.RecordingConflictError):
        recording.resolve_recording_output(str(tmp_path), "run", tmp_path)
    for name in ("", "../x", "a/b", ".hidden", "x" * 129):
        with pytest.raises(ValueError):
            recording.resolve_recording_output(str(tmp_path), name, tmp_path)


def test_resolve_recording_output_rejects_non_writable_directory(tmp_path, monkeypatch):
    monkeypatch.setattr(recording.os, "access", lambda *args, **kwargs: False)
    with pytest.raises(ValueError, match="writable"):
        recording.resolve_recording_output(str(tmp_path), "run", tmp_path)


def test_validate_topic_selection_requires_live_unique_topics():
    graph = {"/tf": ["tf2_msgs/msg/TFMessage"], "/scan": ["sensor_msgs/msg/LaserScan"]}
    assert recording.validate_topic_selection({"topics": ["/tf", "/scan"]}, graph) == ["/tf", "/scan"]
    for payload in ({}, {"topics": []}, {"topics": ["/tf", "/tf"]}, {"topics": ["/ghost"]}, {"topics": [1]}, {"topics": "x"}):
        with pytest.raises(ValueError):
            recording.validate_topic_selection(payload, graph)


def test_browse_recording_directories_lists_writable_children(tmp_path):
    (tmp_path / "bags").mkdir()
    (tmp_path / "a.txt").write_text("x")
    payload = recording.browse_recording_directories("", tmp_path)
    assert payload["path"] == str(tmp_path)
    assert payload["parent"] is None
    assert [entry["name"] for entry in payload["entries"]] == ["bags"]
    assert payload["writable"] is True


def test_recorder_sigint_finalizes_metadata_and_rejects_duplicate_start(tmp_path, monkeypatch):
    monkeypatch.setattr(recording, "SIGINT_TIMEOUT", 3.0)
    recorder = recording.RosbagRecorder(tmp_path, command=_script(tmp_path, SIGINT_SCRIPT))
    try:
        output = tmp_path / "run"
        snapshot = recorder.start(["/tf"], output, now=1.0)
        assert snapshot["state"] == "recording"
        assert snapshot["topics"] == ["/tf"]
        deadline = time.time() + 3
        while "fake rosbag recording" not in recorder.status()["log_tail"]:
            assert time.time() < deadline, recorder.status()
            time.sleep(0.02)
        with pytest.raises(recording.RecordingBusyError):
            recorder.start(["/tf"], tmp_path / "other", now=2.0)
        final = recorder.stop(now=3.0)
        assert (output / "metadata.yaml").read_text() == "version: 5\n"
        assert final["state"] == "stopped", final
        assert final["running"] is False
        assert final["returncode"] == 0
        assert "finalized" in final["message"]
        assert recorder.start(["/scan"], tmp_path / "second", now=4.0)["state"] == "recording"
    finally:
        recorder.cleanup()


def test_recorder_natural_success_and_nonzero_failure(tmp_path):
    success = """
import pathlib, sys
out = pathlib.Path(sys.argv[sys.argv.index("-o") + 1])
out.mkdir(parents=True, exist_ok=False)
(out / "metadata.yaml").write_text("version: 5\\n")
print("done", flush=True)
"""
    recorder = recording.RosbagRecorder(tmp_path, command=_script(tmp_path, success))
    recorder.start(["/tf"], tmp_path / "ok", now=1.0)
    final = _wait_state(recorder, {"stopped", "failed"})
    assert final["state"] == "stopped"
    assert final["returncode"] == 0

    failure = """
import pathlib, sys
out = pathlib.Path(sys.argv[sys.argv.index("-o") + 1])
out.mkdir(parents=True, exist_ok=False)
print("rosbag could not open the output", flush=True)
sys.exit(3)
"""
    recorder = recording.RosbagRecorder(tmp_path, command=_script(tmp_path, failure))
    recorder.start(["/tf"], tmp_path / "bad", now=2.0)
    final = _wait_state(recorder, {"stopped", "failed"})
    assert final["state"] == "failed"
    assert final["returncode"] == 3
    assert "could not open" in final["message"]


def test_recorder_spawn_failure_is_reported_as_failed(tmp_path):
    recorder = recording.RosbagRecorder(tmp_path, command=[str(tmp_path / "missing-command")])
    with pytest.raises(recording.RecordingUnavailableError):
        recorder.start(["/tf"], tmp_path / "run", now=1.0)
    snapshot = recorder.status(now=2.0)
    assert snapshot["state"] == "failed"
    assert "unavailable" in snapshot["message"]


class _FakeStream:
    def __init__(self):
        self._done = threading.Event()
        self.close_calls = 0

    def readline(self):
        self._done.wait()
        return ""

    def close(self):
        self.close_calls += 1
        self._done.set()

    def release(self):
        self.close_calls += 1
        self._done.set()


class _StuckStream:
    def __init__(self):
        self._release = threading.Event()
        self.close_calls = 0

    def readline(self):
        self._release.wait()
        return ""

    def close(self):
        self.close_calls += 1

    def release(self):
        self._release.set()


class _FakeProcess:
    def __init__(self, pid=424242, stream=None):
        self.pid = pid
        self.stdout = stream or _FakeStream()
        self.returncode = None
        self._exited = threading.Event()
        self._lock = threading.Lock()

    def poll(self):
        with self._lock:
            return self.returncode

    def wait(self, timeout=None):
        if timeout is None:
            self._exited.wait()
        elif not self._exited.wait(timeout):
            raise recording.subprocess.TimeoutExpired("fake-rosbag", timeout)
        with self._lock:
            return self.returncode

    def exit(self, code, close_stream=True):
        with self._lock:
            self.returncode = code
        if close_stream:
            self.stdout.close()
        self._exited.set()


def _install_fake_process(tmp_path, monkeypatch, fake, *, sigint=0.5):
    monkeypatch.setattr(recording.shutil, "which", lambda _command: "/fake/ros2")
    monkeypatch.setattr(recording.subprocess, "Popen", lambda *_args, **_kwargs: fake)
    signals = []

    def fake_killpg(pgid, sig):
        signals.append((pgid, sig))
        return None

    monkeypatch.setattr(recording.os, "killpg", fake_killpg)
    monkeypatch.setattr(recording, "SIGINT_TIMEOUT", sigint)
    monkeypatch.setattr(recording, "SIGTERM_TIMEOUT", 0.02)
    monkeypatch.setattr(recording, "SIGKILL_TIMEOUT", 0.02)
    monkeypatch.setattr(recording, "READER_JOIN_TIMEOUT", 0.02)
    recorder = recording.RosbagRecorder(tmp_path, command=["fake-ros2"])
    return recorder, signals


def _finish_bag(output):
    output.mkdir(parents=True, exist_ok=True)
    (output / "metadata.yaml").write_text("version: 5\n")


PIPE_HOLDER_SCRIPT = """
import pathlib, signal, subprocess, sys, time
out = pathlib.Path(sys.argv[sys.argv.index("-o") + 1])
ready = pathlib.Path(sys.argv[sys.argv.index("--ready") + 1])
child_ready = pathlib.Path(sys.argv[sys.argv.index("--child-ready") + 1])
child_code = sys.argv[sys.argv.index("--child-code") + 1]
out.mkdir(parents=True, exist_ok=False)
subprocess.Popen([sys.executable, "-c", child_code, str(child_ready)])
deadline = time.time() + 2.0
while not child_ready.exists():
    if time.time() > deadline:
        raise SystemExit("descendant did not become ready")
    time.sleep(0.01)
def stop(_sig, _frame):
    (out / "metadata.yaml").write_text("version: 5\\n")
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
ready.write_text("ready")
print("fake rosbag recording", flush=True)
while True:
    time.sleep(0.05)
"""

PIPE_HOLDER_CHILD = """
import pathlib, signal, sys, time
signal.signal(signal.SIGINT, signal.SIG_IGN)
pathlib.Path(sys.argv[1]).write_text("ready")
time.sleep(1.0)
"""

MALFORMED_OUTPUT_SCRIPT = """
import os, pathlib, sys
out = pathlib.Path(sys.argv[sys.argv.index("-o") + 1])
out.mkdir(parents=True, exist_ok=False)
os.write(1, b"bad \\xff byte\\n")
for _ in range(2000):
    os.write(1, b"x" * 128 + b"\\n")
(out / "metadata.yaml").write_text("version: 5\\n")
"""


def test_stop_does_not_close_pipe_owned_by_live_reader(tmp_path, monkeypatch):
    monkeypatch.setattr(recording, "SIGINT_TIMEOUT", 2.0)
    monkeypatch.setattr(recording, "READER_JOIN_TIMEOUT", 0.05)
    ready = tmp_path / "ready"
    child_ready = tmp_path / "child_ready"
    output = tmp_path / "run"
    recorder = recording.RosbagRecorder(
        tmp_path, command=_script(tmp_path, PIPE_HOLDER_SCRIPT)
    )
    try:
        recorder.start(
            [
                "--ready", str(ready),
                "--child-ready", str(child_ready),
                "--child-code", PIPE_HOLDER_CHILD,
            ],
            output,
            now=1.0,
        )
        deadline = time.time() + 2.0
        while not ready.exists():
            assert time.time() < deadline, recorder.status()
            time.sleep(0.01)

        started = time.monotonic()
        snapshot = recorder.stop(now=2.0)
        elapsed = time.monotonic() - started
        assert elapsed < 0.7, f"stop blocked for {elapsed:.3f}s closing a live pipe"
        assert snapshot["state"] == "stopping", snapshot
        assert snapshot["running"] is False

        final = _wait_state(recorder, {"stopped", "failed"}, timeout=4.0)
        assert final["state"] == "stopped", final
        assert final["returncode"] == 0
    finally:
        pgid = recorder._pgid
        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        recorder.cleanup()
        reader = recorder._reader
        if reader is not None:
            reader.join(2.0)


def test_recorder_drains_malformed_output_from_real_pipe(tmp_path, monkeypatch):
    monkeypatch.setattr(recording, "SIGINT_TIMEOUT", 2.0)
    recorder = recording.RosbagRecorder(
        tmp_path, command=_script(tmp_path, MALFORMED_OUTPUT_SCRIPT)
    )
    pgid = None
    try:
        recorder.start(["/tf"], tmp_path / "run", now=1.0)
        pgid = recorder._pgid
        final = _wait_state(recorder, {"stopped", "failed"}, timeout=4.0)
        assert final["state"] == "stopped", final
        assert final["returncode"] == 0
        assert final["log_tail"], final
        assert set(final["log_tail"]) == {"x" * 128}, final
    finally:
        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        recorder.cleanup()


def test_recorder_does_not_finalize_when_kill_wait_times_out(tmp_path, monkeypatch):
    fake = _FakeProcess()
    recorder, signals = _install_fake_process(tmp_path, monkeypatch, fake)
    output = tmp_path / "run"
    try:
        assert recorder.start(["/tf"], output, now=1.0)["state"] == "recording"
        snapshot = recorder.stop(now=2.0)
        assert snapshot["state"] == "stopping", snapshot
        assert snapshot["running"] is True
        assert [sig for _pgid, sig in signals] == [
            recording.signal.SIGINT,
            recording.signal.SIGTERM,
            recording.signal.SIGKILL,
        ]
        with pytest.raises(recording.RecordingBusyError):
            recorder.start(["/tf"], tmp_path / "other", now=3.0)

        _finish_bag(output)
        fake.exit(0)
        final = _wait_state(recorder, {"stopped", "failed"})
        assert final["state"] == "failed", final
        assert final["returncode"] == 0
        assert "forced termination" in final["message"]
        assert final["running"] is False
    finally:
        fake.exit(-9)
        recorder.cleanup()


def test_recorder_waits_for_reader_drain_before_finalizing(tmp_path, monkeypatch):
    stream = _StuckStream()
    fake = _FakeProcess(stream=stream)
    recorder, _signals = _install_fake_process(tmp_path, monkeypatch, fake)
    output = tmp_path / "run"
    try:
        recorder.start(["/tf"], output, now=1.0)
        _finish_bag(output)
        fake.exit(0, close_stream=False)

        snapshot = recorder.stop(now=2.0)
        assert snapshot["state"] == "stopping", snapshot
        assert snapshot["running"] is False
        assert stream.close_calls == 0, "stop closed a stream owned by the live reader"
        with pytest.raises(recording.RecordingBusyError):
            recorder.start(["/tf"], tmp_path / "other", now=3.0)

        stream.release()
        final = _wait_state(recorder, {"stopped", "failed"})
        assert final["state"] == "stopped", final
        assert final["returncode"] == 0
    finally:
        stream.release()
        fake.exit(-9)
        recorder.cleanup()


def test_recorder_concurrent_cleanup_waits_for_stop_lock(tmp_path, monkeypatch):
    fake = _FakeProcess()
    recorder, _signals = _install_fake_process(tmp_path, monkeypatch, fake)
    output = tmp_path / "run"
    stop_thread = None
    cleanup_thread = None
    try:
        recorder.start(["/tf"], output, now=1.0)
        _finish_bag(output)
        stop_started = threading.Event()
        cleanup_started = threading.Event()
        errors = []

        def stop_worker():
            stop_started.set()
            try:
                recorder.stop(now=2.0)
            except Exception as exc:  # pragma: no cover - surfaced below
                errors.append(exc)

        def cleanup_worker():
            cleanup_started.set()
            try:
                recorder.cleanup(now=3.0)
            except Exception as exc:  # pragma: no cover - surfaced below
                errors.append(exc)

        stop_thread = threading.Thread(target=stop_worker)
        stop_thread.start()
        assert stop_started.wait(2.0)
        deadline = time.time() + 2.0
        while recorder.status()["state"] != "stopping":
            assert time.time() < deadline
            time.sleep(0.01)

        cleanup_thread = threading.Thread(target=cleanup_worker)
        cleanup_thread.start()
        assert cleanup_started.wait(2.0)
        time.sleep(0.05)
        assert cleanup_thread.is_alive(), "cleanup returned while stop was still working"

        fake.exit(0)
        stop_thread.join(2.0)
        cleanup_thread.join(2.0)
        assert not stop_thread.is_alive()
        assert not cleanup_thread.is_alive()
        assert errors == []
        assert recorder.status()["state"] == "stopped"
    finally:
        fake.exit(-9)
        for thread in (stop_thread, cleanup_thread):
            if thread is not None:
                thread.join(1.0)
        recorder.cleanup()


def test_recorder_metadata_stat_errors_do_not_strand_reader(tmp_path, monkeypatch):
    fake = _FakeProcess()
    recorder, _signals = _install_fake_process(tmp_path, monkeypatch, fake)
    output = tmp_path / "run"
    original_is_file = Path.is_file

    def failing_is_file(path):
        if path.name == "metadata.yaml":
            raise OSError("metadata stat denied")
        return original_is_file(path)

    try:
        recorder.start(["/tf"], output, now=1.0)
        _finish_bag(output)
        monkeypatch.setattr(Path, "is_file", failing_is_file)
        fake.exit(0)
        final = _wait_state(recorder, {"stopped", "failed"})
        assert final["state"] == "failed", final
        assert "metadata" in final["message"]
    finally:
        fake.exit(-9)
        recorder.cleanup()


_RECORDING_FRONTEND_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");

function functionBody(signature) {
  const start = source.indexOf(signature);
  if (start < 0) throw new Error("function not found: " + signature);
  const brace = source.indexOf("{", start);
  let depth = 0;
  for (let index = brace; index < source.length; index += 1) {
    if (source[index] === "{") depth += 1;
    else if (source[index] === "}") {
      depth -= 1;
      if (depth === 0) return source.slice(start, index + 1);
    }
  }
  throw new Error("unterminated function: " + signature);
}

const asyncNames = new Set(["openRecordBagDialog", "submitRecording", "stopRecording"]);
const block = [
  "recordingTimestamp",
  "freshRecordingName",
  "updateRecordingStatus",
  "openRecordBagDialog",
  "submitRecording",
  "stopRecording",
].map((name) => functionBody(`${asyncNames.has(name) ? "async " : ""}function ${name}(`)).join("\n\n");

function expect(condition, message) { if (!condition) throw new Error(message); }

const elements = {};
function element(id, props = {}) {
  const target = {
    id,
    textContent: "",
    value: "",
    disabled: false,
    hidden: false,
    classes: new Set(),
    focus() {},
    select() {},
    classList: {
      toggle(name, enabled) {
        if (enabled) target.classes.add(name);
        else target.classes.delete(name);
      },
    },
    ...props,
  };
  elements[id] = target;
  return target;
}

const open = element("openRecordBag");
const stop = element("stopRecording");
const status = element("recordingStatus");
element("recordingOutput");
const dialog = element("recordBagDialog", {
  showModal() { this.open = true; },
  close() { this.open = false; },
  open: false,
});
const name = element("recordingName");
const directory = element("recordingDirectory", { value: "/bags" });
element("recordingError");
element("startRecording");

global.document = {
  getElementById(id) { return elements[id] || null; },
  querySelectorAll(selector) {
    if (selector === "#recordingTopics input[type=checkbox]:checked") {
      return global.selectedTopics || [];
    }
    return [];
  },
};

const state = {
  recording: {
    available: true,
    state: "stopped",
    topics: ["/tf"],
    output_dir: "/bags/old",
    message: "",
    log_tail: [],
  },
  recordingTopics: [{ name: "/tf", types: ["tf2_msgs/msg/TFMessage"] }],
  recordingTopicsError: null,
  recordingStartError: null,
  recordingActionError: null,
  recordingAutoName: null,
  recordingUsedName: null,
  recordingPending: false,
};

let startCalls = 0;
let stopCalls = 0;
let stopFailure = null;
const postJson = async (url, payload) => {
  if (url === "/api/mapping/record/start") {
    startCalls += 1;
    state.recording = {
      available: true,
      state: "recording",
      topics: payload.topics,
      output_dir: `/bags/${payload.name}`,
      message: "",
      log_tail: [],
    };
    return { recording: state.recording };
  }
  if (url === "/api/mapping/record/stop") {
    stopCalls += 1;
    if (stopFailure) throw new Error(stopFailure);
    state.recording = {
      available: true,
      state: "stopped",
      topics: state.recording.topics,
      output_dir: state.recording.output_dir,
      message: "Recording stopped; bag finalized.",
      log_tail: [],
    };
    return { recording: state.recording };
  }
  throw new Error(`unexpected post: ${url}`);
};

const pollMappingState = () => {
  api.updateRecordingStatus(state.recording);
};

const api = new Function(
  "state",
  "document",
  "getJson",
  "postJson",
  "pollMappingState",
  "renderRecordingTopics",
  "refreshRecordingTopics",
  "loadRecordingDirectory",
  block + `
return {
  recordingTimestamp,
  freshRecordingName,
  updateRecordingStatus,
  openRecordBagDialog,
  submitRecording,
  stopRecording,
};`,
)(state, global.document, async () => ({}), postJson, pollMappingState,
  () => {}, async () => {}, async () => {});

(async () => {
  global.selectedTopics = [{ value: "/tf" }];

  await api.openRecordBagDialog();
  const firstDefault = name.value;
  expect(/^mapping_\d{8}_\d{6}/.test(firstDefault), "first open has timestamped default");
  expect(state.recordingAutoName === firstDefault, "first default is tracked as automatic");

  await api.submitRecording({ preventDefault() {} });
  expect(startCalls === 1, "start was submitted");
  expect(state.recordingUsedName === firstDefault, "successful start records the used name");
  expect(dialog.open === false, "dialog closes after a successful start");

  await api.stopRecording();
  expect(stopCalls === 1, "stop was submitted");
  expect(state.recording.state === "stopped", "stop updates the backend state");

  await api.openRecordBagDialog();
  const secondDefault = name.value;
  expect(secondDefault !== firstDefault, "reopening after recording regenerates the default");
  expect(secondDefault.startsWith("mapping_"), "regenerated default keeps the timestamp prefix");
  expect(state.recordingAutoName === secondDefault, "regenerated default is tracked");

  name.value = "intentional_custom_bag";
  await api.openRecordBagDialog();
  expect(name.value === "intentional_custom_bag", "intentional custom name is preserved");

  state.recording = {
    available: true,
    state: "recording",
    topics: ["/tf"],
    output_dir: "/bags/running",
    message: "",
    log_tail: [],
  };
  state.recordingPending = false;
  state.recordingActionError = null;
  api.updateRecordingStatus(state.recording);
  stopFailure = "network down";
  await api.stopRecording();

  expect(state.recordingActionError.includes("停止录制失败"), "stop error is retained in state");
  expect(state.recordingActionError.includes("network down"), "stop error keeps the real message");
  expect(status.textContent.includes("停止录制失败"), "stop error survives the status poll");
  expect(status.textContent.includes("network down"), "stop error text is visible");
  expect(status.classes.has("recording-status-error"), "stop error uses the error style");
  expect(stop.disabled === false, "backend recording state keeps stop available");
  expect(open.disabled === true, "backend recording state keeps start unavailable");

  await api.openRecordBagDialog();
  expect(state.recordingActionError === null, "opening the dialog clears the action error");
  expect(!status.textContent.includes("停止录制失败"), "cleared action error leaves the status");

  console.log("recording frontend ok");
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
"""


def test_recording_frontend_repeated_defaults_and_persistent_stop_error():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _RECORDING_FRONTEND_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "recording frontend ok" in completed.stdout


def test_recording_browser_hidden_attribute_beats_grid_display():
    styles = (Path(__file__).resolve().parents[1] / "static" / "styles.css").read_text(
        encoding="utf-8"
    )
    block = styles[
        styles.index(".recording-browser[hidden]") : styles.index(
            "}", styles.index(".recording-browser[hidden]")
        )
    ]
    assert "display: none" in block
