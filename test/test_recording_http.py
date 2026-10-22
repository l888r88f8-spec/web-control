import json
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import MethodType, SimpleNamespace

from web_control import recording, web_control_node


FAKE_BAG = """
import pathlib, signal, sys, time
out = pathlib.Path(sys.argv[sys.argv.index("-o") + 1])
out.mkdir(parents=True, exist_ok=False)
def stop(_sig, _frame):
    (out / "metadata.yaml").write_text("version: 5\\n")
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
print("fake rosbag ready", flush=True)
while True:
    time.sleep(0.05)
"""


class _LoggerStub:
    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass

    def debug(self, *_args, **_kwargs):
        pass


class _FakeRecordingNode:
    def __init__(self, root, command):
        self.recorder = recording.RosbagRecorder(root, command=command)
        self.topics = {
            "/tf": ["tf2_msgs/msg/TFMessage"],
            "/scan": ["sensor_msgs/msg/LaserScan"],
        }
        self.static_dir = Path(__file__).resolve().parents[1] / "static"
        self.logger = _LoggerStub()
        self.clock = 10.0
        self.lock = threading.RLock()
        self.mapping_layers = {
            "global_map": SimpleNamespace(summary=lambda now: {}),
            "ground": SimpleNamespace(summary=lambda now: {}),
        }
        self.mapping_path = SimpleNamespace(summary=lambda now: {})
        self.mapping_odom = SimpleNamespace(summary=lambda now: {}, data={})
        self.map_frame = "map"
        self.save_map_status_payload = lambda: {"state": "idle"}
        for name in (
            "record_topics_payload",
            "record_browse_payload",
            "start_recording",
            "stop_recording",
            "recording_status_payload",
            "mapping_state_payload",
        ):
            setattr(self, name, MethodType(getattr(web_control_node.WebControlNode, name), self))

    def now_sec(self):
        self.clock += 0.25
        return self.clock

    def get_logger(self):
        return self.logger

    def get_topic_names_and_types(self):
        return list(self.topics.items())


def _serve(node):
    handler = web_control_node.WebControlNode.make_handler(node)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def _request(url, payload=None, method="GET"):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        request.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def test_recording_http_lifecycle_status_refresh_and_errors(tmp_path):
    script = tmp_path / "fake_bag.py"
    script.write_text(FAKE_BAG)
    node = _FakeRecordingNode(tmp_path, [sys.executable, str(script)])
    server, _thread = _serve(node)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        status, body = _request(f"{base}/api/mapping/record/topics")
        assert status == 200
        assert [topic["name"] for topic in body["topics"]] == ["/scan", "/tf"]

        status, body = _request(f"{base}/api/mapping/record/browse")
        assert status == 200
        assert body["path"] == str(tmp_path)
        assert body["default_directory"]

        payload = {"directory": str(tmp_path), "name": "run_01", "topics": ["/tf"]}
        status, body = _request(f"{base}/api/mapping/record/start", payload, "POST")
        assert status == 200, body
        assert body["recording"]["state"] == "recording"
        deadline = time.time() + 3
        while "fake rosbag ready" not in node.recorder.status()["log_tail"]:
            assert time.time() < deadline
            time.sleep(0.02)

        status, body = _request(f"{base}/api/mapping/record")
        assert status == 200 and body["state"] == "recording"
        status, body = _request(f"{base}/api/mapping/record")
        assert status == 200 and body["state"] == "recording"
        status, body = _request(f"{base}/api/mapping/state")
        assert status == 200 and body["recording"]["state"] == "recording"

        status, body = _request(f"{base}/api/mapping/record/start", payload, "POST")
        assert status == 409, body

        status, body = _request(
            f"{base}/api/mapping/record/start",
            {"directory": str(tmp_path), "name": "run_02", "topics": ["/ghost"]},
            "POST",
        )
        assert status == 400, body

        status, body = _request(f"{base}/api/mapping/record/stop", {}, "POST")
        assert status == 200, body
        assert body["recording"]["state"] == "stopped", body
        assert (tmp_path / "run_01" / "metadata.yaml").read_text() == "version: 5\n"

        status, body = _request(f"{base}/api/mapping/record/start", payload, "POST")
        assert status == 409, body

        status, body = _request(
            f"{base}/api/mapping/record/start",
            {"directory": "relative", "name": "run_03", "topics": ["/tf"]},
            "POST",
        )
        assert status == 400, body
    finally:
        node.recorder.cleanup()
        server.shutdown()
        server.server_close()
