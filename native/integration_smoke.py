#!/usr/bin/env python3
"""Exercise the native HTTP and ROS ingestion paths with local synthetic data."""

import argparse
from contextlib import closing
from http.client import HTTPConnection
import json
from multiprocessing import get_context
import os
from pathlib import Path
import secrets
import socket
import sqlite3
import struct
import subprocess
from tempfile import TemporaryDirectory
import time
from urllib.parse import urlencode


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def available_port():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


def wait_for(action, description, process, timeout=12.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"backend exited with status {process.returncode}")
        value = action()
        if value:
            return value
        time.sleep(0.05)
    raise TimeoutError(f"timed out waiting for {description}")


def request(port, path, method="GET", body=None):
    headers = {} if body is None else {"Content-Type": "application/json"}
    with closing(HTTPConnection("127.0.0.1", port, timeout=2)) as connection:
        connection.request(method, path, body=body, headers=headers)
        with connection.getresponse() as response:
            return response.status, response.read()


def read_json(port, path, status=200, method="GET", body=None):
    actual_status, raw = request(port, path, method, body)
    require(actual_status == status, f"{method} {path} returned {actual_status} instead of {status}: {raw!r}")
    return json.loads(raw)


def validate_routes(port, root):
    status, page = request(port, "/")
    require(status == 200 and b"app.js" in page, "existing web interface was not served")
    state = read_json(port, "/api/state")
    require("grid_terrain" in state["layers"] and state["map_frame"] == "map", "navigation state mismatch")
    require(state["server_time"] > 1_650_000_000, "browser used simulation time instead of wall time")
    maps = read_json(port, "/api/maps")
    require(maps["ok"] and maps["selection"]["valid"], "fixture map was not selected")
    require(maps["selection"]["name"] == "smoke" and maps["selection"]["effective_mode"] == "global",
            "fixture map mode mismatch")
    require(maps["maps"] == [
        {"name": "smoke", "tiled": False, "has_scan_context": False, "warning": None}
    ], "unexpected map catalog")
    pose = read_json(port, "/api/pose")
    require(pose["map_frame"] == "map" and pose["odom"]["pose"] is None, "initial pose mismatch")
    tf = read_json(port, "/api/tf")
    require(tf["ok"] and tf["ros_available"] and tf["edge_count"] == 0, "initial TF mismatch")
    system = read_json(port, "/api/system")
    require(system["memory"]["total_kb"] > 0 and isinstance(system["cpu"]["cores"], list),
            "missing system statistics")
    diagnostics = read_json(port, "/api/diagnostics")
    require(diagnostics["topic_logging_enabled"] is False, "saved diagnostics setting was not applied")
    prefs = read_json(port, "/api/ui/preferences")
    require(prefs["joystick"]["throttle"] == {"size": 200, "x": None, "y": None}
            and prefs["layers"]["grid_terrain"] is False, "legacy preferences were not loaded")

    bad_path = urlencode({"path": str(root / "map" / "..")})
    reply = read_json(port, "/api/mapping/record/browse?" + bad_path, status=400)
    require(reply["ok"] is False and "'..'" in reply["message"], "browse did not reject traversal")
    for body in (b'{"layers":', b'{"layers": {"grid_terrain": "not a boolean"}}'):
        reply = read_json(port, "/api/ui/preferences", status=400, method="POST", body=body)
        require(reply["ok"] is False, "invalid preference POST was accepted")
    read_json(port, "/api/map3d.bin?layer=%FF", status=400)
    require(read_json(port, "/api/state")["map_frame"] == "map", "server failed after invalid UTF-8")


def read_event(response):
    fields = {}
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        line = response.readline()
        if not line:
            raise EOFError("SSE connection closed")
        if not line.strip():
            if "data" in fields:
                return fields["event"], json.loads(fields["data"])
            fields.clear()
        elif not line.startswith(b":"):
            name, _, value = line.decode().partition(":")
            fields[name] = value.strip()
    raise TimeoutError("timed out waiting for SSE metadata")


def validate_terrain(port, process, topic, root):
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("web_control_smoke_publisher")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(MarkerArray, topic, qos)
        vertices = [
            Point(x=-1.5, y=0.0, z=0.5),
            Point(x=0.0, y=2.25, z=0.0),
            Point(x=0.5, y=-1.0, z=4.0),
        ]
        face = Marker(ns="terrain_cells", type=Marker.TRIANGLE_LIST, points=vertices,
                      color=ColorRGBA(b=1.0, a=1.0), colors=[ColorRGBA(r=1.0, a=1.0)])
        face.header.frame_id = "synthetic_map"
        line = Marker(ns="terrain_grid_edges", type=Marker.LINE_LIST, points=vertices[:2])
        line.header.frame_id = "synthetic_map"
        message = MarkerArray(markers=[line, face])

        with closing(HTTPConnection("127.0.0.1", port, timeout=3)) as connection:
            connection.request("GET", "/api/cloud/events?group=nav&layers=grid_terrain")
            with connection.getresponse() as stream:
                require(stream.status == 200 and stream.getheader("Content-Type").startswith("text/event-stream"),
                        "invalid SSE response")
                event, initial = read_event(stream)
                layer, = initial["layers"]
                require(event == "snapshot" and layer["layer"] == "grid_terrain"
                        and layer["count"] == 0 and layer["status"] == "waiting",
                        "invalid initial SSE terrain snapshot")
                wait_for(publisher.get_subscription_count, "synthetic terrain subscription", process)
                publisher.publish(message)

                def terrain():
                    status, raw = request(port, "/api/map3d.bin?layer=grid_terrain")
                    require(status == 200 and raw[:4] == b"WCB1", "invalid terrain binary response")
                    offset = 8 + struct.unpack_from("<I", raw, 4)[0]
                    header = json.loads(raw[8:offset])
                    if header["count"]:
                        return header, raw[offset:]
                    return None

                header, data = wait_for(terrain, "synthetic terrain data", process)
                require(header["epoch"] == initial["epoch"] and header["version"] > layer["version"],
                        "terrain revision did not advance")
                require(header["layer"] == "grid_terrain" and header["frame_id"] == "synthetic_map"
                        and header["format"] == "triangles" and header["count"] == 3,
                        "terrain binary metadata mismatch")
                expected_vertices = tuple(value for point in vertices for value in (point.x, point.y, point.z))
                expected_rgb = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0)
                require(data == struct.pack("<18f", *expected_vertices, *expected_rgb),
                        "terrain vertices or colors changed on the HTTP path")

                event, update = read_event(stream)
                layer, = update["layers"]
                require(event == "cloud" and update["epoch"] == header["epoch"]
                        and layer["version"] == header["version"] and layer["count"] == 3,
                        "SSE terrain metadata did not match the binary revision")
                require("points" not in layer and "colors" not in layer, "SSE contained terrain geometry")

        old_subscribers = publisher.get_subscription_count()
        record = "/api/mapping/record"
        started = read_json(port, record + "/start", method="POST", body=json.dumps({
            "directory": str(root), "name": "fixture", "topics": [topic],
        }))
        require(started["recording"]["state"] == "recording", "native recording did not start")
        wait_for(lambda: publisher.get_subscription_count() > old_subscribers,
                 "native bag subscription", process)
        for _ in range(5):
            publisher.publish(message)
            time.sleep(.05)
        stopped = read_json(port, record + "/stop", method="POST", body="{}")
        require(stopped["recording"]["state"] == "stopped", "native recording did not finalize")
        bag = root / "fixture"
        require((bag / "metadata.yaml").is_file(), "recording metadata was not written")
        database, = bag.glob("*.db3")
        with closing(sqlite3.connect(database)) as storage:
            count, = storage.execute("SELECT COUNT(*) FROM messages").fetchone()
            require(count > 0, "native recording did not save any synthetic messages")
        print(f"Native recording smoke check: {count} synthetic messages saved.")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def stress_publisher(topic, imu, stop_event=None):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

    rclpy.init(args=[])
    node = rclpy.create_node("native_stress_imu" if imu else "native_stress_terrain")
    try:
        if imu:
            from sensor_msgs.msg import Imu

            publisher = node.create_publisher(Imu, topic, qos_profile_sensor_data)
            start = time.monotonic()
            sample = Imu()
            for i in range(5500):
                if stop_event.is_set():
                    break
                publisher.publish(sample)
                stop_event.wait(max(0.0, start + (i + 1) * .005 - time.monotonic()))
        else:
            from geometry_msgs.msg import Point
            from std_msgs.msg import ColorRGBA
            from visualization_msgs.msg import Marker, MarkerArray

            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            publisher = node.create_publisher(MarkerArray, topic, qos)
            point, color = Point(x=1.0, y=2.0), ColorRGBA(r=1.0, a=1.0)
            markers = []
            for offset in range(0, 83869, 2048):
                count = min(2048, 83869 - offset)
                for name, kind, vertices in (
                    ("terrain_cells", Marker.TRIANGLE_LIST, 6),
                    ("terrain_grid_edges", Marker.LINE_LIST, 8),
                ):
                    marker = Marker(ns=name, type=kind,
                                    points=[point] * (count * vertices),
                                    colors=[color] * (count * vertices))
                    marker.header.frame_id = "stress_map"
                    markers.append(marker)
            deadline = time.monotonic() + 8
            while not publisher.get_subscription_count() and time.monotonic() < deadline:
                time.sleep(.05)
            require(publisher.get_subscription_count(), "stress terrain subscriber did not match")
            publisher.publish(MarkerArray(markers=markers))
            time.sleep(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def validate_large_terrain(port, backend, terrain_topic, imu_topic):
    context = get_context("spawn")
    stop_imu = context.Event()
    imu = context.Process(target=stress_publisher, args=(imu_topic, True, stop_imu))
    terrain = context.Process(target=stress_publisher, args=(terrain_topic, False))
    connection = HTTPConnection("127.0.0.1", port, timeout=3)
    connection.request("GET", "/api/cloud/events?group=nav&layers=grid_terrain")
    with closing(connection), connection.getresponse() as stream:
        event, _ = read_event(stream)
        require(event == "snapshot", "stress test did not establish terrain demand")
        imu.start()
        try:
            sample_large_terrain(port, backend, terrain)
        finally:
            stop_imu.set()
            for producer in (terrain, imu):
                if producer.pid is not None:
                    producer.join(3)
                    if producer.is_alive():
                        producer.terminate()
                        producer.join(3)
                    if producer.is_alive():
                        producer.kill()
                        producer.join(3)


def sample_large_terrain(port, backend, terrain):
    def imu_age():
        row = next(row for row in read_json(port, "/api/diagnostics")["topics"]
                   if row["key"] == "imu")
        return row["last_age_sec"]

    wait_for(lambda: imu_age() is not None, "synthetic IMU samples", backend)
    terrain.start()
    ages = []

    def terrain_ready():
        ages.append(imu_age())
        state = read_json(port, "/api/state")
        return state["layers"]["grid_terrain"]["count"] == 503214

    wait_for(terrain_ready, "large terrain conversion", backend, timeout=20)
    terrain.join(5)
    require(terrain.exitcode == 0, "large terrain publisher failed")
    ages.append(imu_age())
    require(all(age is not None and age < 0.5 for age in ages),
            f"IMU observations stalled during load: {ages!r}")
    print(f"Large terrain smoke check: 503214 vertices, peak observed IMU age {max(ages):.3f}s.")


def prepare_home(root):
    fixture = root / "map" / "smoke"
    fixture.mkdir(parents=True)
    for artifact in ("GlobalMap.pcd", "GroundMap.pcd"):
        (fixture / artifact).touch()
    settings = root / ".config" / "web_control" / "selected_map.json"
    settings.parent.mkdir(parents=True)
    settings.write_text(json.dumps({
        "name": "smoke",
        "ui": {
            "joystick": {"size": 200},
            "layers": {"grid_terrain": False},
            "topic_logging_enabled": False,
        },
    }), encoding="utf-8")


def run(binary, stress):
    with TemporaryDirectory(prefix="web-control-smoke-") as directory:
        root = Path(directory)
        prepare_home(root)
        port = available_port()
        topic_prefix = "/web_control_smoke_" + secrets.token_hex(5)
        terrain_topic = topic_prefix + "/terrain"
        imu_topic = topic_prefix + "/imu"
        ros_environment = {
            "ROS_DOMAIN_ID": str(20 + secrets.randbelow(80)),
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_LOG_DIR": str(root / "ros-logs"),
        }
        os.environ.update(ros_environment)
        environment = dict(os.environ, HOME=str(root))
        config = binary.parents[2] / "share" / "web_control" / "config" / "web_control.yaml"
        if not config.is_file():
            config = Path(__file__).resolve().parents[1] / "config" / "web_control.yaml"
        args = [
            str(binary), "--ros-args",
            "--params-file", str(config),
            "-p", "use_sim_time:=true",
            "-p", f"preview_port:={port}",
            "-p", f"recording_root:={root}",
            "-p", f"managed_log_dir:={root / 'managed-logs'}",
            "-p", f"grid_terrain_topic:={terrain_topic}",
            "-p", f"diagnostics_imu_topic:={imu_topic}",
            "-p", f"teleop_cmd_vel_topic:={topic_prefix}/cmd_vel",
        ]
        logfile = root / "backend.log"
        with logfile.open("wb") as output:
            process = subprocess.Popen(args, env=environment, stdout=output, stderr=subprocess.STDOUT)
            try:
                def responsive():
                    try:
                        return request(port, "/api/state")[0] == 200
                    except OSError:
                        return False

                wait_for(responsive, "the local preview server", process)
                validate_routes(port, root)
                validate_terrain(port, process, terrain_topic, root)
                if stress:
                    validate_large_terrain(port, process, terrain_topic, imu_topic)
            except Exception:
                print(logfile.read_text(errors="replace"), end="")
                raise
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=7)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
        require(process.returncode == 0, f"backend shutdown returned {process.returncode}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path, help="path to the native web control executable")
    parser.add_argument("--stress", action="store_true", help="load a large terrain while publishing IMU samples")
    arguments = parser.parse_args()
    run(arguments.executable.resolve(strict=True), arguments.stress)
    print("Native web control smoke check passed.")
