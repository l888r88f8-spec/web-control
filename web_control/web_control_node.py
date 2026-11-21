from __future__ import annotations

import atexit
import json
import math
import mimetypes
import os
import shlex
import signal
import socket
import subprocess
import threading
import time
import urllib.parse
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

try:
    from action_msgs.msg import GoalStatusArray
    import rclpy
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory,
    )
    from geometry_msgs.msg import (
        PoseArray,
        PoseStamped,
        PoseWithCovarianceStamped,
        Twist,
    )
    from nav_msgs.msg import Odometry, Path as PathMsg
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
except ModuleNotFoundError:
    rclpy = None
    GoalStatusArray = None
    PackageNotFoundError = Exception
    get_package_share_directory = None
    PoseStamped = None
    PoseArray = None
    Twist = None
    PoseWithCovarianceStamped = None
    Odometry = None
    PathMsg = None
    Imu = None
    PointCloud2 = None
    point_cloud2 = None
    QoSDurabilityPolicy = None
    QoSHistoryPolicy = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    Node = object

# Keep the new diagnostics IMU type optional and isolated: a partial sensor
# installation must not disable the pre-existing ROS imports above.
try:
    from sensor_msgs.msg import Imu
except ModuleNotFoundError:
    Imu = None

# LiDAR publish statistics use the standard diagnostic_msgs interface.  Keep
# the import optional so an installation without it still starts the web node
# in its honest publisher-only mode.
try:
    from diagnostic_msgs.msg import DiagnosticArray
except ModuleNotFoundError:
    DiagnosticArray = None

try:
    from action_msgs.srv import CancelGoal
except ModuleNotFoundError:
    CancelGoal = None

# The grid planner's latched terrain display.  Keep this optional so a partial
# installation without visualization_msgs still starts the rest of the web UI.
try:
    from visualization_msgs.msg import MarkerArray
except ModuleNotFoundError:
    MarkerArray = None

try:
    from lio_sam_hesai.srv import SaveMapOptions
except ModuleNotFoundError:
    SaveMapOptions = None

# tf2_msgs is declared in package.xml, but keep it in its own optional import so
# a system that lacks only this message package does not disable every ROS
# import in this module.
try:
    from tf2_msgs.msg import TFMessage
except ModuleNotFoundError:
    TFMessage = None

from web_control.diagnostics import (
    DIAGNOSTICS_TARGET,
    DiagnosticsLogWriter,
    DiagnosticsService,
    LidarPublishStatsMonitor,
    MotionTracker,
    TopicRateMonitor,
    DEFAULT_TOPIC_SPECS,
    filter_topic_logging_entry,
)
from web_control.core import (
    ANSI_ESCAPE_RE,
    CpuSnapshot,
    SaveMapBusyError,
    SaveMapJob,
    SaveMapUnavailableError,
    SourceCache,
    TerminalLogBuffer,
    ViewerRegistry,
    build_managed_launch_command,
    build_system_usage_payload,
    downsample_points,
    project_points_2d,
    quaternion_from_yaw,
    validate_pose_payload,
    validate_save_map_payload,
    yaw_from_quaternion,
)
from web_control.log_store import (
    LogStorageError,
    ManagedLogStore,
    resolve_managed_log_dir,
)
from web_control.pointcloud_runtime import (
    DEFAULT_DEMAND_TTL_SEC,
    CloudResponseCache,
    CloudRevisionHub,
    PointCloudIngestWorker,
    parse_pointcloud2,
    utf8_json_bytes,
)
from web_control.cloud_subscriptions import (
    PLAN_BY_KEY,
    reconcile as reconcile_cloud_demand,
)
from web_control.map_catalog import MapCatalog
from web_control.map_selection import (
    SelectionStore,
    default_selection_path,
    validate_auto_relocalization,
    validate_map_mode,
    validate_topic_logging_enabled,
)
from web_control.navigation_control import (
    CLEARING,
    NavigationConflictError,
    NavigationControl,
    NavigationTimeoutError,
    NavigationUnavailableError,
    PAUSING,
    PENDING_CLEAR,
    PENDING_PAUSE,
    RESUMING,
    select_action_status,
)
from web_control.navigation_store import NavigationStore, default_navigation_path
from web_control.recording import (
    RecordingBusyError,
    RecordingConflictError,
    RecordingUnavailableError,
    RosbagRecorder,
    browse_directories,
    browse_recording_directories,
    resolve_recording_output,
    validate_topic_selection,
)
from web_control.teleop import TeleopController
from web_control.tf_tree import TfTreeCache

LAYER_NAMES = {
    "global_map",
    "local_map",
    "current_lidar",
    "mapcloud",
    "mapground",
    "obstacles",
}
MAPPING_LAYER_NAMES = {"global_map", "ground"}
ACTION_STATUS_NAMES = {
    0: "unknown",
    1: "accepted",
    2: "executing",
    3: "canceling",
    4: "succeeded",
    5: "canceled",
    6: "aborted",
}
STOP_SIGINT_TIMEOUT_SEC = 1.5
STOP_SIGTERM_TIMEOUT_SEC = 0.75
STOP_SIGKILL_TIMEOUT_SEC = 0.25
# Bounded wait for an already-signalled reader to drain the last stdout lines.
STOP_READER_DRAIN_TIMEOUT_SEC = 2.0
_SESSION_UNSPECIFIED = object()
CANCEL_GOAL_TIMEOUT_SEC = 2.0
CLIENT_DISCONNECT_ERRORS = (BrokenPipeError, ConnectionResetError)
MAP_ROOT_DISPLAY = "~/map"
# Hardware startup step 1: bring up the CAN interface before the chassis launch.
# ``sudo -n`` never prompts, so a missing sudo grant fails fast and is logged
# instead of hanging the HTTP request or opening an interactive prompt.
CHASSIS_CAN_PREFLIGHT_COMMANDS: tuple[tuple[str, ...], ...] = (
    ("sudo", "-n", "modprobe", "can"),
    (
        "sudo",
        "-n",
        "ip",
        "link",
        "set",
        "can0",
        "up",
        "type",
        "can",
        "bitrate",
        "500000",
    ),
)
# A managed stop only terminates the chassis launch and leaves can0 up. Linux
# rejects reconfiguring an already-up CAN interface with EBUSY, so a restart
# may accept exactly that result from the ``ip link set can0 up`` prerequisite
# instead of aborting -- but only after a read-only check proves the live
# interface already has the requested type and bitrate.
CHASSIS_CAN_ALREADY_UP_ERROR = "Device or resource busy"
# Read-only, unprivileged verification used only for the EBUSY case. It must
# never reconfigure hardware; the bounded timeout keeps a restart responsive.
CHASSIS_CAN_VERIFY_COMMAND: tuple[str, ...] = (
    "ip",
    "-j",
    "-details",
    "link",
    "show",
    "can0",
)
CHASSIS_CAN_VERIFY_TIMEOUT_SEC = 2.0
CHASSIS_CAN_EXPECTED_BITRATE = 500000


class MapSelectionError(ValueError):
    """Raised when no usable map is selected for a managed launch."""


class MapSelectionConflictError(RuntimeError):
    """Raised when a running managed process blocks a selection change."""


class MapSelectionPersistenceError(RuntimeError):
    """Raised when the selection file cannot be durably updated."""


class TopicLoggingPersistenceError(RuntimeError):
    """Raised when the shared topic-logging preference cannot be saved."""


def _launch_argument_name(token: str) -> str:
    """Return the ``key`` of a ``key:=value`` launch argument, else ``""``."""
    name, separator, _value = token.partition(":=")
    return name if separator else ""


def build_selected_launch_command(
    package: str,
    launch_file: str,
    launch_args: str,
    overrides: dict[str, str],
) -> list[str]:
    """Build a ros2 launch argv, injecting selection values over stale config.

    The configured argument string is split without a shell; any configured
    argument whose key is being injected by the web node is dropped so it
    cannot override the freshly revalidated selection. ``argv`` elements are
    passed verbatim, so map paths are never shell-interpreted.
    """
    managed = set(overrides)
    retained = [
        token
        for token in shlex.split(launch_args)
        if _launch_argument_name(token) not in managed
    ]
    retained.extend(f"{key}:={value}" for key, value in overrides.items())
    return ["ros2", "launch", package, launch_file, *retained]


def sanitize_map_entry(entry: dict[str, Any]) -> dict[str, Any]:
    """Return the browser-safe subset of a catalog entry (no absolute paths)."""
    return {
        "name": entry["name"],
        "tiled": entry.get("tiled_path") is not None,
        "has_scan_context": bool(entry.get("has_scan_context")),
        "warning": entry.get("warning"),
    }


def resolve_map_mode(
    entry: dict[str, Any], mode: str
) -> tuple[str | None, bool, str | None]:
    """Resolve a saved loading preference against one fresh catalog entry.

    ``auto`` keeps the historical tiled-first fallback to GlobalMap.pcd.
    Manual ``tiled`` is never silently downgraded: without a validated tiled
    directory it returns no effective mode and an explicit error.  ``global``
    always selects the full GlobalMap.pcd, even when tiled data is available.
    """
    mode = validate_map_mode(mode)
    if mode == "global":
        return "global", True, None
    if mode == "tiled":
        if entry.get("tiled_path") is not None:
            return "tiled", True, None
        detail = entry.get("warning") or "no usable tiled map"
        return (
            None,
            False,
            f"map {entry['name']!r} has no usable tiled map ({detail}); "
            "choose Auto or Global before starting localization",
        )
    if entry.get("tiled_path") is not None:
        return "tiled", True, None
    return "global", True, None


def auto_relocalization_state(
    entry: dict[str, Any] | None,
    requested: bool,
    unavailable_message: str | None,
) -> dict[str, Any]:
    """Describe the requested/available/effective automatic relocalization state.

    A missing Scan Context database makes the capability unavailable but never
    rewrites the persisted preference: the request stays as saved and only the
    effective launch value becomes false.
    """
    available = bool(entry and entry.get("has_scan_context"))
    return {
        "auto_relocalization": bool(requested),
        "auto_relocalization_available": available,
        "effective_auto_relocalization": bool(requested and available),
        "auto_relocalization_message": None if available else unavailable_message,
    }


class ManagedLaunchProcess:
    def __init__(
        self,
        target: str,
        command: list[str],
        log_buffer: TerminalLogBuffer,
        logger: Any,
        on_exit: Callable[[str], None] | None = None,
        log_store: ManagedLogStore | None = None,
    ) -> None:
        self.target = target
        self.command = command
        self.log_buffer = log_buffer
        self.logger = logger
        self.on_exit = on_exit
        self.log_store = log_store
        self.session: Any | None = None
        self.process: subprocess.Popen[str] | None = None
        self.reader_thread: threading.Thread | None = None
        self.reader_done: threading.Event | None = None
        self.descendant_thread: threading.Thread | None = None
        self.pgid: int | None = None
        self.descendant_pids: set[int] = set()
        # Monotonic per-launch token. A reader/descendant thread only writes
        # state or fires the exit callback while it still owns the current
        # round, so a slow reader from an exited process can never overwrite
        # the running state or exit notification of a newer launch.
        self.generation = 0
        self.lock = threading.RLock()

    def start(self) -> None:
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return
            self._begin_session_locked()
            self._start_locked()

    def _begin_session_locked(self) -> None:
        """Open one persistent session file for this launch (best effort)."""
        if self.log_store is None:
            return
        try:
            session = self.log_store.start_session(self.target, self.command)
        except Exception as exc:  # noqa: BLE001 - reported to the browser
            self.session = None
            self.log_buffer.attach_session(None)
            self.log_buffer.set_persist_error(f"persistent log unavailable: {exc}")
            self.logger.error(
                f"Failed to open persistent log for {self.target}: {exc}"
            )
            return
        self.session = session
        self.log_buffer.attach_session(session)

    def _close_session(self, session: Any, message: str | None) -> None:
        self.log_buffer.finish_session(session, message)
        if self.log_store is not None:
            self.log_store.end_session(session)

    def _start_locked(self) -> None:
        if self.process is not None and self.process.poll() is None:
            return
        self.log_buffer.append("$ " + " ".join(self.command))
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        env["RCUTILS_COLORIZED_OUTPUT"] = "0"
        try:
            self.process = subprocess.Popen(
                self.command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
                start_new_session=True,
            )
            self.pgid = os.getpgid(self.process.pid)
            self.descendant_pids = set()
        except FileNotFoundError as exc:
            self.log_buffer.append(f"failed to start: {exc}")
            self.log_buffer.set_process_state(
                enabled=True, running=False, returncode=127
            )
            self._close_session(
                self.session,
                f"[web_control] {self.target} failed to start (exit 127)",
            )
            self.logger.error(f"Failed to start {self.target}: {exc}")
            return
        except Exception as exc:
            self.log_buffer.append(f"failed to start: {exc}")
            self.log_buffer.set_process_state(
                enabled=True, running=False, returncode=1
            )
            self._close_session(
                self.session,
                f"[web_control] {self.target} failed to start (exit 1)",
            )
            self.logger.error(f"Failed to start {self.target}: {exc}")
            return

        self.log_buffer.set_process_state(
            enabled=True, running=True, returncode=None
        )
        self.generation += 1
        generation = self.generation
        process = self.process
        session = self.session
        reader_done = threading.Event()
        self.reader_done = reader_done
        self.reader_thread = threading.Thread(
            target=self._read_output,
            args=(process, generation, session, reader_done),
            name=f"{self.target}_launch_log_reader",
            daemon=True,
        )
        self.reader_thread.start()
        self.descendant_thread = threading.Thread(
            target=self._monitor_descendants,
            args=(process, generation),
            name=f"{self.target}_launch_descendant_monitor",
            daemon=True,
        )
        self.descendant_thread.start()

    def _read_output(
        self,
        process: subprocess.Popen[str],
        generation: int,
        session: Any = _SESSION_UNSPECIFIED,
        done: threading.Event | None = None,
    ) -> None:
        if session is _SESSION_UNSPECIFIED:
            session = self.session
        if done is None:
            done = self.reader_done
        try:
            try:
                if process.stdout is not None:
                    for line in process.stdout:
                        # Write through this reader's own session so a stale
                        # reader can never spill the previous launch into the
                        # new file. An explicit ``None`` stays ``None``.
                        self.log_buffer.append(line, session=session)
                returncode = process.wait()
            except Exception as exc:  # pragma: no cover - defensive reader guard
                self.log_buffer.append(
                    f"log reader failed: {exc}", session=session
                )
                returncode = process.poll()
            stale = False
            with self.lock:
                if generation != self.generation:
                    # A newer launch already owns the buffer state; keep its
                    # running flag and leave its exit notification alone.
                    stale = True
                else:
                    self.log_buffer.set_process_state(
                        enabled=True,
                        running=False,
                        returncode=returncode,
                    )
            if stale:
                # Release the handle but never touch the new round's state.
                self._close_session(session, None)
                return
            self._close_session(
                session,
                f"[web_control] {self.target} exited with code {returncode}",
            )
            # ``on_exit`` reaches back into the node, so never hold this lock
            # while calling it (the node lock is taken before this lock).
            self._notify_exit()
        finally:
            # The reader owns closing its session; signal it only after the
            # final line has been flushed so ``stop`` can safely wait.
            if done is not None:
                done.set()

    def _monitor_descendants(
        self, process: subprocess.Popen[str], generation: int
    ) -> None:
        while process.poll() is None:
            with self.lock:
                if generation != self.generation:
                    return
                self.descendant_pids.update(
                    self._collect_descendant_pids(process.pid)
                )
            time.sleep(0.5)

    def stop(self) -> None:
        with self.lock:
            process = self.process
            if process is None and self.pgid is None:
                return
            if process is not None:
                self.descendant_pids.update(self._collect_descendant_pids(process.pid))
            self.log_buffer.append(f"stopping {self.target}...")
            self._signal_process_group(signal.SIGINT)
            already_exited = process is None or process.poll() is not None
            if not already_exited:
                try:
                    process.wait(timeout=STOP_SIGINT_TIMEOUT_SEC)
                except subprocess.TimeoutExpired:
                    self._signal_process_group(signal.SIGTERM)
                    try:
                        process.wait(timeout=STOP_SIGTERM_TIMEOUT_SEC)
                    except subprocess.TimeoutExpired:
                        self._signal_process_group(signal.SIGKILL)
                        process.kill()
                        try:
                            process.wait(timeout=STOP_SIGKILL_TIMEOUT_SEC)
                        except subprocess.TimeoutExpired:
                            pass
            returncode = None if process is None else process.returncode
            self.log_buffer.set_process_state(
                enabled=True,
                running=False,
                returncode=returncode,
            )
            session = self.session
            reader = self.reader_thread
            done = self.reader_done
        # The reader drains the remaining stdout, then closes its own session
        # and fires the exit callback. Wait for it *outside* ``self.lock`` (the
        # reader needs that lock to settle) and with a bound so a wedged reader
        # can never hang the HTTP handler past the escalation windows.
        if reader is not None and done is not None and reader.is_alive():
            done.wait(timeout=STOP_READER_DRAIN_TIMEOUT_SEC)
        if reader is not None and done is not None and done.is_set():
            # The reader already closed the session and notified exit.
            return
        self._close_session(
            session, f"[web_control] {self.target} stopped (exit {returncode})"
        )
        self._notify_exit()

    def _notify_exit(self) -> None:
        if self.on_exit is None:
            return
        try:
            self.on_exit(self.target)
        except Exception as exc:  # pragma: no cover - defensive callback guard
            self.logger.error(f"Managed exit callback failed for {self.target}: {exc}")

    def _signal_process_group(self, sig: signal.Signals) -> None:
        if self.pgid is not None:
            try:
                os.killpg(self.pgid, sig)
            except ProcessLookupError:
                pass
            except Exception:
                pass
        for pid in sorted(self.descendant_pids):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                continue
            except Exception:
                continue
        if self.process is None:
            return
        try:
            self.process.send_signal(sig)
        except Exception:
            pass

    def is_running(self) -> bool:
        with self.lock:
            return self.process is not None and self.process.poll() is None

    def _collect_descendant_pids(self, root_pid: int) -> set[int]:
        children_by_parent: dict[int, list[int]] = {}
        proc_dir = Path("/proc")
        for entry in proc_dir.iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            try:
                stat = (entry / "stat").read_text(encoding="utf-8")
            except OSError:
                continue
            rparen = stat.rfind(")")
            if rparen == -1:
                continue
            fields = stat[rparen + 2 :].split()
            if len(fields) < 2:
                continue
            try:
                ppid = int(fields[1])
            except ValueError:
                continue
            children_by_parent.setdefault(ppid, []).append(pid)

        descendants: set[int] = set()
        stack = list(children_by_parent.get(root_pid, []))
        while stack:
            pid = stack.pop()
            if pid in descendants:
                continue
            descendants.add(pid)
            stack.extend(children_by_parent.get(pid, []))
        return descendants


class ManagedSequenceLaunchProcess(ManagedLaunchProcess):
    """Managed launch gated by ordered prerequisite commands.

    The prerequisites run one at a time, in order, before the launch command.
    A non-zero prerequisite aborts the sequence and the ROS launch is never
    spawned, so a failed ``sudo``/``ip`` setup cannot leave the chassis half
    started.  Logs, process-group stop, exit notification, and status reporting
    are inherited from :class:`ManagedLaunchProcess`.
    """

    def __init__(
        self,
        target: str,
        prerequisites: list[list[str]],
        command: list[str],
        log_buffer: TerminalLogBuffer,
        logger: Any,
        on_exit: Callable[[str], None] | None = None,
        log_store: ManagedLogStore | None = None,
    ) -> None:
        super().__init__(target, command, log_buffer, logger, on_exit, log_store)
        self.prerequisites = [list(step) for step in prerequisites]
        self.sequence_thread: threading.Thread | None = None
        self.prerequisite_process: subprocess.Popen[str] | None = None
        # Signalled by the sequence thread once the in-flight prerequisite's
        # ``communicate`` returns, so ``stop`` can wait for it without touching
        # the same ``Popen`` concurrently from two threads.
        self.prerequisite_done = threading.Event()
        self.sequence_active = False
        self.sequence_cancelled = False
        # Monotonic per-sequence token. ``stop`` bumps it so a cancelled
        # sequence thread can never spawn another prerequisite or write state
        # that belongs to a newer sequence.
        self.sequence_generation = 0

    def start(self) -> None:
        with self.lock:
            if self.sequence_active:
                return
            if self.process is not None and self.process.poll() is None:
                return
            # One session file spans the CAN preflight and the chassis launch.
            self._begin_session_locked()
            self.sequence_generation += 1
            generation = self.sequence_generation
            self.sequence_active = True
            self.sequence_cancelled = False
            self.prerequisite_process = None
            self.prerequisite_done = threading.Event()
            self.pgid = None
            self.log_buffer.set_process_state(
                enabled=True, running=True, returncode=None
            )
            self.sequence_thread = threading.Thread(
                target=self._run_prerequisite_sequence,
                args=(generation,),
                name=f"{self.target}_prerequisite_sequence",
                daemon=True,
            )
            self.sequence_thread.start()

    def _is_current_locked(self, generation: int) -> bool:
        return (
            self.sequence_generation == generation
            and not self.sequence_cancelled
        )

    def _prerequisite_already_satisfied(
        self, step: list[str], output: str, session: Any = None
    ) -> bool:
        """Return True only when EBUSY is confirmed by read-only CAN details.

        A managed stop leaves the shared CAN interface up, so the next start
        can get EBUSY from the exact ``ip link set can0 up`` command. EBUSY
        alone proves neither the interface type nor its bitrate, so it is
        tolerated only if an unprivileged ``ip -j -details link show can0``
        reports can0 administratively UP, type CAN, and bitrate 500000. Any
        other failure (or a failed/unavailable query) still aborts before the
        ROS launch.
        """
        if not (
            self.target == "chassis"
            and list(step) == list(CHASSIS_CAN_PREFLIGHT_COMMANDS[1])
            and CHASSIS_CAN_ALREADY_UP_ERROR in (output or "")
        ):
            return False

        try:
            verified = subprocess.run(
                list(CHASSIS_CAN_VERIFY_COMMAND),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=CHASSIS_CAN_VERIFY_TIMEOUT_SEC,
                check=False,
            )
        except FileNotFoundError:
            failure = "read-only ip query is unavailable"
        except subprocess.TimeoutExpired:
            failure = (
                "read-only ip query timed out after "
                f"{CHASSIS_CAN_VERIFY_TIMEOUT_SEC:g}s"
            )
        except OSError as exc:
            failure = f"read-only ip query failed: {exc}"
        else:
            if verified.returncode != 0:
                detail = (verified.stderr or verified.stdout or "").strip()
                failure = f"read-only ip query exited {verified.returncode}"
                if detail:
                    failure += f": {detail}"
            else:
                failure = None
                try:
                    links = json.loads(verified.stdout or "")
                except (TypeError, ValueError):
                    failure = "read-only ip query returned invalid JSON"
                else:
                    if (
                        not isinstance(links, list)
                        or len(links) != 1
                        or not isinstance(links[0], dict)
                    ):
                        failure = "read-only ip query returned unexpected JSON"
                    else:
                        link = links[0]
                        flags = link.get("flags")
                        if link.get("ifname") != "can0":
                            failure = "read-only ip query did not report can0"
                        elif not isinstance(flags, list) or "UP" not in flags:
                            failure = "can0 is administratively DOWN"
                        else:
                            linkinfo = link.get("linkinfo")
                            if not isinstance(linkinfo, dict):
                                linkinfo = {}
                            link_type = link.get("link_type")
                            info_kind = linkinfo.get("info_kind")
                            if link_type != "can" or info_kind != "can":
                                failure = (
                                    "can0 is not a CAN interface "
                                    f"(link_type={link_type!r}, "
                                    f"info_kind={info_kind!r})"
                                )
                            else:
                                info_data = linkinfo.get("info_data")
                                if not isinstance(info_data, dict):
                                    info_data = {}
                                bitrate = info_data.get("bitrate")
                                if bitrate is None:
                                    bittiming = info_data.get("bittiming")
                                    if isinstance(bittiming, dict):
                                        bitrate = bittiming.get("bitrate")
                                try:
                                    numeric_bitrate = float(bitrate)
                                except (TypeError, ValueError):
                                    failure = (
                                        "can0 bitrate is unavailable "
                                        f"({bitrate!r})"
                                    )
                                else:
                                    if (
                                        numeric_bitrate
                                        != CHASSIS_CAN_EXPECTED_BITRATE
                                    ):
                                        failure = (
                                            f"can0 bitrate is {bitrate}, "
                                            "expected "
                                            f"{CHASSIS_CAN_EXPECTED_BITRATE}"
                                        )

        if failure is not None:
            self.log_buffer.append(
                f"{' '.join(step)}: EBUSY is not enough ({failure}); "
                f"not starting {self.target}",
                session=session,
            )
            return False
        return True

    def _run_prerequisite_sequence(self, generation: int) -> None:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        env["RCUTILS_COLORIZED_OUTPUT"] = "0"
        with self.lock:
            # Pin this round's session: a later ``stop``/newer round may replace
            # or clear ``self.session`` while this thread is still running.
            session = self.session
        for step in self.prerequisites:
            # Cancellation, spawn, and registration happen under one lock hold.
            # Otherwise ``stop`` could observe the cancelled flag but no
            # registered process while a freshly forked prerequisite is already
            # running, then return and leave that command unattended.
            with self.lock:
                if not self._is_current_locked(generation):
                    return
                self.log_buffer.append("$ " + " ".join(step), session=session)
                failure = None
                try:
                    process = subprocess.Popen(
                        step,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        bufsize=1,
                        env=env,
                        start_new_session=True,
                    )
                except FileNotFoundError as exc:
                    self.log_buffer.append(
                        f"failed to start: {exc}", session=session
                    )
                    process = None
                    failure = 127
                except Exception as exc:
                    self.log_buffer.append(
                        f"failed to start: {exc}", session=session
                    )
                    process = None
                    failure = 1
                if process is not None:
                    self.prerequisite_process = process
                    done = threading.Event()
                    self.prerequisite_done = done
                    try:
                        self.pgid = os.getpgid(process.pid)
                    except OSError:
                        self.pgid = None
            if process is None:
                # Settle outside the lock: the exit callback takes the node
                # lock, which must not be acquired while holding this one.
                self._finish_sequence(generation, failure)
                return
            try:
                try:
                    output, _ = process.communicate()
                except Exception as exc:  # pragma: no cover - defensive guard
                    output = None
                    self.log_buffer.append(
                        f"command failed: {exc}", session=session
                    )
                # Record this round's captured output *before* waking ``stop``:
                # once released it may close the session for a cancelled
                # preflight, so a later write would be dropped.
                if output:
                    self.log_buffer.append(output, session=session)
            finally:
                # Wake ``stop`` even when this sequence has already gone stale.
                done.set()
            with self.lock:
                if self.sequence_generation != generation:
                    # ``stop`` (or a newer sequence) already owns the state.
                    return
                if self.prerequisite_process is process:
                    self.prerequisite_process = None
                    self.pgid = None
                cancelled = self.sequence_cancelled
            if process.returncode != 0:
                if self._prerequisite_already_satisfied(step, output, session):
                    self.log_buffer.append(
                        f"{' '.join(step)}: can0 already up with verified "
                        "UP/CAN/500000 bitrate; continuing",
                        session=session,
                    )
                else:
                    self.log_buffer.append(
                        f"{' '.join(step)} failed (exit {process.returncode}); "
                        f"not starting {self.target}",
                        session=session,
                    )
                    self._finish_sequence(generation, process.returncode)
                    return
            if cancelled:
                self._finish_sequence(generation, None)
                return
        with self.lock:
            if not self._is_current_locked(generation):
                return
            self.sequence_active = False
            self._start_locked()

    def _finish_sequence(self, generation: int, returncode: int | None) -> None:
        with self.lock:
            if self.sequence_generation != generation:
                return
            self.sequence_active = False
            self.prerequisite_process = None
            self.pgid = None
            self.log_buffer.set_process_state(
                enabled=True, running=False, returncode=returncode
            )
            settle = self.sequence_generation == generation
            session = self.session
        if settle:
            self._close_session(
                session,
                f"[web_control] {self.target} prerequisite sequence ended "
                f"(exit {returncode})",
            )
            self._notify_exit()

    def _signal_prerequisite(
        self, prerequisite: Any, pgid: int | None, sig: signal.Signals
    ) -> None:
        if prerequisite is None:
            return
        if pgid is None:
            try:
                pgid = os.getpgid(prerequisite.pid)
            except Exception:
                pgid = None
        if pgid is not None:
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                pass
            except Exception:
                pass
        sender = getattr(prerequisite, "send_signal", None)
        if sender is not None:
            try:
                sender(sig)
            except ProcessLookupError:
                pass
            except Exception:
                pass
        elif sig == signal.SIGKILL:
            try:
                prerequisite.kill()
            except Exception:
                pass

    def _terminate_prerequisite(
        self, prerequisite: Any, done: threading.Event, pgid: int | None
    ) -> None:
        """Stop the in-flight prerequisite's process group, waiting to finish.

        A graceful SIGINT is given the same bounded window as a managed launch;
        an unresponsive command is escalated to SIGTERM then SIGKILL so ``stop``
        cannot hang the HTTP handler.
        """
        if prerequisite is None:
            return
        self._signal_prerequisite(prerequisite, pgid, signal.SIGINT)
        if done.wait(timeout=STOP_SIGINT_TIMEOUT_SEC):
            return
        self._signal_prerequisite(prerequisite, pgid, signal.SIGTERM)
        if done.wait(timeout=STOP_SIGTERM_TIMEOUT_SEC):
            return
        self._signal_prerequisite(prerequisite, pgid, signal.SIGKILL)
        done.wait(timeout=STOP_SIGKILL_TIMEOUT_SEC)

    def stop(self) -> None:
        with self.lock:
            sequence_active = self.sequence_active
            if not sequence_active:
                stopped_generation = None
                prerequisite = None
                done = None
                pgid = None
                cancelled_session = None
            else:
                self.sequence_cancelled = True
                self.sequence_generation += 1
                stopped_generation = self.sequence_generation
                prerequisite = self.prerequisite_process
                done = self.prerequisite_done
                pgid = self.pgid
                # Pin the cancelled round's session now: a newer round may
                # replace ``self.session`` while the prerequisite settles.
                cancelled_session = self.session
                self.sequence_active = False
                self.prerequisite_process = None
                self.pgid = None
        if not sequence_active:
            super().stop()
            return
        self._terminate_prerequisite(prerequisite, done, pgid)
        settle = False
        with self.lock:
            # Only settle if no newer sequence has taken ownership meanwhile.
            if self.sequence_generation == stopped_generation:
                self.log_buffer.set_process_state(
                    enabled=True, running=False, returncode=None
                )
                settle = True
        if settle:
            self._close_session(
                cancelled_session,
                f"[web_control] {self.target} stopped during preflight",
            )
            self._notify_exit()
        else:
            # A newer round now owns the buffer and running state; never touch
            # it, but do release the cancelled round's file handle so its log
            # does not stay active and leak. ``message=None`` avoids appending
            # the old round's status line to the new round's live buffer.
            self._close_session(cancelled_session, None)

    def is_running(self) -> bool:
        with self.lock:
            if self.sequence_active:
                return True
            return self.process is not None and self.process.poll() is None


def normalize_layer_name(name: str) -> str:
    if name not in LAYER_NAMES:
        raise ValueError(f"unknown layer: {name}")
    return name


def normalize_mapping_layer_name(name: str) -> str:
    if name not in MAPPING_LAYER_NAMES:
        raise ValueError(f"unknown mapping layer: {name}")
    return name


_ALL_CLOUD_KEYS = (
    ("nav", "global_map"),
    ("nav", "local_map"),
    ("nav", "current_lidar"),
    ("nav", "mapcloud"),
    ("nav", "mapground"),
    ("nav", "obstacles"),
    ("mapping", "global_map"),
    ("mapping", "ground"),
)

_CLOUD_STREAM_LAYERS = {
    "nav": frozenset(
        {
            "global_map",
            "local_map",
            "current_lidar",
            "mapcloud",
            "mapground",
            "obstacles",
        }
    ),
    "mapping": frozenset({"global_map", "ground"}),
}


def parse_cloud_stream_keys(group: Any, raw_layers: Any) -> tuple[tuple[str, str], ...]:
    """Validate one browser's metadata subscription.

    Only the known navigation/mapping cloud names are accepted.  Omitted
    ``layers`` means every layer in that group, which keeps the stream useful
    for a client that only knows the current mode.
    """
    group_name = str(group or "").strip()
    allowed = _CLOUD_STREAM_LAYERS.get(group_name)
    if allowed is None:
        raise ValueError(f"unknown cloud group: {group_name}")
    if isinstance(raw_layers, str):
        names = [part.strip() for part in raw_layers.split(",")]
    else:
        names = [str(part).strip() for part in (raw_layers or ())]
    names = [name for name in names if name]
    if not names:
        names = sorted(allowed)
    if len(names) > len(allowed):
        raise ValueError("too many cloud layers requested")
    unique = set()
    for name in names:
        if name not in allowed:
            raise ValueError(f"unknown cloud layer: {name}")
        unique.add(name)
    return tuple((group_name, name) for name in sorted(unique))


def advance_cloud_generation(
    owner: Any,
    keys: tuple[tuple[str, str], ...] | list[tuple[str, str]] | None = None,
) -> int:
    """Invalidate queued/in-flight cloud work and serialized bodies.

    Called for every map/nav/mapping clear, manual map change, stop and
    destroy.  A frame submitted before the bump may finish decoding, but its
    generation/key-epoch check rejects it before it can resurrect stale
    geometry.  ``keys`` limits a navigation stop to its own clouds so a
    latched localization map that is still decoding is not lost.
    """
    owner._cloud_generation = int(getattr(owner, "_cloud_generation", 0)) + 1
    generations = getattr(owner, "_cloud_generations", None)
    if generations is None:
        generations = {}
        owner._cloud_generations = generations
    invalidated = list(_ALL_CLOUD_KEYS if keys is None else keys)
    for key in invalidated:
        generations[key] = generations.get(key, 0) + 1
    worker = getattr(owner, "cloud_worker", None)
    if worker is not None:
        worker.invalidate(invalidated)
    responses = getattr(owner, "cloud_responses", None)
    if responses is not None:
        responses.clear()
    return owner._cloud_generation


def _state_generation(owner: Any, key: tuple[str, str]) -> int:
    generations = getattr(owner, "_state_generations", None) or {}
    return int(generations.get(key, 0))


def advance_state_generation(
    owner: Any, keys: tuple[tuple[str, str], ...] | list[tuple[str, str]]
) -> None:
    """Invalidate in-flight path/pose snapshots for affected resources.

    Integer cache versions alone are not a lifecycle identity: a clear and a
    later update can reuse small counters after a restart or map change.  The
    HTTP payloads expose this generation so the browser rejects a response that
    belongs to an older lifecycle.
    """
    generations = getattr(owner, "_state_generations", None)
    if generations is None:
        generations = {}
        owner._state_generations = generations
    for key in keys:
        generations[key] = generations.get(key, 0) + 1


def _all_path_state_keys(owner: Any) -> list[tuple[str, str]]:
    return [("path", topic) for topic in getattr(owner, "paths", {})]


def _all_state_keys(owner: Any) -> list[tuple[str, str]]:
    keys = _all_path_state_keys(owner)
    keys.append(("path", "mapping_path"))
    keys.extend((("pose", "nav"), ("pose", "mapping")))
    return keys


def parse_path_stream_keys(
    owner: Any, raw_paths: Any
) -> tuple[tuple[str, str], ...]:
    """Validate browser-requested path resources against configured topics.

    Path names are never used to create ROS subscriptions; only topics already
    configured on this node may be streamed.
    """
    if isinstance(raw_paths, str):
        names = [part.strip() for part in raw_paths.split(",")]
    else:
        names = [str(part).strip() for part in (raw_paths or ())]
    names = [name for name in names if name]
    if not names:
        return ()
    configured = set(getattr(owner, "paths", {}))
    keys: list[tuple[str, str]] = []
    for name in names:
        if name == "mapping_path":
            if not hasattr(owner, "mapping_path"):
                raise ValueError("mapping path is unavailable")
        elif name not in configured:
            raise ValueError(f"unknown path topic: {name}")
        key = ("path", name)
        if key not in keys:
            keys.append(key)
    return tuple(keys)


def parse_pose_stream_keys(raw_pose: Any) -> tuple[tuple[str, str], ...]:
    if isinstance(raw_pose, str):
        names = [part.strip() for part in raw_pose.split(",")]
    else:
        names = [str(part).strip() for part in (raw_pose or ())]
    names = [name for name in names if name]
    if not names or names == ["0"]:
        return ()
    keys: list[tuple[str, str]] = []
    for name in names:
        if name in ("1", "true", "both"):
            requested = ("nav", "mapping")
        elif name in ("nav", "mapping"):
            requested = (name,)
        else:
            raise ValueError(f"unknown pose resource: {name}")
        for layer in requested:
            key = ("pose", layer)
            if key not in keys:
                keys.append(key)
    return tuple(keys)


def publish_cloud_metadata_if_available(owner: Any) -> None:
    """Notify revision subscribers when this owner has a metadata hub.

    Some callers bind individual node methods without constructing the
    HTTP/SSE hub, so lifecycle helpers must stay optional.
    """
    publish = getattr(owner, "publish_cloud_metadata", None)
    if publish is not None:
        publish()


def _layer_payload_snapshot(
    owner: Any, layer_name: str, mode: str
) -> tuple[dict[str, Any], int]:
    """Read one layer atomically and return payload plus cache generation.

    The projection/downsample for the legacy 2D view runs after the cache lock
    is released; the version is captured under the same lock as the points so
    a response body can never be cached under the wrong generation.
    """
    layer = normalize_layer_name(layer_name)
    now = owner.now_sec()
    key = ("nav", layer)
    with owner.lock:
        cache = owner.layers[layer]
        data = cache.data or {}
        status = cache.status(now)
        frame_id = cache.frame_id
        count = cache.count
        version = cache.version
        generations = getattr(owner, "_cloud_generations", None) or {}
        generation = int(generations.get(key, 0))
        points_3d = data.get("points3d", [])
    hub = getattr(owner, "cloud_hub", None)
    epoch = str(getattr(hub, "epoch", "")) if hub is not None else ""
    if mode == "3d":
        points = points_3d
    else:
        # Legacy /api/map2d compatibility only.  Project XY from the cached
        # 3D points on explicit request without writing a second cache.
        points = project_points_2d(
            downsample_points(points_3d, owner.max_2d_points)
        )
    return (
        {
            "layer": layer,
            "status": status,
            "frame_id": frame_id,
            "count": count,
            "version": version,
            "generation": generation,
            "epoch": epoch,
            "points": points,
        },
        version,
    )


def _mapping_layer_payload_snapshot(
    owner: Any, layer_name: str
) -> tuple[dict[str, Any], int]:
    layer = normalize_mapping_layer_name(layer_name)
    now = owner.now_sec()
    key = ("mapping", layer)
    with owner.lock:
        cache = owner.mapping_layers[layer]
        data = cache.data or {}
        status = cache.status(now)
        frame_id = cache.frame_id
        count = cache.count
        version = cache.version
        generations = getattr(owner, "_cloud_generations", None) or {}
        generation = int(generations.get(key, 0))
        points = data.get("points3d", [])
    hub = getattr(owner, "cloud_hub", None)
    epoch = str(getattr(hub, "epoch", "")) if hub is not None else ""
    return (
        {
            "layer": layer,
            "status": status,
            "frame_id": frame_id,
            "count": count,
            "version": version,
            "generation": generation,
            "epoch": epoch,
            "points": points,
        },
        version,
    )


def success_response(message: str, stamp: float) -> dict[str, Any]:
    return {"ok": True, "message": message, "stamp": stamp}


def action_status_name(status_values: list[int]) -> str:
    if not status_values:
        return "idle"
    return ACTION_STATUS_NAMES.get(int(status_values[-1]), "unknown")


def effective_mapground_status(cache_status: str, seen: bool) -> str:
    """Readiness follows the lifecycle flag, not the display cache.

    Releasing an idle subscription clears the decoded map ground while the
    navigation stack still owns a valid map, so a cache-derived status would
    report "waiting for navigation" during normal operation.  ``seen`` is set
    when a ground frame is applied and reset by the map/navigation lifecycle
    clears.
    """
    return "loaded" if seen else cache_status


def navigation_readiness(odom_status: str, mapground_status: str) -> dict[str, Any]:
    if odom_status == "waiting":
        return {
            "ready": False,
            "status": "waiting_for_relocalization",
            "message": "waiting for relocalization",
        }
    if mapground_status != "loaded":
        return {
            "ready": False,
            "status": "waiting_for_navigation",
            "message": "waiting for navigation",
        }
    return {
        "ready": True,
        "status": "ready",
        "message": "ready",
    }


def error_response(message: str, stamp: float | None = None) -> dict[str, Any]:
    response: dict[str, Any] = {"ok": False, "message": message}
    if stamp is not None:
        response["stamp"] = stamp
    return response


def parse_path_topics(value: Any) -> list[str]:
    if isinstance(value, (list, tuple)):
        items = value
    elif isinstance(value, str):
        items = value.replace(";", ",").split(",")
    else:
        return []
    return [str(item).strip() for item in items if str(item).strip()]


def package_static_dir() -> Path:
    try:
        if get_package_share_directory is None:
            raise PackageNotFoundError()
        return Path(get_package_share_directory("web_control")) / "static"
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1] / "static"


# The reconciler runs on the ROS executor thread so subscription entities
# are created and destroyed where their callbacks are dispatched.
CLOUD_SUBSCRIPTION_TICK_SEC = 0.2
CLOUD_SUBSCRIPTION_RETRY_SEC = 5.0
DEFAULT_CLOUD_SUBSCRIPTION_IDLE_SEC = 5.0
_CLOUD_LAYER_CALLBACKS = frozenset(
    {"pointcloud_callback", "mapping_pointcloud_callback"}
)


class WebControlNode(Node):
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError("ROS 2 Python environment is not available")
        super().__init__("web_control")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("global_map_topic", "/lio_sam/localization/global_map")
        self.declare_parameter("local_map_topic", "/lio_sam/localization/local_map")
        self.declare_parameter(
            "current_lidar_topic", "/lio_sam/localization/current_lidar"
        )
        self.declare_parameter("mapcloud_topic", "mapcloud")
        self.declare_parameter("mapground_topic", "mapground")
        self.declare_parameter("obstacle_topic", "/aggregated_pc")
        self.declare_parameter("odom_topic", "/lio_sam/localization/odometry")
        self.declare_parameter("mapping_global_map_topic", "/lio_sam/mapping/map_global")
        self.declare_parameter("mapping_ground_topic", "/lio_sam/mapping/ground_cloud_global")
        self.declare_parameter("mapping_path_topic", "/lio_sam/mapping/path")
        self.declare_parameter("mapping_odom_topic", "/lio_sam/mapping/odometry")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("goal_topic", "goal_pose_3d")
        self.declare_parameter("action_status_topic", "/p2p_move_base/_action/status")
        self.declare_parameter(
            "cancel_goal_service", "/p2p_move_base/_action/cancel_goal"
        )
        self.declare_parameter("save_map_service", "/lio_sam/save_map_options")
        # Absolute directory tree the recording browser may use. "/" keeps
        # mounted disks available; validation still resolves symlinks/writability.
        self.declare_parameter("recording_root", "/")
        self.declare_parameter("path_topics", "")
        # Predicted local trajectory from the local planner, published as
        # geometry_msgs/PoseArray (not nav_msgs/Path). It is exposed through
        # the same path API/SSE pipeline but needs its own typed subscription.
        # Empty disables it.
        self.declare_parameter("trajectory_topic", "/best_trajectory")
        self.trajectory_topic = str(
            self.get_parameter("trajectory_topic").value
        ).strip()
        # Legacy /api/map2d only: caps the on-request XY projection; point-cloud
        # ingestion never builds or stores a 2D cache.
        self.declare_parameter("max_2d_points_per_layer", 60000)
        self.declare_parameter("max_3d_points_per_layer", 40000)
        # Every cloud layer decodes only while a browser displays it (within
        # cloud_demand_ttl_sec, default 2 s).  The three latched navigation
        # maps keep a 1 Hz output ceiling; idle subscriptions are released
        # after cloud_subscription_idle_sec.
        self.declare_parameter("cloud_demand_ttl_sec", DEFAULT_DEMAND_TTL_SEC)
        # Seconds after the demand expires before an idle subscription and its
        # decoded cache are released.  0 disables idle release only: creation
        # stays demand-driven and existing subscriptions are kept until the
        # process exits.
        self.declare_parameter(
            "cloud_subscription_idle_sec", DEFAULT_CLOUD_SUBSCRIPTION_IDLE_SEC
        )
        self.declare_parameter("stale_timeout_sec", 2.0)
        self.declare_parameter("localization_launch_package", "lio_sam_hesai")
        self.declare_parameter(
            "localization_launch_file", "localization_with_nonground.launch.py"
        )
        self.declare_parameter("localization_launch_args", "use_rviz:=false")
        self.declare_parameter("navigation_launch_package", "p2p_move_base")
        self.declare_parameter(
            "navigation_launch_file", "p2p_move_base_localization.launch.py"
        )
        self.declare_parameter("navigation_launch_args", "")
        self.declare_parameter("mapping_launch_package", "lio_sam_hesai")
        self.declare_parameter("mapping_launch_file", "mapping.launch.py")
        self.declare_parameter("mapping_launch_args", "use_rviz:=false")
        self.declare_parameter("chassis_launch_package", "hunter_base")
        self.declare_parameter("chassis_launch_file", "hunter_base.launch.py")
        self.declare_parameter("chassis_launch_args", "")
        self.declare_parameter("imu_launch_package", "fdilink_ahrs")
        self.declare_parameter("imu_launch_file", "ahrs_driver.launch.py")
        self.declare_parameter("imu_launch_args", "")
        self.declare_parameter("lidar_launch_package", "hesai_ros_driver")
        self.declare_parameter("lidar_launch_file", "start.py")
        self.declare_parameter("lidar_launch_args", "")
        self.declare_parameter("managed_log_lines", 400)
        self.declare_parameter("managed_log_dir", "")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("tf_static_topic", "/tf_static")
        self.declare_parameter("tf_stale_timeout_sec", 2.0)
        self.declare_parameter("tf_cache_size", 4096)
        self.declare_parameter("viewer_timeout_sec", 10.0)
        self.declare_parameter("teleop_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("teleop_max_linear_speed", 1.5)
        self.declare_parameter("teleop_wheelbase", 0.550)
        self.declare_parameter("teleop_max_steer_angle", 0.3270520925102273)
        self.declare_parameter("teleop_steering_exponent", 0.5)
        self.declare_parameter("teleop_deadzone", 0.08)
        self.declare_parameter("teleop_command_timeout", 0.4)
        self.declare_parameter("teleop_publish_rate", 20.0)
        self.declare_parameter("diagnostics_enabled", True)
        self.declare_parameter("diagnostics_imu_topic", "/imu")
        self.declare_parameter("diagnostics_lidar_topic", "/lidar_points")
        self.declare_parameter(
            "diagnostics_lidar_stats_topic", "/lidar_points/statistics"
        )
        self.declare_parameter("diagnostics_lidar_receive_enabled", False)
        self.declare_parameter("diagnostics_lidar_stats_stale_timeout_sec", 3.0)
        self.declare_parameter("diagnostics_chassis_odom_topic", "/odom1")
        self.declare_parameter("diagnostics_window_sec", 5.0)
        self.declare_parameter("diagnostics_stale_timeout_sec", 2.0)
        self.declare_parameter("diagnostics_sample_period_sec", 1.0)
        self.declare_parameter("diagnostics_snapshot_period_sec", 5.0)
        self.declare_parameter("diagnostics_publisher_refresh_sec", 1.0)
        self.declare_parameter("diagnostics_max_log_bytes", 10 * 1024 * 1024)
        self.declare_parameter("diagnostics_retain_days", 7)
        self.declare_parameter("diagnostics_retain_bytes", 100 * 1024 * 1024)
        self.declare_parameter("motion_start_threshold_mps", 0.05)
        self.declare_parameter("motion_stop_threshold_mps", 0.02)
        self.declare_parameter("motion_debounce_sec", 0.3)
        self.declare_parameter("motion_max_gap_sec", 0.5)
        self.declare_parameter("motion_max_speed_mps", 3.0)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.max_2d_points = int(self.get_parameter("max_2d_points_per_layer").value)
        self.max_3d_points = int(self.get_parameter("max_3d_points_per_layer").value)
        self.cloud_demand_ttl = max(
            0.0, float(self.get_parameter("cloud_demand_ttl_sec").value)
        )
        self.cloud_subscription_idle = max(
            0.0,
            float(self.get_parameter("cloud_subscription_idle_sec").value),
        )
        self.stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        self.managed_log_lines = int(self.get_parameter("managed_log_lines").value)
        self.viewer_timeout = float(self.get_parameter("viewer_timeout_sec").value)
        self.teleop = TeleopController(
            max_linear_speed=float(
                self.get_parameter("teleop_max_linear_speed").value
            ),
            wheelbase=float(self.get_parameter("teleop_wheelbase").value),
            max_steer_angle=float(
                self.get_parameter("teleop_max_steer_angle").value
            ),
            steering_exponent=float(
                self.get_parameter("teleop_steering_exponent").value
            ),
            deadzone=float(self.get_parameter("teleop_deadzone").value),
            command_timeout=float(
                self.get_parameter("teleop_command_timeout").value
            ),
            publish_rate=float(self.get_parameter("teleop_publish_rate").value),
        )
        self.static_dir = package_static_dir()
        self.log_store = ManagedLogStore(
            resolve_managed_log_dir(self.get_parameter("managed_log_dir").value),
            logger=self.get_logger(),
        )
        self.diagnostics_enabled = bool(
            self.get_parameter("diagnostics_enabled").value
        )
        # Raw PointCloud2 receipt is opt-in because it copies every full cloud
        # over DDS even with raw=True.  Presence-only monitoring stays enabled.
        self.diagnostics_lidar_receive_enabled = bool(
            self.get_parameter("diagnostics_lidar_receive_enabled").value
        )
        self.diagnostics_lidar_stats_topic = str(
            self.get_parameter("diagnostics_lidar_stats_topic").value
        )
        self.diagnostics_lidar_stats_stale_timeout = max(
            0.5,
            float(
                self.get_parameter(
                    "diagnostics_lidar_stats_stale_timeout_sec"
                ).value
            ),
        )
        self.diagnostics_sample_period = max(
            0.1, float(self.get_parameter("diagnostics_sample_period_sec").value)
        )
        self.diagnostics_snapshot_period = max(
            1.0, float(self.get_parameter("diagnostics_snapshot_period_sec").value)
        )
        self.diagnostics_publisher_refresh = max(
            0.5, float(self.get_parameter("diagnostics_publisher_refresh_sec").value)
        )
        diagnostics_topics = {
            "imu": str(self.get_parameter("diagnostics_imu_topic").value),
            "lidar": str(self.get_parameter("diagnostics_lidar_topic").value),
            "odom": str(self.get_parameter("diagnostics_chassis_odom_topic").value),
        }
        diagnostics_types = {
            "imu": Imu is not None,
            "lidar": PointCloud2 is not None,
            "odom": Odometry is not None,
        }
        topic_window = float(self.get_parameter("diagnostics_window_sec").value)
        topic_stale = float(
            self.get_parameter("diagnostics_stale_timeout_sec").value
        )
        monitors = {
            spec["key"]: TopicRateMonitor(
                spec["key"],
                spec["label"],
                diagnostics_topics[spec["key"]],
                spec["message_type"],
                window_sec=topic_window,
                stale_timeout_sec=topic_stale,
                enabled=self.diagnostics_enabled,
                available=diagnostics_types[spec["key"]],
                receive_enabled=(
                    self.diagnostics_lidar_receive_enabled
                    if spec["key"] == "lidar"
                    else True
                ),
            )
            for spec in DEFAULT_TOPIC_SPECS
        }
        self.diagnostics_service = DiagnosticsService(
            monitors=monitors,
            motion=MotionTracker(
                source_topic=diagnostics_topics["odom"],
                start_threshold_mps=float(
                    self.get_parameter("motion_start_threshold_mps").value
                ),
                stop_threshold_mps=float(
                    self.get_parameter("motion_stop_threshold_mps").value
                ),
                debounce_sec=float(
                    self.get_parameter("motion_debounce_sec").value
                ),
                max_gap_sec=float(self.get_parameter("motion_max_gap_sec").value),
                max_speed_mps=float(
                    self.get_parameter("motion_max_speed_mps").value
                ),
                stale_timeout_sec=topic_stale,
            ),
            lidar_stats=LidarPublishStatsMonitor(
                self.diagnostics_lidar_stats_topic,
                diagnostics_topics["lidar"],
                stale_timeout_sec=self.diagnostics_lidar_stats_stale_timeout,
                enabled=self.diagnostics_enabled,
                available=DiagnosticArray is not None,
            ),
        )
        self.diagnostics_log = DiagnosticsLogWriter(
            self.log_store,
            max_bytes=int(
                self.get_parameter("diagnostics_max_log_bytes").value
            ),
            retain_days=int(self.get_parameter("diagnostics_retain_days").value),
            retain_bytes=int(
                self.get_parameter("diagnostics_retain_bytes").value
            ),
        )
        self._diagnostics_started = False
        self._diagnostics_last_snapshot = 0.0
        self._diagnostics_last_publisher_check = 0.0
        self._diagnostics_log_error = None
        # Serializes flag changes with every log write.  A disable POST either
        # wins the lock before a payload is formatted or waits for it, so no
        # topic payload can be written after the POST reports success.
        self._diagnostics_lock = threading.RLock()
        self.topic_logging_enabled = True
        self._diagnostics_closed = False
        self.tf_cache = TfTreeCache(
            max_edges=int(self.get_parameter("tf_cache_size").value),
            stale_timeout_sec=float(self.get_parameter("tf_stale_timeout_sec").value),
        )
        self.tf_message_type = TFMessage

        self.map_root = Path.home() / "map"
        self.map_catalog = MapCatalog(self.map_root)
        self.map_selection = SelectionStore(default_selection_path())
        self.topic_logging_enabled = self.map_selection.load_topic_logging_enabled()
        self.navigation_store = NavigationStore(default_navigation_path())
        self.navigation_control = NavigationControl(self.navigation_store)
        self.published_goal: dict[str, Any] | None = None

        self.lock = threading.RLock()
        self.layers = {
            "global_map": SourceCache("global_map", self.stale_timeout, static_source=True),
            "local_map": SourceCache("local_map", self.stale_timeout, static_source=True),
            # The localization current scan is a live, map-frame cloud.  It is
            # dynamic: demand-gated and latest-frame coalesced like obstacles.
            "current_lidar": SourceCache("current_lidar", self.stale_timeout),
            "mapcloud": SourceCache("mapcloud", self.stale_timeout, static_source=True),
            "mapground": SourceCache("mapground", self.stale_timeout, static_source=True),
            "obstacles": SourceCache("obstacles", self.stale_timeout, static_source=False),
        }
        self.mapping_layers = {
            "global_map": SourceCache("mapping_global_map", self.stale_timeout, static_source=True),
            "ground": SourceCache("mapping_ground", self.stale_timeout, static_source=True),
        }
        # Navigation readiness latch: set when a map-ground frame is applied,
        # cleared by the navigation/map lifecycle resets.  Independent of the
        # display cache so an idle release cannot report not-ready.
        self.mapground_seen = False
        # One visualization worker for all cloud layers.  Subscription
        # callbacks only replace a pending reference; parsing, finite
        # filtering and downsampling happen here, outside self.lock and the
        # single-threaded ROS executor.
        self._cloud_generation = 0
        self._cloud_generations: dict[tuple[str, str], int] = {}
        # Path/pose lifecycle generations are separate from cloud generations:
        # a path update must not invalidate cloud geometry (or vice versa).
        self._state_generations: dict[tuple[str, str], int] = {}
        # Assigned under ``self.lock`` together with the cache reads, so the
        # revision hub can reject a refresh that returns out of order.
        self._cloud_metadata_order = 0
        self.cloud_responses = CloudResponseCache()
        # Metadata-only revision fan-out for SSE viewers.  The reader never
        # copies or serializes cloud points; the monitor thread notices
        # time-based status transitions (for example active -> stale).
        self.cloud_hub = CloudRevisionHub(self.cloud_metadata_snapshot)
        self.cloud_hub.refresh()
        self.cloud_worker = PointCloudIngestWorker(
            self._apply_pointcloud_result,
            max_points=self.max_3d_points,
            legacy_reader=self._legacy_pointcloud_reader,
            logger=self.get_logger(),
            demand_ttl_sec=self.cloud_demand_ttl,
        )
        self.cloud_worker.start()
        self.odom = SourceCache("odom", self.stale_timeout)
        self.mapping_odom = SourceCache("mapping_odom", self.stale_timeout)
        self.mapping_path = SourceCache("mapping_path", self.stale_timeout)
        self.action_status = SourceCache("action_status", self.stale_timeout)
        self.paths: dict[str, SourceCache] = {}
        self.system_cpu_ticks: CpuSnapshot | None = None
        self.managed_logs = {
            "chassis": TerminalLogBuffer(
                "chassis", max_lines=self.managed_log_lines
            ),
            "imu": TerminalLogBuffer("imu", max_lines=self.managed_log_lines),
            "lidar": TerminalLogBuffer("lidar", max_lines=self.managed_log_lines),
            "localization": TerminalLogBuffer(
                "localization", max_lines=self.managed_log_lines
            ),
            "navigation": TerminalLogBuffer(
                "navigation", max_lines=self.managed_log_lines
            ),
            "mapping": TerminalLogBuffer(
                "mapping", max_lines=self.managed_log_lines
            ),
        }
        self.managed_processes: dict[str, ManagedLaunchProcess] = {}
        self.viewers = ViewerRegistry(timeout_sec=self.viewer_timeout)

        recording_root = str(self.get_parameter("recording_root").value or "/")
        self.recorder = RosbagRecorder(recording_root)
        self.save_map_job = SaveMapJob()
        self.save_map_future: Any | None = None
        self.save_map_client: Any | None = None
        if SaveMapOptions is not None:
            self.save_map_client = self.create_client(
                SaveMapOptions, str(self.get_parameter("save_map_service").value)
            )
        else:
            self.get_logger().warning(
                "lio_sam_hesai SaveMapOptions type is unavailable; "
                "save map requests will be rejected"
            )

        self.cancel_goal_client: Any | None = None
        if CancelGoal is not None:
            self.cancel_goal_client = self.create_client(
                CancelGoal, str(self.get_parameter("cancel_goal_service").value)
            )
        else:
            self.get_logger().warning(
                "action_msgs CancelGoal srv is unavailable; pause and clear "
                "requests will be rejected"
            )

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initialpose_topic").value),
            5,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            5,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("teleop_cmd_vel_topic").value),
            10,
        )
        self.teleop_timer = self.create_timer(
            1.0 / self.teleop.publish_rate, self.teleop_timer_callback
        )

        self.configure_pointcloud_subscriptions()
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            state_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("mapping_odom_topic").value),
            self.mapping_odom_callback,
            state_qos,
        )
        self.create_subscription(
            PathMsg,
            str(self.get_parameter("mapping_path_topic").value),
            self.mapping_path_callback,
            state_qos,
        )
        self.create_subscription(
            GoalStatusArray,
            str(self.get_parameter("action_status_topic").value),
            self.action_status_callback,
            state_qos,
        )

        # Diagnostics are independent of open browser tabs, but the raw LiDAR
        # subscription is opt-in.  The default mode still checks publisher
        # presence from the existing 1 Hz graph query without receiving clouds.
        self.configure_diagnostics_subscriptions()

        # Real /tf + /tf_static subscriptions. /tf is best-effort so it matches
        # both best-effort and reliable publishers; /tf_static is TRANSIENT_LOCAL
        # so latched static transforms are received after a late subscription.
        if TFMessage is not None:
            tf_dynamic_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            tf_static_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                TFMessage,
                str(self.get_parameter("tf_topic").value),
                self.tf_callback,
                tf_dynamic_qos,
            )
            self.create_subscription(
                TFMessage,
                str(self.get_parameter("tf_static_topic").value),
                self.tf_static_callback,
                tf_static_qos,
            )
        else:
            self.get_logger().warning(
                "tf2_msgs/TFMessage is unavailable; the TF tree will stay empty"
            )

        path_topics = parse_path_topics(self.get_parameter("path_topics").value)
        for topic in path_topics:
            if self.trajectory_topic and topic == self.trajectory_topic:
                # The dedicated parameter defines the message type, so a topic
                # mistakenly listed in both is subscribed as a PoseArray below
                # instead of a nav_msgs/Path that could never receive it.
                self.get_logger().warning(
                    f"'{topic}' is listed in path_topics and trajectory_topic; "
                    "subscribing as geometry_msgs/PoseArray"
                )
                continue
            self.paths[topic] = SourceCache(topic, self.stale_timeout)
            self.create_subscription(
                PathMsg,
                topic,
                partial(self.path_callback, topic),
                state_qos,
            )

        # The predicted trajectory is a PoseArray, so it cannot go through
        # path_topics (a nav_msgs/Path subscription would never match it).
        # Reuse the path cache/API/SSE pipeline once its points are extracted.
        if self.trajectory_topic:
            self.paths[self.trajectory_topic] = SourceCache(
                self.trajectory_topic, self.stale_timeout
            )
            self.create_subscription(
                PoseArray,
                self.trajectory_topic,
                partial(self.pose_array_path_callback, self.trajectory_topic),
                state_qos,
            )

        self.httpd: ThreadingHTTPServer | None = None
        self.http_thread: threading.Thread | None = None
        self.configure_managed_processes()
        self.start_http_server()

    def configure_diagnostics_subscriptions(self) -> None:
        """Create diagnostics subscribers, keeping raw LiDAR receive opt-in.

        IMU and chassis odometry use BEST_EFFORT/VOLATILE so they receive from
        both reliable and best-effort publishers.  The driver's small
        ``diagnostic_msgs/DiagnosticArray`` heartbeat is subscribed by default.
        PointCloud2 is subscribed only when
        ``diagnostics_lidar_receive_enabled`` is true, at depth 1 and
        ``raw=True`` so rclpy does not deserialize it into Python points.  Even
        then the DDS/network byte copy remains, so this is not zero-copy.
        """
        self.diagnostics_timer: Any | None = None
        self.diagnostics_lidar_stats_subscription: Any | None = None
        if not self.diagnostics_enabled:
            return
        imu_monitor = self.diagnostics_service.monitors["imu"]
        diagnostics_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        if Imu is not None:
            self.create_subscription(
                Imu,
                imu_monitor.topic,
                self.diagnostics_imu_callback,
                diagnostics_qos,
            )
        stats_monitor = self.diagnostics_service.lidar_stats
        if DiagnosticArray is not None and stats_monitor is not None:
            stats_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self.diagnostics_lidar_stats_subscription = self.create_subscription(
                DiagnosticArray,
                stats_monitor.topic,
                self.diagnostics_lidar_stats_callback,
                stats_qos,
            )
        lidar_monitor = self.diagnostics_service.monitors["lidar"]
        if PointCloud2 is not None and lidar_monitor.receive_enabled:
            lidar_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                PointCloud2,
                lidar_monitor.topic,
                self.diagnostics_lidar_callback,
                lidar_qos,
                raw=True,
            )
        if Odometry is not None:
            self.create_subscription(
                Odometry,
                self.diagnostics_service.monitors["odom"].topic,
                self.diagnostics_odom_callback,
                diagnostics_qos,
            )
        self.diagnostics_timer = self.create_timer(
            self.diagnostics_sample_period,
            self.diagnostics_timer_callback,
        )

    def now_sec(self) -> float:
        return time.time()

    def ros_stamp(self):
        return self.get_clock().now().to_msg()

    def configure_pointcloud_subscriptions(self) -> None:
        """Arm demand-driven subscriptions for the browser cloud layers.

        Nothing is subscribed at start-up.  The reconciler creates one
        subscription per layer while a browser displays it, and teardown
        follows with idle release.  Latched topics replay their last sample on
        re-subscription, so a released map returns without any publisher
        change.
        """
        self.cloud_subscriptions: dict[tuple[str, str], Any] = {}
        self._cloud_subscription_failures: dict[tuple[str, str], float] = {}
        self._cloud_subscription_timer = self.create_timer(
            CLOUD_SUBSCRIPTION_TICK_SEC,
            self.reconcile_cloud_subscriptions,
        )

    def reconcile_cloud_subscriptions(self) -> None:
        """Match live subscriptions to the layers viewers currently display."""
        worker = getattr(self, "cloud_worker", None)
        if worker is None:
            return
        # Demand stamps use the worker clock (time.monotonic).  Mixing in
        # self.now_sec() (wall clock) would make create never fire and release
        # every tick.
        try:
            create_keys, release_keys = reconcile_cloud_demand(
                worker.now(),
                self.cloud_subscriptions.keys(),
                worker.demand_stamps(),
                demand_ttl_sec=self.cloud_demand_ttl,
                # Fallback for nodes constructed before parameters are
                # declared.
                idle_sec=getattr(
                    self,
                    "cloud_subscription_idle",
                    DEFAULT_CLOUD_SUBSCRIPTION_IDLE_SEC,
                ),
            )
        except Exception as exc:
            self.get_logger().warning(f"cloud reconcile failed: {exc}")
            return
        for key in release_keys:
            try:
                self.release_cloud_subscription(key)
            except Exception as exc:
                self.get_logger().warning(
                    f"cloud release {key[0]}/{key[1]} failed: {exc}"
                )
        for key in create_keys:
            try:
                self.create_cloud_subscription(key)
            except Exception as exc:
                self._cloud_subscription_failures[key] = worker.now()
                self.get_logger().warning(
                    f"cloud subscription {key[0]}/{key[1]} failed: {exc}"
                )

    def cloud_layer_qos(self, qos_name: str) -> Any:
        """QoS profile for one plan; ``latched`` must replay on re-subscribe."""
        if qos_name == "latched":
            return QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

    def create_cloud_subscription(self, key: tuple[str, str]) -> bool:
        """Subscribe one cloud layer if a viewer still demands it."""
        plan = PLAN_BY_KEY.get(key)
        worker = getattr(self, "cloud_worker", None)
        if plan is None or worker is None:
            return False
        if key in self.cloud_subscriptions:
            return False
        now = worker.now()
        if not worker.demand_active(key, now=now):
            return False
        failed_at = self._cloud_subscription_failures.get(key)
        if failed_at is not None and now - failed_at < CLOUD_SUBSCRIPTION_RETRY_SEC:
            return False
        message_type = {
            "PointCloud2": PointCloud2,
            "MarkerArray": MarkerArray,
        }.get(plan.message_type)
        if message_type is None:
            return False
        callback = getattr(self, plan.callback, None)
        if callback is None:
            return False
        if plan.callback in _CLOUD_LAYER_CALLBACKS:
            callback = partial(callback, plan.layer)
        try:
            topic_value = self.get_parameter(plan.topic_parameter).value
        except Exception as exc:
            self._cloud_subscription_failures[key] = now
            self.get_logger().warning(
                f"cloud subscription {key[0]}/{key[1]} failed: {exc}"
            )
            return False
        topic = str(topic_value or "").strip()
        if not topic:
            return False
        try:
            subscription = self.create_subscription(
                message_type, topic, callback, self.cloud_layer_qos(plan.qos)
            )
        except Exception as exc:
            self._cloud_subscription_failures[key] = now
            self.get_logger().warning(
                f"cloud subscription {key[0]}/{key[1]} failed: {exc}"
            )
            return False
        self._cloud_subscription_failures.pop(key, None)
        self.cloud_subscriptions[key] = subscription
        return True

    def release_cloud_subscription(self, key: tuple[str, str]) -> bool:
        """Destroy one idle subscription and release everything it produced."""
        subscription = self.cloud_subscriptions.pop(key, None)
        self._cloud_subscription_failures.pop(key, None)
        destroy = getattr(self, "destroy_subscription", None)
        if subscription is not None and destroy is not None:
            try:
                destroy(subscription)
            except Exception as exc:
                self.get_logger().warning(
                    f"cloud unsubscribe {key[0]}/{key[1]} failed: {exc}"
                )
        # Reject an in-flight decode, drop the pending raw frame and its
        # serialized bodies, then release the decoded points themselves.
        advance_cloud_generation(self, [key])
        released = subscription is not None
        with self.lock:
            caches = self.layers if key[0] == "nav" else self.mapping_layers
            cache = caches.get(key[1])
            if cache is not None and cache.data is not None:
                cache.clear()
                released = True
        if released:
            publish_cloud_metadata_if_available(self)
        return subscription is not None

    def release_all_cloud_subscriptions(self) -> None:
        """Destroy every cloud subscription without publishing revisions.

        Called from ``destroy_node`` after the reconciler timer is cancelled
        and before the global generation bump; it neither publishes revisions
        nor clears decoded caches.
        """
        subscriptions = getattr(self, "cloud_subscriptions", None) or {}
        failures = getattr(self, "_cloud_subscription_failures", None)
        destroy = getattr(self, "destroy_subscription", None)
        for key in list(subscriptions):
            subscription = subscriptions.pop(key)
            if destroy is None:
                continue
            try:
                destroy(subscription)
            except Exception:
                pass
        if failures is not None:
            failures.clear()

    def configure_managed_processes(self) -> None:
        for target in self.managed_logs:
            self.managed_logs[target].set_process_state(
                enabled=False, running=False, returncode=None
            )

        chassis_command = build_managed_launch_command(
            str(self.get_parameter("chassis_launch_package").value),
            str(self.get_parameter("chassis_launch_file").value),
            str(self.get_parameter("chassis_launch_args").value),
        )
        imu_command = build_managed_launch_command(
            str(self.get_parameter("imu_launch_package").value),
            str(self.get_parameter("imu_launch_file").value),
            str(self.get_parameter("imu_launch_args").value),
        )
        lidar_command = build_managed_launch_command(
            str(self.get_parameter("lidar_launch_package").value),
            str(self.get_parameter("lidar_launch_file").value),
            str(self.get_parameter("lidar_launch_args").value),
        )
        localization_command = build_managed_launch_command(
            str(self.get_parameter("localization_launch_package").value),
            str(self.get_parameter("localization_launch_file").value),
            str(self.get_parameter("localization_launch_args").value),
        )
        navigation_command = build_managed_launch_command(
            str(self.get_parameter("navigation_launch_package").value),
            str(self.get_parameter("navigation_launch_file").value),
            str(self.get_parameter("navigation_launch_args").value),
        )
        mapping_command = build_managed_launch_command(
            str(self.get_parameter("mapping_launch_package").value),
            str(self.get_parameter("mapping_launch_file").value),
            str(self.get_parameter("mapping_launch_args").value),
        )
        # Insertion order is the operator startup order: hardware (1-3) before
        # localization/navigation/mapping (4-6).
        self.managed_processes["chassis"] = ManagedSequenceLaunchProcess(
            "chassis",
            [list(step) for step in CHASSIS_CAN_PREFLIGHT_COMMANDS],
            chassis_command,
            self.managed_logs["chassis"],
            self.get_logger(),
            on_exit=self.handle_managed_exit,
            log_store=getattr(self, "log_store", None),
        )
        self.managed_processes["imu"] = ManagedLaunchProcess(
            "imu",
            imu_command,
            self.managed_logs["imu"],
            self.get_logger(),
            on_exit=self.handle_managed_exit,
            log_store=getattr(self, "log_store", None),
        )
        self.managed_processes["lidar"] = ManagedLaunchProcess(
            "lidar",
            lidar_command,
            self.managed_logs["lidar"],
            self.get_logger(),
            on_exit=self.handle_managed_exit,
            log_store=getattr(self, "log_store", None),
        )
        self.managed_processes["localization"] = ManagedLaunchProcess(
            "localization",
            localization_command,
            self.managed_logs["localization"],
            self.get_logger(),
            on_exit=self.handle_managed_exit,
            log_store=getattr(self, "log_store", None),
        )
        self.managed_processes["navigation"] = ManagedLaunchProcess(
            "navigation",
            navigation_command,
            self.managed_logs["navigation"],
            self.get_logger(),
            on_exit=self.handle_managed_exit,
            log_store=getattr(self, "log_store", None),
        )
        self.managed_processes["mapping"] = ManagedLaunchProcess(
            "mapping",
            mapping_command,
            self.managed_logs["mapping"],
            self.get_logger(),
            on_exit=self.handle_managed_exit,
            log_store=getattr(self, "log_store", None),
        )

    def clear_navigation_runtime_state(self) -> None:
        with self.lock:
            advance_cloud_generation(
                self,
                [("nav", "mapcloud"), ("nav", "mapground"), ("nav", "obstacles")],
            )
            advance_state_generation(self, _all_path_state_keys(self))
            self.action_status.clear()
            for cache in self.paths.values():
                cache.clear()
            # These clouds come from the navigation process (or its live
            # obstacle debug topic).  Drop them as soon as navigation stops so
            # no stale geometry survives the lifecycle.  global_map/local_map
            # are localization-owned and intentionally remain.
            for name in ("mapcloud", "mapground", "obstacles"):
                if name in self.layers:
                    self.layers[name].clear()
            self.mapground_seen = False
            # Stop is not an operator command: settle the pause/resume state so
            # the browser stops advertising an active goal, while an
            # interrupted Pause or Resume keeps its saved destination paused.
            self.published_goal = None
            self.navigation_control.on_navigation_stopped(self.now_sec())
        publish_cloud_metadata_if_available(self)

    def clear_map_runtime_state(self) -> None:
        with self.lock:
            advance_cloud_generation(
                self,
                [key for key in _ALL_CLOUD_KEYS if key[0] == "nav"],
            )
            advance_state_generation(
                self, _all_path_state_keys(self) + [("pose", "nav")]
            )
            # Every navigation display cloud belongs to the localization or
            # navigation runtime.  A localization restart may load a different
            # map, so the directly-subscribed global map is dropped too rather
            # than leaving an obsolete scene behind.
            for cache in self.layers.values():
                cache.clear()
            self.mapground_seen = False
            self.odom.clear()
            self.action_status.clear()
            for cache in self.paths.values():
                cache.clear()
        publish_cloud_metadata_if_available(self)

    def clear_mapping_runtime_state(self) -> None:
        with self.lock:
            advance_cloud_generation(
                self,
                [key for key in _ALL_CLOUD_KEYS if key[0] == "mapping"],
            )
            advance_state_generation(
                self, [("path", "mapping_path"), ("pose", "mapping")]
            )
            for cache in self.mapping_layers.values():
                cache.clear()
            self.mapping_odom.clear()
            self.mapping_path.clear()
        publish_cloud_metadata_if_available(self)

    def handle_managed_exit(self, target: str) -> None:
        if target == "chassis":
            lifecycle = getattr(
                self, "note_chassis_diagnostics_lifecycle", None
            )
            if callable(lifecycle):
                lifecycle("stop")
        if target == "localization":
            self.clear_map_runtime_state()
        elif target == "navigation":
            self.clear_navigation_runtime_state()
        elif target == "mapping":
            self.clear_mapping_runtime_state()

    def stop_managed_processes(self) -> None:
        threads = [
            threading.Thread(
                target=process.stop,
                name=f"managed_launch_stop_{target}",
                daemon=True,
            )
            for target, process in self.managed_processes.items()
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=3.0)

    def maps_payload(self) -> dict[str, Any]:
        """Return the browser-safe catalog plus the current selection state."""
        maps = [sanitize_map_entry(entry) for entry in self.map_catalog.list_maps()]
        # Name and mode describe one saved selection, so read them under the
        # same lock that serializes select_map writes.  Otherwise a concurrent
        # name/mode update could return a mixed pair from two snapshots.
        with self.lock:
            name = self.map_selection.load()
            mode = self.map_selection.load_mode()
            requested_auto = self.map_selection.load_auto_relocalization()
        if name is None:
            selection: dict[str, Any] = {
                "name": None,
                "valid": False,
                "entry": None,
                "message": "no map selected",
                "mode": mode,
                "effective_mode": None,
                "mode_valid": False,
                "mode_message": "no map selected",
                **auto_relocalization_state(
                    None, requested_auto, "no map selected"
                ),
            }
        else:
            try:
                entry = self.map_catalog.get(name)
            except ValueError as exc:
                selection = {
                    "name": name,
                    "valid": False,
                    "entry": None,
                    "message": str(exc),
                    "mode": mode,
                    "effective_mode": None,
                    "mode_valid": False,
                    "mode_message": (
                        f"selected map {name!r} is unavailable; choose another map"
                    ),
                    **auto_relocalization_state(
                        None,
                        requested_auto,
                        f"selected map {name!r} is unavailable",
                    ),
                }
            else:
                effective_mode, mode_valid, mode_message = resolve_map_mode(entry, mode)
                selection = {
                    "name": name,
                    "valid": True,
                    "entry": sanitize_map_entry(entry),
                    "message": None,
                    "mode": mode,
                    "effective_mode": effective_mode,
                    "mode_valid": mode_valid,
                    "mode_message": mode_message,
                    **auto_relocalization_state(
                        entry,
                        requested_auto,
                        "selected map has no sc_db.bin; automatic relocalization "
                        "is unavailable",
                    ),
                }
        return {
            "ok": True,
            "map_root": MAP_ROOT_DISPLAY,
            "maps": maps,
            "selection": selection,
            "stamp": self.now_sec(),
        }

    def ui_preferences_payload(self) -> dict[str, Any]:
        """Return the validated joystick layout shared by every browser."""
        return {
            "ok": True,
            "joystick": self.map_selection.load_joystick_preferences(),
            "stamp": self.now_sec(),
        }

    def save_ui_preferences(self, payload: Any) -> dict[str, Any]:
        """Validate and atomically merge a complete joystick setting."""
        validated = self.map_selection.save_joystick_preferences(payload)
        response = success_response("ui preferences saved", self.now_sec())
        response["joystick"] = validated
        return response

    def select_map(self, payload: Any) -> dict[str, Any]:
        """Validate and persist a next-start map, loading mode, and/or auto flag."""
        if not isinstance(payload, dict):
            raise ValueError("payload must be an object")
        has_name = "name" in payload
        has_mode = "mode" in payload
        has_auto = "auto_relocalization" in payload
        if not has_name and not has_mode and not has_auto:
            raise ValueError(
                "payload must include name, mode, or auto_relocalization"
            )

        # Resolve the effective name/mode/flag against the catalog, revalidate
        # the running state, and save as one transaction.  Reading the omitted
        # half outside this lock could let concurrent requests combine a name
        # with another request's mode or flag, producing a persisted set that
        # can never launch.
        with self.lock:
            if has_name:
                name = payload.get("name")
                entry = self.map_catalog.get(name)
            else:
                # Mode/flag-only changes apply to the already selected map.
                # Without a selection there is nothing for them to describe.
                name = self.map_selection.load()
                if name is None:
                    raise ValueError(
                        "no map selected; choose a map before changing the "
                        "loading mode or automatic relocalization"
                    )
                try:
                    entry = self.map_catalog.get(name)
                except ValueError as exc:
                    raise ValueError(
                        f"selected map {name!r} is no longer available: {exc}"
                    ) from exc

            mode = (
                validate_map_mode(payload["mode"])
                if has_mode
                else self.map_selection.load_mode()
            )
            requested_auto = (
                validate_auto_relocalization(payload["auto_relocalization"])
                if has_auto
                else self.map_selection.load_auto_relocalization()
            )
            effective_mode, mode_valid, mode_message = resolve_map_mode(entry, mode)
            # A manual tiled request is resolved before anything is written, so
            # a rejected preference leaves the old name, mode, flag, and unknown
            # keys intact.
            if not mode_valid:
                raise ValueError(
                    mode_message or f"loading mode {mode!r} is unavailable"
                )
            # Explicitly enabling a capability this map does not have is a
            # client error.  A previously persisted true is tolerated and
            # simply has no effect, so switching to such a map still works.
            if has_auto and requested_auto and not entry.get("has_scan_context"):
                raise ValueError(
                    "automatic relocalization is unavailable: selected map has "
                    "no sc_db.bin"
                )

            running = [
                target
                for target in ("localization", "navigation")
                if self.managed_processes.get(target) is not None
                and self.managed_processes[target].is_running()
            ]
            if running:
                raise MapSelectionConflictError(
                    "stop "
                    + " and ".join(running)
                    + " before changing the map, loading mode, or automatic "
                    "relocalization"
                )
            try:
                if has_name or has_mode:
                    if has_auto:
                        self.map_selection.save(
                            name,
                            mode if has_mode else None,
                            requested_auto,
                        )
                    else:
                        self.map_selection.save(
                            name, mode if has_mode else None
                        )
                else:
                    # A flag-only POST must not rewrite name or mode.
                    self.map_selection.save_auto_relocalization(requested_auto)
            except OSError as exc:
                raise MapSelectionPersistenceError(
                    f"unable to save map selection: {exc}"
                ) from exc
            if has_name:
                # The next localization launch may publish a different map.
                # Drop the previous latched scene and reject any queued work
                # now so an obsolete map is never shown after the selection
                # changes.  This runs inside the existing select_map lock.
                advance_cloud_generation(
                    self,
                    [key for key in _ALL_CLOUD_KEYS if key[0] == "nav"],
                )
                advance_state_generation(
                    self, _all_path_state_keys(self) + [("pose", "nav")]
                )
                global_cache = self.layers.get("global_map")
                if global_cache is not None:
                    global_cache.clear()
                # The live scan belongs to the previous map/session too.
                current_lidar = self.layers.get("current_lidar")
                if current_lidar is not None:
                    current_lidar.clear()
                odom = getattr(self, "odom", None)
                if odom is not None:
                    odom.clear()
                for cache in getattr(self, "paths", {}).values():
                    cache.clear()
            self.navigation_control.note_map_selected(name, self.now_sec())
            selection = {
                "name": name,
                "valid": True,
                "entry": sanitize_map_entry(entry),
                "message": None,
                "mode": mode,
                "effective_mode": effective_mode,
                "mode_valid": mode_valid,
                "mode_message": mode_message,
                **auto_relocalization_state(
                    entry,
                    requested_auto,
                    "selected map has no sc_db.bin; automatic relocalization "
                    "is unavailable",
                ),
            }
        if has_name:
            # Never publish while node.lock is held: a monitor refresh takes
            # node.lock to read metadata, so doing it here would invert the
            # hub/reader lock order and can deadlock.
            publish_cloud_metadata_if_available(self)
        response = success_response(f"selected map {name}", self.now_sec())
        response["selection"] = selection
        return response

    def require_valid_selection(self) -> dict[str, Any]:
        """Revalidate the saved selection and return its catalog entry."""
        name = self.map_selection.load()
        if name is None:
            raise MapSelectionError(
                "no map selected; choose a map before starting localization or "
                "navigation"
            )
        try:
            return self.map_catalog.get(name)
        except ValueError as exc:
            raise MapSelectionError(
                f"selected map {name!r} is no longer available: {exc}"
            ) from exc

    def managed_start_command(self, target: str) -> list[str]:
        """Build the launch argv for ``target`` from the current selection.

        Localization and navigation revalidate the saved map on every call so a
        stale selection can never silently fall back to a launch default.
        Localization also re-resolves the saved loading mode against the fresh
        catalog; navigation deliberately keeps using the GlobalMap parent.
        Mapping keeps the configured command untouched.
        """
        if target == "localization":
            entry = self.require_valid_selection()
            mode = self.map_selection.load_mode()
            effective_mode, mode_valid, mode_message = resolve_map_mode(entry, mode)
            if not mode_valid or effective_mode is None:
                raise MapSelectionError(
                    mode_message or f"loading mode {mode!r} is unavailable"
                )
            map_path = (
                entry["tiled_path"]
                if effective_mode == "tiled"
                else entry["global_map_path"]
            )
            overrides = {"map_path": str(map_path)}
            # Always pass the resolved flag explicitly: the user preference is
            # authoritative, but a map without a Scan Context database can only
            # start in manual-relocalization mode.  This also drops any stale
            # auto_relocalization value from localization_launch_args.
            auto_requested = self.map_selection.load_auto_relocalization()
            auto_enabled = auto_requested and bool(entry.get("has_scan_context"))
            overrides["auto_relocalization"] = "true" if auto_enabled else "false"
            return build_selected_launch_command(
                str(self.get_parameter("localization_launch_package").value),
                str(self.get_parameter("localization_launch_file").value),
                str(self.get_parameter("localization_launch_args").value),
                overrides,
            )
        if target == "navigation":
            entry = self.require_valid_selection()
            map_root = Path(entry["global_map_path"]).parent
            return build_selected_launch_command(
                str(self.get_parameter("navigation_launch_package").value),
                str(self.get_parameter("navigation_launch_file").value),
                str(self.get_parameter("navigation_launch_args").value),
                {"map_path": str(map_root)},
            )
        return self.managed_processes[target].command

    def note_chassis_diagnostics_lifecycle(self, action: str) -> None:
        service = getattr(self, "diagnostics_service", None)
        tracker = getattr(service, "motion", None)
        if tracker is None:
            return
        try:
            tracker.note_source_lifecycle(action, time.monotonic(), time.time())
        except Exception as exc:  # pragma: no cover - diagnostics must not block control
            self.get_logger().warning(
                f"chassis diagnostics lifecycle reset failed: {exc}"
            )

    def start_managed_target(self, target: str) -> dict[str, Any]:
        if target == "all":
            for item in self.managed_processes:
                self.start_managed_target(item)
            return success_response("managed processes starting", self.now_sec())
        if target == "chassis" and not self.managed_processes[target].is_running():
            lifecycle = getattr(
                self, "note_chassis_diagnostics_lifecycle", None
            )
            if callable(lifecycle):
                lifecycle("start")
        if target not in self.managed_logs:
            raise ValueError(f"unknown managed target: {target}")
        with self.lock:
            process = self.managed_processes[target]
            process.command = self.managed_start_command(target)
            if target == "localization":
                # A restart is a new localization session: never carry the
                # previous session's latched global map, live scan or queued
                # map frames into the new one.
                advance_cloud_generation(
                    self,
                    [("nav", "global_map"), ("nav", "current_lidar")],
                )
                self.layers["global_map"].clear()
                current_lidar = self.layers.get("current_lidar")
                if current_lidar is not None:
                    current_lidar.clear()
            self.managed_logs[target].set_process_state(
                enabled=True,
                running=process.is_running(),
                returncode=None,
            )
            process.start()
        if target == "localization":
            # Publish after releasing node.lock; see select_map.
            publish_cloud_metadata_if_available(self)
        response = success_response(f"{target} starting", self.now_sec())
        response["log_error"] = self.managed_logs[target].session_info().get(
            "persist_error"
        )
        return response

    def stop_managed_target(self, target: str) -> dict[str, Any]:
        if target == "all":
            self.stop_managed_processes()
            return success_response("managed processes stopping", self.now_sec())
        if target not in self.managed_logs:
            raise ValueError(f"unknown managed target: {target}")
        process = self.managed_processes.get(target)
        if process is None:
            self.managed_logs[target].append(f"{target} is not managed by web_control")
            return success_response(f"{target} is not managed", self.now_sec())
        process.stop()
        return success_response(f"{target} stopping", self.now_sec())

    def _enqueue_pointcloud(
        self,
        group: str,
        layer_name: str,
        msg: PointCloud2,
        *,
        clear_on_empty: bool = False,
    ) -> None:
        """Hand a cloud to the worker, or use the legacy path when no worker
        is attached.

        The production node always has ``cloud_worker``.  The synchronous
        branch serves callers that construct the node without a worker,
        including direct callers of ``update_pointcloud_cache``; a live node
        never takes it.
        """
        worker = getattr(self, "cloud_worker", None)
        if worker is None:
            cache = (
                self.layers[layer_name]
                if group == "nav"
                else self.mapping_layers[layer_name]
            )
            self.update_pointcloud_cache(
                cache, msg, clear_on_empty=clear_on_empty
            )
            return
        key = (group, layer_name)
        generations = getattr(self, "_cloud_generations", None)
        generation = 0 if generations is None else generations.get(key, 0)
        worker.submit(
            key,
            msg,
            arrival_stamp=self.now_sec(),
            clear_on_empty=clear_on_empty,
            generation=generation,
        )

    def pointcloud_callback(self, layer_name: str, msg: PointCloud2) -> None:
        # O(1): replace the latest pending frame for this layer.  The decode
        # happens on the visualization worker, never on the ROS executor.
        enqueue = getattr(self, "_enqueue_pointcloud", None)
        if layer_name == "current_lidar":
            # A live scan is a replacement, including an empty scan. Static
            # maps keep their existing empty-message handling below.
            if enqueue is None:
                self.update_pointcloud_cache(
                    self.layers[layer_name], msg, clear_on_empty=True
                )
            else:
                enqueue("nav", layer_name, msg, clear_on_empty=True)
            return
        if enqueue is None:  # bound only to the public callback, without a worker
            self.update_pointcloud_cache(self.layers[layer_name], msg)
            return
        enqueue("nav", layer_name, msg)

    def obstacle_callback(self, msg: PointCloud2) -> None:
        # The aggregate is a live navigation sample, not a latched map.  An
        # empty sample means "no obstacles now" and must erase the previous
        # frame instead of leaving stale red points on screen.
        enqueue = getattr(self, "_enqueue_pointcloud", None)
        if enqueue is None:
            self.update_pointcloud_cache(
                self.layers["obstacles"], msg, clear_on_empty=True
            )
            return
        enqueue("nav", "obstacles", msg, clear_on_empty=True)

    def mapping_pointcloud_callback(self, layer_name: str, msg: PointCloud2) -> None:
        enqueue = getattr(self, "_enqueue_pointcloud", None)
        if enqueue is None:
            self.update_pointcloud_cache(self.mapping_layers[layer_name], msg)
            return
        enqueue("mapping", layer_name, msg)

    def _legacy_pointcloud_reader(self, msg: PointCloud2):
        """Fallback reader for field layouts the NumPy path rejects."""
        if point_cloud2 is None:
            raise ValueError("sensor_msgs_py point_cloud2 is unavailable")
        return point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )

    def _apply_pointcloud_result(
        self,
        group: str,
        layer_name: str,
        points: list[list[float]],
        frame_id: str,
        finite_count: int,
        arrival_stamp: float,
        clear_on_empty: bool,
        generation: int,
    ) -> bool:
        """Apply a decoded result after validating its lifecycle generation.

        Only this small cache update runs under ``self.lock``; the expensive
        decode and list materialization already happened on the worker.
        """
        changed = False
        with self.lock:
            key = (group, layer_name)
            generations = getattr(self, "_cloud_generations", None)
            expected = 0 if generations is None else generations.get(key, 0)
            if generation != expected:
                return False
            caches = self.layers if group == "nav" else self.mapping_layers
            cache = caches.get(layer_name)
            if cache is None:
                return False
            if not points:
                if clear_on_empty:
                    cache.clear()
                    changed = True
                else:
                    return True
            else:
                cache.update(
                    {"points3d": points},
                    stamp=arrival_stamp,
                    frame_id=frame_id,
                    count=finite_count,
                )
                if group == "nav" and layer_name == "mapground":
                    self.mapground_seen = True
                changed = True
        if changed:
            publish = getattr(self, "publish_cloud_metadata", None)
            if publish is not None:
                publish()
        return True

    def update_pointcloud_cache(
        self,
        cache: SourceCache,
        msg: PointCloud2,
        clear_on_empty: bool = False,
    ) -> None:
        points: list[tuple[float, float, float]] = []
        for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))

        if not points:
            if clear_on_empty:
                with self.lock:
                    cache.clear()
                publish = getattr(self, "publish_cloud_metadata", None)
                if publish is not None:
                    publish()
            return

        # Point-cloud ingestion only builds the 3D cache.  The legacy 2D view
        # is projected lazily by layer_payload(), so callbacks never allocate
        # or downsample a second, 2D-only point set.
        points_3d = downsample_points(points, self.max_3d_points)
        payload = {"points3d": [[x, y, z] for x, y, z in points_3d]}
        with self.lock:
            cache.update(
                payload,
                stamp=self.now_sec(),
                frame_id=msg.header.frame_id,
                count=len(points),
            )
        publish = getattr(self, "publish_cloud_metadata", None)
        if publish is not None:
            publish()

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w,
        }
        payload = {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw": yaw_from_quaternion(q),
            "frame_id": msg.header.frame_id,
        }
        with self.lock:
            self.odom.update(payload, stamp=self.now_sec(), frame_id=msg.header.frame_id)
        publish_cloud_metadata_if_available(self)

    def mapping_odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w,
        }
        payload = {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw": yaw_from_quaternion(q),
            "frame_id": msg.header.frame_id,
        }
        with self.lock:
            self.mapping_odom.update(payload, stamp=self.now_sec(), frame_id=msg.header.frame_id)
        publish_cloud_metadata_if_available(self)

    def path_callback(self, topic: str, msg: PathMsg) -> None:
        points = [
            [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ]
            for pose in msg.poses
        ]
        with self.lock:
            self.paths[topic].update(
                {"points": points},
                stamp=self.now_sec(),
                frame_id=msg.header.frame_id,
                count=len(points),
            )
        publish_cloud_metadata_if_available(self)

    def pose_array_path_callback(self, topic: str, msg: PoseArray) -> None:
        """Store a predicted trajectory (geometry_msgs/PoseArray) as a path.

        The local planner publishes ``/best_trajectory`` as a PoseArray of
        future vehicle poses. Only positions are needed, so it shares the same
        points shape and API/SSE pipeline as nav_msgs/Path sources.
        """
        points = [
            [pose.position.x, pose.position.y, pose.position.z]
            for pose in msg.poses
        ]
        with self.lock:
            self.paths[topic].update(
                {"points": points},
                stamp=self.now_sec(),
                frame_id=msg.header.frame_id,
                count=len(points),
            )
        publish_cloud_metadata_if_available(self)

    def mapping_path_callback(self, msg: PathMsg) -> None:
        points = [
            [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ]
            for pose in msg.poses
        ]
        with self.lock:
            self.mapping_path.update(
                {"points": points},
                stamp=self.now_sec(),
                frame_id=msg.header.frame_id,
                count=len(points),
            )
        publish_cloud_metadata_if_available(self)

    def action_status_callback(self, msg: GoalStatusArray) -> None:
        selected = select_action_status(msg.status_list)
        now = self.now_sec()
        if selected is None:
            with self.lock:
                self.action_status.update(
                    {"status": "idle", "status_values": []},
                    stamp=now,
                    frame_id="",
                    count=0,
                )
            self.navigation_control.note_no_active_goals(now)
            with self.lock:
                if not self.navigation_control.has_goal():
                    self.published_goal = None
            return
        with self.lock:
            self.action_status.update(
                {"status": selected["name"], "status_values": selected["values"]},
                stamp=now,
                frame_id="",
                count=len(selected["values"]),
            )
        action = self.navigation_control.note_action_status(
            selected["name"], selected["goal_stamp"], now
        )
        # Keep the node's published goal truthful: drop it once the control has
        # no goal, but never while a clear is still pending or failed.
        with self.lock:
            if not self.navigation_control.has_goal():
                self.published_goal = None
        if action == "retry_cancel":
            token = self.navigation_control.retry_token()
            kind = self.navigation_control.retry_kind()
            if token is not None and kind is not None:
                self._spawn_cancel_retry(kind, token)

    def _cancel_all_goals(self) -> str:
        """Cancel every server goal with a zero UUID.

        Returns ``"accepted"`` when the service accepts the request and reports
        goals canceling, ``"empty"`` when it accepts but matches no goal, and
        ``"rejected"`` for any non-zero ``return_code``.  A raised service
        future is mapped to :class:`NavigationUnavailableError`.
        """
        client = self.cancel_goal_client
        if client is None:
            raise NavigationUnavailableError("cancel goal service unavailable")
        try:
            ready = bool(client.service_is_ready())
        except Exception:
            ready = False
        if not ready:
            raise NavigationUnavailableError("cancel goal service unavailable")
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        try:
            future = client.call_async(request)
        except Exception as exc:
            raise NavigationUnavailableError(
                f"cancel goal service call failed: {exc}"
            ) from exc
        deadline = time.monotonic() + CANCEL_GOAL_TIMEOUT_SEC
        while not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                raise NavigationTimeoutError("cancel goal request timed out")
            time.sleep(0.01)
        try:
            response = future.result()
        except Exception as exc:
            raise NavigationUnavailableError(
                f"cancel goal service call failed: {exc}"
            ) from exc
        if response is None:
            raise NavigationUnavailableError("cancel goal service returned no response")
        if int(getattr(response, "return_code", 0)) != 0:
            return "rejected"
        results = list(getattr(response, "goals_canceling", []) or [])
        return "accepted" if results else "empty"

    def _spawn_cancel_retry(self, kind: str, token: int) -> None:
        """Start the single bounded background cancel retry for ``token``."""
        threading.Thread(
            target=self._retry_startup_cancel,
            args=(kind == PENDING_CLEAR, token),
            name="web_control_retry_cancel",
            daemon=True,
        ).start()

    def _retry_startup_cancel(
        self, clear: bool | None = None, token: int | None = None
    ) -> None:
        """Retry a cancellation that raced goal registration or a restart.

        The retry lease generation and the still-pending intent are revalidated
        under the control lock immediately before the service call, and the lock
        is released before waiting on the future so the single-threaded ROS
        executor can complete it.  ``clear`` selects the Clear registration
        retry so a failure reports a truthful Navigating state with the goal
        retained; a Pause retry uses the same failure handling.
        """
        if clear is None:
            clear = self.navigation_control.pending_kind() == PENDING_CLEAR
        kind = PENDING_CLEAR if clear else PENDING_PAUSE
        if token is None:
            token = self.navigation_control.retry_token()
        try:
            if not self.navigation_control.validate_retry(token, kind):
                return
            try:
                result = self._cancel_all_goals()
            except (NavigationUnavailableError, NavigationTimeoutError):
                self.navigation_control.note_cancel_failed(self.now_sec(), token)
                return
            if result == "accepted":
                self.navigation_control.note_cancel_accepted(token)
            else:
                self.navigation_control.note_cancel_failed(self.now_sec(), token)
        finally:
            self.navigation_control.release_retry(token)
            with self.lock:
                if not self.navigation_control.has_goal():
                    self.published_goal = None

    def state_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            odom_status = self.odom.status(now)
            mapground_status = effective_mapground_status(
                self.layers["mapground"].status(now),
                getattr(self, "mapground_seen", False),
            )
            return {
                "layers": {
                    name: cache.summary(now) for name, cache in self.layers.items()
                },
                "odom": {
                    **self.odom.summary(now),
                    "pose": self.odom.data,
                },
                "paths": {
                    topic: cache.summary(now) for topic, cache in self.paths.items()
                },
                # Lets the browser draw the predicted trajectory distinctly
                # from the global planning path.
                "trajectory_topic": getattr(self, "trajectory_topic", ""),
                "navigation_action": {
                    **self.action_status.summary(now),
                    "data": self.action_status.data
                    or {"status": "idle", "status_values": []},
                },
                "navigation_ready": navigation_readiness(
                    odom_status, mapground_status
                ),
                "viewers": self.viewers.count(now),
                "map_frame": self.map_frame,
                "navigation_control": self.navigation_control.snapshot(
                    self.map_selection.load()
                ),
                "server_time": now,
            }

    def viewer_heartbeat(self, payload: dict[str, Any]) -> dict[str, Any]:
        viewer_id = str(payload.get("viewer_id", "")).strip()
        return {
            **success_response("viewer heartbeat", self.now_sec()),
            "viewers": self.viewers.heartbeat(viewer_id, self.now_sec()),
        }

    def save_map_available(self) -> bool:
        client = self.save_map_client
        if client is None:
            return False
        try:
            return bool(client.service_is_ready())
        except Exception:
            return False

    def start_save_map(self, payload: dict[str, Any]) -> dict[str, Any]:
        # Absolute destinations are forwarded exactly as selected, including
        # /map; the picker and destination validation share the recorder's
        # root. The home directory is guarded, never prepended.
        recorder = getattr(self, "recorder", None)
        root = getattr(recorder, "root", None)
        options = validate_save_map_payload(payload, root=root)
        if not self.save_map_available():
            raise SaveMapUnavailableError("save map service unavailable")
        client = self.save_map_client
        if client is None:
            raise SaveMapUnavailableError("save map service unavailable")

        request = SaveMapOptions.Request()
        request.destination = options["destination"]
        request.resolution = options["resolution"]
        request.ground_resolution = options["ground_resolution"]
        request.save_ground_map = options["save_ground_map"]
        request.save_tiled_map = options["save_tiled_map"]

        token = self.save_map_job.begin(options["destination"], self.now_sec())
        if token is None:
            raise SaveMapBusyError("save map already running")

        try:
            # Async so neither the HTTP thread nor the ROS executor waits on
            # the export; the done callback only updates the job state.
            future = client.call_async(request)
        except Exception as exc:
            message = f"save map request failed: {exc}"
            self.save_map_job.finish(token, False, message, self.now_sec())
            raise SaveMapUnavailableError(message) from exc

        self.save_map_future = future
        future.add_done_callback(partial(self.handle_save_map_result, token))
        return {
            **success_response("map save started", self.now_sec()),
            "available": True,
            "state": self.save_map_job.state(),
            "destination": options["destination"],
        }

    def handle_save_map_result(self, token: int, future: Any) -> None:
        now = self.now_sec()
        if self.save_map_future is future:
            self.save_map_future = None
        try:
            response = future.result()
        except Exception as exc:
            message = f"save map service call failed: {exc}"
            if self.save_map_job.finish(token, False, message, now):
                self.get_logger().warning(message)
            return
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", "") or "")
        if not message:
            message = "map saved" if success else "map save failed"
        if not self.save_map_job.finish(token, success, message, now):
            return
        if success:
            self.get_logger().info(f"Save map: {message}")
        else:
            self.get_logger().error(f"Save map failed: {message}")

    def save_map_status_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        available = self.save_map_available()
        if not available and self.save_map_job.is_running():
            # The service vanished mid-export, so the pending call can never
            # complete; fail it instead of reporting a stuck "running" state.
            self.save_map_job.abandon_running("save map service unavailable", now)
            self.cancel_save_map_future()
        return {
            "available": available,
            **self.save_map_job.snapshot(),
            "stamp": now,
        }

    def cancel_save_map_future(self) -> None:
        future = self.save_map_future
        self.save_map_future = None
        if future is None:
            return
        try:
            if not future.done():
                future.cancel()
        except Exception as exc:
            self.get_logger().warning(f"failed to cancel save map request: {exc}")

    def cleanup_save_map(self) -> None:
        if self.save_map_job.is_running():
            self.save_map_job.abandon_running(
                "web control node shutting down", self.now_sec()
            )
        self.cancel_save_map_future()
        client = self.save_map_client
        self.save_map_client = None
        if client is None:
            return
        try:
            self.destroy_client(client)
        except Exception as exc:
            self.get_logger().warning(f"failed to destroy save map client: {exc}")

    def record_topics_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        topics = [
            {"name": name, "types": list(types)}
            for name, types in self.get_topic_names_and_types()
        ]
        topics.sort(key=lambda item: item["name"])
        return {**success_response("recording topics", now), "topics": topics}

    def record_browse_payload(self, path: str) -> dict[str, Any]:
        now = self.now_sec()
        payload = browse_recording_directories(path, self.recorder.root)
        home = Path(os.path.expanduser("~")).resolve()
        try:
            home.relative_to(self.recorder.root)
            default_directory = str(home)
        except ValueError:
            default_directory = str(self.recorder.root)
        return {
            **success_response("recording directories", now),
            **payload,
            "default_directory": default_directory,
        }

    def save_map_browse_payload(self, path: str) -> dict[str, Any]:
        now = self.now_sec()
        root = self.recorder.root
        payload = browse_directories(path, root, scope="save map")
        home = Path(os.path.expanduser("~")).resolve()
        default_directory = root
        for candidate in (home / "map", home, root):
            try:
                resolved = candidate.resolve()
                resolved.relative_to(root)
            except (OSError, ValueError):
                continue
            if resolved.is_dir():
                default_directory = resolved
                break
        return {
            **success_response("save map directories", now),
            **payload,
            "default_directory": str(default_directory),
        }

    def start_recording(self, payload: dict[str, Any]) -> dict[str, Any]:
        now = self.now_sec()
        if not isinstance(payload, dict):
            raise ValueError("payload must be an object")
        directory = payload.get("directory")
        name = payload.get("name")
        output = resolve_recording_output(directory, name, self.recorder.root)
        available = {
            name: list(types)
            for name, types in self.get_topic_names_and_types()
        }
        topics = validate_topic_selection(payload, available)
        snapshot = self.recorder.start(topics, output, now)
        return {**success_response("recording started", now), "recording": snapshot}

    def stop_recording(self) -> dict[str, Any]:
        now = self.now_sec()
        snapshot = self.recorder.stop(now)
        return {**success_response("recording stop requested", now), "recording": snapshot}

    def recording_status_payload(self) -> dict[str, Any]:
        return self.recorder.status(self.now_sec())

    def mapping_state_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            return {
                "layers": {
                    name: cache.summary(now)
                    for name, cache in self.mapping_layers.items()
                },
                "path": self.mapping_path.summary(now),
                "odom": {
                    **self.mapping_odom.summary(now),
                    "pose": self.mapping_odom.data,
                },
                "map_frame": self.map_frame,
                "save_map": self.save_map_status_payload(),
                "recording": self.recording_status_payload(),
                "server_time": now,
            }

    def pose_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            nav_version = self.odom.version
            mapping_version = self.mapping_odom.version
            nav_generation = _state_generation(self, ("pose", "nav"))
            mapping_generation = _state_generation(self, ("pose", "mapping"))
            payload = {
                "odom": {
                    **self.odom.summary(now),
                    "pose": self.odom.data,
                    "version": nav_version,
                    "generation": nav_generation,
                },
                "mapping_odom": {
                    **self.mapping_odom.summary(now),
                    "pose": self.mapping_odom.data,
                    "version": mapping_version,
                    "generation": mapping_generation,
                },
                "map_frame": self.map_frame,
                "server_time": now,
            }
        hub = getattr(self, "cloud_hub", None)
        epoch = str(getattr(hub, "epoch", "")) if hub is not None else ""
        payload["epoch"] = epoch
        payload["odom"]["epoch"] = epoch
        payload["mapping_odom"]["epoch"] = epoch
        return payload

    def note_cloud_demand(self, group: str, layer_name: str) -> None:
        """Activate the short dynamic-demand window for one cloud layer."""
        worker = getattr(self, "cloud_worker", None)
        if worker is not None:
            worker.note_demand(group, layer_name)

    def cloud_metadata_snapshot(self) -> dict[tuple[str, str], dict[str, Any]]:
        """Return metadata-only revisions for clouds, paths and pose.

        Reads the existing caches under ``self.lock`` but never copies point
        arrays, so this is safe from the SSE/notification paths and never
        triggers JSON serialization.  Pose is a tiny dict carried inline to
        avoid one HTTP request per odometry frame.
        """
        now = self.now_sec()
        metadata: dict[tuple[str, str], dict[str, Any]] = {}
        with self.lock:
            self._cloud_metadata_order = (
                int(getattr(self, "_cloud_metadata_order", 0)) + 1
            )
            snapshot_order = self._cloud_metadata_order
            generations = getattr(self, "_cloud_generations", None) or {}
            groups = (
                ("nav", getattr(self, "layers", {})),
                ("mapping", getattr(self, "mapping_layers", {})),
            )
            for group, caches in groups:
                for layer_name, cache in caches.items():
                    key = (group, layer_name)
                    if key not in _ALL_CLOUD_KEYS:
                        continue
                    metadata[key] = {
                        "group": group,
                        "layer": layer_name,
                        "version": int(cache.version),
                        "generation": int(generations.get(key, 0)),
                        "status": cache.status(now),
                        "frame_id": cache.frame_id,
                        "count": cache.count,
                        "snapshot_order": snapshot_order,
                    }

            for topic, cache in getattr(self, "paths", {}).items():
                key = ("path", topic)
                metadata[key] = {
                    "group": "path",
                    "layer": topic,
                    "version": int(cache.version),
                    "generation": _state_generation(self, key),
                    "status": cache.status(now),
                    "frame_id": cache.frame_id,
                    "count": cache.count,
                    "snapshot_order": snapshot_order,
                }
            mapping_path = getattr(self, "mapping_path", None)
            if mapping_path is not None:
                key = ("path", "mapping_path")
                metadata[key] = {
                    "group": "path",
                    "layer": "mapping_path",
                    "version": int(mapping_path.version),
                    "generation": _state_generation(self, key),
                    "status": mapping_path.status(now),
                    "frame_id": mapping_path.frame_id,
                    "count": mapping_path.count,
                    "snapshot_order": snapshot_order,
                }

            for kind, cache in (
                ("nav", getattr(self, "odom", None)),
                ("mapping", getattr(self, "mapping_odom", None)),
            ):
                if cache is None:
                    continue
                key = ("pose", kind)
                pose = cache.data
                entry = {
                    "group": "pose",
                    "layer": kind,
                    "version": int(cache.version),
                    "generation": _state_generation(self, key),
                    "status": cache.status(now),
                    "frame_id": cache.frame_id,
                    "count": cache.count,
                    "snapshot_order": snapshot_order,
                }
                if isinstance(pose, dict):
                    entry["pose"] = {
                        **pose,
                        "stamp": cache.stamp,
                        "age": (
                            None
                            if cache.stamp is None
                            else max(0.0, now - cache.stamp)
                        ),
                    }
                metadata[key] = entry
        return metadata

    def publish_cloud_metadata(self) -> list[dict[str, Any]]:
        """Push changed revisions to connected browsers; metadata only."""
        hub = getattr(self, "cloud_hub", None)
        if hub is None:
            return []
        return hub.refresh()

    def layer_payload(self, layer_name: str, mode: str) -> dict[str, Any]:
        return _layer_payload_snapshot(self, layer_name, mode)[0]

    def layer_response(self, layer_name: str, mode: str) -> bytes:
        """Return a cached compact JSON body for a navigation cloud layer."""
        cache = getattr(self, "cloud_responses", None)
        if cache is None:
            payload, _version = _layer_payload_snapshot(self, layer_name, mode)
            return utf8_json_bytes(payload)
        # Capture before reading values so a clear between the snapshot and
        # encode cannot let this request resurrect the invalidated body.
        expected_epoch = cache.epoch()
        payload, version = _layer_payload_snapshot(self, layer_name, mode)
        signature = (
            version,
            payload["generation"],
            payload["epoch"],
            payload["status"],
            payload["frame_id"],
            payload["count"],
            self.max_2d_points if mode != "3d" else 0,
        )
        return cache.encode(
            ("nav", payload["layer"], mode),
            signature,
            lambda: utf8_json_bytes(payload),
            expected_epoch=expected_epoch,
        )

    def mapping_layer_payload(self, layer_name: str) -> dict[str, Any]:
        return _mapping_layer_payload_snapshot(self, layer_name)[0]

    def mapping_layer_response(self, layer_name: str) -> bytes:
        cache = getattr(self, "cloud_responses", None)
        if cache is None:
            payload, _version = _mapping_layer_payload_snapshot(self, layer_name)
            return utf8_json_bytes(payload)
        expected_epoch = cache.epoch()
        payload, version = _mapping_layer_payload_snapshot(self, layer_name)
        signature = (
            version,
            payload["generation"],
            payload["epoch"],
            payload["status"],
            payload["frame_id"],
            payload["count"],
        )
        return cache.encode(
            ("mapping", payload["layer"], "3d"),
            signature,
            lambda: utf8_json_bytes(payload),
            expected_epoch=expected_epoch,
        )

    def path_payload(self, topic: str) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            if topic not in self.paths:
                raise ValueError(f"unknown path topic: {topic}")
            cache = self.paths[topic]
            data = cache.data or {"points": []}
            version = cache.version
            key = ("path", topic)
            generation = _state_generation(self, key)
            payload = {
                "topic": topic,
                "status": cache.status(now),
                "frame_id": cache.frame_id,
                "count": cache.count,
                "points": data.get("points", []),
                "version": version,
                "generation": generation,
            }
        hub = getattr(self, "cloud_hub", None)
        payload["epoch"] = str(getattr(hub, "epoch", "")) if hub is not None else ""
        return payload

    def mapping_path_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            data = self.mapping_path.data or {"points": []}
            key = ("path", "mapping_path")
            payload = {
                "topic": "mapping_path",
                "status": self.mapping_path.status(now),
                "frame_id": self.mapping_path.frame_id,
                "count": self.mapping_path.count,
                "points": data.get("points", []),
                "version": self.mapping_path.version,
                "generation": _state_generation(self, key),
            }
        hub = getattr(self, "cloud_hub", None)
        payload["epoch"] = str(getattr(hub, "epoch", "")) if hub is not None else ""
        return payload

    def terminal_logs_payload(self, cursors: dict[str, int] | None = None) -> dict[str, Any]:
        cursors = cursors or {}
        store = getattr(self, "log_store", None)
        return {
            "logs": {
                target: log_buffer.delta(int(cursors.get(target, 0)))
                for target, log_buffer in self.managed_logs.items()
            },
            "sessions": {
                target: log_buffer.session_info()
                for target, log_buffer in self.managed_logs.items()
            },
            "log_dir": store.display_root() if store is not None else None,
        }

    # -- TF tree -----------------------------------------------------------
    def _tf_edge(self, transform: Any) -> dict[str, Any]:
        header = getattr(transform, "header", None)
        stamp = None
        if header is not None:
            stamp_msg = getattr(header, "stamp", None)
            if stamp_msg is not None:
                try:
                    stamp = float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9
                except (TypeError, ValueError, AttributeError):
                    stamp = None
        return {
            "parent": str(getattr(header, "frame_id", "") or ""),
            "child": str(getattr(transform, "child_frame_id", "") or ""),
            "stamp": stamp,
        }

    def tf_callback(self, msg: Any) -> None:
        now = self.now_sec()
        try:
            self.tf_cache.update_dynamic(
                (self._tf_edge(transform) for transform in msg.transforms), now
            )
        except Exception as exc:  # pragma: no cover - defensive ROS guard
            self.get_logger().warning(f"TF dynamic update failed: {exc}")

    def tf_static_callback(self, msg: Any) -> None:
        now = self.now_sec()
        try:
            self.tf_cache.update_static(
                (self._tf_edge(transform) for transform in msg.transforms), now
            )
        except Exception as exc:  # pragma: no cover - defensive ROS guard
            self.get_logger().warning(f"TF static update failed: {exc}")

    def tf_payload(self) -> dict[str, Any]:
        # The cache snapshots and copies under its own lock, so the HTTP thread
        # never formats while a ROS callback is mutating the edge map.
        payload = self.tf_cache.snapshot(self.now_sec())
        payload["topic"] = str(self.get_parameter("tf_topic").value)
        payload["static_topic"] = str(self.get_parameter("tf_static_topic").value)
        payload["ros_available"] = self.tf_message_type is not None
        if self.tf_message_type is None:
            payload["status"] = "unavailable"
            payload["message"] = "tf2_msgs is not available in this environment"
        return payload

    # -- Saved (persistent) logs ------------------------------------------
    def _require_log_target(self, target: Any) -> str:
        name = str(target or "")
        if name == DIAGNOSTICS_TARGET:
            return name
        if name not in self.managed_logs:
            raise ValueError(f"unknown managed target: {name}")
        return name

    def saved_logs_payload(self, target: Any) -> dict[str, Any]:
        name = self._require_log_target(target)
        if name == DIAGNOSTICS_TARGET:
            persist_error = self.diagnostics_log.error
        else:
            persist_error = self.managed_logs[name].session_info().get(
                "persist_error"
            )
        return {
            "ok": True,
            "target": name,
            "log_dir": self.log_store.display_root(),
            "sessions": self.log_store.list_sessions(name),
            "persist_error": persist_error,
            "stamp": self.now_sec(),
        }

    def saved_log_tail(
        self, target: Any, session_id: Any, max_bytes: int = 262144
    ) -> tuple[str, bool]:
        name = self._require_log_target(target)
        text, truncated = self.log_store.read_tail(name, str(session_id or ""), max_bytes)
        return ANSI_ESCAPE_RE.sub("", text), truncated

    def saved_log_download_path(self, target: Any, session_id: Any) -> Path:
        name = self._require_log_target(target)
        return self.log_store.resolve_session_file(name, str(session_id or ""))

    def system_payload(self) -> dict[str, Any]:
        with self.lock:
            previous_ticks = self.system_cpu_ticks
        payload, current_ticks = build_system_usage_payload(
            Path("/proc/stat").read_text(encoding="utf-8"),
            Path("/proc/meminfo").read_text(encoding="utf-8"),
            previous_ticks=previous_ticks,
            stamp=self.now_sec(),
        )
        with self.lock:
            self.system_cpu_ticks = current_ticks
        return payload

    # -- Topic and motion diagnostics -------------------------------------
    def diagnostics_imu_callback(self, msg: Any) -> None:
        try:
            self.diagnostics_service.record_imu(msg, time.monotonic())
        except Exception as exc:  # pragma: no cover - defensive ROS guard
            self.get_logger().warning(f"IMU diagnostics callback failed: {exc}")

    def diagnostics_lidar_callback(self, _raw_msg: Any) -> None:
        # raw=True delivers bytes only; never inspect or deserialize here.
        self.diagnostics_service.record("lidar", time.monotonic())

    def diagnostics_lidar_stats_callback(self, msg: Any) -> None:
        # Small DiagnosticArray heartbeat only.  It never touches the raw
        # point-cloud receive counter or its last-received stamp.
        try:
            self.diagnostics_service.record_lidar_stats(msg, time.monotonic())
        except Exception as exc:  # pragma: no cover - defensive ROS guard
            self.get_logger().warning(
                f"LiDAR publish statistics callback failed: {exc}"
            )

    def diagnostics_odom_callback(self, msg: Any) -> None:
        try:
            self.diagnostics_service.record_odom(msg, time.monotonic())
        except Exception as exc:  # pragma: no cover - defensive ROS guard
            self.get_logger().warning(f"odometry diagnostics callback failed: {exc}")

    def diagnostics_timer_callback(self) -> None:
        now_mono = time.monotonic()
        counts: dict[str, int] = {}
        if (
            now_mono - self._diagnostics_last_publisher_check
            >= self.diagnostics_publisher_refresh
        ):
            self._diagnostics_last_publisher_check = now_mono
            for key, monitor in self.diagnostics_service.monitors.items():
                try:
                    counts[key] = int(self.count_publishers(monitor.topic))
                except Exception:
                    counts[key] = None
            stats_monitor = self.diagnostics_service.lidar_stats
            if stats_monitor is not None:
                try:
                    counts["lidar_stats"] = int(
                        self.count_publishers(stats_monitor.topic)
                    )
                except Exception:
                    counts["lidar_stats"] = None
        payload, events = self.diagnostics_service.tick(now_mono, counts)
        topic_rows = {row["topic"]: row for row in payload["topics"]}
        if not self._diagnostics_started:
            self._diagnostics_started = True
            self._write_diagnostics(
                {
                    "kind": "startup",
                    "source": "diagnostics",
                    "enabled": self.diagnostics_enabled,
                    "lidar_receive_enabled": getattr(
                        self, "diagnostics_lidar_receive_enabled", True
                    ),
                    "source_mode": payload["source_mode"],
                    "topics": payload["topics"],
                    "motion": payload["motion"],
                    "logging": self.diagnostics_log.status(),
                },
                now_mono,
            )
        for event in events:
            row = topic_rows.get(event.get("topic"))
            if row is not None:
                event.setdefault("publisher_count", row["publisher_count"])
                event.setdefault("topic_hz", row["hz"])
                event.setdefault("topic_age_sec", row["last_age_sec"])
                event.setdefault("topic_status", row["status"])
                event.setdefault("status", row["status"])
            event.setdefault("session_id", self.diagnostics_log.session_id)
            # Keep every transition self-contained for offline JSONL review.
            event.setdefault("motion", payload["motion"])
            self._write_diagnostics(event, now_mono)
        if (
            now_mono - self._diagnostics_last_snapshot
            >= self.diagnostics_snapshot_period
        ):
            self._diagnostics_last_snapshot = now_mono
            self._write_diagnostics(
                {
                    "kind": "snapshot",
                    "source": "diagnostics",
                    **payload,
                    "lidar_receive_enabled": getattr(
                        self, "diagnostics_lidar_receive_enabled", True
                    ),
                    "logging": self.diagnostics_log.status(),
                },
                now_mono,
            )

    def _write_diagnostics(self, entry: dict[str, Any], now_mono: float) -> None:
        # Format and write while holding the same lock as a switch change.
        # OFF therefore filters snapshot/startup/shutdown payloads, topic-status
        # rows, IMU observations, and motion-event enrichment at one boundary.
        lock = getattr(self, "_diagnostics_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._diagnostics_lock = lock
        with lock:
            record = filter_topic_logging_entry(
                entry, getattr(self, "topic_logging_enabled", True)
            )
            if record is None:
                return
            try:
                ok = self.diagnostics_log.write(record, now=time.time())
                error = self.diagnostics_log.error
            except Exception as exc:  # pragma: no cover - disk failure must not stop teleop
                ok = False
                error = f"log write failed: {exc}"
        if not ok and error and error != self._diagnostics_log_error:
            self._diagnostics_log_error = error
            self.get_logger().error(f"diagnostics log write failed: {error}")

    def set_topic_logging(self, payload: Any) -> dict[str, Any]:
        """Persist and apply the shared topic/IMU log-writing switch."""
        enabled = validate_topic_logging_enabled(payload)
        with self._diagnostics_lock:
            if enabled == self.topic_logging_enabled:
                response = success_response(
                    "topic logging already "
                    + ("enabled" if enabled else "disabled"),
                    self.now_sec(),
                )
                response["topic_logging_enabled"] = enabled
                response["logging"] = self.diagnostics_log.status()
                return response
            try:
                self.map_selection.save_topic_logging_enabled(enabled)
            except Exception as exc:
                # Never flip the active flag when the shared preference could
                # not be saved; the caller can retry and the UI resyncs.
                raise TopicLoggingPersistenceError(
                    f"could not save topic logging preference: {exc}"
                ) from exc
            self.topic_logging_enabled = enabled
            self._write_diagnostics(
                {
                    "kind": "logging_toggle",
                    "event": "topic_logging",
                    "source": "diagnostics",
                    "topic_logging_enabled": enabled,
                    "enabled": enabled,
                    "message": (
                        "topic/IMU diagnostics log writing enabled"
                        if enabled
                        else "topic/IMU diagnostics log writing disabled"
                    ),
                },
                time.monotonic(),
            )
        response = success_response(
            "topic logging " + ("enabled" if enabled else "disabled"),
            self.now_sec(),
        )
        response["topic_logging_enabled"] = enabled
        response["logging"] = self.diagnostics_log.status()
        return response

    def diagnostics_payload(self) -> dict[str, Any]:
        now_mono = time.monotonic()
        try:
            payload = self.diagnostics_service.snapshot(now_mono)
        except Exception as exc:  # pragma: no cover - defensive HTTP guard
            return {
                "ok": False,
                "status": "unavailable",
                "message": str(exc),
                "stamp": time.time(),
            }
        lock = getattr(self, "_diagnostics_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._diagnostics_lock = lock
        with lock:
            payload["topic_logging_enabled"] = getattr(
                self, "topic_logging_enabled", True
            )
            payload["logging"] = self.diagnostics_log.status()
        payload["enabled"] = self.diagnostics_enabled
        payload["lidar_receive_enabled"] = getattr(
            self, "diagnostics_lidar_receive_enabled", True
        )
        payload["lidar_stats_topic"] = getattr(
            self, "diagnostics_lidar_stats_topic", None
        )
        payload["lidar_stats_stale_timeout_sec"] = getattr(
            self, "diagnostics_lidar_stats_stale_timeout", None
        )
        payload["lidar_stats_available"] = DiagnosticArray is not None
        payload["log_dir"] = self.log_store.display_root()
        payload["sample_period_sec"] = self.diagnostics_sample_period
        payload["snapshot_period_sec"] = self.diagnostics_snapshot_period
        return payload

    def shutdown_diagnostics(self) -> None:
        if not hasattr(self, "diagnostics_log"):
            return
        try:
            now_mono = time.monotonic()
            # tick() drains all motion events queued since the last periodic
            # timer tick and refreshes topic status; write those before the
            # final snapshot so a transition in the last second is never lost.
            final, events = self.diagnostics_service.tick(now_mono)
            motion = final.get("motion") or {}
            for event in events:
                event.setdefault("motion", motion)
                event.setdefault("total_distance_m", motion.get("total_distance_m"))
                event.setdefault("validity", motion.get("validity"))
                self._write_diagnostics(event, now_mono)
            self._write_diagnostics(
                {
                    "kind": "shutdown",
                    "source": "diagnostics",
                    **final,
                    "lidar_receive_enabled": getattr(
                        self, "diagnostics_lidar_receive_enabled", True
                    ),
                    # Keep the final wheel-motion fields directly on the
                    # shutdown row as well as under "motion" for offline review.
                    "motion": motion,
                    "speed_mps": motion.get("speed_mps"),
                    "direction": motion.get("direction"),
                    "segment_distance_m": motion.get("segment_distance_m"),
                    "segment_duration_sec": motion.get("segment_duration_sec"),
                    "total_distance_m": motion.get("total_distance_m"),
                    "validity": motion.get("validity"),
                    "imu_observation": final.get("imu_observation"),
                    "logging": self.diagnostics_log.status(),
                },
                now_mono,
            )
        except Exception:
            pass
        try:
            lock = getattr(self, "_diagnostics_lock", None)
            if lock is not None:
                with lock:
                    self._diagnostics_closed = True
            else:
                self._diagnostics_closed = True
            self.diagnostics_log.close()
        except Exception:
            pass

    def publish_initialpose(self, payload: dict[str, Any]) -> dict[str, Any]:
        pose = validate_pose_payload(payload)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.ros_stamp()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = pose["x"]
        msg.pose.pose.position.y = pose["y"]
        msg.pose.pose.position.z = pose["z"]
        q = quaternion_from_yaw(pose["yaw"])
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initialpose_pub.publish(msg)
        return success_response("initial pose published", self.now_sec())

    def _publish_goal_pose(
        self, pose: dict[str, Any], frame: str | None = None
    ) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.ros_stamp()
        msg.header.frame_id = frame or self.map_frame
        msg.pose.position.x = float(pose["x"])
        msg.pose.position.y = float(pose["y"])
        msg.pose.position.z = float(pose.get("z", 0.0))
        q = quaternion_from_yaw(float(pose["yaw"]))
        msg.pose.orientation.x = q["x"]
        msg.pose.orientation.y = q["y"]
        msg.pose.orientation.z = q["z"]
        msg.pose.orientation.w = q["w"]
        self.goal_pub.publish(msg)

    def apply_teleop_command(self, payload: Any) -> dict[str, Any]:
        """Validate and store one teleop command; the timer publishes it.

        A per-tab sequence number makes late requests harmless: a command older
        than the last accepted one for the same client is reported as stale with
        HTTP 200 so the browser never treats it as a transport error.
        """
        command = self.teleop.apply_command(payload)
        if command is None:
            response = success_response("stale teleop command ignored", self.now_sec())
            response["stale"] = True
            return response
        response = success_response("teleop command accepted", self.now_sec())
        response["stale"] = False
        response["teleop"] = command
        return response

    def publish_teleop_twist(self, command: dict[str, Any]) -> None:
        msg = Twist()
        msg.linear.x = float(command["linear_x"])
        msg.angular.z = float(command["angular_z"])
        self.cmd_vel_pub.publish(msg)

    def teleop_timer_callback(self) -> None:
        command = self.teleop.poll_publish()
        if command is None:
            return
        self.publish_teleop_twist(command)

    def cleanup_teleop(self) -> None:
        """Publish exactly one zero Twist before the node shuts down.

        ``stop`` is terminal, so a second cleanup is a no-op.  A failing
        ``/cmd_vel`` publish is logged rather than raised so ``destroy_node``
        can still tear down the rest of the node.
        """
        command = self.teleop.stop()
        if command is None:
            return
        try:
            self.publish_teleop_twist(command)
        except Exception as exc:
            self.get_logger().error(f"failed to publish teleop stop command: {exc}")

    def pause_navigation(self) -> dict[str, Any]:
        map_name = self.map_selection.load()
        if map_name is None:
            raise NavigationConflictError("no map selected; cannot pause navigation")
        with self.lock:
            goal = dict(self.published_goal) if self.published_goal else None
        if goal is None:
            raise NavigationConflictError("no active navigation goal to pause")
        self.navigation_control.begin_pause(
            goal, self.map_frame, map_name, self.now_sec()
        )
        try:
            result = self._cancel_all_goals()
        except NavigationTimeoutError:
            # Keep the pausing record for polling and schedule exactly one
            # bounded retry so a later Clear is never stuck on an untracked
            # cancel.  A concurrent Clear may have replaced the pending intent,
            # so retry the kind actually stored in the armed lease.
            lease = self.navigation_control.note_cancel_timeout_lease(
                self.now_sec()
            )
            if lease is not None:
                token, kind = lease
                self._spawn_cancel_retry(kind, token)
            raise
        except NavigationUnavailableError:
            self.navigation_control.note_cancel_failed(self.now_sec())
            raise
        if result != "accepted":
            # ``empty`` and ``rejected`` both mean nothing is canceling, so the
            # pause never started; restore the previous navigating state.
            self.navigation_control.note_cancel_failed(self.now_sec())
            raise NavigationConflictError("no active navigation goal to cancel")
        self.navigation_control.note_cancel_accepted()
        response = success_response("pause requested", self.now_sec())
        response["navigation_control"] = self.navigation_control.snapshot(map_name)
        return response

    def resume_navigation(self) -> dict[str, Any]:
        map_name = self.map_selection.load()
        record = self.navigation_control.begin_resume(
            self.now_sec(), map_name, self.map_frame
        )
        try:
            self._publish_goal_pose(record, record["frame"])
        except Exception as exc:
            self.navigation_control.note_resume_publish_failed(self.now_sec())
            raise NavigationUnavailableError(f"resume publish failed: {exc}") from exc
        with self.lock:
            self.published_goal = {
                "x": record["x"],
                "y": record["y"],
                "z": record["z"],
                "yaw": record["yaw"],
            }
        response = success_response("resume published", self.now_sec())
        response["navigation_control"] = self.navigation_control.snapshot(map_name)
        return response

    def clear_navigation(self) -> dict[str, Any]:
        map_name = self.map_selection.load()
        needs_cancel = self.navigation_control.begin_clear(self.now_sec())
        if needs_cancel:
            try:
                result = self._cancel_all_goals()
            except NavigationTimeoutError:
                # Keep clearing/record state for polling and schedule a bounded
                # retry so repeated Clear cannot become permanently 409.  Retry
                # the kind actually stored in the armed lease rather than this
                # caller's intent, which a concurrent command may have changed.
                lease = self.navigation_control.note_cancel_timeout_lease(
                    self.now_sec()
                )
                if lease is not None:
                    token, kind = lease
                    self._spawn_cancel_retry(kind, token)
                raise
            except NavigationUnavailableError:
                self.navigation_control.note_cancel_failed(self.now_sec())
                with self.lock:
                    if not self.navigation_control.has_goal():
                        self.published_goal = None
                raise
            if result == "empty":
                # An empty result may be a registration race; the control keeps
                # clearing and may ask for exactly one bounded retry.
                action = self.navigation_control.note_cancel_empty(self.now_sec())
                if action == "retry_cancel":
                    token = self.navigation_control.retry_token()
                    kind = self.navigation_control.retry_kind()
                    if token is not None and kind is not None:
                        self._spawn_cancel_retry(kind, token)
            elif result == "rejected":
                # The service refused; the goal may still be running, so keep
                # the previous state instead of pretending the clear finished.
                self.navigation_control.note_cancel_failed(self.now_sec())
                with self.lock:
                    if not self.navigation_control.has_goal():
                        self.published_goal = None
                raise NavigationUnavailableError("cancel goal request rejected")
            else:
                self.navigation_control.note_cancel_accepted()
        with self.lock:
            if not self.navigation_control.has_goal():
                self.published_goal = None
        response = success_response("clear requested", self.now_sec())
        response["navigation_control"] = self.navigation_control.snapshot(map_name)
        return response

    def publish_goal(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = self.map_selection.load()
        snapshot = self.navigation_control.snapshot(map_name)
        if snapshot["state"] in (PAUSING, RESUMING, CLEARING):
            raise NavigationConflictError(
                "navigation is busy; wait for the current command to finish"
            )
        pose = validate_pose_payload(payload)
        now = self.now_sec()
        # Arm the transition floor and reject retry-lease conflicts before the
        # ROS publish, but keep the old paused record until it succeeds.
        token = self.navigation_control.prepare_goal_publish(
            pose, self.map_frame, map_name, now
        )
        try:
            self._publish_goal_pose(pose)
        except Exception:
            self.navigation_control.rollback_goal_publish(token)
            raise
        self.navigation_control.commit_goal_publish(token)
        with self.lock:
            self.published_goal = {
                "x": pose["x"],
                "y": pose["y"],
                "z": pose["z"],
                "yaw": pose["yaw"],
            }
        response = success_response("goal published", now)
        response["navigation_control"] = self.navigation_control.snapshot(map_name)
        return response

    def start_http_server(self) -> None:
        hub = getattr(self, "cloud_hub", None)
        if hub is not None:
            hub.refresh()
            hub.start()
        handler = self.make_handler()
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self.httpd.daemon_threads = True
        self.http_thread = threading.Thread(
            target=self.httpd.serve_forever, name="web_control_http", daemon=True
        )
        self.http_thread.start()
        self.get_logger().info(
            f"Web control console serving {self.static_dir} on http://{self.host}:{self.port}"
        )

    def stop_http_server(self) -> None:
        # Wake blocked SSE writers before stopping the accept loop so their
        # handler threads can notice shutdown instead of waiting for a client.
        hub = getattr(self, "cloud_hub", None)
        if hub is not None:
            hub.stop()
        if self.httpd is not None:
            try:
                self.httpd.shutdown()
            finally:
                self.httpd.server_close()
            self.httpd = None
        if self.http_thread is not None:
            self.http_thread.join(timeout=0.5)
            self.http_thread = None

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # type: ignore[no-untyped-def]
                node.get_logger().debug(fmt % args)

            def do_GET(self):  # type: ignore[no-untyped-def]
                parsed = urllib.parse.urlparse(self.path)
                query = urllib.parse.parse_qs(parsed.query)
                try:
                    if parsed.path == "/api/state":
                        self.send_json(node.state_payload())
                    elif parsed.path == "/api/maps":
                        self.send_json(node.maps_payload())
                    elif parsed.path == "/api/ui/preferences":
                        self.send_json(node.ui_preferences_payload())
                    elif parsed.path == "/api/pose":
                        self.send_json(node.pose_payload())
                    elif parsed.path == "/api/cloud/events":
                        group = query.get("group", ["nav"])[0]
                        layers = query.get("layers", [""])[0]
                        paths = query.get("paths", [""])[0]
                        pose = query.get("pose", [""])[0]
                        clouds = query.get("clouds", ["1"])[0]
                        self.stream_cloud_events(
                            node,
                            group,
                            layers,
                            paths=paths,
                            pose=pose,
                            clouds=clouds != "0",
                        )
                    elif parsed.path == "/api/map2d":
                        layer = query.get("layer", ["local_map"])[0]
                        normalize_layer_name(layer)
                        note_demand = getattr(node, "note_cloud_demand", None)
                        if note_demand is not None:
                            note_demand("nav", layer)
                        responder = getattr(node, "layer_response", None)
                        if responder is not None:
                            self.send_encoded_json(responder(layer, "2d"))
                        else:
                            self.send_json(node.layer_payload(layer, "2d"))
                    elif parsed.path == "/api/map3d":
                        layer = query.get("layer", ["local_map"])[0]
                        normalize_layer_name(layer)
                        note_demand = getattr(node, "note_cloud_demand", None)
                        if note_demand is not None:
                            note_demand("nav", layer)
                        responder = getattr(node, "layer_response", None)
                        if responder is not None:
                            self.send_encoded_json(responder(layer, "3d"))
                        else:
                            self.send_json(node.layer_payload(layer, "3d"))
                    elif parsed.path == "/api/path":
                        topic = query.get("topic", [""])[0]
                        self.send_json(node.path_payload(topic))
                    elif parsed.path == "/api/logs":
                        cursors: dict[str, int] = {}
                        for target in node.managed_logs:
                            raw = query.get(target, ["0"])[0]
                            try:
                                cursors[target] = max(0, int(raw))
                            except (TypeError, ValueError):
                                cursors[target] = 0
                        self.send_json(node.terminal_logs_payload(cursors))
                    elif parsed.path == "/api/system":
                        self.send_json(node.system_payload())
                    elif parsed.path == "/api/diagnostics":
                        self.send_json(node.diagnostics_payload())
                    elif parsed.path == "/api/tf":
                        self.send_json(node.tf_payload())
                    elif parsed.path == "/api/logs/sessions":
                        target = query.get("target", [""])[0]
                        self.send_json(node.saved_logs_payload(target))
                    elif parsed.path == "/api/logs/tail":
                        target = query.get("target", [""])[0]
                        session_id = query.get("session", [""])[0]
                        raw_max = query.get("max_bytes", ["262144"])[0]
                        try:
                            max_bytes = int(raw_max)
                        except (TypeError, ValueError):
                            max_bytes = 262144
                        text, truncated = node.saved_log_tail(
                            target, session_id, max_bytes
                        )
                        self.send_text(text, truncated=truncated)
                    elif parsed.path == "/api/logs/download":
                        target = query.get("target", [""])[0]
                        session_id = query.get("session", [""])[0]
                        self.send_log_download(
                            node.saved_log_download_path(target, session_id)
                        )
                    elif parsed.path == "/api/mapping/state":
                        self.send_json(node.mapping_state_payload())
                    elif parsed.path == "/api/mapping/map3d":
                        layer = query.get("layer", ["global_map"])[0]
                        normalize_mapping_layer_name(layer)
                        note_demand = getattr(node, "note_cloud_demand", None)
                        if note_demand is not None:
                            note_demand("mapping", layer)
                        responder = getattr(node, "mapping_layer_response", None)
                        if responder is not None:
                            self.send_encoded_json(responder(layer))
                        else:
                            self.send_json(node.mapping_layer_payload(layer))
                    elif parsed.path == "/api/mapping/path":
                        self.send_json(node.mapping_path_payload())
                    elif parsed.path == "/api/mapping/save_map":
                        self.send_json(node.save_map_status_payload())
                    elif parsed.path == "/api/mapping/save_map/browse":
                        target = query.get("path", [""])[0]
                        self.send_json(node.save_map_browse_payload(target))
                    elif parsed.path == "/api/mapping/record":
                        self.send_json(node.recording_status_payload())
                    elif parsed.path == "/api/mapping/record/topics":
                        self.send_json(node.record_topics_payload())
                    elif parsed.path == "/api/mapping/record/browse":
                        target = query.get("path", [""])[0]
                        self.send_json(node.record_browse_payload(target))
                    else:
                        self.serve_static(parsed.path)
                except CLIENT_DISCONNECT_ERRORS:
                    return
                except FileNotFoundError as exc:
                    self.send_json(
                        error_response(str(exc), node.now_sec()), status=404
                    )
                except LogStorageError as exc:
                    self.send_json(
                        error_response(str(exc), node.now_sec()), status=503
                    )
                except ValueError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=400)
                except Exception as exc:  # pragma: no cover - defensive HTTP guard
                    node.get_logger().error(f"HTTP GET failed: {exc}")
                    self.send_json(error_response("internal server error", node.now_sec()), status=500)

            def do_POST(self):  # type: ignore[no-untyped-def]
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    raw = self.rfile.read(length).decode("utf-8")
                    payload = json.loads(raw or "{}")
                    if self.path == "/api/initialpose":
                        self.send_json(node.publish_initialpose(payload))
                    elif self.path == "/api/maps/select":
                        self.send_json(node.select_map(payload))
                    elif self.path == "/api/ui/preferences":
                        self.send_json(node.save_ui_preferences(payload))
                    elif self.path == "/api/diagnostics/logging":
                        self.send_json(node.set_topic_logging(payload))
                    elif self.path == "/api/goal":
                        self.send_json(node.publish_goal(payload))
                    elif self.path == "/api/navigation/pause":
                        self.send_json(node.pause_navigation())
                    elif self.path == "/api/navigation/resume":
                        self.send_json(node.resume_navigation())
                    elif self.path == "/api/navigation/clear":
                        self.send_json(node.clear_navigation())
                    elif self.path == "/api/managed/start":
                        self.send_json(
                            node.start_managed_target(str(payload.get("target", "")))
                        )
                    elif self.path == "/api/managed/stop":
                        self.send_json(
                            node.stop_managed_target(str(payload.get("target", "")))
                        )
                    elif self.path == "/api/viewer":
                        self.send_json(node.viewer_heartbeat(payload))
                    elif self.path == "/api/teleop":
                        self.send_json(node.apply_teleop_command(payload))
                    elif self.path == "/api/mapping/save_map":
                        self.send_json(node.start_save_map(payload))
                    elif self.path == "/api/mapping/record/start":
                        self.send_json(node.start_recording(payload))
                    elif self.path == "/api/mapping/record/stop":
                        self.send_json(node.stop_recording())
                    else:
                        self.send_json(error_response("unknown endpoint", node.now_sec()), status=404)
                except CLIENT_DISCONNECT_ERRORS:
                    return
                except NavigationTimeoutError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=504)
                except NavigationUnavailableError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=503)
                except NavigationConflictError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=409)
                except SaveMapBusyError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=409)
                except SaveMapUnavailableError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=503)
                except RecordingBusyError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=409)
                except RecordingConflictError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=409)
                except RecordingUnavailableError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=503)
                except MapSelectionConflictError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=409)
                except MapSelectionPersistenceError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=503)
                except TopicLoggingPersistenceError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=503)
                except (json.JSONDecodeError, ValueError) as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=400)
                except Exception as exc:  # pragma: no cover - defensive HTTP guard
                    node.get_logger().error(f"HTTP POST failed: {exc}")
                    self.send_json(error_response("internal server error", node.now_sec()), status=500)

            def _write_sse(self, event: str, payload: dict[str, Any]) -> None:
                body = (
                    f"event: {event}\n"
                    f"data: {json.dumps(payload, separators=(',', ':'))}\n\n"
                ).encode("utf-8")
                self.wfile.write(body)
                self.wfile.flush()

            def stream_cloud_events(
                self,
                node: Any,
                group: str,
                raw_layers: str,
                *,
                paths: str = "",
                pose: str = "",
                clouds: bool = True,
            ) -> None:
                """Serve one metadata-only SSE subscription for one browser.

                The stream never serializes cloud/path points.  It carries
                cloud revisions, configured path revisions, and tiny pose
                payloads.  Demand is renewed only for cloud keys; path/pose
                resources must never create or drive ROS subscriptions.  A 1 s
                idle heartbeat renews dynamic cloud demand and detects a closed
                tab.
                """
                cloud_keys = (
                    parse_cloud_stream_keys(group, raw_layers) if clouds else ()
                )
                keys = tuple(
                    dict.fromkeys(
                        cloud_keys
                        + parse_path_stream_keys(node, paths)
                        + parse_pose_stream_keys(pose)
                    )
                )
                if not keys:
                    raise ValueError("empty metadata subscription")
                cloud_key_set = set(cloud_keys)
                hub = getattr(node, "cloud_hub", None)
                if hub is None:
                    raise ValueError("cloud metadata stream is unavailable")
                try:
                    self.connection.settimeout(5.0)
                except OSError:
                    pass
                self.close_connection = True
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                self.close_connection = True
                try:
                    snapshot = hub.snapshot(keys)
                    seen: dict[tuple[str, str], int] = {}
                    for entry in snapshot:
                        seen[(entry["group"], entry["layer"])] = entry["seq"]
                    self._write_sse(
                        "snapshot",
                        {
                            "epoch": str(getattr(hub, "epoch", "")),
                            "layers": snapshot,
                        },
                    )
                    while not hub.stopped:
                        note_demand = getattr(node, "note_cloud_demand", None)
                        if note_demand is not None:
                            for key in keys:
                                if key in cloud_key_set:
                                    note_demand(key[0], key[1])
                        changes = hub.wait_for_changes(
                            seen, timeout=1.0, keys=keys
                        )
                        if changes == []:
                            break
                        if changes:
                            for entry in changes:
                                seen[(entry["group"], entry["layer"])] = entry[
                                    "seq"
                                ]
                            self._write_sse(
                                "cloud",
                                {
                                    "epoch": str(getattr(hub, "epoch", "")),
                                    "layers": changes,
                                },
                            )
                        else:
                            # The write detects a hidden/closed tab; demand is
                            # not renewed again after it fails.
                            self.wfile.write(b": heartbeat\n\n")
                            self.wfile.flush()
                except (
                    BrokenPipeError,
                    ConnectionResetError,
                    socket.timeout,
                    OSError,
                ):
                    return

            def send_encoded_json(self, body: bytes, status: int = 200) -> None:
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def send_json(self, payload: dict[str, Any], status: int = 200) -> None:
                body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                self.send_encoded_json(body, status=status)

            def send_text(
                self, text: str, status: int = 200, truncated: bool = False
            ) -> None:
                body = text.encode("utf-8", "replace")
                self.send_response(status)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Log-Truncated", "1" if truncated else "0")
                self.end_headers()
                self.wfile.write(body)

            def send_log_download(self, path: Path) -> None:
                try:
                    handle = open(path, "rb")
                except OSError as exc:
                    raise LogStorageError(f"cannot open log file: {exc}") from exc
                try:
                    try:
                        size = os.fstat(handle.fileno()).st_size
                    except OSError as exc:
                        raise LogStorageError(f"cannot stat log file: {exc}") from exc
                    self.send_response(200)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.send_header("Content-Length", str(size))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header(
                        "Content-Disposition",
                        f'attachment; filename="{path.name}"',
                    )
                    self.end_headers()
                    # Stream exactly the snapshot size in bounded chunks; a log
                    # that keeps growing must not exceed the advertised length.
                    remaining = size
                    while remaining > 0:
                        chunk = handle.read(min(65536, remaining))
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        remaining -= len(chunk)
                finally:
                    handle.close()

            def serve_static(self, requested_path: str) -> None:
                rel = "index.html" if requested_path in {"", "/"} else requested_path.lstrip("/")
                rel_path = Path(os.path.normpath(rel))
                if rel_path.is_absolute() or ".." in rel_path.parts:
                    self.send_error(404)
                    return

                path = node.static_dir / rel_path
                if not path.is_file():
                    self.send_error(404)
                    return

                body = path.read_bytes()
                content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        return Handler

    def destroy_node(self) -> bool:
        # Stop the periodic publisher before sending the final zero command so
        # no timer callback can race with shutdown or publish after cleanup.
        if self.teleop_timer is not None:
            self.teleop_timer.cancel()
        diagnostics_timer = getattr(self, "diagnostics_timer", None)
        if diagnostics_timer is not None:
            diagnostics_timer.cancel()
        # Reject queued/in-flight cloud results, release pending raw buffers
        # and serialized bodies, then join the worker within a bounded time
        # before any log or node teardown.
        subscription_timer = getattr(self, "_cloud_subscription_timer", None)
        if subscription_timer is not None:
            subscription_timer.cancel()
        self.release_all_cloud_subscriptions()
        advance_cloud_generation(self)
        advance_state_generation(self, _all_state_keys(self))
        cloud_worker = getattr(self, "cloud_worker", None)
        if cloud_worker is not None:
            cloud_worker.stop()
        self.cleanup_teleop()
        recording = getattr(self, "recorder", None)
        if recording is not None:
            recording.cleanup(self.now_sec())
        self.cleanup_save_map()
        # Stop managed processes while the diagnostics log is still open: their
        # chassis exit hook queues a lifecycle event that shutdown_diagnostics
        # drains into the saved JSONL. Do not close the log before this point.
        self.stop_managed_processes()
        self.shutdown_diagnostics()
        if self.log_store is not None:
            self.log_store.close()
        self.stop_http_server()
        return super().destroy_node()


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python environment is not available")
    # Keep the ROS context alive when Ctrl+C arrives. Python raises
    # KeyboardInterrupt, then the finally block can publish the final teleop
    # zero before destroy_node() and rclpy.shutdown(). With rclpy's default
    # signal handler the context is already invalid by the time cleanup runs.
    rclpy.init(
        args=args,
        signal_handler_options=rclpy.SignalHandlerOptions.NO,
    )
    node = WebControlNode()
    atexit.register(node.stop_managed_processes)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if exc.__class__.__name__ != "ExternalShutdownException":
            raise
    finally:
        atexit.unregister(node.stop_managed_processes)
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
