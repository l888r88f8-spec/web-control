from __future__ import annotations

import atexit
import json
import math
import mimetypes
import os
import shlex
import signal
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
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
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

try:
    from action_msgs.srv import CancelGoal
except ModuleNotFoundError:
    CancelGoal = None

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
    PointCloudIngestWorker,
    parse_pointcloud2,
    utf8_json_bytes,
)
from web_control.resource_diagnostics import (
    DEFAULT_SAMPLE_PERIOD_SEC,
    ResourceLoggingWorker,
    ResourceSampler,
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
    browse_recording_directories,
    resolve_recording_output,
    validate_topic_selection,
)
from web_control.teleop import TeleopController
from web_control.tf_tree import TfTreeCache

LAYER_NAMES = {"global_map", "local_map", "mapcloud", "mapground", "obstacles"}
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
    ("nav", "mapcloud"),
    ("nav", "mapground"),
    ("nav", "obstacles"),
    ("mapping", "global_map"),
    ("mapping", "ground"),
)


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
    with owner.lock:
        cache = owner.layers[layer]
        data = cache.data or {}
        status = cache.status(now)
        frame_id = cache.frame_id
        count = cache.count
        version = cache.version
        points_3d = data.get("points3d", [])
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
            "points": points,
        },
        version,
    )


def _mapping_layer_payload_snapshot(
    owner: Any, layer_name: str
) -> tuple[dict[str, Any], int]:
    layer = normalize_mapping_layer_name(layer_name)
    now = owner.now_sec()
    with owner.lock:
        cache = owner.mapping_layers[layer]
        data = cache.data or {}
        return (
            {
                "layer": layer,
                "status": cache.status(now),
                "frame_id": cache.frame_id,
                "count": cache.count,
                "points": data.get("points3d", []),
            },
            cache.version,
        )


def success_response(message: str, stamp: float) -> dict[str, Any]:
    return {"ok": True, "message": message, "stamp": stamp}


def action_status_name(status_values: list[int]) -> str:
    if not status_values:
        return "idle"
    return ACTION_STATUS_NAMES.get(int(status_values[-1]), "unknown")


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
        # Legacy /api/map2d only: caps the on-request XY projection; point-cloud
        # ingestion never builds or stores a 2D cache.
        self.declare_parameter("max_2d_points_per_layer", 60000)
        self.declare_parameter("max_3d_points_per_layer", 40000)
        # Dynamic and Mapping cloud decoding pauses this many seconds after
        # the last map request for that layer. Only the three latched
        # navigation maps are decoded once per new input without a client.
        self.declare_parameter("cloud_demand_ttl_sec", DEFAULT_DEMAND_TTL_SEC)
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
        self.declare_parameter("teleop_wheelbase", 0.65)
        self.declare_parameter("teleop_max_steer_angle", 0.461)
        self.declare_parameter("teleop_steering_exponent", 0.5)
        self.declare_parameter("teleop_deadzone", 0.08)
        self.declare_parameter("teleop_command_timeout", 0.4)
        self.declare_parameter("teleop_publish_rate", 20.0)
        self.declare_parameter("diagnostics_enabled", True)
        self.declare_parameter("diagnostics_imu_topic", "/imu")
        self.declare_parameter("diagnostics_lidar_topic", "/lidar_points")
        self.declare_parameter("diagnostics_chassis_odom_topic", "/odom1")
        self.declare_parameter("diagnostics_window_sec", 5.0)
        self.declare_parameter("diagnostics_stale_timeout_sec", 2.0)
        self.declare_parameter("diagnostics_sample_period_sec", 1.0)
        self.declare_parameter("diagnostics_snapshot_period_sec", 5.0)
        self.declare_parameter("diagnostics_publisher_refresh_sec", 1.0)
        self.declare_parameter("diagnostics_max_log_bytes", 10 * 1024 * 1024)
        self.declare_parameter("diagnostics_retain_days", 7)
        self.declare_parameter("diagnostics_retain_bytes", 100 * 1024 * 1024)
        self.declare_parameter("resource_logging_enabled", True)
        self.declare_parameter(
            "resource_sample_period_sec", DEFAULT_SAMPLE_PERIOD_SEC
        )
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
        # Resource logging is a separate background baseline and log stream.
        # It deliberately does not use system_cpu_ticks or system_payload(), so
        # browser polling cannot change the CPU values persisted here.
        self.resource_logging_enabled = bool(
            self.get_parameter("resource_logging_enabled").value
        )
        self.resource_sample_period = max(
            0.01,
            float(self.get_parameter("resource_sample_period_sec").value),
        )
        self.resource_sampler = ResourceSampler()
        self.resource_worker: ResourceLoggingWorker | None = None
        self._resource_logging_active = False
        self._resource_logging_error: str | None = None
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
            "mapcloud": SourceCache("mapcloud", self.stale_timeout, static_source=True),
            "mapground": SourceCache("mapground", self.stale_timeout, static_source=True),
            "obstacles": SourceCache("obstacles", self.stale_timeout, static_source=False),
        }
        self.mapping_layers = {
            "global_map": SourceCache("mapping_global_map", self.stale_timeout, static_source=True),
            "ground": SourceCache("mapping_ground", self.stale_timeout, static_source=True),
        }
        # One visualization worker for all cloud layers.  Subscription
        # callbacks only replace a pending reference; parsing, finite
        # filtering and downsampling happen here, outside self.lock and the
        # single-threaded ROS executor.
        self._cloud_generation = 0
        self._cloud_generations: dict[tuple[str, str], int] = {}
        self.cloud_responses = CloudResponseCache()
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

        # Diagnostics are independent of open browser tabs: these subscribers
        # and the 1 Hz timer always run while the node is alive.  LiDAR uses
        # raw=True so rclpy does not deserialize PointCloud2 into Python points;
        # this still incurs the DDS/network byte copy and is not zero-copy.
        self.diagnostics_timer: Any | None = None
        if self.diagnostics_enabled:
            imu_monitor = self.diagnostics_service.monitors["imu"]
            # Monitoring uses BEST_EFFORT/VOLATILE so it receives from both
            # reliable and best-effort publishers.  Visualization/control
            # subscriptions above keep their existing QoS unchanged.
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
            if PointCloud2 is not None:
                lidar_qos = QoSProfile(
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                )
                self.create_subscription(
                    PointCloud2,
                    self.diagnostics_service.monitors["lidar"].topic,
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
            self.paths[topic] = SourceCache(topic, self.stale_timeout)
            self.create_subscription(
                PathMsg,
                topic,
                partial(self.path_callback, topic),
                state_qos,
            )

        self.httpd: ThreadingHTTPServer | None = None
        self.http_thread: threading.Thread | None = None
        self.configure_managed_processes()
        # Start only after managed launch objects exist, and before the HTTP
        # server can serve requests.  The worker is independent of ROS timers,
        # browser tabs and /api/system polling.
        self.start_resource_logging()
        self.start_http_server()

    def now_sec(self) -> float:
        return time.time()

    def ros_stamp(self):
        return self.get_clock().now().to_msg()

    def configure_pointcloud_subscriptions(self) -> None:
        """Subscribe to the navigation map/obstacle and mapping clouds.

        The localization global map is latched exactly like the existing
        static map clouds.  The live obstacle aggregate is volatile and
        best-effort: it matches both best-effort and reliable publishers and
        never replays a stale transient sample after navigation restarts.
        """
        layer_topics = {
            "global_map": str(self.get_parameter("global_map_topic").value),
            "local_map": str(self.get_parameter("local_map_topic").value),
            "mapcloud": str(self.get_parameter("mapcloud_topic").value),
            "mapground": str(self.get_parameter("mapground_topic").value),
        }
        static_pointcloud_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        pointcloud_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        for layer_name, topic in layer_topics.items():
            self.create_subscription(
                PointCloud2,
                topic,
                partial(self.pointcloud_callback, layer_name),
                static_pointcloud_qos,
            )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("obstacle_topic").value),
            self.obstacle_callback,
            pointcloud_qos,
        )

        mapping_layer_topics = {
            "global_map": str(self.get_parameter("mapping_global_map_topic").value),
            "ground": str(self.get_parameter("mapping_ground_topic").value),
        }
        for layer_name, topic in mapping_layer_topics.items():
            self.create_subscription(
                PointCloud2,
                topic,
                partial(self.mapping_pointcloud_callback, layer_name),
                pointcloud_qos,
            )

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
            # Stop is not an operator command: settle the pause/resume state so
            # the browser stops advertising an active goal, while an
            # interrupted Pause or Resume keeps its saved destination paused.
            self.published_goal = None
            self.navigation_control.on_navigation_stopped(self.now_sec())

    def clear_map_runtime_state(self) -> None:
        with self.lock:
            advance_cloud_generation(
                self,
                [key for key in _ALL_CLOUD_KEYS if key[0] == "nav"],
            )
            # Every navigation display cloud belongs to the localization or
            # navigation runtime.  A localization restart may load a different
            # map, so the directly-subscribed global map is dropped too rather
            # than leaving an obsolete scene behind.
            for cache in self.layers.values():
                cache.clear()
            self.odom.clear()
            self.action_status.clear()
            for cache in self.paths.values():
                cache.clear()

    def clear_mapping_runtime_state(self) -> None:
        with self.lock:
            advance_cloud_generation(
                self,
                [key for key in _ALL_CLOUD_KEYS if key[0] == "mapping"],
            )
            for cache in self.mapping_layers.values():
                cache.clear()
            self.mapping_odom.clear()
            self.mapping_path.clear()

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
                global_cache = self.layers.get("global_map")
                if global_cache is not None:
                    global_cache.clear()
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
                # previous session's latched global map or queued map frames
                # into the new one.
                advance_cloud_generation(self, [("nav", "global_map")])
                self.layers["global_map"].clear()
            self.managed_logs[target].set_process_state(
                enabled=True,
                running=process.is_running(),
                returncode=None,
            )
            process.start()
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
        """Hand a cloud to the worker, or use the legacy path on a stub node.

        The production node always has ``cloud_worker``.  The synchronous
        branch keeps ROS-free unit stubs (and direct callers of
        ``update_pointcloud_cache``) working; it is never taken by a live node.
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
        if enqueue is None:  # ROS-free stub bound only to the public callback
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
                return True
            cache.update(
                {"points3d": points},
                stamp=arrival_stamp,
                frame_id=frame_id,
                count=finite_count,
            )
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
            mapground_status = self.layers["mapground"].status(now)
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
        # The web dialog sends a path relative to the web node user's home
        # directory (/map); validation resolves it to an absolute path under
        # that home before the job and the SaveMapOptions request use it.
        options = validate_save_map_payload(payload)
        if not self.save_map_available():
            raise SaveMapUnavailableError("save map service unavailable")
        client = self.save_map_client
        if client is None:
            raise SaveMapUnavailableError("save map service unavailable")
        token = self.save_map_job.begin(options["destination"], self.now_sec())
        if token is None:
            raise SaveMapBusyError("save map already running")

        request = SaveMapOptions.Request()
        request.destination = options["destination"]
        request.resolution = options["resolution"]
        request.ground_resolution = options["ground_resolution"]
        request.save_ground_map = options["save_ground_map"]
        request.save_tiled_map = options["save_tiled_map"]
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
            return {
                "odom": {
                    **self.odom.summary(now),
                    "pose": self.odom.data,
                },
                "mapping_odom": {
                    **self.mapping_odom.summary(now),
                    "pose": self.mapping_odom.data,
                },
                "map_frame": self.map_frame,
                "server_time": now,
            }

    def note_cloud_demand(self, group: str, layer_name: str) -> None:
        """Activate the short dynamic-demand window for one cloud layer."""
        worker = getattr(self, "cloud_worker", None)
        if worker is not None:
            worker.note_demand(group, layer_name)

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
            return {
                "topic": topic,
                "status": cache.status(now),
                "frame_id": cache.frame_id,
                "count": cache.count,
                "points": data.get("points", []),
            }

    def mapping_path_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            data = self.mapping_path.data or {"points": []}
            return {
                "topic": "mapping_path",
                "status": self.mapping_path.status(now),
                "frame_id": self.mapping_path.frame_id,
                "count": self.mapping_path.count,
                "points": data.get("points", []),
            }

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

    # -- Periodic resource diagnostics ------------------------------------
    def resource_scope_payload(self) -> list[dict[str, Any]]:
        """Return only web-owned launch roots, never discovered ROS names."""
        groups: list[dict[str, Any]] = [
            {
                "target": "web_control",
                "kind": "web_process",
                "root_pids": [os.getpid()],
                "extra_pids": [],
                "include_descendants": False,
            }
        ]
        for target in (
            "chassis",
            "imu",
            "lidar",
            "localization",
            "navigation",
            "mapping",
        ):
            roots: list[int] = []
            extras: list[int] = []
            pgid: int | None = None
            managed = self.managed_processes.get(target)
            if managed is not None:
                lock = getattr(managed, "lock", None)
                try:
                    if lock is not None:
                        lock.acquire()
                    process = getattr(managed, "process", None)
                    if process is not None and process.poll() is None:
                        roots = [int(process.pid)]
                        raw_pgid = getattr(managed, "pgid", None)
                        pgid = None if raw_pgid is None else int(raw_pgid)
                        extras = sorted(
                            int(pid)
                            for pid in getattr(managed, "descendant_pids", set())
                            if int(pid) != int(process.pid)
                        )
                except Exception:
                    roots = []
                    extras = []
                    pgid = None
                finally:
                    if lock is not None:
                        try:
                            lock.release()
                        except Exception:
                            pass
            groups.append(
                {
                    "target": target,
                    "kind": "managed_ros_launch",
                    "root_pids": roots,
                    "extra_pids": extras,
                    "pgid": pgid,
                }
            )

        recorder = getattr(self, "recorder", None)
        if recorder is not None:
            lock = getattr(recorder, "_lock", None)
            try:
                if lock is not None:
                    lock.acquire()
                process = getattr(recorder, "_process", None)
                if process is not None and process.poll() is None:
                    groups.append(
                        {
                            "target": "rosbag",
                            "kind": "rosbag_process",
                            "root_pids": [int(process.pid)],
                            "extra_pids": [],
                        }
                    )
            except Exception:
                pass
            finally:
                if lock is not None:
                    try:
                        lock.release()
                    except Exception:
                        pass
        return groups

    def start_resource_logging(self) -> bool:
        """Start the single background resource worker after launch setup."""
        if not getattr(self, "resource_logging_enabled", False):
            return False
        if not hasattr(self, "diagnostics_log"):
            return False
        if self.resource_worker is None:
            self.resource_worker = ResourceLoggingWorker(
                self.resource_sampler,
                self.resource_scope_payload,
                self._write_resources,
                period_sec=self.resource_sample_period,
                logger=self.get_logger(),
            )
        with self._diagnostics_lock:
            if getattr(self, "_diagnostics_closed", False):
                return False
            self._resource_logging_active = True
            self._resource_logging_error = None
        try:
            self.resource_worker.start()
        except Exception as exc:  # pragma: no cover - defensive lifecycle guard
            with self._diagnostics_lock:
                self._resource_logging_active = False
                self._resource_logging_error = (
                    f"resource logging failed to start: {exc}"
                )
            self.get_logger().error(
                f"resource diagnostics worker failed to start: {exc}"
            )
            return False
        return True

    def stop_resource_logging(self) -> None:
        """Stop the worker before the diagnostics log is closed."""
        worker = getattr(self, "resource_worker", None)
        if worker is not None:
            try:
                worker.stop(timeout=2.0)
            except Exception as exc:  # pragma: no cover - defensive shutdown guard
                self._resource_logging_error = (
                    f"resource logging failed to stop cleanly: {exc}"
                )
        with self._diagnostics_lock:
            self._resource_logging_active = False
        if worker is not None and getattr(worker, "error", None):
            self._resource_logging_error = worker.error

    def _write_resources(self, entry: dict[str, Any]) -> bool:
        """Persist a resource row without the topic/IMU filter.

        This intentionally bypasses ``filter_topic_logging_entry``: turning the
        topic/IMU log button OFF must not stop resource diagnosis.  The same
        diagnostics log lock still serializes shutdown and prevents writes
        after the log session closes.
        """
        lock = getattr(self, "_diagnostics_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._diagnostics_lock = lock
        with lock:
            if getattr(self, "_diagnostics_closed", False):
                return False
            if not getattr(self, "_resource_logging_active", False):
                return False
            log = getattr(self, "diagnostics_log", None)
            if log is None:
                return False
            record = dict(entry)
            record.setdefault("kind", "resources")
            record.setdefault("source", "resource_sampler")
            record.setdefault("timestamp_unix", time.time())
            try:
                ok = log.write(
                    record, now=float(record.get("timestamp_unix", time.time()))
                )
                error = log.error
            except Exception as exc:  # pragma: no cover - disk failure guard
                ok = False
                error = f"resource log write failed: {exc}"
        if not ok and error and error != getattr(
            self, "_resource_logging_error", None
        ):
            self._resource_logging_error = error
            self.get_logger().error(
                f"resource diagnostics log write failed: {error}"
            )
        return bool(ok)

    # -- Topic and motion diagnostics -------------------------------------
    def diagnostics_imu_callback(self, msg: Any) -> None:
        try:
            self.diagnostics_service.record_imu(msg, time.monotonic())
        except Exception as exc:  # pragma: no cover - defensive ROS guard
            self.get_logger().warning(f"IMU diagnostics callback failed: {exc}")

    def diagnostics_lidar_callback(self, _raw_msg: Any) -> None:
        # raw=True delivers bytes only; never inspect or deserialize here.
        self.diagnostics_service.record("lidar", time.monotonic())

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
        payload, events = self.diagnostics_service.tick(now_mono, counts)
        topic_rows = {row["topic"]: row for row in payload["topics"]}
        if not self._diagnostics_started:
            self._diagnostics_started = True
            self._write_diagnostics(
                {
                    "kind": "startup",
                    "source": "diagnostics",
                    "enabled": self.diagnostics_enabled,
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
        payload["log_dir"] = self.log_store.display_root()
        payload["sample_period_sec"] = self.diagnostics_sample_period
        payload["snapshot_period_sec"] = self.diagnostics_snapshot_period
        return payload

    def shutdown_diagnostics(self) -> None:
        if not hasattr(self, "diagnostics_log"):
            return
        # The resource worker must be gone before the final topic/motion rows
        # are drained and the shared diagnostics session is closed.
        stop_resources = getattr(self, "stop_resource_logging", None)
        if callable(stop_resources):
            try:
                stop_resources()
            except Exception:
                pass
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
        advance_cloud_generation(self)
        cloud_worker = getattr(self, "cloud_worker", None)
        if cloud_worker is not None:
            cloud_worker.stop()
        self.cleanup_teleop()
        recording = getattr(self, "recorder", None)
        if recording is not None:
            recording.cleanup(self.now_sec())
        self.cleanup_save_map()
        # Stop the independent resource thread first: no resource write may race
        # with the diagnostics close performed by shutdown_diagnostics().
        self.stop_resource_logging()
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
