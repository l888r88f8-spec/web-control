# Idle Point-Cloud Subscriptions Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Subscribe, decode and retain a cloud layer only while a browser displays it, releasing the subscription and decoded cache after an idle grace period.

**Architecture:** A ROS-free planner module decides which `(group, layer)` keys need a subscription from the worker's demand stamps. A 0.2 s node timer performs the rclpy entity work on the executor thread: create missing subscriptions, destroy idle ones, invalidate their pending frames and clear their decoded caches. Navigation readiness moves from the display cache to a lifecycle flag.

**Tech Stack:** Python 3.10, rclpy (Humble), pytest, ROS-free stubs.

---

## File Structure

- Create `web_control/cloud_subscriptions.py` — subscription plans and the pure `reconcile()` decision function. No ROS imports.
- Modify `web_control/pointcloud_runtime.py` — demand gates every layer; expose `demand_stamps()` / `demand_active()`.
- Modify `web_control/web_control_node.py` — plan-driven lazy subscriptions, reconciler timer, release path, readiness flag, idle parameter, shutdown cleanup.
- Modify `config/web_control.yaml` — `cloud_subscription_idle_sec`.
- Create `test/test_idle_cloud_subscriptions.py` — planner, node create/release and readiness contracts.
- Modify `test/test_pointcloud_optimization.py:378-395` — latched maps decode only with demand.
- Modify `test/test_navigation_global_layers.py:72-115` and `test/test_current_lidar_layer.py:88-120` — subscriptions appear after the first demand.

---

### Task 1: Pure subscription planner

**Files:**
- Create: `web_control/cloud_subscriptions.py`
- Test: `test/test_idle_cloud_subscriptions.py`

- [ ] **Step 1: Write the failing planner tests**

Create `test/test_idle_cloud_subscriptions.py`:

```python
"""Demand-driven cloud subscription contracts.

``reconcile`` is pure: it decides which layers need a ROS subscription from
the worker's demand stamps.  The node performs the entity work; these tests
stay ROS-free.
"""

from web_control.cloud_subscriptions import (
    CLOUD_SUBSCRIPTION_PLANS,
    PLAN_BY_KEY,
    reconcile,
)


NOW = 100.0


def _keys(*layers):
    return [("nav", name) for name in layers]


def test_plans_cover_every_browser_cloud_layer():
    assert set(PLAN_BY_KEY) == {
        ("nav", "global_map"),
        ("nav", "local_map"),
        ("nav", "current_lidar"),
        ("nav", "mapcloud"),
        ("nav", "mapground"),
        ("nav", "obstacles"),
        ("nav", "grid_terrain"),
        ("mapping", "global_map"),
        ("mapping", "ground"),
    }
    assert len(CLOUD_SUBSCRIPTION_PLANS) == 9
    assert PLAN_BY_KEY[("nav", "grid_terrain")].message_type == "MarkerArray"
    assert PLAN_BY_KEY[("nav", "obstacles")].qos == "live"
    assert PLAN_BY_KEY[("nav", "mapcloud")].qos == "latched"


def test_reconcile_creates_only_demanded_layers():
    create, release = reconcile(
        NOW,
        (),
        {("nav", "mapcloud"): NOW - 0.5, ("nav", "obstacles"): NOW},
        demand_ttl_sec=2.0,
        idle_sec=5.0,
    )
    assert create == _keys("mapcloud", "obstacles")
    assert release == []


def test_reconcile_ignores_expired_demand_for_creation():
    create, release = reconcile(
        NOW,
        (),
        {("nav", "mapcloud"): NOW - 2.5},
        demand_ttl_sec=2.0,
        idle_sec=5.0,
    )
    assert create == []
    assert release == []


def test_reconcile_keeps_subscriptions_during_the_idle_grace():
    create, release = reconcile(
        NOW,
        _keys("mapcloud"),
        {("nav", "mapcloud"): NOW - 4.0},
        demand_ttl_sec=2.0,
        idle_sec=5.0,
    )
    assert create == []
    assert release == []


def test_reconcile_releases_after_the_idle_grace():
    create, release = reconcile(
        NOW,
        _keys("mapcloud", "obstacles"),
        {("nav", "mapcloud"): NOW - 5.5, ("nav", "obstacles"): NOW - 1.0},
        demand_ttl_sec=2.0,
        idle_sec=5.0,
    )
    assert create == []
    assert release == _keys("mapcloud")


def test_reconcile_releases_a_subscription_with_no_demand_record():
    create, release = reconcile(
        NOW,
        _keys("local_map"),
        {},
        demand_ttl_sec=2.0,
        idle_sec=5.0,
    )
    assert create == []
    assert release == _keys("local_map")


def test_reconcile_idle_zero_never_releases():
    create, release = reconcile(
        NOW,
        _keys("mapcloud"),
        {},
        demand_ttl_sec=2.0,
        idle_sec=0.0,
    )
    assert create == []
    assert release == []


def test_reconcile_never_reports_a_key_as_created_and_released():
    create, release = reconcile(
        NOW,
        _keys("mapcloud"),
        {("nav", "mapcloud"): NOW - 6.0},
        demand_ttl_sec=2.0,
        idle_sec=5.0,
    )
    assert set(create).isdisjoint(release)


def test_reconcile_ignores_unknown_keys():
    create, release = reconcile(
        NOW,
        [("nav", "not_a_layer")],
        {("nav", "not_a_layer"): NOW},
        demand_ttl_sec=2.0,
        idle_sec=5.0,
    )
    assert create == []
    assert release == []


def test_reconcile_never_releases_a_key_still_inside_the_demand_ttl():
    # ``idle_sec`` shorter than the demand TTL must not flap a layer that is
    # still being displayed: release age is max(idle_sec, demand_ttl_sec).
    create, release = reconcile(
        NOW,
        _keys("mapcloud"),
        {("nav", "mapcloud"): NOW - 1.5},
        demand_ttl_sec=2.0,
        idle_sec=1.0,
    )
    assert create == []
    assert release == []
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_idle_cloud_subscriptions.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'web_control.cloud_subscriptions'`.

- [ ] **Step 3: Write the planner module**

Create `web_control/cloud_subscriptions.py`:

```python
"""Demand-driven ROS subscription plans for browser cloud layers.

ROS-free by design: the node performs ``create_subscription`` /
``destroy_subscription`` on the executor thread, and this module only decides
which ``(group, layer)`` keys should stay subscribed and describes how to
subscribe them.
"""

from dataclasses import dataclass
from typing import Iterable, Mapping


@dataclass(frozen=True)
class CloudSubscriptionPlan:
    """How one browser cloud layer maps onto a ROS topic."""

    group: str
    layer: str
    topic_parameter: str
    # "PointCloud2" or "MarkerArray"; resolved against the node's imports so a
    # partial ROS installation leaves the layer unsubscribed instead of
    # crashing start-up.
    message_type: str
    # "latched" = RELIABLE/TRANSIENT_LOCAL (replays on re-subscription);
    # "live" = BEST_EFFORT/VOLATILE (next publisher sample).
    qos: str
    callback: str


CLOUD_SUBSCRIPTION_PLANS: tuple[CloudSubscriptionPlan, ...] = (
    CloudSubscriptionPlan(
        "nav", "global_map", "global_map_topic", "PointCloud2", "latched",
        "pointcloud_callback",
    ),
    CloudSubscriptionPlan(
        "nav", "local_map", "local_map_topic", "PointCloud2", "latched",
        "pointcloud_callback",
    ),
    CloudSubscriptionPlan(
        "nav", "current_lidar", "current_lidar_topic", "PointCloud2", "live",
        "pointcloud_callback",
    ),
    CloudSubscriptionPlan(
        "nav", "mapcloud", "mapcloud_topic", "PointCloud2", "latched",
        "pointcloud_callback",
    ),
    CloudSubscriptionPlan(
        "nav", "mapground", "mapground_topic", "PointCloud2", "latched",
        "pointcloud_callback",
    ),
    CloudSubscriptionPlan(
        "nav", "obstacles", "obstacle_topic", "PointCloud2", "live",
        "obstacle_callback",
    ),
    CloudSubscriptionPlan(
        "nav", "grid_terrain", "grid_terrain_topic", "MarkerArray", "latched",
        "grid_terrain_callback",
    ),
    CloudSubscriptionPlan(
        "mapping", "global_map", "mapping_global_map_topic", "PointCloud2",
        "live", "mapping_pointcloud_callback",
    ),
    CloudSubscriptionPlan(
        "mapping", "ground", "mapping_ground_topic", "PointCloud2", "live",
        "mapping_pointcloud_callback",
    ),
)

PLAN_BY_KEY: dict[tuple[str, str], CloudSubscriptionPlan] = {
    (plan.group, plan.layer): plan for plan in CLOUD_SUBSCRIPTION_PLANS
}


def reconcile(
    now: float,
    subscribed: Iterable[tuple[str, str]],
    last_demand: Mapping[tuple[str, str], float],
    *,
    demand_ttl_sec: float,
    idle_sec: float,
) -> tuple[list[tuple[str, str]], list[tuple[str, str]]]:
    """Return ``(create, release)`` for one reconciler tick.

    ``now`` and every ``last_demand`` stamp must come from the same clock.
    The worker stamps demand with ``time.monotonic``, so callers must pass the
    worker's clock, never a wall-clock time: a mixed domain makes every delta
    astronomically large (create never fires, release fires every tick).

    ``create`` holds keys demanded within ``demand_ttl_sec`` (clamped to
    non-negative) with no subscription.  ``release`` holds subscribed keys
    whose last demand is older than ``max(idle_sec, demand_ttl_sec)``,
    including keys with no demand record at all.  The maximum keeps a key that
    is still demanded from being released and immediately re-created.
    ``idle_sec <= 0`` keeps every subscription for the process lifetime.
    """
    ttl = max(0.0, float(demand_ttl_sec))
    idle = float(idle_sec)
    subscribed_set = set(subscribed)
    create = sorted(
        key
        for key, stamp in last_demand.items()
        if key in PLAN_BY_KEY
        and now - stamp <= ttl
        and key not in subscribed_set
    )
    if idle <= 0.0:
        return create, []
    release_age = max(idle, ttl)
    release = sorted(
        key
        for key in subscribed_set
        if key in PLAN_BY_KEY
        and (key not in last_demand or now - last_demand[key] > release_age)
    )
    return create, release
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_idle_cloud_subscriptions.py -q`
Expected: 8 passed.

- [ ] **Step 5: Commit**

```bash
cd /home/robot/web_ws/src/web_control
git add web_control/cloud_subscriptions.py test/test_idle_cloud_subscriptions.py
git commit -m "feat: add demand-driven cloud subscription planner"
```

---

### Task 2: Demand gates every layer in the worker

**Files:**
- Modify: `web_control/pointcloud_runtime.py:475-492`
- Test: `test/test_pointcloud_optimization.py:378-395`

- [ ] **Step 1: Replace the static-layer test with a demand test**

Replace `test_static_layers_decode_once_per_new_input_without_demand` in
`test/test_pointcloud_optimization.py` with:

```python
def test_latched_maps_decode_only_after_demand_and_keep_the_output_ceiling():
    applied = []
    worker = _worker(applied)
    key = ("nav", "global_map")
    worker.submit(
        key, _frame(1), arrival_stamp=0.0, clear_on_empty=False, generation=0
    )
    # No browser shows the layer, so nothing is decoded even for a latched map.
    assert worker._select_ready(0.0) is None
    assert worker.demand_stamps() == {}

    worker.note_demand("nav", "global_map", now=0.0)
    assert worker.demand_stamps() == {key: 0.0}
    assert worker._select_ready(0.0) == key
    worker._pending.pop(key)
    worker._last_output[key] = 0.0
    worker.submit(
        key, _frame(2), arrival_stamp=0.5, clear_on_empty=False, generation=0
    )
    assert worker._select_ready(0.5) is None  # 1 Hz output ceiling
    assert worker._select_ready(1.0) == key
    worker.stop()
```

In the same file, `test_static_output_period_is_measured_from_admission_not_completion`
submits a `global_map` frame and expects the worker to decode it.  Add the
demand it now needs, before `worker.start()`:

```python
    worker.note_demand("nav", "global_map", now=0.0)
```

Four more tests in the same file submit `("nav", "global_map")` and wait for a
decode, so they need the new demand too:

- `test_callbacks_only_replace_one_pending_frame_and_decode_latest`
- `test_pending_frame_is_released_after_static_apply`
- `test_raw_frame_is_released_while_worker_remains_running_idle`
- `test_bad_cloud_is_logged_and_does_not_stop_worker`

In each, add after the `_worker(...)` construction and before `worker.start()`
(or before the first `_select_ready`):

```python
    worker.note_demand("nav", "global_map")
```

Do not pass `now=`: these tests drive the real worker thread against the
monotonic clock.

Add a test for the accessors the reconciler depends on:

```python
def test_demand_accessors_expose_the_worker_clock_and_copy_the_table():
    clock = _ManualClock()
    worker = _worker([], clock=clock)
    key = ("nav", "global_map")
    assert worker.now() == clock.now
    assert worker.demand_active(key) is False

    worker.note_demand("nav", "global_map")
    assert worker.demand_active(key) is True
    # The reconciler may pass a list/tuple key; normalisation keeps it working.
    assert worker.demand_active(["nav", "global_map"]) is True

    clock.now += 2.5
    assert worker.demand_active(key) is False

    stamps = worker.demand_stamps()
    stamps[key] = -1.0
    assert worker.demand_stamps()[key] >= 0.0, "demand_stamps returns a copy"
```

Do not change any other test.

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_pointcloud_optimization.py::test_latched_maps_decode_only_after_demand_and_keep_the_output_ceiling -q`
Expected: FAIL — the first `_select_ready(0.0)` returns the key because
`global_map` is a static layer, and `demand_stamps` does not exist.

- [ ] **Step 3: Make every layer demand-gated and expose the demand table**

In `web_control/pointcloud_runtime.py`, replace `_demand_active`:

```python
    def _demand_active(self, key: tuple[str, str], now: float) -> bool:
        stamp = self._demand.get(key)
        return stamp is not None and (now - stamp) <= self._demand_ttl
```

Add after `note_demand`:

```python
    def demand_active(
        self, key: tuple[str, str], *, now: Optional[float] = None
    ) -> bool:
        """Return whether ``key`` was demanded within the TTL."""
        with self._cond:
            current = self._clock() if now is None else float(now)
            return self._demand_active((str(key[0]), str(key[1])), current)

    def demand_stamps(self) -> dict[tuple[str, str], float]:
        """Copy of the last demand stamp per layer.

        The reconciler reads this to decide which subscriptions to keep; the
        worker owns the stamps so a released layer cannot keep decoding.
        Stamps use this worker's clock (``time.monotonic`` by default), so the
        reconciler must compare them with ``now()``, never a wall clock.
        """
        with self._cond:
            return dict(self._demand)

    def now(self) -> float:
        """Current worker-clock time; the only clock demand stamps use."""
        return self._clock()
```

Update the module docstring paragraph that says the three latched navigation
maps are decoded without demand to read:

```
Every layer is demand-gated: it decodes only while an HTTP/SSE viewer has
requested it within the demand TTL.  The three latched navigation maps keep a
1 Hz output ceiling so a re-subscribed map is not decoded twice per publisher
sample.  A layer that is never demanded keeps at most one raw frame (its
single pending slot) until demand, invalidation or shutdown.
```

Update `docs/benchmarks/pointcloud-optimization.md:39-45`: replace "Only the
three latched navigation maps (`global_map`, `mapcloud`, `mapground`) are
decoded once per new input without a request for an early/late-browser cache."
with:

```
Every layer is demand-gated. The three latched navigation maps (`global_map`,
`mapcloud`, `mapground`) keep a 1 Hz output ceiling; idle subscriptions are
released (and their decoded caches dropped) after
`cloud_subscription_idle_sec` (default 5 s).
```

Then stage the benchmark doc as well.

- [ ] **Step 4: Run the worker tests and the full optimization module**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_pointcloud_optimization.py -q`
Expected: PASS (existing demand tests still pass; the static test now expects demand).

- [ ] **Step 5: Commit**

```bash
cd /home/robot/web_ws/src/web_control
git add web_control/pointcloud_runtime.py test/test_pointcloud_optimization.py docs/benchmarks/pointcloud-optimization.md
git commit -m "perf: require demand before decoding any cloud layer"
```

---

### Task 3: Create subscriptions on demand

**Files:**
- Modify: `web_control/web_control_node.py:144-150` (imports), `:1440-1448` (constants), `:2033-2103` (`configure_pointcloud_subscriptions`)
- Test: `test/test_idle_cloud_subscriptions.py`

- [ ] **Step 1: Write the failing node creation tests**

Append to `test/test_idle_cloud_subscriptions.py`:

```python
import threading
from types import MethodType, SimpleNamespace

import pytest

from web_control import web_control_node
from web_control.core import SourceCache


class _QosProfile:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class _Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class _FakeWorker:
    def __init__(self, clock):
        self.clock = clock
        self.demands = {}
        self.invalidated = []

    def demand_stamps(self):
        return dict(self.demands)

    def now(self):
        return self.clock()

    def demand_active(self, key, *, now=None):
        stamp = self.demands.get(key)
        current = self.clock() if now is None else now
        return stamp is not None and current - stamp <= 30.0

    def invalidate(self, keys=None):
        for key in keys or ():
            self.demands.pop(key, None)
        self.invalidated.append(list(keys) if keys is not None else None)


class _CloudNode:
    def __init__(self):
        self.now = 10.0
        self.lock = threading.RLock()
        self.parameters = {
            "global_map_topic": "/lio_sam/localization/global_map",
            "local_map_topic": "/lio_sam/localization/local_map",
            "current_lidar_topic": "/lio_sam/localization/current_lidar",
            "mapcloud_topic": "mapcloud",
            "mapground_topic": "/weighted_ground",
            "obstacle_topic": "/aggregated_pc",
            "grid_terrain_topic": "/grid_global_planner/terrain",
            "mapping_global_map_topic": "/lio_sam/mapping/map_global",
            "mapping_ground_topic": "/lio_sam/mapping/ground_cloud_global",
        }
        self.cloud_demand_ttl = 30.0
        self.cloud_subscription_idle = 5.0
        self.layers = {
            "global_map": SourceCache("global_map", 2.0, static_source=True),
            "local_map": SourceCache("local_map", 2.0, static_source=True),
            "current_lidar": SourceCache("current_lidar", 2.0),
            "mapcloud": SourceCache("mapcloud", 2.0, static_source=True),
            "mapground": SourceCache("mapground", 2.0, static_source=True),
            "obstacles": SourceCache("obstacles", 2.0),
            "grid_terrain": SourceCache("grid_terrain", 2.0, static_source=True),
        }
        self.mapping_layers = {
            "global_map": SourceCache("mapping_global_map", 2.0, static_source=True),
            "ground": SourceCache("mapping_ground", 2.0, static_source=True),
        }
        self.cloud_subscriptions = {}
        self._cloud_subscription_failures = {}
        self._cloud_generations = {}
        self._cloud_generation = 0
        self.subscriptions = []
        self.timer = None
        self.destroyed = []
        self.metadata_publishes = 0
        self.logger = _Logger()
        self.worker = _FakeWorker(lambda: self.now)
        self.cloud_worker = self.worker
        self.raise_on_create = False

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_logger(self):
        return self.logger

    def now_sec(self):
        return self.now

    def create_timer(self, period, callback):
        self.timer = (period, callback)
        return object()

    def create_subscription(self, msg_type, topic, callback, qos, **kwargs):
        if self.raise_on_create:
            raise RuntimeError("boom")
        self.subscriptions.append(
            {"msg_type": msg_type, "topic": topic, "callback": callback, "qos": qos}
        )
        return object()

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)

    def publish_cloud_metadata(self):
        self.metadata_publishes += 1

    def pointcloud_callback(self, layer_name, _msg):
        return layer_name

    def obstacle_callback(self, _msg):
        return "obstacles"

    def grid_terrain_callback(self, _msg):
        return "grid_terrain"

    def mapping_pointcloud_callback(self, layer_name, _msg):
        return layer_name


def _bind_cloud(node):
    for name in (
        "configure_pointcloud_subscriptions",
        "reconcile_cloud_subscriptions",
        "create_cloud_subscription",
        "cloud_layer_qos",
    ):
        setattr(
            node, name, MethodType(getattr(web_control_node.WebControlNode, name), node)
        )
    return node


@pytest.fixture
def cloud_node(monkeypatch):
    monkeypatch.setattr(web_control_node, "QoSProfile", _QosProfile)
    monkeypatch.setattr(
        web_control_node, "QoSHistoryPolicy", SimpleNamespace(KEEP_LAST="keep_last")
    )
    monkeypatch.setattr(
        web_control_node,
        "QoSReliabilityPolicy",
        SimpleNamespace(RELIABLE="reliable", BEST_EFFORT="best_effort"),
    )
    monkeypatch.setattr(
        web_control_node,
        "QoSDurabilityPolicy",
        SimpleNamespace(TRANSIENT_LOCAL="transient_local", VOLATILE="volatile"),
    )
    monkeypatch.setattr(web_control_node, "PointCloud2", object)
    monkeypatch.setattr(web_control_node, "MarkerArray", object)
    return _bind_cloud(_CloudNode())


def test_configure_creates_no_cloud_subscription(cloud_node):
    cloud_node.configure_pointcloud_subscriptions()
    assert cloud_node.subscriptions == []
    assert cloud_node.timer[0] == 0.2


def test_reconcile_creates_subscriptions_for_demanded_layers(cloud_node):
    cloud_node.configure_pointcloud_subscriptions()
    cloud_node.worker.demands[("nav", "mapcloud")] = cloud_node.now
    cloud_node.worker.demands[("nav", "obstacles")] = cloud_node.now
    cloud_node.reconcile_cloud_subscriptions()

    by_topic = {entry["topic"]: entry for entry in cloud_node.subscriptions}
    assert set(by_topic) == {"mapcloud", "/aggregated_pc"}
    assert by_topic["mapcloud"]["qos"].kwargs == {
        "history": "keep_last",
        "depth": 1,
        "reliability": "reliable",
        "durability": "transient_local",
    }
    assert by_topic["/aggregated_pc"]["qos"].kwargs == {
        "history": "keep_last",
        "depth": 1,
        "reliability": "best_effort",
        "durability": "volatile",
    }
    assert by_topic["mapcloud"]["callback"].args == ("mapcloud",)
    assert by_topic["/aggregated_pc"]["callback"] == cloud_node.obstacle_callback


def test_reconcile_does_not_subscribe_without_demand(cloud_node):
    cloud_node.configure_pointcloud_subscriptions()
    cloud_node.reconcile_cloud_subscriptions()
    assert cloud_node.subscriptions == []


def test_creation_failure_backs_off_then_retries(cloud_node):
    cloud_node.configure_pointcloud_subscriptions()
    cloud_node.worker.demands[("nav", "mapcloud")] = cloud_node.now
    cloud_node.raise_on_create = True
    cloud_node.reconcile_cloud_subscriptions()
    assert cloud_node.subscriptions == []
    assert len(cloud_node.logger.warnings) == 1

    cloud_node.raise_on_create = False
    cloud_node.now += 1.0
    cloud_node.reconcile_cloud_subscriptions()
    assert cloud_node.subscriptions == []  # still inside the 5 s backoff

    cloud_node.now += 5.0
    cloud_node.reconcile_cloud_subscriptions()
    assert len(cloud_node.subscriptions) == 1
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_idle_cloud_subscriptions.py -q`
Expected: FAIL — `AttributeError: type object 'WebControlNode' has no attribute 'reconcile_cloud_subscriptions'`.

- [ ] **Step 3: Implement the plan-driven configure and create path**

In `web_control/web_control_node.py`, add to the pointcloud_runtime import block (after `DEFAULT_DEMAND_TTL_SEC`):

```python
from web_control.cloud_subscriptions import (
    PLAN_BY_KEY,
    reconcile as reconcile_cloud_demand,
)
```

Add module constants next to `DEFAULT_DEMAND_TTL_SEC` usage (before `class WebControlNode`):

```python
# The reconciler runs on the ROS executor thread so subscription entities are
# created and destroyed where their callbacks are dispatched.
CLOUD_SUBSCRIPTION_TICK_SEC = 0.2
CLOUD_SUBSCRIPTION_RETRY_SEC = 5.0
DEFAULT_CLOUD_SUBSCRIPTION_IDLE_SEC = 5.0
_CLOUD_LAYER_CALLBACKS = frozenset(
    {"pointcloud_callback", "mapping_pointcloud_callback"}
)
```

Replace the whole of `configure_pointcloud_subscriptions` (from the `def` to
the end of `configure_managed_processes`'s preceding method) with:

```python
    def configure_pointcloud_subscriptions(self) -> None:
        """Arm demand-driven subscriptions for the browser cloud layers.

        Nothing is subscribed at start-up.  The reconciler creates one
        subscription per layer while a browser displays it and releases it,
        with its decoded cache, once the idle grace period expires.  Latched
        topics replay their last sample on re-subscription, so a released map
        returns without any publisher change.
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
        create, release = reconcile_cloud_demand(
            worker.now(),
            self.cloud_subscriptions.keys(),
            worker.demand_stamps(),
            demand_ttl_sec=self.cloud_demand_ttl,
            # Task 6 declares the parameter; the default keeps this commit
            # runnable on its own.
            idle_sec=getattr(
                self, "cloud_subscription_idle",
                DEFAULT_CLOUD_SUBSCRIPTION_IDLE_SEC,
            ),
        )
        for key in release:
            self.release_cloud_subscription(key)
        for key in create:
            self.create_cloud_subscription(key)

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
        now = worker.now()
        if not worker.demand_active(key, now=now):
            return False
        failed_at = self._cloud_subscription_failures.get(key)
        if failed_at is not None and now - failed_at < CLOUD_SUBSCRIPTION_RETRY_SEC:
            return False
        topic = str(self.get_parameter(plan.topic_parameter).value or "").strip()
        if not topic:
            return False
        message_type = {
            "PointCloud2": PointCloud2,
            "MarkerArray": MarkerArray,
        }.get(plan.message_type)
        if message_type is None:
            return False
        if plan.callback in _CLOUD_LAYER_CALLBACKS:
            callback = partial(getattr(self, plan.callback), plan.layer)
        else:
            callback = getattr(self, plan.callback)
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
```

The `MarkerArray`/`PointCloud2` names are the existing module imports that may
be `None` on a partial installation.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_idle_cloud_subscriptions.py -q`
Expected: 12 passed.  Release is not implemented yet; the reconciler only
creates because `release` is empty while every demand is fresh.

- [ ] **Step 5: Commit**

```bash
cd /home/robot/web_ws/src/web_control
git add web_control/web_control_node.py test/test_idle_cloud_subscriptions.py
git commit -m "feat: create cloud subscriptions on demand"
```

---

### Task 4: Release idle subscriptions and caches

**Files:**
- Modify: `web_control/web_control_node.py` (add `release_cloud_subscription`, `release_all_cloud_subscriptions` after `create_cloud_subscription`)
- Test: `test/test_idle_cloud_subscriptions.py`

- [ ] **Step 1: Write the failing release tests**

First extend `_bind_cloud` in `test/test_idle_cloud_subscriptions.py` with the
release methods:

```python
        "release_cloud_subscription",
        "release_all_cloud_subscriptions",
```

Then append to the same file:

```python
def test_release_destroys_the_subscription_and_clears_the_cache(cloud_node):
    cloud_node.configure_pointcloud_subscriptions()
    cloud_node.worker.demands[("nav", "mapcloud")] = cloud_node.now
    cloud_node.reconcile_cloud_subscriptions()
    cloud_node.layers["mapcloud"].update(
        {"points3d": [[1.0, 2.0, 3.0]]}, stamp=cloud_node.now, frame_id="map", count=1
    )
    assert cloud_node.layers["mapcloud"].version == 1

    cloud_node.now += 6.0
    cloud_node.reconcile_cloud_subscriptions()

    assert cloud_node.cloud_subscriptions == {}
    assert len(cloud_node.destroyed) == 1
    assert cloud_node.layers["mapcloud"].data is None
    assert cloud_node.layers["mapcloud"].version == 2
    assert cloud_node.metadata_publishes >= 1
    assert ("nav", "mapcloud") not in cloud_node.worker.demands


def test_release_keeps_other_subscriptions(cloud_node):
    cloud_node.configure_pointcloud_subscriptions()
    cloud_node.worker.demands[("nav", "mapcloud")] = cloud_node.now
    cloud_node.worker.demands[("nav", "obstacles")] = cloud_node.now
    cloud_node.reconcile_cloud_subscriptions()
    assert len(cloud_node.cloud_subscriptions) == 2

    cloud_node.now += 6.0
    cloud_node.worker.demands[("nav", "obstacles")] = cloud_node.now
    cloud_node.reconcile_cloud_subscriptions()

    assert set(cloud_node.cloud_subscriptions) == {("nav", "obstacles")}
    assert len(cloud_node.destroyed) == 1


def test_release_all_clouds_destroys_every_handle(cloud_node):
    cloud_node.configure_pointcloud_subscriptions()
    cloud_node.worker.demands[("nav", "mapcloud")] = cloud_node.now
    cloud_node.worker.demands[("mapping", "ground")] = cloud_node.now
    cloud_node.reconcile_cloud_subscriptions()
    assert len(cloud_node.cloud_subscriptions) == 2

    cloud_node.release_all_cloud_subscriptions()

    assert cloud_node.cloud_subscriptions == {}
    assert len(cloud_node.destroyed) == 2
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_idle_cloud_subscriptions.py -q -k "release"`
Expected: FAIL — `release_cloud_subscription` does not exist.

- [ ] **Step 3: Implement the release path**

Add after `create_cloud_subscription` in `web_control/web_control_node.py`:

```python
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
        with self.lock:
            caches = self.layers if key[0] == "nav" else self.mapping_layers
            cache = caches.get(key[1])
            if cache is not None and cache.data is not None:
                cache.clear()
        publish_cloud_metadata_if_available(self)
        return subscription is not None

    def release_all_cloud_subscriptions(self) -> None:
        """Destroy every cloud subscription without publishing revisions."""
        destroy = getattr(self, "destroy_subscription", None)
        for key in list(self.cloud_subscriptions):
            subscription = self.cloud_subscriptions.pop(key)
            if destroy is None:
                continue
            try:
                destroy(subscription)
            except Exception:
                pass
        self._cloud_subscription_failures.clear()
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_idle_cloud_subscriptions.py -q`
Expected: 15 passed.

- [ ] **Step 5: Commit**

```bash
cd /home/robot/web_ws/src/web_control
git add web_control/web_control_node.py test/test_idle_cloud_subscriptions.py
git commit -m "feat: release idle cloud subscriptions and decoded caches"
```

---

### Task 5: Navigation readiness independent of display caches

**Files:**
- Modify: `web_control/web_control_node.py` (helper near `navigation_readiness`, `self.layers` init, `_apply_pointcloud_result`, `clear_navigation_runtime_state`, `clear_map_runtime_state`, `state_payload`)
- Test: `test/test_idle_cloud_subscriptions.py`, `test/test_navigation_global_layers.py:465-500`

- [ ] **Step 1: Write the failing readiness tests**

Append to `test/test_idle_cloud_subscriptions.py`:

```python
def test_mapground_frame_marks_navigation_ready_without_display_cache(cloud_node):
    cloud_node.mapground_seen = False
    cloud_node._apply_pointcloud_result = MethodType(
        web_control_node.WebControlNode._apply_pointcloud_result, cloud_node
    )

    assert cloud_node._apply_pointcloud_result(
        "nav", "mapground", [[1.0, 2.0, 0.0]], "map", 1, 10.0, False, 0
    )
    assert cloud_node.mapground_seen is True

    cloud_node.layers["mapground"].clear()
    assert web_control_node.effective_mapground_status(
        cloud_node.layers["mapground"].status(11.0), cloud_node.mapground_seen
    ) == "loaded"


def test_effective_mapground_status_falls_back_to_the_cache():
    assert web_control_node.effective_mapground_status("waiting", False) == "waiting"
    assert web_control_node.effective_mapground_status("stale", False) == "stale"
    assert web_control_node.effective_mapground_status("waiting", True) == "loaded"
```

Extend `test_navigation_stop_clears_nav_clouds_but_keeps_localization_map` in
`test/test_navigation_global_layers.py` with, before
`node.clear_navigation_runtime_state()`:

```python
    node.mapground_seen = True
```

and after the existing asserts:

```python
    assert node.mapground_seen is False
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_idle_cloud_subscriptions.py::test_mapground_frame_marks_navigation_ready_without_display_cache test/test_idle_cloud_subscriptions.py::test_effective_mapground_status_falls_back_to_the_cache test/test_navigation_global_layers.py::test_navigation_stop_clears_nav_clouds_but_keeps_localization_map -q`
Expected: FAIL — `effective_mapground_status` is missing and `mapground_seen` is never set.

- [ ] **Step 3: Implement the readiness flag**

Add next to `navigation_readiness` in `web_control/web_control_node.py`:

```python
def effective_mapground_status(cache_status: str, seen: bool) -> str:
    """Readiness follows the lifecycle flag, not the display cache.

    Releasing an idle subscription clears the decoded map ground while the
    navigation stack still owns a valid map, so a cache-derived status would
    report "waiting for navigation" during normal operation.  ``seen`` is set
    when a ground frame is applied and reset by the map/navigation lifecycle
    clears.
    """
    return "loaded" if seen else cache_status
```

Initialise the flag with the layer caches:

```python
        self.mapground_seen = False
```

In `_apply_pointcloud_result`, inside the `else` branch that updates the cache:

```python
            else:
                cache.update(
                    {"points3d": points},
                    stamp=arrival_stamp,
                    frame_id=frame_id,
                    count=finite_count,
                )
                changed = True
                if group == "nav" and layer_name == "mapground":
                    self.mapground_seen = True
```

In `clear_navigation_runtime_state` and `clear_map_runtime_state`, inside their
`with self.lock:` blocks, add:

```python
            self.mapground_seen = False
```

In `state_payload`, replace:

```python
            mapground_status = self.layers["mapground"].status(now)
```

with:

```python
            mapground_status = effective_mapground_status(
                self.layers["mapground"].status(now),
                getattr(self, "mapground_seen", False),
            )
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_idle_cloud_subscriptions.py test/test_navigation_global_layers.py::test_navigation_stop_clears_nav_clouds_but_keeps_localization_map test/test_core.py::test_navigation_readiness_waits_for_relocalization_then_navigation -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/robot/web_ws/src/web_control
git add web_control/web_control_node.py test/test_idle_cloud_subscriptions.py test/test_navigation_global_layers.py
git commit -m "fix: keep navigation readiness independent of display caches"
```

---

### Task 6: Idle parameter and shutdown cleanup

**Files:**
- Modify: `web_control/web_control_node.py` (parameter declaration/read, `destroy_node`), `config/web_control.yaml`
- Test: `test/test_idle_cloud_subscriptions.py`

- [ ] **Step 1: Write the failing configuration test**

Append to `test/test_idle_cloud_subscriptions.py`:

```python
def test_config_declares_idle_parameter_with_a_positive_default():
    import yaml

    config = yaml.safe_load(
        (
            __import__("pathlib").Path(__file__).resolve().parents[1]
            / "config"
            / "web_control.yaml"
        ).read_text(encoding="utf-8")
    )["web_control"]["ros__parameters"]
    assert config["cloud_subscription_idle_sec"] == 5.0
    assert config["cloud_demand_ttl_sec"] == 2.0
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_idle_cloud_subscriptions.py -q -k idle_parameter`
Expected: FAIL — the YAML has no `cloud_subscription_idle_sec`.

- [ ] **Step 3: Declare, read and clean up**

In `_declare_parameters`, after `self.declare_parameter("cloud_demand_ttl_sec", DEFAULT_DEMAND_TTL_SEC)`:

```python
        # Seconds after the demand expires before the subscription and its
        # decoded cache are released.  0 keeps subscriptions for the process
        # lifetime (the pre-demand-driven behaviour).
        self.declare_parameter(
            "cloud_subscription_idle_sec", DEFAULT_CLOUD_SUBSCRIPTION_IDLE_SEC
        )
```

Next to `self.cloud_demand_ttl = ...`:

```python
        self.cloud_subscription_idle = max(
            0.0,
            float(self.get_parameter("cloud_subscription_idle_sec").value),
        )
```

In `destroy_node`, before `advance_cloud_generation(self)`:

```python
        subscription_timer = getattr(self, "_cloud_subscription_timer", None)
        if subscription_timer is not None:
            subscription_timer.cancel()
        release_subscriptions = getattr(
            self, "release_all_cloud_subscriptions", None
        )
        if release_subscriptions is not None:
            release_subscriptions()
```

Also finish the benchmark paragraph: change "All layers retain only the bounded
3D cache, not the raw ROS buffer." to "Decoded layers retain only the 3D cache;
a never-demanded layer keeps at most its one pending raw frame." and stage
`docs/benchmarks/pointcloud-optimization.md` with this task.

While in the release code, add the shutdown-only precondition to
`release_all_cloud_subscriptions`' docstring ("Called from destroy_node before
the global generation bump; it neither publishes revisions nor clears caches")
and add a mapping-layer release test to `test/test_idle_cloud_subscriptions.py`:

```python
def test_release_clears_a_mapping_cache(cloud_node):
    cloud_node.configure_pointcloud_subscriptions()
    cloud_node.worker.demands[("mapping", "ground")] = cloud_node.now
    cloud_node.reconcile_cloud_subscriptions()
    cloud_node.mapping_layers["ground"].update(
        {"points3d": [[1.0, 2.0, 3.0]]}, stamp=cloud_node.now, frame_id="map", count=1
    )

    cloud_node.now += 6.0
    cloud_node.reconcile_cloud_subscriptions()

    assert cloud_node.mapping_layers["ground"].data is None
    assert cloud_node.mapping_layers["ground"].version == 2
    assert len(cloud_node.destroyed) == 1
```

In `config/web_control.yaml`, next to `cloud_demand_ttl_sec`:

```yaml
    cloud_demand_ttl_sec: 2.0             # Pause cloud decode after last map request (s)
    cloud_subscription_idle_sec: 5.0      # Destroy idle cloud subscriptions and free their decoded cache (s); 0 disables
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_idle_cloud_subscriptions.py -q`
Expected: 18 passed.

- [ ] **Step 5: Commit**

```bash
cd /home/robot/web_ws/src/web_control
git add web_control/web_control_node.py config/web_control.yaml test/test_idle_cloud_subscriptions.py docs/benchmarks/pointcloud-optimization.md
git commit -m "feat: add cloud subscription idle period and shutdown cleanup"
```

---

### Task 7: Align the legacy subscription tests

**Files:**
- Modify: `test/test_navigation_global_layers.py:72-150`, `test/test_current_lidar_layer.py:109-150`
- Modify: `test/test_core.py` (three source-string tests below)

Three `test_core.py` tests assert source strings that the plan-driven
reconciler removed.  Update them to assert the plan table instead of the old
inline QoS constants:

1. `test_navigation_page_adds_localization_global_map_and_obstacles` —
   replace `assert 'self.get_parameter("obstacle_topic").value' in node_source`
   with:

   ```python
   from web_control.cloud_subscriptions import PLAN_BY_KEY

   assert PLAN_BY_KEY[("nav", "obstacles")].topic_parameter == "obstacle_topic"
   ```

2. `test_web_control_subscribes_mapping_topics_and_exposes_mapping_api` —
   replace `assert "self.mapping_pointcloud_callback" in web_control_node`
   with:

   ```python
   from web_control.cloud_subscriptions import PLAN_BY_KEY

   assert (
       PLAN_BY_KEY[("mapping", "ground")].callback
       == "mapping_pointcloud_callback"
   )
   ```

3. `test_web_control_uses_compatible_qos_profiles_for_visualization_topics` —
   delete the four assertions on `static_pointcloud_qos`/`pointcloud_qos` and
   assert the shared helper plus one plan per QoS class:

   ```python
   from web_control.cloud_subscriptions import PLAN_BY_KEY

   assert "def cloud_layer_qos(self, qos_name: str)" in web_control_node
   assert "durability=QoSDurabilityPolicy.TRANSIENT_LOCAL" in web_control_node
   assert "durability=QoSDurabilityPolicy.VOLATILE" in web_control_node
   assert PLAN_BY_KEY[("nav", "mapcloud")].qos == "latched"
   assert PLAN_BY_KEY[("nav", "obstacles")].qos == "live"
   ```

Keep the assertions on `state_qos`/`Odometry`/`PathMsg`/`GoalStatusArray`
unchanged; those subscriptions did not move.

- [ ] **Step 1: Update the navigation layer stub and helper**

In `test/test_navigation_global_layers.py`, extend `_SubscriptionNode.__init__` with:

```python
        self.now = 10.0
        self.cloud_demand_ttl = 30.0
        self.cloud_subscription_idle = 5.0
        self.lock = threading.RLock()
        self.layers = {
            name: SourceCache(name, 2.0, static_source=True)
            for name in (
                "global_map",
                "local_map",
                "current_lidar",
                "mapcloud",
                "mapground",
                "obstacles",
                "grid_terrain",
            )
        }
        self.mapping_layers = {
            "global_map": SourceCache("mapping_global_map", 2.0, static_source=True),
            "ground": SourceCache("mapping_ground", 2.0, static_source=True),
        }
        self.cloud_subscriptions = {}
        self._cloud_subscription_failures = {}
        self._cloud_generations = {}
        self._cloud_generation = 0
        self.timer = None
        self.demanded = set()

    def now_sec(self):
        return self.now

    def get_logger(self):
        return SimpleNamespace(warning=lambda message: None)

    def create_timer(self, period, callback):
        self.timer = (period, callback)
        return object()

    def destroy_subscription(self, subscription):
        return None
```

Replace `_configure_subscriptions` with:

```python
def _configure_subscriptions(monkeypatch, node):
    monkeypatch.setattr(web_control_node, "QoSProfile", _QosProfile)
    monkeypatch.setattr(
        web_control_node,
        "QoSHistoryPolicy",
        SimpleNamespace(KEEP_LAST="keep_last"),
    )
    monkeypatch.setattr(
        web_control_node,
        "QoSReliabilityPolicy",
        SimpleNamespace(RELIABLE="reliable", BEST_EFFORT="best_effort"),
    )
    monkeypatch.setattr(
        web_control_node,
        "QoSDurabilityPolicy",
        SimpleNamespace(TRANSIENT_LOCAL="transient_local", VOLATILE="volatile"),
    )
    monkeypatch.setattr(web_control_node, "PointCloud2", object)
    monkeypatch.setattr(web_control_node, "MarkerArray", object)
    node.configure_pointcloud_subscriptions = MethodType(
        web_control_node.WebControlNode.configure_pointcloud_subscriptions, node
    )
    node.create_cloud_subscription = MethodType(
        web_control_node.WebControlNode.create_cloud_subscription, node
    )
    node.cloud_layer_qos = MethodType(
        web_control_node.WebControlNode.cloud_layer_qos, node
    )
    node.configure_pointcloud_subscriptions()
    assert node.subscriptions == [], "no cloud subscription exists before demand"

    node.cloud_worker = SimpleNamespace(
        demand_active=lambda key, *, now=None: True,
        demand_stamps=lambda: {},
        now=lambda: node.now,
        invalidate=lambda keys=None: None,
    )
    for key in web_control_node.PLAN_BY_KEY:
        node.create_cloud_subscription(key)
    return {entry["topic"]: entry for entry in node.subscriptions}
```

- [ ] **Step 2: Apply the same change to the current-lidar test**

In `test/test_current_lidar_layer.py`, make the identical edits to
`_SubscriptionNode` and `_configure_subscriptions` (the two files already share
the stub verbatim).  Keep the file's existing layer caches; only the helper
changes.

- [ ] **Step 3: Run the two legacy files**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test/test_navigation_global_layers.py test/test_current_lidar_layer.py -q`
Expected: PASS.  The tests that assert callback routing and QoS still read the
same `subscriptions` entries, now created through the plan.

- [ ] **Step 4: Run the whole package suite**

Run: `cd /home/robot/web_ws/src/web_control && python3 -m pytest test -q`
Expected: the idle-subscription tests pass.  Compare the remaining failures
against the pre-change baseline (123 failing tests in this working tree before
the change, all unrelated HTTP/front-end stubs); no new failure may reference
cloud subscriptions, demand, pose or perception windows.

- [ ] **Step 5: Commit**

```bash
cd /home/robot/web_ws/src/web_control
git add test/test_navigation_global_layers.py test/test_current_lidar_layer.py
git commit -m "test: align cloud layer tests with lazy subscriptions"
```

---

## Self-Review Notes

- Spec coverage: planner (Task 1), worker gating and `demand_stamps` (Task 2),
  lazy creation (Task 3), release with cache clearing (Task 4), readiness flag
  (Task 5), parameter plus shutdown cleanup (Task 6), legacy test alignment
  (Task 7).
- The `release_cloud_subscription` / `release_all_cloud_subscriptions` names are
  used consistently in Tasks 3, 4 and 6.
- `create_cloud_subscription` depends on `PointCloud2` / `MarkerArray` module
  imports, which the tests monkeypatch.
