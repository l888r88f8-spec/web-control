"""Demand-driven ROS subscription plans for browser cloud layers.

The node performs ``create_subscription`` / ``destroy_subscription`` on the
executor thread; this module only decides which ``(group, layer)`` keys should
stay subscribed and describes how to subscribe them.
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

    ``now`` and every ``last_demand`` stamp must come from the same clock,
    namely the worker clock (``PointCloudIngestWorker._clock``, which defaults
    to ``time.monotonic``) -- never a wall clock such as ``time.time()``.

    ``create`` holds demanded keys with no subscription.  ``release`` holds
    subscribed keys whose last demand is older than
    ``max(idle_sec, demand_ttl_sec)``, including keys with no demand record at
    all.  ``idle_sec <= 0`` keeps every subscription for the process lifetime.
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
