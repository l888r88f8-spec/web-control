# Idle point-cloud subscriptions

Date: 2026-11-21
Status: approved

## Problem

Decoding already pauses per layer when no browser has requested it, but the
node still holds a ROS subscription for every cloud topic from start-up until
shutdown.  Even at depth 1 that keeps the DDS copy, deserialization buffer and
subscription callback load alive for topics nobody is looking at.  The decoded
caches are worse: with `max_points = 0` every retained cloud is an unbounded
Python list, so an untouched `mapcloud` keeps hundreds of megabytes resident.

Three navigation maps (`global_map`, `mapcloud`, `mapground`) and
`grid_terrain` bypass demand entirely, so they decode even with no browser
connected.

## Behaviour

- A cloud topic is subscribed only while at least one browser displays that
  layer, plus a grace period after the last request.
- When the grace period expires the backend destroys the subscription, drops
  the pending raw frame, clears the decoded cache and publishes a `waiting`
  revision so the browser forgets the geometry.
- Re-displaying a layer re-creates the subscription.  Latched publishers
  (`RELIABLE` / `TRANSIENT_LOCAL`) replay their last sample; volatile topics
  deliver on their next publish.  The layer then decodes and the browser pulls
  it on the new revision.
- Non-cloud subscriptions (paths, odometry, action status, TF, diagnostics) and
  the teleop/goal publishers keep their current lifecycle.
- Navigation readiness must not depend on the display cache: `mapground` may be
  released while navigation runs, so readiness uses a separate "map ground
  seen since the last lifecycle clear" flag.  A fresh process with no viewer
  reports "waiting for navigation" until the first browser demands
  `mapground` (within one reconcile tick plus one decode); nothing in the
  control path reads that readiness, only `/api/state` consumers do.

## Components

### `web_control/cloud_subscriptions.py` (new, ROS-free)

- `CloudSubscriptionPlan`: group, layer, topic parameter name, message type
  name, QoS profile name (`latched` or `live`), callback name.
  `CLOUD_SUBSCRIPTION_PLANS` covers the seven navigation layers, the two
  Mapping layers and `grid_terrain`.
- `reconcile(now, subscribed, last_demand, *, demand_ttl_sec, idle_sec)`:
  pure decision function returning `(create, release)` key tuples.
  - `create` = keys demanded within `demand_ttl_sec` with no subscription;
  - `release` = subscribed keys whose last demand is older than
    `max(idle_sec, demand_ttl_sec)`, so a still-displayed layer can never be
    released and immediately re-created;
  - `idle_sec <= 0` disables releasing entirely (escape hatch);
  - `now` and the demand stamps share the worker clock (`time.monotonic` by
    default).  The node passes `worker.now()`; a wall-clock value would make
    every demand look ancient.
  A key never appears in both sets: once chosen for release it is not also
  reported as demanded.

### `PointCloudIngestWorker` (`pointcloud_runtime.py`)

- Remove the `STATIC_LAYERS` demand exemption: every layer needs an unexpired
  demand.  `LAYER_RATES` keeps its 1 Hz ceiling for the latched maps.
- Add `demand_stamps()` returning a copy of `{(group, layer): stamp}` for the
  reconciler.
- Keep `invalidate(keys)` as the single teardown of pending frames and demand.

### `WebControlNode`

- `configure_pointcloud_subscriptions()` builds the plan table and starts a
  0.2 s reconciler timer instead of creating subscriptions.
- `_reconcile_cloud_subscriptions()` runs on the executor thread:
  1. `desired, release = reconcile(...)`;
  2. create missing subscriptions from the plan table;
  3. for each release: `advance_cloud_generation(self, [key])`, clear the layer
     cache under `self.lock`, mark `mapground_seen` accordingly and publish
     cloud metadata.
- Subscription handles live in `self.cloud_subscriptions[key]`; creation
  failures are logged once per key and retried on a later tick.
- `destroy_node()` cancels the reconciler timer and destroys any remaining
  subscriptions before stopping the worker.
- `mapground_seen` is set when a `mapground` frame is applied, cleared by the
  existing navigation/map lifecycle clears, and used by `navigation_readiness`.

### Configuration (`config/web_control.yaml`)

- `cloud_demand_ttl_sec: 2.0` — seconds a layer counts as displayed after the
  last HTTP/SSE request; unchanged meaning.
- `cloud_subscription_idle_sec: 5.0` — new; seconds after the demand expires
  before the subscription is destroyed and the decoded cache released.  Must be
  at least the TTL; larger values absorb checkbox flicker at the cost of idle
  resources.  `0` keeps subscriptions for the process lifetime (old behaviour).

## Data flow

Browser checkbox change → new SSE URL with the visible layer set → one demand
stamp per second per visible layer → reconciler creates the subscription
within 0.2 s → publisher sample → worker decode → metadata revision → browser
fetches `/api/map3d`.  Unchecking, closing the tab or leaving the view lets the
demand expire (2 s) and the subscription go idle (5 s).

## Error handling

- Creating a subscription raises: log a warning for that key at most once every
  5 s, keep it out of `cloud_subscriptions`, retry next tick.
- `grid_terrain` conversion stops with its subscription; the latched
  `MarkerArray` replays on re-subscription.
- A released layer's cached body is invalidated through
  `advance_cloud_generation`, so an in-flight decode cannot resurrect geometry.
- The browser shows `waiting` for a released layer; the next revision restores
  it.

## Testing

- `reconcile`: create on fresh demand, keep during grace, release after idle,
  never create without demand, `idle_sec = 0` escape hatch.
- Worker: no layer decodes without demand, including the three latched maps;
  `demand_stamps()` reflects the last demand.
- Node stub: first demand creates exactly the planned subscriptions; idle
  release destroys them, clears the cache and publishes `waiting`; creation
  failure retries; destroy_node releases everything.
- Readiness: `mapground_seen` keeps navigation ready after its cache is
  released and resets on a map change.
- Existing eager-subscription assertions in `test_navigation_global_layers.py`
  and `test_current_lidar_layer.py` move to "after the first demand".
