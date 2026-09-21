# Web Navigation Pause and Resume Design

## Goal

Allow an operator to pause the current navigation goal without stopping the
navigation launch, resume the saved goal from the robot's current position, or
clear the goal permanently. A paused goal survives browser refreshes and
`web_control` node restarts. Publishing a new goal always replaces a paused
goal.

## Existing behavior and constraints

- The browser sends a pose to `POST /api/goal`; `web_control` publishes it as a
  `geometry_msgs/msg/PoseStamped` on `goal_pose_3d`.
- `clicked2goal.py` converts that topic message into a
  `dddmr_sys_core/action/PToPMoveBase` goal on `/p2p_move_base`.
- `web_control` subscribes to `/p2p_move_base/_action/status`, but it is not an
  Action client and does not own the resulting goal handle.
- The existing Navigation **Stop** button terminates the complete navigation
  launch. It remains available and keeps that meaning.
- The Action server supports cancellation through
  `/p2p_move_base/_action/cancel_goal` and stops the route while publishing zero
  velocity when it processes a cancellation.
- No independent program sends goals to this Action server. A zero UUID
  `action_msgs/srv/CancelGoal` request can therefore safely cancel all server
  goals and represents cancellation of the current navigation goal.
- Cancellation ends an Action goal. Resume must publish the saved destination
  again and let navigation compute a new route from the robot's current pose.
  It does not preserve the old planner's internal progress.

## User interface

Place two controls in the Navigation page's **Target Pose** section:

1. One state-dependent primary button:
   - `Pause Navigation` while a goal is accepted or executing.
   - `Pausing...` while cancellation is in progress.
   - `Resume Navigation` while a destination is paused.
   - `Resuming...` after republishing the destination and before the new Action
     becomes accepted or executing.
   - Disabled `Pause Navigation` when there is no current or paused goal.
2. A `Clear Goal` button, enabled whenever a current or paused goal exists. It
   reads `Clearing...` and is disabled while an active goal is being cleared.

The page shows one of `Idle`, `Navigating`, `Pausing`, `Paused`, `Resuming`, or
`Clearing`. During a transitional state, controls that could publish another
goal are disabled so old and new Action state transitions cannot overlap.

After a pause is confirmed, the destination remains visible as the paused
destination and Resume becomes available. After Clear, success, or a terminal
failure, the target marker and navigation paths are cleared. The existing
Navigation Stop button remains in the terminal section and continues to stop
the launch rather than operate on a goal.

## State and command semantics

The server owns navigation control state. The browser renders that state and
does not treat its local variables as authoritative.

```text
Idle -- publish goal --> Navigating
Navigating -- Pause --> Pausing -- canceled --> Paused
Paused -- Resume --> Resuming -- accepted/executing --> Navigating
Paused -- publish new goal --> Navigating (old paused goal is discarded)
Navigating -- Clear --> Clearing -- canceled --> Idle
Paused -- Clear --> Idle
Navigating -- succeeded/aborted --> Idle
```

When Pause begins, the server records that cancellation is intended to retain
the goal. When Clear cancels an active goal, it records that cancellation is
intended to discard the goal. This pending intent lets the same Action
`canceled` status produce the correct final state.

A newly published goal takes priority over a paused goal. The server removes
the paused record and publishes the new destination. Publishing is rejected
while a pause or resume transition is still pending. Resume republishes the
same x, y, z, yaw, and frame saved by Pause.

Action status selection must prefer active states (`accepted`, `executing`, or
`canceling`) and otherwise use the newest goal timestamp. It must not depend
only on the last array element because `GoalStatusArray` can contain recently
completed goals as well as the current one.

## Backend API and ROS integration

Add a configurable ROS parameter with this default:

```yaml
cancel_goal_service: "/p2p_move_base/_action/cancel_goal"
```

`web_control` creates an `action_msgs/srv/CancelGoal` client. It uses a zero
goal UUID and zero timestamp. This keeps the web package independent of
`dddmr_sys_core` and requires no navigation package changes.

Add these HTTP endpoints:

- `POST /api/navigation/pause`: validate an executing goal, preserve its
  destination, send the cancel request, and enter `Pausing`.
- `POST /api/navigation/resume`: validate the saved destination and selected
  map, republish the destination, and enter `Resuming`.
- `POST /api/navigation/clear`: cancel an active goal when necessary and remove
  the current and paused destination. Clearing an already paused destination
  does not call the Action service.

The cancel operation waits for the service response for a short bounded period
in the HTTP worker thread. The ROS executor continues spinning and can deliver
the service response. A successful service response means cancellation was
accepted; the state reaches `Paused` or `Idle` only after the Action status
reports `canceled`.

Extend `GET /api/state` with server-owned controls such as:

```json
{
  "navigation_control": {
    "state": "paused",
    "has_goal": true,
    "can_pause": false,
    "can_resume": true,
    "can_clear": true
  }
}
```

The browser derives button labels and enabled states from these values. It
continues polling the existing state endpoint after a command, so a refresh or
another browser sees the same result.

## Persistence and map safety

Persist resumable state atomically at:

```text
~/.config/web_control/paused_navigation.json
```

The record contains a schema version, lifecycle phase, x/y/z/yaw, frame, saved
map name, and timestamp. Writes use a fully written temporary sibling followed
by `os.replace`, matching the existing selected-map persistence pattern.

Pause first writes the destination with phase `pausing`, then sends the cancel
request. An explicit cancellation rejection removes that record and returns to
`Navigating`; a timeout retains it because the cancellation result is unknown.
The confirmed `canceled` status updates the phase to `paused`. Resume changes
the phase to `resuming` before publishing. It removes the record only after the
new goal is accepted or executing; a definite publish failure restores
`paused`.

On startup, the server validates the JSON types, finite coordinate values,
schema version, frame, and saved map name before exposing Resume. Invalid or
unreadable data is ignored with a warning. Resume is allowed only when the
currently selected map matches the saved map. Selecting another map clears the
paused destination because coordinates from the old map are not valid in the
new map.

Startup also reconciles an interrupted transition after receiving fresh Action
status. A `pausing` record with an active goal retries cancellation; without an
active goal it becomes `paused`. A `resuming` record with an active goal is
removed and state becomes `Navigating`; without an active goal it returns to
`paused`. This prevents a process restart during a command from silently losing
the destination or publishing it twice.

The paused record is removed when a replacement goal becomes active, Resume
successfully reaches `accepted` or `executing`, Clear completes, or the saved
map changes. A browser refresh has no effect on the record. Restarting
`web_control` reloads it and reconstructs the paused controls.

## Failure handling

- If navigation or the cancel service is unavailable, Pause reports a clear
  error and does not claim the robot is paused.
- A cancel response with no matching goal reports that no active navigation
  goal exists. State is reconciled with the latest Action status.
- A cancel timeout keeps the saved destination and reports that the result is
  uncertain. State polling determines whether cancellation later completes;
  the UI does not clear the route early.
- Duplicate Pause or Resume requests return a conflict response and do not send
  duplicate ROS commands.
- Resume with a missing, corrupt, or different-map record is rejected without
  publishing a pose.
- Clear remains able to delete a paused record when navigation is stopped. If a
  goal is active, it is not reported as cleared until cancellation is accepted.
- A replacement goal is rejected during `Pausing` or `Resuming`. Once `Paused`,
  a replacement goal discards the old paused destination and runs normally.

## Verification

Automated tests cover:

- the navigation state transitions and legal commands;
- Pause cancellation without terminating the managed navigation process;
- distinction between Pause cancellation and Clear cancellation;
- Resume republishing exactly the saved pose;
- a new goal replacing and deleting the paused destination;
- duplicate commands, unavailable service, empty cancellation response, and
  timeout behavior;
- atomic persistence, restart reload, corrupt files, coordinate validation, and
  map mismatch invalidation;
- selection of the active/newest Action status from a mixed status array;
- HTTP response status and `/api/state` button capabilities;
- frontend labels, disabled states, terminal-state cleanup, and JavaScript
  syntax.

The implementation should run the web package's Python test suite,
`node --check static/app.js`, Python syntax compilation, and `git diff --check`.
The user will compile the ROS workspace separately.
