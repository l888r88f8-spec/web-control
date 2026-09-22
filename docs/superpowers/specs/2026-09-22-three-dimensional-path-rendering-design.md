# Three-Dimensional Path Rendering Design

## Goal

Render navigation and mapping paths at their real ROS `z` coordinates so paths
on stacked floors appear on the correct floor in the web 3D view.

## Scope

This change only covers path transport and rendering in `web_control`.
Localization ground-height selection, goal floor selection, and initial-pose
height are outside this change.

## Data Flow

1. `WebControlNode.path_callback` stores every navigation path pose as
   `[x, y, z]` instead of `[x, y]`.
2. `WebControlNode.mapping_path_callback` uses the same three-dimensional
   representation.
3. Existing path API responses expose these three-value points without another
   conversion.
4. `updatePaths` and `updateMappingPath` pass the stored `z` coordinate through
   `rosToThree`.
5. Rendering adds `0.08 m` to the real `z` so the tube remains visible just
   above the ground surface.

## Compatibility

The browser accepts legacy two-value `[x, y]` points and renders them at
`z=0.08`. New server responses always contain `[x, y, z]`.

## Tests

- A backend test verifies both navigation and mapping callbacks retain pose
  height.
- A Node-based frontend test verifies two paths with different ROS heights are
  rendered at different Three.js heights.
- Existing web tests, JavaScript syntax checks, Python compilation, and diff
  checks remain green.

## Acceptance Criteria

- A `/global_path` pose at upper-floor height is returned by the web API with
  that height intact.
- Navigation and mapping path tubes use their actual height plus the small
  visibility offset.
- Legacy two-dimensional path points still render without errors.
