# Web Map Selector Design

## Goal

Let an operator choose one map from the web node user's `~/map` directory. The next web-managed localization and navigation launches must use that same map. Remember the choice across web node restarts. Map selection does not reload a running ROS process.

## Existing behavior and constraints

- Localization currently loads `localization.map_path` from `lio_sam_hesai/config/relocalization.yaml` during node construction. Its current default points to `hesai_outdoor_yifu/tiled`.
- Navigation's `p2p_move_base_localization.launch.py` already accepts `map_dir` and `ground_dir`; its current defaults point to `hesai_sim2`.
- The web node owns the localization and navigation launch processes. The external `~/bringup.sh` and direct `ros2 launch` commands remain outside this feature.
- `~/map/<name>` contains `GlobalMap.pcd` and `GroundMap.pcd` for each currently usable map. Some maps also contain `tiled/manifest.yaml` and `sc_db.bin`.
- Neither localization nor navigation reloads its map while running. The user must stop both affected web-managed processes before choosing a different map.

## Map catalog

The server scans only direct child directories of the web node user's `~/map`. It accepts a map as selectable when both `GlobalMap.pcd` and `GroundMap.pcd` are regular readable files. Names are returned in stable sorted order. The API returns the directory name and capability flags for `sc_db.bin` and a tiled manifest with schema version 1 whose referenced tile paths stay within `tiled/` and exist. A malformed tiled manifest means the map remains selectable in global mode, with a warning; it is not advertised as tiled-capable. A missing map root yields an empty catalog with a clear UI message. The API never accepts a client-supplied filesystem path.

The server resolves the chosen child against the map root and rejects names containing path separators, `.`/`..`, symlink escapes, missing files, and files that disappear before launch. Each start revalidates the saved choice. If the saved choice is no longer valid, the UI shows it as unavailable and localization/navigation Start is rejected until a valid map is selected. An empty or corrupt selection file is treated as no selection, with an error logged rather than silently using an unrelated map.

If there is no saved selection, the UI asks the user to choose a map before starting localization or navigation. It does not silently choose one of the currently inconsistent launch defaults. Mapping Start and Save Map are unaffected.

## Selection and persistence

The UI places a map selector with the localization/navigation controls. It shows the selected name, available maps, and two capability labels: tiled loading and automatic relocalization. A map without `sc_db.bin` remains selectable and displays “Manual initial pose required.” The list can be refreshed after a new map is saved.

`GET /api/maps` returns the catalog, current selection, and validity. `POST /api/maps/select` accepts only `{ "name": "<direct-child-name>" }`. Selection is rejected with HTTP 409 while either web-managed localization or navigation is running; the response asks the operator to stop both before retrying. The selected name is atomically written to `~/.config/web_control/selected_map.json` (create parent directory as needed). The web node reloads it on startup. The API never exposes arbitrary file reads or writes.

## Launch behavior

At web-managed localization Start, the web node revalidates the selected map and passes `map_path:=<selected tiled directory or GlobalMap.pcd>` to the existing `localization_with_nonground.launch.py`, which applies it as `localization.map_path`. It uses `tiled/` when the manifest passes the catalog checks; otherwise it uses `GlobalMap.pcd`. When `sc_db.bin` is absent it also passes `auto_relocalization:=false`, which the launch wires to `localization.auto_relocalization`. The web node preserves other configured launch arguments and never writes a generated parameter file into `~/.cache`.

At web-managed navigation Start, the node passes `map_dir:=<selected>/GlobalMap.pcd` and `ground_dir:=<selected>/GroundMap.pcd` to the existing navigation launch arguments. Both commands are built from the current saved selection immediately before start, so a selection change affects the next start without rebuilding the web node. The implementation must preserve unrelated configured launch arguments and avoid shell interpolation.

The web node does not rewrite `lio_sam_hesai` or navigation source config files. Direct launches still use their own configured defaults.

## Failure handling and verification

- Missing required PCDs, vanished selection, or unreadable map files produce a visible error and prevent the affected launch from starting with a different map. A malformed tiled manifest uses the valid global PCD path and is shown as a warning.
- Map names and capabilities are obtained from the server; the browser does not construct absolute robot paths.
- Tests cover catalog filtering and traversal/symlink rejection, persistence and reload, running-process conflict, command generation for tiled/global maps, missing `sc_db.bin`, stale selections, and HTTP responses. Verify the localization launch arguments (`map_path` and `auto_relocalization`) and the navigation argument. Run the web package's Python tests and JavaScript syntax check. The user will run `colcon build` themselves.
