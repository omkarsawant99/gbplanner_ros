# autoassess_bridge

A small ROS 1 (Noetic) node that connects gbplanner to AutoAssess, which stores its data in
Cognite Data Fusion (CDF). It does three things:

1. It follows the newest **Ready** inspection plan in the CDF project and publishes it: the plan JSON,
   inspection target poses, and the plan id. It can also set gbplanner's global bound.
2. It publishes the plan's **reference map** as a latched point cloud on
   `/ballast_tank/pointcloud`. You no longer need to download the map and run `pcd_to_pointcloud`.
3. Optionally, it **uploads the finished mission** (mesh and point clouds) as a new AutoAssess
   campaign, and stores findings reported on `/autoassess/findings` as **defects** on that
   campaign for review.

By default the node only reads from CDF (`instances.list` / `instances.retrieve`, file
downloads). With `~upload_enabled: true` it also writes the mission upload
([Mission upload](#mission-upload)); nothing else is ever written.

## Typical setup

```bash
# 1. Build (cognite-sdk is not in rosdep)
pip3 install -r autoassess_bridge/requirements.txt
catkin build autoassess_bridge            # or catkin_make; needs planner_msgs

# 2. Credentials: environment only; keep them in a file you never commit
export COGNITE_PROJECT=... COGNITE_CLUSTER=... COGNITE_TENANT_ID=... COGNITE_CLIENT_ID=... COGNITE_CLIENT_SECRET=...

# 3. Start the bridge next to gbplanner
roslaunch autoassess_bridge autoassess_bridge.launch
# with automatic mission upload (below), also pass your odometry:
#   roslaunch autoassess_bridge autoassess_bridge.launch odometry_topic:=<your nav_msgs/Odometry topic>
```

- **Which plan.** No area id is needed. The bridge follows the newest Ready plan of the whole
  project, and that plan's area drives the global bound, the reference map and the upload area.
  Marking any plan Ready in AutoAssess switches the robot to it. To stay on one vessel or one
  area, pass `vessel_external_id:=vessel-…` and/or `area_external_id:=area-…` to the launch file
  (or set them in the yaml).

- **gbplanner's reference map.** Keep `/ballast_tank/pointcloud` as the map topic in the
  gbplanner launch file. The bridge publishes the plan's map there, latched, in frame `world`,
  just as `rosrun pcl_ros pcd_to_pointcloud <file>.pcd 1.0 _frame_id:=world _latch:=true
  cloud_pcd:=/ballast_tank/pointcloud` did. To keep publishing the map yourself, set
  `publish_map: false`.
- **Automatic upload after the mission (optional).**
  - On **gbplanner_node**, set `<param name="mesh_filename" value="/tmp/gbplanner_mesh.ply"/>`.
  - In `config/autoassess_bridge.yaml` (or your own copy, passed with `config:=`), set:
    ```yaml
    upload_enabled: true
    mesh_filename: /tmp/gbplanner_mesh.ply   # the same path as on gbplanner_node
    mission_dir: ""                          # optional: folder with extra .ply/.pcd files
    ```
  - When the robot is home and still, the bridge uploads the mesh and creates a campaign. Watch
    it on `rostopic echo /autoassess/upload_status`, or trigger it by hand with
    `rosservice call /autoassess_bridge/upload_mission`.

Check that it works:

```bash
rostopic echo -n1 /autoassess/plan_id                   # which plan
rostopic echo -n1 /autoassess/inspection_targets        # one pose per task
rostopic echo -n1 /ballast_tank/pointcloud --noarr      # the reference map (frame world)
```

## Plan topics

Every `~poll_period_s` the node looks up the newest non-deleted plan with status `Ready`
(newest by last update, then creation time). It searches the whole project, or only
`~vessel_external_id`'s areas and/or `~area_external_id` when set. The plan's area is taken from
the plan (`areaExternalId` / `areaName` in the plan JSON). Each switch is logged as
"Following plan <id> '<name>' in <vessel>/<area>". When that plan's content changes (a different
plan, possibly in another area, or edited tasks), it publishes:

| Topic | Type | Content |
| --- | --- | --- |
| `/autoassess/plan` | `std_msgs/String` | the plan JSON, the same text `dss plan download` writes to `plan.json` |
| `/autoassess/inspection_targets` | `geometry_msgs/PoseArray` | one inspection pose per task, in task order, in `~frame_id` |
| `/autoassess/plan_id` | `std_msgs/String` | the plan's externalId |

All three are latched, so late subscribers get the current plan. Tasks without a target point are
left out of the PoseArray (and logged); the full list stays in `/autoassess/plan`. If there is no
Ready plan, the node logs "No Ready plan (filter: …)" once and waits. When the newest Ready
plan moves to another area, the plan, targets and map are republished and the bound is resent.

**Inspection poses.**
- Region tasks: `standoff_m` out from `position3d`, along `normalVector`.
- Element tasks (no normal): `standoff_m` above the element centre, looking straight down.
- The orientation points body +x at the target, with zero roll (REP-103).

**Global bound (optional).**
- With `~set_global_bound: true`, the node sends the box around the area's structural-element
  centres, padded by `~bound_margin_m`, once per new plan. It goes to `gbplanner/set_global_bound`
  (`planner_msgs/planner_set_global_bound`, `use_z_val: true`).
- If the service is not up yet, the node tries again on every poll.
- If the planner rejects the box, the node logs a warning and does not resend it. gbplanner
  rejects it when the robot's current position is outside the box.

## Reference map

With `~publish_map: true` (default), the node publishes the plan's reference map. The map is the
campaign named by the plan's `mapExternalId`, the same one `dss plan download-map` downloads. It
goes out as a latched `sensor_msgs/PointCloud2` on `~map_topic` (default
`/ballast_tank/pointcloud`), in `~map_frame_id` (default: `~frame_id`, i.e. `world`).

- **Which file.** The node takes the campaign's point cloud whose label matches
  `~map_file_label`, or the first point cloud (logged). If the campaign has no point cloud, it
  uses the vertices of its first PLY mesh, with xyz and packed rgb.
- **PCD contents.** All fields of the PCD are published unchanged, as `pcd_to_pointcloud` does.
  ascii and binary PCDs are supported; for `binary_compressed`, re-save the file as ascii or
  binary.
- **Cache.** Files are downloaded once into `~map_cache_dir` (default
  `~/.ros/autoassess_maps/<file id>/`). A restart publishes from the cache.
- **When it is published.** Loading runs in the background, after the plan topics are
  published. The map is published once per map; it is republished only when the plan's
  `mapExternalId` changes. The log says which file and how many points. If a load fails, it is
  logged and retried after `max(poll_period_s, 30)` s.
- **Plans without a map.** A plan without a `mapExternalId` is logged once, and the last map
  stays published.

## Findings from the robot

The detection stack reports findings by publishing JSON on `/autoassess/findings`
(`std_msgs/String`) — one object, or an array of objects:

```json
{
  "x": 1.0, "y": 2.0, "z": 3.0,
  "id": "corr-001",
  "nx": 0.0, "ny": 0.0, "nz": 1.0,
  "radius": 0.3,
  "inspection_type": "visual",
  "class": "corrosion",
  "confidence": 0.9,
  "description": "pitting near the weld"
}
```

| field | meaning |
| --- | --- |
| `x, y, z` (required) | position in metres, in the reference map's frame (`world`) |
| `id` | stable finding id; default: a hash of the values. Repeats are deduped |
| `nx, ny, nz` | surface normal (all three or none) |
| `radius` | task radius in metres (default 0.3) |
| `inspection_type` | `visual` (default) or `ndt_thickness` |
| `class`, `confidence`, `description` | optional metadata (`confidence` 0..1) |

Bad entries are logged (once per distinct error) and skipped; good ones are buffered for the
current mission. At mission end — after the upload, in the same step — each buffered finding
becomes a **DefectDetection** node on the just-created campaign (findings closer than
`~findings_merge_radius_m`, 0.5 m, merge into one defect):

- `boundingBox3d`: the finding position with zero extents/rotation (like manually placed
  defects), `normal3d` only when the finding gave one;
- `defectClass`: the finding's `class` (merged classes joined with `+`), else `finding`;
- `probability`: the finding's `confidence`, else 0.5 (the field is required);
- `status: New`, `source: ml`, external id `defect-<missionId>-<hash>` (deterministic, so
  retries never duplicate).

The final `/autoassess/upload_status` then has
`findings: {"count": …, "defectExternalIds": […]}`. The review path is the viewer's
**Defects tab**: Confirm a defect → it appears under Suggestions → add it to a plan as a
task. No plan is created automatically.

- The topic is always subscribed. With `~upload_enabled: false` the node warns that no
  defects will be stored.
- If storing the defects fails, the findings are kept; `~submit_findings`
  (`std_srvs/Trigger`) retries just the defects. Re-detections in a *later* mission
  intentionally become that mission's defects.

## Mission upload

Off unless `~upload_enabled: true`. An upload sends the mission's output to CDF, where the
AutoAssess viewer shows it as a new campaign. The campaign and file tags use the area of the plan
the bridge was following when the mission started. If the bridge has not followed any plan yet,
the upload fails with a clear message; it never creates areas. The node does not build the 3D model.
That is done afterwards by the AutoAssess `dss worker`, which is being added to the AutoAssess
SDK (upcoming, not released yet).

1. If `~mesh_filename` is set, the node calls `~generate_mesh_service`
   (`/gbplanner_node/generate_mesh`, `std_srvs/Empty`). It then waits up to `~mesh_timeout_s` for
   that file to be rewritten. voxblox inside gbplanner_node writes the PLY only when
   **gbplanner_node's own `mesh_filename` parameter** is set to the same path. That parameter is
   read at start-up.
2. It collects the mesh plus every `.ply` / `.pcd` under `~mission_dir`. Other files are listed
   as skipped.
3. It creates a campaign: an `InspectionResult` node `result-<uuid>` with status `InProgress`,
   today's date and `createdBy: autoassess_bridge`.
4. It uploads each file as a CogniteFile with these tags:
   - `autoassess`;
   - `ply_mesh`, or `pcd_pointcloud` plus `label:<name>`;
   - `area:<id>`, `plan:<id>` and `mission:<id>`.
5. It writes the file ids into the campaign and sets it `Complete`.

These are the same conventions as `dss campaign upload`. If a file fails, the campaign stays
`InProgress`. Calling `~upload_mission` again resumes the same mission: same campaign, and files
already uploaded are skipped. Nothing that already exists in CDF is changed or deleted.

Triggers:
- `~upload_mission` (`std_srvs/Trigger`): upload now. The response names the campaign.
- `~upload_on_mission_end: true` (default): automatically, once per mission. gbplanner has no
  "mission finished" signal, so the node guesses:
  - a mission starts with the first `/gbplanner_path`;
  - it has finished when `/gbplanner_is_homing` is true, no new path has come for
    `~mission_end_quiet_s`, and the robot's speed on `odometry` is at most
    `~mission_end_max_speed`.
  - `odometry` is a `nav_msgs/Odometry` topic: remap it, or pass `odometry_topic:=` to the
    launch file.

Progress is published, latched, as JSON on `/autoassess/upload_status` (`std_msgs/String`):
`state` (`idle`, `exporting_mesh`, `uploading`, `complete`, `failed`), `missionId`,
`areaExternalId`, `planExternalId`, `campaignExternalId`, `cdfFileIds`, `pcdFileIds`,
`pcdFileLabels`, `failedFiles`, `skippedFiles`, `message`, `updatedAt`.

## Parameters

All parameters are private (`~`), with defaults in
[`config/autoassess_bridge.yaml`](config/autoassess_bridge.yaml):

| Group | Parameters |
| --- | --- |
| Plan | `vessel_external_id`, `area_external_id` (optional filters), `space`, `poll_period_s`, `frame_id`, `standoff_m` |
| Reference map | `publish_map`, `map_topic`, `map_frame_id`, `map_file_label`, `map_cache_dir` |
| Findings | `findings_merge_radius_m` |
| Global bound | `set_global_bound`, `bound_margin_m`, `global_bound_service`, `service_timeout_s` |
| Mission upload | `upload_enabled`, `upload_on_mission_end`, `mesh_filename`, `generate_mesh_service`, `mesh_timeout_s`, `mission_dir`, `mission_end_quiet_s`, `mission_end_max_speed`, `path_topic`, `homing_topic` |

## Running the worker with the bridge (full loop)

The 3D models the viewer streams are built by the AutoAssess SDK's `dss worker` (upcoming,
not released yet). `launch/autoassess_full.launch` runs the bridge **and** that worker under
roslaunch, so one launch gives the whole loop:

```bash
roslaunch autoassess_bridge autoassess_full.launch     area_external_id:=area-XXXX uidss_dir:=/path/to/autoassess-sdk
```

- `area_external_id` is **mandatory** here: the worker only builds models for that area's
  meshes (`scripts/worker_node` refuses a command without `--area`, because an unfiltered
  worker would build for the whole CDF project).
- The worker is run as `uv run --project <uidss_dir> dss worker --area … --poll 30`, wrapped
  in the thin `worker_node` exec script with roslaunch `respawn="true" respawn_delay="30"`
  (`dss worker` loops by itself; respawn only covers exits).

**Install once** (ground station):

```bash
git clone <the AutoAssess SDK repository> autoassess-sdk
cd autoassess-sdk && uv sync   # first run downloads a Python interpreter — takes minutes
uv run dss --help              # warm-up / sanity check
```

Under systemd, services have a minimal `PATH`: pass the absolute uv path with
`uv:=/home/robot/.local/bin/uv` (or a full `worker_command:=…`). A unit example:

```ini
[Unit]
Description=AutoAssess bridge + worker
After=network-online.target

[Service]
User=robot
EnvironmentFile=/home/robot/autoassess/.env
ExecStart=/opt/ros/noetic/env.sh roslaunch autoassess_bridge autoassess_full.launch     area_external_id:=area-XXXX uidss_dir:=/home/robot/autoassess-sdk uv:=/home/robot/.local/bin/uv
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Alternatively run the worker in Docker next to ROS: `docker run --env-file .env <sdk image>
dss worker --area area-XXXX --poll 30` — the worker only needs CDF, not ROS.

## Credentials

Credentials are read only from the environment of the process that starts the node, never from
ROS parameters or files: `COGNITE_PROJECT`, `COGNITE_CLUSTER`, `COGNITE_TENANT_ID`,
`COGNITE_CLIENT_ID`, `COGNITE_CLIENT_SECRET` (OAuth client credentials).

- A UUID tenant uses Azure AD (`login.microsoftonline.com/<tenant>`, scope
  `https://<cluster>.cognitedata.com/.default`). Any other value uses the Cognite IdP
  (`auth.cognite.com`).
- The client needs read access to the AutoAssess data-model space and to files (for the map).
- Uploads also need `files:write` and `dataModelInstances:write` on the AutoAssess space.

**Using a `.env` file:** copy [`.env.example`](.env.example) to `autoassess_bridge/.env` (git-ignored) and fill in the values, then load it in the shell that launches the node:

```bash
cp autoassess_bridge/.env.example autoassess_bridge/.env   # once; edit the values
set -a; source autoassess_bridge/.env; set +a
roslaunch autoassess_bridge autoassess_bridge.launch
```

With Docker, pass it with `docker run --env-file autoassess_bridge/.env …`. Never commit `.env`.

A missing variable stops the node with a clear message.

## Tests

The logic lives in the ROS-free package `src/autoassess_bridge`. Only
`scripts/autoassess_bridge_node` imports rospy.

| Module | What it does |
| --- | --- |
| `cdf.py` | CDF reads |
| `plan.py` | pose geometry |
| `poller.py` | change detection |
| `bound.py` | global-bound retries |
| `map.py` | reference map |
| `findings.py` | findings parsing, buffer, merging, task planning |
| `upload.py` | mission upload |
| `mission_end.py` | mission-end detection |

The tests use a fake CDF client:

```bash
cd autoassess_bridge && python3 -m pytest test
```
