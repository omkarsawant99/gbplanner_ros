# autoassess_bridge

A small ROS 1 (Noetic) node that polls AutoAssess inspection plans from Cognite Data Fusion (CDF)
and publishes them for the autonomy stack. By default it only reads from CDF (`instances.list` /
`instances.retrieve`). With `~upload_enabled: true` it also uploads the finished mission (see
[Mission upload](#mission-upload)); that is the only time it writes.

Every `~poll_period_s` it looks up the newest non-deleted plan with status `Ready` for the
configured area. When that plan's content changes (another plan, or edited tasks), it publishes:

| Topic | Type | Content |
| --- | --- | --- |
| `/autoassess/plan` | `std_msgs/String` | plan JSON, the same text `dss plan download` writes to `plan.json` |
| `/autoassess/inspection_targets` | `geometry_msgs/PoseArray` | one inspection pose per task, in task order, in `~frame_id` |
| `/autoassess/plan_id` | `std_msgs/String` | the plan's externalId |

All three are latched, so late subscribers get the current plan. Tasks without a target point are
skipped in the PoseArray (and logged); the full list stays in `/autoassess/plan`.

**Inspection poses.** Region tasks: `standoff_m` out from `position3d` along `normalVector`. Element
tasks (no normal): `standoff_m` above the element centre, looking straight down. The orientation
points body +x at the target with zero roll (REP-103).

**Global bound (optional).** With `~set_global_bound: true`, the box around the area's structural
element centres, padded by `~bound_margin_m`, is sent once per new plan to
`gbplanner/set_global_bound` (`planner_msgs/planner_set_global_bound`, `use_z_val: true`). If the
service is not up yet, the node tries again every tick. If the planner rejects the box (gbplanner
does this when the robot's current position is outside it), the node logs a warning and does not
resend it.

## Mission upload

Off unless `~upload_enabled: true`. An upload sends the mission's output to CDF, where the
AutoAssess viewer shows it as a campaign of the area (the 3D model is built afterwards by the
AutoAssess `dss worker`, not by this node):

1. If `~mesh_filename` is set, the node calls `~generate_mesh_service`
   (`/gbplanner_node/generate_mesh`, `std_srvs/Empty`) and waits up to `~mesh_timeout_s` for that
   file to be rewritten. voxblox inside gbplanner_node writes the PLY only when **gbplanner_node's
   own `mesh_filename` parameter** is set to the same path (it is read at start-up), e.g.
   `<param name="mesh_filename" value="/tmp/gbplanner_mesh.ply" />` in the gbplanner_node launch.
2. It collects the mesh plus every `.ply` / `.pcd` under `~mission_dir` (other files are listed
   as skipped).
3. It creates a campaign (`InspectionResult` node `result-<uuid>`, status `InProgress`, today's
   date, `createdBy: autoassess_bridge`), uploads each file as a CogniteFile tagged `autoassess`,
   `ply_mesh` or `pcd_pointcloud` (+ `label:<name>`), `area:<id>`, `plan:<id>` and
   `mission:<id>`, writes the file ids into the campaign and sets it `Complete`. These are the
   same conventions as `dss campaign upload`. If a file fails, the campaign stays `InProgress`;
   calling `~upload_mission` again resumes the same mission (same campaign, uploaded files are
   skipped). Nothing existing in CDF is changed or deleted.

Triggers:
- `~upload_mission` (`std_srvs/Trigger`): upload now. The response says which campaign.
- With `~upload_on_mission_end: true` (default), automatically once per mission. gbplanner has
  no "mission finished" signal, so the node guesses: a mission starts with the first
  `/gbplanner_path`; it has finished when `/gbplanner_is_homing` is true, no new path has come
  for `~mission_end_quiet_s` and the robot's speed on `odometry` (`nav_msgs/Odometry`, remap it,
  or pass `odometry_topic:=` to the launch file) is at most `~mission_end_max_speed`.

Progress is published, latched, as JSON on `/autoassess/upload_status` (`std_msgs/String`):
`state` (`idle`, `exporting_mesh`, `uploading`, `complete`, `failed`), `missionId`,
`areaExternalId`, `planExternalId`, `campaignExternalId`, `cdfFileIds`, `pcdFileIds`,
`pcdFileLabels`, `failedFiles`, `skippedFiles`, `message`, `updatedAt`.

The CDF client then needs write access in addition to read: `files:write` and
`dataModelInstances:write` on the AutoAssess space.

## Parameters

See [`config/autoassess_bridge.yaml`](config/autoassess_bridge.yaml): `area_external_id`
(required), `space`, `poll_period_s`, `frame_id`, `standoff_m`, `set_global_bound`,
`bound_margin_m`, `global_bound_service`, `service_timeout_s`, and for uploads `upload_enabled`,
`upload_on_mission_end`, `mesh_filename`, `generate_mesh_service`, `mesh_timeout_s`,
`mission_dir`, `mission_end_quiet_s`, `mission_end_max_speed`, `path_topic`, `homing_topic`.

## Credentials

Only from the environment of the process that starts the node (not from ROS parameters or files):
`COGNITE_PROJECT`, `COGNITE_CLUSTER`, `COGNITE_TENANT_ID`, `COGNITE_CLIENT_ID`,
`COGNITE_CLIENT_SECRET` (OAuth client credentials). A UUID tenant uses Azure AD
(`login.microsoftonline.com/<tenant>`, scope `https://<cluster>.cognitedata.com/.default`); any
other value uses the Cognite IdP (`auth.cognite.com`). The client only needs read access to the
AutoAssess data-model space (plus write access for uploads, see above).

## Build and run

```bash
pip3 install -r autoassess_bridge/requirements.txt   # cognite-sdk (not available via rosdep)
catkin build autoassess_bridge                       # or catkin_make; needs planner_msgs
export COGNITE_PROJECT=... COGNITE_CLUSTER=... COGNITE_TENANT_ID=... COGNITE_CLIENT_ID=... COGNITE_CLIENT_SECRET=...
roslaunch autoassess_bridge autoassess_bridge.launch area_external_id:=<area externalId>
```

## Tests

The logic lives in the ROS-free package `src/autoassess_bridge` (`cdf.py` CDF reads, `plan.py`
geometry, `poller.py` change detection, `bound.py` global-bound retries, `upload.py` mission
upload, `mission_end.py` mission-end detection). Only `scripts/autoassess_bridge_node` imports
rospy. The tests use a fake CDF client:

```bash
cd autoassess_bridge && python3 -m pytest test
```
