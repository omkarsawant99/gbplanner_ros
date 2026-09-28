# autoassess_bridge

A small ROS 1 (Noetic) node that polls AutoAssess inspection plans from Cognite Data Fusion (CDF)
and publishes them for the autonomy stack. It only reads from CDF (`instances.list` /
`instances.retrieve`), never writes.

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

## Parameters

See [`config/autoassess_bridge.yaml`](config/autoassess_bridge.yaml): `area_external_id`
(required), `space`, `poll_period_s`, `frame_id`, `standoff_m`, `set_global_bound`,
`bound_margin_m`, `global_bound_service`, `service_timeout_s`.

## Credentials

Only from the environment of the process that starts the node (not from ROS parameters or files):
`COGNITE_PROJECT`, `COGNITE_CLUSTER`, `COGNITE_TENANT_ID`, `COGNITE_CLIENT_ID`,
`COGNITE_CLIENT_SECRET` (OAuth client credentials). A UUID tenant uses Azure AD
(`login.microsoftonline.com/<tenant>`, scope `https://<cluster>.cognitedata.com/.default`); any
other value uses the Cognite IdP (`auth.cognite.com`). The client only needs read access to the
AutoAssess data-model space.

## Build and run

```bash
pip3 install -r autoassess_bridge/requirements.txt   # cognite-sdk (not available via rosdep)
catkin build autoassess_bridge                       # or catkin_make; needs planner_msgs
export COGNITE_PROJECT=... COGNITE_CLUSTER=... COGNITE_TENANT_ID=... COGNITE_CLIENT_ID=... COGNITE_CLIENT_SECRET=...
roslaunch autoassess_bridge autoassess_bridge.launch area_external_id:=<area externalId>
```

## Tests

The logic lives in the ROS-free package `src/autoassess_bridge` (`cdf.py` CDF reads, `plan.py`
geometry, `poller.py` change detection, `bound.py` global-bound retries). Only
`scripts/autoassess_bridge_node` imports rospy. The tests use a fake CDF client:

```bash
cd autoassess_bridge && python3 -m pytest test
```
