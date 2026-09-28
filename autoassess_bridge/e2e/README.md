# autoassess_bridge full-loop e2e

`full_loop.sh` walks the whole loop against a **test area** of a real CDF project: Ready plan →
topics → findings → stub mission → upload → campaign + DefectDetection nodes. It writes to CDF
(only in `$TEST_AREA`), so use a test project/area and credentials you are allowed to write with.

## What you need

- Docker, and this repository checked out.
- A `.env` file with the five `COGNITE_*` variables (see `../.env.example`), with **write**
  access (`files:write`, `dataModelInstances:write` on the AutoAssess space).
- A folder containing `test_mesh.ply` (any PLY; it plays the role of gbplanner's mesh).
- The externalId of the test area, and an inspection plan in it you may flip Draft/Ready.

## Run

```bash
docker run --rm --env-file .env -e TEST_AREA=area-... \
  -v "$PWD":/src:ro -v /path/with/mesh:/data:ro \
  ros:noetic-ros-base bash /src/autoassess_bridge/e2e/full_loop.sh
```

Ordered checklist (the script asserts each step and exits non-zero on a miss):

1. catkin build + pytest inside the container.
2. Bridge starts for `$TEST_AREA` with uploads enabled.
3. **"Mark an inspection plan Ready in the AutoAssess UI now"** — do that in the web app
   (or export `MARK_READY_CMD` with a command that does it) within `READY_TIMEOUT_S` (600 s).
4. `/autoassess/plan_id`, `/autoassess/plan`, `/autoassess/inspection_targets` arrive
   (frame `world`).
5. Three findings are published on `/autoassess/findings`: one with a normal, one without,
   one bad (missing `x`) that must be logged and skipped.
6. The stub gbplanner (`stub_gbplanner.py`) serves `generate_mesh` and flies one fake mission:
   three paths, homing, then standing still.
7. The bridge detects the mission end, uploads the mesh, creates the campaign, stores one
   `DefectDetection` node per (merged) finding on it, and publishes one final `upload_status`
   with `state: complete` and `findings: {count, defectExternalIds}`. The script prints
   `CAMPAIGN:` and `DEFECTS:`.
8. The defects are read back from CDF: `status: New`, `source: ml`, attached to the campaign,
   one with the given normal and one without.

Afterwards, in the AutoAssess viewer: the campaign appears in the area (its 3D model once the
`dss worker` has built it — start it with `roslaunch autoassess_bridge autoassess_full.launch`),
and the findings show up in the Defects tab as `New` for review (Confirm → Suggestions → task).
Clean up by deleting/ignoring the test campaign and defects as your project prefers; the bridge
never deletes anything.
