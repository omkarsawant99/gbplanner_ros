#!/bin/bash
# SPDX-License-Identifier: BSD-3-Clause
#
# Full-loop e2e for autoassess_bridge, run inside ros:noetic-ros-base (see e2e/README.md):
#
#   docker run --rm --env-file <credentials.env> -e TEST_AREA=area-... \
#     -v <repo>:/src:ro -v <folder with test_mesh.ply>:/data:ro \
#     ros:noetic-ros-base bash /src/autoassess_bridge/e2e/full_loop.sh
#
# It builds the package (message generation included), runs pytest, starts the bridge for
# $TEST_AREA with uploads enabled, waits for a Ready plan (mark one Ready in the UI, or set
# MARK_READY_CMD), asserts the plan topics (autoassess_bridge/InspectionPlan + the JSON mirror),
# publishes three findings (a typed autoassess_bridge/Finding with a normal; JSON without a
# normal; JSON bad -> skipped), runs the stub gbplanner mission, and asserts
# /autoassess/upload_status (autoassess_bridge/UploadStatus) reaches `complete` with the
# findings stored as DefectDetection nodes on the new campaign (read back from CDF).
# Any missing step exits non-zero. CDF is only written in $TEST_AREA.
set -eo pipefail
: "${TEST_AREA:?set TEST_AREA to the e2e area externalId (the only area that will be written to)}"
dump_logs() {
  status=$?
  if [ "$status" -ne 0 ] && [ -f /tmp/bridge.log ]; then
    echo "--- bridge log (on failure)"; sed -E 's/\x1b\[[0-9;]*m//g' /tmp/bridge.log | tail -50
  fi
  exit "$status"
}
trap dump_logs EXIT
READY_TIMEOUT_S="${READY_TIMEOUT_S:-600}"
UPLOAD_TIMEOUT_S="${UPLOAD_TIMEOUT_S:-1200}"
step() { echo; echo "##### $*"; }

step setup
apt-get update -qq >/dev/null && apt-get install -y -qq python3-pip python3-pytest python3-numpy >/dev/null
pip3 install -q -r /src/autoassess_bridge/requirements.txt 2>&1 | grep -v "WARNING: Running pip as" || true
mkdir -p /ws/src && cp -r /src/planner_msgs /src/autoassess_bridge /ws/src/
find /ws/src -name __pycache__ -prune -exec rm -rf {} +
source /opt/ros/noetic/setup.bash
cd /ws && catkin_make -DCATKIN_WHITELIST_PACKAGES="planner_msgs;autoassess_bridge" 2>&1 | tail -1
source /ws/devel/setup.bash

step pytest
(cd /ws/src/autoassess_bridge && python3 -m pytest -q -p no:cacheprovider test 2>&1 | tail -1)

step start roscore + bridge for $TEST_AREA
roscore >/tmp/roscore.log 2>&1 &
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
PYTHONUNBUFFERED=1 rosrun autoassess_bridge autoassess_bridge_node _area_external_id:="$TEST_AREA" \
  _poll_period_s:=5 _set_global_bound:=false _upload_enabled:=true \
  _mesh_filename:=/tmp/gbplanner_mesh.ply _mission_end_quiet_s:=5 \
  odometry:=/odometry >/tmp/bridge.log 2>&1 &
BRIDGE=$!

step wait for a Ready plan in $TEST_AREA
echo ">>> Mark an inspection plan in $TEST_AREA Ready in the AutoAssess UI now (waiting ${READY_TIMEOUT_S}s)."
if [ -n "$MARK_READY_CMD" ]; then eval "$MARK_READY_CMD"; fi
if ! timeout "$READY_TIMEOUT_S" rostopic echo -n1 /autoassess/plan_id; then
  echo "FAIL: no Ready plan appeared in $TEST_AREA within ${READY_TIMEOUT_S}s"; exit 1
fi

step assert plan topics
python3 - <<'PY'
import json, rospy
from autoassess_bridge.msg import InspectionPlan, InspectionTask
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
rospy.init_node("e2e_assert_plan", anonymous=True)
plan = rospy.wait_for_message("/autoassess/plan", InspectionPlan, timeout=30)
assert plan.header.frame_id == "world", plan.header.frame_id
assert plan.external_id and plan.area_external_id, plan
for task in plan.tasks:
    assert task.task_type in (InspectionTask.REGION, InspectionTask.ELEMENT), task
    assert task.inspection_type in (InspectionTask.VISUAL, InspectionTask.NDT_THICKNESS), task
plan_json = json.loads(rospy.wait_for_message("/autoassess/plan_json", String, timeout=30).data)
assert plan_json["planExternalId"] == plan.external_id, plan_json
assert len(plan_json["tasks"]) == len(plan.tasks), plan_json
targets = rospy.wait_for_message("/autoassess/inspection_targets", PoseArray, timeout=30)
assert targets.header.frame_id == "world", targets.header.frame_id
print("plan:", plan.external_id, "area:", plan.area_external_id,
      "map:", plan.map_external_id, "tasks:", len(plan.tasks), "targets:", len(targets.poses))
PY

step "publish findings (typed with a normal; JSON without; JSON bad)"
rostopic pub -1 /autoassess/findings autoassess_bridge/Finding \
  "{id: 'e2e-norm', position: {x: 1.0, y: 2.0, z: 1.5}, normal: {x: 0.0, y: 0.0, z: 1.0},
    has_normal: true, defect_class: 'corrosion', confidence: 0.9}"
rostopic pub -1 /autoassess/findings_json std_msgs/String "data: '[
  {\"id\": \"e2e-nonorm\", \"x\": 4.0, \"y\": 5.0, \"z\": 1.0},
  {\"id\": \"e2e-bad\", \"y\": 1.0, \"z\": 1.0}
]'"
sleep 5
grep -q "Bad finding skipped" /tmp/bridge.log || { echo "FAIL: bad finding was not reported"; exit 1; }
grep -q "2 in total" /tmp/bridge.log || { echo "FAIL: 2 good findings were not buffered"; exit 1; }

step stub gbplanner mission
python3 /ws/src/autoassess_bridge/e2e/stub_gbplanner.py >/tmp/stub.log 2>&1 &
STUB=$!

step wait for upload complete with defects on the campaign
python3 - "$UPLOAD_TIMEOUT_S" <<'PY'
import json, sys, rospy
from autoassess_bridge.msg import UploadStatus
from std_msgs.msg import String
rospy.init_node("e2e_assert_upload", anonymous=True)
deadline = rospy.get_time() + float(sys.argv[1])
final = None
def on_status(msg):
    global final
    print("upload_status:", msg.state, msg.mission_id, msg.campaign_external_id,
          "files {}/{}".format(msg.files_done, msg.files_total),
          "findings", msg.findings_count, msg.message, flush=True)
    if msg.state in (UploadStatus.COMPLETE, UploadStatus.FAILED):
        final = msg
rospy.Subscriber("/autoassess/upload_status", UploadStatus, on_status)
while not rospy.is_shutdown() and final is None and rospy.get_time() < deadline:
    rospy.sleep(0.5)
assert final is not None, "no terminal upload_status"
assert final.state == UploadStatus.COMPLETE, final
assert final.files_done == final.files_total and final.files_done >= 1, final
assert final.findings_count == 2, final
ids = list(final.defect_external_ids)
assert len(ids) == 2 and all(i.startswith("defect-") for i in ids), final
mirror = json.loads(rospy.wait_for_message("/autoassess/upload_status_json", String, timeout=10).data)
assert mirror["state"] == "complete", mirror
assert mirror["campaignExternalId"] == final.campaign_external_id, mirror
assert sorted(mirror["findings"]["defectExternalIds"]) == sorted(ids), mirror
print("CAMPAIGN:", final.campaign_external_id)
print("DEFECTS:", " ".join(ids))
with open("/tmp/e2e_result.env", "w") as fh:
    fh.write("CAMPAIGN={}\nDEFECTS={}\n".format(final.campaign_external_id, ",".join(ids)))
PY

step read the defects back from CDF
source /tmp/e2e_result.env
python3 - "$CAMPAIGN" "$DEFECTS" <<'PY'
import os, sys
sys.path.insert(0, "/ws/src/autoassess_bridge/src")
from autoassess_bridge import cdf
from cognite.client.data_classes.data_modeling import ViewId
campaign, ids = sys.argv[1], sys.argv[2].split(",")
client = cdf.make_client(os.environ)
view = ViewId("autoassess", "DefectDetectionView", "1")
nodes = client.data_modeling.instances.retrieve(
    nodes=[("autoassess", i) for i in ids], sources=[view]
).nodes
assert len(nodes) == len(ids), "expected {} defects, got {}".format(len(ids), len(nodes))
saw_normal = saw_no_normal = False
for node in nodes:
    props = dict(node.properties[view])
    assert props["status"] == "New", props
    assert props["source"] == "ml", props
    assert props["campaign"]["externalId"] == campaign, props
    assert len(props["boundingBox3d"]) == 9, props
    saw_normal = saw_normal or "normal3d" in props
    saw_no_normal = saw_no_normal or "normal3d" not in props
    print("defect:", node.external_id, props["defectClass"], props["probability"],
          props["boundingBox3d"][:3], props.get("normal3d"))
assert saw_normal and saw_no_normal, "expected one defect with and one without a normal"
print("DEFECTS-IN-CDF-OK")
PY

kill -INT $BRIDGE $STUB 2>/dev/null || true; wait $BRIDGE $STUB 2>/dev/null || true
echo "--- bridge log"; sed -E 's/\x1b\[[0-9;]*m//g' /tmp/bridge.log
kill %1 2>/dev/null || true
echo; echo "##### FULL LOOP OK"
