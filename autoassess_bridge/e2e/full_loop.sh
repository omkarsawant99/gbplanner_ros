#!/bin/bash
# SPDX-License-Identifier: BSD-3-Clause
#
# Full-loop e2e for autoassess_bridge, run inside ros:noetic-ros-base (see e2e/README.md):
#
#   docker run --rm --env-file <credentials.env> -e TEST_AREA=area-... \
#     -v <repo>:/src:ro -v <folder with test_mesh.ply>:/data:ro \
#     ros:noetic-ros-base bash /src/autoassess_bridge/e2e/full_loop.sh
#
# It builds the package, runs pytest, starts the bridge for $TEST_AREA with uploads enabled,
# waits for a Ready plan (mark one Ready in the UI, or set MARK_READY_CMD), asserts the plan
# topics, publishes three findings (one with a normal, one without, one bad -> skipped),
# runs the stub gbplanner mission, and asserts /autoassess/upload_status reaches `complete`
# with a findings plan id. Any missing step exits non-zero. CDF is only written in $TEST_AREA.
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
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
rospy.init_node("e2e_assert_plan", anonymous=True)
plan = json.loads(rospy.wait_for_message("/autoassess/plan", String, timeout=30).data)
targets = rospy.wait_for_message("/autoassess/inspection_targets", PoseArray, timeout=30)
assert targets.header.frame_id == "world", targets.header.frame_id
print("plan:", plan["planExternalId"], "area:", plan["areaExternalId"],
      "map:", plan["mapExternalId"], "tasks:", len(plan["tasks"]), "targets:", len(targets.poses))
PY

step "publish findings (one with a normal, one without, one bad)"
rostopic pub -1 /autoassess/findings std_msgs/String "data: '[
  {\"id\": \"e2e-norm\", \"x\": 1.0, \"y\": 2.0, \"z\": 1.5, \"nx\": 0, \"ny\": 0, \"nz\": 1, \"class\": \"corrosion\", \"confidence\": 0.9},
  {\"id\": \"e2e-nonorm\", \"x\": 4.0, \"y\": 5.0, \"z\": 1.0},
  {\"id\": \"e2e-bad\", \"y\": 1.0, \"z\": 1.0}
]'"
sleep 5
grep -q "Bad finding skipped" /tmp/bridge.log || { echo "FAIL: bad finding was not reported"; exit 1; }
grep -q "Buffered 2 finding" /tmp/bridge.log || { echo "FAIL: 2 good findings were not buffered"; exit 1; }

step stub gbplanner mission
python3 /ws/src/autoassess_bridge/e2e/stub_gbplanner.py >/tmp/stub.log 2>&1 &
STUB=$!

step wait for upload complete with a findings plan
python3 - "$UPLOAD_TIMEOUT_S" <<'PY'
import json, sys, rospy
from std_msgs.msg import String
rospy.init_node("e2e_assert_upload", anonymous=True)
deadline = rospy.get_time() + float(sys.argv[1])
final = None
def on_status(msg):
    global final
    status = json.loads(msg.data)
    print("upload_status:", json.dumps(status), flush=True)
    if status["state"] in ("complete", "failed"):
        final = status
rospy.Subscriber("/autoassess/upload_status", String, on_status)
while not rospy.is_shutdown() and final is None and rospy.get_time() < deadline:
    rospy.sleep(0.5)
assert final is not None, "no terminal upload_status"
assert final["state"] == "complete", final
findings = final["findings"]
assert findings and findings.get("planExternalId", "").startswith("plan-"), final
assert findings["count"] >= 1, final
print("CAMPAIGN:", final["campaignExternalId"])
print("FINDINGS_PLAN:", findings["planExternalId"])
PY

kill -INT $BRIDGE $STUB 2>/dev/null || true; wait $BRIDGE $STUB 2>/dev/null || true
echo "--- bridge log"; sed -E 's/\x1b\[[0-9;]*m//g' /tmp/bridge.log
kill %1 2>/dev/null || true
echo; echo "##### FULL LOOP OK"
