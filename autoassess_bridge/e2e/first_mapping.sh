#!/bin/bash
# SPDX-License-Identifier: BSD-3-Clause
#
# First-mapping e2e: an area with NO Ready plan gets explored; the mission uploads
# CAMPAIGN-LESS (files only, grouped later in the viewer). Run inside ros:noetic-ros-base:
#
#   docker run --rm --env-file <credentials.env> -e TEST_AREA=area-... \
#     -v <repo>:/src:ro -v <folder with test_mesh.ply>:/data:ro \
#     -v <folder with extra .pcd files>:/mission:ro \
#     ros:noetic-ros-base bash /src/autoassess_bridge/e2e/first_mapping.sh
#
# Asserts: the bridge logs "No Ready plan"; a finding is buffered; the stub mission ends; the
# final upload_status is complete with campaignExternalId null and the first-mapping note; the
# uploaded CogniteFiles are found in CDF by their mission:<id> tag with the uidss tags and no
# campaign lists their ids; the finding became a DefectDetection attached to the AREA.
# CDF is only written in $TEST_AREA's scope. Any missing step exits non-zero.
set -eo pipefail
: "${TEST_AREA:?set TEST_AREA to the e2e area externalId (the only area that will be written to)}"
UPLOAD_TIMEOUT_S="${UPLOAD_TIMEOUT_S:-1200}"
step() { echo; echo "##### $*"; }
dump_logs() {
  status=$?
  if [ "$status" -ne 0 ] && [ -f /tmp/bridge.log ]; then
    echo "--- bridge log (on failure)"; sed -E 's/\x1b\[[0-9;]*m//g' /tmp/bridge.log | tail -50
  fi
  exit "$status"
}
trap dump_logs EXIT

step setup
apt-get update -qq >/dev/null && apt-get install -y -qq python3-pip python3-pytest python3-numpy >/dev/null
pip3 install -q -r /src/autoassess_bridge/requirements.txt 2>&1 | grep -v "WARNING: Running pip as" || true
mkdir -p /ws/src && cp -r /src/planner_msgs /src/autoassess_bridge /ws/src/
find /ws/src -name __pycache__ -prune -exec rm -rf {} +
source /opt/ros/noetic/setup.bash
cd /ws && catkin_make -DCATKIN_WHITELIST_PACKAGES="planner_msgs;autoassess_bridge" 2>&1 | tail -1
source /ws/devel/setup.bash

step start roscore + bridge for $TEST_AREA with uploads enabled
roscore >/tmp/roscore.log 2>&1 &
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
PYTHONUNBUFFERED=1 rosrun autoassess_bridge autoassess_bridge_node _area_external_id:="$TEST_AREA" \
  _poll_period_s:=5 _set_global_bound:=false _publish_map:=false _upload_enabled:=true \
  _mesh_filename:=/tmp/gbplanner_mesh.ply _mission_dir:=/mission _mission_end_quiet_s:=5 \
  odometry:=/odometry >/tmp/bridge.log 2>&1 &
BRIDGE=$!

step precondition: no Ready plan in $TEST_AREA
sleep 8
if ! grep -q "No Ready plan" /tmp/bridge.log; then
  echo "FAIL: the area has a Ready plan (or the bridge did not start); this scenario needs none"
  exit 1
fi

step publish one finding
rostopic pub -1 /autoassess/findings std_msgs/String "data: '{\"id\": \"fm-1\", \"x\": 2.0, \"y\": 3.0, \"z\": 1.0, \"nx\": 0, \"ny\": 0, \"nz\": 1, \"class\": \"anomaly\"}'"
sleep 3
grep -q "Buffered 1 finding" /tmp/bridge.log || { echo "FAIL: the finding was not buffered"; exit 1; }

step stub gbplanner mission
python3 /ws/src/autoassess_bridge/e2e/stub_gbplanner.py >/tmp/stub.log 2>&1 &
STUB=$!

step wait for the campaign-less upload
python3 - "$UPLOAD_TIMEOUT_S" <<'PY'
import json, sys, rospy
from std_msgs.msg import String
rospy.init_node("e2e_first_mapping", anonymous=True)
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
assert final["campaignExternalId"] is None, final
assert final["note"] == "no plan followed: uploaded without a campaign (first mapping)", final
assert final["planExternalId"] is None, final
assert len(final["cdfFileIds"]) == 1 and len(final["pcdFileIds"]) == 2, final
assert sorted(final["pcdFileLabels"]) == ["Labeled Cloud", "Pointcloud"], final
assert final["findings"]["count"] == 1, final
print("MISSION:", final["missionId"])
print("FILE_IDS:", " ".join(str(i) for i in final["cdfFileIds"] + final["pcdFileIds"]))
print("DEFECTS:", " ".join(final["findings"]["defectExternalIds"]))
with open("/tmp/e2e_result.env", "w") as fh:
    fh.write("MISSION={}\nDEFECT={}\n".format(final["missionId"], final["findings"]["defectExternalIds"][0]))
PY

step read the files and the defect back from CDF
source /tmp/e2e_result.env
python3 - "$TEST_AREA" "$MISSION" "$DEFECT" <<'PY'
import os, sys
sys.path.insert(0, "/ws/src/autoassess_bridge/src")
from autoassess_bridge import cdf
from cognite.client.data_classes.data_modeling import NodeId, ViewId
area, mission, defect = sys.argv[1], sys.argv[2], sys.argv[3]
client = cdf.make_client(os.environ)
file_view = ViewId("cdf_cdm", "CogniteFile", "v1")
files = client.data_modeling.instances.list(
    instance_type="node", sources=[file_view], space="autoassess", limit=1000,
    filter={"containsAny": {"property": ["cdf_cdm", "CogniteFile/v1", "tags"],
                            "values": ["mission:" + mission]}},
)
rows = [(n.external_id, dict(n.properties[file_view])) for n in files]
assert len(rows) == 3, rows
names = sorted(props["name"] for _, props in rows)
assert names == ["gbplanner_mesh.ply", "labeled_cloud.pcd", "pointcloud.pcd"], names
file_ids = []
for external_id, props in rows:
    tags = props["tags"]
    assert "autoassess" in tags and ("area:" + area) in tags, tags
    assert not any(t.startswith("plan:") for t in tags), tags
    metadata = client.files.retrieve(instance_id=NodeId("autoassess", external_id))
    assert metadata is not None and metadata.uploaded, external_id
    file_ids.append(int(metadata.id))
    print("file:", metadata.id, external_id, props["name"], tags)

# no campaign in the project lists these files
result_view = ViewId("autoassess", "InspectionResultView", "1")
campaigns = client.data_modeling.instances.list(
    instance_type="node", sources=[result_view], limit=1000
)
for node in campaigns:
    props = dict(node.properties[result_view])
    listed = set(props.get("cdfFileIds") or []) | set(props.get("pcdFileIds") or [])
    overlap = listed.intersection(file_ids)
    assert not overlap, (node.external_id, overlap)
print("NO-CAMPAIGN-REFERENCES-OK")

defect_view = ViewId("autoassess", "DefectDetectionView", "1")
node = client.data_modeling.instances.retrieve(
    nodes=[("autoassess", defect)], sources=[defect_view]
).nodes[0]
props = dict(node.properties[defect_view])
assert props["status"] == "New" and props["source"] == "ml", props
assert "campaign" not in props, props
assert props["area"]["externalId"] == area, props
print("defect:", node.external_id, props["defectClass"], props["area"]["externalId"])
print("AREA-DEFECT-OK")
PY

kill -INT $BRIDGE $STUB 2>/dev/null || true; wait $BRIDGE $STUB 2>/dev/null || true
echo "--- bridge log"; sed -E 's/\x1b\[[0-9;]*m//g' /tmp/bridge.log
kill %1 2>/dev/null || true
echo; echo "##### FIRST MAPPING OK"
