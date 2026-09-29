# The AutoAssess inspection plan the bridge is following, published latched on
# /autoassess/plan whenever the followed plan (or its content) changes.
#
# This is the typed view of the plan; the verbatim plan.json text (as written by
# `dss plan download`, including fields not modelled here) is published latched on
# /autoassess/plan_json (std_msgs/String).

std_msgs/Header header        # frame_id: the frame of the task positions (normally `world`)
string external_id            # the plan's CDF externalId (also on /autoassess/plan_id)
string name                   # human-readable plan name; "" when unnamed
string area_external_id       # the area the plan inspects
string map_external_id        # the plan's reference map campaign; "" when the plan has none
InspectionTask[] tasks        # all tasks, in plan order
