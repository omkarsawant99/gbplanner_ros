# One task of an AutoAssess inspection plan (see InspectionPlan.msg).
#
# All positions and normals are in the plan's frame (InspectionPlan.header.frame_id,
# the reference map's frame, normally `world`).

# task_type values. (No trailing comments here: a ROS string constant takes the whole
# rest of the line as its value.)
# REGION: inspect a point on the structure (position + optional normal).
# ELEMENT: inspect a whole structural element (position = element centre).
string REGION=region
string ELEMENT=element

# inspection_type values
string VISUAL=visual
string NDT_THICKNESS=ndt_thickness

string id                     # the task's CDF externalId (stable, unique within the plan)
string task_type              # REGION or ELEMENT
string inspection_type        # VISUAL or NDT_THICKNESS
geometry_msgs/Point position  # the point to inspect: region position3d, or the element centre.
                              # (0,0,0) when the task has no target in CDF (such tasks are also
                              # left out of /autoassess/inspection_targets; the plan JSON on
                              # /autoassess/plan_json has the full record)
geometry_msgs/Vector3 normal  # surface normal at the target; only valid when has_normal
bool has_normal               # region tasks may carry a normal; element tasks never do
float32 radius_m              # region radius in metres; 0 = not set
string target_element_id      # element tasks: the structural element's CDF externalId; else ""
