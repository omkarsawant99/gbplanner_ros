# Progress of the mission upload, published latched on /autoassess/upload_status.
#
# The full JSON status (per-file ids and labels, failed/skipped file lists, the
# first-mapping note, timestamps) is mirrored latched on /autoassess/upload_status_json
# (std_msgs/String).

# state values, in the order a mission goes through them. (No trailing comments here:
# a ROS string constant takes the whole rest of the line as its value.)
# IDLE: no upload yet (published once at start-up).
# EXPORTING_MESH: waiting for voxblox to write the mesh PLY.
# UPLOADING: creating the campaign / uploading files.
# COMPLETE: terminal, everything uploaded.
# FAILED: terminal, retry with ~upload_mission (resumes).
string IDLE=idle
string EXPORTING_MESH=exporting_mesh
string UPLOADING=uploading
string COMPLETE=complete
string FAILED=failed

std_msgs/Header header
string state                  # one of the constants above
string mission_id             # mission-<UTC start time>; "" before the first upload
string campaign_external_id   # the created campaign; "" while none exists, and on a
                              # campaign-less first-mapping upload (no plan followed)
uint32 files_done             # files uploaded so far (meshes + point clouds)
uint32 files_total            # files the upload attempted (files_done + failed files)
uint32 findings_count         # findings stored as DefectDetection nodes (final status only)
string[] defect_external_ids  # their CDF externalIds (final status only)
string message                # human-readable progress / error text
