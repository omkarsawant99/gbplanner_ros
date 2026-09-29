# One finding reported by the robot's detection stack, subscribed on /autoassess/findings.
#
# Findings are buffered during the mission, merged within ~findings_merge_radius_m, and
# stored as DefectDetection nodes on the mission's campaign at mission end (uploads only).
# The same finding can instead be sent as JSON on /autoassess/findings_json
# (std_msgs/String, the `dss plan import-findings` CSV fields); both topics feed the
# same buffer. Invalid findings are logged and skipped, not raised.

# inspection_type values
string VISUAL=visual
string NDT_THICKNESS=ndt_thickness

string id                     # stable finding id, deduped across reports; "" = derive one
                              # from the values (identical repeats then dedupe too).
                              # Must not contain '+'.
geometry_msgs/Point position  # metres, in the reference map's frame (`world`)
geometry_msgs/Vector3 normal  # surface normal at the finding; only read when has_normal
                              # (must be non-zero then; it is normalised on receipt)
bool has_normal
float32 radius_m              # extent of the finding in metres; 0 = default (0.3 m)
string inspection_type        # VISUAL or NDT_THICKNESS; "" = visual
string defect_class           # e.g. "corrosion"; "" = none (stored as class `finding`)
float32 confidence            # (0..1]; 0 = not given (stored as probability 0.5)
string description            # free text; "" = none
