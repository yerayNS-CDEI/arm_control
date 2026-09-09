# One sample of everything needed to judge an arm-driven wall sweep
# (ARM_SWEEP_PLAN §7.B "Diagnostics for bench validation").
#
# Published throughout every leg so a `ros2 bag` of one run is enough to plot:
# TCP path in the wall tangent/normal frame, speed vs time, force vs time, and
# orientation error vs time -- which is what §9 says the sweep thresholds must be
# tuned from, rather than from the guessed defaults they ship with.

std_msgs/Header header          # stamp; frame_id is the arm planning frame

# --- identity ---------------------------------------------------------------
int32 partition_index           # 0-based, as the FSM numbers them
int32 partition_count
float64 line_z                  # scan-line height (map frame)
string phase                    # traverse | plunge | sweep | retract

# --- commanded vs measured TCP, in the arm planning frame -------------------
geometry_msgs/Point desired_position
geometry_msgs/Point measured_position
float64 progress                # 0..1 along the partition
float64 lateral_travelled       # m along the partition tangent
float64 lateral_speed           # measured wall-tangent TCP speed, m/s
float64 lateral_speed_desired   # the commanded sweep speed
float64 lateral_error           # signed deviation along the tangent, m
float64 normal_error            # deviation perpendicular to the commanded line, m

# --- plate orientation vs the wall ------------------------------------------
# From wall_alignment_estimator, read-only: the executor never commands the arm
# from this, it only watches it (§4.2).
float64 wall_tilt_deg           # plate +Z vs the fitted wall normal -- what S6 triggers on
float64 orientation_error_deg   # commanded plate orientation vs the measured one
float64[3] wall_normal_plate    # fitted wall normal, plate frame; [0,0,1] is parallel
bool wall_fit_valid             # false when too few sensors agree on one plane

# --- the six range sensors --------------------------------------------------
float64[6] distances            # raw ranges, m
bool[6] distance_used           # which ones survived the window + plane trim
int32 n_sensors_used
float64 plate_wall_distance     # mean of the used ranges, m (NaN when none)

# --- contact ----------------------------------------------------------------
geometry_msgs/Wrench ft_wrench  # TCP force/torque, tool0 frame
float64 press_force             # wall-normal component; negative means pressing
bool ft_valid                   # false when no wrench has arrived (always so in Gazebo)

# --- execution --------------------------------------------------------------
int32 replans                   # orientation-correction replans so far (§4.2, S6)
