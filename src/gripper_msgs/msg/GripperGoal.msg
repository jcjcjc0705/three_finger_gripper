# Where the fingers should end up, and how hard they may push to get there.
# Send it as often as you like: the servo re-plans from wherever it is, so a
# slow deliberate planner and a fast policy use the same interface.
#
# Joint order is motor 1..8:
#   C_2_Joint  C_3_Joint  A_2_Joint  A_3_Joint  B_2_Joint  B_3_Joint
#   A_1_Joint  B_1_Joint
# that is thumb proximal/distal, index proximal/distal, middle proximal/distal,
# then the index and middle spread joints. Zero is fully open, negative closes.

float32[8] position        # rad
float32[8] current_limit   # Goal Current ceiling, motor register units
float32 max_velocity       # rad/s, 0 keeps the configured default
float32 max_acceleration   # rad/s^2, 0 keeps the configured default
