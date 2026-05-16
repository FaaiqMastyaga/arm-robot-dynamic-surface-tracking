# Aggregated status message sent from Backend to GUI
bool is_enabled
bool is_faulted
int8 servo_status

# End-Effector Pose (Flat floats to keep dependencies simple)
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw

# Joint States (in degrees)
float64[6] joint_positions