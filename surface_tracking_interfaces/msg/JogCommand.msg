# Command sent from GUI to Backend to trigger MoveIt Servo
string jog_type         # "cart" or "joint"
int8 axis               # 0 to 5 (e.g., X=0, Y=1, Z=2)
float32 direction       # -1.0 for negative, 1.0 for positive, 0.0 to stop
float32 velocity_scale  # 0.0 to 1.0 (e.g., 40% = 0.4)
string frame_id         # "elfin_base_link" or "elfin_end_link" (Used only if jog_type is "cart")