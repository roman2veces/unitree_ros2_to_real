uint8 level_flag
uint16 comm_version                  # Old version Aliengo does not have
uint16 robot_id                      # Old version Aliengo does not have
uint32 sn                           # Old version Aliengo does not have
uint8 band_width                     # Old version Aliengo does not have
uint8 mode
IMU imu
float32 forward_speed
float32 side_speed
float32 rotate_speed
float32 body_height
float32 updown_speed
float32 forward_position       # (will be float type next version)   # Old version Aliengo is different
float32 side_position          # (will be float type next version)   # Old version Aliengo is different
Cartesian[4] foot_position2body
Cartesian[4] foot_speed2body
int16[4] foot_force                  # Old version Aliengo is different
int16[4] foot_force_est               # Old version Aliengo does not have
uint32 tick               
uint8[40] wireless_remote
uint32 reserve                      # Old version Aliengo does not have
uint32 crc

# Under are not defined in SDK yet. # Old version Aliengo does not have
Cartesian[4] ee_force            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.
float32[12] joint_p              # for visualization