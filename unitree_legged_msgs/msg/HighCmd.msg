uint8 level_flag
uint16 comm_version              # Old version Aliengo does not have
uint16 robot_id                  # Old version Aliengo does not have
uint32 sn                       # Old version Aliengo does not have
uint8 band_width                     # Old version Aliengo does not have
uint8 mode
float32 forward_speed
float32 side_speed
float32 rotate_speed
float32 body_height
float32 foot_raise_height
float32 yaw
float32 pitch
float32 roll
LED[4] led
uint8[40] wireless_remote
uint8[40] app_remote             # Old version Aliengo does not have
uint32 reserve                  # Old version Aliengo does not have
int32 crc