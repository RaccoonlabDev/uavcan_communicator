# The same as https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#fix
Header header
int64 longitude_deg_1e8
int64 latitude_deg_1e8
int64 height_msl_mm

geometry_msgs/Vector3 ned_velocity

uint8 sats_used
uint8 status

float32 pdop
