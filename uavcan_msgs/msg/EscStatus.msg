# The same as https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#status-2
uint32 error_count          # Resets when the motor restarts

float32 voltage             # Volt
float32 current             # Ampere. Can be negative in case of a regenerative braking.
float32 temperature         # Kelvin

int32 rpm                   # Negative value indicates reverse rotation

uint8 power_rating_pct      # Instant demand factor in percent (percent of maximum power); range 0% to 127%.

uint8 esc_index