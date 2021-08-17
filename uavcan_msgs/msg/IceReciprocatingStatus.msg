# The same as https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#uavcanequipmenticereciprocating
uint8 state                                 # uint2. Enum. Abstract engine state. The flags defined below can provide further elaboration

uint32 flags                                # uint30. Enum. General status flags
uint8 engine_load_percent                   # uint7. Percent. Engine load estimate
uint32 engine_speed_rpm                     # uint17. Rpm. Engine speed.

float32 spark_dwell_time_ms                 # float16. millisecond. Spark dwell time.
float32 atmospheric_pressure_kpa            # float16. kilopascal. Atmospheric (barometric) pressure.
float32 intake_manifold_pressure_kpa        # float16. kilopascal. Engine intake manifold pressure.
float32 intake_manifold_temperature         # float16. kelvin. Engine intake manifold temperature.
float32 coolant_temperature                 # float16. kelvin. Engine coolant temperature.
float32 oil_pressure                        # float16. kilopascal. Oil pressure.
float32 oil_temperature                     # float16. kelvin. Oil temperature.
float32 fuel_pressure                       # float16. kilopascal. Fuel pressure.
float32 fuel_consumption_rate_cm3pm         # float32. (centimeter^3)/minute. Instant fuel consumption estimate.
float32 estimated_consumed_fuel_volume_cm3  # float32. centimeter^3. Estimate of the consumed fuel since the start of the engine.

uint8 throttle_position_percent             # uint7. Percent. Throttle position.
uint8 ecu_index                             # uint6. The index of the publishing ECU.
uint8 spark_plug_usage                      # uint3. Enum. Spark plug activity report.