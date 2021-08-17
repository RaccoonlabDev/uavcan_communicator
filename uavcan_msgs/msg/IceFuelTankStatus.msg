# The same as https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#fueltankstatus
uint8 available_fuel_volume_percent         # uint7. percent. The estimated amount of fuel.
float32 available_fuel_volume_cm3           # float32. centimeter^3. The estimated amount of fuel.
float32 fuel_consumption_rate_cm3pm         # float32. (centimeter^3)/minute. Estimate of the current fuel consumption rate.
float32 fuel_temperature                    # float16. kelvin. Fuel temperature.
float32 fuel_tank_id                        # uint8. The ID of the current fuel tank.
