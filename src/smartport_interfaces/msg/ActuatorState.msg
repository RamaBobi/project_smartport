std_msgs/Header header

float64 actual_velocity         # Current vehicle speed (m/s)
float64 actual_steering_angle   # Current virtual center steering angle (rad)
float64 throttle_input          # Normalized throttle applied [0.0 to 1.0]
float64 brake_input             # Normalized brake applied [0.0 to 1.0]
float64 fuel_consumption_rate   # Instantaneous consumption (L/s or kW/s for EV)
float64 cumulative_fuel_used    # Total consumption during the run (L or kWh)
