# Current message that our Logitech steering wheel and pedals broadcast
# Note, naming scheme a bit confusing due to generalization to SVEA, RCVE, and CARLA
# gear_name is either "forward" or "reverse"
# speed_mode is either "slow" or "fast"
# steer_mode is either "2WS" or "4WS"
# operation_mode is either "auto" or "takeover"

Header header
int16 steer_percent
int16 gas_percent
int16 brake_percent
int16 alpha_percent
string gear_name
string speed_mode
string steer_mode
string operation_mode
