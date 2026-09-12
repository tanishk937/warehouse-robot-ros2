# Classified battery status derived from sensor_msgs/BatteryState.
# NOTE: when produced by battery_manager in simulation mode, this is
# explicitly a simulated telemetry source (see battery_manager.py /
# 'simulate_battery' parameter) and is architected so a real hardware
# BatteryState publisher can be swapped in with zero downstream changes.

uint8 NORMAL=0
uint8 LOW=1
uint8 CRITICAL=2
uint8 CHARGING=3

std_msgs/Header header
float32 percentage
float32 voltage
float32 current
uint8 state
bool is_charging
bool is_simulated
float32 estimated_remaining_minutes
