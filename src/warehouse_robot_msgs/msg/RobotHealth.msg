# Top-level aggregated system health snapshot, published for the
# monitoring dashboard and consumed by any supervisory logic.
#
# health_state follows the formal state machine documented in
# docs/architecture.md:
#   BOOTING -> HEALTHY <-> DEGRADED <-> WARNING <-> CRITICAL
#   any state -> E_STOP (on e-stop activation)
#   any state -> SHUTDOWN (on node shutdown, external orchestration only)

uint8 BOOTING=0
uint8 HEALTHY=1
uint8 DEGRADED=2
uint8 WARNING=3
uint8 CRITICAL=4
uint8 E_STOP=5
uint8 SHUTDOWN=6

std_msgs/Header header
uint8 health_state
BatteryStatus battery
LocalizationStatus localization
SensorHeartbeatArray sensor_heartbeats
bool e_stop_active
string safety_state
string[] active_faults
string dashboard_json
