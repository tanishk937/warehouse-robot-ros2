# Per-sensor heartbeat / health record produced by sensor_heartbeat_monitor.

uint8 CRITICAL=0
uint8 NON_CRITICAL=1

uint8 ONLINE=0
uint8 STALE=1
uint8 FAILED=2

string sensor_name
string topic_name
string msg_type
uint8 criticality
uint8 state
bool is_active
bool is_stale
float64 frequency_hz
float64 expected_frequency_hz
float64 seconds_since_last_msg
int32 failure_count
