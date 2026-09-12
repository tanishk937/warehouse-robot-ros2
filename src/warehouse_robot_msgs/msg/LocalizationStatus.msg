# Pose-tracking confidence and staleness report.
#
# NOTE ON SCOPE: 'source' documents which upstream estimator this
# status describes. In the current system it is always "wheel_odometry"
# (Gazebo's diff-drive plugin, /odom) -- there is no AMCL/EKF sensor
# fusion in this project. This message is intentionally source-agnostic
# so a future EKF-fused or AMCL-based estimator can publish the same
# message on the same topic with no consumer changes.

string source

std_msgs/Header header
geometry_msgs/Pose pose
float64 linear_speed
float64 angular_speed
float64 pose_jump_distance
bool is_healthy
bool is_stale
bool pose_jump_detected
string status_message
