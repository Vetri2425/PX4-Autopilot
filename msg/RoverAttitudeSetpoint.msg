# Rover Attitude Setpoint

uint64 timestamp      # [us] Time since system start
float32 yaw_setpoint  # [rad] [@range -inf, inf] [@frame NED] Yaw setpoint
float32 yaw_rate_feedforward # [rad/s] Optional yaw-rate feedforward (e.g. trajectory_setpoint.yawspeed). NaN = none. Scaled by RD_YAW_RATE_FF (default 0) and added to, never replaces, the yaw-P feedback term in DifferentialAttControl.
