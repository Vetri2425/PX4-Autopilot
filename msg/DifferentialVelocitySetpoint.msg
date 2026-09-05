uint64 timestamp # time since system start (microseconds)

float32 speed   # [m/s] [-inf, inf] Speed setpoint (Backwards driving if negative)
float32 bearing # [rad] [-pi,pi] from North.
float32 yaw_rate_feedforward # [rad/s] Companion yaw-rate feedforward (trajectory_setpoint.yawspeed). NaN = none.
