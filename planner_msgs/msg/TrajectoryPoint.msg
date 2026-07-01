builtin_interfaces/Duration time_from_start   # offset from header.stamp
geometry_msgs/Point   position                # x, y, z  (z=0 in 2D)
geometry_msgs/Vector3 velocity                # dr/dt   (m/s)
geometry_msgs/Vector3 acceleration            # d2r/dt2 (m/s^2)
float64 yaw                                   # atan2(vy, vx)
float64 pitch                                 # atan2(vz, hypot(vx,vy))   (ENU)
float64 speed                                 # ||velocity||