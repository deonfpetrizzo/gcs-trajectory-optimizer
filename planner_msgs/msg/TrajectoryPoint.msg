builtin_interfaces/Duration time_from_start   # offset from header.stamp
geometry_msgs/Point   position                # x, y, z  (z=0 in 2D)
geometry_msgs/Vector3 velocity                # dr/dt   (m/s)
geometry_msgs/Vector3 acceleration            # d2r/dt2 (m/s^2)
float64 yaw                                   # atan2(vy, vx) -- heading of travel direction
float64 pitch                                 # attitude pitch (differential flatness: from (acceleration - gravity) + yaw)
float64 roll                                  # attitude roll  (differential flatness: from (acceleration - gravity) + yaw)
float64 speed                                 # ||velocity||