# Short demo ROG-X

Repository for the scripts to be used as demos of ROG-X

# Scripts short description

* `setpoint_buffer.py`: setpoint buffer for the offboard mode. It is also responsible for arming the drone and driving it at flight height.
* `guidance_library.py`: library containing functions to create offboard targets and other simple routines.
* `attitude_library.py`: library containing functions to change attitude representations (quaternion, Euler angles and attitude matrices).
* `path_generation.py`: script to send simple trajectories to the drone in offboard mode.

# Before launching

Set the parameters of the experiment.

In `guidance_library.py` modify the following parameters:

* `takeoff_height` (default 1.5 m)
* `takeoff_speed` (default 0.3 m/s)

In `path_generation.py` modify the following parameters:

* Waypoint list: the waypoints are encoded as vectors of position setpoints along East and North, and yaw angle setpoints (yaw is independent of the position setpoint):

  * `waypoint_list_x` (position setpoint along local East, m)
  * `waypoint_list_y` (position setpoint along local North, m)
  * `yaw_sp` (yaw angle setpoint, positive CCW from above and zero aligned to East, consistent with ENU notation, deg)

* `speed` (cruise speed, default 0.1 m/s)
* `landing_speed` (default 0.7 m/s)
* `speed_yaw` (max yaw rate, default 15.0 deg/s)

Note: the notation is ENU, consistent with ROS.

# Instructions to launch the scripts

Open a terminal

```
  ROS_NAMESPACE=iris python setpoint_buffer.py
```

Open another terminal

```
  ROS_NAMESPACE=iris python path_generation.py
```
