# ECET581 - "ROS Programming" course project

## What we have done
* Wall following with PD control regarding to LiDAR reading from "/scan"
* Drop waypoints on the hallmap, manually
    * Tried to follow the waypoints with PID control regarding to orientation error and distance error(we have tried fixed distance), but not working well
    > I guess we should use some extra information (like imu readings) to estimate car pose between two consecutive AMCL readings

## What we need to do next(according to LinZ's imagination)
1. Map cartesian coordinates to the grid world.
> An example of getting the grid map can be found in "/path/to/map_follower/src/map_listener.py"
2. Path-Planning within the grid world
> LinZ is working on this part currently, trying to figure out with  A\*\-search
3. Generate waypoints in cartesian coordinate system, automatically, according to step\-2
4. PID control following waypoints with IMU estimations
> You can experiment this part with manually set waypoints as in "path/to/map_follower/src/man_wp_follow.py"
