# Waypoint following on hallway map

## IMU estimate car pose
update car pose 
> refer to: http://planning.cs.uiuc.edu/node658.html
IMU was calibrated using 100 reading at stationary 
## Car control
1. Determine which point is next
2. Compute errors
3. Compute PID control based on errors
4. Update car states
## PID control
* Seems tuning PID parameters is the key to well following waypoints defined path
* To slow down the car before it entering a turn, more closely arranged waypoints may be needed. 

