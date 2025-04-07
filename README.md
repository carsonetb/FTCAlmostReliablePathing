# Almost Reliable Pathing

ARP is a dynamic path follower for FTC robots. It has
these features:

- Automatic tuning of velocity coefficients (kV and headingkV)
- Changing the path in realtime
- Lines
- Turning
- Lines while rotating
- Splines
- Waiting
- Running code at a certain point in the path
- PID for position and rotation so the robot can error correct.

It currently supports these configurations:

- Mecanum drive with two wheel odometry and IMU sensor.
- Hopefully more in the future.
