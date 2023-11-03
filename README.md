# Navigation-Kalman
An implementation of an asynchronous navigation Kalman filter, fusing low frequency GNSS data with high frequency accelerations to predict position and velocity.

Filter output (green) and raw GNSS position (yellow). generated in flight on an RC aircraft:
![Filter outputs generated on an RC aircraft, showing the smoothing of the filter](FilterTest.png)

**Examples:**
```cpp
#include "nav_kalman.h"
```
To create a simple filter object:
```cpp
navKalman mykalman = navKalman(R_position, R_velocity, a_std);
```
Run prediction step with acceleration data:
```cpp
mykalman.accel_update(Dt, accel);
```
Run update step with GNSS position and velocity:
```cpp
mykalman.GNSS_update(position, velocity);
```
Access the predictions:
```cpp
position = mykalman.U_hat[0]
velocity = mykalman.U_hat[1]
```

(An object corresponds to a single axis)

NOTE: The GNSS readings and accelerations should be in the same frame. For a normal implementation, the NED (north-east-down) frame, that is the default for the GNSS was used. The accelerations will most likely be in the inertial frame, and should be rotated to the NED frame using an IMU.
