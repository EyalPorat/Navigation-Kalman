# Navigation-Kalman
An implementation of an asynchronous navigation Kalman filter, fusing GNSS data with accelerations to predict position and velocity, on a single axis.
The filter is used to predict higher frequency data off of raw GNSS readings using an IMU, for real-time control systems.
U_hat holds the predicted values of the filter (position, velocity).


Filter outputs generated on an RC aircraft, showing the smoothing of the filter:
![Filter outputs generated on an RC aircraft, showing the smoothing of the filter](https://github.com/EyalPorat/Navigation-Kalman/blob/main/Filter%20Test.jpeg)

**Examples**

To create a simple filter object:
```
navKalman mykalman = navKalman(R_position, R_velocity, a_std);
```
Run prediction step with acceleration data:
```
mykalman.accel_update(Dt, accel);
```
Run update step with GNSS position and velocity:
```
mykalman.GNSS_update(position, velocity);
```

