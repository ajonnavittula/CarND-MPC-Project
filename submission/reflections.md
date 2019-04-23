# Model Predictive Control Rubric Discussion

## Compilation

The code compiles with cmake and make. There were no modifications to the CMakeLists.txt.

## Implementation

The MPC algorithm minimizes the cost function by finding the best set of control inputs. Implementation involves a simplified kinematic model of a car and finding the throttle and steering angles that minimize the cost function.

### The model

#### State equations

The objective of the model is to be able to calculate the throttle (acceleration) and the steering angle (orientation) of the car given a trajectory. To obtain these values, the cartesian position (`px`, `py`), orientation (`psi`) and speed (`v`) of the car are used state variables. Additionally the cross track error (`cte`) and angular difference between vehicle trajectory and planned trajectory (`epsi`) are also considered. Using these variables, the new state of the car can be obtained as following:

```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] * delta[t] / Lf * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
Where:

`dt` denotes the time elapsed

`Lf` is the distance between the front of the vehicle and its center of gravity

`f(x)` is the evaluation of the polynomial `f()` at `x`

#### Cost function

The cost function aims to reduce the 



## Simulation

The following video shows the car running around the track.

[![PID Controller](http://img.youtube.com/vi/QS_azTC0frc/0.jpg)](http://www.youtube.com/watch?v=QS_azTC0frc)
