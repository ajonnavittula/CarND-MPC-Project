# Model Predictive Control Rubric Discussion

## Compilation

The code compiles with cmake and make. There were no modifications to the CMakeLists.txt.

## Implementation

The MPC algorithm minimizes the cost function by finding the best set of control inputs. Implementation involves a simplified kinematic model of a car and finding the throttle and steering angles that minimize the cost function.

### The model

#### State equations

The objective of the model is to be able to calculate the throttle (acceleration) and the steering angle (orientation) of the car given a trajectory. To obtain these values, the cartesian position (`px`, `py`), orientation (`psi`) and speed (`v`) of the car are used as state variables. Additionally the cross track error (`cte`) and angular difference between vehicle trajectory and planned trajectory (`epsi`) are also considered. Using these variables, the new state of the car can be obtained as following:

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

### Timestep Length and Elapsed Duration (N & dt)

The choice of N and dt was mainly obtained through trial and error. The final choice was `N = 10` and `dt = 0.1`. It was observed that having N lower causes the car to plan for fewer number of steps going forward and the car has trouble with sharp turns. With higher dt the calculation time was higher and there were oscillations in the car's motion especially at slow speeds.

### Polynomial Fitting and MPC Preprocessing

A polynomial was fitted using the polyfit function. To simplify calculations, geometric transformations were used to convert the incoming positions from the map co-ords to car co-ords. The transfomations simply used cartesian equations for tranformation of the origin.

### Model Predictive Control with Latency

To account for latency the state equations were used to predict the state of the car 100ms in the future and the predicted state was used to calculate the outputs.

## Simulation

The following video shows the car running around the track.

[![PID Controller](http://img.youtube.com/vi/mcPMGE0wDbs/0.jpg)](http://www.youtube.com/watch?v=mcPMGE0wDbs)
